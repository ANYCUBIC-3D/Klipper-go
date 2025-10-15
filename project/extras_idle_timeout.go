package project

import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"math"
	"runtime/debug"
)

// Here marco is not compatible with pongo2 and will not stop warming up
const DEFAULT_IDLE_GCODE = "TURN_OFF_HEATERS\nM107\nM84\n"

const (
	// PIN_MIN_TIME  = 0.100
	READY_TIMEOUT = .500
)

const cmd_SET_IDLE_TIMEOUT_help = "Set the idle timeout in seconds"

type IdleTimeout struct {
	printer                  *Printer
	reactor                  IReactor
	gcode                    *GCodeDispatch
	toolhead                 *Toolhead
	timeout_timer            *ReactorTimer
	idle_timeout             float64
	idle_gcode               *TemplateWrapper
	state                    string
	last_print_start_systime float64
}

func NewIdleTimeout(config *ConfigWrapper) *IdleTimeout {
	self := new(IdleTimeout)
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	self.gcode = MustLookupGcode(self.printer)
	self.toolhead = nil
	self.timeout_timer = nil
	self.printer.Register_event_handler("project:ready", self.Handle_ready)
	self.idle_timeout = config.Getfloat("timeout", 600., 0, 0, 0, 0., true)
	gcode_macro := self.printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	self.idle_gcode = gcode_macro.Load_template(config, "gcode",
		DEFAULT_IDLE_GCODE)
	self.gcode.Register_command("SET_IDLE_TIMEOUT",
		self.Cmd_SET_IDLE_TIMEOUT, false,
		cmd_SET_IDLE_TIMEOUT_help)
	self.state = "Idle"
	self.last_print_start_systime = 0.
	return self
}

func (self *IdleTimeout) Get_status(eventtime float64) map[string]interface{} {
	printing_time := 0.
	if self.state == "Printing" {
		printing_time = eventtime - self.last_print_start_systime
	}
	return map[string]interface{}{"state": self.state, "printing_time": printing_time}
}

func (self *IdleTimeout) Handle_ready([]interface{}) error {
	self.toolhead = MustLookupToolhead(self.printer)
	self.timeout_timer = self.reactor.Register_timer(self.Timeout_handler, constants.NEVER)
	self.printer.Register_event_handler("toolhead:sync_print_time",
		self.Handle_sync_print_time)
	return nil
}

func (self *IdleTimeout) transition_idle_state(eventtime float64) (idletime float64) {
	self.state = "Printing"
	var (
		err    error
		script string
	)

	defer func() {
		r := recover()
		if r != nil || err != nil {
			s := string(debug.Stack())
			logger.Error("idle timeout gcode execution", sys.GetGID(), err, s)
			self.state = "Ready"
			idletime = eventtime + 1.
		}
	}()

	script, err = self.idle_gcode.Render(nil)
	if err != nil {
		logger.Panic("idle timeout gcode execution")
		self.state = "Ready"
		return eventtime + 1.
	}
	self.gcode.Run_script(script)

	print_time := self.toolhead.Get_last_move_time()
	self.state = "Idle"
	self.printer.Send_event("idle_timeout:idle", []interface{}{print_time})
	return constants.NEVER
}

func (self *IdleTimeout) Check_idle_timeout(eventtime float64) float64 {
	// Make sure toolhead class isn't busy
	print_time, est_print_time, lookahead_empty := self.toolhead.Check_busy(
		eventtime)
	idle_time := est_print_time - print_time

	if !lookahead_empty || idle_time < 1. {
		// Toolhead is busy
		return eventtime + self.idle_timeout
	}

	if idle_time < self.idle_timeout {
		// Wait for idle timeout
		return eventtime + self.idle_timeout - idle_time
	}

	if self.gcode.Get_mutex().Test() {
		// Gcode class busy
		return eventtime + 1.
	}

	// Idle timeout has elapsed
	return self.transition_idle_state(eventtime)
}

func (self *IdleTimeout) Timeout_handler(eventtime float64) float64 {
	if self.printer.Is_shutdown() {
		return constants.NEVER
	}

	if self.state == "Ready" {
		return self.Check_idle_timeout(eventtime)
	}

	// Check if need to transition to "ready" state
	print_time, est_print_time, lookahead_empty := self.toolhead.Check_busy(
		eventtime)
	buffer_time := math.Min(2., print_time-est_print_time)

	if !lookahead_empty {
		// Toolhead is busy
		return eventtime + READY_TIMEOUT + math.Max(0., buffer_time)
	}

	if buffer_time > -READY_TIMEOUT {
		// Wait for ready timeout
		return eventtime + READY_TIMEOUT + buffer_time
	}

	if self.gcode.Get_mutex().Test() {
		// Gcode class busy
		return eventtime + READY_TIMEOUT
	}

	// Transition to "ready" state
	self.state = "Ready"
	self.printer.Send_event("idle_timeout:ready",
		[]interface{}{est_print_time + PIN_MIN_TIME})
	return eventtime + self.idle_timeout
}

func (self *IdleTimeout) Handle_sync_print_time(argv []interface{}) error {
	if self.state == "Printing" {
		return nil
	}
	//Transition to "printing" state
	curtime := argv[0].(float64)
	est_print_time := argv[1].(float64)
	print_time := argv[2].(float64)

	self.state = "Printing"
	self.last_print_start_systime = curtime
	check_time := READY_TIMEOUT + print_time - est_print_time
	self.reactor.Update_timer(self.timeout_timer, curtime+check_time)
	self.printer.Send_event("idle_timeout:printing",
		[]interface{}{est_print_time + PIN_MIN_TIME})
	return nil
}

func (self *IdleTimeout) Cmd_SET_IDLE_TIMEOUT(argv []interface{}) {
	gcmd := argv[0].(*GCodeCommand)
	timeout := gcmd.Get_float("TIMEOUT", self.idle_timeout, nil, nil, cast.Float64P(0.), nil)
	self.idle_timeout = timeout
	gcmd.Respond_info(fmt.Sprintf("idle_timeout: Timeout set to %.2f s", timeout), true)
	if self.state == "Ready" {
		checktime := self.reactor.Monotonic() + timeout
		self.reactor.Update_timer(self.timeout_timer, checktime)
	}
}

func Load_config_idletimeout(config *ConfigWrapper) interface{} {
	return NewIdleTimeout(config)
}
