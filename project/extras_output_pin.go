package project

import (
	"fmt"
	"k3c/common/utils/object"
	"math"
	"strings"
	"sync"
)

const (
	//PIN_MIN_TIME = 0.100
	RESEND_HOST_TIME  = 0.300 + 0.100
	MAX_SCHEDULE_TIME = 5.0
)

type GCodeRequestQueue struct {
	printer             *Printer
	mcu                 *MCU
	callback            func(float64, float64) (string, float64)
	rqueue              [][]float64
	next_min_flush_time float64
	toolhead            *Toolhead
	mu                  sync.Mutex
}

func NewGCodeRequestQueue(config *ConfigWrapper, mcu *MCU, callback func(float64, float64) (string, float64)) *GCodeRequestQueue {
	self := new(GCodeRequestQueue)
	printer := config.Get_printer()
	self.printer = printer
	self.mcu = mcu
	self.callback = callback
	self.rqueue = [][]float64{}
	self.next_min_flush_time = 0.
	self.toolhead = nil
	mcu.Register_flush_callback(self._flush_notification)
	printer.Register_event_handler("project:connect", self._handle_connect)
	return self
}

func (self *GCodeRequestQueue) _handle_connect([]interface{}) error {
	self.toolhead = self.printer.Lookup_object("toolhead", nil).(*Toolhead)
	return nil
}

func (self *GCodeRequestQueue) _flush_notification(print_time float64, clock int64) {

	for len(self.rqueue) > 0 {
		next_time := math.Max(self.rqueue[0][0], self.next_min_flush_time)
		if next_time > print_time {
			return
		}

		// Skip requests that have been overridden by the following request
		pos := 0
		for pos+1 < len(self.rqueue) && self.rqueue[pos+1][0] <= next_time {
			pos++
		}
		req_val := self.rqueue[pos][1]
		//Invoke callback for the request
		action, min_wait := "", 0.
		action, min_wait = self.callback(next_time, req_val)
		if action != "" {
			//Handle special cases
			if action == "discard" {
				self.rqueue = self.rqueue[pos+1:]
				continue
			}
			if action == "delay" {
				pos--
			}
		}

		self.rqueue = self.rqueue[pos+1:]
		self.next_min_flush_time = next_time + math.Max(min_wait, 0.01) // PIN_MIN_TIME
		//Ensure following queue items are flushed
		self.toolhead.Note_mcu_movequeue_activity(self.next_min_flush_time, true)
	}
}

func (self *GCodeRequestQueue) _queue_request(print_time float64, value float64) {
	self.rqueue = append(self.rqueue, []float64{print_time, value})
	self.toolhead.Note_mcu_movequeue_activity(print_time, true)
}

func (self *GCodeRequestQueue) queue_gcode_request(value float64) {
	self.toolhead.Register_lookahead_callback(func(pt float64) {
		self._queue_request(pt, value)
	})
}

func (self *GCodeRequestQueue) send_async_request(value float64, print_time interface{}) {
	if print_time == nil {
		systime := self.printer.Get_reactor().Monotonic()
		print_time = self.mcu.Estimated_print_time(systime + 0.01) // PIN_MIN_TIME
	}
	for {
		next_time := math.Max(print_time.(float64), self.next_min_flush_time)
		//Invoke callback for the request
		action, min_wait := "normal", 0.
		action, min_wait = self.callback(next_time, value)
		if action != "" {
			//Handle special cases
			if action == "discard" {
				break
			}
		}
		self.next_min_flush_time = next_time + math.Max(min_wait, 0.01) // PIN_MIN_TIME
		if action != "delay" {
			break
		}
	}
}

type PrinterOutputPin struct {
	printer            *Printer
	is_pwm             bool
	mcu_pin            interface{}
	scale              float64
	last_cycle_time    float64
	last_print_time    float64
	reactor            IReactor
	resend_timer       *ReactorTimer
	resend_interval    float64
	last_value         float64
	shutdown_value     float64
	default_cycle_time float64
	gcrq               *GCodeRequestQueue
}

const cmd_SET_PIN_help = "Set the value of an output pin"

func NewPrinterOutputPin(config *ConfigWrapper) *PrinterOutputPin {
	self := &PrinterOutputPin{}
	self.printer = config.Get_printer()
	ppins := self.printer.Lookup_object("pins", object.Sentinel{}).(*PrinterPins) // Adjust the type assertion as needed
	self.is_pwm = config.Getboolean("pwm", false, true)

	if self.is_pwm {
		self.mcu_pin = ppins.Setup_pin("pwm", config.Get("pin", "", true).(string))
		cycle_time := config.Getfloat("cycle_time", 0.100, 0.0, MAX_SCHEDULE_TIME, 0., 0., true)
		hardware_pwm := config.Getboolean("hardware_pwm", false, true)

		self.mcu_pin.(*MCU_pwm).Setup_cycle_time(cycle_time, hardware_pwm)
		self.scale = config.Getfloat("scale", 1.0, 0.0, 0, 0, 0, true)
	} else {
		self.mcu_pin = ppins.Setup_pin("digital_out", config.Get("pin", "", true).(string))
		self.scale = 1.0
	}

	self.last_value = config.Getfloat(
		"value", 0., 0., self.scale, 0, 0, true) / self.scale
	self.shutdown_value = config.Getfloat(
		"shutdown_value", 0., 0., self.scale, 0, 0, true) / self.scale
	if _, ok := self.mcu_pin.(*MCU_pwm); ok {
		self.mcu_pin.(*MCU_pwm).Setup_max_duration(0.)
		self.mcu_pin.(*MCU_pwm).Setup_start_value(self.last_value, self.shutdown_value)
		//# Create gcode request self
		self.gcrq = NewGCodeRequestQueue(config, self.mcu_pin.(*MCU_pwm).Get_mcu(),
			self._set_pin)
	} else if _, ok := self.mcu_pin.(*MCU_digital_out); ok {
		self.mcu_pin.(*MCU_digital_out).Setup_max_duration(0.)
		self.mcu_pin.(*MCU_digital_out).Setup_start_value(self.last_value, self.shutdown_value)
		//# Create gcode request self
		self.gcrq = NewGCodeRequestQueue(config, self.mcu_pin.(*MCU_digital_out).Get_mcu(),
			self._set_pin)
	}

	//# Template handling
	//todo
	//self.template_eval = lookup_template_eval(config)
	//# Register commands
	pin_name := strings.Split(config.Get_name(), " ")[1]
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Register_mux_command("SET_PIN", "PIN", pin_name, self.cmd_SET_PIN,
		cmd_SET_PIN_help)

	return self
}

func (self *PrinterOutputPin) Get_status(eventTime float64) map[string]float64 {
	return map[string]float64{"value": self.last_value}
}

func (self *PrinterOutputPin) _set_pin(print_time float64, value float64) (string, float64) {

	if value == self.last_value {
		return "discard", 0.
	}

	self.last_value = value
	if self.is_pwm {
		self.mcu_pin.(*MCU_pwm).Set_pwm(print_time, value)
	} else {
		self.mcu_pin.(*MCU_digital_out).Set_digital(print_time, int(value))
	}
	return "", 0.
}

func (self *PrinterOutputPin) cmd_SET_PIN(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	zero := 0.0
	value := gcmd.Get_float("VALUE", 0.0, &zero, &self.scale, nil, nil)
	value /= self.scale
	if !self.is_pwm && (value != 0.0 && value != 1.0) {
		return fmt.Errorf("Invalid pin value")
	}
	self.gcrq.queue_gcode_request(value)
	return nil
}

func Load_config_prefix_OutputPin(config *ConfigWrapper) interface{} {
	return NewPrinterOutputPin(config)
}
