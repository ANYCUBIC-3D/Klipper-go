package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/object"
	"k3c/common/value"
)

const DISABLE_STALL_TIME = 0.100

// Tracking of shared stepper enable pins
type StepperEnablePin struct {
	mcu_enable   *MCU_digital_out
	enable_count int
	is_dedicated bool
}

func NewStepperEnablePin(mcu_enable *MCU_digital_out, enable_count int) *StepperEnablePin {
	self := StepperEnablePin{}
	self.mcu_enable = mcu_enable
	self.enable_count = enable_count
	self.is_dedicated = true
	return &self
}

func (self *StepperEnablePin) Set_enable(print_time float64) {
	if self.enable_count == 0 {
		self.mcu_enable.Set_digital(print_time, 1)
	}
	self.enable_count++
}

func (self *StepperEnablePin) Set_disable(print_time float64) {
	self.enable_count--
	if self.enable_count == 0 {
		self.mcu_enable.Set_digital(print_time, 0)
	}
}

func Setup_enable_pin(printer *Printer, pin string) *StepperEnablePin {
	if pin == "" {
		// No enable line (stepper always enabled)
		enable := NewStepperEnablePin(nil, 9999)
		enable.is_dedicated = false
		return enable
	}

	ppins := printer.Lookup_object("pins", object.Sentinel{})
	pin_params := ppins.(*PrinterPins).Lookup_pin(pin, true, false, "stepper_enable")
	enable := pin_params["class"]
	if enable != nil {
		// Shared enable line
		enable.(*StepperEnablePin).is_dedicated = false
		return enable.(*StepperEnablePin)
	}

	mcu_enable := pin_params["chip"].(*MCU).Setup_pin("digital_out", pin_params)
	mcu_enable.(*MCU_digital_out).Setup_max_duration(0)
	enable = NewStepperEnablePin(mcu_enable.(*MCU_digital_out), 0)
	pin_params["class"] = enable
	return enable.(*StepperEnablePin)
}

// Enable line tracking for each stepper motor
type EnableTracking struct {
	stepper    *MCU_stepper
	enable     *StepperEnablePin
	callbacks  []func(float64, bool)
	is_enabled bool
}

func NewEnableTracking(stepper *MCU_stepper, enable *StepperEnablePin) *EnableTracking {
	self := EnableTracking{}
	self.stepper = stepper
	self.enable = enable
	self.callbacks = []func(float64, bool){}
	self.is_enabled = false
	self.stepper.Add_active_callback(self.Motor_enable)
	return &self
}

func (self *EnableTracking) Register_state_callback(callback func(float64, bool)) {
	self.callbacks = append(self.callbacks, callback)
}

func (self *EnableTracking) Motor_enable(print_time float64) {
	if !self.is_enabled {
		// Enable stepper on future stepper movement
		for _, cb := range self.callbacks {
			cb(print_time, true)
		}
		self.enable.Set_enable(print_time)
		self.is_enabled = true
	}
}

func (self *EnableTracking) Motor_disable(print_time float64) {
	if self.is_enabled {
		for _, cb := range self.callbacks {
			cb(print_time, false)
		}
		self.enable.Set_disable(print_time)
		self.is_enabled = false
		self.stepper.Add_active_callback(self.Motor_enable)
	}
}

func (self *EnableTracking) Is_motor_enabled() bool {
	return self.is_enabled
}

func (self *EnableTracking) Has_dedicated_enable() bool {
	return self.enable.is_dedicated
}

// Global stepper enable line trackingstruct
type PrinterStepperEnable struct {
	printer      *Printer
	enable_lines map[string]*EnableTracking
}

// NewPrinterStepperEnable constructor
func NewPrinterStepperEnable(config *ConfigWrapper) *PrinterStepperEnable {
	self := PrinterStepperEnable{}
	self.printer = config.Get_printer()
	self.enable_lines = map[string]*EnableTracking{}
	self.printer.Register_event_handler("gcode:request_restart", self._handle_request_restart)
	// Register M18/M84 commands
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_command("M18", self.cmd_M18, false, "")
	gcode.(*GCodeDispatch).Register_command("M84", self.cmd_M18, false, "")
	gcode.(*GCodeDispatch).Register_command("SET_STEPPER_ENABLE",
		self.cmd_SET_STEPPER_ENABLE,
		false,
		self.cmd_SET_STEPPER_ENABLE_help())
	return &self
}

func (self *PrinterStepperEnable) Register_stepper(config *ConfigWrapper, mcu_stepper *MCU_stepper) {
	name := mcu_stepper.Get_name(false)
	enable := Setup_enable_pin(self.printer, config.Get("enable_pin", value.None, true).(string))
	self.enable_lines[name] = NewEnableTracking(mcu_stepper, enable)
}

func (self *PrinterStepperEnable) Motor_off() {
	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{})
	toolhead.(*Toolhead).Dwell(DISABLE_STALL_TIME)
	print_time := toolhead.(*Toolhead).Get_last_move_time()
	for _, el := range self.enable_lines {
		el.Motor_disable(print_time)
	}
	self.printer.Send_event("stepper_enable:motor_off", []interface{}{print_time})
	toolhead.(*Toolhead).Dwell(DISABLE_STALL_TIME)
}

func (self *PrinterStepperEnable) motor_debug_enable(stepper string, enable bool) {
	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{})
	toolhead.(*Toolhead).Dwell(DISABLE_STALL_TIME)
	print_time := toolhead.(*Toolhead).Get_last_move_time()
	el := self.enable_lines[stepper]
	if enable {
		el.Motor_enable(print_time)
		logger.Debugf("%s has been manually enabled", stepper)
	} else {
		el.Motor_disable(print_time)
		logger.Debugf("%s has been manually disabled", stepper)
	}
	toolhead.(*Toolhead).Dwell(DISABLE_STALL_TIME)
}

func (self *PrinterStepperEnable) Get_status(eventtime float64) map[string]interface{} {
	steppers := []map[string]bool{}
	for k, v := range self.enable_lines {
		steppers = append(steppers, map[string]bool{k: v.Is_motor_enabled()})
	}
	return map[string]interface{}{"steppers": steppers}
}

func (self *PrinterStepperEnable) _handle_request_restart([]interface{}) error {
	self.Motor_off()
	return nil
}

func (self *PrinterStepperEnable) cmd_M18(argv interface{}) error {
	// Turn off motors
	self.Motor_off()
	return nil
}

func (self *PrinterStepperEnable) cmd_SET_STEPPER_ENABLE_help() string {
	return "Enable/disable individual stepper by name"
}

func (self *PrinterStepperEnable) cmd_SET_STEPPER_ENABLE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	stepper_name := gcmd.Get("STEPPER", nil, "", nil, nil, nil, nil)
	if _, ok := self.enable_lines[stepper_name]; !ok {
		gcmd.Respond_info(fmt.Sprintf("set_stepper_enable: Invalid stepper %s", stepper_name), true)
		return nil
	}
	stepper_enable := gcmd.Get_int("ENABLE", 1, nil, nil)
	logger.Debug("enable stepper moter:", stepper_name)
	self.motor_debug_enable(stepper_name, stepper_enable == 1)
	return nil
}

func (self *PrinterStepperEnable) Lookup_enable(name string) (*EnableTracking, error) {
	if _, ok := self.enable_lines[name]; !ok {
		return nil, errors.New(fmt.Sprintf("Unknown stepper '%s'", name))
	}
	return self.enable_lines[name], nil
}

func (self *PrinterStepperEnable) Get_steppers() []string {
	keys := []string{}
	for ke, _ := range self.enable_lines {
		keys = append(keys, ke)
	}
	return keys
}

// LoadConfig load config
func Load_config_StepperEnable(config *ConfigWrapper) interface{} {
	return NewPrinterStepperEnable(config)
}
