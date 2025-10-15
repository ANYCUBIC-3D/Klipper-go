package project

import (
	"k3c/common/constants"
	"k3c/common/utils/object"
)

const CHECK_RUNOUT_TIMEOUT = .250

type EncoderSensor struct {
	printer                    *Printer
	extruder_name              string
	detection_length           float64
	reactor                    IReactor
	runout_helper              *RunoutHelper
	get_status                 func(eventtime float64) map[string]interface{}
	extruder                   *PrinterExtruder
	estimated_print_time       func(float64) float64
	filament_runout_pos        float64
	_extruder_pos_update_timer *ReactorTimer
}

func NewEncoderSensor(config *ConfigWrapper) *EncoderSensor {
	self := EncoderSensor{}
	// Read config
	self.printer = config.Get_printer()
	switch_pin := config.Get("switch_pin", object.Sentinel{}, true).(string)
	self.extruder_name = config.Get("extruder", object.Sentinel{}, true).(string)
	self.detection_length = config.Getfloat(
		"detection_length", 7., 0., 0., 0., 0., true)
	// Configure pins
	buttons := self.printer.Load_object(config, "buttons", object.Sentinel{}).(*PrinterButtons)
	buttons.Register_buttons([]string{switch_pin}, self.encoder_event)
	// Get printer objects
	self.reactor = self.printer.Get_reactor()
	self.runout_helper = NewRunoutHelper(config)
	self.get_status = self.runout_helper.Get_status
	self.extruder = nil
	self.estimated_print_time = nil
	// Initialise internal state
	self.filament_runout_pos = 0
	// Register commands and event handlers
	self.printer.Register_event_handler("project:ready",
		self._handle_ready)
	self.printer.Register_event_handler("idle_timeout:printing",
		self._handle_printing)
	self.printer.Register_event_handler("idle_timeout:ready",
		self._handle_not_printing)
	self.printer.Register_event_handler("idle_timeout:idle",
		self._handle_not_printing)
	return &self
}

func (self *EncoderSensor) _update_filament_runout_pos(eventtime float64) {
	if eventtime == 0 {
		eventtime = self.reactor.Monotonic()
	}
	self.filament_runout_pos = self._get_extruder_pos(eventtime) +
		self.detection_length
}

func (self *EncoderSensor) _handle_ready(argv []interface{}) error {
	self.extruder = self.printer.Lookup_object(self.extruder_name, object.Sentinel{}).(*PrinterExtruder)
	mcu := self.printer.Lookup_object("mcu", object.Sentinel{})
	self.estimated_print_time = mcu.(*MCU).Estimated_print_time
	self._update_filament_runout_pos(0)
	self._extruder_pos_update_timer = self.reactor.Register_timer(self._extruder_pos_update_event, constants.NEVER)
	return nil
}

func (self *EncoderSensor) _handle_printing(argv []interface{}) error {
	self.reactor.Update_timer(self._extruder_pos_update_timer, constants.NOW)
	return nil
}

func (self *EncoderSensor) _handle_not_printing(argv []interface{}) error {
	self.reactor.Update_timer(self._extruder_pos_update_timer, constants.NEVER)
	return nil
}

func (self *EncoderSensor) _get_extruder_pos(eventtime float64) float64 {
	if eventtime == 0 {
		eventtime = self.reactor.Monotonic()
	}
	print_time := self.estimated_print_time(eventtime)
	return self.extruder.Find_past_position(print_time)
}

func (self *EncoderSensor) _extruder_pos_update_event(eventtime float64) float64 {
	extruder_pos := self._get_extruder_pos(eventtime)
	//Check for filament runout
	self.runout_helper.Note_filament_present(extruder_pos < self.filament_runout_pos)
	return eventtime + CHECK_RUNOUT_TIMEOUT
}

func (self *EncoderSensor) encoder_event(eventtime float64, state int) {
	if self.extruder != nil {
		self._update_filament_runout_pos(eventtime)
		// Check for filament insertion
		// Filament is always assumed to be present on an encoder event
		self.runout_helper.Note_filament_present(true)
	}
}

func Load_config_prefix_EncoderSensor(config *ConfigWrapper) interface{} {
	return NewEncoderSensor(config)
}
