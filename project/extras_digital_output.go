package project

import (
	"k3c/common/utils/object"
)

const OUTPIN_MIN_TIME = 0.100

type PrinterDigitalOut struct {
	printer        *Printer
	mcu_pin        *MCU_digital_out
	last_value     int
	shutdown_value int
	reactor        IReactor
}

func NewPrinterDigitalOut(config *ConfigWrapper) *PrinterDigitalOut {
	self := &PrinterDigitalOut{}
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	self.printer.Register_event_handler("project:ready", self._handle_ready)
	self.printer.Register_event_handler("project:shutdown", self._handle_shutdown)

	ppins := self.printer.Lookup_object("pins", object.Sentinel{}).(*PrinterPins)
	self.mcu_pin = ppins.Setup_pin("digital_out", config.Get("pin", "", true).(string)).(*MCU_digital_out)
	self.last_value = config.Getint(
		"value", 0, 0, 0, true)
	self.shutdown_value = config.Getint(
		"shutdown_value", 0., 0., 0, true)
	self.mcu_pin.Setup_max_duration(0.)
	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint(
		"power/set_power_pin", self.SetPowerPin)
	return self
}

func (self *PrinterDigitalOut) Get_status(eventTime float64) map[string]int {
	return map[string]int{"value": self.last_value}
}

func (self *PrinterDigitalOut) SetPowerPin(web_request *WebRequest) (interface{}, error) {

	pin_status := int(web_request.Get_float("S", object.Sentinel{}))
	self._set_pin(pin_status)
	return nil, nil
}

func (self *PrinterDigitalOut) _set_pin(value int) {
	curtime := self.reactor.Monotonic()
	est_time := self.mcu_pin.Get_mcu().Estimated_print_time(curtime)
	next_cmd_time := est_time + OUTPIN_MIN_TIME
	self.mcu_pin.Set_digital(next_cmd_time, value)
}

func (self *PrinterDigitalOut) _handle_ready(args []interface{}) error {
	self._set_pin(self.last_value)
	return nil
}

func (self *PrinterDigitalOut) _handle_shutdown(args []interface{}) error {
	self._set_pin(self.shutdown_value)
	return nil
}

func Load_config_prefix_DigitalOut(config *ConfigWrapper) interface{} {
	return NewPrinterDigitalOut(config)
}
