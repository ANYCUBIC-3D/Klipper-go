package project

import (
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
)

type PrinterExtruderStepper struct {
	printer          *Printer
	extruder_stepper *ExtruderStepper
	extruder_name    string
}

func NewPrinterExtruderStepper(config *ConfigWrapper) *PrinterExtruderStepper {
	self := new(PrinterExtruderStepper)
	self.printer = config.Get_printer()
	self.extruder_stepper = NewExtruderStepper(config)
	self.extruder_name = cast.ToString(config.Get("extruder", object.Sentinel{}, true))
	self.printer.Register_event_handler("project:connect", self.handle_connect)

	return self
}

func (self *PrinterExtruderStepper) handle_connect([]interface{}) error {
	self.extruder_stepper.Sync_to_extruder(self.extruder_name)
	return nil
}

func (self *PrinterExtruderStepper) Get_status(eventtime float64) map[string]float64 {
	return self.extruder_stepper.Get_status(eventtime)
}

func Load_config_prefix_PrinterExtruderStepper(config *ConfigWrapper) interface{} {
	return NewPrinterExtruderStepper(config)
}
