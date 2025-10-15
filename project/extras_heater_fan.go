package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
)

const HEATER_FAN_PIN_MIN_TIME = 0.100

type PrinterHeaterFan struct {
	printer      *Printer
	heater_names []string
	heater_temp  float64
	heaters      []*Heater
	fan          *Fan
	fan_speed    float64
	last_speed   float64
}

func NewPrinterHeaterFan(config *ConfigWrapper) *PrinterHeaterFan {
	self := new(PrinterHeaterFan)
	self.printer = config.Get_printer()
	self.printer.Load_object(config, "heaters", object.Sentinel{})
	self.printer.Register_event_handler("project:ready", self.handle_ready)
	self.heater_names = cast.ToStringSlice(config.Getlist("heater", []string{"extruder"}, ",", 0, true))
	self.heater_temp = config.Getfloat("heater_temp", 50.0, 0, 0, 0, 0, true)
	self.heaters = make([]*Heater, 0)
	self.fan = NewFan(config, 1.0)
	self.fan_speed = config.Getfloat("fan_speed", 1.0, 0, 1.0, 0, 0, true)
	self.last_speed = 0.0
	return self
}

func (self *PrinterHeaterFan) handle_ready([]interface{}) error {
	pheaters := self.printer.Lookup_object("heaters", object.Sentinel{})
	pheater, ok := pheaters.(*PrinterHeaters)
	if !ok {
		logger.Errorf("pheaters type is %T not *PrinterHeaters\n", pheater)
		return fmt.Errorf("pheaters type is %T not *PrinterHeaters", pheater)
	}

	for _, v := range self.heater_names {
		self.heaters = append(self.heaters, pheater.Lookup_heater(v))
	}
	reactor := self.printer.Get_reactor()
	reactor.Register_timer(self.callback, reactor.Monotonic()+HEATER_FAN_PIN_MIN_TIME)
	return nil
}

func (self *PrinterHeaterFan) get_status(eventtime float64) map[string]float64 {
	return self.fan.Get_status(eventtime)
}

func (self *PrinterHeaterFan) callback(eventtime float64) float64 {
	speed := 0.0
	for _, heater := range self.heaters {
		current_temp, target_temp := heater.Get_temp(eventtime)

		if target_temp > 0 || current_temp > self.heater_temp {
			speed = self.fan_speed
		}

	}

	if speed != self.last_speed {
		self.last_speed = speed
		self.fan.set_speed(speed, nil)
	}

	return eventtime + 1.0
}

func Load_config_heater_fan(config *ConfigWrapper) interface{} {
	return NewPrinterHeaterFan(config)
}
