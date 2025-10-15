package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"strings"
)

const PIN_MIN_TIME = 0.100

type ControllerFan struct {
	printer        *Printer
	stepper_names  []string
	stepper_enable *PrinterStepperEnable

	heaters      []*Heater
	fan          *Fan
	fan_speed    float64
	idle_speed   float64
	idle_timeout int
	heater_names []string
	last_on      int
	last_speed   float64
}

func NewControllerFan(config *ConfigWrapper) *ControllerFan {
	self := new(ControllerFan)
	self.printer = config.Get_printer()
	self.printer.Register_event_handler("project:ready", self.handle_ready)
	self.printer.Register_event_handler("project:connect", self.handle_connect)
	self.stepper_names = cast.ToStringSlice(config.Getlist("stepper", []string{}, ",", 0, true))
	self.stepper_enable = self.printer.Load_object(config, "stepper_enable", object.Sentinel{}).(*PrinterStepperEnable)
	self.printer.Load_object(config, "heaters", object.Sentinel{})
	self.heaters = make([]*Heater, 0)
	self.fan = NewFan(config, 0.0)
	self.fan_speed = config.Getfloat("fan_speed", 1.0, 0, 1.0, 0, 0, true)
	self.idle_speed = config.Getfloat("idle_speed", self.fan_speed, 0, 1.0, 0, 0, true)
	self.idle_timeout = config.Getint("idle_timeout", 30, 0, 0, true)
	self.heater_names = cast.ToStringSlice(config.Getlist("heater", []string{"extruder"}, ",", 0, true))
	self.last_on = self.idle_timeout
	self.last_speed = 0.
	return self
}

func (self *ControllerFan) handle_connect([]interface{}) error {
	// Heater lookup
	pheaters := self.printer.Lookup_object("heaters", object.Sentinel{}).(*PrinterHeaters)
	for _, v := range self.heater_names {
		self.heaters = append(self.heaters, pheaters.Lookup_heater(v))
	}

	// Stepper lookup
	all_steppers := self.stepper_enable.Get_steppers()
	if len(self.stepper_names) == 0 {
		self.stepper_names = all_steppers
		return nil
	}

	for _, v := range self.stepper_names {
		if !contains(all_steppers, v) {
			return fmt.Errorf("One or more of these steppers are unknown: %s (valid steppers are: %s)",
				self.stepper_names, strings.Join(all_steppers, ", "))
		}
	}
	return nil
}

func (self *ControllerFan) handle_ready([]interface{}) error {
	reactor := self.printer.Get_reactor()
	reactor.Register_timer(self.callback, reactor.Monotonic()+PIN_MIN_TIME)
	return nil
}

func (self *ControllerFan) get_status(eventtime float64) map[string]float64 {
	return self.fan.Get_status(eventtime)
}

func (self *ControllerFan) callback(eventtime float64) float64 {
	speed := 0.0
	active := false

	for _, name := range self.stepper_names {
		et, err := self.stepper_enable.Lookup_enable(name)
		if err == nil {
			if et.Is_motor_enabled() {
				active = true
			}
		} else {
			logger.Errorf("stepper_enable Lookup_enable %s error: %v\n", name, err)
		}
	}

	for _, heater := range self.heaters {
		_, target_temp := heater.Get_temp(eventtime)
		if target_temp > 0 {
			active = true
		}
	}

	if active {
		self.last_on = 0
		speed = self.fan_speed
	} else if self.last_on < self.idle_timeout {
		speed = self.idle_speed
		self.last_on += 1
	}

	if speed != self.last_speed {
		self.last_speed = speed
		self.fan.set_speed(speed, nil)
	}

	return eventtime + 1.0
}

func Load_config_controller_fan(config *ConfigWrapper) interface{} {
	return NewControllerFan(config)
}

func contains(s []string, str string) bool {
	for _, v := range s {
		if v == str {
			return true
		}
	}
	return false
}
