package project

import (
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
)

type HomingHeaters struct {
	printer         *Printer
	disable_heaters []string
	flaky_steppers  []string
	pheaters        *PrinterHeaters
	target_save     map[string]float64
}

func NewHomingHeaters(config *ConfigWrapper) *HomingHeaters {
	self := new(HomingHeaters)
	self.printer = config.Get_printer()
	self.printer.Register_event_handler("project:connect",
		self.handle_connect)
	self.printer.Register_event_handler("homing:homing_move_begin",
		self.handle_homing_move_begin)
	self.printer.Register_event_handler("homing:homing_move_end",
		self.handle_homing_move_end)
	self.disable_heaters = cast.ToStringSlice(config.Getlist("heaters", []string{}, ",", 0, true))
	self.flaky_steppers = cast.ToStringSlice(config.Getlist("steppers", []string{}, ",", 0, true))

	pheaters := self.printer.Load_object(config, "heaters", object.Sentinel{})
	pheater, ok := pheaters.(*PrinterHeaters)
	if !ok {
		panic(fmt.Errorf("pheaters type is %T not *PrinterHeaters", pheater))
	}
	self.pheaters = pheater
	self.target_save = make(map[string]float64)
	return self
}

func (self *HomingHeaters) handle_connect([]interface{}) error {
	// heaters to disable
	all_heaters := self.pheaters.Get_all_heaters()
	if len(self.disable_heaters) == 0 {
		self.disable_heaters = all_heaters
	} else {
		for _, v := range self.disable_heaters {
			if !contains(all_heaters, v) {
				return fmt.Errorf(
					"One or more of these heaters are unknown: %v",
					self.disable_heaters)
			}
		}
	}

	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{})
	kin := toolhead.(*Toolhead).Get_kinematics().(IKinematics)
	var all_steppers []string
	for _, s := range kin.Get_steppers() {
		all_steppers = append(all_steppers, s.(*MCU_stepper).Get_name(false))
	}
	_ = all_steppers

	if len(self.flaky_steppers) == 0 {
		return nil
	}

	for _, v := range self.flaky_steppers {
		if !contains(all_steppers, v) {
			return fmt.Errorf(
				"One or more of these steppers are unknown: %v",
				self.flaky_steppers)
		}
	}
	return nil
}

func (self *HomingHeaters) check_eligible(endstops []interface{}) bool {
	if len(self.flaky_steppers) == 0 {
		return true
	}

	steppers_being_homed := []string{}
	for _, x := range steppers_being_homed {
		if contains(self.flaky_steppers, x) {
			return true
		}
	}
	return false

}

func (self *HomingHeaters) handle_homing_move_begin(args []interface{}) error {
	hmove := args[0].(*HomingMove)
	if !self.check_eligible(hmove.Get_mcu_endstops()) {
		return nil
	}

	for _, heater_name := range self.disable_heaters {
		heater := self.pheaters.Lookup_heater(heater_name)
		_, self.target_save[heater_name] = heater.Get_temp(0)
		heater.Set_temp(0.0)
	}
	return nil
}

func (self *HomingHeaters) handle_homing_move_end(args []interface{}) error {
	hmove := args[0].(*HomingMove)
	if !self.check_eligible(hmove.Get_mcu_endstops()) {
		return nil
	}

	for _, heater_name := range self.disable_heaters {
		heater := self.pheaters.Lookup_heater(heater_name)
		heater.Set_temp(self.target_save[heater_name])
	}

	return nil
}

func Load_config_homing_heaters(config *ConfigWrapper) interface{} {
	return NewHomingHeaters(config)
}
