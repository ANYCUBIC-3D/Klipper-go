/*
Code for handling the kinematics of cartesian robots

Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

type NoneKinematics struct {
	Axes_minmax []string
}

func NewNoneKinematics(toolhead *Toolhead, config *ConfigWrapper) *NoneKinematics {
	self := &NoneKinematics{}

	Axes_minmax := toolhead.Coord
	if Axes_minmax == nil {
		Axes_minmax = []string{"0.", "0.", "0.", "0."}
	}
	self.Axes_minmax = make([]string, len(Axes_minmax))
	copy(self.Axes_minmax, Axes_minmax)
	return self
}
func (self *NoneKinematics) Get_steppers() []interface{} {
	return []interface{}{}
}
func (self *NoneKinematics) Calc_position(stepper_positions map[string]float64) []float64 {

	return []float64{0, 0, 0}
}
func (self *NoneKinematics) Set_position(newpos []float64, homing_axes []int) {

}
func (self *NoneKinematics) Note_z_not_homed() {

}
func (self *NoneKinematics) Home_axis(homing_state *Homing, axis int, rail *PrinterRail) {
}
func (self *NoneKinematics) Home(homing_state *Homing) {
}
func (self *NoneKinematics) Motor_off(argv []interface{}) error {
	return nil
}

func (self *NoneKinematics) Check_endstops(move *Move) error {

	return nil
}

func (self *NoneKinematics) Check_move(move *Move) {
}
func (self *NoneKinematics) Get_status(eventtime float64) map[string]interface{} {

	return map[string]interface{}{
		"homed_axes":   "",
		"axis_minimum": self.Axes_minmax,
		"axis_maximum": self.Axes_minmax,
	}
}

// Dual carriage support
func (self *NoneKinematics) Activate_carriage(carriage int) {
}

func (self *NoneKinematics) cmd_SET_DUAL_CARRIAGE_help() string {
	return "Set which carriage is active"
}
func (self *NoneKinematics) Cmd_SET_DUAL_CARRIAGE(gcmd *GCodeCommand) {
}
func Load_kinematics_none(toolhead *Toolhead, config *ConfigWrapper) interface{} {
	return NewNoneKinematics(toolhead, config)
}
