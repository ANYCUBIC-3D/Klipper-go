/*
Code for handling the kinematics of cartesian robots

Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"k3c/common/utils/collections"
	"k3c/common/utils/maths"
	"math"
	"strings"
)

type CorexyKinematics struct {
	Printer             *Printer
	Dual_carriage_axis  interface{}
	Dual_carriage_rails []*PrinterRail
	Rails               []*PrinterRail
	Max_z_velocity      float64
	Max_z_accel         float64
	Limits              [][]float64
	Axes_min            []float64
	Axes_max            []float64
}

func NewCorexyKinematics(toolhead *Toolhead, config *ConfigWrapper) *CorexyKinematics {
	self := &CorexyKinematics{}
	self.Printer = config.Get_printer()
	// Setup axis rails
	rails := []*PrinterRail{}
	str_arr := []string{"x", "y", "z"}
	for _, n := range str_arr {
		rails = append(rails, LookupMultiRail(config.Getsection("stepper_"+n), true, nil, false))

	}

	for _, s := range rails[1].Get_steppers() {
		rails[0].Get_endstops()[0].Front().Value.(*MCU_endstop).Add_stepper(s)
	}
	for _, s := range rails[0].Get_steppers() {
		rails[1].Get_endstops()[0].Front().Value.(*MCU_endstop).Add_stepper(s)
	}

	self.Rails = rails
	length := len(self.Rails)
	if length < len(str_arr) {
		length = len(str_arr)
	}
	str_arr2 := []string{"+", "-", "z"}
	for i := 0; i < length; i++ {
		rail := self.Rails[i]
		axis := str_arr2[i]
		if i < 2 {
			rail.Setup_itersolve("corexy_stepper_alloc", []byte(axis)[0])
		} else {
			rail.Setup_itersolve("cartesian_stepper_alloc", []byte(axis)[0])
		}
	}
	for _, s := range self.Get_steppers() {
		s.(*MCU_stepper).Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.(*MCU_stepper).Generate_steps)
	}
	self.Printer.Register_event_handler("stepper_enable:motor_off",
		self.Motor_off)
	// Setup boundary checks
	max_velocity, max_accel := toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat("max_z_velocity", max_velocity, 0, max_velocity, 0., 0, true)
	self.Max_z_accel = config.Getfloat("max_z_accel", max_accel, 0, max_accel, 0., 0, true)
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	var ranges [][]float64
	for _, r := range self.Rails {
		position_min, position_max := r.Get_range()
		ranges = append(ranges, []float64{position_min, position_max})
	}
	var ranges_r0_arr []float64
	var ranges_r1_arr []float64
	for _, r := range ranges {
		ranges_r0_arr = append(ranges_r0_arr, r[0])
		ranges_r1_arr = append(ranges_r1_arr, r[1])
	}
	self.Axes_min = []float64{ranges_r0_arr[0], ranges_r0_arr[1], ranges_r0_arr[2], 0}
	self.Axes_max = []float64{ranges_r1_arr[0], ranges_r1_arr[1], ranges_r1_arr[2], 0}
	return self
}
func (self *CorexyKinematics) Get_steppers() []interface{} {
	rails := []*PrinterRail{}
	var steppers []interface{}
	rails = append([]*PrinterRail{}, self.Rails...)
	for _, rail := range rails {
		for _, s := range rail.Get_steppers() {
			steppers = append(steppers, s)
		}
	}
	return steppers
}

func (self *CorexyKinematics) Calc_position(stepper_positions map[string]float64) []float64 {
	var pos []float64
	for _, rail := range self.Rails {
		pos = append(pos, stepper_positions[rail.Get_name(false)])
	}
	var newpos = []float64{0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]}
	return newpos
}
func (self *CorexyKinematics) Set_position(newpos []float64, homing_axes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if collections.InInt(i, homing_axes) {
			self.Limits[i][0], self.Limits[i][1] = rail.Get_range()
		}
	}
}
func (self *CorexyKinematics) Note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}

func (self *CorexyKinematics) Home(homing_state *Homing) {
	// Each axis is homed independently and in order
	for _, axis := range homing_state.Get_axes() {
		rail := self.Rails[axis]
		//# Determine movement
		position_min, position_max := rail.Get_range()
		hi := rail.Get_homing_info()
		homepos := []interface{}{nil, nil, nil, nil}
		homepos[axis] = hi.Position_endstop
		forcepos := make([]interface{}, len(homepos))
		copy(forcepos, homepos)
		if hi.Positive_dir {
			forcepos[axis] = forcepos[axis].(float64) - 1.5*(hi.Position_endstop-position_min)
		} else {
			forcepos[axis] = forcepos[axis].(float64) + 1.5*(position_max-hi.Position_endstop)
		}
		// Perform homing
		homing_state.Home_rails([]*PrinterRail{rail}, forcepos, homepos)
	}
}

func (self *CorexyKinematics) Motor_off(argv []interface{}) error {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	return nil
}

func (self *CorexyKinematics) Check_endstops(move *Move) error {
	end_pos := move.End_pos
	for i := 0; i < 3; i++ {
		if move.Axes_d[i] != 0.0 &&
			(maths.Check_below_limit(end_pos[i], self.Limits[i][0]) ||
				maths.Check_above_limit(end_pos[i], self.Limits[i][1])) {

			if self.Limits[i][0] > self.Limits[i][1] {
				return move.Move_error("Must home axis first")
			}
			return move.Move_error("Move out of range")
		}
	}
	return nil
}

func (self *CorexyKinematics) Check_move(move *Move) {
	limits := self.Limits
	xpos := move.End_pos[0]
	ypos := move.End_pos[1]

	if maths.Check_below_limit(xpos, limits[0][0]) ||
		maths.Check_above_limit(xpos, limits[0][1]) ||
		maths.Check_below_limit(ypos, limits[1][0]) ||
		maths.Check_above_limit(ypos, limits[1][1]) {

		err := self.Check_endstops(move)
		if err != nil {
			panic(err)
		}
	}
	if move.Axes_d[2] == 0 {
		// Normal XY move - use defaults
		return
	}
	// Move with Z - update velocity and accel for slower Z axis
	self.Check_endstops(move)
	z_ratio := move.Move_d / math.Abs(move.Axes_d[2])
	move.Limit_speed(self.Max_z_velocity*z_ratio, self.Max_z_accel*z_ratio)
}

func (self *CorexyKinematics) Get_status(eventtime float64) map[string]interface{} {
	str_arr := []string{"x", "y", "z"}
	var axes []string
	for i, str := range str_arr {
		l := self.Limits[i][0]
		h := self.Limits[i][1]
		if l <= h {
			axes = append(axes, str)
		}
	}
	return map[string]interface{}{
		"homed_axes":   strings.Join(axes, ""),
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
	}
}

func (self *CorexyKinematics) Get_axis_range(axis int) (float64, float64) {
	return self.Rails[axis].Get_range()
}

func Load_kinematics_corexy(toolhead *Toolhead, config *ConfigWrapper) interface{} {
	return NewCorexyKinematics(toolhead, config)
}
