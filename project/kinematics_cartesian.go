/*
Code for handling the kinematics of cartesian robots

Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"k3c/common/utils/cast"
	"k3c/common/utils/collections"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"math"
	"strconv"
	"strings"
)

type CartKinematics struct {
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

func NewCartKinematics(toolhead *Toolhead, config *ConfigWrapper) *CartKinematics {
	self := &CartKinematics{}
	self.Printer = config.Get_printer()
	// Setup axis rails
	self.Dual_carriage_axis = nil
	self.Dual_carriage_rails = []*PrinterRail{}
	rails := []*PrinterRail{}
	str_arr := []string{"x", "y", "z"}
	for _, n := range str_arr {
		rails = append(rails, LookupMultiRail(config.Getsection("stepper_"+n), true, nil, false))
	}
	self.Rails = rails
	length := len(self.Rails)
	if length < len(str_arr) {
		length = len(str_arr)
	}
	for i := 0; i < length; i++ {
		rail := self.Rails[i]
		axis := str_arr[i]
		rail.Setup_itersolve("cartesian_stepper_alloc", []byte(axis)[0])
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
	// Check for dual carriage support
	if config.Has_section("dual_carriage") {
		dc_config := config.Getsection("dual_carriage")
		dc_axis := dc_config.Getchoice("axis", map[interface{}]interface{}{"x": "x", "y": "y"}, object.Sentinel{}, true)
		self.Dual_carriage_axis = map[string]int{"x": 0, "y": 1}[dc_axis.(string)]
		dc_rail := LookupMultiRail(dc_config, true, nil, false)
		axis, _ := strconv.ParseUint(dc_axis.(string), 10, 32)
		dc_rail.Setup_itersolve("cartesian_stepper_alloc", axis)
		for _, s := range dc_rail.Get_steppers() {
			toolhead.Register_step_generator(s.Generate_steps)
		}
		self.Dual_carriage_rails = []*PrinterRail{self.Rails[cast.ToInt(self.Dual_carriage_axis)], dc_rail}
		gocode := self.Printer.Lookup_object("gcode", object.Sentinel{})
		gocode.(*GCodeDispatch).Register_command("SET_DUAL_CARRIAGE", self.Cmd_SET_DUAL_CARRIAGE, false, self.cmd_SET_DUAL_CARRIAGE_help())
	}
	return self
}
func (self *CartKinematics) Get_steppers() []interface{} {
	rails := []*PrinterRail{}
	var steppers []interface{}
	if self.Dual_carriage_axis != nil {
		dca, _ := self.Dual_carriage_axis.(int)
		rails = append(rails, self.Rails[:dca]...)
		rails = append(rails, self.Dual_carriage_rails...)
		rails = append(rails, self.Rails[dca+1:]...)
	} else {
		rails = append([]*PrinterRail{}, self.Rails...)
	}
	for _, rail := range rails {
		for _, s := range rail.Get_steppers() {
			steppers = append(steppers, s)
		}
	}

	return steppers
}
func (self *CartKinematics) Calc_position(stepper_positions map[string]float64) []float64 {
	var position_arr []float64
	for _, rail := range self.Rails {
		position_arr = append(position_arr, stepper_positions[rail.Get_name(false)])
	}
	return position_arr
}
func (self *CartKinematics) Set_position(newpos []float64, homing_axes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if collections.InInt(i, homing_axes) {
			self.Limits[i][0], self.Limits[i][1] = rail.Get_range()
		}
	}
}
func (self *CartKinematics) Note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}
func (self *CartKinematics) Home_axis(homing_state *Homing, axis int, rail *PrinterRail) {
	// Determine movement
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
func (self *CartKinematics) Home(homing_state *Homing) {
	// Each axis is homed independently and in order
	for _, axis := range homing_state.Get_axes() {
		if axis == self.Dual_carriage_axis {
			dc1 := self.Dual_carriage_rails[0]
			dc2 := self.Dual_carriage_rails[1]
			altc := 0
			if self.Rails[axis] == dc2 {
				altc = 1
			} else {
				altc = 0
			}
			self.Activate_carriage(0)
			self.Home_axis(homing_state, axis, dc1)
			self.Activate_carriage(1)
			self.Home_axis(homing_state, axis, dc2)
			self.Activate_carriage(altc)
		} else {
			self.Home_axis(homing_state, axis, self.Rails[axis])
		}
	}
}

func (self *CartKinematics) Motor_off(argv []interface{}) error {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	return nil
}

func (self *CartKinematics) Check_endstops(move *Move) error {
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

func (self *CartKinematics) Check_move(move *Move) {
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
func (self *CartKinematics) Get_status(eventtime float64) map[string]interface{} {
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

// Dual carriage support
func (self *CartKinematics) Activate_carriage(carriage int) {
	_toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{})
	toolhead := _toolhead.(*Toolhead)
	toolhead.Flush_step_generation()
	dc_rail := self.Dual_carriage_rails[carriage]
	dc_axis, _ := self.Dual_carriage_axis.(int)
	self.Rails[dc_axis].Set_trapq(nil)
	dc_rail.Set_trapq(toolhead.Get_trapq())
	self.Rails[dc_axis] = dc_rail
	pos := toolhead.Get_position()
	pos[dc_axis] = dc_rail.Get_commanded_position()
	toolhead.Set_position(pos, []int{})
	if self.Limits[dc_axis][0] <= self.Limits[dc_axis][1] {
		self.Limits[dc_axis][0], self.Limits[dc_axis][1] = dc_rail.Get_range()
	}
}
func (self *CartKinematics) cmd_SET_DUAL_CARRIAGE_help() string {
	return "Set which carriage is active"
}
func (self *CartKinematics) Cmd_SET_DUAL_CARRIAGE(gcmd *GCodeCommand) {
	minval := 0
	maxval := 1
	carriage := gcmd.Get_int("CARRIAGE", nil, &minval, &maxval)
	self.Activate_carriage(carriage)
}
func Load_kinematics_cartesian(toolhead *Toolhead, config *ConfigWrapper) interface{} {
	return NewCartKinematics(toolhead, config)
}
