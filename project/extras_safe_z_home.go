/*
# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"k3c/common/utils/object"
	"k3c/common/utils/reflects"
	//"k3c/common/value"

	"strings"
	//"reflect"
	//"strings"
)

type SafeZHoming struct {
	Printer              *Printer
	Home_x_pos           float64
	Home_y_pos           float64
	Custom_Home_x_re_pos float64
	Custom_Home_y_re_pos float64
	Z_hop                float64
	Z_hop_speed          float64
	Max_z                float64
	Speed                float64
	Move_to_previous     bool
	Gcode                *GCodeDispatch
	Prev_G28             func(interface{}) error
	Prev_L28             func(interface{}) error
}

func NewSafeZHoming(config *ConfigWrapper) *SafeZHoming {
	self := SafeZHoming{}
	self.Printer = config.Get_printer()
	pos := config.Getfloatlist("home_xy_position", nil, ",", 2, true)
	self.Home_x_pos = pos[0]
	self.Home_y_pos = pos[1]
	self.Z_hop = config.Getfloat("z_hop", 0.0, 0, 0, 0.0, 0, true)
	self.Z_hop_speed = config.Getfloat("z_hop_speed", 15., 0, 0, 0.0, 0, true)
	zconfig := config.Getsection("stepper_z")
	self.Max_z = zconfig.Getfloat("position_max", 0, 0, 0, 0, 0, false)
	self.Speed = config.Getfloat("speed", 50.0, 0, 0, 0.0, 0, true)
	self.Move_to_previous = config.Getboolean("move_to_previous", false, false)
	self.Printer.Load_object(config, "homing", object.Sentinel{})
	self.Gcode = MustLookupGcode(self.Printer)
	Prev_G28 := self.Gcode.Register_command("G28", nil, false, "")
	self.Prev_G28 = Prev_G28.(func(interface{}) error)

	self.Gcode.Register_command("G28", self.Cmd_G28, false, "")
	self.Gcode.Register_command("H28", self.Cmd_H28, false, "")

	if config.Has_section("homing_override") {
		panic("homing_override and safe_z_homing cannot be used simultaneously")
	}
	return &self
}

func (self *SafeZHoming) Cmd_G28(cmd interface{}) error {

	gcmd := cmd.(*GCodeCommand)
	toolhead := MustLookupToolhead(self.Printer)
	// Perform Z Hop if necessary
	if self.Z_hop != 0.0 {
		// Check if Z axis is homed and its last known position
		curtime := self.Printer.Get_reactor().Monotonic()
		kin_status := toolhead.Get_kinematics().(IKinematics).Get_status(curtime)
		pos := toolhead.Get_position()

		if strings.Index(kin_status["homed_axes"].(string), "z") == -1 {
			pos[2] = 0
			toolhead.Set_position(pos, []int{2})
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
			if reflects.Hasattr(toolhead.Get_kinematics(), "Note_z_not_homed") {
				toolhead.Get_kinematics().(IKinematics).Note_z_not_homed()
			}
		} else if pos[2] < self.Z_hop {
			// If the Z axis is homed, and below z_hop, lift it to z_hop
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
		}
	}
	// Determine which axes we need to home
	need_x := gcmd.Has("X")
	need_y := gcmd.Has("Y")
	need_z := gcmd.Has("Z")

	if !need_x && !need_y && !need_z {
		need_x, need_y, need_z = true, true, true
	}
	// Home XY axes if necessary
	new_params := map[string]string{}
	if need_x {
		new_params["X"] = "0"
	}
	if need_y {
		new_params["Y"] = "0"
	}
	if len(new_params) > 0 {
		g28_gcmd := self.Gcode.Create_gcode_command("G28", "G28", new_params)
		self.Prev_G28(g28_gcmd)
	}
	// Home Z axis if necessary
	if need_z {
		// Throw an error if X or Y are not homed
		curtime := self.Printer.Get_reactor().Monotonic()
		kin_status := toolhead.Get_kinematics().(IKinematics).Get_status(curtime)
		if strings.Index(kin_status["homed_axes"].(string), "x") == -1 ||
			strings.Index(kin_status["homed_axes"].(string), "y") == -1 {
			panic("Must home X and Y axes first")
		}
		// Move to safe XY homing position
		prevpos := toolhead.Get_position()
		home_x_pos, home_y_pos := self.Home_x_pos, self.Home_y_pos
		toolhead.Manual_move([]interface{}{home_x_pos, home_y_pos}, self.Speed)
		// Home Z
		g28_gcmd := self.Gcode.Create_gcode_command("G28", "G28", map[string]string{"Z": "0"})
		self.Prev_G28(g28_gcmd)
		// Perform Z Hop again for pressure-based probes
		if self.Z_hop != 0.0 {
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
		}
		// Move XY back to previous positions
		if self.Move_to_previous {
			aa := []interface{}{}
			for _, v := range prevpos[:2] {
				aa = append(aa, v)
			}
			toolhead.Manual_move(aa, self.Speed)
		}
	}
	return nil
}

func (self *SafeZHoming) Cmd_H28(cmd interface{}) error {
	gcmd := cmd.(*GCodeCommand)
	toolhead := MustLookupToolhead(self.Printer)
	// Perform Z Hop if necessary
	if self.Z_hop != 0.0 {
		// Check if Z axis is homed and its last known position
		curtime := self.Printer.Get_reactor().Monotonic()
		kin_status := toolhead.Get_kinematics().(IKinematics).Get_status(curtime)
		pos := toolhead.Get_position()

		if strings.Index(kin_status["homed_axes"].(string), "z") == -1 {
			pos[2] = 0
			toolhead.Set_position(pos, []int{2})
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
			if reflects.Hasattr(toolhead.Get_kinematics(), "Note_z_not_homed") {
				toolhead.Get_kinematics().(IKinematics).Note_z_not_homed()
			}
		} else if pos[2] < self.Z_hop {
			// If the Z axis is homed, and below z_hop, lift it to z_hop
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
		}
	}
	// Determine which axes we need to home
	need_x := gcmd.Has("X")
	need_y := gcmd.Has("Y")
	need_z := gcmd.Has("Z")

	if !need_x && !need_y && !need_z {
		need_x, need_y, need_z = true, true, true
	}
	curtime := self.Printer.Get_reactor().Monotonic()
	kin_status := toolhead.Get_kinematics().(IKinematics).Get_status(curtime)
	// Home XY axes if necessary
	new_params := map[string]string{}
	if need_x && strings.Index(kin_status["homed_axes"].(string), "x") == -1 {
		new_params["X"] = "0"
	}
	if need_y && strings.Index(kin_status["homed_axes"].(string), "y") == -1 {
		new_params["Y"] = "0"
	}

	if len(new_params) > 0 {
		g28_gcmd := self.Gcode.Create_gcode_command("G28", "G28", new_params)
		self.Prev_G28(g28_gcmd)
	}
	// Home Z axis if necessary
	if need_z && strings.Index(kin_status["homed_axes"].(string), "z") == -1 {
		kin_status = toolhead.Get_kinematics().(IKinematics).Get_status(curtime)
		// Throw an error if X or Y are not homed
		if strings.Index(kin_status["homed_axes"].(string), "x") == -1 ||
			strings.Index(kin_status["homed_axes"].(string), "y") == -1 {
			panic("Must home X and Y axes first")
		}
		// Move to safe XY homing position
		prevpos := toolhead.Get_position()
		home_x_pos, home_y_pos := self.Home_x_pos, self.Home_y_pos
		toolhead.Manual_move([]interface{}{home_x_pos, home_y_pos}, self.Speed)
		// Home Z
		g28_gcmd := self.Gcode.Create_gcode_command("G28", "G28", map[string]string{"Z": "0"})
		self.Prev_G28(g28_gcmd)
		// Perform Z Hop again for pressure-based probes
		if self.Z_hop != 0.0 {
			toolhead.Manual_move([]interface{}{nil, nil, self.Z_hop}, self.Z_hop_speed)
		}
		// Move XY back to previous positions
		if self.Move_to_previous {
			aa := []interface{}{}
			for _, v := range prevpos[:2] {
				aa = append(aa, v)
			}
			toolhead.Manual_move(aa, self.Speed)
		}
	}
	return nil
}
func Load_config_safe_z_home(config *ConfigWrapper) interface{} {
	return NewSafeZHoming(config)
}
