package project

import (
	"fmt"
	"k3c/common/utils/object"
	"math"
	"strings"
)

/*
# Helper script to adjust parameters based on Z level
#
# Copyright (C) 2019  Kevin O"Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/

const CANCEL_Z_DELTA = 2.0

type TuningTower struct {
	Printer            *Printer
	Normal_transform   Itransform
	Last_position      []float64
	Last_z             float64
	Start              float64
	Factor             float64
	Band               float64
	Last_command_value *float64
	Command_fmt        string
	Gcode_move         *GCodeMove
	Gcode              *GCodeDispatch
	Step_height        float64
	Step_delta         float64
	Skip               float64
}

func NewTuningTower(config *ConfigWrapper) *TuningTower {
	self := &TuningTower{}
	self.Printer = config.Get_printer()
	self.Normal_transform = nil
	self.Last_position = []float64{0., 0., 0., 0.}
	self.Last_z = 0.
	self.Start = 0.
	self.Factor = 0.
	self.Band = 0.
	zero := 0.
	self.Last_command_value = &zero
	self.Command_fmt = ""
	self.Gcode_move = self.Printer.Load_object(config, "gcode_move", object.Sentinel{}).(*GCodeMove)
	// Register command
	self.Gcode = self.Printer.Load_object(config, "gcode", object.Sentinel{}).(*GCodeDispatch)
	self.Gcode.Register_command("TUNING_TOWER", self.Cmd_TUNING_TOWER, false,
		cmd_TUNING_TOWER_help)
	return self
}

const cmd_TUNING_TOWER_help = "Tool to adjust a parameter at each Z height"

func (self *TuningTower) Cmd_TUNING_TOWER(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	if self.Normal_transform != nil {
		self.End_test()
	}
	// Get parameters
	command := gcmd.Get("COMMAND", object.Sentinel{}, nil, nil, nil, nil, nil)
	parameter := gcmd.Get("PARAMETER", object.Sentinel{}, nil, nil, nil, nil, nil)
	self.Start = gcmd.Get_float("START", 0., nil, nil, nil, nil)
	self.Factor = gcmd.Get_float("FACTOR", 0., nil, nil, nil, nil)
	maxval := 0.0
	self.Band = gcmd.Get_float("BAND", 0., &maxval, nil, nil, nil)
	self.Step_delta = gcmd.Get_float("STEP_DELTA", 0., nil, nil, nil, nil)
	self.Step_height = gcmd.Get_float("STEP_HEIGHT", 0., &maxval, nil, nil, nil)
	self.Skip = gcmd.Get_float("SKIP", 0., &maxval, nil, nil, nil)
	if self.Factor != 0.0 && (self.Step_height != 0.0 || self.Step_delta != 0.0) {
		panic("Must specify both STEP_DELTA and STEP_HEIGHT")
	}
	if (self.Step_delta != 0.) != (self.Step_height != 0.) {
		panic("Must specify both STEP_DELTA and STEP_HEIGHT")
	}
	// Enable test mode
	if self.Gcode.Is_traditional_gcode(command) {
		self.Command_fmt = fmt.Sprintf("%s %s%%.9f", command, parameter)
	} else {
		self.Command_fmt = fmt.Sprintf("%s %s=%%.9f", command, parameter)
	}
	nt := self.Gcode_move.Set_move_transform(self, true)
	self.Normal_transform = nt
	self.Last_z = -99999999.9
	self.Last_command_value = nil
	self.Get_position()
	message_parts := []string{}
	message_parts = append(message_parts, fmt.Sprintf("start=%.6f", self.Start))
	if self.Factor != 0.0 {
		message_parts = append(message_parts, fmt.Sprintf("factor=%.6f", self.Factor))
		if self.Band != 0.0 {
			message_parts = append(message_parts, fmt.Sprintf("band=%.6f", self.Band))
		}
	} else {
		message_parts = append(message_parts, fmt.Sprintf("step_delta=%.6f", self.Step_delta))
		message_parts = append(message_parts, fmt.Sprintf("step_height=%.6f", self.Step_height))
	}
	if self.Skip != 0.0 {
		message_parts = append(message_parts, fmt.Sprintf("skip=%.6f", self.Skip))
	}
	gcmd.Respond_info("Starting tuning test ("+strings.Join(message_parts, " ")+")", true)
	return nil
}
func (self *TuningTower) Get_position() []float64 {
	pos := self.Normal_transform.Get_position()
	copy(self.Last_position, pos)
	return pos
}
func (self *TuningTower) Calc_value(z float64) float64 {
	if self.Skip != 0.0 {
		z = math.Max(0., z-self.Skip)
	}
	if self.Step_height != 0.0 {
		return self.Start + self.Step_delta*math.Floor(z/self.Step_height)
	}
	if self.Band != 0.0 {
		z = (math.Floor(z/self.Band) + .5) * self.Band
	}
	return self.Start + z*self.Factor
}
func (self *TuningTower) Move(newpos []float64, speed float64) {
	normal_transform := self.Normal_transform
	is_eq := true
	for i := 0; i < 3; i++ {
		if newpos[i] != self.Last_position[i] {
			is_eq = false
			break
		}
	}
	if newpos[3] > self.Last_position[3] && newpos[2] != self.Last_z && is_eq {
		// Extrusion move at new z height
		z := newpos[2]
		if z < self.Last_z-CANCEL_Z_DELTA {
			// Extrude at a lower z height - probably start of new print
			self.End_test()
		} else {
			// Process update
			gcode_z := self.Gcode_move.Get_status(0.)["gcode_position"].([]float64)[2]
			newval := self.Calc_value(gcode_z)
			self.Last_z = z
			if newval != *self.Last_command_value {
				self.Last_command_value = &newval
				self.Gcode.Run_script_from_command(fmt.Sprintf(self.Command_fmt, newval))
			}
		}
	}
	// Forward move to actual handler
	self.Last_position = newpos
	normal_transform.Move(newpos, speed)
}
func (self *TuningTower) End_test() {
	self.Gcode.Respond_info("Ending tuning test mode", true)
	self.Gcode_move.Set_move_transform(self.Normal_transform, true)
	self.Normal_transform = nil
}

func (self *TuningTower) Is_active() bool {
	return self.Normal_transform != nil
}
func Load_config_tuning_tower(config *ConfigWrapper) {
	config.Get_printer().Add_object("tuning_tower", NewTuningTower(config))
}
