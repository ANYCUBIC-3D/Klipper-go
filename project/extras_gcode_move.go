// G-Code G1 movement commands (and associated coordinate manipulation)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/reflects"
	"math"
	"reflect"
	"strconv"
	"strings"
	"sync"
	//"k3c/common/utils/sys"
)

type Itransform interface {
	Move([]float64, float64)
	Get_position() []float64
}

type GCodeMove struct {
	printer                      *Printer
	is_printer_ready             bool
	Coord                        []string
	absolute_coord               bool
	absolute_extrude             bool
	base_position                []float64
	last_position                []float64
	homing_position              []float64
	speed                        float64
	speed_factor                 float64
	extrude_factor               float64
	exclude_extrude              float64
	Saved_states                 map[string]*Saved_states
	move_transform               Itransform
	move_with_transform          func([]float64, float64)
	position_with_transform      func() []float64
	Cmd_SET_GCODE_OFFSET_help    string
	Cmd_SAVE_GCODE_STATE_help    string
	Cmd_RESTORE_GCODE_STATE_help string
	Cmd_GET_POSITION_help        string
	Lock                         sync.RWMutex
	kin                          IKinematics
}

type Saved_states struct {
	Absolute_coord   bool      `json:"absolute_coord"`
	Absolute_extrude bool      `json:"absolute_extrude"`
	Base_position    []float64 `json:"base_position"`
	Last_position    []float64 `json:"last_position"`
	Homing_position  []float64 `json:"homing_position"`
	Speed            float64   `json:"speed"`
	Speed_factor     float64   `json:"speed_factor"`
	Extrude_factor   float64   `json:"extrude_factor"`
}

func NewGCodeMove(config *ConfigWrapper) *GCodeMove {
	self := GCodeMove{}
	self.printer = config.Get_printer()
	var printer = self.printer
	printer.Register_event_handler("project:ready", self._handle_ready)
	printer.Register_event_handler("project:shutdown", self._handle_shutdown)
	printer.Register_event_handler("toolhead:set_position",
		self.Reset_last_position)
	printer.Register_event_handler("toolhead:manual_move",
		self.Reset_last_position)
	printer.Register_event_handler("gcode:command_error",
		self.Reset_last_position)
	printer.Register_event_handler("extruder:activate_extruder",
		self._handle_activate_extruder)
	printer.Register_event_handler("homing:home_rails_end",
		self._handle_home_rails_end)
	self.is_printer_ready = false
	self.Cmd_SET_GCODE_OFFSET_help = "Set a virtual offset to g-code positions"
	self.Cmd_SAVE_GCODE_STATE_help = "Save G-Code coordinate state"
	self.Cmd_RESTORE_GCODE_STATE_help = "Restore a previously saved G-Code state"
	self.Cmd_GET_POSITION_help = "Return information on the current location of the toolhead"
	// Register g-code commands
	gcode := MustLookupGcode(printer)
	var handlers = [...]string{
		"G1", "G20", "G21",
		"M82", "M83", "G60", "G61", "G90", "G91", "G92", "M220", "M221",
		"SET_GCODE_OFFSET", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE",
	}

	for _, cmd := range handlers {
		var _func = reflects.GetMethod(&self, "Cmd_"+cmd)
		var desc = reflects.ReflectFieldValue(&self, "Cmd_"+cmd+"_help")
		if desc == nil {
			desc = ""
		}
		gcode.Register_command(cmd, _func, false, desc.(string))
	}

	gcode.Register_command("G0", self.Cmd_G1, false, "")
	gcode.Register_command("M114", self.Cmd_M114, true, "")
	gcode.Register_command("GET_POSITION", self.Cmd_GET_POSITION, true, "")

	self.Coord = append([]string{}, gcode.Coord...)
	// G-Code coordinate manipulation
	self.absolute_coord = true
	self.absolute_extrude = true
	self.base_position = []float64{0.0, 0.0, 0.0, 0.0}
	self.last_position = []float64{0.0, 0.0, 0.0, 0.0}
	self.homing_position = []float64{0.0, 0.0, 0.0, 0.0}
	self.speed = 25.0
	self.speed_factor = 1.0 / 60.0
	self.extrude_factor = 1.0
	// G-Code state
	self.Saved_states = map[string]*Saved_states{}
	self.move_transform = nil
	self.move_with_transform = nil
	self.position_with_transform = func() []float64 { return []float64{0.0, 0.0, 0.0, 0.0} }
	return &self
}

func (self *GCodeMove) _handle_ready(args []interface{}) error {
	self.is_printer_ready = true
	toolhead := MustLookupToolhead(self.printer)
	if self.move_transform == nil {
		self.move_with_transform = toolhead.Move
		self.position_with_transform = toolhead.Get_position
	}
	self.kin = toolhead.Get_kinematics().(IKinematics)
	self.Reset_last_position(nil)
	return nil
}

func (self *GCodeMove) _handle_shutdown(args []interface{}) error {
	if self.is_printer_ready == false {
		return nil
	}

	self.is_printer_ready = false
	logger.Infof("gcode state: absolute_coord=%v absolute_extrude=%v "+
		"base_position=%v last_position=%v "+
		"homing_position=%v speed_factor=%v "+
		"extrude_factor=%v speed=%v",
		self.absolute_coord, self.absolute_extrude,
		self.base_position, self.last_position,
		self.homing_position, self.speed_factor,
		self.extrude_factor, self.speed)
	return nil
}

func (self *GCodeMove) _handle_activate_extruder(args []interface{}) error {
	_ = self.Reset_last_position(nil)
	self.extrude_factor = 1.
	self.base_position[3] = self.last_position[3]
	return nil
}

func (self *GCodeMove) _handle_home_rails_end(args []interface{}) error {
	self.Reset_last_position(nil)
	homing_state := args[0].(*Homing)
	for _, axis := range homing_state.Get_axes() {
		self.base_position[axis] = self.homing_position[axis]
	}
	return nil
}

func (self *GCodeMove) Set_move_transform(transform Itransform, force bool) Itransform {

	if self.move_transform != nil && !force {
		panic("G-Code move transform already specified")
	}

	var old_transform = self.move_transform
	if old_transform == nil {
		toolhead := self.printer.Lookup_object("toolhead", nil)
		if toolhead != nil {
			old_transform = toolhead.(*Toolhead).Get_transform()
		}
	}
	self.move_transform = transform
	self.move_with_transform = transform.Move
	self.position_with_transform = transform.Get_position
	return old_transform
}

func (self *GCodeMove) _get_gcode_position() []float64 {
	var p []float64
	length := int(math.Min(float64(len(self.last_position)), float64(len(self.base_position))))
	for i := 0; i < length; i++ {
		lp := self.last_position[i]
		bp := self.base_position[i]
		p = append(p, lp-bp)
	}

	p[3] /= self.extrude_factor
	return p
}

func (self *GCodeMove) _get_gcode_speed() float64 {
	return self.speed / self.speed_factor
}

func (self *GCodeMove) _get_gcode_speed_override() float64 {
	return self.speed_factor * 60.
}

func (self *GCodeMove) Get_status(eventtime float64) map[string]interface{} {
	move_position := self._get_gcode_position()

	return map[string]interface{}{
		"speed_factor":         self._get_gcode_speed_override(),
		"speed":                self._get_gcode_speed(),
		"extrude_factor":       self.extrude_factor,
		"absolute_coordinates": self.absolute_coord,
		"absolute_extrude":     self.absolute_extrude,
		"homing_origin":        []float64{self.homing_position[0], self.homing_position[1], self.homing_position[2], self.homing_position[3]},
		"position":             []float64{self.last_position[0], self.last_position[1], self.last_position[2], self.last_position[3]},
		"gcode_position":       []float64{move_position[0], move_position[1], move_position[2], move_position[3]},
	}
}

func (self *GCodeMove) Reset_last_position(args []interface{}) error {
	if self.is_printer_ready == true {
		self.last_position = self.position_with_transform()
	}
	return nil
}

// G-Code movement commands
func (self *GCodeMove) Cmd_G1(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	var params = gcmd.Get_command_parameters()
	for pos, axis := range strings.Split("X Y Z", " ") {
		if _, ok := params[axis]; ok {
			v, err := strconv.ParseFloat(params[axis], 64)
			if err != nil {
				logger.Error(err)
			}
			if self.absolute_coord == false {
				// value relative to position of last move
				source_last_position := make([]float64, len(self.last_position))
				copy(source_last_position, self.last_position)
				self.last_position[pos] += v
			} else {
				// value relative to base coordinate position
				self.last_position[pos] = v + self.base_position[pos]
			}
		}
	}

	if _, ok := params["E"]; ok {
		e_val, err := strconv.ParseFloat(params["E"], 64)
		if err != nil {
			logger.Error(err)
		}
		var v = e_val * self.extrude_factor
		if !self.absolute_coord || !self.absolute_extrude {
			// value relative to position of last move
			self.last_position[3] += v
		} else {
			// value relative to base coordinate position
			self.last_position[3] = v + self.base_position[3]
		}
	}

	if _, ok := params["F"]; ok {
		gcode_speed, err := strconv.ParseFloat(params["F"], 64)
		if err != nil {
			panic(fmt.Sprintf("Unable to parse move '%s'",
				gcmd.Get_commandline()))
		}
		if gcode_speed <= 0 {
			panic(fmt.Sprintf("Invalid speed in '%s'",
				gcmd.Get_commandline()))
		}
		self.speed = gcode_speed * self.speed_factor
	}
	self.move_with_transform(self.last_position, self.speed)
	return nil
}

// G-Code coordinate manipulation
func (self *GCodeMove) Cmd_G20(argv interface{}) error {
	// Set units to inches
	panic("Machine does not support G20 (inches) command")
	return nil
}

func (self *GCodeMove) Cmd_G21(argv interface{}) error {
	// Set units to millimeters
	return nil
}

func (self *GCodeMove) Cmd_M82(argv interface{}) error {
	// Use absolute distances for extrusion
	self.absolute_extrude = true
	return nil
}
func (self *GCodeMove) Cmd_M83(argv interface{}) error {
	// Use relative distances for extrusion
	self.absolute_extrude = false
	return nil
}
func (self *GCodeMove) Cmd_G90(argv interface{}) error {
	// Use absolute coordinates
	self.absolute_coord = true
	return nil
}

func (self *GCodeMove) Cmd_G91(argv interface{}) error {
	// Use relative coordinates
	self.absolute_coord = false
	return nil
}

func (self *GCodeMove) Cmd_G92(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Set position
	var offsets []interface{}
	for _, a := range strings.Split("X Y Z E", " ") {
		f := gcmd.Get_float(a, math.NaN(), nil, nil, nil, nil)
		if math.IsNaN(f) == true {
			offsets = append(offsets, nil)
		} else {
			offsets = append(offsets, f)
		}
	}

	for i, offset := range offsets {
		if offset != nil {
			if i == 3 {
				offset = offset.(float64) * self.extrude_factor
			}
			self.base_position[i] = self.last_position[i] - offset.(float64)
		}
	}

	if reflect.DeepEqual(offsets, []interface{}{nil, nil, nil, nil}) == true {
		copy(self.base_position, self.last_position)
	}
	return nil
}

func (self *GCodeMove) Cmd_M114(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Get Current Position
	var p = self._get_gcode_position()
	gcmd.Respond_raw(fmt.Sprintf("X:%.3f Y:%.3f Z:%.3f E:%.3f", p[0], p[1], p[2], p[3]))
	return nil
}

func (self *GCodeMove) Cmd_M220(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Set speed factor override percentage
	zero := 0.
	val := gcmd.Get_float("S", 100., nil, nil, &zero, nil) / (60. * 100.)
	self.speed = self._get_gcode_speed() * val
	self.speed_factor = val
	return nil
}
func (self *GCodeMove) cmd_M221(argv interface{}) error {
	//# Set extrude factor override percentage
	gcmd := argv.(*GCodeCommand)
	above := 0.
	new_extrude_factor := gcmd.Get_float("S", 100., nil, nil, &above, nil) / 100.
	last_e_pos := self.last_position[3]
	e_value := (last_e_pos - self.base_position[3]) / self.extrude_factor
	self.base_position[3] = last_e_pos - e_value*new_extrude_factor
	self.extrude_factor = new_extrude_factor
	return nil
}

func (self *GCodeMove) Cmd_SET_GCODE_OFFSET(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	var move_delta = []float64{0., 0., 0., 0.}
	for pos, axis := range strings.Split("X Y Z E", " ") {
		var offset = gcmd.Get_float(axis, math.NaN(), nil, nil, nil, nil)
		if math.IsNaN(offset) {
			offset = gcmd.Get_float(axis+"_ADJUST", math.NaN(), nil, nil, nil, nil)
			if math.IsNaN(offset) {
				continue
			}
			offset += self.homing_position[pos]
		}
		var delta = offset - self.homing_position[pos]
		move_delta[pos] = delta
		self.base_position[pos] += delta
		self.homing_position[pos] = offset
	}
	// Move the toolhead the given offset if requested
	if gcmd.Get_int("MOVE", 0, nil, nil) != 0 {
		var speed = gcmd.Get_float("MOVE_SPEED", self.speed, nil, nil, nil, nil)
		for pos, delta := range move_delta {
			self.last_position[pos] += delta
		}
		self.move_with_transform(self.last_position, speed)
	}
}

func (self *GCodeMove) Cmd_SAVE_GCODE_STATE(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	state_name := gcmd.Get("NAME", "default", nil, nil, nil, nil, nil)
	base_position_back := make([]float64, len(self.base_position))
	last_position_back := make([]float64, len(self.last_position))
	homing_position_back := make([]float64, len(self.homing_position))
	copy(base_position_back, self.base_position)
	copy(last_position_back, self.last_position)
	copy(homing_position_back, self.homing_position)
	sta := &Saved_states{}
	sta.Absolute_coord = self.absolute_coord
	sta.Absolute_extrude = self.absolute_extrude
	sta.Base_position = base_position_back
	sta.Last_position = last_position_back
	sta.Homing_position = homing_position_back
	sta.Speed = self.speed
	sta.Speed_factor = self.speed_factor
	sta.Extrude_factor = self.extrude_factor
	self.Saved_states[state_name] = sta
}

func (self *GCodeMove) Cmd_RESTORE_GCODE_STATE(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	var state_name = gcmd.Get("NAME", "default", nil, nil, nil, nil, nil)
	var state = self.Saved_states[state_name]
	if state == nil {
		panic(fmt.Sprintf("Unknown g-code state: %s", state_name))
	}
	// Restore state
	base_position_back := make([]float64, len(state.Base_position))
	homing_position_back := make([]float64, len(state.Homing_position))
	copy(base_position_back, state.Base_position)
	copy(homing_position_back, state.Homing_position)
	self.absolute_coord = state.Absolute_coord
	self.absolute_extrude = state.Absolute_extrude
	self.base_position = base_position_back
	self.homing_position = homing_position_back
	self.speed = state.Speed
	self.speed_factor = state.Speed_factor
	self.extrude_factor = state.Extrude_factor
	e_diff := self.last_position[3] - state.Last_position[3]
	self.base_position[3] += e_diff
	// Move the toolhead back if requested
	if gcmd.Get_int("MOVE", 0, nil, nil) != 0 {
		zero := 0.
		var speed = gcmd.Get_float("MOVE_SPEED", self.speed, nil, nil, &zero, nil)
		self.last_position[0] = state.Last_position[0]
		self.last_position[1] = state.Last_position[1]
		self.last_position[2] = state.Last_position[2]
		self.move_with_transform(self.last_position, speed)
	}
}

func (self *GCodeMove) Cmd_GET_POSITION(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	var toolhead_obj = self.printer.Lookup_object("toolhead", nil)
	if toolhead_obj == nil {
		panic("Printer not ready")
	}
	toolhead := toolhead_obj.(*Toolhead)
	var kin = toolhead.Get_kinematics().(*CartKinematics)
	var steppers = kin.Get_steppers()

	var mcu_pos strings.Builder
	for _, s := range steppers {
		var str = fmt.Sprintf("%s:%d ", s.(*MCU_stepper).Get_name(false), s.(*MCU_stepper).Get_mcu_position())
		mcu_pos.Write([]byte(str))
	}

	var cinfo map[string]float64
	cinfo = make(map[string]float64)

	var stepper_pos strings.Builder
	for _, s := range steppers {
		cinfo[s.(*MCU_stepper).Get_name(false)] = s.(*MCU_stepper).Get_commanded_position()
		var str = fmt.Sprintf("%s:%.6f ", s.(*MCU_stepper).Get_name(false), s.(*MCU_stepper).Get_commanded_position())
		stepper_pos.Write([]byte(str))
	}

	var kinfo map[string][]float64
	kinfo = make(map[string][]float64)
	for _, s := range strings.Split("X Y Z", " ") {
		kinfo[s] = kin.Calc_position(cinfo)
	}

	var kin_pos strings.Builder
	for k, v := range kinfo {
		var str = fmt.Sprintf("%s:%.6f ", k, v)
		kin_pos.Write([]byte(str))
	}

	var toolhead_pos strings.Builder
	str_arr := []string{"X", "Y", "Z", "E"}
	float_arr := toolhead.Get_position()
	for i := 0; i < len(str_arr); i++ {
		a := str_arr[i]
		v := float_arr[i]
		toolhead_pos.Write([]byte(fmt.Sprintf("%s:%.6f", a, v)))
	}

	var gcode_pos strings.Builder
	for i := 0; i < len(str_arr); i++ {
		a := str_arr[i]
		v := self.last_position[i]
		gcode_pos.Write([]byte(fmt.Sprintf("%s:%.6f", a, v)))
	}

	var base_pos strings.Builder
	for i := 0; i < len(str_arr); i++ {
		a := str_arr[i]
		v := self.base_position[i]
		base_pos.Write([]byte(fmt.Sprintf("%s:%.6f", a, v)))
	}

	var homing_pos strings.Builder
	str_arr1 := []string{"X", "Y", "Z"}
	for i := 0; i < len(str_arr1); i++ {
		a := str_arr1[i]
		v := self.homing_position[i]
		homing_pos.Write([]byte(fmt.Sprintf("%s:%.6f", a, v)))
	}

	var msg = fmt.Sprintf("mcu: %s\n"+
		"stepper: %s\n"+
		"kinematic: %s\n"+
		"toolhead: %s\n"+
		"gcode: %s\n"+
		"gcode base: %s\n"+
		"gcode homing: %s",
		mcu_pos.String(), stepper_pos.String(), kin_pos.String(), toolhead_pos.String(),
		gcode_pos.String(), base_pos.String(), homing_pos.String())
	gcmd.Respond_info(msg, true)
}

func Load_config_gcode_move(config *ConfigWrapper) interface{} {
	return NewGCodeMove(config)
}
