/*
# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2022  Kevin O"Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 licensself.
*/
package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/value"
	"k3c/project/chelper"
	"math"
	"reflect"
	"strings"
)

type ExtruderStepper struct {
	Printer                      *Printer
	Name                         string
	Pressure_advance             interface{}
	Pressure_advance_smooth_time float64
	Config_pa                    float64
	Config_smooth_time           float64
	Stepper                      *MCU_stepper
	Sk_extruder                  interface{}
}

func NewExtruderStepper(config *ConfigWrapper) *ExtruderStepper {
	self := &ExtruderStepper{}
	self.Printer = config.Get_printer()
	name_arr := strings.Split(config.Get_name(), " ")
	self.Name = name_arr[len(name_arr)-1]
	self.Pressure_advance, self.Pressure_advance_smooth_time = 0., 0.
	self.Config_pa = config.Getfloat("pressure_advance", 0., 0., 0, 0, 0, true)
	self.Config_smooth_time = config.Getfloat(
		"pressure_advance_smooth_time", 0.040, 0., .200, .0, .0, true)
	// Setup stepper
	self.Stepper = PrinterStepper(config, false)
	//ffi_lib := chelper.Get_ffi()
	self.Sk_extruder = chelper.Extruder_stepper_alloc()
	//runtime.SetFinalizer(self,self._ExtruderStepper)
	self.Stepper.Set_stepper_kinematics(self.Sk_extruder)
	self.Printer.Register_event_handler("project:connect",
		self.Handle_connect)
	gcode := MustLookupGcode(self.Printer)
	if self.Name == "extruder" {
		gcode.Register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", "", self.Cmd_default_SET_PRESSURE_ADVANCE, cmd_SET_PRESSURE_ADVANCE_help)
	}

	gcode.Register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", self.Name, self.Cmd_SET_PRESSURE_ADVANCE, cmd_SET_PRESSURE_ADVANCE_help)
	gcode.Register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER", self.Name, self.Cmd_SET_E_ROTATION_DISTANCE, cmd_SET_E_ROTATION_DISTANCE_help)
	gcode.Register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER", self.Name, self.Cmd_SYNC_EXTRUDER_MOTION, cmd_SYNC_EXTRUDER_MOTION_help)
	gcode.Register_mux_command("SET_EXTRUDER_STEP_DISTANCE", "EXTRUDER", self.Name, self.Cmd_SET_E_STEP_DISTANCE, cmd_SET_E_STEP_DISTANCE_help)
	gcode.Register_mux_command("SYNC_STEPPER_TO_EXTRUDER", "STEPPER", self.Name, self.Cmd_SYNC_STEPPER_TO_EXTRUDER, cmd_SYNC_STEPPER_TO_EXTRUDER_help)
	return self
}
func (self *ExtruderStepper) _ExtruderStepper() {
	chelper.Free(self.Sk_extruder)
}
func (self *ExtruderStepper) Handle_connect([]interface{}) error {
	toolhead := MustLookupToolhead(self.Printer)
	toolhead.Register_step_generator(self.Stepper.Generate_steps)
	self.Set_pressure_advance(self.Config_pa, self.Config_smooth_time)
	return nil
}

func (self *ExtruderStepper) Get_status(eventtime float64) map[string]float64 {
	return map[string]float64{
		"pressure_advance": cast.ToFloat64(self.Pressure_advance),
		"smooth_time":      self.Pressure_advance_smooth_time,
	}
}

func (self *ExtruderStepper) Find_past_position(print_time float64) float64 {
	mcuPos := self.Stepper.Get_past_mcu_position(print_time)
	return self.Stepper.Mcu_to_commanded_position(mcuPos)
}

func (self *ExtruderStepper) Sync_to_extruder(extruder_name string) {
	toolhead := MustLookupToolhead(self.Printer)
	toolhead.Flush_step_generation()
	if extruder_name == "" {
		self.Stepper.Set_trapq(nil)
		return
	}
	extruder := self.Printer.Lookup_object(extruder_name, nil)
	if extruder == nil || !(reflect.TypeOf(extruder).Elem().Name() == "PrinterExtruder") {
		panic(fmt.Sprintf("%s' is not a valid extruder", extruder_name))
	}
	self.Stepper.Set_position([]float64{extruder.(*PrinterExtruder).Last_position, 0., 0.})
	self.Stepper.Set_trapq(extruder.(*PrinterExtruder).Get_trapq())
}
func (self *ExtruderStepper) Set_pressure_advance(pressureAdvance interface{}, smoothTime float64) {
	oldSmoothTime := self.Pressure_advance_smooth_time
	if self.Pressure_advance == nil {
		oldSmoothTime = 0
	}
	new_smooth_time := smoothTime
	if pressureAdvance == nil {
		new_smooth_time = 0
	}
	toolhead := MustLookupToolhead(self.Printer)
	toolhead.Note_step_generation_scan_time(new_smooth_time*0.5, oldSmoothTime*0.5)
	espa := chelper.Extruder_set_pressure_advance
	espa(self.Sk_extruder, cast.ToFloat64(pressureAdvance), new_smooth_time)
	self.Pressure_advance = pressureAdvance
	self.Pressure_advance_smooth_time = smoothTime
}

const cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"

func (self *ExtruderStepper) Cmd_default_SET_PRESSURE_ADVANCE(argv interface{}) error {
	//gcmd := argv[0].(*GCodeCommand)
	toolhead := MustLookupToolhead(self.Printer)
	extruder := toolhead.Get_extruder()
	if extruder.Get_extruder_stepper() == nil {
		panic("Active extruder does not have a stepper")
	}
	strapq := extruder.Get_extruder_stepper().Stepper.Get_trapq()
	if strapq != extruder.Get_trapq() {
		panic("Unable to infer active extruder stepper")
	}
	extruder.Get_extruder_stepper().Cmd_SET_PRESSURE_ADVANCE(argv)
	return nil
}
func (self *ExtruderStepper) Cmd_SET_PRESSURE_ADVANCE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	zero := 0.
	maxval := .200
	pressure_advance := 0.0
	pressure_advance = gcmd.Get_float("ADVANCE", self.Pressure_advance, &zero, nil, nil, nil)
	smooth_time := gcmd.Get_float("SMOOTH_TIME", self.Pressure_advance_smooth_time, &zero, &maxval, nil, nil)
	self.Set_pressure_advance(pressure_advance, smooth_time)
	msg := fmt.Sprintf("pressure_advance: %.6f\n pressure_advance_smooth_time: %.6f", pressure_advance, smooth_time)
	self.Printer.Set_rollover_info(self.Name, fmt.Sprintf("%s: %s", self.Name, msg), false)
	gcmd.Respond_info(msg, true)
	return nil
}

const cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance"

func (self *ExtruderStepper) Cmd_SET_E_ROTATION_DISTANCE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	rotationDist := gcmd.Get_float("DISTANCE", nil, nil, nil, nil, nil)
	if &rotationDist != nil {
		if rotationDist == 0.0 {
			panic("Rotation distance can not be zero")
		}
		_, origInvertDir := self.Stepper.Get_dir_inverted()
		next_invert_dir := origInvertDir
		if rotationDist < 0. {
			next_invert_dir = ^origInvertDir
			rotationDist = -rotationDist
		}
		toolhead := MustLookupToolhead(self.Printer)
		toolhead.Flush_step_generation()
		self.Stepper.Set_rotation_distance(rotationDist)
		self.Stepper.Set_dir_inverted(next_invert_dir)
	} else {
		rotationDist, _ = self.Stepper.Get_rotation_distance()
	}
	invertDir, origInvertDir := self.Stepper.Get_dir_inverted()
	if invertDir != origInvertDir {
		rotationDist = -rotationDist
	}
	gcmd.Respond_info(fmt.Sprintf("Extruder '%s' rotation distance set to %0.6f", self.Name, rotationDist), true)
	return nil
}

const cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"

func (self *ExtruderStepper) Cmd_SYNC_EXTRUDER_MOTION(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	ename := gcmd.Get("MOTION_QUEUE", object.Sentinel{}, "", nil, nil, nil, nil)
	self.Sync_to_extruder(ename)
	gcmd.Respond_info(fmt.Sprintf("Extruder stepper now syncing with '%s'", ename), true)
	return nil
}

const cmd_SET_E_STEP_DISTANCE_help = "Set extruder step distance"

func (self *ExtruderStepper) Cmd_SET_E_STEP_DISTANCE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	zero := 0.
	step_dist := gcmd.Get_float("DISTANCE", object.Sentinel{}, nil, nil, &zero, nil)
	if &step_dist != nil {
		toolhead := MustLookupToolhead(self.Printer)
		toolhead.Flush_step_generation()
		_, steps_per_rotation := self.Stepper.Get_rotation_distance()
		self.Stepper.Set_rotation_distance(step_dist * float64(steps_per_rotation))
	} else {
		step_dist = self.Stepper.Get_step_dist()
	}
	gcmd.Respond_info(fmt.Sprintf("Extruder '%s' step distance set to %0.6f",
		self.Name, step_dist), true)
	return nil
}

const cmd_SYNC_STEPPER_TO_EXTRUDER_help = "Set extruder stepper"

func (self *ExtruderStepper) Cmd_SYNC_STEPPER_TO_EXTRUDER(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	ename := gcmd.Get("EXTRUDER", object.Sentinel{}, "", nil, nil, nil, nil)
	self.Sync_to_extruder(ename)
	gcmd.Respond_info(fmt.Sprintf("Extruder stepper now syncing with '%s'", ename), true)
	return nil
}

// Tracking for hotend heater, extrusion motion queue, and extruder stepper
type PrinterExtruder struct {
	Printer           *Printer
	Name              string
	Last_position     float64
	Heater            interface{}
	Nozzle_diameter   float64
	Filament_area     float64
	Max_extrude_ratio float64
	Max_e_velocity    float64
	Max_e_accel       float64
	Max_e_dist        float64
	Instant_corner_v  float64
	Trapq             interface{}
	Trapq_append      func(tq interface{}, print_time,
		accel_t, cruise_t, decel_t,
		start_pos_x, start_pos_y, start_pos_z,
		axes_r_x, axes_r_y, axes_r_z,
		start_v, cruise_v, accel float64)
	Trapq_finalize_moves func(interface{}, float64, float64)
	Extruder_stepper     *ExtruderStepper
}

func NewPrinterExtruder(config *ConfigWrapper, extruder_num int) *PrinterExtruder {
	self := &PrinterExtruder{}
	self.Printer = config.Get_printer()
	self.Name = config.Get_name()
	self.Last_position = 0.
	// Setup hotend heater
	shared_heater := config.Get("shared_heater", value.None, true)
	pheaters := self.Printer.Load_object(config, "heaters", object.Sentinel{})
	gcode_id := fmt.Sprintf("T%d", extruder_num)
	if shared_heater == nil {
		self.Heater = pheaters.(*PrinterHeaters).Setup_heater(config, gcode_id)
	} else {
		config.Deprecate("shared_heater", "")
		self.Heater = pheaters.(*PrinterHeaters).Lookup_heater(shared_heater.(string))
	}
	// Setup kinematic checks
	self.Nozzle_diameter = config.Getfloat("nozzle_diameter", 0, 0, 0, 0., 0, true)
	filament_diameter := config.Getfloat("filament_diameter", 0, self.Nozzle_diameter, 0, 0., 0, true)
	self.Filament_area = math.Pi * math.Pow(filament_diameter*.5, 2)
	def_max_cross_section := 4. * self.Nozzle_diameter * self.Nozzle_diameter
	def_max_extrude_ratio := def_max_cross_section / self.Filament_area
	max_cross_section := config.Getfloat("max_extrude_cross_section", def_max_cross_section, 0, 0, 0., 0, true)
	self.Max_extrude_ratio = max_cross_section / self.Filament_area
	toolhead := MustLookupToolhead(self.Printer)
	max_velocity, max_accel := toolhead.Get_max_velocity()
	self.Max_e_velocity = config.Getfloat("max_extrude_only_velocity", max_velocity*def_max_extrude_ratio, 0, 0, 0., 0, true)
	self.Max_e_accel = config.Getfloat("max_extrude_only_accel", max_accel*def_max_extrude_ratio, 0, 0, 0., 0, true)
	self.Max_e_dist = config.Getfloat("max_extrude_only_distance", 50., 0, 0, 0., 0, true)
	self.Instant_corner_v = config.Getfloat("instantaneous_corner_velocity", 1., 0, 0, 0., 0, true)
	// Setup extruder trapq (trapezoidal motion queue)
	self.Trapq = chelper.Trapq_alloc()
	self.Trapq_append = chelper.Trapq_append
	self.Trapq_finalize_moves = chelper.Trapq_finalize_moves
	// Setup extruder stepper
	self.Extruder_stepper = nil
	if config.Get("step_pin", value.None, true) != nil ||
		config.Get("dir_pin", value.None, true) != nil ||
		config.Get("rotation_distance", value.None, true) != nil {

		self.Extruder_stepper = NewExtruderStepper(config)
		self.Extruder_stepper.Stepper.Set_trapq(self.Trapq)
	}
	// Register commands
	gcode := MustLookupGcode(self.Printer)
	if self.Name == "extruder" {
		toolhead.Set_extruder(self, 0.)
		gcode.Register_command("M104", self.Cmd_M104, false, "")
		gcode.Register_command("M109", self.Cmd_M109, false, "")
	}
	gcode.Register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER", self.Name,
		self.Cmd_ACTIVATE_EXTRUDER,
		cmd_ACTIVATE_EXTRUDER_help)
	return self
}

func (self *PrinterExtruder) _PrinterExtruder() {
	chelper.Trapq_free(self.Trapq)
}
func (self *PrinterExtruder) Update_move_time(flush_time float64, clear_history_time float64) {
	self.Trapq_finalize_moves(self.Trapq, flush_time, clear_history_time)
}
func (self *PrinterExtruder) Get_status(eventtime float64) map[string]interface{} {
	sts := self.Heater.(*Heater).Get_status(eventtime)
	statuss := make(map[string]interface{})
	for k, v := range sts {
		statuss[k] = v
	}
	val := false
	if self.Heater.(*Heater).Can_extrude {
		val = true
	} else {
		val = false
	}
	statuss["can_extrude"] = val
	statuss["Nozzle_diameter"] = self.Nozzle_diameter
	if self.Extruder_stepper != nil {
		for k, v := range self.Extruder_stepper.Get_status(eventtime) {
			statuss[k] = v
		}
	}
	return statuss
}
func (self *PrinterExtruder) Get_name() string {
	return self.Name
}
func (self *PrinterExtruder) Get_heater() interface{} {
	return self.Heater
}
func (self *PrinterExtruder) Get_trapq() interface{} {
	return self.Trapq
}
func (self *PrinterExtruder) Stats(eventtime float64) (bool, string) {
	return self.Heater.(*Heater).Stats(eventtime)
}
func (self *PrinterExtruder) Check_move(move *Move) error {
	axis_r := move.Axes_r[3]
	if !self.Heater.(*Heater).Can_extrude {
		return errors.New("Extrude below minimum temp\n" +
			"See the 'min_extrude_temp' config option for details")
	}
	if (move.Axes_d[0] == 0.0 && move.Axes_d[1] == 0.0) || axis_r < 0. {
		// Extrude only move (or retraction move) - limit accel and velocity
		if math.Abs(move.Axes_d[3]) > self.Max_e_dist {
			return errors.New(fmt.Sprintf(
				"Extrude only move too long (%.3fmm vs %.3fmm)\n"+
					"See the 'max_extrude_only_distance' config"+
					" option for details", move.Axes_d[3], self.Max_e_dist))
		}
		inv_extrude_r := 1. / math.Abs(axis_r)
		move.Limit_speed(self.Max_e_velocity*inv_extrude_r,
			self.Max_e_accel*inv_extrude_r)
	} else if axis_r > self.Max_extrude_ratio {
		if move.Axes_d[3] <= self.Nozzle_diameter*self.Max_extrude_ratio {
			// Permit extrusion if amount extruded is tiny
			return nil
		}
		area := axis_r * self.Filament_area
		logger.Debugf("Overextrude: %f vs %f (area=%.3f dist=%.3f)",
			axis_r, self.Max_extrude_ratio, area, move.Move_d)
		return errors.New(
			fmt.Sprintf("Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"+
				"See the 'max_extrude_cross_section' config option for details",
				area, self.Max_extrude_ratio*self.Filament_area))
	}
	return nil
}
func (self *PrinterExtruder) Calc_junction(prev_move, move *Move) float64 {
	diff_r := move.Axes_r[3] - prev_move.Axes_r[3]
	if diff_r != 0.0 {
		return math.Pow(self.Instant_corner_v/math.Abs(diff_r), 2)
	}
	return move.Max_cruise_v2
}

func (self *PrinterExtruder) Move(print_time float64, move *Move) {
	axis_r := move.Axes_r[3]
	accel := move.Accel * axis_r
	start_v := move.Start_v * axis_r
	cruise_v := move.Cruise_v * axis_r
	can_pressure_advance := 0.0
	if axis_r > 0. && (move.Axes_d[0] != 0.0 || move.Axes_d[1] != 0.0) {
		can_pressure_advance = 1.0
	}
	//# Queue movement (x is extruder movement, y is pressure advance flag)
	self.Trapq_append(self.Trapq, print_time,
		move.Accel_t, move.Cruise_t, move.Decel_t,
		move.Start_pos[3], 0., 0.,
		1., can_pressure_advance, 0.,
		start_v, cruise_v, accel)
	self.Last_position = move.End_pos[3]
}
func (self *PrinterExtruder) Find_past_position(print_time float64) float64 {
	if self.Extruder_stepper == nil {
		return 0.
	}
	return self.Extruder_stepper.Find_past_position(print_time)
}
func (self *PrinterExtruder) Cmd_M104(gcmd interface{}) error {
	// Set Extruder Temperature
	temp := gcmd.(*GCodeCommand).Get_float("S", 0., nil, nil, nil, nil)
	zero := 0
	index := gcmd.(*GCodeCommand).Get_int("T", nil, &zero, nil)
	var extruder IExtruder
	if index != 0 {
		section := "extruder"
		if index != 0 {
			section = fmt.Sprintf("extruder%d", index)
		}
		extruder1 := self.Printer.Lookup_object(section, value.None)
		if extruder1 == nil {
			if temp <= 0 {
				return nil
			}
			panic("Extruder not configured")
		}
		extruder = extruder1.(IExtruder)
	} else {
		extruder = MustLookupToolhead(self.Printer).Get_extruder()
	}
	pheaters := self.Printer.Lookup_object("heaters", object.Sentinel{})
	return pheaters.(*PrinterHeaters).Set_temperature(extruder.Get_heater().(*Heater), temp, false)
}
func (self *PrinterExtruder) Cmd_M109(gcmd interface{}) error {
	// Set Extruder Temperature
	temp := gcmd.(*GCodeCommand).Get_float("S", 0., nil, nil, nil, nil)
	zero := 0
	index := gcmd.(*GCodeCommand).Get_int("T", nil, &zero, nil)
	var extruder IExtruder
	if index != 0 {
		section := "extruder"
		if index != 0 {
			section = fmt.Sprintf("extruder%d", index)
		}
		extruder1 := self.Printer.Lookup_object(section, value.None)
		if extruder1 == nil {
			if temp <= 0 {
				return nil
			}
			panic("Extruder not configured")
		}
		extruder = extruder1.(IExtruder)
	} else {
		extruder = MustLookupToolhead(self.Printer).Get_extruder()
	}
	pheaters := self.Printer.Lookup_object("heaters", object.Sentinel{})
	return pheaters.(*PrinterHeaters).Set_temperature(extruder.Get_heater().(*Heater), temp, true)
}

const cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"

func (self *PrinterExtruder) Cmd_ACTIVATE_EXTRUDER(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	toolhead := MustLookupToolhead(self.Printer)
	if toolhead.Get_extruder() == self {
		gcmd.Respond_info(fmt.Sprintf("Extruder %s already active", self.Name), true)
		return nil
	}
	gcmd.Respond_info(fmt.Sprintf("Activating extruder %s", self.Name), true)
	toolhead.Flush_step_generation()
	toolhead.Set_extruder(self, self.Last_position)
	self.Printer.Send_event("extruder:activate_extruder", nil)
	return nil
}
func (self *PrinterExtruder) Get_extruder_stepper() *ExtruderStepper {
	return self.Extruder_stepper
}

// Dummy extruder class used when a printer has no extruder at all
type DummyExtruder struct {
	Printer *Printer
}

func NewDummyExtruder(printer *Printer) *DummyExtruder {
	self := &DummyExtruder{}
	self.Printer = printer
	return self
}
func (self *DummyExtruder) Update_move_time(flush_time float64, clear_history_time float64) {}
func (self *DummyExtruder) Check_move(move *Move) error {
	return move.Move_error("Extrude when no extruder present")
}
func (self *DummyExtruder) Find_past_position(print_time float64) float64 {
	return 0.
}
func (self *DummyExtruder) Calc_junction(prev_move, move *Move) float64 {
	return move.Max_cruise_v2
}
func (self *DummyExtruder) Get_name() string {
	return ""
}
func (self *DummyExtruder) Get_heater() interface{} {
	panic("Extruder not configured")
}
func (self *DummyExtruder) Get_trapq() interface{} {
	panic("Extruder not configured")
}
func (self *DummyExtruder) Get_extruder_stepper() *ExtruderStepper {
	panic("Extruder not configured")
	return nil
}

func Add_printer_objects_extruder(config *ConfigWrapper) {
	printer := config.Get_printer()
	for i := 0; i < 99; i++ {
		section := "extruder"
		if i > 0 {
			section = fmt.Sprintf("extruder%d", i)
		}
		if !config.Has_section(section) {
			break
		}
		pe := NewPrinterExtruder(config.Getsection(section), i)
		printer.Add_object(section, pe)
	}
}
