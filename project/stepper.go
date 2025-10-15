package project

import "C"
import (
	"container/list"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/value"
	"k3c/project/chelper"

	//"runtime"

	"math"
	"reflect"
	"strconv"
	"strings"
)

const MIN_BOTH_EDGE_DURATION = 0.000000200

type MCU_stepper struct {
	_name                     string
	_rotation_dist            float64
	_steps_per_rotation       int
	_step_pulse_duration      interface{}
	_units_in_radians         bool
	_step_dist                float64
	_mcu                      *MCU
	_oid                      int
	_step_pin                 interface{}
	_invert_step              int
	_dir_pin                  interface{}
	_invert_dir               uint32
	_orig_invert_dir          uint32
	_step_both_edge           bool
	_req_step_both_edge       bool
	_mcu_position_offset      float64
	_reset_cmd_tag            interface{}
	_get_position_cmd         interface{}
	_active_callbacks         []interface{}
	_stepqueue                interface{}
	_stepper_kinematics       interface{}
	_itersolve_generate_steps func(interface{}, float64) int32
	_itersolve_check_active   func(interface{}, float64) float64
	_trapq                    interface{}
}

func NewMCU_stepper(name string, step_pin_params map[string]interface{}, dir_pin_params map[string]interface{}, rotation_dist float64, steps_per_rotation int,
	step_pulse_duration interface{}, _units_in_radians bool) *MCU_stepper {
	self := MCU_stepper{}
	self._name = name
	self._rotation_dist = rotation_dist
	self._steps_per_rotation = steps_per_rotation
	self._step_pulse_duration = step_pulse_duration
	self._units_in_radians = _units_in_radians
	self._step_dist = rotation_dist / float64(steps_per_rotation)
	self._mcu, _ = step_pin_params["chip"].(*MCU)
	self._oid = self._mcu.Create_oid()
	self._mcu.Register_config_callback(self._build_config)
	self._step_pin, _ = step_pin_params["pin"]
	invert_step, ok := step_pin_params["invert"]
	if ok {
		self._invert_step, _ = invert_step.(int)
	}
	if dir_pin_params["chip"].(*MCU) != self._mcu {
		panic("Stepper dir pin must be on same mcu as step pin")
	}
	self._dir_pin, _ = dir_pin_params["pin"]
	invert_dir, ok1 := dir_pin_params["invert"]
	if ok1 {
		self._invert_dir = uint32(invert_dir.(int))
		self._orig_invert_dir = self._invert_dir
	}
	self._step_both_edge = false
	self._req_step_both_edge = false
	self._mcu_position_offset = 0.
	self._reset_cmd_tag = nil
	self._get_position_cmd = nil
	self._active_callbacks = []interface{}{}
	self._stepqueue = chelper.Stepcompress_alloc(uint32(self._oid))
	//runtime.SetFinalizer(self,self._MCU_stepper)
	chelper.Stepcompress_set_invert_sdir(self._stepqueue, self._invert_dir)
	self._mcu.Register_stepqueue(self._stepqueue)
	self._stepper_kinematics = nil
	self._itersolve_generate_steps = chelper.Itersolve_generate_steps
	self._itersolve_check_active = chelper.Itersolve_check_active
	self._trapq = nil
	self._mcu.Get_printer().Register_event_handler("project:connect", self._query_mcu_position)
	return &self
}
func (self *MCU_stepper) _MCU_stepper() {
	chelper.Stepcompress_free(self._stepqueue)
	chelper.Free(self._stepper_kinematics)
}
func (self *MCU_stepper) Get_mcu() *MCU {
	return self._mcu
}

func (self *MCU_stepper) Get_name(short bool) string {
	if short && strings.HasPrefix(self._name, "stepper_") {
		return self._name[8:]
	}
	return self._name
}

func (self *MCU_stepper) Units_in_radians() bool {
	// Returns true if distances are in radians instead of millimeters
	return self._units_in_radians
}

func (self *MCU_stepper) Get_pulse_duration() (interface{}, bool) {
	return self._step_pulse_duration, self._req_step_both_edge
}

func (self *MCU_stepper) Setup_default_pulse_duration(pulseduration interface{}, step_both_edge bool) {
	if self._step_pulse_duration == nil {
		self._step_pulse_duration = pulseduration
	}
	self._req_step_both_edge = step_both_edge
}

func (self *MCU_stepper) Setup_itersolve(alloc_func string, params interface{}) {
	if alloc_func == "cartesian_stepper_alloc" {
		axis := params.([]interface{})[0].(uint8)
		sk := chelper.Cartesian_stepper_alloc(int8(axis))
		self.Set_stepper_kinematics(sk)
	} else if alloc_func == "corexy_stepper_alloc" {
		axis := params.([]interface{})[0].(uint8)
		sk := chelper.Corexy_stepper_alloc(int8(axis))
		self.Set_stepper_kinematics(sk)
	} else {
		logger.Debug("ERROR: Setup_itersolve not implement:", alloc_func)
	}
}

func (self *MCU_stepper) _build_config() {
	if self._step_pulse_duration == nil {
		self._step_pulse_duration = 0.000002
	}
	invertStep := self._invert_step
	sbe1, ok := self._mcu.Get_constants()["STEPPER_BOTH_EDGE"]
	if !ok {
		sbe1 = 0
	}
	sbe, _ := sbe1.(float64)
	if self._req_step_both_edge && sbe != 0 && cast.ToFloat64(self._step_pulse_duration) <= MIN_BOTH_EDGE_DURATION {
		self._step_both_edge = true
		self._step_pulse_duration = 0.
		invertStep = -1
	}
	stepPulseTicks := self._mcu.Seconds_to_clock(cast.ToFloat64(self._step_pulse_duration))
	self._mcu.Add_config_cmd(fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d", self._oid, self._step_pin, self._dir_pin,
		invertStep, stepPulseTicks), false, false)
	self._mcu.Add_config_cmd(fmt.Sprintf("reset_step_clock oid=%d clock=0", self._oid), false, true)
	stepCmdTag := self._mcu.Lookup_command_tag(
		"queue_step oid=%c interval=%u count=%hu add=%hi")
	dirCmdTag := self._mcu.Lookup_command_tag(
		"set_next_step_dir oid=%c dir=%c")
	self._reset_cmd_tag = self._mcu.Lookup_command_tag(
		"reset_step_clock oid=%c clock=%u")
	self._get_position_cmd = self._mcu.Lookup_query_command(
		"stepper_get_position oid=%c",
		"stepper_position oid=%c pos=%i", self._oid, nil, false)
	max_error := self._mcu.Get_max_stepper_error()
	maxErrorTicks := self._mcu.Seconds_to_clock(max_error)
	//ffiMain, ffiLib := chelper.get_ffi()
	chelper.Stepcompress_fill(self._stepqueue, uint32(maxErrorTicks),
		int32(stepCmdTag.(int)), int32(dirCmdTag.(int)))
}

func (self *MCU_stepper) Get_oid() int {
	return self._oid
}
func (self *MCU_stepper) Get_step_dist() float64 {
	return self._step_dist
}

func (self *MCU_stepper) Get_rotation_distance() (float64, int) {
	return self._rotation_dist, self._steps_per_rotation
}

func (self *MCU_stepper) Set_rotation_distance(rotation_dist float64) {
	mcu_pos := self.Get_mcu_position()
	self._rotation_dist = rotation_dist
	self._step_dist = rotation_dist / float64(self._steps_per_rotation)
	self.Set_stepper_kinematics(self._stepper_kinematics)
	self._set_mcu_position(mcu_pos)
}

func (self *MCU_stepper) Get_dir_inverted() (uint32, uint32) {
	return self._invert_dir, self._orig_invert_dir
}

func (self *MCU_stepper) Set_dir_inverted(invert_dir uint32) {
	if invert_dir == self._invert_dir {
		return
	}
	self._invert_dir = invert_dir
	//ffi_main, ffi_lib := chelper.Get_ffi()

	chelper.Stepcompress_set_invert_sdir(self._stepqueue, invert_dir)
	self._mcu.Get_printer().Send_event("stepper:set_dir_inverted", []interface{}{self})
}

func (self *MCU_stepper) Calc_position_from_coord(coord []float64) float64 {
	//ffi_main, ffi_lib := chelper.Get_ffi()
	return chelper.Itersolve_calc_position_from_coord(
		self._stepper_kinematics, coord[0], coord[1], coord[2])
}

func (self *MCU_stepper) Set_position(coord []float64) {
	mcu_pos := self.Get_mcu_position()
	sk := self._stepper_kinematics
	//ffi_main, ffi_lib := chelper.Get_ffi()
	chelper.Itersolve_set_position(sk, coord[0], coord[1], coord[2])
	self._set_mcu_position(mcu_pos)
}

func (self *MCU_stepper) Get_commanded_position() float64 {
	//ffi_main, ffi_lib = chelper.get_ffi()
	return chelper.Itersolve_get_commanded_pos(self._stepper_kinematics)
}

func (self *MCU_stepper) Get_mcu_position() int {
	mcu_pos_dist := self.Get_commanded_position() + self._mcu_position_offset
	mcu_pos := mcu_pos_dist / self._step_dist
	if mcu_pos >= 0 {
		return int(mcu_pos + 0.5)
	}
	return int(mcu_pos - 0.5)
}

func (self *MCU_stepper) _set_mcu_position(mcu_pos int) {
	mcu_pos_dist := float64(mcu_pos) * self._step_dist
	self._mcu_position_offset = mcu_pos_dist - self.Get_commanded_position()
}

func (self *MCU_stepper) Get_past_mcu_position(print_time float64) int {
	clock := self._mcu.Print_time_to_clock(print_time)
	pos := chelper.Stepcompress_find_past_position(self._stepqueue, uint64(clock))
	return int(pos)
}

func (self *MCU_stepper) Mcu_to_commanded_position(mcu_pos int) float64 {
	return float64(mcu_pos)*self._step_dist - self._mcu_position_offset
}

func (self *MCU_stepper) Dump_steps(count int, start_clock uint64, end_clock uint64) ([]interface{}, int) {
	//ffi_main, ffi_lib = chelper.get_ffi()
	data := chelper.New_pull_history_steps()
	_data := []interface{}{}
	for _, d := range data {
		_data = append(_data, d)
	}
	count = chelper.Stepcompress_extract_old(self._stepqueue, data, count,
		start_clock, end_clock)
	//return data, count
	return _data, count
}

func (self *MCU_stepper) Set_stepper_kinematics(sk interface{}) interface{} {
	old_sk := self._stepper_kinematics
	mcu_pos := 0
	if old_sk != nil {
		mcu_pos = self.Get_mcu_position()
	}
	self._stepper_kinematics = sk
	chelper.Itersolve_set_stepcompress(sk, self._stepqueue, self._step_dist)
	self.Set_trapq(self._trapq)
	self._set_mcu_position(mcu_pos)
	return old_sk
}

func (self *MCU_stepper) Note_homing_end() {
	ret := chelper.Stepcompress_reset(self._stepqueue, 0)
	if ret > 0 {
		panic("Internal error in stepcompress")
	}
	data := append([]uint32{uint32(self._reset_cmd_tag.(int)), uint32(self._oid), 0})

	ret = chelper.Stepcompress_queue_msg(self._stepqueue, data, len(data))
	if ret > 0 {
		panic("Internal error in stepcompress")
	}

	self._query_mcu_position(nil)
}

func (self *MCU_stepper) _query_mcu_position([]interface{}) error {
	if self._mcu.Is_fileoutput() {
		return nil
	}
	params := self._get_position_cmd.(*CommandQueryWrapper).Send([]int64{int64(self._oid)}, 0, 0).(map[string]interface{})
	_last_pos := params["pos"]
	last_pos := int(int32(_last_pos.(int64)))
	if self._invert_dir > 0 {
		last_pos = -last_pos
	}
	_receive_time := params["#receive_time"]
	receive_time := chelper.CdoubleTofloat64(_receive_time)
	print_time := self._mcu.Estimated_print_time(receive_time)
	clock := self._mcu.Print_time_to_clock(print_time)
	ret := chelper.Stepcompress_set_last_position(self._stepqueue, uint64(clock), int64(last_pos))
	if ret > 0 {
		panic("Internal error in stepcompress")
	}
	self._set_mcu_position(last_pos)
	self._mcu.Get_printer().Send_event("stepper:sync_mcu_position", []interface{}{self})
	return nil
}

func (self *MCU_stepper) Get_trapq() interface{} {
	return self._trapq
}

func (self *MCU_stepper) Set_trapq(tq interface{}) interface{} {
	//ffi_main, ffi_lib := chelper.GetFFI()
	if tq == nil {
		tq = nil
	}
	chelper.Itersolve_set_trapq(self._stepper_kinematics, tq)
	old_tq := self._trapq
	self._trapq = tq
	return old_tq
}

func (self *MCU_stepper) Add_active_callback(cb func(float64)) {
	self._active_callbacks = append(self._active_callbacks, cb)
}

func (self *MCU_stepper) Generate_steps(flush_time float64) {
	// Check for activity if necessary
	if len(self._active_callbacks) > 0 {
		sk := self._stepper_kinematics
		ret := self._itersolve_check_active(sk, flush_time)
		if ret > 0 {
			cbs := self._active_callbacks
			self._active_callbacks = []interface{}{}
			for _, cb := range cbs {
				cb.(func(float64))(ret)
			}
		}
	}
	// Generate steps
	sk := self._stepper_kinematics
	ret := self._itersolve_generate_steps(sk, flush_time)
	if ret > 0 {
		panic("Internal error in stepcompress")
	}
}

func (self *MCU_stepper) Is_active_axis(axis int8) int32 {
	return chelper.Itersolve_is_active_axis(self._stepper_kinematics, uint8(axis))
}

// PrinterStepper Helper code to build a stepper object from a config section
func PrinterStepper(config *ConfigWrapper, units_in_radians bool) *MCU_stepper {
	printer := config.Get_printer()
	name := config.Get_name()

	// Stepper definition
	ppins := printer.Lookup_object("pins", object.Sentinel{})
	stepPin := config.Get("step_pin", object.Sentinel{}, true).(string)
	stepPinParams := ppins.(*PrinterPins).Lookup_pin(stepPin, true, false, nil)
	dirPin := config.Get("dir_pin", object.Sentinel{}, true).(string)
	dirPinParams := ppins.(*PrinterPins).Lookup_pin(dirPin, true, false, nil)
	rotationDist, stepsPerRotation := Parse_step_distance(config, units_in_radians, true)
	stepPulseDuration := config.GetfloatNone("step_pulse_duration", nil, 0, 0, .001, 0, false)
	mcu_stepper := NewMCU_stepper(name, stepPinParams, dirPinParams, rotationDist, stepsPerRotation, stepPulseDuration, units_in_radians)

	// Register with helper modules
	for _, mname := range []string{"stepper_enable", "force_move", "motion_report"} {
		m := printer.Load_object(config, mname, object.Sentinel{})
		if m != nil {
			if mname == "stepper_enable" {
				m.(*PrinterStepperEnable).Register_stepper(config, mcu_stepper)
			} else if mname == "force_move" {
				m.(*ForceMove).Register_stepper(config, mcu_stepper)
			} else if mname == "motion_report" {
				m.(*PrinterMotionReport).Register_stepper(config, mcu_stepper)
			}
		}
	}
	return mcu_stepper
}

func parse_gear_ratio(config *ConfigWrapper, noteValid bool) float64 {
	gearRatio := config.Getlists("gear_ratio", []interface{}{}, []string{":", ","}, 2, reflect.Float64, noteValid)
	result := 1.
	for _, v := range gearRatio.([]interface{}) {
		g1, g2 := v.([]float64)[0], v.([]float64)[1]
		result *= g1 / g2
	}
	return result
}

func Parse_step_distance(config *ConfigWrapper, units_in_radians interface{}, note_valid bool) (float64, int) {
	if units_in_radians == nil {
		// Caller doesn't know if units are in radians - infer it
		rd := config.Get("rotation_distance", value.None, false)
		gr := config.Get("gear_ratio", value.None, false)
		units_in_radians = rd == nil && gr != nil
	}

	var rotation_dist float64
	if units_in_radians.(bool) {
		rotation_dist = 2. * math.Pi
		config.Get("gear_ratio", object.Sentinel{}, note_valid)
	} else {
		rotation_dist = config.Getfloat("rotation_distance", object.Sentinel{}, 0, 0, 0, 0, note_valid)
	}
	// Newer config format with rotation_distance
	microsteps := config.Getint("microsteps", 0, 1, 0, note_valid)
	full_steps := config.Getint("full_steps_per_rotation", 200, 1, 0, note_valid)
	if full_steps%4 != 0 {
		panic(fmt.Sprintf("full_steps_per_rotation invalid in section '%s'", config.Get_name()))
	}
	gearing := parse_gear_ratio(config, note_valid)
	return rotation_dist, int(float64(full_steps) * float64(microsteps) * gearing)
}

/////////////////////////////////////////////////////////////////////
// Stepper controlled rails
/////////////////////////////////////////////////////////////////////

// A motor control "rail" with one (or more) steppers and one (or more)
// endstops.
type PrinterRail struct {
	stepper_units_in_radians bool
	steppers                 []*MCU_stepper
	endstops                 []list.List
	endstop_map              map[string]interface{}
	Get_name                 func(bool) string
	Get_commanded_position   func() float64
	calc_position_from_coord func(coord []float64) float64
	Position_endstop         float64
	position_min             float64
	position_max             float64
	homing_speed             float64
	second_homing_speed      float64
	homing_retract_speed     float64
	homing_retract_dist      float64
	homing_positive_dir      bool
}

func NewPrinterRail(config *ConfigWrapper, need_position_minmax bool,
	default_position_endstop interface{}, units_in_radians bool) *PrinterRail {
	self := PrinterRail{}
	self.stepper_units_in_radians = units_in_radians
	self.steppers = []*MCU_stepper{}
	self.endstops = []list.List{}
	self.endstop_map = map[string]interface{}{}
	self.Add_extra_stepper(config)
	mcu_stepper := self.steppers[0]
	self.Get_name = mcu_stepper.Get_name
	self.Get_commanded_position = mcu_stepper.Get_commanded_position
	self.calc_position_from_coord = mcu_stepper.Calc_position_from_coord
	mcu_endstop := self.endstops[0].Front().Value
	// Get_position_endstop
	ok := false
	if mcu_endstop != nil {
		_, ok = reflect.TypeOf(mcu_endstop).MethodByName("Get_position_endstop")
	}
	if ok {
		self.Position_endstop = mcu_endstop.(*ProbeEndstopWrapper).Get_position_endstop()

	} else if default_position_endstop.(*float64) == nil {
		self.Position_endstop = config.Getfloat("position_endstop", object.Sentinel{}, 0, 0, 0, 0, false)
	} else {
		self.Position_endstop = config.Getfloat("position_endstop", default_position_endstop, 0, 0, 0, 0, false)
	}
	if need_position_minmax {
		self.position_min = config.Getfloat("position_min", 0., 0, 0, 0, 0, false)
		self.position_max = config.Getfloat("position_max", object.Sentinel{}, 0, 0, self.position_min, 0, false)
	} else {
		self.position_min = 0.
		self.position_max = self.Position_endstop
	}
	if self.Position_endstop < self.position_min || self.Position_endstop > self.position_max {
		panic(fmt.Errorf("Position_endstop '%f' in section '%s' must be between position_min and position_max", self.Position_endstop, config.Get_name()))
	}
	self.homing_speed = config.Getfloat("homing_speed", 5.0, 0, 0, 0., 0, false)
	self.second_homing_speed = config.Getfloat("second_homing_speed", self.homing_speed/2., 0, 0, 0., 0, false)
	self.homing_retract_speed = config.Getfloat("homing_retract_speed", self.homing_speed, 0, 0, 0., 0, false)
	self.homing_retract_dist = config.Getfloat("homing_retract_dist", 5., 0, 0, 0., 0, false)
	self.homing_positive_dir = config.Getboolean("homing_positive_dir", false, false)
	if self.homing_positive_dir == false {
		axisLen := self.position_max - self.position_min
		if self.Position_endstop <= self.position_min+axisLen/4. {
			self.homing_positive_dir = false
		} else if self.Position_endstop >= self.position_max-axisLen/4. {
			self.homing_positive_dir = true
		} else {
			panic(fmt.Errorf("Unable to infer homing_positive_dir in section '%s'", config.Get_name()))
		}
		config.Getboolean("homing_positive_dir", self.homing_positive_dir, false)
	} else if (self.homing_positive_dir && self.Position_endstop == self.position_min) || (!self.homing_positive_dir && self.Position_endstop == self.position_max) {
		panic(fmt.Errorf("Invalid homing_positive_dir / Position_endstop in '%s'", config.Get_name()))
	}
	return &self
}

func (self *PrinterRail) Get_range() (float64, float64) {
	return self.position_min, self.position_max
}

type HomingInfo struct {
	Speed               float64
	Position_endstop    float64
	Retract_speed       float64
	Retract_dist        float64
	Positive_dir        bool
	Second_homing_speed float64
}

func (self *PrinterRail) Get_homing_info() *HomingInfo {
	h := &HomingInfo{
		Speed:               self.homing_speed,
		Position_endstop:    self.Position_endstop,
		Retract_speed:       self.homing_retract_speed,
		Retract_dist:        self.homing_retract_dist,
		Positive_dir:        self.homing_positive_dir,
		Second_homing_speed: self.second_homing_speed,
	}
	return h
}

func (self *PrinterRail) Get_steppers() []*MCU_stepper {
	steppersBack := make([]*MCU_stepper, len(self.steppers))
	copy(steppersBack, self.steppers)
	return steppersBack
}
func (self *PrinterRail) Get_endstops() []list.List {
	endstopsBack := make([]list.List, len(self.endstops))
	copy(endstopsBack, self.endstops)
	return endstopsBack
}
func (self *PrinterRail) Add_extra_stepper(config *ConfigWrapper) {
	stepper := PrinterStepper(config, self.stepper_units_in_radians)
	self.steppers = append(self.steppers, stepper)
	if len(self.endstops) > 0 && config.Get("endstop_pin", value.None, false) == nil {
		self.endstops[0].Front().Value.(*MCU_endstop).Add_stepper(stepper)
		return
	}
	endstopPin := config.Get("endstop_pin", object.Sentinel{}, false)
	printer := config.Get_printer()
	pins := printer.Lookup_object("pins", object.Sentinel{})
	pinParams := pins.(*PrinterPins).Parse_pin(endstopPin.(string), true, true)
	// Normalize pin name
	pinName := fmt.Sprintf("%s:%s", pinParams["chip_name"], pinParams["pin"])
	// Look for already-registered endstop
	endstop := self.endstop_map[pinName]
	var mcuEndstop interface{}
	if endstop == nil {
		mcuEndstop = pins.(*PrinterPins).Setup_pin("endstop", endstopPin.(string))
		self.endstop_map[pinName] = map[string]interface{}{
			"endstop": mcuEndstop,
			"invert":  pinParams["invert"],
			"pullup":  pinParams["pullup"],
		}
		name := stepper.Get_name(true)
		var list list.List
		list.PushBack(mcuEndstop)
		list.PushBack(name)
		self.endstops = append(self.endstops, list)
		queryEndstops := printer.Load_object(config, "query_endstops", object.Sentinel{}).(*QueryEndstops)
		queryEndstops.Register_endstop(mcuEndstop, name)
	} else {
		mcuEndstop = endstop.(map[string]interface{})["endstop"]
		changedInvert := pinParams["invert"] != endstop.(map[string]interface{})["invert"]
		changedPullup := pinParams["pullup"] != endstop.(map[string]interface{})["pullup"]
		if changedInvert || changedPullup {
			panic(fmt.Errorf("pinter rail %s shared endstop pin %s must specify the same pullup/invert settings", self.Get_name(false), pinName))
		}
	}
	if _, ok := mcuEndstop.(*MCU_endstop); ok {
		mcuEndstop.(*MCU_endstop).Add_stepper(stepper)
	}
	if _, ok := mcuEndstop.(*ProbeEndstopWrapper); ok {
		mcuEndstop.(*ProbeEndstopWrapper).Add_stepper(stepper)
	}
}

func (self *PrinterRail) Setup_itersolve(alloc_func string, params ...interface{}) {
	for _, stepper := range self.steppers {
		stepper.Setup_itersolve(alloc_func, params)
	}
}

func (self *PrinterRail) Generate_steps(flush_time float64) {
	for _, stepper := range self.steppers {
		stepper.Generate_steps(flush_time)
	}
}

func (self *PrinterRail) Set_trapq(trapq interface{}) {
	for _, stepper := range self.steppers {
		stepper.Set_trapq(trapq)
	}
}

func (self *PrinterRail) Set_position(coord []float64) {
	for _, stepper := range self.steppers {
		stepper.Set_position(coord)
	}
}

func LookupMultiRail(config *ConfigWrapper, need_position_minmax bool, default_position_endstop interface{}, units_in_radians bool) *PrinterRail {
	dpe, ok := default_position_endstop.(*float64)
	if !ok {
		dpe = nil
	}
	rail := NewPrinterRail(config, need_position_minmax, dpe, units_in_radians)
	for i := 1; i < 99; i++ {
		if !config.Has_section(config.Get_name() + strconv.Itoa(i)) {
			break
		}
		rail.Add_extra_stepper(config.Getsection(config.Get_name() + strconv.Itoa(i)))
	}
	return rail
}
