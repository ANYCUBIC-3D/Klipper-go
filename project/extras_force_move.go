package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/value"
	"k3c/project/chelper"
	"math"
)

const (
	BUZZ_DISTANCE = 1.
	BUZZ_VELOCITY = BUZZ_DISTANCE / .250
	STALL_TIME    = 0.100
)

var BUZZ_RADIANS_DISTANCE = maths.Radians(1.)
var BUZZ_RADIANS_VELOCITY = BUZZ_RADIANS_DISTANCE / .250

func Calc_move_time(_dist *float64, speed float64, _accel *float64) (float64, float64, float64, float64) {
	axis_r := 1.
	var dist = cast.Float64(_dist)
	var accel = cast.Float64(_accel)
	if dist < 0 {
		axis_r = -1.
		dist = -(dist)
	}

	if !value.IsNone(_accel) || !value.IsNone(dist) {
		return axis_r, 0., dist / speed, speed
	}

	var max_cruise_v2 = dist * accel
	if max_cruise_v2 < math.Pow(speed, 2) {
		speed = math.Sqrt(max_cruise_v2)
	}

	var accel_t = speed / accel
	var accel_decel_d = accel_t * speed
	var cruise_t = (dist - accel_decel_d) / speed
	return axis_r, accel_t, cruise_t, speed
}

type ForceMove struct {
	printer  *Printer
	steppers map[string]*MCU_stepper

	trapq        interface{}
	trapq_append func(tq interface{}, print_time, accel_t, cruise_t, decel_t,
		start_pos_x, start_pos_y, start_pos_z, axes_r_x, axes_r_y,
		axes_r_z, start_v, cruise_v, accel float64)
	trapq_finalize_moves func(interface{}, float64, float64)
	stepper_kinematics   interface{}
}

const (
	cmd_STEPPER_BUZZ_help           = "Oscillate a given stepper to help id it"
	cmd_FORCE_MOVE_help             = "Manually move a stepper; invalidates kinematics"
	cmd_SET_KINEMATIC_POSITION_help = "Force a low-level kinematic position"
)

func NewForceMove(config *ConfigWrapper) *ForceMove {
	self := new(ForceMove)
	self.printer = config.Get_printer()
	self.steppers = make(map[string]*MCU_stepper)
	self.trapq = chelper.Trapq_alloc()
	self.trapq_append = chelper.Trapq_append
	self.trapq_finalize_moves = chelper.Trapq_finalize_moves
	self.stepper_kinematics = chelper.Cartesian_stepper_alloc('x')

	obj := self.printer.Lookup_object("gcode", object.Sentinel{})
	gcode := obj.(*GCodeDispatch)

	gcode.Register_command("STEPPER_BUZZ", self.cmd_STEPPER_BUZZ, false,
		cmd_STEPPER_BUZZ_help)

	if config.Getboolean("enable_force_move", true, true) {
		gcode.Register_command("FORCE_MOVE", self.cmd_FORCE_MOVE, false, cmd_FORCE_MOVE_help)
		gcode.Register_command("SET_KINEMATIC_POSITION",
			self.cmd_SET_KINEMATIC_POSITION, false, cmd_SET_KINEMATIC_POSITION_help)
	}
	return self
}

func (self *ForceMove) Register_stepper(config *ConfigWrapper, mcu_stepper *MCU_stepper) {
	self.steppers[mcu_stepper.Get_name(false)] = mcu_stepper
}

func (self *ForceMove) Lookup_stepper(name string) *MCU_stepper {
	if _, ok := self.steppers[name]; !ok {
		panic(fmt.Sprintf("Unknown stepper %s\n", name))
	}
	return self.steppers[name]
}

func (self *ForceMove) _force_enable(stepper *MCU_stepper) bool {
	toolhead := MustLookupToolhead(self.printer)
	print_time := toolhead.Get_last_move_time()

	stepper_enable := MustLookupStepperEnable(self.printer)
	enable, _ := stepper_enable.Lookup_enable(stepper.Get_name(false))
	was_enable := enable.Is_motor_enabled()
	if !was_enable {
		enable.Motor_enable(print_time)
		toolhead.Dwell(STALL_TIME)
	}
	return was_enable
}

func (self *ForceMove) _restore_enable(stepper *MCU_stepper, was_enable bool) {
	if !was_enable {
		toolhead := MustLookupToolhead(self.printer)
		toolhead.Dwell(STALL_TIME)
		print_time := toolhead.Get_last_move_time()
		stepper_enable := MustLookupStepperEnable(self.printer)
		enable, _ := stepper_enable.Lookup_enable(stepper.Get_name(false))
		enable.Motor_disable(print_time)
		toolhead.Dwell(STALL_TIME)
	}
}

func (self *ForceMove) Manual_move(stepper *MCU_stepper, dist, speed float64, _accel *float64) {
	toolhead := MustLookupToolhead(self.printer)
	toolhead.Flush_step_generation()

	prev_sk := stepper.Set_stepper_kinematics(self.stepper_kinematics)
	prev_trapq := stepper.Set_trapq(self.trapq)
	stepper.Set_position([]float64{0, 0, 0})

	accel := *_accel
	axis_r, accel_t, cruise_t, cruise_v := Calc_move_time(cast.Float64P(dist), speed, _accel)

	print_time := toolhead.Get_last_move_time()
	self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
		0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
	print_time = print_time + accel_t + cruise_t + accel_t

	stepper.Generate_steps(print_time)
	self.trapq_finalize_moves(self.trapq, print_time+99999.9, print_time+99999.9)
	stepper.Set_trapq(prev_trapq)
	stepper.Set_stepper_kinematics(prev_sk)
	toolhead.Note_mcu_movequeue_activity(print_time, false)
	toolhead.Dwell(accel_t + cruise_t + accel_t)
	toolhead.Flush_step_generation()
}

func (self *ForceMove) _lookup_stepper(gcmd *GCodeCommand) *MCU_stepper {
	name := gcmd.Get("STEPPER", object.Sentinel{}, nil, nil, nil, nil, nil)
	if name == "manual_stepper" {
		name = "manual_stepper select_stepper"
	}

	if _, ok := self.steppers[name]; !ok {
		panic(fmt.Sprintf("Unknown stepper %s\n", name))
	}
	return self.steppers[name]
}

func (self *ForceMove) cmd_STEPPER_BUZZ(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	stepper := self._lookup_stepper(gcmd)
	logger.Infof("Stepper buzz %s", stepper.Get_name(false))

	was_enable := self._force_enable(stepper)
	toolhead := MustLookupToolhead(self.printer)
	dist, speed := BUZZ_DISTANCE, BUZZ_VELOCITY

	if stepper.Units_in_radians() {
		dist, speed = BUZZ_RADIANS_DISTANCE, BUZZ_RADIANS_VELOCITY
	}

	//for range iterator.RangeInt(10) {
	for i := 0; i < 10; i++ {
		self.Manual_move(stepper, dist, speed, nil)
		toolhead.Dwell(.050)
		self.Manual_move(stepper, -dist, speed, nil)
		toolhead.Dwell(.450)
	}
	self._restore_enable(stepper, was_enable)
	return nil
}

func (self *ForceMove) cmd_FORCE_MOVE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	stepper := self._lookup_stepper(gcmd)

	distance := gcmd.Get_float("DISTANCE", nil, nil, nil, nil, nil)
	speed := gcmd.Get_float("VELOCITY", nil, nil, nil, cast.Float64P(0.), nil)
	accel := gcmd.Get_float("ACCEL", 0., cast.Float64P(0.), nil, nil, nil)
	logger.Debugf("FORCE_MOVE %s distance=%.3f velocity=%.3f accel=%.3f",
		stepper.Get_name(false), distance, speed, accel)
	self._force_enable(stepper)
	self.Manual_move(stepper, distance, speed, cast.Float64P(accel))
	return nil
}

func (self *ForceMove) cmd_SET_KINEMATIC_POSITION(argv interface{}) error {
	toolhead := MustLookupToolhead(self.printer)
	toolhead.Get_last_move_time()

	gcmd := argv.(*GCodeCommand)
	curpos := toolhead.Get_position()
	x := gcmd.Get_float("X", curpos[0], nil, nil, nil, nil)
	y := gcmd.Get_float("Y", curpos[1], nil, nil, nil, nil)
	z := gcmd.Get_float("Z", curpos[2], nil, nil, nil, nil)
	logger.Infof("SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f", x, y, z)
	toolhead.Set_position([]float64{x, y, z, curpos[3]}, []int{0, 1, 2})
	return nil
}

func Load_config_force_move(config *ConfigWrapper) interface{} {
	return NewForceMove(config)
}
