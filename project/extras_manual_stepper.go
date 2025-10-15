package project

import (
	"k3c/common/utils/object"
	"k3c/project/chelper"
	"strings"
)

type ManualStepper struct {
	printer       *Printer
	rail          *PrinterRail
	steppers      []*MCU_stepper
	velocity      float64
	accel         float64
	homing_accel  float64
	next_cmd_time float64

	trapq        interface{}
	trapq_append func(tq interface{}, print_time, accel_t, cruise_t, decel_t,
		start_pos_x, start_pos_y, start_pos_z, axes_r_x, axes_r_y,
		axes_r_z, start_v, cruise_v, accel float64)
	trapq_finalize_moves func(interface{}, float64, float64)
	can_home             bool
}

// NewManualStepper is the constructor for ManualStepper
func NewManualStepper(config *ConfigWrapper) *ManualStepper {
	self := new(ManualStepper)
	self.printer = config.Get_printer()

	self.can_home = false

	if config.Get("endstop_pin", "", true) != "" {
		self.can_home = true
		var default_position_endstop *float64
		self.rail = NewPrinterRail(config, false, default_position_endstop, false)
		self.steppers = self.rail.Get_steppers()
	} else {
		self.can_home = false
		stepper := PrinterStepper(config, false)
		self.steppers = []*MCU_stepper{stepper}
	}

	self.velocity = config.Getfloat("velocity", 5.0, 0, 0, 0, 0, true)
	self.accel = config.Getfloat("accel", 0.0, 0, 0, 0, 0, true)
	self.homing_accel = self.accel
	self.next_cmd_time = 0.0

	// Setup iterative solver
	self.trapq = chelper.Trapq_alloc()
	self.trapq_append = chelper.Trapq_append
	self.trapq_finalize_moves = chelper.Trapq_finalize_moves

	self.rail.Setup_itersolve("cartesian_stepper_alloc", uint8('x'))
	self.rail.Set_trapq(self.trapq)

	// Register commands
	stepper_name := strings.Split(config.Get_name(), " ")[1]
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Register_mux_command("MANUAL_STEPPER", "STEPPER",
		stepper_name, self.cmd_MANUAL_STEPPER, "Command a manually configured stepper")

	return self
}

func (self *ManualStepper) sync_print_time() {
	toolhead := MustLookupToolhead(self.printer)
	printTime := toolhead.Get_last_move_time()
	if self.next_cmd_time > printTime {
		toolhead.Dwell(self.next_cmd_time - printTime)
	} else {
		self.next_cmd_time = printTime
	}
}

func (self *ManualStepper) do_enable(enable bool) {
	self.sync_print_time()
	stepperEnable := self.printer.Lookup_object("stepper_enable", object.Sentinel{}).(*PrinterStepperEnable)
	if enable {
		for _, s := range self.steppers {
			se, _ := stepperEnable.Lookup_enable(s.Get_name(false))
			se.Motor_enable(self.next_cmd_time)
		}
	} else {
		for _, s := range self.steppers {
			se, _ := stepperEnable.Lookup_enable(s.Get_name(false))
			se.Motor_disable(self.next_cmd_time)
		}
	}
	self.sync_print_time()
}

func (self *ManualStepper) do_set_position(setpos float64) {
	self.rail.Set_position([]float64{setpos, 0.0, 0.0})
}

func (self *ManualStepper) do_move(movepos, speed, accel float64, sync bool) {
	self.sync_print_time()
	cp := self.rail.Get_commanded_position()
	dist := movepos - cp
	axisR, accel_t, cruise_t, cruiseV := Calc_move_time(&dist, speed, &accel)
	self.trapq_append(self.trapq, self.next_cmd_time, accel_t, cruise_t, accel_t,
		cp, 0, 0, axisR, 0, 0,
		0, cruiseV, accel)
	self.next_cmd_time += accel_t + cruise_t + accel_t
	self.rail.Generate_steps(self.next_cmd_time)
	self.trapq_finalize_moves(self.trapq, self.next_cmd_time+99999.9, self.next_cmd_time+99999.9)
	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	toolhead.Note_mcu_movequeue_activity(self.next_cmd_time, false)
	if sync {
		self.sync_print_time()
	}
}

func (self *ManualStepper) do_homing_move(movepos, speed, accel float64, triggered, check_trigger bool) error {
	if !self.can_home {
		panic("No endstop for this manual stepper")
	}
	self.homing_accel = accel
	pos := []float64{movepos, 0.0, 0.0, 0.0}
	endstops := self.rail.Get_endstops()
	phoming := self.printer.Lookup_object("homing", object.Sentinel{}).(*PrinterHoming)
	phoming.Manual_home(self, endstops, pos, speed, triggered, check_trigger)
	return nil
}

func (self *ManualStepper) cmd_MANUAL_STEPPER(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	if enable := gcmd.Get_int("ENABLE", -1, nil, nil); enable != -1 {
		self.do_enable(enable == 1)
	}

	if setpos := gcmd.Get_float("SET_POSITION", -1.0, nil, nil, nil, nil); setpos != -1.0 {
		self.do_set_position(setpos)
	}

	speed := gcmd.Get_float("SPEED", self.velocity, nil, nil, nil, nil)
	accel := gcmd.Get_float("ACCEL", self.accel, nil, nil, nil, nil)
	homing_move := gcmd.Get_int("STOP_ON_ENDSTOP", 0, nil, nil)
	if homing_move != 0 {
		movepos := gcmd.Get_float("MOVE", 0.0, nil, nil, nil, nil)
		self.do_homing_move(movepos, speed, accel, homing_move > 0, homing_move == 1)
	} else if movepos := gcmd.Get_float("MOVE", -1.0, nil, nil, nil, nil); movepos != -1.0 {
		sync := gcmd.Get_int("SYNC", 1, nil, nil)
		self.do_move(movepos, speed, accel, sync == 1)
	} else if gcmd.Get_int("SYNC", 0, nil, nil) == 1 {
		self.sync_print_time()
	}
	return nil
}

func (self *ManualStepper) Flush_step_generation() {
	self.sync_print_time()
}

func (self *ManualStepper) Get_position() []float64 {
	ret := []float64{self.rail.Get_commanded_position(), 0.0, 0.0, 0.0}
	return ret
}

func (self *ManualStepper) Set_position(newpos []float64, homingAxes []int) {
	self.do_set_position(newpos[0])
}

// overload IKinematics

func (self *ManualStepper) Check_move(move *Move) {

}

func (self *ManualStepper) Get_status(eventtime float64) map[string]interface{} {
	var ret = map[string]interface{}{}
	return ret
}

func (self *ManualStepper) Home(homing_state *Homing) {
}

func (self *ManualStepper) Note_z_not_homed() {
}

func (self *ManualStepper) Get_last_move_time() float64 {
	self.sync_print_time()
	return self.next_cmd_time
}

func ManualStepper_max(x, y float64) float64 {
	if x > y {
		return x
	}
	return y
}

func (self *ManualStepper) Dwell(delay float64) {
	self.next_cmd_time += ManualStepper_max(0.0, delay)
}

func (self *ManualStepper) Drip_move(newpos []float64, speed float64, drip_completion *ReactorCompletion) error {
	self.do_move(newpos[0], speed, self.homing_accel, true)
	return nil
}

func (self *ManualStepper) Get_kinematics() interface{} {
	return self
}

func (self *ManualStepper) Get_steppers() []interface{} {
	var steppers []interface{}

	for _, s := range self.steppers {
		steppers = append(steppers, s)
	}

	return steppers
}

func (self *ManualStepper) Calc_position(stepper_positions map[string]float64) []float64 {
	return []float64{stepper_positions[self.rail.Get_name(false)], 0.0, 0.0}
}

func Load_config_manual_stepper(config *ConfigWrapper) interface{} {
	return NewManualStepper(config)
}
