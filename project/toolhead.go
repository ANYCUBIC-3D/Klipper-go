/*
// Code for coordinating events on the printer Toolhead
//
// Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import "C"
import (
	"errors"
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/object"
	"k3c/project/chelper"
	"math"
	"strings"
)

/*
// Common suffixes: _d is distance (in mm), _v is velocity (in
//   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
//   seconds), _r is ratio (scalar between 0.0 and 1.0)
*/

type Move struct {
	Toolhead           *Toolhead
	Start_pos          []float64
	End_pos            []float64
	Accel              float64
	Junction_deviation float64
	Timing_callbacks   []func(float64)
	Is_kinematic_move  bool
	Axes_d             []float64
	Move_d             float64
	Axes_r             []float64
	Min_move_t         float64
	Max_start_v2       float64
	Max_cruise_v2      float64
	Delta_v2           float64
	Max_smoothed_v2    float64
	Smooth_delta_v2    float64
	Start_v            float64
	Cruise_v           float64
	End_v              float64
	Accel_t            float64
	Cruise_t           float64
	Decel_t            float64
}

func NewMove(toolhead *Toolhead, start_pos, end_pos []float64, speed float64) *Move {
	self := &Move{}
	self.Toolhead = toolhead
	self.Start_pos = append([]float64{}, start_pos...)
	self.End_pos = append([]float64{}, end_pos...)
	self.Accel = toolhead.Max_accel
	self.Junction_deviation = toolhead.Junction_deviation
	self.Timing_callbacks = []func(float64){}
	velocity := math.Min(speed, toolhead.Max_velocity)
	self.Is_kinematic_move = true
	axes_d := []float64{
		end_pos[0] - start_pos[0],
		end_pos[1] - start_pos[1],
		end_pos[2] - start_pos[2],
		end_pos[3] - start_pos[3],
	}
	self.Axes_d = axes_d
	Move_d := math.Sqrt(axes_d[0]*axes_d[0] + axes_d[1]*axes_d[1] + axes_d[2]*axes_d[2])
	self.Move_d = Move_d
	inv_move_d := 0.
	if Move_d < 0.000000001 {
		// Extrude-only move
		self.End_pos = []float64{start_pos[0], start_pos[1], start_pos[2], end_pos[3]}
		axes_d[0], axes_d[1], axes_d[2] = 0., 0., 0.
		move_d := math.Abs(axes_d[3])
		self.Move_d = move_d
		inv_move_d = 0.
		if move_d != 0. {
			inv_move_d = 1. / move_d
		}
		self.Accel = 99999999.9
		velocity = speed
		self.Is_kinematic_move = false
	} else {
		inv_move_d = 1. / self.Move_d
	}
	self.Axes_r = []float64{
		axes_d[0] * inv_move_d,
		axes_d[1] * inv_move_d,
		axes_d[2] * inv_move_d,
		axes_d[3] * inv_move_d,
	}
	self.Min_move_t = self.Move_d / velocity
	// Junction speeds are tracked in velocity squared.  The deltaV2 is
	// the maximum amount of this squared-velocity that can change in
	// this move.
	self.Max_start_v2 = 0.
	self.Max_cruise_v2 = velocity * velocity
	self.Delta_v2 = 2. * self.Move_d * self.Accel
	self.Max_smoothed_v2 = 0.
	self.Smooth_delta_v2 = 2. * self.Move_d * toolhead.Max_accel_to_decel
	return self
}
func (self *Move) Limit_speed(speed float64, accel float64) {
	speed2 := speed * speed
	if speed2 < self.Max_cruise_v2 {
		self.Max_cruise_v2 = speed2
		self.Min_move_t = self.Move_d / speed
	}
	self.Accel = math.Min(self.Accel, accel)
	self.Delta_v2 = 2.0 * self.Move_d * self.Accel
	self.Smooth_delta_v2 = math.Min(self.Smooth_delta_v2, self.Delta_v2)
}

func (self *Move) Move_error(msg string) error {
	ep := self.End_pos
	m := fmt.Sprintf("%s: %.3f %.3f %.3f [%.3f]", msg, ep[0], ep[1], ep[2], ep[3])
	return errors.New(m)
}

func CustomMin(values ...float64) float64 {
	if len(values) == 0 {
		return 0
	}
	minValue := values[0]
	for _, v := range values[1:] {
		if v < minValue {
			minValue = v
		}
	}
	return minValue
}

func (self *Move) Calc_junction(prev_move *Move) {
	if !self.Is_kinematic_move || !prev_move.Is_kinematic_move {
		return
	}
	// Allow extruder to calculate its maximum junction
	extruder_v2 := self.Toolhead.Extruder.Calc_junction(prev_move, self)
	max_start_v2 := CustomMin(extruder_v2, self.Max_cruise_v2, prev_move.Max_cruise_v2, prev_move.Max_start_v2+prev_move.Delta_v2)
	// Find max velocity using "approximated centripetal velocity"
	axes_r := self.Axes_r
	prev_axes_r := prev_move.Axes_r
	junction_cos_theta := -(axes_r[0]*prev_axes_r[0] + axes_r[1]*prev_axes_r[1] + axes_r[2]*prev_axes_r[2])
	sin_theta_d2 := math.Sqrt(math.Max(0.5*(1.0-junction_cos_theta), 0.))
	cos_theta_d2 := math.Sqrt(math.Max(0.5*(1.0+junction_cos_theta), 0.))
	one_minus_sin_theta_d2 := 1. - sin_theta_d2
	if one_minus_sin_theta_d2 > 0. && cos_theta_d2 > 0. {
		R_jd := sin_theta_d2 / one_minus_sin_theta_d2
		move_jd_v2 := R_jd * self.Junction_deviation * self.Accel
		pmove_jd_v2 := R_jd * prev_move.Junction_deviation * prev_move.Accel
		// Approximated circle must contact moves no further than mid-move
		//   centripetal_v2 = .5 * self.move_d * self.accel * tan_theta_d2
		quarter_tan_theta_d2 := .25 * sin_theta_d2 / cos_theta_d2
		move_centripetal_v2 := self.Delta_v2 * quarter_tan_theta_d2
		pmove_centripetal_v2 := prev_move.Delta_v2 * quarter_tan_theta_d2
		max_start_v2 = CustomMin(max_start_v2, move_jd_v2, pmove_jd_v2,
			move_centripetal_v2, pmove_centripetal_v2)
	}
	// Apply limits
	self.Max_start_v2 = max_start_v2
	self.Max_smoothed_v2 = math.Min(max_start_v2, prev_move.Max_smoothed_v2+prev_move.Smooth_delta_v2)
}

func (self *Move) Set_junction(start_v2, cruise_v2, end_v2 float64) {
	// Determine accel, cruise, and decel portions of the move distance
	half_inv_accel := 0.5 / self.Accel
	accel_d := (cruise_v2 - start_v2) * half_inv_accel
	decel_d := (cruise_v2 - end_v2) * half_inv_accel
	cruise_d := self.Move_d - accel_d - decel_d
	// Determine move velocities
	start_v := math.Sqrt(start_v2)
	self.Start_v = start_v
	cruise_v := math.Sqrt(cruise_v2)
	self.Cruise_v = cruise_v
	end_v := math.Sqrt(end_v2)
	self.End_v = end_v
	// Determine time spent in each portion of move (time is the
	// distance divided by average velocity)
	self.Accel_t = accel_d / ((start_v + cruise_v) * 0.5)
	self.Cruise_t = cruise_d / cruise_v
	self.Decel_t = decel_d / ((end_v + cruise_v) * 0.5)
}

const LOOKAHEAD_FLUSH_TIME = 0.250

/*
// Class to track a list of pending move requests and to facilitate
// "look-ahead" across moves to reduce acceleration between moveself.
*/
type LookAheadQueue struct {
	Toolhead       *Toolhead
	queue          []*Move
	Junction_flush float64
}

func NewMoveQueue(Toolhead *Toolhead) *LookAheadQueue {
	return &LookAheadQueue{
		Toolhead:       Toolhead,
		queue:          []*Move{},
		Junction_flush: LOOKAHEAD_FLUSH_TIME,
	}
}

func (self *LookAheadQueue) Reset() {
	self.queue = []*Move{}
	self.Junction_flush = LOOKAHEAD_FLUSH_TIME
}

func (self *LookAheadQueue) Set_flush_time(flush_time float64) {
	self.Junction_flush = flush_time
}

func (self *LookAheadQueue) Get_last() *Move {
	if len(self.queue) > 0 {
		return self.queue[len(self.queue)-1]
	}
	return nil
}

type delayed_node struct {
	Move  *Move
	Ms_v2 float64
	Me_v2 float64
}

// lazy=False
func (self *LookAheadQueue) Flush(lazy bool) {
	self.Junction_flush = LOOKAHEAD_FLUSH_TIME
	update_flush_count := lazy
	queue := self.queue
	flush_count := len(queue)
	// Traverse queue from last to first move and determine maximum
	// junction speed assuming the robot comes to a complete stop
	// after the last move.
	delayed := []delayed_node{}
	next_end_v2, next_smoothed_v2, peak_cruise_v2 := 0., 0., 0.
	for i := flush_count - 1; i >= 0; i-- {
		move := queue[i]
		reachable_start_v2 := next_end_v2 + move.Delta_v2
		reachable_smoothed_v2 := next_smoothed_v2 + move.Smooth_delta_v2
		start_v2 := math.Min(move.Max_start_v2, reachable_start_v2)
		smoothed_v2 := math.Min(move.Max_smoothed_v2, reachable_smoothed_v2)
		if smoothed_v2 < reachable_smoothed_v2 {
			// It"s possible for this move to accelerate
			if smoothed_v2+move.Smooth_delta_v2 > next_smoothed_v2 || len(delayed) > 0 {
				// This move can decelerate or this is a full accel
				// move after a full decel move
				if update_flush_count && peak_cruise_v2 != 0 {
					flush_count = i
					update_flush_count = false
				}
				// this
				peak_cruise_v2 = math.Min(move.Max_cruise_v2, (smoothed_v2+reachable_smoothed_v2)*.5)
				if len(delayed) > 0 {
					// Propagate peak_cruise_v2 to any delayed moves
					if !update_flush_count && i < flush_count {
						mc_v2 := peak_cruise_v2
						for j := len(delayed) - 1; j >= 0; j-- {
							m := delayed[j].Move
							ms_v2 := delayed[j].Ms_v2
							me_v2 := delayed[j].Me_v2
							mc_v2 = math.Min(mc_v2, ms_v2)
							m.Set_junction(math.Min(ms_v2, mc_v2), mc_v2, math.Min(me_v2, mc_v2))
						}
					}
					delayed = nil
				}
			}
			if !update_flush_count && i < flush_count {
				cruise_v2 := math.Min((start_v2+reachable_start_v2)*.5,
					math.Min(move.Max_cruise_v2, peak_cruise_v2))
				move.Set_junction(math.Min(start_v2, cruise_v2), cruise_v2,
					math.Min(next_end_v2, cruise_v2))
			}
		} else {
			// Delay calculating this move until peak_cruise_v2 is known
			delayed = append(delayed, delayed_node{move, start_v2, next_end_v2})
		}
		next_end_v2 = start_v2
		next_smoothed_v2 = smoothed_v2
	}
	if update_flush_count || flush_count <= 0 {
		return
	}
	// Generate step times for all moves ready to be flushed
	self.Toolhead.Process_moves(queue[:flush_count])
	// Remove processed moves from the queue
	if flush_count < len(self.queue) && len(self.queue) > 0 {
		self.queue = self.queue[flush_count:]
	} else if len(self.queue) > 0 {
		self.queue = []*Move{}
	}
}
func (self *LookAheadQueue) Add_move(move *Move) {
	self.queue = append(self.queue, move)
	if len(self.queue) == 1 {
		return
	}
	move.Calc_junction(self.queue[len(self.queue)-2])
	self.Junction_flush -= move.Min_move_t
	if self.Junction_flush <= 0. {
		// Enough moves have been queued to reach the target flush time.
		self.Flush(true)
	}
}

const (
	BUFFER_TIME_LOW         = 1.0
	BUFFER_TIME_HIGH        = 2.0
	BUFFER_TIME_START       = 0.250
	BGFLUSH_LOW_TIME        = 0.200
	BGFLUSH_BATCH_TIME      = 0.200
	BGFLUSH_EXTRA_TIME      = 0.250
	MIN_KIN_TIME            = 0.100
	MOVE_BATCH_TIME         = 0.500
	STEPCOMPRESS_FLUSH_TIME = 0.050
	SDS_CHECK_TIME          = 0.001 // step+dir+step filter in stepcompresself.c
	MOVE_HISTORY_EXPIRE     = 30.
	DRIP_SEGMENT_TIME       = 0.050
	DRIP_TIME               = 0.100
)

type DripModeEndSignal struct {
}

func (d *DripModeEndSignal) Error() string {
	panic("implement me")
}

type IToolhead interface {
	Get_position() []float64
	Get_kinematics() interface{}
	Flush_step_generation()
	Get_last_move_time() float64
	Dwell(delay float64)
	Drip_move(newpos []float64, speed float64, drip_completion *ReactorCompletion) error
	Set_position(newpos []float64, homingAxes []int)
}

// Main code to track events (and their timing) on the printer Toolhead
type Toolhead struct {
	Printer                  *Printer
	Reactor                  IReactor
	All_mcus                 []*MCU
	Mcu                      *MCU
	Can_pause                bool
	do_kick_flush_timer      bool
	lookahead                *LookAheadQueue
	Commanded_pos            []float64
	Max_velocity             float64
	Max_accel                float64
	Requested_accel_to_decel float64
	Max_accel_to_decel       float64
	Square_corner_velocity   float64
	Junction_deviation       float64
	Print_time               float64
	Check_stall_time         float64
	Special_queuing_state    string
	Need_check_pause         float64
	Flush_timer              *ReactorTimer
	Priming_timer            *ReactorTimer
	Print_stall              float64
	Drip_completion          *ReactorCompletion
	Kin_flush_delay          float64
	Kin_flush_times          []float64
	last_flush_time          float64
	min_restart_time         float64
	need_flush_time          float64
	step_gen_time            float64
	clear_history_time       float64
	Trapq                    interface{}
	Trapq_append             func(tq interface{}, print_time,
		accel_t, cruise_t, decel_t,
		start_pos_x, start_pos_y, start_pos_z,
		axes_r_x, axes_r_y, axes_r_z,
		start_v, cruise_v, accel float64)
	Trapq_finalize_moves     func(interface{}, float64, float64)
	Step_generators          []func(float64 float64)
	Coord                    []string
	Extruder                 IExtruder
	Kin                      IKinematics
	VelocityRangeLimit       [][2]float64
	VelocityRangeLimitHitLog bool
	move_transform           Itransform
}

func NewToolhead(config *ConfigWrapper) *Toolhead {
	self := &Toolhead{}
	self.Printer = config.Get_printer()
	self.Reactor = self.Printer.Get_reactor()
	object_arr := self.Printer.Lookup_objects("mcu")
	self.All_mcus = []*MCU{}
	for _, m := range object_arr {
		for k1, m1 := range m.(map[string]interface{}) {
			if strings.HasPrefix(k1, "mcu") {
				self.All_mcus = append(self.All_mcus, m1.(*MCU))
			}
		}
	}
	self.Mcu = self.All_mcus[0]
	self.lookahead = NewMoveQueue(self)
	self.lookahead.Set_flush_time(BUFFER_TIME_HIGH)
	self.Commanded_pos = []float64{0., 0., 0., 0.}
	// Velocity and acceleration control
	self.Max_velocity = config.Getfloat("max_velocity", object.Sentinel{}, 0.0, 0.0, 0.0, 0.0, true)
	self.Max_accel = config.Getfloat("max_accel", object.Sentinel{}, 0.0, 0.0, 0.0, 0.0, true)
	self.Requested_accel_to_decel = config.Getfloat("max_accel_to_decel", self.Max_accel*0.5, 0.0, 0.0, 0., 0.0, true)
	self.Max_accel_to_decel = self.Requested_accel_to_decel
	self.Square_corner_velocity = config.Getfloat("square_corner_velocity", 5., 0.0, 0.0, 0., 0.0, true)
	self.Junction_deviation = 0.
	self.Calc_junction_deviation()
	// Input stall detection
	self.Check_stall_time = 0.
	self.Print_stall = 0
	// Input pause tracking
	self.Can_pause = true
	if self.Mcu.Is_fileoutput() {
		self.Can_pause = false
	}
	self.Need_check_pause = -1.
	// Print time tracking
	self.Print_time = 0.
	self.Special_queuing_state = "NeedPrime"
	self.Drip_completion = nil
	// Kinematic step generation scan window time tracking
	self.Kin_flush_delay = SDS_CHECK_TIME
	self.Kin_flush_times = []float64{}
	// Flush tracking
	self.Flush_timer = self.Reactor.Register_timer(self._flush_handler, constants.NEVER)
	self.do_kick_flush_timer = true
	self.last_flush_time, self.min_restart_time = 0., 0.
	self.need_flush_time, self.step_gen_time, self.clear_history_time = 0., 0., 0.
	// Setup iterative solver
	self.Trapq = chelper.Trapq_alloc()
	self.Trapq_append = chelper.Trapq_append
	self.Trapq_finalize_moves = chelper.Trapq_finalize_moves
	self.Step_generators = []func(float64 float64){}
	// Create kinematics class
	gcode_obj := self.Printer.Lookup_object("gcode", object.Sentinel{})

	gcode := gcode_obj.(*GCodeDispatch)
	self.Coord = append([]string{}, gcode.Coord...)
	self.Extruder = NewDummyExtruder(self.Printer)
	kin_name := config.Get("kinematics", object.Sentinel{}, true)
	kinematics := Load_kinematics(kin_name.(string))
	self.Kin = kinematics.(func(*Toolhead, *ConfigWrapper) interface{})(self, config).(IKinematics)
	// Register commands
	gcode.Register_command("G4", self.Cmd_G4, false, "")
	gcode.Register_command("M400", self.Cmd_M400, false, "")
	gcode.Register_command("SET_VELOCITY_LIMIT",
		self.Cmd_SET_VELOCITY_LIMIT, false,
		cmd_SET_VELOCITY_LIMIT_help)
	gcode.Register_command("M204", self.Cmd_M204, false, "")
	self.Printer.Register_event_handler("project:shutdown",
		self.Handle_shutdown)
	modules := []string{"gcode_move", "homing", "statistics", "idle_timeout",
		"manual_probe", "tuning_tower"}
	for _, module_name := range modules {
		self.Printer.Load_object(config, module_name, object.Sentinel{})
	}
	return self
}
func (self *Toolhead) _Toolhead() {
	chelper.Trapq_free(self.Trapq)
}

func (self *Toolhead) Get_transform() Itransform {
	return self.move_transform
}

// Print time and flush tracking
func (self *Toolhead) _advance_flush_time(flush_time float64) {
	flush_time = math.Max(flush_time, self.last_flush_time)
	//Generate steps via itersolve
	sg_flush_want := math.Min(flush_time+STEPCOMPRESS_FLUSH_TIME, self.Print_time-self.Kin_flush_delay)
	sg_flush_time := math.Max(sg_flush_want, flush_time)
	for _, sg := range self.Step_generators {
		sg(sg_flush_time)
	}
	self.min_restart_time = math.Max(self.min_restart_time, sg_flush_time)
	//Free trapq entries that are no longer needed
	clear_history_time := self.clear_history_time
	if !self.Can_pause {
		clear_history_time = flush_time - MOVE_HISTORY_EXPIRE
	}
	free_time := sg_flush_time - self.Kin_flush_delay
	self.Trapq_finalize_moves(self.Trapq, free_time, clear_history_time)
	self.Extruder.Update_move_time(free_time, clear_history_time)
	//Flush stepcompress and mcu steppersync
	for _, m := range self.All_mcus {
		m.Flush_moves(flush_time, clear_history_time)
	}
	self.last_flush_time = flush_time
}

func (self *Toolhead) _advance_move_time(next_print_time float64) {
	pt_delay := self.Kin_flush_delay + STEPCOMPRESS_FLUSH_TIME
	flush_time := math.Max(self.last_flush_time, self.Print_time-pt_delay)
	self.Print_time = math.Max(self.Print_time, next_print_time)
	want_flush_time := math.Max(flush_time, self.Print_time-pt_delay)
	for {
		flush_time = math.Min(flush_time+MOVE_BATCH_TIME, want_flush_time)
		self._advance_flush_time(flush_time)
		if flush_time >= want_flush_time {
			break
		}
	}
}

func (self *Toolhead) _calc_print_time() {
	curtime := self.Reactor.Monotonic()
	est_print_time := self.Mcu.Estimated_print_time(curtime)
	kin_time := math.Max(est_print_time+MIN_KIN_TIME, self.min_restart_time)
	kin_time += self.Kin_flush_delay
	min_print_time := math.Max(est_print_time+BUFFER_TIME_START, kin_time)
	if min_print_time > self.Print_time {
		self.Print_time = min_print_time
		self.Printer.Send_event("toolhead:sync_print_time", []interface{}{curtime, est_print_time, self.Print_time})
	}
}
func (self *Toolhead) Process_moves(moves []*Move) {
	// Resync print_time if necessary
	if len(self.Special_queuing_state) > 0 {
		if self.Special_queuing_state != "Drip" {
			// Transition from "NeedPrime"/"Priming" state to main state
			self.Special_queuing_state = ""
			self.Need_check_pause = -1.
		}
		self._calc_print_time()
	}

	// Queue moves into trapezoid motion queue (trapq)
	next_move_time := self.Print_time
	for _, move := range moves {
		if move.Is_kinematic_move {
			self.Trapq_append(self.Trapq, next_move_time, move.Accel_t, move.Cruise_t, move.Decel_t,
				move.Start_pos[0], move.Start_pos[1], move.Start_pos[2],
				move.Axes_r[0], move.Axes_r[1], move.Axes_r[2],
				move.Start_v, move.Cruise_v, move.Accel)
		}
		if move.Axes_d[3] != 0.0 {
			self.Extruder.(*PrinterExtruder).Move(next_move_time, move)
		}
		next_move_time = next_move_time + move.Accel_t + move.Cruise_t + move.Decel_t
		for _, cb := range move.Timing_callbacks {
			cb(next_move_time)
		}
	}

	// Generate steps for moves
	if self.Special_queuing_state != "" {
		self.Update_drip_move_time(next_move_time)
	}
	self.Note_mcu_movequeue_activity(next_move_time+self.Kin_flush_delay, true)
	self._advance_move_time(next_move_time)
}

func (self *Toolhead) _flush_lookahead() {
	// Transit from "NeedPrime"/"Priming"/"Drip"/main state to "NeedPrime"
	self.lookahead.Flush(false)
	self.Special_queuing_state = "NeedPrime"
	self.Need_check_pause = -1.
	self.lookahead.Set_flush_time(BUFFER_TIME_HIGH)
	self.Check_stall_time = 0.
}

func (self *Toolhead) Flush_step_generation() {
	self._flush_lookahead()
	self._advance_flush_time(self.step_gen_time)
	self.min_restart_time = math.Max(self.min_restart_time, self.Print_time)
}

func (self *Toolhead) Get_last_move_time() float64 {
	if self.Special_queuing_state != "" {
		self._flush_lookahead()
		self._calc_print_time()
	} else {
		self.lookahead.Flush(false)
	}
	return self.Print_time
}
func (self *Toolhead) _Check_pause() {
	eventtime := self.Reactor.Monotonic()
	est_print_time := self.Mcu.Estimated_print_time(eventtime)
	buffer_time := self.Print_time - est_print_time
	if self.Special_queuing_state != "" {
		if self.Check_stall_time > 0.0 {
			// Was in "NeedPrime" state and got there from idle input
			if est_print_time < self.Check_stall_time {
				self.Print_stall += 1
			}
			self.Check_stall_time = 0.
		}
		// Transition from "NeedPrime"/"Priming" state to "Priming" state
		self.Special_queuing_state = "Priming"
		self.Check_stall_time = -1.
		if self.Priming_timer == nil {
			self.Priming_timer = self.Reactor.Register_timer(self.Priming_handler, constants.NEVER)
		}
		wtime := eventtime + math.Max(0.100, buffer_time-BUFFER_TIME_LOW)
		self.Reactor.Update_timer(self.Priming_timer, wtime)
	}
	// Check if there are lots of queued moves and pause if so
	for {
		pause_time := buffer_time - BUFFER_TIME_HIGH
		if pause_time <= 0. {
			break
		}
		if !self.Can_pause {
			self.Need_check_pause = constants.NEVER
			return
		}
		eventtime = self.Reactor.Pause(eventtime + math.Min(1, pause_time))
		est_print_time = self.Mcu.Estimated_print_time(eventtime)
		buffer_time = self.Print_time - est_print_time
	}
	if self.Special_queuing_state == "" {
		// In main state - defer pause checking until needed
		self.Need_check_pause = est_print_time + BUFFER_TIME_HIGH + 0.100
	}
}

func (self *Toolhead) Priming_handler(eventtime float64) float64 {
	defer func() {
		if r := recover(); r != nil {
			logger.Error("Exception in priming_handler")
			self.Printer.Invoke_shutdown("Exception in priming_handler")
		}
	}()
	self.Reactor.Unregister_timer(self.Priming_timer)
	self.Priming_timer = nil
	if self.Special_queuing_state == "Priming" {
		self._flush_lookahead()
		self.Check_stall_time = self.Print_time
	}
	return constants.NEVER
}

func (self *Toolhead) _flush_handler(eventtime float64) float64 {
	defer func() {
		if err := recover(); err != nil {
			logger.Error("Exception in flush_handler")
			self.Printer.Invoke_shutdown("Exception in flush_handler")
		}
	}()
	est_print_time := self.Mcu.Estimated_print_time(eventtime)
	if self.Special_queuing_state == "" {
		// In "main" state - flush lookahead if buffer runs low
		print_time := self.Print_time
		buffer_time := print_time - est_print_time
		if buffer_time > BUFFER_TIME_LOW {
			// Running normally - reschedule check
			return eventtime + buffer_time - BUFFER_TIME_LOW
		}
		// Under ran low buffer mark - flush lookahead queue
		self._flush_lookahead()
		if print_time != self.Print_time {
			self.Check_stall_time = self.Print_time
		}
	}
	// In "NeedPrime"/"Priming" state - flush queues if needed
	for {
		end_flush := self.need_flush_time + BGFLUSH_EXTRA_TIME
		if self.last_flush_time >= end_flush {
			self.do_kick_flush_timer = true
			return constants.NEVER
		}
		buffer_time := self.last_flush_time - est_print_time
		if buffer_time > BGFLUSH_LOW_TIME {
			return eventtime + buffer_time - BGFLUSH_LOW_TIME
		}
		ftime := est_print_time + BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
		self._advance_flush_time(math.Min(end_flush, ftime))
	}
	return constants.NEVER
}

// Movement commands
func (self *Toolhead) Get_position() []float64 {
	commanded_pos_back := make([]float64, len(self.Commanded_pos))
	copy(commanded_pos_back, self.Commanded_pos)
	return commanded_pos_back
}
func (self *Toolhead) Set_position(newpos []float64, homingAxes []int) {
	self.Flush_step_generation()
	//ffiLib := chelper.Get_ffi()
	chelper.Trapq_set_position(self.Trapq, self.Print_time,
		newpos[0], newpos[1], newpos[2])
	self.Commanded_pos = append([]float64{}, newpos...)
	self.Kin.Set_position(newpos, homingAxes)
	self.Printer.Send_event("toolhead:set_position", nil)
}
func (self *Toolhead) Move(newpos []float64, speed float64) {
	move := NewMove(self, self.Commanded_pos, newpos, speed)
	if move.Move_d == 0.0 {
		return
	}
	if move.Is_kinematic_move {
		self.Kin.Check_move(move)
	}
	if move.Axes_d[3] != 0.0 {
		self.Extruder.Check_move(move)
	}
	self.Commanded_pos = move.End_pos
	self.lookahead.Add_move(move)
	if self.Print_time > self.Need_check_pause {
		self._Check_pause()
	}
}

func (self *Toolhead) Manual_move(coord []interface{}, speed float64) {
	length := int(math.Max(float64(len(self.Commanded_pos)), float64(len(coord))))
	curpos := make([]float64, length)
	copy(curpos, self.Commanded_pos)
	for i := 0; i < len(coord); i++ {
		if coord[i] != nil {
			curpos[i] = coord[i].(float64)
		}
	}
	self.Move(curpos, speed)
	self.Printer.Send_event("toolhead:manual_move", nil)
}

func (self *Toolhead) Dwell(delay float64) {
	next_print_time := self.Get_last_move_time() + math.Max(0., delay)
	self._advance_move_time(next_print_time)
	self._Check_pause()
}

func (self *Toolhead) Wait_moves() {
	self._flush_lookahead()
	eventtime := self.Reactor.Monotonic()
	for self.Special_queuing_state == "" || self.Print_time >= self.Mcu.Estimated_print_time(eventtime) {
		if !self.Can_pause {
			break
		}
		eventtime = self.Reactor.Pause(eventtime + 0.100)
	}
}

func (self *Toolhead) Set_extruder(extruder IExtruder, extrude_pos float64) {
	self.Extruder = extruder
	self.Commanded_pos[3] = extrude_pos
}

func (self *Toolhead) Get_extruder() IExtruder {
	return self.Extruder
}

// Homing "drip move" handling
func (self *Toolhead) Update_drip_move_time(next_print_time float64) error {
	flush_delay := DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + self.Kin_flush_delay
	for self.Print_time < next_print_time {
		if self.Drip_completion.Test() {
			panic(&DripModeEndSignal{})
		}
		curTime := self.Reactor.Monotonic()
		est_printTime := self.Mcu.Estimated_print_time(curTime)
		wait_time := self.Print_time - est_printTime - flush_delay
		if wait_time > 0. && self.Can_pause {
			// Pause before sending more steps
			self.Drip_completion.Wait(curTime+wait_time, nil)
			continue
		}
		npt := math.Min(self.Print_time+DRIP_SEGMENT_TIME, next_print_time)
		self.Note_mcu_movequeue_activity(npt+self.Kin_flush_delay, true)
		self._advance_move_time(npt)
	}
	return nil
}
func (self *Toolhead) Drip_move(newpos []float64, speed float64, drip_completion *ReactorCompletion) error {
	self.Dwell(self.Kin_flush_delay)
	// Transition from "NeedPrime"/"Priming"/main state to "Drip" state、
	self.lookahead.Flush(false)
	self.Special_queuing_state = "Drip"
	self.Need_check_pause = constants.NEVER
	self.Reactor.Update_timer(self.Flush_timer, constants.NEVER)
	self.do_kick_flush_timer = false
	self.lookahead.Set_flush_time(BUFFER_TIME_HIGH)
	self.Check_stall_time = 0.
	self.Drip_completion = drip_completion
	// Submit move
	self.tryCatchDrip_moveMove(newpos, speed)
	// Transmit move in "drip" mode
	self.tryCatchDrip_moveFlush(false)
	// Exit "Drip" state
	self.Reactor.Update_timer(self.Flush_timer, constants.NOW)
	self.Flush_step_generation()
	return nil
}
func (self *Toolhead) tryCatchDrip_moveMove(newpos []float64, speed float64) {
	defer func() {
		if err := recover(); err != nil {
			_, ok := err.(*CommandError)
			if ok {
				self.Reactor.Update_timer(self.Flush_timer, constants.NOW)
				self.Flush_step_generation()
			}
			panic(err)
		}
	}()
	self.Move(newpos, speed)
}
func (self *Toolhead) tryCatchDrip_moveFlush(lazy bool) {
	defer func() {
		if err := recover(); err != nil {
			_, ok := err.(*DripModeEndSignal)
			if ok {
				self.lookahead.Reset()
				self.Trapq_finalize_moves(self.Trapq, constants.NEVER, 0)
				return
			}
			panic(err)
		}
	}()
	self.lookahead.Flush(lazy)
}

// Misc commands
func (self *Toolhead) Stats(eventtime float64) (bool, string) {
	max_queue_time := math.Max(self.Print_time, self.last_flush_time)
	for _, m := range self.All_mcus {
		m.Check_active(max_queue_time, eventtime)
	}
	est_print_time := self.Mcu.Estimated_print_time(eventtime)
	self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
	buffer_time := self.Print_time - self.Mcu.Estimated_print_time(eventtime)
	is_active := buffer_time > -60.0 || self.Special_queuing_state == ""
	if self.Special_queuing_state == "Drip" {
		buffer_time = 0.
	}
	return is_active, fmt.Sprintf("print_time=%.3f buffer_time=%.3f print_stall=%.f",
		self.Print_time, math.Max(buffer_time, 0.), self.Print_stall)
}

func (self *Toolhead) Check_busy(eventtime float64) (float64, float64, bool) {
	est_print_time := self.Mcu.Estimated_print_time(eventtime)
	lookahead_empty := len(self.lookahead.queue) == 0
	return self.Print_time, est_print_time, lookahead_empty
}
func (self *Toolhead) Get_status(eventtime float64) map[string]interface{} {
	print_time := self.Print_time
	estimated_print_time := self.Mcu.Estimated_print_time(eventtime)
	res := self.Kin.Get_status(eventtime)
	res["print_time"] = print_time
	res["stalls"] = self.Print_stall
	res["estimated_print_time"] = estimated_print_time
	res["extruder"] = self.Extruder.Get_name()

	coord := make([]float64, len(self.Commanded_pos))
	copy(coord, self.Commanded_pos)
	res["position"] = coord
	res["max_velocity"] = self.Max_velocity
	res["max_accel"] = self.Max_accel
	res["max_accel_to_decel"] = self.Requested_accel_to_decel
	res["square_corner_velocity"] = self.Square_corner_velocity
	return res
}
func (self *Toolhead) Handle_shutdown([]interface{}) error {
	self.Can_pause = false
	self.lookahead.Reset()
	return nil
}
func (self *Toolhead) Get_kinematics() interface{} {
	return self.Kin
}
func (self *Toolhead) Get_trapq() interface{} {
	return self.Trapq
}
func (self *Toolhead) Register_step_generator(handler func(float64)) {
	self.Step_generators = append(self.Step_generators, handler)
}
func (self *Toolhead) Note_step_generation_scan_time(delay, old_delay float64) {
	self.Flush_step_generation()
	// cur_delay := self.Kin_flush_delay
	if old_delay != 0.0 {
		index := 0
		for i, v := range self.Kin_flush_times {
			if v == old_delay {
				index = i
				break
			}
		}
		self.Kin_flush_times = append(self.Kin_flush_times[:index], self.Kin_flush_times[index+1:]...)
	}
	if delay != 0.0 {
		self.Kin_flush_times = append(self.Kin_flush_times, delay)
	}
	new_delay := 0.
	for _, val := range self.Kin_flush_times {
		val += SDS_CHECK_TIME
		if val > new_delay {
			new_delay = val
		}
	}
	self.Kin_flush_delay = new_delay
}
func (self *Toolhead) Register_lookahead_callback(callback func(float64)) {
	last_move := self.lookahead.Get_last()
	if last_move == nil {
		callback(self.Get_last_move_time())
		return
	}
	last_move.Timing_callbacks = append(last_move.Timing_callbacks, callback)
}
func (self *Toolhead) Note_mcu_movequeue_activity(mq_time float64, set_step_gen_time bool) {
	self.need_flush_time = math.Max(self.need_flush_time, mq_time)
	if set_step_gen_time {
		self.step_gen_time = math.Max(self.step_gen_time, mq_time)
	}

	if self.do_kick_flush_timer {
		self.do_kick_flush_timer = false
		self.Reactor.Update_timer(self.Flush_timer, constants.NOW)
	}
}
func (self *Toolhead) Get_max_velocity() (float64, float64) {
	return self.Max_velocity, self.Max_accel
}
func (self *Toolhead) Calc_junction_deviation() {
	scv2 := self.Square_corner_velocity * self.Square_corner_velocity
	self.Junction_deviation = scv2 * (math.Sqrt(2.) - 1.) / self.Max_accel
	self.Max_accel_to_decel = math.Min(self.Requested_accel_to_decel,
		self.Max_accel)
}
func (self *Toolhead) Cmd_G4(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	minval := 0.
	delay := 0.0
	delay = gcmd.Get_float("P", 0., &minval, nil, nil, nil) / 1000.
	self.Dwell(delay)
	return nil
}
func (self *Toolhead) Cmd_M400(gcmd interface{}) error {
	// Wait for current moves to finish
	self.Wait_moves()
	return nil
}

const cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

func (self *Toolhead) Cmd_SET_VELOCITY_LIMIT(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	above := 0.0
	minval := 0.
	max_velocity := gcmd.Get_float("VELOCITY", nil, nil, nil, &above, nil)
	max_accel := gcmd.Get_float("ACCEL", nil, nil, nil, &above, nil)
	square_corner_velocity := gcmd.Get_float("SQUARE_CORNER_VELOCITY", nil, &minval, nil, nil, nil)
	requested_accel_to_decel := gcmd.Get_float("ACCEL_TO_DECEL", nil, nil, nil, &above, nil)

	if max_velocity != 0.0 {
		if self.Max_velocity != max_velocity {
			self.Max_velocity = max_velocity
		}

	}
	if max_accel != 0.0 {
		if self.Max_accel != max_accel {
			self.Max_accel = max_accel
		}
	}
	if square_corner_velocity != 0.0 {
		if self.Square_corner_velocity != square_corner_velocity {
			self.Square_corner_velocity = square_corner_velocity
		}

	}
	if requested_accel_to_decel != 0.0 {
		if self.Requested_accel_to_decel != requested_accel_to_decel {
			self.Requested_accel_to_decel = requested_accel_to_decel
		}
	}

	self.Calc_junction_deviation()
	msg := fmt.Sprintf("max_velocity: %.6f\n"+
		"max_accel: %.6f\n"+
		"max_accel_to_decel: %.6f\n"+
		"square_corner_velocity: %.6f",
		self.Max_velocity, self.Max_accel, self.Requested_accel_to_decel, self.Square_corner_velocity)
	self.Printer.Set_rollover_info("toolhead", fmt.Sprintf("toolhead: %s", msg), true)
	if max_velocity == 0.0 && max_accel == 0.0 && square_corner_velocity == 0.0 && requested_accel_to_decel == 0.0 {
		logger.Debugf(msg)
	}
	return nil
}
func (self *Toolhead) Cmd_M204(cmd interface{}) error {
	gcmd := cmd.(*GCodeCommand)
	// Use S for accel
	above := 0.0
	accel := gcmd.Get_float("S", math.NaN(), nil, nil, &above, nil)
	if math.IsNaN(accel) {
		// Use minimum of P and T for accel
		p := gcmd.Get_float("P", math.NaN(), nil, nil, &above, nil)
		t := gcmd.Get_float("T", math.NaN(), nil, nil, &above, nil)

		if math.IsNaN(p) == false && math.IsNaN(t) == false {
			accel = math.Min(p, t)
		} else if math.IsNaN(p) == false {
			accel = p
		} else if math.IsNaN(t) == false {
			accel = t
		} else {
			gcmd.Respond_info(fmt.Sprintf("Invalid M204 command: %s", gcmd.Get_commandline()), true)
			return nil
		}
	}

	self.M204(accel)
	return nil
}

func (self *Toolhead) M204(accel float64) {
	self.Max_accel = accel
	self.Calc_junction_deviation()
}
func Add_printer_objects_toolhead(config *ConfigWrapper) {
	config.Get_printer().Add_object("toolhead", NewToolhead(config))
	Add_printer_objects_extruder(config)
}
