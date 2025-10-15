package project

import (
	"fmt"
	"hash/crc32"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/file"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"k3c/project/chelper"
	"log"
	"math"
	"reflect"
	"strconv"
	"strings"
	"time"
)

type erro struct {
	err string
}

type MCU_trsync struct {
	Trdispatch             interface{}
	Trsync_start_cmd       *CommandWrapper
	Mcu                    *MCU
	Trsync_trigger_cmd     *CommandWrapper
	Steppers               []interface{}
	Stepper_stop_cmd       *CommandWrapper
	Home_end_clock         interface{}
	Trdispatch_mcu         interface{}
	Cmd_queue              interface{}
	Reactor                IReactor
	Oid                    int
	Trigger_completion     *ReactorCompletion
	Trsync_set_timeout_cmd *CommandWrapper
	Trsync_query_cmd       *CommandQueryWrapper
}

var REASON_ENDSTOP_HIT int64 = 1
var REASON_HOST_REQUEST int64 = 2
var REASON_PAST_END_TIME int64 = 3
var REASON_COMMS_TIMEOUT int64 = 4

func NewMCU_trsync(mcu *MCU, trdispatch interface{}) *MCU_trsync {
	self := MCU_trsync{}
	self.Mcu = mcu
	self.Trdispatch = trdispatch
	self.Reactor = mcu.Get_printer().Get_reactor()
	self.Steppers = []interface{}{}
	self.Trdispatch_mcu = nil
	self.Oid = mcu.Create_oid()
	self.Cmd_queue = mcu.Alloc_command_queue()
	self.Trsync_start_cmd = nil
	self.Trsync_set_timeout_cmd = self.Trsync_start_cmd
	self.Trsync_trigger_cmd = nil
	self.Trsync_query_cmd = nil
	self.Stepper_stop_cmd = nil
	self.Trigger_completion = nil
	self.Home_end_clock = nil
	mcu.Register_config_callback(self.Build_config)
	printer := mcu.Get_printer()
	printer.Register_event_handler("project:shutdown", self.Shutdown)
	return &self
}

func (self *MCU_trsync) Get_mcu() *MCU {

	return self.Mcu
}
func (self *MCU_trsync) Get_oid() int {

	return self.Oid
}
func (self *MCU_trsync) Get_command_queue() interface{} {

	return self.Cmd_queue
}
func (self *MCU_trsync) Add_stepper(stepper interface{}) {

	for _, o := range self.Steppers {
		if o == stepper {
			return
		}
	}
	self.Steppers = append(self.Steppers, stepper)

}
func (self *MCU_trsync) Get_steppers() []interface{} {
	steppers_back := make([]interface{}, len(self.Steppers))
	copy(steppers_back, self.Steppers)
	return steppers_back
}
func (self *MCU_trsync) Build_config() {

	mcu := self.Mcu
	// Setup config
	mcu.Add_config_cmd(fmt.Sprintf("config_trsync oid=%d", self.Oid), false, false)
	mcu.Add_config_cmd(fmt.Sprintf("trsync_start oid=%d report_clock=0 report_ticks=0 expire_reason=0", self.Oid), false, true)
	//Lookup commands
	self.Trsync_start_cmd, _ = mcu.Lookup_command("trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c", self.Cmd_queue)
	self.Trsync_set_timeout_cmd, _ = mcu.Lookup_command("trsync_set_timeout oid=%c clock=%u", self.Cmd_queue)
	self.Trsync_trigger_cmd, _ = mcu.Lookup_command("trsync_trigger oid=%c reason=%c", self.Cmd_queue)
	self.Trsync_query_cmd = mcu.Lookup_query_command("trsync_trigger oid=%c reason=%c", "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u", self.Oid, self.Cmd_queue, false)
	self.Stepper_stop_cmd, _ = mcu.Lookup_command("stepper_stop_on_trigger oid=%c trsync_oid=%c", self.Cmd_queue)
	// Create trdispatch_mcu object
	set_timeout_tag := mcu.Lookup_command_tag("trsync_set_timeout oid=%c clock=%u")
	trigger_tag := mcu.Lookup_command_tag("trsync_trigger oid=%c reason=%c")
	state_tag := mcu.Lookup_command_tag("trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u")

	self.Trdispatch_mcu = chelper.Trdispatch_mcu_alloc(self.Trdispatch, mcu.Serial.Serialqueue,
		self.Cmd_queue, self.Oid, uint32(set_timeout_tag.(int)), uint32(trigger_tag.(int)),
		uint32(state_tag.(int)))
}
func (self *MCU_trsync) _MCU_trsync() {
	chelper.Free(self.Trdispatch_mcu)
}

func (self *MCU_trsync) Shutdown([]interface{}) error {

	tc := self.Trigger_completion
	if tc != nil {
		self.Trigger_completion = nil
		tc.Complete(false)
	}
	return nil
}
func (self *MCU_trsync) Handle_trsync_state(params map[string]interface{}) error {
	if params["can_trigger"] != nil && params["can_trigger"] != int64(1) {
		tc := self.Trigger_completion
		if (tc) != nil {
			self.Trigger_completion = nil
			reason, _ := params["trigger_reason"].(int64)
			is_failure := reason >= REASON_COMMS_TIMEOUT
			self.Reactor.Async_complete(tc, map[string]interface{}{"aa": is_failure})
		}
	} else if self.Home_end_clock != nil {
		clock := self.Mcu.Clock32_to_clock64(params["clock"].(int64))
		if clock >= self.Home_end_clock.(int64) {
			self.Home_end_clock = nil
			self.Trsync_trigger_cmd.Send([]int64{int64(self.Oid), REASON_PAST_END_TIME}, 0, 0)
		}

	} else {
		return nil
	}
	return nil
}
func (self *MCU_trsync) Start(print_time float64, report_offset float64, trigger_completion *ReactorCompletion, expire_timeout float64) {

	self.Trigger_completion = trigger_completion
	self.Home_end_clock = nil
	clock := self.Mcu.Print_time_to_clock(print_time)
	expire_ticks := self.Mcu.Seconds_to_clock(expire_timeout)
	expire_clock := clock + expire_ticks
	report_ticks := self.Mcu.Seconds_to_clock(expire_timeout * 0.3)
	report_clock := clock + int64(float64(report_ticks)*report_offset+0.5)
	min_extend_ticks := int64(float64(report_ticks)*0.8 + 0.5)
	chelper.Trdispatch_mcu_setup(self.Trdispatch_mcu, uint64(clock), uint64(expire_clock), uint64(expire_ticks), uint64(min_extend_ticks))
	self.Mcu.Register_response(self.Handle_trsync_state, "trsync_state", self.Oid)
	self.Trsync_start_cmd.Send([]int64{int64(self.Oid), report_clock, report_ticks,
		REASON_COMMS_TIMEOUT}, 0, report_clock)
	for _, s := range self.Steppers {
		self.Stepper_stop_cmd.Send([]int64{int64(s.(*MCU_stepper).Get_oid()), int64(self.Oid)}, 0, 0)
	}
	self.Trsync_set_timeout_cmd.Send([]int64{int64(self.Oid), expire_clock}, 0, expire_clock)
}
func (self *MCU_trsync) Set_home_end_time(home_end_time float64) {

	self.Home_end_clock = self.Mcu.Print_time_to_clock(home_end_time)
}
func (self *MCU_trsync) Stop() interface{} {

	self.Mcu.Register_response(nil, "trsync_state", self.Oid)
	self.Trigger_completion = nil
	if self.Mcu.Is_fileoutput() {
		return REASON_ENDSTOP_HIT
	}
	params := self.Trsync_query_cmd.Send([]int64{int64(self.Oid), REASON_HOST_REQUEST}, 0, 0)
	for _, s := range self.Steppers {
		s.(*MCU_stepper).Note_homing_end()
	}
	return params.(map[string]interface{})["trigger_reason"]
}

var TRSYNC_TIMEOUT = 0.025
var TRSYNC_SINGLE_MCU_TIMEOUT = 0.25

type MCU_endstop struct {
	Pin                interface{}
	Trdispatch         interface{}
	Home_cmd           *CommandWrapper
	Invert             int
	Trigger_completion *ReactorCompletion
	Mcu                *MCU
	Pullup             interface{}
	Trsyncs            []*MCU_trsync
	Oid                int
	Rest_ticks         int64
	Query_cmd          *CommandQueryWrapper
}

var RETRY_QUERY = 1.000

func NewMCU_endstop(mcu *MCU, pin_params map[string]interface{}) interface{} {
	self := MCU_endstop{}
	self.Mcu = mcu
	self.Pin = pin_params["pin"]
	self.Pullup = pin_params["pullup"]
	self.Invert = pin_params["invert"].(int)
	self.Oid = self.Mcu.Create_oid()
	self.Home_cmd = nil
	self.Query_cmd = nil
	self.Mcu.Register_config_callback(self.Build_config)
	self.Trigger_completion = nil
	self.Rest_ticks = 0
	self.Trdispatch = chelper.Trdispatch_alloc()
	//runtime.SetFinalizer(&self,self._MCU_endstop)
	self.Trsyncs = []*MCU_trsync{NewMCU_trsync(mcu, self.Trdispatch)}
	return &self
}
func (self *MCU_endstop) _MCU_endstop() {
	chelper.Free(self.Trdispatch)
}
func (self *MCU_endstop) Get_mcu() *MCU {
	return self.Mcu
}
func (self *MCU_endstop) Add_stepper(_stepper interface{}) {
	trsyncs := map[*MCU]*MCU_trsync{}
	for _, trsync := range self.Trsyncs {
		trsyncs[trsync.Get_mcu()] = trsync
	}
	stepper_ := _stepper.(*MCU_stepper)
	trsync := trsyncs[stepper_.Get_mcu()]
	if trsync == nil {
		trsync = NewMCU_trsync(stepper_.Get_mcu(), self.Trdispatch)
		self.Trsyncs = append(self.Trsyncs, trsync)
	}
	trsync.Add_stepper(_stepper)
	// Check for unsupported multi-mcu shared stepper rails
	sname := stepper_.Get_name(false)
	if strings.HasPrefix(sname, "stepper_") {
		for _, ot := range self.Trsyncs {
			for _, s := range ot.Get_steppers() {
				if ot != trsync && strings.HasPrefix(s.(*MCU_stepper).Get_name(false), sname[:9]) {
					//cerror := self.Mcu.Get_printer().Config_error
					log.Print("Multi-mcu homing not supported on" +
						" multi-mcu shared axis")
				}
			}
		}
	}
}
func (self *MCU_endstop) Get_steppers() []interface{} {
	res := []interface{}{}
	for _, trsync := range self.Trsyncs {
		for _, s := range trsync.Get_steppers() {
			res = append(res, s)
		}
	}
	return res
}
func (self *MCU_endstop) Build_config() {
	//Setup config
	self.Mcu.Add_config_cmd(fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", self.Oid, self.Pin, self.Pullup), false, false)
	self.Mcu.Add_config_cmd(fmt.Sprintf("endstop_home oid=%d clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0", self.Oid), false, true)
	//Lookup commands
	cmd_queue := self.Trsyncs[0].Get_command_queue()
	self.Home_cmd, _ = self.Mcu.Lookup_command("endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c", cmd_queue)
	self.Query_cmd = self.Mcu.Lookup_query_command("endstop_query_state oid=%c", "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c", self.Oid, cmd_queue, false)
}
func (self *MCU_endstop) Home_start(print_time float64, sample_time float64, sample_count int64, rest_time float64, triggered int64) interface{} {

	clock := self.Mcu.Print_time_to_clock(print_time)
	rest_ticks := self.Mcu.Print_time_to_clock(print_time+rest_time) - clock
	self.Rest_ticks = rest_ticks
	reactor := self.Mcu.Get_printer().Get_reactor()
	self.Trigger_completion = reactor.Completion()
	expire_timeout := TRSYNC_TIMEOUT
	if len(self.Trsyncs) == 1 {
		expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
	}
	for i, trsync := range self.Trsyncs {
		report_offset := float64(i) / float64(len(self.Trsyncs))
		trsync.Start(print_time, report_offset, self.Trigger_completion, expire_timeout)
	}
	etrsync := (self.Trsyncs)[0]

	chelper.Trdispatch_start(self.Trdispatch, uint32(REASON_HOST_REQUEST))
	self.Home_cmd.Send([]int64{int64(self.Oid), clock, self.Mcu.Seconds_to_clock(sample_time),
		sample_count, rest_ticks, triggered ^ int64(self.Invert),
		int64(etrsync.Get_oid()), REASON_ENDSTOP_HIT}, 0, clock)
	return self.Trigger_completion
}
func (self *MCU_endstop) Home_wait(home_end_time float64) float64 {

	etrsync := (self.Trsyncs)[0]
	etrsync.Set_home_end_time(home_end_time)
	if self.Mcu.Is_fileoutput() {
		self.Trigger_completion.Complete(true)
	}
	self.Trigger_completion.Wait(constants.NEVER, nil)
	self.Home_cmd.Send([]int64{int64(self.Oid), 0, 0, 0, 0, 0, 0, 0}, 0, 0)
	chelper.Trdispatch_stop(self.Trdispatch)

	res := []interface{}{}
	for _, trsync := range self.Trsyncs {
		res = append(res, trsync.Stop())
	}
	cnt := 0
	for _, r := range res {
		if r == REASON_COMMS_TIMEOUT {
			cnt++
		}
	}
	if len(res) == cnt {
		return -1.
	}
	if res[0] != REASON_ENDSTOP_HIT {
		return 0.
	}
	if self.Mcu.Is_fileoutput() {
		return home_end_time
	}
	params := self.Query_cmd.Send([]int64{int64(self.Oid)}, 0, 0).(map[string]interface{})
	next_clock := self.Mcu.Clock32_to_clock64(params["next_clock"].(int64))
	return self.Mcu.Clock_to_print_time(next_clock - self.Rest_ticks)
}

func (self *MCU_endstop) Query_endstop(print_time float64) int {

	clock := self.Mcu.Print_time_to_clock(print_time)
	if self.Mcu.Is_fileoutput() {
		return 0
	}
	params := self.Query_cmd.Send([]int64{int64(self.Oid)}, clock, 0)
	return cast.ForceInt(params.(map[string]interface{})["pin_value"].(int64) ^ int64(self.Invert))
}

type MCU_digital_out struct {
	Pin            string
	Invert         int
	Mcu            *MCU
	_last_clock    int64
	Max_duration   float64
	_set_cmd       *CommandWrapper
	Oid            int
	Start_value    int
	Shutdown_value int
}

func NewMCU_digital_out(mcu *MCU, pin_params map[string]interface{}) interface{} {
	self := MCU_digital_out{}
	self.Mcu = mcu
	self.Oid = -1
	self.Mcu.Register_config_callback(self.Build_config)
	self.Pin = pin_params["pin"].(string)
	self.Invert = pin_params["invert"].(int)
	self.Start_value = self.Invert
	self.Shutdown_value = self.Start_value
	self.Max_duration = 2.0
	self._last_clock = 0
	self._set_cmd = nil
	return &self
}

func (self *MCU_digital_out) Get_mcu() *MCU {

	return self.Mcu
}
func (self *MCU_digital_out) Setup_max_duration(max_duration float64) {

	self.Max_duration = max_duration
}
func (self *MCU_digital_out) Setup_start_value(start_value float64, shutdown_value float64) {
	var start_value_temp int
	var shutdown_value_temp int
	if start_value != 0 {
		start_value_temp = 1
	} else {
		start_value_temp = 0
	}
	if shutdown_value != 0 {
		shutdown_value_temp = 1
	} else {
		shutdown_value_temp = 0
	}

	self.Start_value = start_value_temp ^ self.Invert
	self.Shutdown_value = shutdown_value_temp ^ self.Invert
}
func (self *MCU_digital_out) Build_config() {
	if self.Max_duration != 0 && (self.Start_value) != self.Shutdown_value {
		panic("Pin with max duration must have start value equal to shutdown value")
	}
	mdur_ticks := self.Mcu.Seconds_to_clock(self.Max_duration)
	if mdur_ticks >= (1 << 31) {
		panic("Digital pin max duration too large")
	}

	self.Mcu.Request_move_queue_slot()
	self.Oid = self.Mcu.Create_oid()
	self.Mcu.Add_config_cmd(fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d", self.Oid, self.Pin, self.Start_value, self.Shutdown_value, mdur_ticks), false, false)
	self.Mcu.Add_config_cmd(fmt.Sprintf("update_digital_out oid=%d value=%d", self.Oid, self.Start_value), false, true)
	cmd_queue := self.Mcu.Alloc_command_queue()
	self._set_cmd, _ = self.Mcu.Lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", cmd_queue)
}
func (self *MCU_digital_out) Set_digital(print_time float64, value int) {

	clock := self.Mcu.Print_time_to_clock(print_time)
	var value_temp int64
	if value != 0 {
		value_temp = 1
	} else {
		value_temp = 0
	}
	self._set_cmd.Send([]int64{int64(self.Oid), clock, value_temp ^ int64(self.Invert)}, self._last_clock, clock)
	self._last_clock = clock
}

type MCU_pwm struct {
	Pin            string
	Invert         int
	Mcu            *MCU
	_pwm_max       float64
	Hardware_pwm   interface{}
	_last_clock    int64
	Cycle_time     float64
	Start_value    float64
	_set_cmd       *CommandWrapper
	Oid            int
	Max_duration   float64
	Shutdown_value float64
}

func NewMCU_pwm(mcu *MCU, pin_params map[string]interface{}) interface{} {
	self := MCU_pwm{}
	self.Mcu = mcu
	self.Hardware_pwm = false
	self.Cycle_time = 0.1
	self.Max_duration = 2.0
	self.Oid = -1
	self.Mcu.Register_config_callback(self.Build_config)
	self.Pin = pin_params["pin"].(string)
	self.Invert = pin_params["invert"].(int)
	self.Start_value = float64(self.Invert)
	self.Shutdown_value = self.Start_value
	self._last_clock = 0
	self._pwm_max = 0.0
	self._set_cmd = nil
	return &self
}
func (self *MCU_pwm) Get_mcu() *MCU {

	return self.Mcu
}
func (self *MCU_pwm) Setup_max_duration(max_duration float64) {

	self.Max_duration = max_duration
}
func (self *MCU_pwm) Setup_cycle_time(cycle_time float64, hardware_pwm bool) {

	self.Cycle_time = cycle_time
	self.Hardware_pwm = hardware_pwm
}
func (self *MCU_pwm) Setup_start_value(start_value float64, shutdown_value float64) {

	if self.Invert > 0 {
		start_value = 1.0 - start_value
		shutdown_value = 1.0 - shutdown_value
	}
	self.Start_value = math.Max(0.0, math.Min(1.0, start_value))
	self.Shutdown_value = math.Max(0.0, math.Min(1.0, shutdown_value))
}
func (self *MCU_pwm) Build_config() {
	var svalue int
	if self.Max_duration != 0 && (self.Start_value) != self.Shutdown_value {
		panic("Pin with max duration must have start value equal to shutdown value")
	}
	cmd_queue := self.Mcu.Alloc_command_queue()
	curtime := self.Mcu.Get_printer().Get_reactor().Monotonic()
	printtime := self.Mcu.Estimated_print_time(curtime)
	self._last_clock = self.Mcu.Print_time_to_clock(printtime + 0.2)
	cycle_ticks := self.Mcu.Seconds_to_clock(self.Cycle_time)
	mdur_ticks := self.Mcu.Seconds_to_clock(self.Max_duration)
	if mdur_ticks >= (1 << 31) {
		panic("PWM pin max duration too large")
	}
	if self.Hardware_pwm.(bool) {
		self._pwm_max = self.Mcu.Get_constant_float("_pwm_max")
		self.Mcu.Request_move_queue_slot()
		self.Oid = self.Mcu.Create_oid()
		self.Mcu.Add_config_cmd(fmt.Sprintf("config_pwm_out oid=%d pin=%s cycle_ticks=%d value=%d default_value=%d max_duration=%d", self.Oid, self.Pin, cycle_ticks, int(self.Start_value*self._pwm_max), int(self.Shutdown_value*self._pwm_max), mdur_ticks), false, false)
		svalue = int(self.Start_value*self._pwm_max + 0.5)
		self.Mcu.Add_config_cmd(fmt.Sprintf("queue_pwm_out oid=%d clock=%d value=%d", self.Oid, self._last_clock, svalue), false, true)
		self._set_cmd, _ = self.Mcu.Lookup_command("queue_pwm_out oid=%c clock=%u value=%hu", cmd_queue)
		return
	}
	// Software PWM
	if self.Shutdown_value != 0.0 && self.Shutdown_value != 1.0 {
		panic("shutdown value must be 0.0 or 1.0 on soft pwm")
	}

	if cycle_ticks >= (1 << 31) {
		panic("PWM pin cycle time too large")
	}
	self.Mcu.Request_move_queue_slot()
	self.Oid = self.Mcu.Create_oid()
	start_value := 0
	if self.Start_value >= 1.0 {
		start_value = 1
	} else {
		start_value = 0
	}
	shutdown_value := 0
	if (self.Shutdown_value) >= 0.5 {
		shutdown_value = 1
	} else {
		shutdown_value = 0
	}
	self.Mcu.Add_config_cmd(fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d", self.Oid, self.Pin, start_value, shutdown_value, mdur_ticks), false, false)
	self.Mcu.Add_config_cmd(fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", self.Oid, cycle_ticks), false, false)
	self._pwm_max = float64(cycle_ticks)
	svalue = int(self.Start_value*float64(cycle_ticks) + 0.5)
	self.Mcu.Add_config_cmd(fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", self.Oid, self._last_clock, svalue), true, false)
	self._set_cmd, _ = self.Mcu.Lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", cmd_queue)
}
func (self *MCU_pwm) Set_pwm(print_time float64, val float64) {
	if self.Invert != 0 {
		val = 1.0 - val
	}
	v := int64(math.Max(0.0, math.Min(1.0, float64(val)))*float64(self._pwm_max) + 0.5)
	clock := self.Mcu.Print_time_to_clock(print_time)
	self._set_cmd.Send([]int64{int64(self.Oid), clock, v}, self._last_clock, clock)
	self._last_clock = clock
}

type MCU_adc struct {
	Pin               string
	Mcu               *MCU
	Report_clock      int64
	Min_sample        float64
	Max_sample        float64
	Last_state        []float64
	Inv_max_adc       float64
	Sample_count      int
	Range_check_count int
	Oid               int
	Callback          func(float64, float64)
	Sample_time       float64
	Report_time       float64
}

func NewMCU_adc(mcu *MCU, pin_params map[string]interface{}) interface{} {
	self := MCU_adc{}
	self.Mcu = mcu
	self.Pin = pin_params["pin"].(string)
	self.Min_sample = 0.0
	self.Max_sample = self.Min_sample
	self.Sample_time = 0.0
	self.Report_time = self.Sample_time
	self.Sample_count = 0
	self.Range_check_count = self.Sample_count
	self.Report_clock = 0
	self.Last_state = []float64{0.0, 0.0}
	self.Oid = -1
	self.Callback = nil
	self.Mcu.Register_config_callback(self.Build_config)
	self.Inv_max_adc = 0.0
	return &self
}

func (self *MCU_adc) Get_mcu() *MCU {

	return self.Mcu
}
func (self *MCU_adc) Setup_minmax(sample_time float64, sample_count int, minval float64, maxval float64, range_check_count int) {

	self.Sample_time = sample_time
	self.Sample_count = sample_count
	self.Min_sample = minval
	self.Max_sample = maxval
	self.Range_check_count = range_check_count
}
func (self *MCU_adc) Setup_adc_callback(report_time float64, callback func(float64, float64)) {

	self.Report_time = report_time
	self.Callback = callback
}
func (self *MCU_adc) Get_last_value() []float64 {

	return self.Last_state
}
func (self *MCU_adc) Build_config() {

	if self.Sample_count == 0 {
		return
	}
	self.Oid = self.Mcu.Create_oid()
	self.Mcu.Add_config_cmd(fmt.Sprintf("config_analog_in oid=%d pin=%s", self.Oid, self.Pin), false, false)
	clock := self.Mcu.Get_query_slot(self.Oid)
	sample_ticks := self.Mcu.Seconds_to_clock(self.Sample_time)
	mcu_adc_max := self.Mcu.Get_constant_float("ADC_MAX")
	max_adc := float64(self.Sample_count) * mcu_adc_max
	self.Inv_max_adc = 1.0 / max_adc
	self.Report_clock = self.Mcu.Seconds_to_clock(self.Report_time)
	min_sample := math.Max(0, math.Min(65535, float64(int(self.Min_sample*max_adc))))
	max_sample := math.Max(0, math.Min(65535, float64(int(math.Ceil(self.Max_sample*max_adc)))))
	self.Mcu.Add_config_cmd(fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
		self.Oid, clock, sample_ticks, self.Sample_count, self.Report_clock, int(min_sample), int(max_sample), self.Range_check_count), true, false)
	self.Mcu.Register_response(self.Handle_analog_in_state, "analog_in_state", self.Oid)
}
func (self *MCU_adc) Handle_analog_in_state(params map[string]interface{}) error {

	last_value := float64((params)["value"].(int64)) * self.Inv_max_adc
	next_clock := self.Mcu.Clock32_to_clock64(params["next_clock"].(int64))
	last_read_clock := next_clock - self.Report_clock
	last_read_time := self.Mcu.Clock_to_print_time(last_read_clock)
	self.Last_state = []float64{last_value, last_read_time}
	if self.Callback != nil {
		self.Callback(last_read_time, last_value)
	}
	return nil
}

// Class to retry sending of a query command until a given response is received
type RetryAsyncCommand struct {
	Completion     interface{}
	Reactor        IReactor
	Serial         *SerialReader
	Oid            int
	Min_query_time float64
	Name           string
	need_response  bool
}

var TIMEOUT_TIME = 5.0
var RETRY_TIME = 0.500

func (self *RetryAsyncCommand) __init__(serial interface{}, name string, oid int) {

	self.Serial = serial.(*SerialReader)
	self.Name = name
	self.Oid = oid
	self.Reactor = self.Serial.Get_reactor()
	self.Completion = self.Reactor.Completion()
	self.Min_query_time = self.Reactor.Monotonic()
	self.need_response = true
	self.Serial.Register_response(self.Handle_callback, name, oid)
}
func (self *RetryAsyncCommand) Handle_callback(params map[string]interface{}) error {

	send_time := chelper.CdoubleTofloat64(params["#sent_time"])
	if self.need_response && send_time >= self.Min_query_time {
		self.need_response = false
		self.Reactor.Async_complete(self.Completion.(*ReactorCompletion), params)
	}

	return nil
}

func (self *RetryAsyncCommand) get_response(cmds []interface{}, cmd_queue interface{}, minclock, reqclock int64) (map[string]interface{}, error) {

	cmd := cmds[0]
	self.Serial.Raw_send_wait_ack(cmd.([]int), minclock, reqclock, cmd_queue)
	self.Min_query_time = 0
	first_query_time := self.Reactor.Monotonic()
	query_time := first_query_time
	for {
		params := self.Completion.(*ReactorCompletion).Wait(query_time+RETRY_TIME, nil)
		if params != nil {
			self.Serial.Register_response(nil, self.Name, self.Oid)
			return params.(map[string]interface{}), nil
		}
		query_time = self.Reactor.Monotonic()
		if (query_time) > (first_query_time + TIMEOUT_TIME) {
			self.Serial.Register_response(nil, self.Name, self.Oid)
			panic(fmt.Sprintf("Timeout on wait for '%s' response '%d'", self.Name, self.Oid))
		}
		self.Serial.Raw_send(cmd.([]int), minclock, minclock, cmd_queue)
	}
}

type RetryCommand interface {
	get_response(cmds []interface{}, cmd_queue interface{}, minclock, reqclock int64) (map[string]interface{}, error)
}

func NewRetryAsyncCommand(serial *SerialReader, name string, oid int) *RetryAsyncCommand {
	ob := RetryAsyncCommand{}
	ob.__init__(serial, name, oid)
	return &ob
}

// Wrapper around query commands
type CommandQueryWrapper struct {
	Xmit_helper bool
	Cmd_queue   interface{}
	Error       interface{}
	Response    string
	Serial      *SerialReader
	Oid         int
	Cmd         interface{} //*MessageFormat
}

func NewCommandQueryWrapper(serial *SerialReader, msgformat string, respformat string, oid int, cmd_queue interface{}, is_async bool, error interface{}) *CommandQueryWrapper {
	self := new(CommandQueryWrapper)
	self.Serial = serial
	cmd, err := serial.Get_msgparser().Lookup_command(msgformat)
	if err != nil {
		logger.Error(err)
	}
	self.Cmd = cmd
	serial.Get_msgparser().Lookup_command(respformat)
	self.Response = strings.Split(respformat, " ")[0]
	self.Oid = oid
	self.Error = error
	self.Xmit_helper = is_async

	if cmd_queue == nil {
		cmd_queue = serial.Get_default_command_queue()
	}
	self.Cmd_queue = cmd_queue
	return self
}

func (self *CommandQueryWrapper) Do_send(cmds [][]int, minclock, reqclock int64) interface{} {
	var xh interface{}
	if self.Xmit_helper {
		xh = NewRetryAsyncCommand(self.Serial, self.Response, self.Oid)
	} else {
		xh = NewSerialRetryCommand(self.Serial, self.Response, self.Oid)
	}

	reqclock = maths.Max64(minclock, reqclock)
	cmd := []interface{}{}
	for _, c := range cmds {
		cmd = append(cmd, c)
	}
	res, _ := xh.(RetryCommand).get_response(cmd, self.Cmd_queue, minclock, reqclock)
	return res
}
func (self *CommandQueryWrapper) Send(data interface{}, minclock, reqclock int64) interface{} {
	return self.Do_send([][]int{self.Cmd.(*MessageFormat).Encode(data)}, minclock, reqclock)
}
func (self *CommandQueryWrapper) Send_with_preface(preface_cmd *CommandWrapper, preface_data interface{}, data interface{}, minclock, reqclock int64) interface{} {
	cmds := [][]int{preface_cmd.Cmd.Encode(preface_data.([]interface{})), self.Cmd.(*MessageFormat).Encode([]interface{}{data})}
	return self.Do_send(cmds, minclock, reqclock)
}

// Wrapper around command sending
type CommandWrapper struct {
	Cmd_queue interface{}
	Serial    *SerialReader
	Cmd       *MessageFormat
}

func NewCommandWrapper(serial *SerialReader, msgformat interface{}, cmd_queue interface{}) (*CommandWrapper, error) {
	self := CommandWrapper{}
	self.Serial = serial
	cmd, err := serial.Get_msgparser().Lookup_command(msgformat.(string))
	if err != nil {
		return nil, err
	}
	if cmd != nil {
		self.Cmd = cmd.(*MessageFormat)
	}
	if (cmd_queue) == nil {
		cmd_queue = serial.Get_default_command_queue()
	}
	self.Cmd_queue = cmd_queue
	return &self, nil
}

func (self *CommandWrapper) Send(data interface{}, minclock, reqclock int64) {
	cmd := self.Cmd.Encode(data)
	self.Serial.Raw_send(cmd, minclock, reqclock, self.Cmd_queue)
}

type MCU struct {
	error                string
	_printer             *Printer
	_clocksync           ClockSyncAble
	_reactor             IReactor
	_name                string
	Serial               *SerialReader
	_baud                int
	_canbus_iface        interface{}
	_serialport          string
	_restart_method      string
	_reset_cmd           *CommandWrapper
	_is_mcu_bridge       bool
	_emergency_stop_cmd  *CommandWrapper
	_is_shutdown         bool
	_shutdown_clock      int64
	_shutdown_msg        string
	Oid_count            int
	_config_callbacks    []interface{}
	_config_cmds         []string
	_restart_cmds        []string
	_init_cmds           []string
	_mcu_freq            float64
	_ffi_lib             interface{}
	_max_stepper_error   float64
	_reserved_move_slots int64
	_stepqueues          []interface{}
	_steppersync         interface{}
	_get_status_info     map[string]interface{}
	_stats_sumsq_base    float64
	_mcu_tick_avg        float64
	_mcu_tick_stddev     float64
	_mcu_tick_awake      float64
	_config_reset_cmd    *CommandWrapper
	_is_timeout          bool
	_flush_callbacks     []func(float64, int64)
}

func NewMCU(config *ConfigWrapper, clocksync ClockSyncAble) *MCU {
	self := MCU{}
	self._printer = config.Get_printer()
	printer := config.Get_printer()
	self._clocksync = clocksync
	self._reactor = printer.Get_reactor()
	self._name = config.Get_name()
	if strings.HasPrefix(self._name, "mcu ") {
		self._name = self._name[4:]
	}
	// Serial port
	wp := fmt.Sprintf("mcu '%s': ", self._name)
	self.Serial = NewSerialReader(self._reactor, wp)
	self._baud = 0
	self._canbus_iface = nil
	canbus_uuid := config.Get("canbus_uuid", value.None, true)
	if canbus_uuid != nil && canbus_uuid.(string) != "" {
		self._serialport = canbus_uuid.(string)
		self._canbus_iface = config.Get("canbus_interface", "can0", true)
		//cbid := self._printer.Load_object(config, "canbus_ids")
		//cbid.Add_uuid(config, canbus_uuid, self._canbus_iface)
	} else {
		self._serialport = config.Get("serial", object.Sentinel{}, true).(string)
		if (strings.HasPrefix(self._serialport, "/dev/rpmsg_") ||
			strings.HasPrefix(self._serialport, "/tmp/klipper_host_")) == false {
			self._baud = config.Getint("baud", 250000, 2400, 0, true)
		}
	}
	// Restarts
	restart_methods := []string{"", "arduino", "cheetah", "command", "rpi_usb"}
	self._restart_method = "command"
	if self._baud > 0 {
		rmethods := map[interface{}]interface{}{}
		for _, m := range restart_methods {
			rmethods[m] = m
		}
		self._restart_method = config.Getchoice("restart_method",
			rmethods, nil, true).(string)
	}
	self._reset_cmd, self._config_reset_cmd = nil, nil
	self._is_mcu_bridge = false
	self._emergency_stop_cmd = nil
	self._is_shutdown, self._is_timeout = false, false
	self._shutdown_clock = 0
	self._shutdown_msg = ""
	// Config building
	pins := printer.Lookup_object("pins", object.Sentinel{})
	pins.(*PrinterPins).Register_chip(self._name, &self)
	self.Oid_count = 0
	self._config_callbacks = []interface{}{}
	self._config_cmds = []string{}
	self._restart_cmds = []string{}
	self._init_cmds = []string{}
	self._mcu_freq = 0.
	// Move command queuing
	self._ffi_lib = chelper.Get_ffi()
	//ffi_main := self._ffi_lib
	self._max_stepper_error = config.Getfloat("max_stepper_error", 0.000025,
		0., 0, 0, 0, true)
	self._reserved_move_slots = 0
	self._stepqueues = []interface{}{}
	self._steppersync = nil
	// Stats
	self._get_status_info = map[string]interface{}{}
	self._stats_sumsq_base = 0.
	self._mcu_tick_avg = 0.
	self._mcu_tick_stddev = 0.
	self._mcu_tick_awake = 0.
	// Register handlers
	printer.Register_event_handler("project:firmware_restart",
		self._firmware_restart)
	printer.Register_event_handler("project:mcu_identify",
		self._mcu_identify)
	printer.Register_event_handler("project:connect", self._connect)
	printer.Register_event_handler("project:shutdown", self._shutdown)
	printer.Register_event_handler("project:disconnect", self._disconnect)
	printer.Register_event_handler("project:ready", self._ready)
	return &self
}

func (self *MCU) Get_status_info() map[string]interface{} {
	return self._get_status_info
}

func (self *MCU) _handle_mcu_stats(params map[string]interface{}) error {
	count := cast.ToFloat64(params["count"])
	tick_sum := cast.ToFloat64(params["sum"])
	tick_sumsq := cast.ToFloat64(params["sumsq"])
	c := 1.0 / (count * self._mcu_freq)
	self._mcu_tick_avg = tick_sum * c
	tick_sumsq = tick_sumsq * self._stats_sumsq_base
	diff := count*tick_sumsq - tick_sum*tick_sum
	self._mcu_tick_stddev = c * math.Sqrt(math.Max(0.0, diff))
	self._mcu_tick_awake = tick_sum / self._mcu_freq
	return nil
}

func (self *MCU) _handle_shutdown(params map[string]interface{}) error {
	if self._is_shutdown {
		return nil
	}
	self._is_shutdown = true
	clock := params["clock"]
	if clock != nil {
		// self._shutdown_clock = self.Clock32_to_clock64(uint64(clock.(int)))
		self._shutdown_clock, _ = clock.(int64)
	}
	self._shutdown_msg = params["static_string_id"].(string)
	msg := params["static_string_id"].(string)
	err_msg := fmt.Sprintf("MCU '%s' %s: %s\n%s\n%s", self._name, params["#name"],
		self._shutdown_msg, self._clocksync.Dump_debug(),
		self.Serial.Dump_debug())
	log.Println(err_msg)
	prefix := fmt.Sprintf("MCU '%s' shutdown: ", self._name)
	if params["#name"] == "is_shutdown" {
		prefix = fmt.Sprintf("Previous MCU '%s' shutdown: ", self._name)
	}
	//self._printer.Invoke_async_shutdown(prefix + msg + error_help(msg))
	self._printer.invoke_async_shutdown(prefix + msg)
	gcode := self._printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Respond_info(fmt.Sprintf("MCU '%s' %s: %s", self._name, params["#name"], self._shutdown_msg), true)
	return nil
}
func (self *MCU) _handle_starting(params map[string]interface{}) {
	if self._is_shutdown == false {
		self._printer.invoke_async_shutdown(fmt.Sprintf("MCU '%s' spontaneous restart", self._name))
	}

}

// Connection phase
func (self *MCU) _check_restart(reason string) {
	start_reason := self._printer.Get_start_args()["start_reason"]
	if start_reason == "firmware_restart" {
		return
	}
	logger.Debugf("Attempting automated MCU '%s' restart: %s",
		self._name, reason)
	self._printer.Request_exit("firmware_restart")
	self._reactor.Pause(self._reactor.Monotonic() + 2.000)
	panic(fmt.Sprintf("Attempt MCU '%s' restart failed", self._name))
}
func (self *MCU) _connect_file(pace bool) {
	// In a debugging mode.  Open debug output file and read data dictionary
	//start_args := self._printer.Get_start_args()
	if self._name == "mcu" {
		//out_fname := start_args["debugoutput"]
		//dict_fname := start_args["dictionary"]
	} else {
		//out_fname := start_args["debugoutput"+"-"+self._name]
		//dict_fname := start_args["dictionary_"+self._name]
	}
	//outfile := open(out_fname, "wb")
	//dfile := open(dict_fname, "rb")
	//dict_data := dfile.read()
	//dfile.close()
	//self.Serial.Connect_file(outfile, dict_data)
	//self._clocksync.Connect_file(self.Serial, pace)
	//// Handle pacing
	//if pace == false {
	//	self.estimated_print_time = func(eventtime float64) float64 {
	//		return 0.
	//	}
	//}
}

func (self *MCU) _send_config(prev_crc *uint32) error {
	// Build config commands
	for _, cb := range self._config_callbacks {
		cb.(func())()
	}
	config_cmds := self._config_cmds
	self._config_cmds = []string{}
	self._config_cmds = append(self._config_cmds, fmt.Sprintf("allocate_oids count=%d", self.Oid_count))
	self._config_cmds = append(self._config_cmds, config_cmds...)
	// Resolve pin names
	self.Serial.Get_msgparser().Get_constant("MCU", nil, reflect.String)
	ppins := self._printer.Lookup_object("pins", object.Sentinel{})
	pin_resolver := ppins.(*PrinterPins).Get_pin_resolver(self._name)
	cmdlists := [][]string{self._config_cmds, self._restart_cmds, self._init_cmds}
	for _, cmdlist := range cmdlists {
		for i, cmd := range cmdlist {
			cmdlist[i] = pin_resolver.Update_command(cmd)
		}
	}
	// Calculate config CRC
	str := self._config_cmds
	encoded_config := strings.Join(str, "\n")
	config_crc := crc32.ChecksumIEEE([]byte(encoded_config)) & 0xffffffff
	self.Add_config_cmd(fmt.Sprintf("finalize_config crc=%d", config_crc), false, false)
	if prev_crc != nil {
		logger.Debug("config_crc:", config_crc, " prev_crc:", *prev_crc)
	} else {
		logger.Debug("config_crc:", config_crc)
	}
	if prev_crc != nil && config_crc != *prev_crc {
		self._check_restart("CRC mismatch")
		panic(fmt.Sprintf("MCU '%s' CRC does not match config", self._name))
	}
	// Transmit config messages (if needed)
	self.Register_response(self._handle_starting, "starting", nil)
	if prev_crc == nil {
		logger.Debugf("Sending MCU '%s' printer configuration...",
			self._name)
		for _, c := range self._config_cmds {
			self.Serial.Send(c, 0, 0)
		}
	} else {
		for _, c := range self._restart_cmds {
			self.Serial.Send(c, 0, 0)
		}
	}
	// Transmit init messages
	for _, c := range self._init_cmds {
		self.Serial.Send(c, 0, 0)
	}
	return nil
}
func (self *MCU) _send_get_config() map[string]interface{} {
	get_config_cmd := self.Lookup_query_command(
		"get_config",
		"config is_config=%c crc=%u is_shutdown=%c move_count=%hu", -1, nil, false)
	if self.Is_fileoutput() {
		return map[string]interface{}{"is_config": 0, "move_count": 500, "crc": 0}
	}
	config_params := get_config_cmd.Send([]int64{}, 0, 0)
	if config_params == nil {
		return nil
	}
	if self._is_shutdown {
		logger.Panicf("MCU '%s' error during config: %s", self._name, self._shutdown_msg)
		return nil
	}
	if config_params.(map[string]interface{})["is_shutdown"].(int64) == 1 {
		logger.Panicf("Can not update MCU '%s' config as it is shutdown", self._name)
		return nil
	}
	return config_params.(map[string]interface{})
}
func (self *MCU) _log_info() string {
	msgparser := self.Serial.Get_msgparser()
	message_count := len(msgparser.Get_messages())
	version, build_versions := msgparser.Get_version_info()
	str := []string{}
	for k, v := range self.Get_constants() {
		strings.Join(str, fmt.Sprintf("%s=%s ", k, v))
	}
	return fmt.Sprintf("Loaded MCU '%s' %d commands (%s / %s) MCU '%s' config: %s",
		self._name, message_count, version, build_versions, self._name, str)
}

func (self *MCU) _connect([]interface{}) error {
	config_params := self._send_get_config()
	if config_params == nil {
		//self._printer.Invoke_shutdown("MCU is shudown,please firmware restart")
		return fmt.Errorf("MCU is shudown,please firmware restart")
	}
	if config_params["is_config"].(int64) == 0 {
		if self._restart_method == "rpi_usb" {
			// Only configure mcu after usb power reset
			self._check_restart("full reset before config")
		}
		// Not configured - Send config and issue get_config again
		self._send_config(nil)
		config_params = self._send_get_config()
		if (config_params == nil || config_params["is_config"] == 0) && self.Is_fileoutput() == false {
			panic(fmt.Sprintf("Unable to configure MCU '%s'", self._name))
		}
	} else {
		start_reason := self._printer.Get_start_args()["start_reason"]
		if start_reason == "firmware_restart" {
			panic(fmt.Sprintf("Failed automated reset of MCU '%s'", self._name))
		}
		aa := uint32(config_params["crc"].(int64))
		// Already configured - Send init commands
		self._send_config(&aa)
	}
	// Setup steppersync with the move_count returned by get_config
	move_count, _ := config_params["move_count"].(int64)
	if move_count < self._reserved_move_slots {
		logger.Panicf("Too few moves available on MCU '%s'", self._name)
		return nil
	}

	self._steppersync =
		chelper.Steppersync_alloc(self.Serial.Serialqueue, self._stepqueues,
			len(self._stepqueues), int(move_count-self._reserved_move_slots))

	chelper.Steppersync_set_time(self._steppersync, 0., self._mcu_freq)
	// log config information
	move_msg := fmt.Sprintf("Configured MCU '%s' (%d moves)", self._name, move_count)
	logger.Debugf(move_msg)
	log_info := self._log_info() + "\n" + move_msg
	self._printer.Set_rollover_info(self._name, log_info, false)
	return nil
}

func (self *MCU) _mcu_identify(argv []interface{}) error {
	if self.Is_fileoutput() {
		self._connect_file(false)
	} else {
		resmeth := self._restart_method
		exist, err := file.PathExists(self._serialport)
		if err != nil {
			logger.Error(err.Error())
		}
		if resmeth == "rpi_usb" && !exist {
			// Try toggling usb power
			self._check_restart("enable power")
		}

		if self._canbus_iface != nil {
			//cbid := self._printer.Lookup_object("canbus_ids")
			//nodeid := cbid.Get_nodeid(self._serialport)
			//self.Serial.Connect_canbus(self._serialport, nodeid,
			//	self._canbus_iface)
		} else if strings.HasPrefix(self._serialport, "tcp@") {
			self.Serial.Connect_remote(self._serialport)
		} else if self._baud > 0 {
			// Cheetah boards require RTS to be deasserted
			// else a reset will trigger the built-in bootloader.
			rts := (resmeth != "cheetah")
			self.Serial.Connect_uart(self._serialport, self._baud, rts)
		} else {
			self.Serial.Connect_pipe(self._serialport)
		}
		self._clocksync.Connect(self.Serial)
	}
	logger.Info(self._log_info())
	ppins := self._printer.Lookup_object("pins", object.Sentinel{})
	pin_resolver := ppins.(*PrinterPins).Get_pin_resolver(self._name)
	for cname, value := range self.Get_constants() {
		if strings.HasPrefix(cname, "RESERVE_PINS_") {
			for _, pin := range strings.Split(value.(string), ",") {
				pin_resolver.Reserve_pin(pin, cname[13:])
			}
		}
	}
	self._mcu_freq = self.Get_constant_float("CLOCK_FREQ")
	self._stats_sumsq_base = self.Get_constant_float("STATS_SUMSQ_BASE")
	self._emergency_stop_cmd, _ = self.Lookup_command("emergency_stop", nil)
	reset_cmd := self.Try_lookup_command("reset")
	if reset_cmd != nil {
		self._reset_cmd = reset_cmd.(*CommandWrapper)
	}
	config_reset_cmd := self.Try_lookup_command("config_reset")
	if config_reset_cmd != nil {
		self._config_reset_cmd = config_reset_cmd.(*CommandWrapper)
	}
	ext_only := self._reset_cmd == nil && self._config_reset_cmd == nil
	msgparser := self.Serial.Get_msgparser()
	mbaud := msgparser.Get_constant("SERIAL_BAUD", nil, reflect.Float64)
	if self._restart_method == "" && mbaud == nil && !ext_only {
		self._restart_method = "command"
	}
	if msgparser.Get_constant("CANBUS_BRIDGE", 0, reflect.String) != 0 {
		self._is_mcu_bridge = true
		self._printer.Register_event_handler("project:firmware_restart",
			self._firmware_restart_bridge)
	}
	version, build_versions := msgparser.Get_version_info()
	self._get_status_info["mcu_version"] = version
	self._get_status_info["mcu_build_versions"] = build_versions
	self._get_status_info["mcu_constants"] = msgparser.Get_constants()
	self.Register_response(self._handle_shutdown, "shutdown", nil)
	self.Register_response(self._handle_shutdown, "is_shutdown", nil)
	self.Register_response(self._handle_mcu_stats, "stats", nil)
	return nil
}

// Config creation helpers
func (self *MCU) Setup_pin(pin_type string, pin_params map[string]interface{}) interface{} {
	pcs := map[string]interface{}{"endstop": NewMCU_endstop,
		"digital_out": NewMCU_digital_out, "pwm": NewMCU_pwm, "adc": NewMCU_adc}
	if pcs[pin_type] == nil {
		// raise pins.error("pin type %s not supported on mcu" % (pin_type,))
		//log.Printf("pin type %s not supported on mcu", pin_type)
		return nil
	}
	return pcs[pin_type].(func(*MCU, map[string]interface{}) interface{})(self, pin_params)
}

func (self *MCU) Create_oid() int {
	self.Oid_count += 1
	return self.Oid_count - 1
}

func (self *MCU) Register_config_callback(cb interface{}) {
	self._config_callbacks = append(self._config_callbacks, cb)
}

func (self *MCU) Add_config_cmd(cmd string, is_init bool, on_restart bool) {
	if is_init {
		self._init_cmds = append(self._init_cmds, cmd)
	} else if on_restart {
		self._restart_cmds = append(self._restart_cmds, cmd)
	} else {
		self._config_cmds = append(self._config_cmds, cmd)
	}
}

func (self *MCU) Get_query_slot(oid int) int64 {
	slot := self.Seconds_to_clock(float64(oid) * 0.01)
	t := self.Estimated_print_time(self._reactor.Monotonic()) + 1.5
	return self.Print_time_to_clock(t) + slot
}

func (self *MCU) Register_stepqueue(stepqueue interface{}) {
	self._stepqueues = append(self._stepqueues, stepqueue)
}

func (self *MCU) Request_move_queue_slot() {
	self._reserved_move_slots += 1
}
func (self *MCU) Seconds_to_clock(time float64) int64 {
	return int64(time * self._mcu_freq)
}
func (self *MCU) Get_max_stepper_error() float64 {
	return self._max_stepper_error
}

// Wrapper functions
func (self *MCU) Get_printer() *Printer {
	return self._printer
}
func (self *MCU) Get_name() string {
	return self._name
}
func (self *MCU) Register_response(cb interface{}, msg string, oid interface{}) {
	self.Serial.Register_response(cb, msg, oid)
}
func (self *MCU) Alloc_command_queue() interface{} {
	return self.Serial.Alloc_command_queue()
}
func (self *MCU) Lookup_command(msgformat string, cq interface{}) (*CommandWrapper, error) {
	return NewCommandWrapper(self.Serial, msgformat, cq)
}
func (self *MCU) Lookup_query_command(msgformat string, respformat string, oid int,
	cq interface{}, is_async bool) *CommandQueryWrapper {
	return NewCommandQueryWrapper(self.Serial, msgformat, respformat, oid,
		cq, is_async, self._printer.Command_error)
}
func (self *MCU) Try_lookup_command(msgformat string) interface{} {
	ret, err := self.Lookup_command(msgformat, nil)
	if err != nil {
		logger.Error(err)
		return nil
	}
	return ret
}

func (self *MCU) Lookup_command_tag(msgformat string) interface{} {
	all_msgs := self.Serial.Get_msgparser().Get_messages()
	res := map[string]interface{}{}
	for _, msg := range all_msgs {
		msgs := msg.([]interface{})
		msgtag := msgs[0].(int)
		//msgtype := msgs[1]
		fmt := msgs[2].(string)
		res[fmt] = msgtag
	}

	return res[msgformat]
}
func (self *MCU) Get_enumerations() map[string]interface{} {
	return self.Serial.Get_msgparser().Get_enumerations()
}
func (self *MCU) Get_constants() map[string]interface{} {
	return self.Serial.Get_msgparser().Get_constants()
}
func (self *MCU) Get_constant_float(name string) float64 {
	return self.Serial.Get_msgparser().Get_constant_float(name, nil)
}
func (self *MCU) Print_time_to_clock(print_time float64) int64 {
	return self._clocksync.Print_time_to_clock(print_time)
}
func (self *MCU) Clock_to_print_time(clock int64) float64 {
	return self._clocksync.Clock_to_print_time(clock)
}
func (self *MCU) Estimated_print_time(eventtime float64) float64 {
	return self._clocksync.Estimated_print_time(eventtime)
}
func (self *MCU) Clock32_to_clock64(clock32 int64) int64 {
	return self._clocksync.Clock32_to_clock64(clock32)
}

// Restarts
func (self *MCU) _disconnect(argv []interface{}) error {
	self.Serial.Disconnect()
	chelper.Steppersync_free(self._steppersync)
	self._steppersync = nil
	return nil
}

func (self *MCU) _ready(argv []interface{}) error {
	if self.Is_fileoutput() {
		return nil
	}
	// Check that reported mcu frequency is in range
	mcu_freq := self._mcu_freq
	systime := self._reactor.Monotonic()
	calc_freq := self._clocksync.Get_clock(systime+1) - self._clocksync.Get_clock(systime)
	freq_diff := math.Abs(mcu_freq - float64(calc_freq))
	mcu_freq_mhz := int64(mcu_freq/1000000. + 0.5)
	calc_freq_mhz := int64(float64(calc_freq)/1000000. + 0.5)
	if mcu_freq_mhz != calc_freq_mhz && freq_diff > mcu_freq*0.01 {
		logger.Errorf("MCU %s configured for %dMhz but running at %dMhz!",
			self._name, mcu_freq_mhz, calc_freq_mhz)
	} else {
		logger.Debugf("MCU %s configured for %dMhz running at %dMhz",
			self._name, mcu_freq_mhz, calc_freq_mhz)
	}
	return nil
}

func (self *MCU) _shutdown(argv []interface{}) error {
	force := false
	if argv != nil && len(argv) != 0 {
		force = argv[0].(bool)
	}
	if self._emergency_stop_cmd == nil ||
		(self._is_shutdown && force == false) {
		return nil
	}
	self._emergency_stop_cmd.Send([]int64{}, 0, 0)
	return nil
}

func (self *MCU) _restart_arduino() {
	logger.Debugf("Attempting MCU '%s' reset", self._name)
	self._disconnect([]interface{}{})
	Arduino_reset(self._serialport, self._reactor)
}
func (self *MCU) _restart_cheetah() {
	logger.Debugf("Attempting MCU '%s' Cheetah-style reset", self._name)
	self._disconnect([]interface{}{})
	Cheetah_reset(self._serialport, self._reactor)
}
func (self *MCU) _restart_via_command() {
	if (self._reset_cmd == nil && self._config_reset_cmd == nil) ||
		self._clocksync.Is_active() == false {
		logger.Errorf("Unable to issue reset command on MCU '%s'",
			self._name)
		return
	}
	if self._reset_cmd == nil {
		// Attempt reset via config_reset command
		logger.Debugf("Attempting MCU '%s' config_reset command", self._name)
		self._is_shutdown = true
		self._shutdown([]interface{}{true})
		self._reactor.Pause(self._reactor.Monotonic() + 0.015)
		self._config_reset_cmd.Send([]int64{}, 0, 0)
	} else {
		// Attempt reset via reset command
		logger.Debugf("Attempting MCU '%s' reset command", self._name)
		self._reset_cmd.Send([]int64{}, 0, 0)
	}
	//self._reactor.Pause(self._reactor.Monotonic() + 0.015)
	time.Sleep(15 * time.Millisecond)

	self._disconnect([]interface{}{})
}
func (self *MCU) _restart_rpi_usb() {
	logger.Debugf("Attempting MCU '%s' reset via rpi usb power", self._name)
	self._disconnect([]interface{}{})
	//chelper.Run_hub_ctrl(0)
	self._reactor.Pause(self._reactor.Monotonic() + 2.)
	//chelper.Run_hub_ctrl(1)
}
func (self *MCU) _firmware_restart(argv []interface{}) error {
	var force bool
	if argv != nil {
		force = argv[0].(bool)
	} else {
		force = false
	}
	if self._is_mcu_bridge && force == false {
		return nil
	}
	if self._restart_method == "rpi_usb" {
		self._restart_rpi_usb()
	} else if self._restart_method == "command" {
		self._restart_via_command()
	} else if self._restart_method == "cheetah" {
		self._restart_cheetah()
	} else {
		self._restart_arduino()
	}
	return nil
}

func (self *MCU) _firmware_restart_bridge([]interface{}) error {
	self._firmware_restart([]interface{}{true})
	return nil
}

// # Move queue tracking
func (self *MCU) register_stepqueue(stepqueue interface{}) {
	self._stepqueues = append(self._stepqueues, stepqueue)
}

func (self *MCU) request_move_queue_slot() {
	self._reserved_move_slots += 1
}

func (self *MCU) Register_flush_callback(callback func(float64, int64)) {
	self._flush_callbacks = append(self._flush_callbacks, callback)
}

func (self *MCU) Flush_moves(print_time float64, clear_history_time float64) {
	if self._steppersync == nil {
		return
	}
	clock := self.Print_time_to_clock(print_time)
	if clock < 0 {
		return
	}

	for _, cb := range self._flush_callbacks {
		cb(print_time, clock)
	}
	clear_history_clock := math.Max(0, float64(self.Print_time_to_clock(clear_history_time)))
	ret := chelper.Steppersync_flush(self._steppersync, uint64(clock), uint64(clear_history_clock))
	if ret != 0 {
		logger.Error("Internal error in MCU stepcompress:", self._name)
	}
}
func (self *MCU) Check_active(print_time float64, eventtime float64) {
	if self._steppersync == nil {
		return
	}
	clock := self._clocksync.Calibrate_clock(print_time, eventtime)
	offset := clock[0]
	freq := clock[1]
	chelper.Steppersync_set_time(self._steppersync, offset, freq)
	if self._clocksync.Is_active() || self.Is_fileoutput() ||
		self._is_timeout {
		return
	}
	self._is_timeout = true
	err_str := fmt.Sprintf("Lost communication with MCU %s", self._name)
	logger.Errorf("Timeout with MCU '%s' (eventtime=%f), ERROR:%s",
		self._name, eventtime, err_str)
	self._printer.Invoke_shutdown(err_str)

	gcode := self._printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Respond_info(err_str, true)
}

// Misc external commands
func (self *MCU) Is_fileoutput() bool {
	return self._printer.Get_start_args()["debugoutput"] != nil
}
func (self *MCU) Is_shutdown() bool {
	return self._is_shutdown
}
func (self *MCU) Get_shutdown_clock() int64 {
	return self._shutdown_clock
}

func (self *MCU) Get_status(eventtime float64) map[string]interface{} {
	return sys.DeepCopyMap(self._get_status_info)
}

func (self *MCU) Stats(eventtime float64) (bool, string) {
	load := fmt.Sprintf("mcu_awake=%.03f mcu_task_avg=%.06f mcu_task_stddev=%.06f",
		self._mcu_tick_awake, self._mcu_tick_avg, self._mcu_tick_stddev)
	stats := fmt.Sprintf("%s %s %s", load, self.Serial.stats(eventtime), self._clocksync.Stats(eventtime))
	parts := map[string]string{}
	for _, s := range strings.Split(stats, " ") {
		spl := strings.Split(s, "=")
		parts[spl[0]] = spl[1]
	}
	last_stats := map[string]interface{}{}
	for k, v := range parts {
		if strings.Contains(v, ".") {
			last_stats[k], _ = strconv.ParseFloat(v, 64)
		} else {
			last_stats[k], _ = strconv.Atoi(v)
		}
	}
	self._get_status_info["last_stats"] = last_stats
	return true, fmt.Sprintf("%s: %s", self._name, stats)
}
func Add_printer_objects_mcu(config *ConfigWrapper) {
	printer := config.Get_printer()
	reactor := printer.Get_reactor()
	mainsync := NewClockSync(reactor)
	printer.Add_object("mcu", NewMCU(config.Getsection("mcu"), mainsync))
	for _, s := range config.Get_prefix_sections("mcu ") {
		printer.Add_object(s.Section, NewMCU(
			s, NewSecondarySync(reactor, mainsync)))
	}
}

func Get_printer_mcu(printer *Printer, name string) *MCU {
	if name == "mcu" {
		return printer.Lookup_object(name, object.Sentinel{}).(*MCU)
	}

	return printer.Lookup_object("mcu "+name, object.Sentinel{}).(*MCU)
}
