package project

/*
#include <stdio.h>
#cgo CFLAGS: -I${SRCDIR}/chelper

#include "stepcompress.h"
*/
import "C"
import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"k3c/common/value"
	"k3c/project/chelper"
	"math"
	"reflect"
	"sort"
	"strings"
)

const API_UPDATE_INTERVAL = 0.500

// Helper to periodically transmit data to a set of API clients
type APIDumpHelper struct {
	printer         *Printer
	data_cb         func(eventtime float64) map[string]interface{}
	startstop_cb    func(is_start bool)
	update_interval float64
	is_started      bool
	update_timer    *ReactorTimer
	clients         map[IConnectClient]map[string]interface{}
}

func NewAPIDumpHelper(printer *Printer, data_cb func(float64) map[string]interface{}, startstop_cb func(bool),
	update_interval float64) *APIDumpHelper {
	self := new(APIDumpHelper)

	self.printer = printer
	self.data_cb = data_cb

	if value.IsNone(startstop_cb) {
		startstop_cb = func(is_start bool) {}
	}
	self.startstop_cb = startstop_cb
	self.is_started = false
	self.update_interval = update_interval
	self.update_timer = nil
	self.clients = make(map[IConnectClient]map[string]interface{})
	return self
}

func (self *APIDumpHelper) _stop() float64 {
	self.clients = make(map[IConnectClient]map[string]interface{})
	reactor := self.printer.Get_reactor()
	reactor.Unregister_timer(self.update_timer)
	self.update_timer = nil
	if value.Not(self.is_started) {
		return constants.NEVER
	}
	self.tryCatchExcept()
	self.is_started = false

	if len(self.clients) > 0 {
		self._start()
	}
	return constants.NEVER
}

func (self *APIDumpHelper) tryCatchExcept() {
	defer func() {
		if r := recover(); r != nil {
			logger.Errorf("API Dump Helper stop callback error: %v", r)
		}
		self.clients = make(map[IConnectClient]map[string]interface{})
	}()
	self.startstop_cb(false)
}

func (self *APIDumpHelper) _start() {
	if self.is_started {
		return
	}

	self.is_started = true
	func() {
		defer func() {
			if r := recover(); r != nil {
				logger.Errorf("API Dump Helper start callback error: %v", r)
				self.is_started = false
				self.clients = make(map[IConnectClient]map[string]interface{})
				panic(r)
			}
		}()
		self.startstop_cb(true)
	}()

	reactor := self.printer.Get_reactor()
	systime := reactor.Monotonic()
	waketime := systime + self.update_interval
	self.update_timer = reactor.Register_timer(self._update, waketime)
}

func (self *APIDumpHelper) add_client(web_request *WebRequest) {
	cconn := web_request.Get_client_connection()
	template := web_request.Get_dict("response_template", nil)
	self.clients[cconn] = template
	self._start()
}

func (self *APIDumpHelper) add_internal_client() ICcon {
	cconn := NewInternalDumpClient()
	self.clients[cconn] = make(map[string]interface{})
	self._start()
	return cconn
}

func (self *APIDumpHelper) _update(eventtime float64) float64 {
	msg, updated, r := self.tryCatchExceptForDataCb(eventtime)
	if r != nil {
		logger.Errorf("API Dump Helper data callback error")
		return updated
	}

	if msg == nil {
		return eventtime + self.update_interval
	}
	for cconn, template := range self.clients {
		if cconn.Is_closed() {
			delete(self.clients, cconn)
			if len(self.clients) == 0 {
				return self._stop()
			}
			continue
		}
		tmp := make(map[string]interface{})
		for k, v := range template {
			tmp[k] = v
		}

		tmp["params"] = msg
		cconn.Send(tmp)
	}

	return eventtime + self.update_interval
}

func (self *APIDumpHelper) tryCatchExceptForDataCb(eventtime float64) (msg interface{}, updated float64, r interface{}) {
	defer func() {
		if r = recover(); r != nil {
			logger.Error("API Dump Helper data callback error", r)
			updated = self._stop()
		}
	}()
	msg = self.data_cb(eventtime)
	return
}

// An "internal webhooks" wrapper for using APIDumpHelper internally
type InternalDumpClient struct {
	msgs    []map[string]map[string]interface{}
	is_done bool
}

type IConnectClient interface {
	Is_closed() bool
	Send(msg interface{})
}

func NewInternalDumpClient() *InternalDumpClient {
	self := new(InternalDumpClient)
	self.msgs = make([]map[string]map[string]interface{}, 0)
	self.is_done = false
	return self
}

func (self *InternalDumpClient) Get_messages() []map[string]map[string]interface{} {
	return self.msgs
}

func (self *InternalDumpClient) Finalize() {
	self.is_done = true
}

func (self *InternalDumpClient) Is_closed() bool {
	return self.is_done
}

func (self *InternalDumpClient) Send(msg interface{}) {
	var _msg map[string]map[string]interface{}
	_msg = make(map[string]map[string]interface{})
	for k, v := range msg.(map[string]interface{}) {
		_msg[k] = v.(map[string]interface{})
	}
	self.msgs = append(self.msgs, _msg)
	if len(self.msgs) >= 10000 {
		// Avoid filling up memory with too many samples
		self.Finalize()
	}
}

// Extract stepper queue_step messages

type DumpStepper struct {
	printer        *Printer
	mcu_stepper    *MCU_stepper
	last_api_clock uint64
	api_dump       *APIDumpHelper
}

func NewDumpStepper(printer *Printer, mcu_stepper *MCU_stepper) *DumpStepper {
	self := new(DumpStepper)
	self.printer = printer
	self.mcu_stepper = mcu_stepper
	self.last_api_clock = 0
	self.api_dump = NewAPIDumpHelper(printer, self._api_update, nil, API_UPDATE_INTERVAL)
	wh := MustLookupWebhooks(self.printer)
	wh.Register_mux_endpoint("motion_report/dump_stepper", "name",
		mcu_stepper.Get_name(false), self._add_api_client)
	return self
}

type step_result struct {
	count int
	data  []interface{}
}

func (self *DumpStepper) get_step_queue(start_clock, end_clock uint64) ([]interface{}, []step_result) {
	mcu_stepper := self.mcu_stepper
	res := make([]step_result, 0)

	for {
		data, count := mcu_stepper.Dump_steps(128, start_clock, end_clock)
		if count == 0 {
			break
		}

		res = append(res, step_result{
			data:  data,
			count: count,
		})
		if count < len(data) {
			break
		}

		end_clock = uint64(data[count-1].(C.struct_pull_history_steps).first_clock)
	}

	for left, right := 0, len(res)-1; left < right; left, right = left+1, right-1 {
		res[left], res[right] = res[right], res[left]
	}
	var step = make([]interface{}, 0)
	for _, res := range res {
		for i := res.count - 1; i <= 0; i-- {
			step = append(step, res.data[i])
		}
	}
	return step, res
}

func (self *DumpStepper) log_steps(data []interface{}) {
	if len(data) == 0 {
		return
	}
	var out = []string{}
	out = append(out, fmt.Sprintf("Dumping stepper '%s' (%s) %d queue_step:", self.mcu_stepper.Get_name(false),
		self.mcu_stepper.Get_mcu().Get_name(), len(data)))
	for i, si := range data {
		s := si.(C.struct_pull_history_steps)
		out = append(out, fmt.Sprintf("queue_step %d: t=%d p=%d i=%d c=%d a=%d",
			i, s.first_clock, s.start_position, s.interval,
			s.step_count, s.add))
	}
	logger.Debug(strings.Join(out, "\n"))
}

func (self *DumpStepper) _api_update(eventime float64) map[string]interface{} {
	data, _ := self.get_step_queue(self.last_api_clock, 1<<63)
	if len(data) == 0 {
		return map[string]interface{}{}
	}
	clock_to_print_time := self.mcu_stepper.Get_mcu().Clock_to_print_time
	first := data[0].(C.struct_pull_history_steps)
	first_clock := first.first_clock
	first_time := clock_to_print_time(int64(first_clock))
	self.last_api_clock = uint64(data[len(data)-1].(C.struct_pull_history_steps).last_clock)
	last_clock := int64(self.last_api_clock)
	last_time := clock_to_print_time(last_clock)

	mcu_pos := int(first.start_position)
	start_position := self.mcu_stepper.Mcu_to_commanded_position(mcu_pos)
	step_dist := self.mcu_stepper.Get_step_dist()
	v1, _ := self.mcu_stepper.Get_dir_inverted()
	if v1 != 0 {
		step_dist = -step_dist
	}

	var d [][]int
	for _, si := range data {
		s := si.(C.struct_pull_history_steps)
		d = append(d, []int{
			int(s.interval),
			int(s.step_count),
			int(s.add),
		})
	}
	return map[string]interface{}{
		"data": d, "start_position": start_position,
		"start_mcu_position": mcu_pos, "step_distance": step_dist,
		"first_clock": first_clock, "first_step_time": first_time,
		"last_clock": last_clock, "last_step_time": last_time,
	}
}

func (self *DumpStepper) _add_api_client(web_request *WebRequest) {
	self.api_dump.add_client(web_request)
	hdr := []string{"interval", "count", "add"}
	web_request.Send(map[string]interface{}{"header": hdr})
}

const NEVER_TIME = 9999999999999999.

// Extract trapezoidal motion queue (trapq)
type DumpTrapQ struct {
	printer      *Printer
	name         string
	trapq        interface{}
	last_api_msg []interface{}
	api_dump     *APIDumpHelper
}

func NewDumpTrapQ(printer *Printer, name string, trapq interface{}) *DumpTrapQ {
	self := new(DumpTrapQ)
	self.printer = printer
	self.name = name
	self.trapq = trapq
	self.last_api_msg = []interface{}{0, 0}
	self.api_dump = NewAPIDumpHelper(printer, self._api_update, nil, API_UPDATE_INTERVAL)

	wh := MustLookupWebhooks(self.printer)
	wh.Register_mux_endpoint("motion_report/dump_trapq", "name", name,
		self._add_api_client)
	return self
}

type trapq_result struct {
	count int
	data  []interface{}
}

func (self *DumpTrapQ) extract_trapq(start_time, end_time float64) ([]interface{}, []trapq_result) {
	res := make([]trapq_result, 0)
	for {
		//data := make([]C.struct_pull_move, 128)
		data := chelper.Pull_move_alloc(128)
		count := chelper.Trapq_extract_old(self.trapq, data, len(data), start_time, end_time)
		if count == 0 {
			break
		}

		_data := make([]interface{}, 0, len(data))
		for _, d := range data {
			_data = append(_data, d)
		}
		res = append(res, trapq_result{
			data:  _data,
			count: count,
		})

		if count < len(data) {
			break
		}
		end_time = float64(data[count-1].Print_time)
	}

	for left, right := 0, len(res)-1; left < right; left, right = left+1, right-1 {
		res[left], res[right] = res[right], res[left]
	}
	var trapq = make([]interface{}, 0)
	for _, res := range res {
		for i := res.count - 1; i <= 0; i-- {
			trapq = append(trapq, res.data[i])
		}
	}
	return trapq, res
}

func (self *DumpTrapQ) log_trapq(data []interface{}) {
	if len(data) == 0 {
		return
	}
	var out = []string{}
	out = append(out, fmt.Sprintf("Dumping trapq '%s' %d moves:", self.name, len(data)))
	for i, mi := range data {
		m := mi.(chelper.CStruct_pull_move)
		out = append(out, fmt.Sprintf("move %d: pt=%.6f mt=%.6f sv=%.6f a=%.6f"+
			" sp=(%.6f,%.6f,%.6f) ar=(%.6f,%.6f,%.6f)",
			i, m.Print_time, m.Move_t, m.Start_v, m.Accel,
			m.Start_x, m.Start_y, m.Start_z, m.X_r, m.Y_r, m.Z_r))
	}
	logger.Debug(strings.Join(out, "\n"))
}

func (self *DumpTrapQ) get_trapq_position(print_time float64) ([]float64, float64) {
	data := chelper.Pull_move_alloc(1)
	//data := make([]C.struct_pull_move, 1)
	count := chelper.Trapq_extract_old(self.trapq, data, 1, 0., print_time)
	if count == 0 {
		return nil, -1
	}
	move := data[0]
	move_time := math.Max(0., math.Min(float64(move.Move_t), print_time-float64(move.Print_time)))
	dist := (float64(move.Start_v) + .5*float64(move.Accel)*float64(move_time)) * move_time
	pos := []float64{float64(move.Start_x) + float64(move.X_r)*dist, float64(move.Start_y) + float64(move.Y_r)*dist, float64(move.Start_z) + float64(move.Z_r)*dist}
	velocity := float64(move.Start_v) + float64(move.Accel)*move_time
	return pos, velocity
}

func (self *DumpTrapQ) _api_update(eventime float64) map[string]interface{} {
	qtime := self.last_api_msg[0].(float64) + math.Min(self.last_api_msg[1].(float64), 0.100)
	data, _ := self.extract_trapq(qtime, NEVER_TIME)

	/**

		 d = [(m.print_time, m.move_t, m.start_v, m.accel,
	              (m.start_x, m.start_y, m.start_z), (m.x_r, m.y_r, m.z_r))
	             for m in data]
	*/
	var d = make([][]interface{}, 0)
	for _, mi := range data {
		m := mi.(chelper.CStruct_pull_move)
		d = append(d, []interface{}{
			m.Print_time,
			m.Move_t,
			m.Start_v,
			m.Accel,
			[]float64{float64(m.Start_x), float64(m.Start_y), float64(m.Start_z)},
			[]float64{float64(m.X_r), float64(m.Y_r), float64(m.Z_r)},
		})
	}

	if len(d) > 0 && interfaceSliceEqual(d[0], self.last_api_msg) {
		d = d[1:]
	}

	if len(d) == 0 {
		return map[string]interface{}{}
	}
	self.last_api_msg = d[len(d)-1]
	return map[string]interface{}{
		"data": d,
	}
}

func (self *DumpTrapQ) _add_api_client(web_request *WebRequest) {
	self.api_dump.add_client(web_request)
	hdr := []string{"time", "duration", "start_velocity", "acceleration",
		"start_position", "direction"}
	web_request.Send(map[string]interface{}{"header": hdr})
}

func interfaceSliceEqual(v1 []interface{}, v2 []interface{}) bool {
	if len(v1) != len(v2) {
		return false
	}

	for i, v := range v1 {
		if !reflect.DeepEqual(v, v2[i]) {
			return false
		}
	}
	return true
}

const STATUS_REFRESH_TIME = 0.250

type PrinterMotionReport struct {
	printer          *Printer
	steppers         map[string]interface{}
	trapqs           map[string]interface{}
	next_status_time float64
	last_status      map[string]interface{}
}

func NewPrinterMotionReport(config *ConfigWrapper) *PrinterMotionReport {
	self := new(PrinterMotionReport)
	self.printer = config.Get_printer()
	self.steppers = make(map[string]interface{})
	self.trapqs = make(map[string]interface{})
	// get_status information
	self.next_status_time = 0.
	//gcode := MustLookupGcode(self.printer)
	self.last_status = map[string]interface{}{
		"live_position": []float64{0., 0., 0., 0.},
		"live_velocity": 0., "live_extruder_velocity": 0.,
		"steppers": []interface{}{}, "trapq": []interface{}{},
	}

	// Register handlers
	self.printer.Register_event_handler("project:connect", self._connect)
	self.printer.Register_event_handler("project:shutdown", self._shutdown)

	return self
}

func (self *PrinterMotionReport) Register_stepper(config *ConfigWrapper, mcu_stepper *MCU_stepper) {
	ds := NewDumpStepper(self.printer, mcu_stepper)
	self.steppers[mcu_stepper.Get_name(false)] = ds
}

type IExtruderGetTrapq interface {
	Get_trapq() interface{}
}

func (self *PrinterMotionReport) _connect(argv []interface{}) error {
	// Lookup toolhead trapq
	toolhead := MustLookupToolhead(self.printer)
	trapq := toolhead.Get_trapq()
	self.trapqs["toolhead"] = NewDumpTrapQ(self.printer, "toolhead", trapq)

	//for i := range iterator.RangeInt(99) {
	for i := 0; i < 99; i++ {
		ename := fmt.Sprintf("extruder%d", i)
		if ename == "extruder0" {
			ename = "extruder"
		}

		extruder := self.printer.Lookup_object(ename, value.None)
		if value.IsNone(extruder) {
			break
		}

		etrapq := extruder.(IExtruderGetTrapq).Get_trapq()
		self.trapqs[ename] = NewDumpTrapQ(self.printer, ename, etrapq)
	}
	// Populate 'trapq' and 'steppers' in get_status result
	steppers := str.MapStringKeys(self.steppers)
	sort.Strings(steppers)
	self.last_status["steppers"] = steppers
	trapqs := str.MapStringKeys(self.trapqs)
	sort.Strings(trapqs)
	self.last_status["trapq"] = trapqs
	return nil
}

// Shutdown handling
func (self *PrinterMotionReport) _dump_shutdown(argv interface{}) interface{} {
	// Log stepper queue_steps on mcu that started shutdown (if any)
	var shutdown_time = NEVER_TIME

	for _, dstepper := range self.steppers {
		mcu := dstepper.(*DumpStepper).mcu_stepper.Get_mcu()
		sc := mcu.Get_shutdown_clock()
		if sc == 0 {
			continue
		}

		shutdown_time = math.Min(shutdown_time, mcu.Clock_to_print_time(sc))
		clock_100ms := mcu.Seconds_to_clock(0.100)
		start_clock := maths.Max64(0, sc-clock_100ms)
		end_clock := sc + clock_100ms
		data, _ := dstepper.(*DumpStepper).get_step_queue(uint64(start_clock), uint64(end_clock))
		dstepper.(*DumpStepper).log_steps(data)
	}
	if shutdown_time >= NEVER_TIME {
		return nil
	}

	// Log trapqs around time of shutdown
	for _, dtrapq := range self.trapqs {
		data, _ := dtrapq.(*DumpTrapQ).extract_trapq(shutdown_time-.100,
			shutdown_time+.100)
		dtrapq.(*DumpTrapQ).log_trapq(data)
	}

	// Log estimated toolhead position at time of shutdown
	dtrapq := self.trapqs["toolhead"]
	if value.IsNone(dtrapq) {
		return nil
	}

	pos, velocity := dtrapq.(*DumpTrapQ).get_trapq_position(shutdown_time)
	_ = velocity
	if value.IsNotNone(pos) {
		logger.Debugf("Requested toolhead position at shutdown time %.6f: %v",
			shutdown_time, pos)
	}
	return nil
}

func (self *PrinterMotionReport) _shutdown(argv []interface{}) error {
	self.printer.Get_reactor().Register_callback(self._dump_shutdown, constants.NOW)
	return nil
}

// Status reporting
func (self *PrinterMotionReport) get_status(eventtime float64) map[string]interface{} {
	if eventtime < self.next_status_time || len(self.trapqs) == 0 {
		return self.last_status
	}
	self.next_status_time = eventtime + STATUS_REFRESH_TIME
	xyzpos := []float64{0., 0., 0.}
	epos := []float64{0.}
	xyzvelocity := 0.
	evelocity := 0.
	// Calculate current requested toolhead position
	mcu := MustLookupMCU(self.printer, "mcu")
	print_time := mcu.Estimated_print_time(eventtime)
	pos, velocity := self.trapqs["toolhead"].(*DumpTrapQ).get_trapq_position(print_time)
	if value.IsNotNone(pos) {
		xyzpos = pos[:3]
		xyzvelocity = velocity
	}
	// Calculate requested position of currently active extruder
	toolhead := MustLookupToolhead(self.printer)
	ehandler := self.trapqs[toolhead.Get_extruder().Get_name()]
	if value.IsNotNone(ehandler) {
		pos, velocity = ehandler.(*DumpTrapQ).get_trapq_position(print_time)
		if value.IsNotNone(pos) {
			epos = []float64{pos[0]}
			evelocity = velocity
		}
	}

	// Report status
	self.last_status["live_position"] = append(xyzpos, epos...)
	self.last_status["live_velocity"] = xyzvelocity
	self.last_status["live_extruder_velocity"] = evelocity
	return self.last_status
}

func Load_config_PrinterMotionReport(config *ConfigWrapper) interface{} {
	return NewPrinterMotionReport(config)
}
