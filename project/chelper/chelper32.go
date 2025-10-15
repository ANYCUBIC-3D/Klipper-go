//go:build arm || 386
// +build arm 386

package chelper

/*

#cgo CFLAGS: -I${SRCDIR}
#cgo LDFLAGS: -L${SRCDIR} -lc_helper -lm
#include "pyhelper.h"
#include "serialqueue.h"
#include "stepcompress.h"
#include "itersolve.h"
#include "trapq.h"
#include <stdlib.h>
#include <stdio.h>

struct trdispatch {
    struct list_head tdm_list;

    pthread_mutex_t lock; // protects variables below
    uint32_t is_active, can_trigger, dispatch_reason;
};

struct trdispatch_mcu {
    struct fastreader fr;
    struct trdispatch *td;
    struct list_node node;
    struct serialqueue *sq;
    struct command_queue *cq;
    uint32_t trsync_oid, set_timeout_msgtag, trigger_msgtag;

    // Remaining fields protected by trdispatch lock
    uint64_t last_status_clock, expire_clock;
    uint64_t expire_ticks, min_extend_ticks;
    struct clock_estimate ce;
};
void trdispatch_start(struct trdispatch *td, uint32_t dispatch_reason);
void trdispatch_stop(struct trdispatch *td);
struct trdispatch *trdispatch_alloc(void);
struct trdispatch_mcu *trdispatch_mcu_alloc(struct trdispatch *td
        , struct serialqueue *sq, struct command_queue *cq, uint32_t trsync_oid
        , uint32_t set_timeout_msgtag, uint32_t trigger_msgtag
        , uint32_t state_msgtag);
void trdispatch_mcu_setup(struct trdispatch_mcu *tdm
        , uint64_t last_status_clock, uint64_t expire_clock
        , uint64_t expire_ticks, uint64_t min_extend_ticks);


int input_shaper_set_shaper_params(struct stepper_kinematics *sk, char axis ,int n, double a[], double t[]);
double input_shaper_get_step_generation_window(int n, double a[], double t[]);
struct stepper_kinematics *input_shaper_alloc(void);
int input_shaper_set_sk(struct stepper_kinematics *sk
                    , struct stepper_kinematics *orig_sk);

struct stepper_kinematics *extruder_stepper_alloc(void);
void extruder_set_pressure_advance(struct stepper_kinematics *sk
        , double pressure_advance, double smooth_time);
struct stepper_kinematics *cartesian_stepper_alloc(char axis);
struct stepper_kinematics *cartesian_reverse_stepper_alloc(char axis);
struct stepper_kinematics *corexy_stepper_alloc(char type);
*/
import "C"
import (
	"fmt"
	"io/ioutil"
	"k3c/common/utils/file"
	"log"
	"math"
	"os/exec"
	"unsafe"
)

const GCC_CMD = "gcc"

var NULL = C.NULL

type FFI_lib struct {
}

// type FFI_main struct{
//
// }
func (self *FFI_lib) Get_monotonic() float64 {
	//log.Print("${SRCDIR}")
	return float64(C.get_monotonic())
	//return 0
}

func Free(p interface{}) {
	if p != nil {
		C.free(p.(unsafe.Pointer))
	}
}

// Update filenames to an absolute path
func get_abs_files(srcdir string, filelist []string) []string {
	//return [os.path.join(srcdir, fname) for fname in filelist]
	return nil
}

// Return the list of file modification times
func get_mtimes(filelist []string) []float64 {
	out := []float64{}
	for _, filename := range filelist {

		t, err := file.GetMtime(filename)
		if err != nil {
			continue
		}

		out = append(out, t)
	}
	return out
}

// Check if the code needs to be compiled
func check_build_code(sources []string, target string) bool {
	src_times := get_mtimes(sources)
	obj_times := get_mtimes([]string{target})
	if len(obj_times) != 0 {
		srcT := 0.
		for _, st := range src_times {
			srcT = math.Max(srcT, st)
		}
		objT := 999999999999.
		for _, obj := range obj_times {
			objT = math.Min(objT, obj)
		}
		return srcT > objT
	}
	return false
}

// Check if the current gcc version supports a particular command-line option
func check_gcc_option(option string) bool {
	cmd := fmt.Sprintf("%s %s -S -o /dev/null -xc /dev/null > /dev/null 2>&1", GCC_CMD, option)
	res := exec.Command(cmd)
	stdout, err := res.StdoutPipe()
	if err != nil {
		return false
	}
	defer stdout.Close()

	if err := res.Start(); err != nil {
		return false
	}

	if opBytes, err := ioutil.ReadAll(stdout); err != nil {
		return false
	} else {
		log.Println(string(opBytes))
	}
	return true
}

// var main *FFI_main
var lib *FFI_lib

func Get_ffi() *FFI_lib {
	if lib == nil {
		lib = &FFI_lib{}

	}
	return lib
}

//	func Get_ffi() (*FFI_main, *FFI_lib) {
//		if lib == nil {
//			lib = &FFI_lib{}
//			main = &FFI_main{}
//		}
//		return main, lib
//	}
func New_pull_queue_message() *C.struct_pull_queue_message {
	return &C.struct_pull_queue_message{}
}

func Serialqueue_pull(serialqueue interface{}, response interface{}) {
	if serialqueue == nil || response == nil {
		return
	}
	C.serialqueue_pull(serialqueue.(*C.struct_serialqueue), response.(*C.struct_pull_queue_message))
}

// malloc c struct serialqueue
func Serialqueue_alloc(fileno uintptr, serial_fd_type byte, client_id int) *C.struct_serialqueue {
	serialqueue := C.serialqueue_alloc(C.int(fileno), C.char(serial_fd_type), C.int(client_id))
	return serialqueue
}

// free c struct serialqueue
func Serialqueue_free(serialqueue interface{}) {
	C.serialqueue_free(serialqueue.(*C.struct_serialqueue))

}

// malloc c struct command_queue
func Serialqueue_alloc_commandqueue() *C.struct_command_queue {
	commandqueue := C.serialqueue_alloc_commandqueue()
	return commandqueue
}

// free c struct command_queue
func Serialqueue_free_commandqueue(commandqueue interface{}) {
	if commandqueue != nil {
		C.serialqueue_free_commandqueue(commandqueue.(*C.struct_command_queue))
	}
}

func Serialqueue_exit(serialqueue interface{}) {
	C.serialqueue_exit(serialqueue.(*C.struct_serialqueue))
}

func Serialqueue_set_wire_frequency(serialqueue interface{}, wire_freq float64) {
	C.serialqueue_set_wire_frequency(serialqueue.(*C.struct_serialqueue), C.double(wire_freq))

}
func Serialqueue_set_receive_window(serialqueue interface{}, receive_window int) {
	C.serialqueue_set_receive_window(serialqueue.(*C.struct_serialqueue), C.int(receive_window))
}
func Serialqueue_set_clock_est(serialqueue interface{}, freq float64, conv_time float64, conv_clock uint64, last_clock uint64) {
	C.serialqueue_set_clock_est(
		serialqueue.(*C.struct_serialqueue), C.double(freq), C.double(conv_time), C.ulonglong(conv_clock), C.ulonglong(last_clock))
}
func Serialqueue_send(serialqueue interface{}, cmd_queue interface{}, cmd []int, count int, minclock, reqclock, last int64) {
	if serialqueue == nil {
		return
	}
	if cmd_queue == nil {
		return
	}
	c := []uint8{}
	for _, k := range cmd {
		c = append(c, uint8(k))
	}

	C.serialqueue_send(
		serialqueue.(*C.struct_serialqueue), cmd_queue.(*C.struct_command_queue), (*C.uchar)(unsafe.Pointer(&c[0])), C.int(count), C.ulonglong(minclock), C.ulonglong(reqclock), C.ulonglong(last))
}
func Serialqueue_get_stats(serialqueue interface{}, stats_buf []byte) {
	C.serialqueue_get_stats(serialqueue.(*C.struct_serialqueue), (*C.char)(unsafe.Pointer(&stats_buf[0])), C.int(len(stats_buf)))
}

// malloc c struct command_queue
func Steppersync_alloc(serialqueue interface{}, sc_list interface{}, sc_num int, move_num int) *C.struct_steppersync {
	var rows []*C.struct_stepcompress
	if sc_num < 1 {
		rows = make([]*C.struct_stepcompress, 1)
	} else {
		rows = []*C.struct_stepcompress{}
		for _, sc := range sc_list.([]interface{}) {
			rows = append(rows, sc.(*C.struct_stepcompress))
		}
	}
	commandqueue := C.steppersync_alloc(serialqueue.(*C.struct_serialqueue), (**C.struct_stepcompress)(unsafe.Pointer(&rows[0])), C.int(sc_num), C.int(move_num))
	return commandqueue
}

// free c struct command_queue
func Steppersync_free(syc interface{}) {
	if syc != nil {
		C.steppersync_free(syc.(*C.struct_steppersync))
	}
}

func Steppersync_set_time(syc interface{}, time_offset float64, mcu_freq float64) {
	C.steppersync_set_time(syc.(*C.struct_steppersync), C.double(time_offset), C.double(mcu_freq))
}

func Steppersync_flush(syc interface{}, move_clock uint64, clear_history_clock uint64) int {
	return int(C.steppersync_flush(syc.(*C.struct_steppersync), C.uint64_t(move_clock), C.uint64_t(clear_history_clock)))
}

// malloc c struct command_queue
func Trdispatch_mcu_alloc(td interface{}, sq interface{},
	cq interface{}, trsync_oid int, set_timeout_msgtag uint32,
	trigger_msgtag uint32, state_msgtag uint32) *C.struct_trdispatch_mcu {
	trdispatch := C.trdispatch_mcu_alloc(td.(*C.struct_trdispatch), sq.(*C.struct_serialqueue),
		cq.(*C.struct_command_queue), C.uint(trsync_oid), C.uint(set_timeout_msgtag),
		C.uint(trigger_msgtag), C.uint(state_msgtag))
	return trdispatch
}

func Trdispatch_alloc() *C.struct_trdispatch {
	return C.trdispatch_alloc()
}

func Trdispatch_start(td interface{}, dispatch_reason uint32) {
	C.trdispatch_start(td.(*C.struct_trdispatch), C.uint(dispatch_reason))
}

func Trdispatch_stop(td interface{}) {
	C.trdispatch_stop(td.(*C.struct_trdispatch))
}

func Trdispatch_mcu_setup(tdm interface{}, last_status_clock uint64,
	expire_clock uint64, expire_ticks uint64, min_extend_ticks uint64) {

	C.trdispatch_mcu_setup(tdm.(*C.struct_trdispatch_mcu), C.ulonglong(last_status_clock),
		C.ulonglong(expire_clock), C.ulonglong(expire_ticks),
		C.ulonglong(min_extend_ticks))
}
func Stepcompress_alloc(oid uint32) *C.struct_stepcompress {
	return C.stepcompress_alloc(C.uint(oid))
}
func Stepcompress_free(p interface{}) {
	if p != nil {
		C.stepcompress_free(p.(*C.struct_stepcompress))
	}
}
func Stepcompress_fill(sc interface{}, max_error uint32, queue_step_msgtag int32, set_next_step_dir_msgtag int32) {
	C.stepcompress_fill(sc.(*C.struct_stepcompress), C.uint(max_error), C.int(queue_step_msgtag), C.int(set_next_step_dir_msgtag))
}
func Stepcompress_set_invert_sdir(sc interface{}, invert_sdir uint32) {
	C.stepcompress_set_invert_sdir(sc.(*C.struct_stepcompress), C.uint(invert_sdir))
}

func Stepcompress_find_past_position(sc interface{}, clock uint64) int64 {
	return int64(C.stepcompress_find_past_position(sc.(*C.struct_stepcompress), C.ulonglong(clock)))
}

func Stepcompress_extract_old(sc interface{}, p interface{}, max int, start_clock uint64, end_clock uint64) int {
	return int(C.stepcompress_extract_old(sc.(*C.struct_stepcompress), p.(*C.struct_pull_history_steps), C.int(max), C.ulonglong(start_clock), C.ulonglong(end_clock)))
}

func Stepcompress_reset(sc interface{}, last_step_clock uint64) int {
	return int(C.stepcompress_reset(sc.(*C.struct_stepcompress), C.ulonglong(last_step_clock)))
}
func Stepcompress_queue_msg(sc interface{}, data interface{}, len int) int {
	dat := data.([]uint32)
	return int(C.stepcompress_queue_msg(sc.(*C.struct_stepcompress), (*C.uint)(unsafe.Pointer(&dat[0])), C.int(len)))
}
func Stepcompress_set_last_position(sc interface{}, clock uint64, last_position int64) int {
	return int(C.stepcompress_set_last_position(sc.(*C.struct_stepcompress), C.ulonglong(clock), C.longlong(last_position)))
}
func New_pull_history_steps() []C.struct_pull_history_steps {
	return []C.struct_pull_history_steps{}
}

func Itersolve_calc_position_from_coord(sk interface{}, x float64, y float64, z float64) float64 {
	return float64(C.itersolve_calc_position_from_coord(sk.(*C.struct_stepper_kinematics), C.double(x), C.double(y), C.double(z)))
}

func Itersolve_set_position(sk interface{}, x float64, y float64, z float64) {
	C.itersolve_set_position(sk.(*C.struct_stepper_kinematics), C.double(x), C.double(y), C.double(z))
}

func Itersolve_get_commanded_pos(sk interface{}) float64 {
	return float64(C.itersolve_get_commanded_pos(sk.(*C.struct_stepper_kinematics)))
}

func Itersolve_set_stepcompress(sk interface{}, sc interface{}, step_dist float64) {
	C.itersolve_set_stepcompress(sk.(*C.struct_stepper_kinematics), sc.(*C.struct_stepcompress), C.double(step_dist))
}

func Itersolve_set_trapq(sk interface{}, tq interface{}) {
	if tq == nil {
		tq1 := new(C.struct_trapq)
		C.itersolve_set_trapq(sk.(*C.struct_stepper_kinematics), tq1)
	} else {
		C.itersolve_set_trapq(sk.(*C.struct_stepper_kinematics), tq.(*C.struct_trapq))
	}
}

func Itersolve_is_active_axis(sk interface{}, axis byte) int32 {
	return int32(C.itersolve_is_active_axis(sk.(*C.struct_stepper_kinematics), C.char(axis)))
}
func Itersolve_generate_steps(sk interface{}, flush_time float64) int32 {
	return int32(C.itersolve_generate_steps(sk.(*C.struct_stepper_kinematics), C.double(flush_time)))
}

func Itersolve_check_active(sk interface{}, flush_time float64) float64 {
	return float64(C.itersolve_check_active(sk.(*C.struct_stepper_kinematics), C.double(flush_time)))
}

func Trapq_alloc() *C.struct_trapq {
	return C.trapq_alloc()
}

func Trapq_free(tq interface{}) {
	if tq != nil {
		C.trapq_free(tq.(*C.struct_trapq))
	}
}

func Trapq_append(tq interface{}, print_time float64,
	accel_t float64, cruise_t float64, decel_t float64,
	start_pos_x float64, start_pos_y float64, start_pos_z float64,
	axes_r_x float64, axes_r_y float64, axes_r_z float64,
	start_v float64, cruise_v float64, accel float64) {

	C.trapq_append(tq.(*C.struct_trapq), C.double(print_time), C.double(accel_t),
		C.double(cruise_t), C.double(decel_t), C.double(start_pos_x),
		C.double(start_pos_y), C.double(start_pos_z),
		C.double(axes_r_x), C.double(axes_r_y),
		C.double(axes_r_z), C.double(start_v), C.double(cruise_v), C.double(accel))
}

func Trapq_finalize_moves(tq interface{}, print_time float64, clear_history_time float64) {
	C.trapq_finalize_moves(tq.(*C.struct_trapq), C.double(print_time), C.double(clear_history_time))
}

func Trapq_set_position(tq interface{}, print_time float64, pos_x float64, pos_y float64, pos_z float64) {
	C.trapq_set_position(tq.(*C.struct_trapq), C.double(print_time), C.double(pos_x), C.double(pos_y), C.double(pos_z))
}

func Input_shaper_set_shaper_params(sk interface{}, axis int8, n int, a []float64, t []float64) int {
	if len(a) == 0 || len(t) == 0 {
		return int(C.input_shaper_set_shaper_params(sk.(*C.struct_stepper_kinematics), C.char(axis), C.int(n), (*C.double)(unsafe.Pointer(NULL)), (*C.double)(unsafe.Pointer(NULL))))
	}
	return int(C.input_shaper_set_shaper_params(sk.(*C.struct_stepper_kinematics), C.char(axis), C.int(n), (*C.double)(unsafe.Pointer(&a[0])), (*C.double)(unsafe.Pointer(&t[0]))))
}

func Input_shaper_get_step_generation_window(n int, a []float64, t []float64) float64 {
	return float64(C.input_shaper_get_step_generation_window(C.int(n), (*C.double)(unsafe.Pointer(&a[0])), (*C.double)(unsafe.Pointer(&t[0]))))
}

func Input_shaper_alloc() *C.struct_stepper_kinematics {
	return C.input_shaper_alloc()
}

func Input_shaper_set_sk(sk interface{}, orig_sk interface{}) int {
	return int(C.input_shaper_set_sk(sk.(*C.struct_stepper_kinematics), orig_sk.(*C.struct_stepper_kinematics)))
}

func Extruder_stepper_alloc() *C.struct_stepper_kinematics {
	return C.extruder_stepper_alloc()
}

func Extruder_set_pressure_advance(sk interface{}, pressure_advance float64, smooth_time float64) {
	C.extruder_set_pressure_advance(sk.(*C.struct_stepper_kinematics), C.double(pressure_advance), C.double(smooth_time))
}

func CdoubleTofloat64(val interface{}) float64 {
	if val != nil {
		v, ok := val.(C.double)
		if ok {
			return float64(v)
		} else {
			return val.(float64)
		}
	} else {
		return 0.
	}
}

func Corexy_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return C.corexy_stepper_alloc(C.char(axis))
}

func Cartesian_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return C.cartesian_stepper_alloc(C.char(axis))
}

func Cartesian_reverse_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return C.cartesian_reverse_stepper_alloc(C.char(axis))
}

type CStruct_pull_move = C.struct_pull_move

func Trapq_extract_old(tq interface{}, data []CStruct_pull_move, max int, start_time, end_time float64) int {
	return int(C.trapq_extract_old(tq.(*C.struct_trapq), (*C.struct_pull_move)(unsafe.Pointer(&data[0])), C.int(max), C.double(start_time), C.double(end_time)))
}

func Pull_move_alloc(size int) []CStruct_pull_move {
	return make([]C.struct_pull_move, size)
}
