//go:build !linux
// +build !linux

package chelper

/*
#include <stdint.h>

#define MESSAGE_MAX 64


struct pull_queue_message {
    uint8_t Msg[MESSAGE_MAX];
    int Len;
    double Sent_time;
    double Receive_time;
    uint64_t Notify_id;
};

struct pull_history_steps {
	uint64_t first_clock;
	uint64_t last_clock;
	int64_t start_position;
	int step_count;
	int interval;
	int add;
};

struct pull_move {
	double Print_time;
	double Move_t;
	double Start_v;
	double Accel;
	double Start_x;
	double Start_y;
	double Start_z;
	double X_r;
	double Y_r;
	double Z_r;
};

typedef struct pull_queue_message struct_pull_queue_message;
typedef struct pull_history_steps struct_pull_history_steps;
typedef struct pull_move struct_pull_move;

struct serialqueue;
struct command_queue;
struct trdispatch;
struct trdispatch_mcu;
struct steppersync;
struct stepcompress;
struct stepper_kinematics;
struct trapq;
struct clock_estimate {
    uint64_t last_clock;
    uint64_t conv_clock;
    double conv_time;
    double est_freq;
};

struct fastreader;

*/
import "C"

import (
	"unsafe"
)

const GCC_CMD = "gcc"

var NULL = unsafe.Pointer(nil)

type FFI_lib struct{}

func (self *FFI_lib) Get_monotonic() float64 {
	return 0
}

func Free(p interface{}) {}

func get_abs_files(srcdir string, filelist []string) []string {
	return nil
}

func get_mtimes(filelist []string) []float64 {
	return nil
}

func check_build_code(sources []string, target string) bool {
	return false
}

func check_gcc_option(option string) bool {
	return false
}

var stubLib = &FFI_lib{}

func Get_ffi() *FFI_lib {
	return stubLib
}

func New_pull_queue_message() *C.struct_pull_queue_message {
	msg := new(C.struct_pull_queue_message)
	msg.Len = -1
	return msg
}

func Serialqueue_pull(serialqueue interface{}, response interface{}) {
	if response == nil {
		return
	}
	resp := response.(*C.struct_pull_queue_message)
	resp.Len = -1
}

func Serialqueue_alloc(fileno uintptr, serial_fd_type byte, client_id int) *C.struct_serialqueue {
	return nil
}

func Serialqueue_free(serialqueue interface{}) {}

func Serialqueue_alloc_commandqueue() *C.struct_command_queue {
	return nil
}

func Serialqueue_free_commandqueue(commandqueue interface{}) {}

func Serialqueue_exit(serialqueue interface{}) {}

func Serialqueue_set_wire_frequency(serialqueue interface{}, wire_freq float64) {}

func Serialqueue_set_receive_window(serialqueue interface{}, receive_window int) {}

func Serialqueue_set_clock_est(serialqueue interface{}, freq float64, conv_time float64, conv_clock uint64, last_clock uint64) {
}

func Serialqueue_send(serialqueue interface{}, cmd_queue interface{}, cmd []int, count int, minclock, reqclock, last int64) {
}

func Serialqueue_get_stats(serialqueue interface{}, stats_buf []byte) {}

func Steppersync_alloc(serialqueue interface{}, sc_list interface{}, sc_num int, move_num int) *C.struct_steppersync {
	return nil
}

func Steppersync_free(syc interface{}) {}

func Steppersync_set_time(syc interface{}, time_offset float64, mcu_freq float64) {}

func Steppersync_flush(syc interface{}, move_clock uint64, clear_history_clock uint64) int {
	return 0
}

func Trdispatch_mcu_alloc(td interface{}, sq interface{}, cq interface{}, trsync_oid int, set_timeout_msgtag uint32, trigger_msgtag uint32, state_msgtag uint32) *C.struct_trdispatch_mcu {
	return nil
}

func Trdispatch_alloc() *C.struct_trdispatch {
	return nil
}

func Trdispatch_start(td interface{}, dispatch_reason uint32) {}

func Trdispatch_stop(td interface{}) {}

func Trdispatch_mcu_setup(tdm interface{}, last_status_clock uint64, expire_clock uint64, expire_ticks uint64, min_extend_ticks uint64) {
}

func Stepcompress_alloc(oid uint32) *C.struct_stepcompress {
	return nil
}

func Stepcompress_free(p interface{}) {}

func Stepcompress_fill(sc interface{}, max_error uint32, queue_step_msgtag int32, set_next_step_dir_msgtag int32) {
}

func Stepcompress_set_invert_sdir(sc interface{}, invert_sdir uint32) {}

func Stepcompress_find_past_position(sc interface{}, clock uint64) int64 {
	return 0
}

func Stepcompress_extract_old(sc interface{}, p interface{}, max int, start_clock uint64, end_clock uint64) int {
	return 0
}

func Stepcompress_reset(sc interface{}, last_step_clock uint64) int {
	return 0
}

func Stepcompress_queue_msg(sc interface{}, data interface{}, length int) int {
	return 0
}

func Stepcompress_set_last_position(sc interface{}, clock uint64, last_position int64) int {
	return 0
}

func New_pull_history_steps() []C.struct_pull_history_steps {
	return []C.struct_pull_history_steps{}
}

func Itersolve_calc_position_from_coord(sk interface{}, x float64, y float64, z float64) float64 {
	return 0
}

func Itersolve_set_position(sk interface{}, x float64, y float64, z float64) {}

func Itersolve_get_commanded_pos(sk interface{}) float64 {
	return 0
}

func Itersolve_set_stepcompress(sk interface{}, sc interface{}, step_dist float64) {}

func Itersolve_set_trapq(sk interface{}, tq interface{}) {}

func Itersolve_is_active_axis(sk interface{}, axis byte) int32 {
	return 0
}

func Itersolve_generate_steps(sk interface{}, flush_time float64) int32 {
	return 0
}

func Itersolve_check_active(sk interface{}, flush_time float64) float64 {
	return 0
}

func Trapq_alloc() *C.struct_trapq {
	return nil
}

func Trapq_free(tq interface{}) {}

func Trapq_append(tq interface{}, print_time float64, accel_t float64, cruise_t float64, decel_t float64, start_pos_x float64, start_pos_y float64, start_pos_z float64, axes_r_x float64, axes_r_y float64, axes_r_z float64, start_v float64, cruise_v float64, accel float64) {
}

func Trapq_finalize_moves(tq interface{}, print_time float64, clear_history_time float64) {}

func Trapq_set_position(tq interface{}, print_time float64, pos_x float64, pos_y float64, pos_z float64) {
}

func Input_shaper_set_shaper_params(sk interface{}, axis int8, n int, a []float64, t []float64) int {
	return 0
}

func Input_shaper_get_step_generation_window(n int, a []float64, t []float64) float64 {
	return 0
}

func Input_shaper_alloc() *C.struct_stepper_kinematics {
	return nil
}

func Input_shaper_set_sk(sk interface{}, orig_sk interface{}) int {
	return 0
}

func Extruder_stepper_alloc() *C.struct_stepper_kinematics {
	return nil
}

func Extruder_set_pressure_advance(sk interface{}, pressure_advance float64, smooth_time float64) {}

func CdoubleTofloat64(val interface{}) float64 {
	switch v := val.(type) {
	case C.double:
		return float64(v)
	case float64:
		return v
	default:
		return 0
	}
}

func Cartesian_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return nil
}

func Corexy_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return nil
}

func Cartesian_reverse_stepper_alloc(axis int8) *C.struct_stepper_kinematics {
	return nil
}

type CStruct_pull_move = C.struct_pull_move

func Trapq_extract_old(tq interface{}, data []CStruct_pull_move, max int, start_time, end_time float64) int {
	return 0
}

func Pull_move_alloc(size int) []CStruct_pull_move {
	return make([]C.struct_pull_move, size)
}

type CStructPullHistorySteps = C.struct_pull_history_steps
type CStructPullMove = C.struct_pull_move
type CStructPullQueueMessage = C.struct_pull_queue_message
