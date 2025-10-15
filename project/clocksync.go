package project

import "C"
import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"k3c/project/chelper"
	"math"
)

const (
	RTT_AGE        = .000010 / (60. * 60.)
	DECAY          = 1. / 30.
	TRANSMIT_EXTRA = .001
)

type ClockSync struct {
	reactor         IReactor
	serial          *SerialReader
	get_clock_timer *ReactorTimer
	get_clock_cmd   []int
	cmd_queue       interface{}
	queries_pending int
	mcu_freq        float64
	last_clock      int64
	clock_est       [3]float64
	// Minimum round-trip-time tracking
	min_half_rtt float64
	min_rtt_time float64
	// Linear regression of mcu clock and system sent_time
	time_avg             float64
	time_variance        float64
	clock_avg            float64
	clock_covariance     float64
	prediction_variance  float64
	last_prediction_time float64
}

func NewClockSync(reactor IReactor) *ClockSync {
	self := ClockSync{}
	self.reactor = reactor
	self.serial = nil
	self.get_clock_timer = reactor.Register_timer(self._get_clock_event, constants.NEVER)
	self.get_clock_cmd = []int{}
	self.cmd_queue = nil
	self.queries_pending = 0
	self.mcu_freq = 1.
	self.last_clock = 0
	self.clock_est = [3]float64{0., 0., 0.}
	// Minimum round-trip-time tracking
	self.min_half_rtt = 999999999.9
	self.min_rtt_time = 0.
	// Linear regression of mcu clock and system sent_time
	self.time_avg = 0
	self.time_variance = 0.
	self.clock_avg = 0.
	self.clock_covariance = 0.
	self.prediction_variance = 0.
	self.last_prediction_time = 0.
	return &self
}
func (self *ClockSync) Connect(serial *SerialReader) {
	self.serial = serial
	self.mcu_freq = serial.Msgparser.Get_constant_float("CLOCK_FREQ", nil)
	// Load initial clock and frequency
	params, _ := serial.Send_with_response("get_uptime", "uptime")
	self.last_clock = (params["high"].(int64) << 32) | params["clock"].(int64)
	self.clock_avg = float64(self.last_clock)
	self.time_avg = chelper.CdoubleTofloat64(params["#sent_time"])
	self.clock_est = [3]float64{self.time_avg, self.clock_avg, self.mcu_freq}
	self.prediction_variance = math.Pow(0.001*self.mcu_freq, 2)
	// Enable periodic get_clock timer
	for i := 0; i < 8; i++ {
		self.reactor.Pause(self.reactor.Monotonic() + 0.050)
		self.last_prediction_time = -9999.
		params, _ = serial.Send_with_response("get_clock", "clock")
		self._handle_clock(params)
	}
	self.get_clock_cmd = serial.Get_msgparser().Create_command("get_clock")
	self.cmd_queue = serial.Alloc_command_queue()
	serial.Register_response(self._handle_clock, "clock", nil)
	self.reactor.Update_timer(self.get_clock_timer, constants.NOW)
}
func (self *ClockSync) Connect_file(serial *SerialReader, pace bool) {
	self.serial = serial
	self.mcu_freq = serial.Msgparser.Get_constant_float("CLOCK_FREQ", object.Sentinel{})
	self.clock_est = [3]float64{0., 0., self.mcu_freq}
	freq := 1000000000000.
	if pace {
		freq = self.mcu_freq
	}
	serial.Set_clock_est(freq, self.reactor.Monotonic(), 0, 0)
}

// MCU clock querying (_handle_clock is invoked from background thread)
func (self *ClockSync) _get_clock_event(eventtime float64) float64 {
	defer sys.CatchPanic()
	self.serial.Raw_send(self.get_clock_cmd, 0, 0, self.cmd_queue)
	self.queries_pending += 1
	// Use an unusual time for the next event so clock messages
	// don't resonate with other periodic events.
	return eventtime + .9839
}
func (self *ClockSync) _handle_clock(params map[string]interface{}) error {
	self.queries_pending = 0

	// Extend clock to 64bit
	lastClock := self.last_clock
	clock := (lastClock &^ 0xffffffff) | params["clock"].(int64)
	if clock < lastClock {
		clock += 0x100000000
	}
	self.last_clock = clock

	// Check if this is the best round-trip-time seen so far
	sentTime := chelper.CdoubleTofloat64(params["#sent_time"])
	if sentTime == 0 {
		return nil
	}
	receiveTime := chelper.CdoubleTofloat64(params["#receive_time"])
	halfRTT := (receiveTime - sentTime) * 0.5
	agedRTT := (sentTime - self.min_rtt_time) * RTT_AGE
	if halfRTT < self.min_half_rtt+agedRTT {
		self.min_half_rtt = halfRTT
		self.min_rtt_time = sentTime
	}

	// Filter out samples that are extreme outliers
	expClock := (sentTime-self.time_avg)*self.clock_est[2] + self.clock_avg
	clockDiff2 := math.Pow(float64(clock-int64(expClock)), 2)
	if clockDiff2 > 25.*self.prediction_variance && clockDiff2 > math.Pow(.000500*self.mcu_freq, 2) {
		if clock > int64(expClock) && sentTime < self.last_prediction_time+10. {
			//log.Printf("Ignoring clock sample %.3f: freq=%f diff=%d stddev=%.3f", sentTime, self.clock_est[2], clock-int(expClock), math.Sqrt(self.prediction_variance))
			return nil
		}
		//log.Printf("Resetting prediction variance %.3f: freq=%d diff=%d stddev=%.3f", sentTime, self.clock_est[2], clock-int(expClock), math.Sqrt(self.prediction_variance))
		self.prediction_variance = math.Pow(.001*self.mcu_freq, 2)
	} else {
		self.last_prediction_time = sentTime
		self.prediction_variance = (1. - DECAY) * (self.prediction_variance + clockDiff2*DECAY)
	}

	// Add clock and sent_time to linear regression
	diffSentTime := sentTime - self.time_avg
	self.time_avg += DECAY * diffSentTime
	self.time_variance = (1. - DECAY) * (self.time_variance + math.Pow(diffSentTime, 2)*DECAY)

	diffClock := clock - int64(self.clock_avg)
	self.clock_avg += DECAY * float64(diffClock)
	self.clock_covariance = (1. - DECAY) * (self.clock_covariance + diffSentTime*float64(diffClock)*DECAY)

	// Update prediction from linear regression
	newFreq := self.clock_covariance / self.time_variance
	predStddev := math.Sqrt(self.prediction_variance)

	self.serial.Set_clock_est(newFreq, self.time_avg+TRANSMIT_EXTRA,
		int64(self.clock_avg-3.*predStddev), clock)
	self.clock_est = [3]float64{self.time_avg + self.min_half_rtt,
		self.clock_avg, newFreq}
	return nil
}

// clock frequency conversions
func (self *ClockSync) Print_time_to_clock(print_time float64) int64 {
	return int64(print_time * self.mcu_freq)
}
func (self *ClockSync) Clock_to_print_time(clock int64) float64 {
	return float64(clock) / self.mcu_freq
}

// system time conversions
func (self *ClockSync) Get_clock(eventtime float64) int64 {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]
	return int64(clock + (eventtime-sample_time)*freq)
}
func (self *ClockSync) Estimate_clock_systime(reqclock uint64) float64 {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]
	return (float64(reqclock)-clock)/freq + sample_time
}
func (self *ClockSync) Estimated_print_time(eventtime float64) float64 {
	return self.Clock_to_print_time(self.Get_clock(eventtime))
}

// misc commands
func (self *ClockSync) Clock32_to_clock64(clock32 int64) int64 {
	last_clock := self.last_clock
	clock_diff := (last_clock - clock32) & 0xffffffff
	if clock_diff&0x80000000 == 0x80000000 {
		return last_clock + 0x100000000 - clock_diff
	}
	return last_clock - clock_diff
}
func (self *ClockSync) Is_active() bool {
	return self.queries_pending <= 4
}
func (self *ClockSync) Dump_debug() string {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]
	return fmt.Sprintf("clocksync state: mcu_freq=%f last_clock=%d"+
		" clock_est=(%.3f %f %.3f) min_half_rtt=%.6f min_rtt_time=%.3f"+
		" time_avg=%.3f(%.3f) clock_avg=%.3f(%.3f)"+
		" pred_variance=%.3f",
		self.mcu_freq, self.last_clock, sample_time, clock, freq,
		self.min_half_rtt, self.min_rtt_time,
		self.time_avg, self.time_variance,
		self.clock_avg, self.clock_covariance,
		self.prediction_variance)
}
func (self *ClockSync) Stats(eventtime float64) string {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]

	return fmt.Sprintf("sample_time=%f clock=%f freq=%f", sample_time, clock, freq)
}
func (self *ClockSync) Calibrate_clock(print_time float64, eventtime float64) []float64 {
	return []float64{0., self.mcu_freq}
}

// Clock syncing code for secondary MCUs (whose clocks are sync'ed to a
// primary MCU)
type SecondarySync struct {
	reactor         IReactor
	serial          *SerialReader
	get_clock_timer *ReactorTimer
	get_clock_cmd   []int
	cmd_queue       interface{}
	queries_pending int
	mcu_freq        float64
	last_clock      int64
	clock_est       [3]float64
	// Minimum round-trip-time tracking
	min_half_rtt float64
	min_rtt_time float64
	// Linear regression of mcu clock and system sent_time
	time_avg             float64
	time_variance        float64
	clock_avg            float64
	clock_covariance     float64
	prediction_variance  float64
	last_prediction_time float64
	main_sync            *ClockSync
	clock_adj            []float64
	last_sync_time       float64
}

func NewSecondarySync(reactor IReactor, main_sync *ClockSync) ClockSyncAble {
	self := &SecondarySync{}
	//
	self.reactor = reactor
	self.serial = nil
	self.get_clock_timer = reactor.Register_timer(self._get_clock_event, constants.NEVER)
	self.get_clock_cmd = []int{}
	self.cmd_queue = nil
	self.queries_pending = 0
	self.mcu_freq = 1.
	self.last_clock = 0
	self.clock_est = [3]float64{0., 0., 0.}
	// Minimum round-trip-time tracking
	self.min_half_rtt = 999999999.9
	self.min_rtt_time = 0.
	// Linear regression of mcu clock and system sent_time
	self.time_avg = 0
	self.time_variance = 0.
	self.clock_avg = 0.
	self.clock_covariance = 0.
	self.prediction_variance = 0.
	self.last_prediction_time = 0.
	//
	self.main_sync = main_sync
	self.clock_adj = []float64{0., 1.}
	self.last_sync_time = 0.

	return self
}
func (self *SecondarySync) Connect(serial *SerialReader) {
	self.serial = serial
	self.mcu_freq = serial.Msgparser.Get_constant_float("CLOCK_FREQ", nil)
	// Load initial clock and frequency
	params, _ := serial.Send_with_response("get_uptime", "uptime")
	self.last_clock = (params["high"].(int64) << 32) | params["clock"].(int64)
	self.clock_avg = float64(self.last_clock)
	self.time_avg = chelper.CdoubleTofloat64(params["#sent_time"])
	self.clock_est = [3]float64{self.time_avg, self.clock_avg, self.mcu_freq}
	self.prediction_variance = math.Pow(0.001*self.mcu_freq, 2)
	// Enable periodic get_clock timer
	for i := 0; i < 8; i++ {
		self.reactor.Pause(self.reactor.Monotonic() + 0.050)
		self.last_prediction_time = -9999.
		params, _ = serial.Send_with_response("get_clock", "clock")
		self._handle_clock(params)
	}
	self.get_clock_cmd = serial.Get_msgparser().Create_command("get_clock")
	self.cmd_queue = serial.Alloc_command_queue()
	serial.Register_response(self._handle_clock, "clock", nil)
	self.reactor.Update_timer(self.get_clock_timer, constants.NOW)

	self.clock_adj = []float64{0., self.mcu_freq}
	curtime := self.reactor.Monotonic()
	main_print_time := self.main_sync.Estimated_print_time(curtime)
	local_print_time := self.Estimated_print_time(curtime)
	self.clock_adj = []float64{main_print_time - local_print_time, self.mcu_freq}
	self.Calibrate_clock(0., curtime)
}
func (self *SecondarySync) Connect_file(serial *SerialReader, pace bool) {
	self.serial = serial
	self.mcu_freq = serial.Msgparser.Get_constant_float("CLOCK_FREQ", object.Sentinel{})
	self.clock_est = [3]float64{0., 0., self.mcu_freq}
	freq := 1000000000000.
	if pace {
		freq = self.mcu_freq
	}
	serial.Set_clock_est(freq, self.reactor.Monotonic(), 0, 0)
	self.clock_adj = []float64{0., self.mcu_freq}
}

// MCU clock querying (_handle_clock is invoked from background thread)
func (self *SecondarySync) _get_clock_event(eventtime float64) float64 {
	defer sys.CatchPanic()
	self.serial.Raw_send(self.get_clock_cmd, 0, 0, self.cmd_queue)
	self.queries_pending += 1
	// Use an unusual time for the next event so clock messages
	// don't resonate with other periodic events.
	return eventtime + .9839
}
func (self *SecondarySync) _handle_clock(params map[string]interface{}) error {
	self.queries_pending = 0

	// Extend clock to 64bit
	lastClock := self.last_clock
	clock := (lastClock &^ 0xffffffff) | params["clock"].(int64)
	if clock < lastClock {
		clock += 0x100000000
	}
	self.last_clock = clock

	// Check if this is the best round-trip-time seen so far
	sentTime := chelper.CdoubleTofloat64(params["#sent_time"])
	if sentTime == 0 {
		return nil
	}
	receiveTime := chelper.CdoubleTofloat64(params["#receive_time"])
	halfRTT := (receiveTime - sentTime) * 0.5
	agedRTT := (sentTime - self.min_rtt_time) * RTT_AGE
	if halfRTT < self.min_half_rtt+agedRTT {
		self.min_half_rtt = halfRTT
		self.min_rtt_time = sentTime
	}

	// Filter out samples that are extreme outliers
	expClock := (sentTime-self.time_avg)*self.clock_est[2] + self.clock_avg
	clockDiff2 := math.Pow(float64(clock-int64(expClock)), 2)
	if clockDiff2 > 25.*self.prediction_variance && clockDiff2 > math.Pow(.000500*self.mcu_freq, 2) {
		if clock > int64(expClock) && sentTime < self.last_prediction_time+10. {
			//log.Printf("Ignoring clock sample %.3f: freq=%f diff=%d stddev=%.3f", sentTime, self.clock_est[2], clock-int(expClock), math.Sqrt(self.prediction_variance))
			return nil
		}
		//log.Printf("Resetting prediction variance %.3f: freq=%d diff=%d stddev=%.3f", sentTime, self.clock_est[2], clock-int(expClock), math.Sqrt(self.prediction_variance))
		self.prediction_variance = math.Pow(.001*self.mcu_freq, 2)
	} else {
		self.last_prediction_time = sentTime
		self.prediction_variance = (1. - DECAY) * (self.prediction_variance + clockDiff2*DECAY)
	}

	// Add clock and sent_time to linear regression
	diffSentTime := sentTime - self.time_avg
	self.time_avg += DECAY * diffSentTime
	self.time_variance = (1. - DECAY) * (self.time_variance + math.Pow(diffSentTime, 2)*DECAY)

	diffClock := clock - int64(self.clock_avg)
	self.clock_avg += DECAY * float64(diffClock)
	self.clock_covariance = (1. - DECAY) * (self.clock_covariance + diffSentTime*float64(diffClock)*DECAY)

	// Update prediction from linear regression
	newFreq := self.clock_covariance / self.time_variance
	predStddev := math.Sqrt(self.prediction_variance)

	self.serial.Set_clock_est(newFreq, self.time_avg+TRANSMIT_EXTRA,
		int64(self.clock_avg-3.*predStddev), clock)
	self.clock_est = [3]float64{self.time_avg + self.min_half_rtt,
		self.clock_avg, newFreq}
	return nil
}

// clock frequency conversions
func (self *SecondarySync) Print_time_to_clock(print_time float64) int64 {
	adjusted_offset := self.clock_adj[0]
	adjusted_freq := self.clock_adj[1]
	return int64((print_time - adjusted_offset) * adjusted_freq)
}
func (self *SecondarySync) Clock_to_print_time(clock int64) float64 {
	adjusted_offset := self.clock_adj[0]
	adjusted_freq := self.clock_adj[1]
	return float64(clock)/adjusted_freq + adjusted_offset
}

// system time conversions
func (self *SecondarySync) Get_clock(eventtime float64) int64 {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]
	return int64(clock + (eventtime-sample_time)*freq)
}
func (self *SecondarySync) Estimate_clock_systime(reqclock uint64) float64 {
	sample_time := self.clock_est[0]
	clock := self.clock_est[1]
	freq := self.clock_est[2]
	return (float64(reqclock)-clock)/freq + sample_time
}
func (self *SecondarySync) Estimated_print_time(eventtime float64) float64 {
	return self.Clock_to_print_time(self.Get_clock(eventtime))
}

// misc commands
func (self *SecondarySync) Clock32_to_clock64(clock32 int64) int64 {
	last_clock := self.last_clock
	clock_diff := (last_clock - clock32) & 0xffffffff
	if clock_diff&0x80000000 == 0x80000000 {
		return last_clock + 0x100000000 - clock_diff
	}
	return last_clock - clock_diff
}
func (self *SecondarySync) Is_active() bool {
	return self.queries_pending <= 4
}

// misc commands
func (self *SecondarySync) Dump_debug() string {
	adjusted_offset := self.clock_adj[0]
	adjusted_freq := self.clock_adj[1]
	return fmt.Sprintf("%s clock_adj=(%.3f %.3f)",
		self.main_sync.Dump_debug(), adjusted_offset, adjusted_freq)
}
func (self *SecondarySync) Stats(eventtime float64) string {
	//adjusted_offset := self.clock_adj[0]
	adjusted_freq := self.clock_adj[1]
	return fmt.Sprintf("%s adj=%f", self.main_sync.Stats(eventtime), adjusted_freq)
}
func (self *SecondarySync) Calibrate_clock(print_time float64, eventtime float64) []float64 {
	//Calculate: est_print_time = main_sync.estimatated_print_time()
	ser_time := self.main_sync.clock_est[0]
	ser_clock := self.main_sync.clock_est[1]
	ser_freq := self.main_sync.clock_est[2]
	main_mcu_freq := self.main_sync.mcu_freq
	est_main_clock := (eventtime-ser_time)*ser_freq + ser_clock
	est_print_time := est_main_clock / main_mcu_freq
	// Determine sync1_print_time and sync2_print_time
	sync1_print_time := math.Max(print_time, est_print_time)
	sync2_print_time := math.Max(sync1_print_time+4., self.last_sync_time)
	sync2_print_time = math.Max(sync2_print_time, print_time+2.5*(print_time-est_print_time))
	// Calc sync2_sys_time (inverse of main_sync.estimatated_print_time)
	sync2_main_clock := sync2_print_time * main_mcu_freq
	sync2_sys_time := ser_time + (sync2_main_clock-ser_clock)/ser_freq
	// Adjust freq so estimated print_time will match at sync2_print_time
	sync1_clock := self.Print_time_to_clock(sync1_print_time)
	sync2_clock := self.Get_clock(sync2_sys_time)
	adjusted_freq := float64(sync2_clock-sync1_clock) / (sync2_print_time - sync1_print_time)
	adjusted_offset := sync1_print_time - float64(sync1_clock)/adjusted_freq
	// Apply new values
	self.clock_adj = []float64{adjusted_offset, adjusted_freq}
	self.last_sync_time = sync2_print_time
	return self.clock_adj
}

type ClockSyncAble interface {
	Connect(serial *SerialReader)
	Connect_file(serial *SerialReader, pace bool)
	Print_time_to_clock(print_time float64) int64
	Clock_to_print_time(clock int64) float64
	Get_clock(eventtime float64) int64
	Estimate_clock_systime(reqclock uint64) float64
	Estimated_print_time(eventtime float64) float64
	Clock32_to_clock64(clock32 int64) int64
	Is_active() bool
	Dump_debug() string
	Stats(eventtime float64) string
	Calibrate_clock(print_time float64, eventtime float64) []float64
}
