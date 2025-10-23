package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
	"os"
	"strings"
	"time"
)

// Helper class to obtain measurements
type AccelQueryHelper struct {
	printer            *Printer
	ccon               ICcon
	request_start_time float64
	request_end_time   float64
	//samples            []*Accel_Measurement
	samples     [][]float64
	raw_samples []map[string]map[string]interface{}
}

type ICcon interface {
	Finalize()
	Get_messages() []map[string]map[string]interface{}
}

func NewAccelQueryHelper(printer *Printer, ccon ICcon) *AccelQueryHelper {
	self := &AccelQueryHelper{}
	self.printer = printer
	self.ccon = ccon
	print_time := MustLookupToolhead(self.printer).Get_last_move_time()
	self.request_start_time = print_time
	self.request_end_time = print_time
	self.samples = [][]float64{}
	self.raw_samples = nil
	return self
}

func (self *AccelQueryHelper) Finish_measurements() {
	toolhead := MustLookupToolhead(self.printer)
	self.request_end_time = toolhead.Get_last_move_time()
	toolhead.Wait_moves()
	self.ccon.Finalize()
}

func (self *AccelQueryHelper) _get_raw_samples() []map[string]map[string]interface{} {
	raw_samples := self.ccon.Get_messages()
	if len(raw_samples) > 0 {
		self.raw_samples = raw_samples
	}
	return self.raw_samples
}

func (self *AccelQueryHelper) Has_valid_samples() bool {
	raw_samples := self._get_raw_samples()
	for i, msg := range raw_samples {
		if _, ok := msg["params"]["data"].([][]float64); !ok {
			fmt.Printf("%v", i)
			continue
		}
		data := msg["params"]["data"].([][]float64)
		first_sample_time := data[0][0]
		last_sample_time := data[len(data)-1][0]
		if first_sample_time > self.request_end_time ||
			last_sample_time < self.request_start_time {
			continue
		}
		// The time intervals [first_sample_time, last_sample_time]
		// and [request_start_time, request_end_time] have non-zero
		// intersection. It is still theoretically possible that none
		// of the samples from raw_samples fall into the time interval
		// [request_start_time, request_end_time] if it is too narrow
		// or on very heavy data losses. In practice, that interval
		// is at least 1 second, so this possibility is negligible.
		return true
	}
	return false
}

func (self *AccelQueryHelper) Get_samples() [][]float64 {
	raw_samples := self._get_raw_samples()
	if len(raw_samples) == 0 {
		return self.samples
	}
	total := 0
	for _, m := range raw_samples {
		if _, ok := m["params"]["data"].([][]float64); !ok {
			continue
		}
		total += len(m["params"]["data"].([][]float64))
	}
	count := 0
	self.samples = make([][]float64, total)
	samples := self.samples

	for _, msg := range raw_samples {
		if _, ok := msg["params"]["data"].([][]float64); !ok {
			continue
		}
		for _, msg_params_data := range msg["params"]["data"].([][]float64) {
			samp_time := msg_params_data[0]
			x := msg_params_data[1]
			y := msg_params_data[2]
			z := msg_params_data[3]
			if samp_time < self.request_start_time {
				continue
			}
			if samp_time > self.request_end_time {
				break
			}
			samples[count] = []float64{samp_time, x, y, z}
			count += 1
		}
	}

	samples = samples[0:count]
	self.samples = samples
	return self.samples
}

func (self *AccelQueryHelper) Write_to_file(filename string) {
	write_impl := func() {
		// Try to re-nice writing process
		// todo os.nice(20)
		f, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, os.ModeExclusive|0666)
		if err != nil {
			panic(err)
		}
		defer f.Close()
		f.Write([]byte("#time,accel_x,accel_y,accel_z\r\n"))
		samples := [][]float64{}
		if len(self.samples) > 0 {
			samples = self.samples
		} else if len(self.Get_samples()) > 0 {
			samples = self.Get_samples()
		}
		for _, val := range samples {
			t := val[0]
			accel_x := val[1]
			accel_y := val[2]
			accel_z := val[3]
			f.Write([]byte(fmt.Sprintf("%.6f,%.6f,%.6f,%.6f\n", t, accel_x, accel_y, accel_z)))
		}
	}

	go write_impl()
}

// Helper class for G-Code commands
type AccelCommandHelper struct {
	printer   *Printer
	chip      IAccelChip
	bg_client IAclient
	base_name string
	name      string
}

func NewAccelCommandHelper(config *ConfigWrapper, chip IAccelChip) *AccelCommandHelper {
	// Register commands
	self := &AccelCommandHelper{}
	self.printer = config.Get_printer()
	self.chip = chip
	self.bg_client = nil
	name_parts := strings.Split(config.Get_name(), " ")
	self.base_name = name_parts[0]
	self.name = name_parts[len(name_parts)-1]
	self.Register_commands(config.Get_name())
	if len(name_parts) == 1 {
		if self.name == "adxl345" || !config.Has_section("adxl345") {
			self.Register_commands("")
		}
	}
	return self
}

func (self *AccelCommandHelper) Register_commands(name string) {
	// Register commands

	gcode := MustLookupGcode(self.printer)
	gcode.Register_mux_command(
		"ACCELEROMETER_MEASURE", "CHIP", name,
		self.Cmd_ACCELEROMETER_MEASURE,
		cmd_ACCELEROMETER_MEASURE_help)
	gcode.Register_mux_command("ACCELEROMETER_QUERY", "CHIP", name,
		self.Cmd_ACCELEROMETER_QUERY,
		cmd_ACCELEROMETER_QUERY_help)
	gcode.Register_mux_command("ACCELEROMETER_DEBUG_READ", "CHIP", name,
		self.Cmd_ACCELEROMETER_DEBUG_READ,
		cmd_ACCELEROMETER_DEBUG_READ_help)
	gcode.Register_mux_command("ACCELEROMETER_DEBUG_WRITE", "CHIP", name,
		self.Cmd_ACCELEROMETER_DEBUG_WRITE,
		cmd_ACCELEROMETER_DEBUG_WRITE_help)

}

const cmd_ACCELEROMETER_MEASURE_help = "Start/stop accelerometer"

func (self *AccelCommandHelper) Cmd_ACCELEROMETER_MEASURE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	if value.IsNone(self.bg_client) {
		// Start measurements
		self.bg_client = self.chip.Start_internal_client()
		logger.Debug("accelerometer measurements started")
		return nil
	}
	// End measurements
	name := gcmd.Get("NAME", time.Now().Format("20230215_161242"), value.None, nil, nil, nil, nil)
	normalized := strings.ReplaceAll(strings.ReplaceAll(name, "-", ""), "_", "")
	if !str.IsAlphanum(normalized) {
		panic(errors.New("Invalid NAME parameter"))
	}

	bg_client := self.bg_client
	self.bg_client = nil

	bg_client.Finish_measurements()
	// Write data to file
	var filename string
	if self.base_name == self.name {
		filename = fmt.Sprintf("/tmp/%s-%s.csv", self.base_name, name)
	} else {
		filename = fmt.Sprintf("/tmp/%s-%s-%s.csv", self.base_name, self.name, name)
	}

	bg_client.Write_to_file(filename)
	gcmd.Respond_info(fmt.Sprintf("Writing raw accelerometer data to %s file", filename), true)
	return nil
}

const cmd_ACCELEROMETER_QUERY_help = "Query accelerometer for the current values"

func (self *AccelCommandHelper) Cmd_ACCELEROMETER_QUERY(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	aclient := self.chip.Start_internal_client()
	MustLookupToolhead(self.printer).Dwell(1.)
	aclient.Finish_measurements()
	values := aclient.Get_samples()
	if len(values) == 0 {
		return errors.New("No accelerometer measurements found")
	}

	accel_x := values[len(values)-1][1]
	accel_y := values[len(values)-1][2]
	accel_z := values[len(values)-1][3]
	gcmd.Respond_info(fmt.Sprintf("accelerometer values (x, y, z): %.6f, %.6f, %.6f",
		accel_x, accel_y, accel_z), true)
	return nil
}

const cmd_ACCELEROMETER_DEBUG_READ_help = "Query register (for debugging)"

func (self *AccelCommandHelper) Cmd_ACCELEROMETER_DEBUG_READ(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)

	reg := cast.ToInt(gcmd.Get("REG", 0, value.None, cast.Float64P(0), cast.Float64P(126), nil, nil))
	val := self.chip.Read_reg(reg)
	gcmd.Respond_info(fmt.Sprintf("Accelerometer REG[0x%x] = 0x%x", reg, val), true)
	return nil
}

const cmd_ACCELEROMETER_DEBUG_WRITE_help = "Set register (for debugging)"

func (self *AccelCommandHelper) Cmd_ACCELEROMETER_DEBUG_WRITE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	reg := gcmd.Get("REG", value.None, value.None, cast.Float64P(0), cast.Float64P(126), nil, nil)
	val := gcmd.Get("VAL", value.None, value.None, cast.Float64P(0), cast.Float64P(255), nil, nil)
	self.chip.Set_reg(cast.ToInt(reg), cast.ToInt(val), 0)
	return nil
}

// Helper class for chip clock synchronization via linear regression
type ClockSyncRegression struct {
	mcu                   *MCU
	chip_clock_smooth     float64
	decay                 float64
	last_chip_clock       float64
	last_exp_mcu_clock    float64
	mcu_clock_avg         float64
	mcu_clock_variance    float64
	chip_clock_avg        float64
	chip_clock_covariance float64
	last_mcu_clock        float64
}

// NewClockSyncRegression
// default: decay = 1.0/20.0
func NewClockSyncRegression(mcu *MCU, chip_clock_smooth float64, decay float64) *ClockSyncRegression {
	self := new(ClockSyncRegression)
	self.mcu = mcu
	self.chip_clock_smooth = chip_clock_smooth
	self.decay = decay
	self.last_chip_clock = 0.
	self.last_exp_mcu_clock = 0.
	self.mcu_clock_avg = 0.
	self.mcu_clock_variance = 0.
	self.chip_clock_avg = 0.
	self.chip_clock_covariance = 0.
	return self
}

func (self *ClockSyncRegression) Reset(mcu_clock, chip_clock float64) {
	self.mcu_clock_avg = mcu_clock
	self.last_mcu_clock = mcu_clock
	self.chip_clock_avg = chip_clock
	self.mcu_clock_variance = 0.
	self.chip_clock_covariance = 0.
	self.last_chip_clock = 0.
	self.last_exp_mcu_clock = 0.
}

func (self *ClockSyncRegression) Update(mcu_clock, chip_clock float64) {
	// Update linear regression
	decay := self.decay
	diff_mcu_clock := mcu_clock - self.mcu_clock_avg
	self.mcu_clock_avg += decay * diff_mcu_clock
	self.mcu_clock_variance = (1. - decay) *
		(self.mcu_clock_variance + math.Pow(diff_mcu_clock, 2)*decay)
	diff_chip_clock := chip_clock - self.chip_clock_avg
	self.chip_clock_avg += decay * diff_chip_clock
	self.chip_clock_covariance = (1. - decay) * (self.chip_clock_covariance + diff_mcu_clock*diff_chip_clock*decay)
}

func (self *ClockSyncRegression) Set_last_chip_clock(chip_clock float64) {
	base_mcu, base_chip, inv_cfreq := self.Get_clock_translation()
	self.last_chip_clock = chip_clock
	self.last_exp_mcu_clock = base_mcu + (chip_clock-base_chip)*inv_cfreq
}

func (self *ClockSyncRegression) Get_clock_translation() (float64, float64, float64) {
	inv_chip_freq := self.mcu_clock_variance / self.chip_clock_covariance
	if value.Not(self.last_chip_clock) {
		return self.mcu_clock_avg, self.chip_clock_avg, inv_chip_freq
	}
	// Find mcu clock associated with future chip_clock
	s_chip_clock := self.last_chip_clock + self.chip_clock_smooth
	scdiff := s_chip_clock - self.chip_clock_avg
	s_mcu_clock := self.mcu_clock_avg + scdiff*inv_chip_freq
	// Calculate frequency to converge at future point
	mdiff := s_mcu_clock - self.last_exp_mcu_clock
	s_inv_chip_freq := mdiff / self.chip_clock_smooth
	return self.last_exp_mcu_clock, self.last_chip_clock, s_inv_chip_freq
}

func (self *ClockSyncRegression) Get_time_translation() (float64, float64, float64) {
	base_mcu, base_chip, inv_cfreq := self.Get_clock_translation()
	clock_to_print_time := self.mcu.Clock_to_print_time
	base_time := clock_to_print_time(int64(base_mcu))
	inv_freq := clock_to_print_time(int64(base_mcu+inv_cfreq)) - base_time
	return base_time, base_chip, inv_freq
}

type IAclient interface {
	Finish_measurements()
	Write_to_file(file string)
	Has_valid_samples() bool
	Get_samples() [][]float64
}

type IAccelChip interface {
	Start_internal_client() IAclient
	Set_reg(reg, val int, minclock int64) error
	Read_reg(reg int) byte
	Get_name() string
}
