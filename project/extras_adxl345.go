package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"math"
	"strings"
	"sync"
)

// ADXL345 registers
var ADXL345_REGISTERS = map[string]int{
	"REG_DEVID":       0x00,
	"REG_BW_RATE":     0x2C,
	"REG_POWER_CTL":   0x2D,
	"REG_DATA_FORMAT": 0x31,
	"REG_FIFO_CTL":    0x38,
	"REG_MOD_READ":    0x80,
	"REG_MOD_MULTI":   0x40,
}

var ADXL345_QUERY_RATES = map[int]int{
	25: 0x8, 50: 0x9, 100: 0xa, 200: 0xb, 400: 0xc,
	800: 0xd, 1600: 0xe, 3200: 0xf,
}

var ADXL345_CLK = map[string]float64{
	"MIN_MSG_TIME":      0.100,
	"BYTES_PER_SAMPLE":  5,
	"SAMPLES_PER_BLOCK": 10,
}

var ADXL345_INFO = map[string]interface{}{
	"DEV_ID":         0xe5,
	"SET_FIFO_CTL":   0x90,
	"FREEFALL_ACCEL": 9.80665 * 1000.,
	"SCALE_XY":       0.003774 * 9.80665 * 1000., // 1 / 265 (at 3.3V) mg/LSB,
	"SCALE_Z":        0.003906 * 9.80665 * 1000., // 1 / 256 (at 3.3V) mg/LSB,
}

// Printer class that controls ADXL345 chip
type ADXL345 struct {
	printer                  *Printer
	query_rate               int
	axes_map                 [][]float64
	data_rate                int
	lock                     sync.Mutex
	raw_samples              []map[string]interface{}
	spi                      *MCU_SPI
	mcu                      *MCU
	oid                      int
	query_adxl345_cmd        *CommandWrapper
	query_adxl345_end_cmd    *CommandQueryWrapper
	query_adxl345_status_cmd *CommandQueryWrapper
	last_sequence            int
	max_query_duration       int64
	last_limit_count         int
	last_error_count         int
	clock_sync               *ClockSyncRegression
	api_dump                 *APIDumpHelper
	name                     string
}

func NewADXL345(config *ConfigWrapper) *ADXL345 {
	self := new(ADXL345)
	self.printer = config.Get_printer()
	NewAccelCommandHelper(config, self)
	self.query_rate = 0
	am := map[string][]float64{
		"x": {0, ADXL345_INFO["SCALE_XY"].(float64)}, "y": {1, ADXL345_INFO["SCALE_XY"].(float64)}, "z": {2, ADXL345_INFO["SCALE_Z"].(float64)},
		"-x": {0, -ADXL345_INFO["SCALE_XY"].(float64)}, "-y": {1, -ADXL345_INFO["SCALE_XY"].(float64)}, "-z": {2, -ADXL345_INFO["SCALE_Z"].(float64)},
	}
	axes_map := config.Getlist("axes_map", []string{"x", "y", "z"}, ",", 3, true).([]string)
	self.axes_map = make([][]float64, len(axes_map))
	for i, v := range axes_map {
		if _, ok := am[v]; !ok {
			panic(errors.New("Invalid adxl345 axes_map parameter"))
		}
		self.axes_map[i] = am[strings.TrimSpace(v)]
	}

	self.data_rate = config.Getint("rate", 3200, 0, 0, true)
	if _, ok := ADXL345_QUERY_RATES[self.data_rate]; !ok {
		panic(errors.New("Invalid adxl345 axes_map parameter"))
	}
	// Measurement storage (accessed from background thread)
	self.lock = sync.Mutex{}
	self.raw_samples = make([]map[string]interface{}, 0)
	// Setup mcu sensor_adxl345 bulk query code
	var err error
	self.spi, err = MCU_SPI_from_config(config, 3, "cs_pin", 5000000, nil, false)
	if err != nil {
		panic(fmt.Errorf("MCU_SPI_from_config error: %v", err))
	}
	self.mcu = self.spi.get_mcu()
	mcu := self.mcu
	self.oid = mcu.Create_oid()
	oid := self.oid
	self.query_adxl345_cmd, self.query_adxl345_end_cmd = nil, nil
	self.query_adxl345_status_cmd = nil
	mcu.Add_config_cmd(fmt.Sprintf("config_adxl345 oid=%d spi_oid=%d",
		oid, self.spi.Get_oid()), false, false)
	mcu.Add_config_cmd(fmt.Sprintf("query_adxl345 oid=%d clock=0 rest_ticks=0",
		oid), false, true)
	mcu.Register_config_callback(self.Build_config)
	mcu.Register_response(self._handle_adxl345_data, "adxl345_data", oid)
	// Clock tracking
	self.last_sequence, self.max_query_duration = 0, 0
	self.last_limit_count, self.last_error_count = 0, 0
	self.clock_sync = NewClockSyncRegression(self.mcu, 640, 1./20.)
	// API server endpoints
	self.api_dump = NewAPIDumpHelper(
		self.printer, self._api_update, self._api_startstop, 0.100)
	self.name = str.LastName(config.Get_name())
	wh := MustLookupWebhooks(self.printer)
	if err != nil {
		logger.Error(err)
	}
	wh.Register_mux_endpoint("adxl345/dump_adxl345", "sensor", self.name, self._handle_dump_adxl345)
	return self
}

func (self *ADXL345) Build_config() {
	cmdqueue := self.spi.get_command_queue()
	self.query_adxl345_cmd, _ = self.mcu.Lookup_command(
		"query_adxl345 oid=%c clock=%u rest_ticks=%u", cmdqueue)
	self.query_adxl345_end_cmd = self.mcu.Lookup_query_command(
		"query_adxl345 oid=%c clock=%u rest_ticks=%u",
		"adxl345_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"+
			" buffered=%c fifo=%c limit_count=%hu", self.oid, cmdqueue, false)
	self.query_adxl345_status_cmd = self.mcu.Lookup_query_command(
		"query_adxl345_status oid=%c",
		"adxl345_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"+
			" buffered=%c fifo=%c limit_count=%hu", self.oid, cmdqueue, false)

}

func (self *ADXL345) Read_reg(reg int) byte {
	params := self.spi.Spi_transfer([]int{reg | ADXL345_REGISTERS["REG_MOD_READ"], 0x00}, 0, 0)
	response := params.(map[string]interface{})["response"].([]int)
	return byte(response[1])
}

func (self *ADXL345) Set_reg(reg, val int, minclock int64) error {
	self.spi.Spi_send([]int{reg, val & 0xFF}, minclock, 0)
	stored_val := self.Read_reg(reg)
	if int(stored_val) != val {
		panic(fmt.Errorf("Failed to set ADXL345 register [0x%x] to 0x%x: got 0x%x. "+
			"This is generally indicative of connection problems "+
			"(e.g. faulty wiring) or a faulty adxl345 chip.",
			reg, val, stored_val))
	}
	return nil
}

// Measurement collection
func (self *ADXL345) Is_measuring() bool {
	return self.query_rate > 0
}

func (self *ADXL345) _handle_adxl345_data(params map[string]interface{}) error {
	self.lock.Lock()
	defer self.lock.Unlock()
	self.raw_samples = append(self.raw_samples, params)
	return nil
}

func (self *ADXL345) Extract_samples(raw_samples []map[string]interface{}) [][]float64 {
	// Load variables to optimize inner loop below
	x_pos := self.axes_map[0][0]
	x_scale := self.axes_map[0][1]
	y_pos := self.axes_map[1][0]
	y_scale := self.axes_map[1][1]
	z_pos := self.axes_map[2][0]
	z_scale := self.axes_map[2][1]
	last_sequence := self.last_sequence
	time_base, chip_base, inv_freq := self.clock_sync.Get_time_translation()
	// Process every message in raw_samples
	count := 0
	seq := 0
	var samples [][]float64
	for i := 0; i < len(raw_samples)*int(ADXL345_CLK["SAMPLES_PER_BLOCK"]); i++ {
		samples = append(samples, nil)
	}
	var i = 0
	for _, params := range raw_samples {
		seq_diff := (last_sequence - int(params["sequence"].(int64))) & 0xffff
		seq_diff -= (seq_diff & 0x8000) << 1
		seq = last_sequence - seq_diff
		d := params["data"].([]int)
		msg_cdiff := float64(seq)*float64(ADXL345_CLK["SAMPLES_PER_BLOCK"]) - chip_base
		for i = 0; i < len(d)/int(ADXL345_CLK["BYTES_PER_SAMPLE"]); i++ {
			d_xyz := d[i*int(ADXL345_CLK["BYTES_PER_SAMPLE"]) : (i+1)*int(ADXL345_CLK["BYTES_PER_SAMPLE"])]
			xlow := d_xyz[0]
			ylow := d_xyz[1]
			zlow := d_xyz[2]
			xzhigh := d_xyz[3]
			yzhigh := d_xyz[4]
			if yzhigh&0x80 != 0 {
				self.last_error_count += 1
				continue
			}
			rx := (xlow | ((xzhigh & 0x1f) << 8)) - ((xzhigh & 0x10) << 9)
			ry := (ylow | ((yzhigh & 0x1f) << 8)) - ((yzhigh & 0x10) << 9)
			rz := (zlow | ((xzhigh & 0xe0) << 3) | ((yzhigh & 0xe0) << 6)) -
				((yzhigh & 0x40) << 7)
			raw_xyz := []int{rx, ry, rz}
			x := maths.Round(float64(raw_xyz[int(x_pos)])*x_scale, 6)
			y := maths.Round(float64(raw_xyz[int(y_pos)])*y_scale, 6)
			z := maths.Round(float64(raw_xyz[int(z_pos)])*z_scale, 6)
			ptime := maths.Round(time_base+(msg_cdiff+float64(i))*inv_freq, 6)
			samples[count] = []float64{ptime, x, y, z}
			count += 1
		}
	}
	self.clock_sync.Set_last_chip_clock(float64(seq*int(ADXL345_CLK["SAMPLES_PER_BLOCK"]) + i))
	// del samples[count:]
	return samples[0:count]
}

func (self *ADXL345) _update_clock(minclock int64) error {
	// Query current state
	var fifo int
	// Query current state
	var isDone = false
	var params map[string]interface{}
	for retry := 0; retry < 5; retry++ {
		params = self.query_adxl345_status_cmd.Send([]int64{int64(self.oid)}, minclock, 0).(map[string]interface{})
		if params != nil {
			fifo = int(params["fifo"].(int64)) & 0x7f
			if fifo <= 32 {
				isDone = true
				break
			}
		}
	}

	if !isDone {
		panic(errors.New("Unable to query adxl345 fifo"))
	}
	mcu_clock := self.mcu.Clock32_to_clock64(params["clock"].(int64))
	sequence := self.last_sequence&(^0xffff) | int(params["next_sequence"].(int64))
	if sequence < self.last_sequence {
		sequence += 0x10000
	}
	self.last_sequence = sequence
	buffered := int(params["buffered"].(int64))
	limit_count := self.last_limit_count&(^0xffff) | int(params["limit_count"].(int64))
	if limit_count < self.last_limit_count {
		limit_count += 0x10000
	}
	self.last_limit_count = limit_count
	duration := params["query_ticks"].(int64)
	if duration > self.max_query_duration {
		// Skip measurement as a high query time could skew clock tracking
		self.max_query_duration = int64(math.Max(float64(2*self.max_query_duration),
			float64(self.mcu.Seconds_to_clock(.000005))))
		return nil
	}
	self.max_query_duration = 2 * duration
	msg_count := sequence*int(ADXL345_CLK["SAMPLES_PER_BLOCK"]) + buffered/int(ADXL345_CLK["BYTES_PER_SAMPLE"]) + fifo
	// The "chip clock" is the message counter plus .5 for average
	// inaccuracy of query responses and plus .5 for assumed offset
	// of adxl345 hw processing time.
	chip_clock := msg_count + 1
	self.clock_sync.Update(float64(int64(mcu_clock)+duration/2), float64(chip_clock))
	return nil
}

func (self *ADXL345) _start_measurements() error {
	if self.Is_measuring() {
		return nil
	}
	// In case of miswiring, testing ADXL345 device ID prevents treating
	// noise or wrong signal as a correctly initialized device
	dev_id := self.Read_reg(ADXL345_REGISTERS["REG_DEVID"])
	if dev_id != byte(ADXL345_INFO["DEV_ID"].(int)) {
		panic(fmt.Errorf("Invalid adxl345 id (got %x vs %x).\n"+
			"This is generally indicative of connection problems\n"+
			"(e.g. faulty wiring) or a faulty adxl345 chip.",
			dev_id, byte(ADXL345_INFO["DEV_ID"].(int))))
	}
	// Setup chip in requested query rate
	self.Set_reg(ADXL345_REGISTERS["REG_POWER_CTL"], 0x00, 0)
	self.Set_reg(ADXL345_REGISTERS["REG_DATA_FORMAT"], 0x0B, 0)
	self.Set_reg(ADXL345_REGISTERS["REG_FIFO_CTL"], 0x00, 0)
	self.Set_reg(ADXL345_REGISTERS["REG_BW_RATE"], ADXL345_QUERY_RATES[self.data_rate], 0)
	self.Set_reg(ADXL345_REGISTERS["REG_FIFO_CTL"], ADXL345_INFO["SET_FIFO_CTL"].(int), 0)
	// Setup samples
	self.lock.Lock()
	self.raw_samples = make([]map[string]interface{}, 0)
	self.lock.Unlock()
	// Start bulk reading
	systime := self.printer.Get_reactor().Monotonic()
	print_time := self.mcu.Estimated_print_time(systime) + ADXL345_CLK["MIN_MSG_TIME"]
	reqclock := self.mcu.Print_time_to_clock(print_time)
	rest_ticks := self.mcu.Seconds_to_clock(4. / float64(self.data_rate))
	self.query_rate = self.data_rate
	self.query_adxl345_cmd.Send([]int64{int64(self.oid), reqclock, rest_ticks}, 0, reqclock)
	logger.Debugf("ADXL345 starting '%s' measurements", self.name)
	// Initialize clock tracking
	self.last_sequence = 0
	self.last_limit_count, self.last_error_count = 0, 0
	self.clock_sync.Reset(float64(reqclock), 0)
	self.max_query_duration = 1 << 31
	self._update_clock(reqclock)
	self.max_query_duration = 1 << 31
	return nil
}

func (self *ADXL345) _finish_measurements() {
	if !self.Is_measuring() {
		return
	}
	// Halt bulk reading
	self.query_adxl345_end_cmd.Send([]int64{int64(self.oid), 0, 0}, 0, 0)
	self.query_rate = 0
	self.lock.Lock()
	self.raw_samples = make([]map[string]interface{}, 0)
	self.lock.Unlock()
	logger.Debugf("ADXL345 finished '%s' measurements", self.name)
}

// API interface
func (self *ADXL345) _api_update(eventtime float64) map[string]interface{} {
	self._update_clock(0)
	self.lock.Lock()
	raw_samples := self.raw_samples
	self.raw_samples = make([]map[string]interface{}, 0)
	self.lock.Unlock()
	if len(raw_samples) == 0 {
		return map[string]interface{}{}
	}
	samples := self.Extract_samples(raw_samples)
	if len(samples) == 0 {
		fmt.Print("len(samples) = 0 ")
		return map[string]interface{}{}
	}
	return map[string]interface{}{
		"data": samples, "errors": self.last_error_count,
		"overflows": self.last_limit_count,
	}

}

func (self *ADXL345) _api_startstop(is_start bool) {
	if is_start {
		self._start_measurements()
	} else {
		self._finish_measurements()
	}
}

func (self *ADXL345) _handle_dump_adxl345(web_request *WebRequest) {
	self.api_dump.add_client(web_request)
	hdr := []string{"time", "x_acceleration", "y_acceleration", "z_acceleration"}
	web_request.Send(map[string][]string{"header": hdr})

}

func (self *ADXL345) Start_internal_client() IAclient {
	cconn := self.api_dump.add_internal_client()
	return NewAccelQueryHelper(self.printer, cconn)
}

func (self *ADXL345) Get_name() string {
	return self.name
}
func Load_config_ADXL345(config *ConfigWrapper) interface{} {
	return NewADXL345(config)
}

func Load_config_prefixg_ADXL345(config *ConfigWrapper) *ADXL345 {
	return NewADXL345(config)
}
