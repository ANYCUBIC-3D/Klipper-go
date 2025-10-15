package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"log"
	"math"
	"strings"
	"sync"
)

// LIS2DW12 registers
var LIS2DW12_REGISTERS = map[string]int{
	"REG_DEVID":     0x0F,
	"REG_CTRL1":     0x20,
	"REG_CTRL6":     0x25,
	"REG_FIFO_CTRL": 0x2E,
	"REG_MOD_READ":  0x80,
}

var LIS2DW12_QUERY_RATES = map[int]int{
	25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400: 0x7,
	800: 0x8, 1600: 0x9,
}

var LIS2DW12_CLK = map[string]float64{
	"MIN_MSG_TIME":      0.100,
	"BYTES_PER_SAMPLE":  6,
	"SAMPLES_PER_BLOCK": 8,
}

var LIS2DW12_INFO = map[string]interface{}{
	"DEV_ID":           0x44,
	"POWER_OFF":        0x00,
	"SET_CTRL1_MODE":   0x04,
	"SET_FIFO_CTL":     0xC0,
	"SET_CTRL6_ODR_FS": 0x04,
	"FREEFALL_ACCEL":   9.80665 * 1000.,
	"SCALE_XY":         0.000244140625 * 9.80665 * 1000, // 1 / 4096 (at 3.3V) mg/LSB,
	"SCALE_Z":          0.000244140625 * 9.80665 * 1000, // 1 / 4096 (at 3.3V) mg/LSB,
}

// Printer class that controls LIS2DW12 chip
type LIS2DW12 struct {
	printer                 *Printer
	query_rate              int
	axes_map                [][]float64
	data_rate               int
	lock                    sync.Mutex
	raw_samples             []map[string]interface{}
	spi                     *MCU_SPI
	mcu                     *MCU
	oid                     int
	query_sensor_cmd        *CommandWrapper
	query_sensor_end_cmd    *CommandQueryWrapper
	query_sensor_status_cmd *CommandQueryWrapper
	last_sequence           int
	max_query_duration      int64
	last_limit_count        int
	last_error_count        int
	clock_sync              *ClockSyncRegression
	api_dump                *APIDumpHelper
	name                    string
}

func NewLIS2DW12(config *ConfigWrapper) *LIS2DW12 {
	self := new(LIS2DW12)
	self.printer = config.Get_printer()
	NewAccelCommandHelper(config, self)
	self.query_rate = 0
	am := map[string][]float64{
		"x": {0, LIS2DW12_INFO["SCALE_XY"].(float64)}, "y": {1, LIS2DW12_INFO["SCALE_XY"].(float64)}, "z": {2, LIS2DW12_INFO["SCALE_Z"].(float64)},
		"-x": {0, -LIS2DW12_INFO["SCALE_XY"].(float64)}, "-y": {1, -LIS2DW12_INFO["SCALE_XY"].(float64)}, "-z": {2, -LIS2DW12_INFO["SCALE_Z"].(float64)},
	}
	axes_map := config.Getlist("axes_map", []interface{}{"x", "y", "z"}, ",", 3, true)

	self.axes_map = make([][]float64, 3)
	for i, v := range axes_map.([]interface{}) {
		if _, ok := am[v.(string)]; !ok {
			panic(errors.New("Invalid lis2dw12 axes_map parameter"))
		}
		self.axes_map[i] = am[strings.TrimSpace(v.(string))]
	}

	self.data_rate = config.Getint("rate", 1600, 0, 0, true)
	if _, ok := LIS2DW12_QUERY_RATES[self.data_rate]; !ok {
		panic(errors.New("Invalid lis2dw12 axes_map parameter"))
	}
	// Measurement storage (accessed from background thread)
	self.lock = sync.Mutex{}
	self.raw_samples = make([]map[string]interface{}, 0)
	// Setup mcu sensor_sensor bulk query code
	var err error
	self.spi, err = MCU_SPI_from_config(config, 3, "cs_pin", 5000000, nil, false)
	if err != nil {
		panic(fmt.Errorf("MCU_SPI_from_config error: %v", err))
	}
	self.mcu = self.spi.get_mcu()
	mcu := self.mcu
	self.oid = mcu.Create_oid()
	oid := self.oid
	self.query_sensor_cmd, self.query_sensor_end_cmd = nil, nil
	self.query_sensor_status_cmd = nil
	mcu.Add_config_cmd(fmt.Sprintf("config_lis2dw12 oid=%d spi_oid=%d",
		oid, self.spi.Get_oid()), false, false)
	mcu.Add_config_cmd(fmt.Sprintf("query_lis2dw12 oid=%d clock=0 rest_ticks=0",
		oid), false, true)
	mcu.Register_config_callback(self.Build_config)
	mcu.Register_response(self._handle_sensor_data, "lis2dw12_data", oid)
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
	wh.Register_mux_endpoint("lis2dw12/dump_lis2dw12", "sensor", self.name, self._handle_dump_lis2dw12)
	return self
}

func (self *LIS2DW12) Build_config() {
	cmdqueue := self.spi.get_command_queue()
	self.query_sensor_cmd, _ = self.mcu.Lookup_command(
		"query_lis2dw12 oid=%c clock=%u rest_ticks=%u", cmdqueue)
	self.query_sensor_end_cmd = self.mcu.Lookup_query_command(
		"query_lis2dw12 oid=%c clock=%u rest_ticks=%u",
		"lis2dw12_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"+
			" buffered=%c fifo=%c limit_count=%hu", self.oid, cmdqueue, false)
	self.query_sensor_status_cmd = self.mcu.Lookup_query_command(
		"query_lis2dw12_status oid=%c",
		"lis2dw12_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"+
			" buffered=%c fifo=%c limit_count=%hu", self.oid, cmdqueue, false)

}

func (self *LIS2DW12) Read_reg(reg int) byte {
	params := self.spi.Spi_transfer([]int{reg | LIS2DW12_REGISTERS["REG_MOD_READ"], 0x00}, 0, 0)
	response := params.(map[string]interface{})["response"].([]int)
	return byte(response[1])
}

func (self *LIS2DW12) Set_reg(reg, val int, minclock int64) error {
	self.spi.Spi_send([]int{reg, val & 0xFF}, minclock, 0)
	stored_val := self.Read_reg(reg)
	if int(stored_val) != val {
		panic(fmt.Errorf("Failed to set LIS2DW12 register [0x%x] to 0x%x: got 0x%x. "+
			"This is generally indicative of connection problems "+
			"(e.g. faulty wiring) or a faulty lis2dw12 chip.",
			reg, val, stored_val))
	}
	return nil
}

// Measurement collection
func (self *LIS2DW12) Is_measuring() bool {
	return self.query_rate > 0
}

func (self *LIS2DW12) _handle_sensor_data(params map[string]interface{}) error {
	self.lock.Lock()
	defer self.lock.Unlock()
	self.raw_samples = append(self.raw_samples, params)
	return nil
}

func (self *LIS2DW12) Extract_samples(raw_samples []map[string]interface{}) [][]float64 {
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
	for i := 0; i < len(raw_samples)*int(LIS2DW12_CLK["SAMPLES_PER_BLOCK"]); i++ {
		samples = append(samples, nil)
	}
	var i = 0
	for _, params := range raw_samples {
		seq_diff := (last_sequence - int(params["sequence"].(int64))) & 0xffff
		seq_diff -= (seq_diff & 0x8000) << 1
		seq = last_sequence - seq_diff
		d := params["data"].([]int)
		msg_cdiff := float64(seq)*float64(LIS2DW12_CLK["SAMPLES_PER_BLOCK"]) - chip_base
		for i = 0; i < len(d)/int(LIS2DW12_CLK["BYTES_PER_SAMPLE"]); i++ {
			d_xyz := d[i*int(LIS2DW12_CLK["BYTES_PER_SAMPLE"]) : (i+1)*int(LIS2DW12_CLK["BYTES_PER_SAMPLE"])]
			xlow := d_xyz[0]
			ylow := d_xyz[1]
			zlow := d_xyz[2]
			xhigh := d_xyz[3]
			yhigh := d_xyz[4]
			zhigh := d_xyz[5]

			rx := int16(xlow&0xfc | ((xhigh & 0xff) << 8))
			ry := int16(ylow&0xfc | ((yhigh & 0xff) << 8))
			rz := int16(zlow&0xfc | ((zhigh & 0xff) << 8))
			raw_xyz := []int16{rx, ry, rz}
			x := maths.Round(float64(raw_xyz[int(x_pos)])*x_scale/4, 6)
			y := maths.Round(float64(raw_xyz[int(y_pos)])*y_scale/4, 6)
			z := maths.Round(float64(raw_xyz[int(z_pos)])*z_scale/4, 6)
			ptime := maths.Round(time_base+(msg_cdiff+float64(i))*inv_freq, 6)
			samples[count] = []float64{ptime, x, y, z}
			count += 1
		}
	}
	self.clock_sync.Set_last_chip_clock(float64(seq*int(LIS2DW12_CLK["SAMPLES_PER_BLOCK"]) + i))
	// del samples[count:]
	return samples[0:count]
}

func (self *LIS2DW12) _update_clock(minclock int64) error {
	// Query current state
	var fifo int
	// Query current state
	var isDone = false
	var params map[string]interface{}
	for retry := 0; retry < 5; retry++ {
		params = self.query_sensor_status_cmd.Send([]int64{int64(self.oid)}, minclock, 0).(map[string]interface{})
		if params != nil {
			fifo = int(params["fifo"].(int64)) & 0x7f
			if fifo <= 32 {
				isDone = true
				break
			}
		}
	}

	if !isDone {
		panic(errors.New("Unable to query lis2dw12 fifo"))
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
	msg_count := sequence*int(LIS2DW12_CLK["SAMPLES_PER_BLOCK"]) + buffered/int(LIS2DW12_CLK["BYTES_PER_SAMPLE"]) + fifo
	// The "chip clock" is the message counter plus .5 for average
	// inaccuracy of query responses and plus .5 for assumed offset
	// of lis2dw12 hw processing time.
	chip_clock := msg_count + 1
	self.clock_sync.Update(float64(int64(mcu_clock)+duration/2), float64(chip_clock))
	return nil
}

func (self *LIS2DW12) _start_measurements() error {
	if self.Is_measuring() {
		return nil
	}
	// In case of miswiring, testing LIS2DW12 device ID prevents treating
	// noise or wrong signal as a correctly initialized device
	dev_id := self.Read_reg(LIS2DW12_REGISTERS["REG_DEVID"])
	if dev_id != byte(LIS2DW12_INFO["DEV_ID"].(int)) {
		panic(fmt.Errorf("Invalid lis2dw12 id (got %x vs %x).\n"+
			"This is generally indicative of connection problems\n"+
			"(e.g. faulty wiring) or a faulty lis2dw12 chip.",
			dev_id, byte(LIS2DW12_INFO["DEV_ID"].(int))))
	}

	// Setup chip in requested query rate
	self.Set_reg(LIS2DW12_REGISTERS["REG_CTRL1"], LIS2DW12_QUERY_RATES[self.data_rate]<<4|LIS2DW12_INFO["SET_CTRL1_MODE"].(int), 0)
	self.Set_reg(LIS2DW12_REGISTERS["REG_FIFO_CTRL"], 0x00, 0)
	self.Set_reg(LIS2DW12_REGISTERS["REG_CTRL6"], LIS2DW12_INFO["SET_CTRL6_ODR_FS"].(int), 0)
	self.Set_reg(LIS2DW12_REGISTERS["REG_FIFO_CTRL"], LIS2DW12_INFO["SET_FIFO_CTL"].(int), 0)

	// Setup samples
	self.lock.Lock()
	self.raw_samples = make([]map[string]interface{}, 0)
	self.lock.Unlock()
	// Start bulk reading
	systime := self.printer.Get_reactor().Monotonic()
	print_time := self.mcu.Estimated_print_time(systime) + LIS2DW12_CLK["MIN_MSG_TIME"]
	reqclock := self.mcu.Print_time_to_clock(print_time)
	rest_ticks := self.mcu.Seconds_to_clock(4. / float64(self.data_rate))
	self.query_rate = self.data_rate
	self.query_sensor_cmd.Send([]int64{int64(self.oid), reqclock, rest_ticks}, 0, reqclock)
	log.Printf("LIS2DW12 starting '%s' measurements", self.name)
	// Initialize clock tracking
	self.last_sequence = 0
	self.last_limit_count, self.last_error_count = 0, 0
	self.clock_sync.Reset(float64(reqclock), 0)
	self.max_query_duration = 1 << 31
	self._update_clock(reqclock)
	self.max_query_duration = 1 << 31
	return nil
}

func (self *LIS2DW12) _finish_measurements() {
	if !self.Is_measuring() {
		return
	}
	// Halt bulk reading
	self.query_sensor_end_cmd.Send([]int64{int64(self.oid), 0, 0}, 0, 0)
	self.query_rate = 0
	self.lock.Lock()
	self.raw_samples = make([]map[string]interface{}, 0)
	self.lock.Unlock()
	log.Println("LIS2DW12 finished '%s' measurements", self.name)
}

// API interface
func (self *LIS2DW12) _api_update(eventtime float64) map[string]interface{} {
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

func (self *LIS2DW12) _api_startstop(is_start bool) {
	if is_start {
		self._start_measurements()
	} else {
		self._finish_measurements()
	}
}

func (self *LIS2DW12) _handle_dump_lis2dw12(web_request *WebRequest) {
	self.api_dump.add_client(web_request)
	hdr := []string{"time", "x_acceleration", "y_acceleration", "z_acceleration"}
	web_request.Send(map[string][]string{"header": hdr})

}

func (self *LIS2DW12) Start_internal_client() IAclient {
	cconn := self.api_dump.add_internal_client()
	return NewAccelQueryHelper(self.printer, cconn)
}

func (self *LIS2DW12) Get_name() string {
	return self.name
}
func Load_config_LIS2DW12(config *ConfigWrapper) interface{} {
	return NewLIS2DW12(config)
}

func Load_config_prefixg_LIS2DW12(config *ConfigWrapper) *LIS2DW12 {
	return NewLIS2DW12(config)
}
