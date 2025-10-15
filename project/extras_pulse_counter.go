package project

import (
	"fmt"
	"k3c/common/utils/object"
)

// MCU_counter
type MCU_counter struct {
	mcu         *MCU
	oid         int
	pin         string
	pullup      int
	poll_time   float64
	poll_ticks  int64
	sample_time float64
	callback    interface{}
	last_count  int64
}

func NewMCUCounter(printer *Printer, pin string, sample_time float64, poll_time float64) *MCU_counter {
	ppin := printer.Lookup_object("pins", object.Sentinel{})
	pinParams := ppin.(*PrinterPins).Lookup_pin(pin, true, false, nil)
	self := new(MCU_counter)
	self.mcu = pinParams["chip"].(*MCU)
	self.oid = self.mcu.Create_oid()
	self.pin = pinParams["pin"].(string)
	self.pullup = pinParams["pullup"].(int)
	self.poll_time = poll_time
	self.poll_ticks = 0
	self.sample_time = sample_time
	self.callback = nil
	self.last_count = 0
	self.mcu.Register_config_callback(self.Build_config)
	return self
}

func (self *MCU_counter) Build_config() {
	self.mcu.Add_config_cmd(fmt.Sprintf("config_counter oid=%d pin=%v pull_up=%v",
		self.oid, self.pin, self.pullup), false, false)
	clock := self.mcu.Get_query_slot(self.oid)
	self.poll_ticks = self.mcu.Seconds_to_clock(self.poll_time)
	sampleTicks := self.mcu.Seconds_to_clock(self.sample_time)
	self.mcu.Add_config_cmd(fmt.Sprintf("query_counter oid=%d clock=%d poll_ticks=%d sample_ticks=%d",
		self.oid, clock, self.poll_ticks, sampleTicks), true, false)
	self.mcu.Register_response(self._handle_counter_state, "counter_state", self.oid)
}

// # Callback is called periodically every sample_time
func (self *MCU_counter) Setup_callback(cb interface{}) {
	self.callback = cb
}

func (self *MCU_counter) _handle_counter_state(params map[string]interface{}) error {
	nextClock := self.mcu.Clock32_to_clock64(params["next_clock"].(int64))
	time := self.mcu.Clock_to_print_time(nextClock - self.poll_ticks)
	count_clock := self.mcu.Clock32_to_clock64(params["count_clock"].(int64))
	count_time := self.mcu.Clock_to_print_time(count_clock)
	// handle 32-bit counter overflow
	last_count := self.last_count
	deltaCount := (params["count"].(int64) - last_count) & 0xffffffff
	count := last_count + deltaCount
	self.last_count = count
	if self.callback != nil {
		self.callback.(func(float64, int64, float64))(time, count, count_time)
	}
	return nil
}

type FrequencyCounter struct {
	callback   interface{}
	last_time  float64
	last_count int64
	freq       float64
	counter    *MCU_counter
}

func NewFrequencyCounter(printer *Printer, pin string, sample_time float64, poll_time float64) *FrequencyCounter {
	self := new(FrequencyCounter)
	self.callback = nil
	self.last_time, self.last_count = 0, 0
	self.freq = 0.
	self.counter = NewMCUCounter(printer, pin, sample_time, poll_time)
	self.counter.Setup_callback(self.counter_callback)
	return self
}

func (self *FrequencyCounter) counter_callback(time float64, count int64, count_time float64) {
	if self.last_time == 0 { //  First sample
		self.last_time = time
	} else {
		delta_time := count_time - self.last_time
		if delta_time > 0 {
			self.last_time = count_time
			deltaCount := count - self.last_count
			self.freq = float64(deltaCount) / delta_time
		} else { // No counts since last sample
			self.last_time = time
			self.freq = 0.0
		}
		if self.callback != nil {
			self.callback.(func(float64, float64))(time, self.freq)
		}
	}
	self.last_count = count
}

func (self *FrequencyCounter) Get_frequency() float64 {
	return self.freq
}
