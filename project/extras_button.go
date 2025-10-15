package project

import (
	"fmt"
	"k3c/common/utils/object"
	"math"
	"strings"
)

const QUERY_TIME = .002
const RETRANSMIT_COUNT = 50

type MCU_buttons struct {
	reactor     IReactor
	mcu         *MCU
	pin_list    [][]interface{}
	callbacks   [][]interface{}
	invert      int
	last_button int
	ack_cmd     interface{}
	ack_count   int
	oid         int
}

func NewMCU_buttons(printer *Printer, mcu *MCU) *MCU_buttons {
	self := &MCU_buttons{}
	self.reactor = printer.Get_reactor()
	self.mcu = mcu
	self.mcu.Register_config_callback(self.Build_config)
	self.pin_list = [][]interface{}{}
	self.callbacks = [][]interface{}{}
	self.invert = 0
	self.last_button = 0
	self.ack_cmd = nil
	self.ack_count = 0
	return self
}

func (self *MCU_buttons) Setup_buttons(pins []map[string]interface{}, callback func(float64, int)) {
	mask := 0
	shift := len(self.pin_list)
	for _, pinParams := range pins {
		if pinParams["invert"] == 1 {
			self.invert |= 1 << len(self.pin_list)
		}
		mask |= 1 << len(self.pin_list)
		self.pin_list = append(self.pin_list, []interface{}{pinParams["pin"], pinParams["pullup"]})
	}
	self.callbacks = append(self.callbacks, []interface{}{mask, shift, callback})
}

func (self *MCU_buttons) Build_config() {
	if len(self.pin_list) == 0 {
		return
	}
	self.oid = self.mcu.Create_oid()
	self.mcu.Add_config_cmd(fmt.Sprintf("config_buttons oid=%d button_count=%d", self.oid, len(self.pin_list)), false, false)
	for i, pin_params := range self.pin_list {
		pin := pin_params[0]
		pull_up := pin_params[1]
		self.mcu.Add_config_cmd(fmt.Sprintf("buttons_add oid=%d pos=%d pin=%s pull_up=%d", self.oid, i, pin, pull_up), true, false)
	}
	cmd_queue := self.mcu.Alloc_command_queue()
	self.ack_cmd, _ = self.mcu.Lookup_command("buttons_ack oid=%c count=%c", cmd_queue)
	clock := self.mcu.Get_query_slot(self.oid)
	restTicks := self.mcu.Seconds_to_clock(QUERY_TIME)
	self.mcu.Add_config_cmd(fmt.Sprintf("buttons_query oid=%d clock=%d rest_ticks=%d retransmit_count=%d invert=%d", self.oid, clock, restTicks, RETRANSMIT_COUNT, self.invert), true, false)
	self.mcu.Register_response(self.Handle_buttons_state, "buttons_state", self.oid)
}

func (self *MCU_buttons) Handle_buttons_state(params map[string]interface{}) error {
	// Expand the message ack_count from 8-bit
	ack_count := self.ack_count
	ack_diff := (ack_count - int(params["ack_count"].(int64))) & 0xff
	if ack_diff&0x80 != 0 {
		ack_diff -= 0x100
	}
	msg_ack_count := ack_count - ack_diff
	// Determine new buttons
	buttons := params["state"].([]int)
	new_count := msg_ack_count + len(buttons) - self.ack_count
	if new_count <= 0 {
		return nil
	}
	new_buttons := buttons[len(buttons)-new_count:]
	// Send ack to project.MCU
	self.ack_cmd.(*CommandWrapper).Send([]int64{int64(self.oid), int64(new_count)}, 0, 0)
	self.ack_count += new_count
	// Call self.handle_button() with this event in main thread
	for _, nb := range new_buttons {
		self.reactor.Register_async_callback(func(e interface{}) interface{} {
			self.Handle_button(e.(float64), int(nb))
			return nil
		}, 0)
	}
	return nil
}

func (self *MCU_buttons) Handle_button(eventtime float64, button int) {
	button ^= self.invert
	changed := button ^ self.last_button
	for _, item := range self.callbacks {
		mask := item[0].(int)
		shift := item[1].(int)
		callback := item[2].(func(float64, int))
		if changed&mask != 0 {
			callback(eventtime, (button&mask)>>shift)
		}
	}
	self.last_button = button
}

/*
######################################################################
# ADC button tracking
######################################################################
*/
type MCU_ADC_buttons struct {
	reactor           IReactor
	buttons           [][]interface{}
	last_button       *int
	last_pressed      *int
	last_debouncetime float64
	pullup            float64
	pin               string
	min_value         float64
	max_value         float64
	mcu_adc           *MCU_adc
	ADC_REPORT_TIME   float64
	ADC_DEBOUNCE_TIME float64
	ADC_SAMPLE_TIME   float64
	ADC_SAMPLE_COUNT  int
}

func NewMCU_ADC_buttons(printer *Printer, pin string, pullup float64) *MCU_ADC_buttons {
	self := &MCU_ADC_buttons{}
	self.ADC_REPORT_TIME = 0.015
	self.ADC_DEBOUNCE_TIME = 0.025
	self.ADC_SAMPLE_TIME = 0.001
	self.ADC_SAMPLE_COUNT = 6
	self.reactor = printer.Get_reactor()
	self.buttons = [][]interface{}{}
	self.last_button = nil
	self.last_pressed = nil
	self.last_debouncetime = 0
	self.pullup = pullup
	self.pin = pin
	self.min_value = 999999999999.9
	self.max_value = 0.
	ppins := printer.Lookup_object("pins", object.Sentinel{})
	self.mcu_adc = ppins.(*PrinterPins).Setup_pin("adc", self.pin).(*MCU_adc)
	self.mcu_adc.Setup_minmax(self.ADC_SAMPLE_TIME, self.ADC_SAMPLE_COUNT, 0, 1, 0)
	self.mcu_adc.Setup_adc_callback(self.ADC_REPORT_TIME, self.Adc_callback)
	query_adc := printer.Lookup_object("query_adc", object.Sentinel{}).(QueryADC)
	query_adc.Register_adc("adc_button:"+strings.TrimSpace(pin), self.mcu_adc)
	return self
}

func (self *MCU_ADC_buttons) Setup_button(min_value, max_value float64, callback interface{}) {
	self.min_value = math.Min(self.min_value, min_value)
	self.max_value = math.Max(self.max_value, max_value)
	self.buttons = append(self.buttons, []interface{}{min_value, max_value, callback})
}

func (self *MCU_ADC_buttons) Adc_callback(read_time float64, read_value float64) {
	adc := math.Max(.00001, math.Min(.99999, read_value))
	value := self.pullup * adc / (1.0 - adc)

	// Determine button pressed
	var btn *int = nil
	if self.min_value <= value && value <= self.max_value {
		for i, item := range self.buttons {
			min_value := item[0].(float64)
			max_value := item[1].(float64)
			if min_value < value && value < max_value {
				btn = &i
				break
			}
		}
	}
	// If the button changed, due to noise or pressing:
	if btn != self.last_button {
		// reset the debouncing timer
		self.last_debouncetime = read_time
	}

	// button debounce check & new button pressed
	if read_time-self.last_debouncetime > self.ADC_DEBOUNCE_TIME &&
		self.last_button == btn && self.last_pressed != btn {
		// release last_pressed
		if self.last_pressed != nil {
			self.Call_button(*self.last_pressed, false)
			self.last_pressed = nil
		}
		if btn != nil {
			self.Call_button(*btn, true)
			self.last_pressed = btn
		}
	}

	self.last_button = btn
}

func (self *MCU_ADC_buttons) Call_button(button int, state bool) {
	// minval := self.buttons[button][0].(float64)
	// maxval := self.buttons[button][1].(float64)
	callback := self.buttons[button][2].(func(eventTime float64, state bool))
	self.reactor.Register_async_callback(func(e interface{}) interface{} {
		callback(e.(float64), state)
		return nil
	}, 0)
}

/*
######################################################################
# Rotary Encoders
######################################################################

# Rotary encoder handler https://github.com/brianlow/Rotary
# Copyright 2011 Ben Buxton (bb@cactii.net).
# Licenced under the GNU GPL Version 3.
*/
type BaseRotaryEncoder struct {
	Cw_callback    interface{}
	Ccw_callback   interface{}
	Encoder_state  int
	ENCODER_STATES [][]int
	R_START        int
	R_DIR_CW       int
	R_DIR_CCW      int
	R_DIR_MSK      int
}

func NewBaseRotaryEncoder(cw_callback, ccw_callback interface{}) *BaseRotaryEncoder {
	self := &BaseRotaryEncoder{}
	self.R_START = 0x0
	self.R_DIR_CW = 0x10
	self.R_DIR_CCW = 0x20
	self.R_DIR_MSK = 0x30
	self.Cw_callback = cw_callback
	self.Ccw_callback = ccw_callback
	self.Encoder_state = self.R_START
	return self
}

func (self *BaseRotaryEncoder) Encoder_callback(eventtime float64, state int) {
	es := self.ENCODER_STATES[self.Encoder_state&0xf][state&0x3]
	self.Encoder_state = es
	if es&self.R_DIR_MSK == self.R_DIR_CW {
		self.Cw_callback.(func(float64))(eventtime)
	} else if es&self.R_DIR_MSK == self.R_DIR_CCW {
		self.Ccw_callback.(func(float64))(eventtime)
	}
}

type FullStepRotaryEncoder struct {
	BaseRotaryEncoder
	R_CW_FINAL  int
	R_CW_BEGIN  int
	R_CW_NEXT   int
	R_CCW_BEGIN int
	R_CCW_FINAL int
	R_CCW_NEXT  int
}

func NewFullStepRotaryEncoder(cw_callback, ccw_callback interface{}) *FullStepRotaryEncoder {
	self := &FullStepRotaryEncoder{}
	self.BaseRotaryEncoder = *NewBaseRotaryEncoder(cw_callback, ccw_callback)
	self.R_CW_FINAL = 0x1
	self.R_CW_BEGIN = 0x2
	self.R_CW_NEXT = 0x3
	self.R_CCW_BEGIN = 0x4
	self.R_CCW_FINAL = 0x5
	self.R_CCW_NEXT = 0x6
	// Use the full-step state table (emits a code at 00 only)
	self.ENCODER_STATES = [][]int{
		// R_START
		{self.R_START, self.R_CW_BEGIN, self.R_CCW_BEGIN, self.R_START},
		// R_CW_FINAL
		{self.R_CW_NEXT, self.R_START, self.R_CW_FINAL, self.R_START | self.R_DIR_CW},
		// R_CW_BEGIN
		{self.R_CW_NEXT, self.R_CW_BEGIN, self.R_START, self.R_START},
		// R_CW_NEXT
		{self.R_CW_NEXT, self.R_CW_BEGIN, self.R_CW_FINAL, self.R_START},
		// R_CCW_BEGIN
		{self.R_CCW_NEXT, self.R_START, self.R_CCW_BEGIN, self.R_START},
		// R_CCW_FINAL
		{self.R_CCW_NEXT, self.R_CCW_FINAL, self.R_START, self.R_START | self.R_DIR_CCW},
		// R_CCW_NEXT
		{self.R_CCW_NEXT, self.R_CCW_FINAL, self.R_CCW_BEGIN, self.R_START},
	}
	return self
}

type HalfStepRotaryEncoder struct {
	BaseRotaryEncoder
	// Use the half-step state table (emits a code at 00 and 11)
	R_CCW_BEGIN   int
	R_CW_BEGIN    int
	R_START_M     int
	R_CW_BEGIN_M  int
	R_CCW_BEGIN_M int
}

func NewHalfStepRotaryEncoder(cw_callback, ccw_callback interface{}) *HalfStepRotaryEncoder {
	self := &HalfStepRotaryEncoder{}
	self.R_CCW_BEGIN = 0x1
	self.R_CW_BEGIN = 0x2
	self.R_START_M = 0x3
	self.R_CW_BEGIN_M = 0x4
	self.R_CCW_BEGIN_M = 0x5
	self.Cw_callback = cw_callback
	self.Ccw_callback = ccw_callback
	self.ENCODER_STATES = [][]int{
		// R_START (00)
		{self.R_START_M, self.R_CW_BEGIN, self.R_CCW_BEGIN, self.R_START},
		// R_CCW_BEGIN
		{self.R_START_M | self.R_DIR_CCW, self.R_START, self.R_CCW_BEGIN, self.R_START},
		// R_CW_BEGIN
		{self.R_START_M | self.R_DIR_CW, self.R_CW_BEGIN, self.R_START, self.R_START},
		// R_START_M (11)
		{self.R_START_M, self.R_CCW_BEGIN_M, self.R_CW_BEGIN_M, self.R_START},
		// R_CW_BEGIN_M
		{self.R_START_M, self.R_START_M, self.R_CW_BEGIN_M, self.R_START | self.R_DIR_CW},
		// R_CCW_BEGIN_M
		{self.R_START_M, self.R_CCW_BEGIN_M, self.R_START_M, self.R_START | self.R_DIR_CCW},
	}
	return self
}

type PrinterButtons struct {
	Printer     *Printer
	Mcu_buttons map[string]*MCU_buttons
	Adc_buttons map[string]*MCU_ADC_buttons
}

func NewPrinterButtons(config *ConfigWrapper) *PrinterButtons {
	self := &PrinterButtons{}
	self.Printer = config.Get_printer()
	self.Printer.Load_object(config, "query_adc", object.Sentinel{})
	self.Mcu_buttons = make(map[string]*MCU_buttons)
	self.Adc_buttons = make(map[string]*MCU_ADC_buttons)
	return self
}

func (self *PrinterButtons) Register_adc_button(pin string, minVal float64, maxVal float64, pullup float64, callback func(eventTime float64, state bool)) {
	adc_buttons := self.Adc_buttons[pin]
	if adc_buttons == nil {
		adc_buttons = NewMCU_ADC_buttons(self.Printer, pin, pullup)
		self.Adc_buttons[pin] = adc_buttons
	}
	adc_buttons.Setup_button(minVal, maxVal, callback)
}

func (self *PrinterButtons) Register_adc_button_push(pin string, minVal float64, maxVal float64, pullup float64, callback func(eventTime float64)) {
	helper := func(eventTime float64, state bool) {
		if state {
			callback(eventTime)
		}
	}
	self.Register_adc_button(pin, minVal, maxVal, pullup, helper)
}

func (self *PrinterButtons) Register_buttons(pins []string, callback func(float64, int)) {
	ppins := self.Printer.Lookup_object("pins", object.Sentinel{}).(*PrinterPins)
	var mcu *MCU
	mcu_name := ""
	pin_params_list := []map[string]interface{}{}
	for _, pin := range pins {
		pin_params := ppins.Lookup_pin(pin, true, true, nil)
		if mcu != nil && pin_params["chip"] != mcu {
			panic("button pins must be on same mcu")
		}
		mcu = pin_params["chip"].(*MCU)
		mcu_name = pin_params["chip_name"].(string)
		pin_params_list = append(pin_params_list, pin_params)
	}
	// Register pins and callback with the appropriate project.MCU
	mcu_buttons := self.Mcu_buttons[mcu_name]
	if mcu_buttons == nil || len(mcu_buttons.pin_list)+len(pin_params_list) > 8 {
		mcu_buttons = NewMCU_buttons(self.Printer, mcu)
		self.Mcu_buttons[mcu_name] = mcu_buttons
	}
	mcu_buttons.Setup_buttons(pin_params_list, callback)
}
func (self *PrinterButtons) Register_rotary_encoder(pin1, pin2 string, cw_callback, ccw_callback func(), steps_per_detent int) {
	var re interface{}
	if steps_per_detent == 2 {
		re = NewHalfStepRotaryEncoder(cw_callback, ccw_callback)
	} else if steps_per_detent == 4 {
		re = NewFullStepRotaryEncoder(cw_callback, ccw_callback)
	} else {
		panic(fmt.Sprintf("%d steps per detent not supported", steps_per_detent))
	}
	self.Register_buttons([]string{pin1, pin2}, re.(*BaseRotaryEncoder).Encoder_callback)
}

func (self *PrinterButtons) Register_button_push(pin string, callback func(float64)) {
	helper := func(eventtime float64, state int) {
		if state == 1 {
			callback(eventtime)
		}
	}
	self.Register_buttons([]string{pin}, helper)
}

func Load_config_PrinterButtons(config *ConfigWrapper) interface{} {
	return NewPrinterButtons(config)
}
