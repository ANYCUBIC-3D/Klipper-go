package project

import (
	"k3c/common/logger"
	"k3c/common/utils/object"
)

type FilamentTracker struct {
	printer          *Printer
	reactor          IReactor
	gcode            *GCodeDispatch
	detect_adc       *MCU_adc
	encoder_adc      *MCU_adc
	block_pin_state  SigalStatus
	break_pin_state  SigalStatus
	filament_present int
	safe_unwind_len  float64
	length_per_pulse float64
	signal_type      string
	trackerStatus    *FilamentTrackerStatus
	lastPosMap       map[string]float64
	callback         func(float64, int)
}

type FilamentTrackerStatus struct {
	Detection_length     float64 `json:"detection_length"`
	Filament_present     int     `json:"filament_present"`
	Encoder_pulse        int     `json:"encoder_pulse"`
	Encoder_signal_state int     `json:"encoder_signal_state"`
}

type SigalStatus struct {
	raw_adc_value    float64
	filteredAdcValue float64
	event_time       float64
}

var (
	ADC_REPORT_TIME   = 0.005
	ADC_SAMPLE_TIME   = 0.001
	ADC_SAMPLE_COUNT  = 31
	ADC_REFER_VOLTAGE = 0.70
)

func NewFilamentTracker(config *ConfigWrapper) *FilamentTracker {
	self := new(FilamentTracker)
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{})
	self.gcode = gcode.(*GCodeDispatch)

	self.trackerStatus = new(FilamentTrackerStatus)

	self.trackerStatus.Filament_present = 0
	self.trackerStatus.Encoder_pulse = 0
	self.trackerStatus.Encoder_signal_state = 0
	self.trackerStatus.Detection_length = -1.0

	self.lastPosMap = make(map[string]float64)

	self.block_pin_state.event_time = 0
	self.block_pin_state.raw_adc_value = 0
	self.block_pin_state.filteredAdcValue = 0

	self.break_pin_state.event_time = 0
	self.break_pin_state.raw_adc_value = 0
	self.break_pin_state.filteredAdcValue = 0

	break_pin := config.Get("tracker_break_pin", nil, true).(string)
	block_pin := config.Get("tracker_block_pin", nil, true).(string)
	self.safe_unwind_len = config.Getfloat("safe_unwind_len", 100.0, 0, 0, 0, 0, true)
	self.length_per_pulse = config.Getfloat("length_per_pulse", 0.0, 0, 0, 0, 0, true)
	self.signal_type = config.Get("signal_type", "gpio", true).(string)
	if break_pin == "" || block_pin == "" {
		return self
	}

	if self.signal_type == "adc" {
		ppins := self.printer.Lookup_object("pins", object.Sentinel{})
		self.detect_adc = ppins.(*PrinterPins).Setup_pin("adc", break_pin).(*MCU_adc)
		self.detect_adc.Setup_adc_callback(ADC_REPORT_TIME, self._break_button_adc_handler)
		self.detect_adc.Setup_minmax(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0, 1, 0)
		self.encoder_adc = ppins.(*PrinterPins).Setup_pin("adc", block_pin).(*MCU_adc)
		self.encoder_adc.Setup_adc_callback(ADC_REPORT_TIME, self._block_button_adc_handler)
		self.encoder_adc.Setup_minmax(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0, 1, 0)
	} else if self.signal_type == "gpio" {
		buttons := self.printer.Load_object(config, "buttons", object.Sentinel{})
		buttons.(*PrinterButtons).Register_buttons([]string{break_pin, block_pin}, self._break_button_io_handler)
	}

	return self
}

func (self *FilamentTracker) _note_filament_present(is_filament_present int) {

	if is_filament_present == self.trackerStatus.Filament_present {
		return
	}
	self.trackerStatus.Filament_present = is_filament_present
	if self.callback != nil {
		self.callback(self.reactor.Monotonic(), is_filament_present)
	}

}

// Is_filament_present
func (self *FilamentTracker) Is_filament_present() bool {
	return self.trackerStatus.Filament_present == 1
}

func (self *FilamentTracker) _break_button_adc_handler(read_time, read_value float64) {

	self.break_pin_state.raw_adc_value = read_value
	self.break_pin_state.event_time = read_time
	self.update_tracker_state(read_time)
}

func (self *FilamentTracker) _block_button_adc_handler(read_time, read_value float64) {
	self.block_pin_state.raw_adc_value = read_value
	self.block_pin_state.event_time = read_time
	self.update_tracker_state(read_time)
}

func (self *FilamentTracker) Register_callback(callback func(float64, int)) {
	self.callback = callback
}
func (self *FilamentTracker) update_tracker_state(eventtime float64) {
	//If the voltages of both pins are greater than 0.91, it means that there is no filament.
	if self.block_pin_state.raw_adc_value > ADC_REFER_VOLTAGE && self.break_pin_state.raw_adc_value > ADC_REFER_VOLTAGE {
		self._note_filament_present(0)
	} else {
		self._note_filament_present(1)
	}

	encoder_state := 0
	if self.block_pin_state.raw_adc_value > ADC_REFER_VOLTAGE {
		encoder_state = 1
	}
	if self.trackerStatus.Encoder_signal_state != encoder_state {
		self.trackerStatus.Encoder_signal_state = encoder_state
		self.trackerStatus.Encoder_pulse++
		if self.trackerStatus.Encoder_pulse%20 == 0 {
			logger.Debug("filament tracker encoder pulse:", self.trackerStatus.Encoder_pulse)
		}
	}
}

func (self *FilamentTracker) _break_button_io_handler(eventtime float64, state int) {
	current_state := 0
	if state == 0 {
		current_state = 0
	} else {
		current_state = 1
	}
	if self.filament_present != current_state {
		logger.Debug("filament state gpio:", state)
		self.filament_present = current_state
		if self.callback != nil {
			self.callback(eventtime, current_state)
		}
	}
}

func (self *FilamentTracker) Get_safe_unwind_len() int {
	return int(self.safe_unwind_len)
}
func (self *FilamentTracker) Get_detection_length() float64 {
	return self.trackerStatus.Detection_length
}

func (self *FilamentTracker) StartPosRecord(label string) {
	self.lastPosMap[label] = self.trackerStatus.Detection_length
}

func (self *FilamentTracker) GetPosRecord(label string) float64 {
	return self.trackerStatus.Detection_length - self.lastPosMap[label]
}

func (self *FilamentTracker) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"detection_length":     self.trackerStatus.Detection_length,
		"filament_present":     self.trackerStatus.Filament_present,
		"encoder_pulse":        self.trackerStatus.Encoder_pulse,
		"encoder_signal_state": self.trackerStatus.Encoder_signal_state,
	}
}

func Load_config_filament_tracker(config *ConfigWrapper) interface{} {
	return NewFilamentTracker(config)
}
