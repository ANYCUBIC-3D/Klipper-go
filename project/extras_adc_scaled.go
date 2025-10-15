package project

import (
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"math"
	"strings"
)

type MCU_scaled_adc struct {
	_main        *PrinterADCScaled
	_last_state  [2]float64
	_mcu_adc     *MCU_adc
	_callback    func(float64, float64)
	setup_minmax func(sample_time float64, sample_count int, minval float64, maxval float64, range_check_count int)
	get_mcu      func() *MCU
}

func NewMCU_scaled_adc(main *PrinterADCScaled, pin_params map[string]interface{}) *MCU_scaled_adc {
	self := new(MCU_scaled_adc)
	self._main = main
	self._last_state = [2]float64{0, 0}
	self._mcu_adc = main.mcu.Setup_pin("adc", pin_params).(*MCU_adc)
	obj := main.printer.Lookup_object("query_adc", object.Sentinel{})
	query_adc := obj.(*QueryADC)

	qname := main.name + ":" + cast.ToString(pin_params["pin"])
	query_adc.Register_adc(qname, self._mcu_adc)
	self._callback = nil
	self.setup_minmax = self._mcu_adc.Setup_minmax
	self.get_mcu = self._mcu_adc.Get_mcu
	return self
}

func (self *MCU_scaled_adc) _handle_callback(read_time, read_value float64) {
	max_adc := self._main.last_vref[1]
	min_adc := self._main.last_vssa[1]
	scaled_val := (read_value - min_adc) / (max_adc - min_adc)
	self._last_state = [2]float64{scaled_val, read_time}
	self._callback(read_time, scaled_val)
}

func (self *MCU_scaled_adc) setup_adc_callback(report_time float64, callback func(float64, float64)) {
	self._callback = callback
	self._mcu_adc.Setup_adc_callback(report_time, self._handle_callback)
}

func (self *MCU_scaled_adc) get_last_value() [2]float64 {
	return self._last_state
}

type PrinterADCScaled struct {
	printer         *Printer
	name            string
	last_vref       [2]float64
	last_vssa       [2]float64
	mcu_vref        *MCU_adc
	mcu_vssa        *MCU_adc
	inv_smooth_time float64
	mcu             *MCU
}

func NewPrinterADCScaled(config *ConfigWrapper) *PrinterADCScaled {
	self := new(PrinterADCScaled)
	self.printer = config.Get_printer()
	self.name = strings.Split(config.Get_name(), " ")[1]
	self.last_vref = [2]float64{0., 0.}
	self.last_vssa = [2]float64{0., 0.}
	// Configure vref and vssa pins
	self.mcu_vref = self._config_pin(config, "vref", self.Vref_callback).(*MCU_adc)
	self.mcu_vssa = self._config_pin(config, "vssa", self.Vssa_callback).(*MCU_adc)
	smooth_time := config.Getfloat("smooth_time", 2., 0, 0, 0, 0, true)
	self.inv_smooth_time = 1. / smooth_time
	self.mcu = self.mcu_vref.Get_mcu()

	if self.mcu != self.mcu_vssa.Get_mcu() {
		panic("vref and vssa must be on same mcu")
	}

	// Register setup_pin
	obj := self.printer.Lookup_object("pins", object.Sentinel{})
	ppins := obj.(*PrinterPins)
	ppins.Register_chip(self.name, self)
	return self
}

func (self *PrinterADCScaled) _config_pin(config *ConfigWrapper, name string, callback func(float64, float64)) interface{} {
	pin_name := cast.ToString(config.Get(name+"_pin", object.Sentinel{}, true))

	obj := self.printer.Lookup_object("pins", object.Sentinel{})
	ppins := obj.(*PrinterPins)

	mcu_adc := ppins.Setup_pin("adc", pin_name).(*MCU_adc)
	mcu_adc.Setup_adc_callback(REPORT_TIME, callback)
	mcu_adc.Setup_minmax(SAMPLE_TIME, SAMPLE_COUNT, 0., 1.,
		RANGE_CHECK_COUNT)

	obj = config.Get_printer().Load_object(config, "query_adc", object.Sentinel{})
	query_adc := obj.(*QueryADC)
	query_adc.Register_adc(self.name+":"+name, mcu_adc)
	return mcu_adc
}

func (self *PrinterADCScaled) Setup_pin(pin_type string, pin_params map[string]interface{}) *MCU_scaled_adc {
	if pin_type != "adc" {
		panic("adc_scaled only supports adc pins")
	}
	return NewMCU_scaled_adc(self, pin_params)
}

func (self *PrinterADCScaled) Calc_smooth(read_time, read_value float64, last [2]float64) [2]float64 {
	last_time, last_value := last[0], last[1]
	time_diff := read_time - last_time
	value_diff := read_value - last_value
	adj_time := math.Min(time_diff*self.inv_smooth_time, 1.)
	smoothed_value := last_value + value_diff*adj_time
	return [2]float64{read_time, smoothed_value}
}

func (self *PrinterADCScaled) Vref_callback(read_time, read_value float64) {
	self.last_vref = self.Calc_smooth(read_time, read_value, self.last_vref)
}

func (self *PrinterADCScaled) Vssa_callback(read_time, read_value float64) {
	self.last_vssa = self.Calc_smooth(read_time, read_value, self.last_vssa)
}

func Load_config_PrinterADCScaled(config *ConfigWrapper) interface{} {
	return NewPrinterADCScaled(config)
}
