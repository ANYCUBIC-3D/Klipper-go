package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/object"
	"math"
	"strings"
)

/*
# Obtain temperature using linear interpolation of ADC values
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/

/*
######################################################################
# Interface between MCU adc and heater temperature callbacks
######################################################################
*/

const (
	SAMPLE_TIME       = 0.001
	SAMPLE_COUNT      = 8
	REPORT_TIME       = 0.300
	RANGE_CHECK_COUNT = 4
)

// Interface between ADC and heater temperature callbacks
type PrinterADCtoTemperature struct {
	Adc_convert          Linear
	MCU_adc              *MCU_adc
	Temperature_callback func(read_time float64, temp float64)
}

func NewPrinterADCtoTemperature(config *ConfigWrapper, adc_convert interface{}) *PrinterADCtoTemperature {
	self := &PrinterADCtoTemperature{}
	self.Adc_convert = adc_convert.(Linear)
	ppins := config.Get_printer().Lookup_object("pins", object.Sentinel{})
	self.MCU_adc = ppins.(*PrinterPins).Setup_pin("adc",
		config.Get("sensor_pin", object.Sentinel{}, true).(string)).(*MCU_adc)
	self.MCU_adc.Setup_adc_callback(REPORT_TIME, self.Adc_callback)
	query_adc := config.Get_printer().Load_object(config, "query_adc", object.Sentinel{})
	query_adc.(*QueryADC).Register_adc(config.Get_name(), self.MCU_adc)
	return self
}
func (self *PrinterADCtoTemperature) Setup_callback(temperature_callback func(read_time float64, temp float64)) {
	self.Temperature_callback = temperature_callback
}

func (self *PrinterADCtoTemperature) Get_report_time_delta() float64 {
	return REPORT_TIME
}

func (self *PrinterADCtoTemperature) Adc_callback(read_time, read_value float64) {
	temp := self.Adc_convert.Calc_temp(read_value)
	self.Temperature_callback(read_time+SAMPLE_COUNT*SAMPLE_TIME, temp)
}

func (self *PrinterADCtoTemperature) Setup_minmax(min_temp, max_temp float64) {
	val_1 := self.Adc_convert.Calc_adc(min_temp)
	val_2 := self.Adc_convert.Calc_adc(max_temp)
	self.MCU_adc.Setup_minmax(SAMPLE_TIME, SAMPLE_COUNT,
		math.Min(val_1, val_2), math.Max(val_1, val_2), RANGE_CHECK_COUNT)
}

/*
######################################################################
# Linear interpolation
######################################################################
*/

// Helper code to perform linear interpolation
type LinearInterpolate struct {
	Keys   []float64
	Slopes [][]float64
}

func NewLinearInterpolate(samples [][]float64) (*LinearInterpolate, error) {
	self := &LinearInterpolate{}
	self.Keys = []float64{}
	self.Slopes = [][]float64{}
	var last_key, last_value interface{}
	sort_arr := samples
	for i := 0; i < len(sort_arr)-1; i++ {
		for j := 0; j < len(sort_arr)-1-i; j++ {
			if sort_arr[j][0] < sort_arr[j+1][0] {
				sort_arr[j], sort_arr[j+1] = sort_arr[j+1], sort_arr[j]
			}
		}
	}
	for _, item := range sort_arr {
		key := item[1]
		value := item[0]
		if last_value == nil {
			last_key = key
			last_value = value
			continue
		}
		if key <= last_key.(float64) {
			return nil, errors.New("duplicate value")
		}
		gain := (value - last_value.(float64)) / (key - last_key.(float64))
		offset := last_value.(float64) - last_key.(float64)*gain
		if len(self.Slopes) != 0 &&
			self.Slopes[len(self.Slopes)-1][0] == gain &&
			self.Slopes[len(self.Slopes)-1][1] == offset {
			continue
		}
		last_value = value
		last_key = key
		self.Keys = append(self.Keys, key)
		self.Slopes = append(self.Slopes, []float64{gain, offset})
	}
	if len(self.Keys) == 0 {
		return nil, errors.New("dneed at least two samples")
	}
	self.Keys = append(self.Keys, 9999999999999.)
	self.Slopes = append(self.Slopes, self.Slopes[len(self.Slopes)-1])
	return self, nil
}

func (self *LinearInterpolate) Interpolate(key float64) float64 {

	low := 0
	high := len(self.Keys) - 1

	for low <= high {
		mid := low + (high-low)/2

		if self.Keys[mid] == key {
			low = mid
			break
		} else if self.Keys[mid] < key {
			low = mid + 1
		} else {
			high = mid - 1
		}
	}
	gain := self.Slopes[low][0]
	offset := self.Slopes[low][1]
	return key*gain + offset
}

func (self *LinearInterpolate) Reverse_interpolate(value float64) float64 {
	var values []float64
	for i := 0; i < len(self.Keys); i++ {
		key := self.Keys[i]
		gain := self.Slopes[i][0]
		offset := self.Slopes[i][1]
		values = append(values, key*gain+offset)
	}
	var valid []int
	if values[0] < values[len(values)-2] {
		for i := 0; i < len(values); i++ {
			if values[i] >= value {
				valid = append(valid, i)
			}
		}
	} else {
		for i := 0; i < len(values); i++ {
			if values[i] <= value {
				valid = append(valid, i)
			}
		}
	}

	min_pos := math.Min(float64(valid[0]), float64(len(values)-1))
	gain := self.Slopes[int(min_pos)][0]
	offset := self.Slopes[int(min_pos)][1]
	return (value - offset) / gain
}

/*
######################################################################
# Linear voltage to temperature converter
######################################################################
*/

// Linear style conversion chips calibrated from temperature measurements
type LinearVoltage struct {
	Calc_temp_b func(float64) float64
	Calc_adc_b  func(float64) float64
}

func NewLinearVoltage(config *ConfigWrapper, params [][]float64) (interface{}, error) {
	self := &LinearVoltage{}
	adc_voltage := config.Getfloat("adc_voltage", 5., 0., 0., 0., 0., true)
	voltage_offset := config.Getfloat("voltage_offset", 5., 0., 0., 0., 0., true)
	sample := [][]float64{}
	for _, item := range params {
		temp := item[0]
		volt := item[1]
		adc := (volt - voltage_offset) / adc_voltage
		if adc < 0. || adc > 1. {
			logger.Debugf("Ignoring adc sample %.3f/%.3f in heater %s",
				temp, volt, config.Get_name())
			continue
		}
		sample = append(sample, []float64{adc, temp})
		li, err := NewLinearInterpolate(sample)
		if err != nil {
			return nil, errors.New(fmt.Sprintf("adc_temperature %s in heater %s",
				err, config.Get_name()))
		}
		self.Calc_temp_b = li.Interpolate
		self.Calc_adc_b = li.Reverse_interpolate
	}
	return self, nil
}

func (self *LinearVoltage) Calc_temp(val float64) float64 {
	return self.Calc_temp_b(val)
}

func (self *LinearVoltage) Calc_adc(val float64) float64 {
	return self.Calc_adc(val)
}

type ICustomLinear interface {
	Get_name() string
	Create(config *ConfigWrapper) interface{}
}

// Custom defined sensors from the config file
type CustomLinearVoltage struct {
	name   string
	params [][]float64
}

func NewCustomLinearVoltage(config *ConfigWrapper) ICustomLinear {
	self := &CustomLinearVoltage{}
	self.params = [][]float64{}
	for i := 1; i < 1000; i++ {
		t := config.GetfloatNone(fmt.Sprintf("temperature%d", i), 0., 0., 0., 0., 0., true)
		if t == nil {
			break
		}
		v := config.GetfloatNone(fmt.Sprintf("voltage%d", i), 0., 0., 0., 0., 0., true)
		self.params = append(self.params, []float64{t.(float64), v.(float64)})
	}
	return self
}
func (self *CustomLinearVoltage) Get_name() string {
	return self.name
}

func (self *CustomLinearVoltage) Create(config *ConfigWrapper) interface{} {
	lv, err := NewLinearVoltage(config, self.params)
	if err != nil {
		logger.Error(err)
	}
	return NewPrinterADCtoTemperature(config, lv)
}

type Linear interface {
	Calc_temp(adc float64) float64
	Calc_adc(temp float64) float64
}

/*
######################################################################
# Linear resistance to temperature converter
######################################################################

*/

// Linear resistance calibrated from temperature measurements
type LinearResistance struct {
	Pullup float64
	Li     *LinearInterpolate
}

func NewLinearResistance(config *ConfigWrapper, samples [][]float64) (*LinearResistance, error) {
	self := &LinearResistance{}
	self.Pullup = config.Getfloat("pullup_resistor", 4700., 0., 0., 0., 0., true)
	var arr [][]float64
	for _, item := range samples {
		r := item[0]
		t := item[1]
		arr = append(arr, []float64{r, t})
	}
	li, err := NewLinearInterpolate(arr)
	if err != nil {
		return nil, errors.New(fmt.Sprintf("adc_temperature %s in heater %s",
			err, config.Get_name()))
	}
	self.Li = li
	return self, nil
}
func (self *LinearResistance) Calc_temp(adc float64) float64 {
	// Calculate temperature from adc
	adc = math.Max(.00001, math.Min(.99999, adc))
	r := self.Pullup * adc / (1.0 - adc)
	return self.Li.Interpolate(r)
}

func (self *LinearResistance) Calc_adc(temp float64) float64 {
	// Calculate adc reading from a temperature
	r := self.Li.Reverse_interpolate(temp)
	return r / (self.Pullup + r)
}

// Custom defined sensors from the config file
type CustomLinearResistance struct {
	name   string
	Sample [][]float64
}

func NewCustomLinearResistance(config *ConfigWrapper) ICustomLinear {
	self := &CustomLinearResistance{}
	self.name = strings.Join(strings.Split(config.Get_name(), " ")[1:], " ")
	self.Sample = [][]float64{}
	for i := 1; i < 1000; i++ {
		t := config.GetfloatNone(fmt.Sprintf("temperature%d", i), nil, 0, 0, 0, 0, true)
		r := config.GetfloatNone(fmt.Sprintf("resistance%d", i), nil, 0, 0, 0, 0, true)
		if t == nil {
			break
		}
		self.Sample = append(self.Sample, []float64{t.(float64), r.(float64)})
	}
	return self
}

func (self *CustomLinearResistance) Get_name() string {
	return self.name
}

func (self *CustomLinearResistance) Create(config *ConfigWrapper) interface{} {
	lr, err := NewLinearResistance(config, self.Sample)
	if err != nil {
		logger.Error(err)
	}
	return NewPrinterADCtoTemperature(config, lr)
}

/*
######################################################################
# Default sensors
######################################################################
*/

var AD595 = [][]float64{
	{0., .0027}, {10., .101}, {20., .200}, {25., .250}, {30., .300},
	{40., .401}, {50., .503}, {60., .605}, {80., .810}, {100., 1.015},
	{120., 1.219}, {140., 1.420}, {160., 1.620}, {180., 1.817}, {200., 2.015},
	{220., 2.213}, {240., 2.413}, {260., 2.614}, {280., 2.817}, {300., 3.022},
	{320., 3.227}, {340., 3.434}, {360., 3.641}, {380., 3.849}, {400., 4.057},
	{420., 4.266}, {440., 4.476}, {460., 4.686}, {480., 4.896},
}

var AD597 = [][]float64{
	{0., 0.}, {10., .097}, {20., .196}, {25., .245}, {30., .295},
	{40., 0.395}, {50., 0.496}, {60., 0.598}, {80., 0.802}, {100., 1.005},
	{120., 1.207}, {140., 1.407}, {160., 1.605}, {180., 1.801}, {200., 1.997},
	{220., 2.194}, {240., 2.392}, {260., 2.592}, {280., 2.794}, {300., 2.996},
	{320., 3.201}, {340., 3.406}, {360., 3.611}, {380., 3.817}, {400., 4.024},
	{420., 4.232}, {440., 4.440}, {460., 4.649}, {480., 4.857}, {500., 5.066},
}

var AD8494 = [][]float64{
	{-180, -0.714}, {-160, -0.658}, {-140, -0.594}, {-120, -0.523},
	{-100, -0.446}, {-80, -0.365}, {-60, -0.278}, {-40, -0.188},
	{-20, -0.095}, {0, 0.002}, {20, 0.1}, {25, 0.125}, {40, 0.201},
	{60, 0.303}, {80, 0.406}, {100, 0.511}, {120, 0.617}, {140, 0.723},
	{160, 0.829}, {180, 0.937}, {200, 1.044}, {220, 1.151}, {240, 1.259},
	{260, 1.366}, {280, 1.473}, {300, 1.58}, {320, 1.687}, {340, 1.794},
	{360, 1.901}, {380, 2.008}, {400, 2.114}, {420, 2.221}, {440, 2.328},
	{460, 2.435}, {480, 2.542}, {500, 2.65}, {520, 2.759}, {540, 2.868},
	{560, 2.979}, {580, 3.09}, {600, 3.203}, {620, 3.316}, {640, 3.431},
	{660, 3.548}, {680, 3.666}, {700, 3.786}, {720, 3.906}, {740, 4.029},
	{760, 4.152}, {780, 4.276}, {800, 4.401}, {820, 4.526}, {840, 4.65},
	{860, 4.774}, {880, 4.897}, {900, 5.018}, {920, 5.138}, {940, 5.257},
	{960, 5.374}, {980, 5.49}, {1000, 5.606}, {1020, 5.72}, {1040, 5.833},
	{1060, 5.946}, {1080, 6.058}, {1100, 6.17}, {1120, 6.282}, {1140, 6.394},
	{1160, 6.505}, {1180, 6.616}, {1200, 6.727},
}

var AD8495 = [][]float64{
	{-260, -0.786}, {-240, -0.774}, {-220, -0.751}, {-200, -0.719},
	{-180, -0.677}, {-160, -0.627}, {-140, -0.569}, {-120, -0.504},
	{-100, -0.432}, {-80, -0.355}, {-60, -0.272}, {-40, -0.184}, {-20, -0.093},
	{0, 0.003}, {20, 0.1}, {25, 0.125}, {40, 0.2}, {60, 0.301}, {80, 0.402},
	{100, 0.504}, {120, 0.605}, {140, 0.705}, {160, 0.803}, {180, 0.901},
	{200, 0.999}, {220, 1.097}, {240, 1.196}, {260, 1.295}, {280, 1.396},
	{300, 1.497}, {320, 1.599}, {340, 1.701}, {360, 1.803}, {380, 1.906},
	{400, 2.01}, {420, 2.113}, {440, 2.217}, {460, 2.321}, {480, 2.425},
	{500, 2.529}, {520, 2.634}, {540, 2.738}, {560, 2.843}, {580, 2.947},
	{600, 3.051}, {620, 3.155}, {640, 3.259}, {660, 3.362}, {680, 3.465},
	{700, 3.568}, {720, 3.67}, {740, 3.772}, {760, 3.874}, {780, 3.975},
	{800, 4.076}, {820, 4.176}, {840, 4.275}, {860, 4.374}, {880, 4.473},
	{900, 4.571}, {920, 4.669}, {940, 4.766}, {960, 4.863}, {980, 4.959},
	{1000, 5.055}, {1020, 5.15}, {1040, 5.245}, {1060, 5.339}, {1080, 5.432},
	{1100, 5.525}, {1120, 5.617}, {1140, 5.709}, {1160, 5.8}, {1180, 5.891},
	{1200, 5.98}, {1220, 6.069}, {1240, 6.158}, {1260, 6.245}, {1280, 6.332},
	{1300, 6.418}, {1320, 6.503}, {1340, 6.587}, {1360, 6.671}, {1380, 6.754},
}

var AD8496 = [][]float64{
	{-180, -0.642}, {-160, -0.59}, {-140, -0.53}, {-120, -0.464},
	{-100, -0.392}, {-80, -0.315}, {-60, -0.235}, {-40, -0.15}, {-20, -0.063},
	{0, 0.027}, {20, 0.119}, {25, 0.142}, {40, 0.213}, {60, 0.308},
	{80, 0.405}, {100, 0.503}, {120, 0.601}, {140, 0.701}, {160, 0.8},
	{180, 0.9}, {200, 1.001}, {220, 1.101}, {240, 1.201}, {260, 1.302},
	{280, 1.402}, {300, 1.502}, {320, 1.602}, {340, 1.702}, {360, 1.801},
	{380, 1.901}, {400, 2.001}, {420, 2.1}, {440, 2.2}, {460, 2.3},
	{480, 2.401}, {500, 2.502}, {520, 2.603}, {540, 2.705}, {560, 2.808},
	{580, 2.912}, {600, 3.017}, {620, 3.124}, {640, 3.231}, {660, 3.34},
	{680, 3.451}, {700, 3.562}, {720, 3.675}, {740, 3.789}, {760, 3.904},
	{780, 4.02}, {800, 4.137}, {820, 4.254}, {840, 4.37}, {860, 4.486},
	{880, 4.6}, {900, 4.714}, {920, 4.826}, {940, 4.937}, {960, 5.047},
	{980, 5.155}, {1000, 5.263}, {1020, 5.369}, {1040, 5.475}, {1060, 5.581},
	{1080, 5.686}, {1100, 5.79}, {1120, 5.895}, {1140, 5.999}, {1160, 6.103},
	{1180, 6.207}, {1200, 6.311},
}

var AD8497 = [][]float64{
	{-260, -0.785}, {-240, -0.773}, {-220, -0.751}, {-200, -0.718},
	{-180, -0.676}, {-160, -0.626}, {-140, -0.568}, {-120, -0.503},
	{-100, -0.432}, {-80, -0.354}, {-60, -0.271}, {-40, -0.184},
	{-20, -0.092}, {0, 0.003}, {20, 0.101}, {25, 0.126}, {40, 0.2},
	{60, 0.301}, {80, 0.403}, {100, 0.505}, {120, 0.605}, {140, 0.705},
	{160, 0.804}, {180, 0.902}, {200, 0.999}, {220, 1.097}, {240, 1.196},
	{260, 1.296}, {280, 1.396}, {300, 1.498}, {320, 1.599}, {340, 1.701},
	{360, 1.804}, {380, 1.907}, {400, 2.01}, {420, 2.114}, {440, 2.218},
	{460, 2.322}, {480, 2.426}, {500, 2.53}, {520, 2.634}, {540, 2.739},
	{560, 2.843}, {580, 2.948}, {600, 3.052}, {620, 3.156}, {640, 3.259},
	{660, 3.363}, {680, 3.466}, {700, 3.569}, {720, 3.671}, {740, 3.773},
	{760, 3.874}, {780, 3.976}, {800, 4.076}, {820, 4.176}, {840, 4.276},
	{860, 4.375}, {880, 4.474}, {900, 4.572}, {920, 4.67}, {940, 4.767},
	{960, 4.863}, {980, 4.96}, {1000, 5.055}, {1020, 5.151}, {1040, 5.245},
	{1060, 5.339}, {1080, 5.433}, {1100, 5.526}, {1120, 5.618}, {1140, 5.71},
	{1160, 5.801}, {1180, 5.891}, {1200, 5.981}, {1220, 6.07}, {1240, 6.158},
	{1260, 6.246}, {1280, 6.332}, {1300, 6.418}, {1320, 6.503}, {1340, 6.588},
	{1360, 6.671}, {1380, 6.754},
}

// base=100.
func Calc_pt100(base float64) [][]float64 {
	// Calc PT100/PT1000 resistances using Callendar-Van Dusen formula
	A, B := 3.9083e-3, -5.775e-7
	var arr [][]float64
	for t := 0; t < 500; t += 10 {
		item := []float64{float64(t), base * (1. + A*float64(t) + B*float64(t)*float64(t))}
		arr = append(arr, item)
	}
	return arr
}
func Calc_ina826_pt100() [][]float64 {
	// Standard circuit is 4400ohm pullup with 10x gain to 5V
	var arr [][]float64
	for _, item := range Calc_pt100(100.) {
		t := item[0]
		r := item[1]
		arr = append(arr, []float64{t, 10. * 5. * r / (4400. + r)})
	}
	return arr
}

type SensorsNode struct {
	Sensor_type string
	Params      [][]float64
}

var DefaultVoltageSensors = []SensorsNode{
	{"AD595", AD595}, {"AD597", AD597}, {"AD8494", AD8494}, {"AD8495", AD8495},
	{"AD8496", AD8496}, {"AD8497", AD8497},
	{"PT100 INA826", Calc_ina826_pt100()},
}

var DefaultResistanceSensors = []SensorsNode{
	{"PT1000", Calc_pt100(1000.)},
}

func Load_config_adc_temperature(config *ConfigWrapper) interface{} {
	// Register default sensors
	pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{}).(*PrinterHeaters)
	for _, item := range DefaultVoltageSensors {
		sensor_type := item.Sensor_type
		params := item.Params
		func_ := func(config *ConfigWrapper, params_in [][]float64) interface{} {
			paramsInner := params
			if len(params_in) != 0 {
				paramsInner = params_in
			}
			lv, err := NewLinearVoltage(config, paramsInner)
			if err != nil {
				logger.Error(err)
			}
			return NewPrinterADCtoTemperature(config, lv)
		}
		pheaters.Add_sensor_factory(sensor_type, func_)
	}

	for _, item := range DefaultResistanceSensors {
		sensor_type := item.Sensor_type
		params := item.Params
		func_ := func(config *ConfigWrapper, params_in [][]float64) *PrinterADCtoTemperature {
			paramsInner := params
			if len(params_in) != 0 {
				paramsInner = params_in
			}
			lv, err := NewLinearResistance(config, paramsInner)
			if err != nil {
				logger.Error(err)
			}
			return NewPrinterADCtoTemperature(config, lv)
		}
		pheaters.Add_sensor_factory(sensor_type, func_)
	}
	return nil
}

func Load_config_prefix_customLinear(config *ConfigWrapper) interface{} {
	var custom_sensor ICustomLinear
	if config.Get("resistance1", nil, true) == nil {
		custom_sensor = NewCustomLinearVoltage(config)
	} else {
		custom_sensor = NewCustomLinearResistance(config)
	}
	pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{}).(*PrinterHeaters)
	pheaters.Add_sensor_factory(custom_sensor.Get_name(), custom_sensor.Create)
	return nil
}
