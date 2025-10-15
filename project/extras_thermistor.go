package project

import (
	"k3c/common/utils/object"
	"log"
	"math"
	"strings"
)

/*
# Temperature measurements with thermistors
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/

const (
	KELVIN_TO_CELSIUS_THERMISTOR = -273.15
)

// Analog voltage to temperature converter for thermistors
type Thermistor struct {
	Pullup          float64
	Inline_resistor float64
	C1              float64
	C2              float64
	C3              float64
}

func NewThermistor(pullup, inline_resistor float64) *Thermistor {
	self := &Thermistor{}
	self.Pullup = pullup
	self.Inline_resistor = inline_resistor
	self.C1 = 0.
	self.C2 = 0.
	self.C3 = 0.
	return self
}

func (self *Thermistor) Setup_coefficients(t1, r1, t2, r2, t3, r3 float64, name string) {
	// Calculate Steinhart-Hart coefficents from temp measurements.
	// Arrange samples as 3 linear equations and solve for c1, c2, and c3.
	inv_t1 := 1. / (t1 - KELVIN_TO_CELSIUS)
	inv_t2 := 1. / (t2 - KELVIN_TO_CELSIUS)
	inv_t3 := 1. / (t3 - KELVIN_TO_CELSIUS)
	ln_r1 := math.Log(r1)
	ln_r2 := math.Log(r2)
	ln_r3 := math.Log(r3)
	ln3_r1, ln3_r2, ln3_r3 := math.Pow(ln_r1, 3), math.Pow(ln_r2, 3), math.Pow(ln_r3, 3)
	inv_t12, inv_t13 := inv_t1-inv_t2, inv_t1-inv_t3
	ln_r12, ln_r13 := ln_r1-ln_r2, ln_r1-ln_r3
	ln3_r12, ln3_r13 := ln3_r1-ln3_r2, ln3_r1-ln3_r3
	self.C3 = (inv_t12 - inv_t13*ln_r12/ln_r13) / (ln3_r12 - ln3_r13*ln_r12/ln_r13)

	if self.C3 <= 0 {
		beta := ln_r13 / inv_t13
		log.Printf("Using thermistor beta %.3f in heater %s\n", beta, name)
		self.Setup_coefficients_beta(t1, r1, beta)
		return
	}
	self.C2 = (inv_t12 - self.C3*ln3_r12) / ln_r12
	self.C1 = inv_t1 - self.C2*ln_r1 - self.C3*ln3_r1
}

func (self *Thermistor) Setup_coefficients_beta(t1, r1, beta float64) {
	// Calculate equivalent Steinhart-Hart coefficents from beta
	inv_t1 := 1. / (t1 - KELVIN_TO_CELSIUS)
	ln_r1 := math.Log(r1)
	self.C3 = 0.
	self.C2 = 1. / beta
	self.C1 = inv_t1 - self.C2*ln_r1
}

func (self *Thermistor) Calc_temp(adc float64) float64 {
	// Calculate temperature from adc
	adc = math.Max(.00001, math.Min(.99999, adc))
	r := self.Pullup * adc / (1.0 - adc)
	ln_r := math.Log(r - self.Inline_resistor)
	inv_t := self.C1 + self.C2*ln_r + self.C3*math.Pow(ln_r, 3)
	return 1.0/inv_t + KELVIN_TO_CELSIUS
}

func (self *Thermistor) Calc_adc(temp float64) float64 {
	// Calculate adc reading from a temperature
	if temp <= KELVIN_TO_CELSIUS {
		return 1.
	}
	inv_t := 1. / (temp - KELVIN_TO_CELSIUS)
	ln_r := 0.
	if self.C3 != 0. {
		// Solve for ln_r using Cardano's formula
		y := (self.C1 - inv_t) / (2. * self.C3)
		x := math.Sqrt(math.Pow(self.C2/(3.*self.C3), 3) + math.Pow(y, 2))
		ln_r = math.Pow(x-y, 1./3.) - math.Pow(x+y, 1./3.)
	} else {
		ln_r = (inv_t - self.C1) / self.C2
	}
	r := math.Exp(ln_r) + self.Inline_resistor
	return r / (self.Pullup + r)
}

// Create an ADC converter with a thermistor
func PrinterThermistor(config *ConfigWrapper, params map[string]float64) *PrinterADCtoTemperature {
	pullup := config.Getfloat("pullup_resistor", 4700., 0., 0., 0., 0., true)
	inline_resistor := config.Getfloat("inline_resistor", 0., 0., 0., 0., 0., true)
	thermistor := NewThermistor(pullup, inline_resistor)
	if _, ok := params["beta"]; ok {
		thermistor.Setup_coefficients_beta(params["t1"], params["r1"], params["beta"])
	} else {
		thermistor.Setup_coefficients(
			params["t1"], params["r1"], params["t2"], params["r2"],
			params["t3"], params["r3"], config.Get_name())
	}
	return NewPrinterADCtoTemperature(config, thermistor)
}

// Custom defined thermistors from the config file
type CustomThermistor struct {
	Name   string
	T1     float64
	R1     float64
	Beta   float64
	T2     float64
	R2     float64
	T3     float64
	R3     float64
	Params map[string]float64
}

func NewCustomThermistor(config *ConfigWrapper) *CustomThermistor {
	self := &CustomThermistor{}
	self.Name = strings.Join(strings.Split(config.Get_name(), " ")[1:], " ")
	t1 := config.Getfloat("temperature1", 0., KELVIN_TO_CELSIUS, 0., 0., 0., true)
	r1 := config.Getfloat("resistance1", 0., 0., 0., 0., 0., true)
	beta := config.Getfloat("beta", nil, 0., 0., 0., 0., true)
	if beta != 0. {
		self.Params = map[string]float64{"t1": t1, "r1": r1, "beta": beta}
		return self
	}
	t2 := config.Getfloat("temperature2", 0., KELVIN_TO_CELSIUS, 0., 0., 0., true)
	r2 := config.Getfloat("resistance2", 0., 0., 0., 0., 0., true)
	t3 := config.Getfloat("temperature3", 0., KELVIN_TO_CELSIUS, 0., 0., 0., true)
	r3 := config.Getfloat("resistance3", 0., 0., 0., 0., 0., true)
	arr := [][]float64{{t1, r1}, {t2, r2}, {t3, r3}}
	for i := 0; i < len(arr)-1; i++ {
		for j := 0; j < len(arr)-1-i; j++ {
			if arr[j][0] < arr[j+1][0] {
				arr[j], arr[j+1] = arr[j+1], arr[j]
			}
		}
	}
	self.Params = map[string]float64{
		"t1": arr[0][0], "r1": arr[0][1], "t2": arr[1][0], "r2": arr[1][1],
		"t3": arr[2][0], "r3": arr[2][1],
	}
	return self
}

func (self *CustomThermistor) Create(config *ConfigWrapper) interface{} {
	return PrinterThermistor(config, self.Params)
}

func Load_config_thermistor(config *ConfigWrapper) interface{} {
	thermistor := NewCustomThermistor(config)
	pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{})
	pheaters.(*PrinterHeaters).Add_sensor_factory(thermistor.Name, thermistor.Create)
	return nil
}
