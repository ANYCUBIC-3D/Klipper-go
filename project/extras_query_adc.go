/*
Utility for querying the current state of adc pins

Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/

package project

import (
	"fmt"
	"math"
	"strings"
)

type QueryADC struct {
	Printer *Printer
	Adc     map[string]interface{}
}

func NewQueryADC(config *ConfigWrapper) *QueryADC {
	self := &QueryADC{}
	self.Printer = config.Get_printer()
	self.Adc = make(map[string]interface{})
	gcode := MustLookupGcode(self.Printer)
	gcode.Register_command("QUERY_ADC", self.Cmd_QUERY_ADC,
		false, "Report the last value of an analog pin")
	return self
}

func (self *QueryADC) Register_adc(name string, mcu_adc interface{}) {
	self.Adc[name] = mcu_adc
}

func (self *QueryADC) Cmd_QUERY_ADC(gcmd *GCodeCommand) {
	name := gcmd.Get("NAME", nil, nil, nil, nil, nil, nil)
	for _, value := range self.Adc {
		if name == value {
			keys := make([]string, len(self.Adc))
			for key := range self.Adc {
				keys = append(keys, key)
			}
			keys = SelectSortString(keys)
			str := strings.Join(keys, ", ")
			gcmd.Respond_info(fmt.Sprintf("Available ADC objects: %s", str), true)
			return
		}
	}

	arr := self.Adc[name].(*MCU_adc).Get_last_value()
	value, timestamp := arr[0], arr[1]
	msg := fmt.Sprintf("ADC object \"%s\" has value %.6f (timestamp %.3f)",
		name, value, timestamp)

	pullup := gcmd.Get_float("PULLUP", nil, nil, nil, nil, nil)
	if pullup != 0. {
		v := math.Max(0.00001, math.Max(0.99999, value))
		r := pullup * v / (1.0 - v)
		msg += fmt.Sprintf("\n resistance %.3f (with %.0f pullup)", r, pullup)
	}
	gcmd.Respond_info(msg, true)
}

func SelectSortString(arr []string) []string {
	length := len(arr)
	if length <= 1 {
		return arr
	} else {
		for i := 0; i < length-1; i++ {
			min := i
			for j := i + 1; j < length; j++ {
				if strings.Compare(arr[min], arr[j]) > 0 {
					min = j
				}
			}
			if i != min {
				arr[i], arr[min] = arr[min], arr[i]
			}
		}
		return arr
	}
}

func Load_config_query_adc(config *ConfigWrapper) interface{} {
	return NewQueryADC(config)
}
