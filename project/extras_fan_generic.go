// Support fans that are controlled by gcode
//
// Copyright (C) 2016-2020  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"k3c/common/utils/object"
	"strings"
)

const (
	cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"
)

type PrinterFanGeneric struct {
	printer  *Printer
	fan      *Fan
	fan_name string
}

func NewPrinterFanGeneric(config *ConfigWrapper) *PrinterFanGeneric {
	var self = PrinterFanGeneric{}
	self.printer = config.Get_printer()
	self.fan = NewFan(config, 0.)
	fan_name := strings.Split(config.Get_name(), " ")
	self.fan_name = fan_name[len(fan_name)-1]

	var gcode = self.printer.Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_mux_command("SET_FAN_SPEED", "FAN",
		self.fan_name,
		self.cmd_SET_FAN_SPEED,
		cmd_SET_FAN_SPEED_help)
	return &self
}

func (self PrinterFanGeneric) get_status(eventtime float64) map[string]float64 {
	return self.fan.Get_status(eventtime)
}

func (self PrinterFanGeneric) cmd_SET_FAN_SPEED(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	var speed = gcmd.Get_float("SPEED", 0., nil, nil, nil, nil)
	self.fan.Set_speed_from_command(speed)
	return nil
}

func Load_config_prefix(config *ConfigWrapper) *PrinterFanGeneric {
	return NewPrinterFanGeneric(config)
}
