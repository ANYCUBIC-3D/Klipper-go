package project

import (
	"fmt"
)

const (
	cmd_SET_RETRACTION_help = "Set firmware retraction parameters"
	cmd_GET_RETRACTION_help = "Report firmware retraction paramters"
)

type FirmwareRetraction struct {
	printer                *Printer
	retract_length         float64
	retract_speed          float64
	unretract_extra_length float64
	unretract_speed        float64
	unretract_length       float64
	is_retracted           bool
	gcode                  *GCodeDispatch
}

func NewFirmwareRetraction(config *ConfigWrapper) *FirmwareRetraction {
	self := new(FirmwareRetraction)
	self.printer = config.Get_printer()
	self.retract_length = config.Getfloat("retract_length", 0, 0, 0, 0, 0, true)
	self.retract_speed = config.Getfloat("retract_speed", 20.0, 1.0, 0, 0, 0, true)
	self.unretract_extra_length = config.Getfloat(
		"unretract_extra_length", 0.0, 0.0, 0.0, 0.0, 0.0, true)
	self.unretract_speed = config.Getfloat("unretract_speed", 10.0, 1.0, 0, 0, 0, true)
	self.unretract_length = self.retract_length + self.unretract_extra_length
	self.is_retracted = false
	self.gcode = MustLookupGcode(self.printer)
	self.gcode.Register_command("SET_RETRACTION", self.cmd_SET_RETRACTION, false,
		cmd_SET_RETRACTION_help)
	self.gcode.Register_command("GET_RETRACTION", self.cmd_GET_RETRACTION, false,
		cmd_GET_RETRACTION_help)
	self.gcode.Register_command("G10", self.cmd_G10, false, "")
	self.gcode.Register_command("G11", self.cmd_G11, false, "")
	return self
}

func (self *FirmwareRetraction) Get_status(eventtime float64) map[string]float64 {
	return map[string]float64{
		"retract_length":         self.retract_length,
		"retract_speed":          self.retract_speed,
		"unretract_extra_length": self.unretract_extra_length,
		"unretract_speed":        self.unretract_speed,
	}
}

func (self *FirmwareRetraction) cmd_SET_RETRACTION(gcmdi interface{}) {
	gcmd := gcmdi.(*GCodeCommand)
	var retract_length_minval = 0.0
	self.retract_length = gcmd.Get_float("RETRACT_LENGTH", self.retract_length, &retract_length_minval, nil, nil, nil)

	var retract_speed_minval = 1.0
	self.retract_speed = gcmd.Get_float("RETRACT_SPEED", self.retract_speed, &retract_speed_minval, nil, nil, nil)

	var unretract_extra_length_minval = 0.0
	self.unretract_extra_length = gcmd.Get_float("UNRETRACT_EXTRA_LENGTH", self.unretract_extra_length, &unretract_extra_length_minval, nil, nil, nil)

	var unretract_speed_minval = 1.0
	self.unretract_speed = gcmd.Get_float("UNRETRACT_SPEED", self.unretract_speed, &unretract_speed_minval, nil, nil, nil)

	self.unretract_length = self.retract_length + self.unretract_extra_length
	self.is_retracted = false
}

func (self *FirmwareRetraction) cmd_GET_RETRACTION(gcmd *GCodeCommand) {
	gcmd.Respond_info(fmt.Sprintf(
		"RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f"+
			" UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f",
		self.retract_length,
		self.retract_speed,
		self.unretract_extra_length,
		self.unretract_speed,
	), true)
}

func (self *FirmwareRetraction) cmd_G10(gcmd *GCodeCommand) {
	if !self.is_retracted {
		self.gcode.Run_script_from_command(fmt.Sprintf(
			"SAVE_GCODE_STATE NAME=_retract_state\n"+
				"G91\n"+
				"G1 E-%.5f F%d\n"+
				"RESTORE_GCODE_STATE NAME=_retract_state",
			self.retract_length, int64(self.retract_speed*60)))
		self.is_retracted = true
	}
}

func (self *FirmwareRetraction) cmd_G11(gcmd *GCodeCommand) {
	if self.is_retracted {
		self.gcode.Run_script_from_command(fmt.Sprintf(
			"SAVE_GCODE_STATE NAME=_retract_state\n"+
				"G91\n"+
				"G1 E%.5f F%d\n"+
				"RESTORE_GCODE_STATE NAME=_retract_state",
			self.unretract_length, int64(self.unretract_speed*60)))
		self.is_retracted = false
	}
}

func Load_config_firmware_retraction(config *ConfigWrapper) interface{} {
	return NewFirmwareRetraction(config)
}
