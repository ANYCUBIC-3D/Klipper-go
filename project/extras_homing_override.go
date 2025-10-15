package project

import (
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/value"
	"strings"
)

type HomingOverride struct {
	printer   *Printer
	start_pos []float64
	axes      string
	template  *TemplateWrapper
	in_script bool
	gcode     *GCodeDispatch
	prev_G28  func(interface{})
}

func NewHomingOverride(config *ConfigWrapper) *HomingOverride {
	self := new(HomingOverride)
	self.printer = config.Get_printer()

	for _, a := range "xyz" {
		v := config.GetfloatNone("set_position_"+string(a), 0, 0, 0, 0, 0, true)
		if value.IsNone(v) {
			self.start_pos = append(self.start_pos, value.Float64None)
		} else {
			self.start_pos = append(self.start_pos, cast.ToFloat64(v))
		}
	}

	self.axes = strings.ToUpper(cast.ToString(config.Get("axes", "XYZ", true)))
	gcode_macro := self.printer.Load_object(config, "gcode_macro", object.Sentinel{}).(*PrinterGCodeMacro)
	self.template = gcode_macro.Load_template(config, "gcode", value.StringNone)
	self.in_script = false

	self.printer.Load_object(config, "homing", object.Sentinel{})
	gcodei := self.printer.Lookup_object("gcode", object.Sentinel{})
	self.gcode = gcodei.(*GCodeDispatch)
	self.prev_G28 = self.gcode.Register_command("G28", nil, false, "").(func(interface{}))
	self.gcode.Register_command("G28", self.cmd_G28, false, "")
	return self
}

func (self *HomingOverride) cmd_G28(arg interface{}) {
	if self.in_script {
		//# Was called recursively - invoke the real G28 command
		self.prev_G28(arg)
		return
	}
	//# if no axis is given as parameter we assume the override
	var no_axis = true
	gcmd := arg.(*GCodeCommand)

	for _, c := range "XYZ" {
		if gcmd.Get(string(c), "", nil, nil, nil, nil, nil) != "" {
			no_axis = false
		}
	}

	var override bool
	if no_axis {
		override = true
	} else {
		//# check if we home an axis which needs the override
		override = false
		for _, axis := range self.axes {
			if gcmd.Get(string(axis), "", nil, nil, nil, nil, nil) != "" {
				override = false
			}
		}
	}

	if !override {
		self.prev_G28(arg)
		return
	}
	//# Calculate forced position (if configured)
	toolhead := MustLookupToolhead(self.printer)
	pos := toolhead.Get_position()
	homing_axes := make([]int, 0)
	for axis, loc := range self.start_pos {
		if !value.IsNone(loc) {
			pos[axis] = loc
			homing_axes = append(homing_axes, axis)
		}
	}

	toolhead.Set_position(pos, homing_axes)
	// Perform homing
	context := self.template.create_template_context(nil)
	context.(map[string]interface{})["params"] = gcmd.Get_command_parameters()
	self.in_script = true
	err := self.template.Run_gcode_from_command(context.(map[string]interface{}))
	if err != nil {
		self.in_script = false
	}
}

func Load_config_homing_override(config *ConfigWrapper) interface{} {
	return NewHomingOverride(config)
}
