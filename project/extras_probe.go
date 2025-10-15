// Z-Probe support
//
// Copyright (C) 2017-2021  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"container/list"
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/value"
	"math"
	"reflect"
	"strings"
)

const HINT_TIMEOUT = "If the probe did not move far enough to trigger, then consider reducing the Z axis minimum position so the probecan travel further (the Z minimum position can be negative)."

type PrinterProbe struct {
	Printer             *Printer
	Name                string
	Mcu_probe           interface{}
	Speed               float64
	Lift_speed          float64
	X_offset            float64
	Y_offset            float64
	Z_offset            float64
	Probe_calibrate_z   float64
	Multi_probe_pending bool
	Last_state          bool
	Last_z_result       float64
	Gcode_move          *GCodeMove
	Sample_count        int
	Sample_retract_dist float64
	Samples_result      interface{}
	Samples_tolerance   float64
	Samples_retries     int
	Z_position          float64
	Gcode               *GCodeDispatch
	final_speed         float64
}

func NewPrinterProbe(config *ConfigWrapper, mcu_probe interface{}) *PrinterProbe {
	var self = &PrinterProbe{}
	self.Printer = config.Get_printer()
	self.Name = config.Get_name()
	self.Mcu_probe = mcu_probe
	self.Speed = config.Getfloat("speed", 5.0, 0, 0, 0., 0, true)
	self.Lift_speed = config.Getfloat("lift_speed", self.Speed, 0, 0, 0., 0, true)
	self.X_offset = config.Getfloat("x_offset", 0., 0, 0, 0., 0, true)
	self.Y_offset = config.Getfloat("y_offset", 0., 0, 0, 0., 0, true)
	self.Z_offset = config.Getfloat("z_offset", 0, 0, 0, 0., 0, true)
	self.final_speed = config.Getfloat("final_speed", 2., 0, 0, 0., 0, true)
	self.Probe_calibrate_z = 0.
	self.Multi_probe_pending = false
	self.Last_state = false
	self.Last_z_result = 0.
	self.Gcode_move = self.Printer.Load_object(config, "gcode_move", object.Sentinel{}).(*GCodeMove)
	// Infer Z position to move to during a probe
	if config.Has_section("stepper_z") {
		var zconfig = config.Getsection("stepper_z")
		self.Z_position = zconfig.Getfloat("position_min", 0.,
			0, 0, 0., 0, false)
	} else {
		var pconfig = config.Getsection("printer")
		self.Z_position = pconfig.Getfloat("minimum_z_position", 0.,
			0, 0, 0., 0, false)
	}
	// Multi-sample support (for improved accuracy)
	self.Sample_count = config.Getint("samples", 1, 1, 0, false)
	self.Sample_retract_dist = config.Getfloat("sample_retract_dist", 2.,
		0, 0, 0., 0, false)
	var atypes = map[interface{}]interface{}{"median": "median", "average": "average"}
	self.Samples_result = config.Getchoice("samples_result", atypes,
		"average", true)
	self.Samples_tolerance = config.Getfloat("samples_tolerance", 0.100,
		0., 0, 0, 0, false)
	self.Samples_retries = config.Getint("samples_tolerance_retries", 0,
		0., 0, false)
	// Register z_virtual_endstop pin
	pins := self.Printer.Lookup_object("pins", object.Sentinel{})
	pins.(*PrinterPins).Register_chip("probe", self)
	// Register homing event handlers
	self.Printer.Register_event_handler("homing:homing_move_begin",
		self.Handle_homing_move_begin)
	self.Printer.Register_event_handler("homing:homing_move_end",
		self.Handle_homing_move_end)
	self.Printer.Register_event_handler("homing:home_rails_begin",
		self.Handle_home_rails_begin)
	self.Printer.Register_event_handler("homing:home_rails_end",
		self.Handle_home_rails_end)
	self.Printer.Register_event_handler("gcode:command_error",
		self.Handle_command_error)

	// Register PROBE/QUERY_PROBE commands
	self.Gcode = MustLookupGcode(self.Printer)
	self.Gcode.Register_command("PROBE",
		self.Cmd_PROBE, false,
		cmd_PROBE_help)
	self.Gcode.Register_command("QUERY_PROBE",
		self.Cmd_QUERY_PROBE, false,
		cmd_QUERY_PROBE_help)
	self.Gcode.Register_command("PROBE_CALIBRATE",
		self.Cmd_PROBE_CALIBRATE, false,
		cmd_PROBE_CALIBRATE_help)
	self.Gcode.Register_command("PROBE_ACCURACY",
		self.Cmd_PROBE_ACCURACY, false,
		cmd_PROBE_ACCURACY_help)

	self.Gcode.Register_command("Z_OFFSET_APPLY_PROBE",
		self.Cmd_Z_OFFSET_APPLY_PROBE, false,
		cmd_Z_OFFSET_APPLY_PROBE_help)

	return self
}

func (self *PrinterProbe) Handle_homing_move_begin(args []interface{}) error {
	hmove := args[0].(*HomingMove)
	for _, e := range hmove.Get_mcu_endstops() {
		es := e.(list.List)
		if _, ok := es.Front().Value.(*ProbeEndstopWrapper); ok {
			if self.Mcu_probe == es.Front().Value.(*ProbeEndstopWrapper) {
				self.Mcu_probe.(*ProbeEndstopWrapper).Probe_prepare(hmove)
				break
			}
		}
	}

	return nil
}

func (self *PrinterProbe) Handle_homing_move_end(args []interface{}) error {
	hmove := args[0].(*HomingMove)
	for _, e := range hmove.Get_mcu_endstops() {
		es := e.(list.List)
		if _, ok := es.Front().Value.(*ProbeEndstopWrapper); ok {
			if self.Mcu_probe == es.Front().Value.(*ProbeEndstopWrapper) {
				self.Mcu_probe.(*ProbeEndstopWrapper).Probe_finish(hmove)
				break
			}
		}
	}
	return nil
}

func (self *PrinterProbe) Handle_home_rails_begin(args []interface{}) error {
	rails := args[1].([]*PrinterRail)
	var endstops []interface{}
	for _, rail := range rails {
		for _, val := range rail.Get_endstops() {
			es := val.Front().Value
			endstops = append(endstops, es)
		}
	}
	for _, val := range endstops {
		if self.Mcu_probe == val {
			self.Multi_probe_begin()
			break
		}
	}
	return nil
}

func (self *PrinterProbe) Handle_home_rails_end(args []interface{}) error {
	rails := args[1].([]*PrinterRail)
	var endstops []interface{}
	for _, rail := range rails {
		for _, val := range rail.Get_endstops() {
			es := val.Front().Value
			endstops = append(endstops, es)
		}
	}
	for _, val := range endstops {
		if self.Mcu_probe == val {
			self.Multi_probe_end()
			break
		}
	}

	return nil
}

func (self *PrinterProbe) Handle_command_error(args []interface{}) error {
	self.Multi_probe_end()

	return nil
}

func (self *PrinterProbe) Multi_probe_begin() {
	self.Printer.Send_event("homing:multi_probe_begin", nil)
	self.Mcu_probe.(*ProbeEndstopWrapper).Multi_probe_begin()
	self.Multi_probe_pending = true
}

func (self *PrinterProbe) Multi_probe_end() {
	if self.Multi_probe_pending {
		self.Multi_probe_pending = false
		pp, ppOk := self.Mcu_probe.(*PrinterProbe)
		if ppOk {
			pp.Multi_probe_end()
		}
		pepw, pepwOk := self.Mcu_probe.(*ProbeEndstopWrapper)
		if pepwOk {
			pepw.Multi_probe_end()
		}
	}
	self.Printer.Send_event("homing:multi_probe_end", nil)
}

func (self *PrinterProbe) Setup_pin(pin_type string, pin_params map[string]interface{}) interface{} {
	if pin_type != "endstop" || pin_params["pin"] != "z_virtual_endstop" {
		panic("Probe virtual endstop only useful as endstop pin")
	}

	if cast.ToInt(pin_params["invert"]) != 0 || cast.ToInt(pin_params["pullup"]) != 0 {
		panic("Can not pullup/invert probe virtual endstop")
	}
	return self.Mcu_probe
}

func (self *PrinterProbe) Get_lift_speed(gcmd *GCodeCommand) float64 {
	if gcmd != nil {
		zero := 0.0
		return gcmd.Get_float("LIFT_SPEED", self.Lift_speed, nil, nil, &zero, nil)
	}
	return self.Lift_speed
}

func (self *PrinterProbe) Get_offsets() (float64, float64, float64) {
	return self.X_offset, self.Y_offset, self.Z_offset
}
func (self *PrinterProbe) Probe(speed float64) []float64 {
	toolhead := MustLookupToolhead(self.Printer)
	var curtime = self.Printer.Get_reactor().Monotonic()
	str, _ := toolhead.Get_status(curtime)["homed_axes"]
	if strings.Index(str.(string), "z") == -1 {
		panic(("Must home before probe"))
	}
	phoming_obj := self.Printer.Lookup_object("homing", object.Sentinel{})
	phoming := phoming_obj.(*PrinterHoming)
	var pos = toolhead.Get_position()
	pos[2] = self.Z_position
	// try:
	var epos []float64
	if _, ok := self.Mcu_probe.(*MCU_endstop); ok {
		epos = phoming.Probing_move(self.Mcu_probe.(*MCU_endstop), pos, speed)
	} else {
		epos = phoming.Probing_move(self.Mcu_probe.(*ProbeEndstopWrapper), pos, speed)
	}
	self.Gcode.Respond_info(fmt.Sprintf("probe at %.3f,%.3f is z=%.6f", epos[0], epos[1], epos[2]), true)

	return epos[:3]
}

func (self *PrinterProbe) Move(coord interface{}, speed float64) {
	toolhead := MustLookupToolhead(self.Printer)
	if _, ok := coord.([]*float64); ok {
		coord_interface := make([]interface{}, len(coord.([]*float64)))
		for i, item := range coord.([]*float64) {
			if item != nil {
				coord_interface[i] = *item
			} else {
				coord_interface[i] = nil
			}
		}
		toolhead.Manual_move(coord_interface, speed)
	} else {
		toolhead.Manual_move(coord.([]interface{}), speed)
	}

}

func (self *PrinterProbe) Calc_mean(positions [][]float64) []float64 {
	var count = float64(len(positions))
	var sum float64
	var res []float64
	for i := 0; i < 3; i++ {
		sum = 0
		for _, pos := range positions {
			sum += pos[i]
		}
		res = append(res, sum/count)
	}
	return res
}

func (self *PrinterProbe) Calc_median(positions [][]float64) []float64 {
	z_sorted := make([][]float64, len(positions))
	copy(z_sorted, positions)
	for i := 0; i < len(positions)-1; i++ {
		for j := 1; j < len(positions)-i-1; i++ {
			if positions[j][2] < positions[j+1][2] {
				positions[j], positions[j+1] = positions[j+1], positions[j]
			}
		}
	}
	middle := len(positions) / 2
	if len(positions)&1 == 1 {
		// odd number of samples
		return z_sorted[middle]
	}
	// even number of samples
	return self.Calc_mean(z_sorted[middle-1 : middle+1])
}

func (self *PrinterProbe) Run_probe(gcmd *GCodeCommand) []float64 {
	zero := 0.
	var speed = gcmd.Get_float("PROBE_SPEED", self.Speed, nil, nil, &zero, nil)
	var lift_speed = self.Get_lift_speed(gcmd)
	one := 1
	var sample_count = gcmd.Get_int("SAMPLES", self.Sample_count, &one, nil)
	var sample_retract_dist = gcmd.Get_float("SAMPLE_RETRACT_DIST",
		self.Sample_retract_dist, nil, nil, &zero, nil)
	var samples_tolerance = gcmd.Get_float("SAMPLES_TOLERANCE",
		self.Samples_tolerance, &zero, nil, &zero, nil)
	zero_int := 0
	var samples_retries = gcmd.Get_int("SAMPLES_TOLERANCE_RETRIES",
		self.Samples_retries, &zero_int, nil)
	var samples_result = gcmd.Get("SAMPLES_RESULT", self.Samples_result,
		"", &zero, &zero, &zero, &zero)
	var must_notify_multi_probe = !self.Multi_probe_pending
	if must_notify_multi_probe {
		self.Multi_probe_begin()
	}
	probexy := MustLookupToolhead(self.Printer).Get_position()[:2]
	var retries = 0
	var positions = [][]float64{}
	for len(positions) < sample_count {
		// Probe position
		pos := self.Probe(speed)
		positions = append(positions, pos)
		// Check samples tolerance
		var z_positions []float64
		for _, p := range positions {
			z_positions = append(z_positions, p[2])
		}
		var max, min = z_positions[0], z_positions[0]
		for _, v := range z_positions {
			if max < v {
				max = v
			}
			if min > v {
				min = v
			}
		}

		if (max - min) > samples_tolerance {
			if retries >= samples_retries {
				panic("Probe samples exceed samples_tolerance")
			}
			gcmd.Respond_info("Probe samples exceed tolerance. Retrying...", true)
			retries += 1
			positions = [][]float64{}
		}
		// Retract
		if len(positions) < sample_count {
			arr := make([]interface{}, 0)
			for _, item := range probexy {
				arr = append(arr, item)
			}
			arr = append(arr, pos[2]+sample_retract_dist)
			self.Move(arr, lift_speed)
		}
	}

	if must_notify_multi_probe {
		self.Multi_probe_end()
	}
	// Calculate and return result
	if samples_result == "median" {
		return self.Calc_median(positions)
	}
	return self.Calc_mean(positions)
}

const cmd_PROBE_help = "Probe Z-height at current XY position"

func (self *PrinterProbe) Cmd_PROBE(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	var pos = self.Run_probe(gcmd)
	gcmd.Respond_info(fmt.Sprintf("Result is z=%.6f", pos[2]), true)
	self.Last_z_result = pos[2]
	return nil
}

const cmd_QUERY_PROBE_help = "Return the status of the z-probe"

func (self *PrinterProbe) Cmd_QUERY_PROBE(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	count := gcmd.Get_int("COUNT", 5, nil, nil)
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	for i := 0; i < count; i++ {
		var print_time = toolhead.(*Toolhead).Get_last_move_time()
		var res = self.Mcu_probe.(*ProbeEndstopWrapper).Query_endstop(print_time)
		if res == 1 {
			self.Last_state = true
		} else {
			self.Last_state = false
		}
		if res == 1 {
			gcmd.Respond_info(fmt.Sprintf("probe: %s", "TRIGGERED"), true)
			break
		} else {
			gcmd.Respond_info(fmt.Sprintf("probe: %s", "open"), true)
		}
	}

	return nil
}

func (self *PrinterProbe) get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{"last_query": self.Last_state,
		"last_z_result": self.Last_z_result}
}

const cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"

func (self *PrinterProbe) Cmd_PROBE_ACCURACY(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	zero := 0.
	var speed = gcmd.Get_float("PROBE_SPEED", self.Speed, nil, nil, &zero, nil)
	var lift_speed = self.Get_lift_speed(gcmd)
	one_int := 1
	var sample_count = gcmd.Get_int("SAMPLES", 10, &one_int, nil)
	var sample_retract_dist = gcmd.Get_float("SAMPLE_RETRACT_DIST",
		self.Sample_retract_dist, nil, nil, &zero, nil)
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	//if err != nil {
	//	logger.Error(err)
	//}
	var pos = toolhead.(*Toolhead).Get_position()
	gcmd.Respond_info(fmt.Sprintf("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"+
		" (samples=%d retract=%.3f"+
		" speed=%.1f lift_speed=%.1f)\n", pos[0], pos[1], pos[2],
		sample_count, sample_retract_dist,
		speed, lift_speed), true)
	// Probe bed sample_count times
	self.Multi_probe_begin()
	var positions = [][]float64{}
	for {
		var pos = self.Probe(speed)
		positions = append(positions, pos)
		// Retract
		val := pos[2] + sample_retract_dist
		var liftpos = []*float64{nil, nil, &val}
		self.Move(liftpos, lift_speed)
		if len(positions) >= sample_count {
			break
		}
	}

	self.Multi_probe_end()
	// Calculate maximum, minimum and average values
	var max_value, min_value = positions[0][2], positions[0][2]
	for _, item := range positions {
		if item[2] > max_value {
			max_value = item[2]
		} else if item[2] < min_value {
			min_value = item[2]
		}
	}
	var range_value = max_value - min_value
	var avg_value = self.Calc_mean(positions)[2]
	var median = self.Calc_median(positions)[2]
	// calculate the standard deviation
	var deviation_sum = 0.
	for i := 0; i < len(positions); i++ {
		deviation_sum += math.Pow(positions[i][2]-avg_value, 2.)
	}

	var sigma = math.Pow(deviation_sum/float64(len(positions)), 0.5)
	// Show information
	gcmd.Respond_info(
		fmt.Sprintf("probe accuracy results: maximum %.6f, minimum %.6f, range %.6f,"+
			"average %.6f, median %.6f, standard deviation %.6f",
			max_value, min_value, range_value, avg_value, median, sigma), true)
	return nil
}
func (self *PrinterProbe) Probe_calibrate_finalize(kin_pos []float64) {
	if len(kin_pos) == 0 {
		return
	}
	var z_offset = self.Probe_calibrate_z - kin_pos[2]
	self.Gcode.Respond_info(fmt.Sprintf(
		"%s: z_offset: %.3f\n"+
			"The SAVE_CONFIG command will update the printer config file\n"+
			"with the above and restart the printer.", self.Name, z_offset), true)
	configfile := self.Printer.Lookup_object("configfile", object.Sentinel{})
	configfile.(*PrinterConfig).Set(self.Name, "z_offset", fmt.Sprintf("%.3f", z_offset))
}

const cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"

func (self *PrinterProbe) Cmd_PROBE_CALIBRATE(gcmd interface{}) error {
	Verify_no_manual_probe(self.Printer)
	// Perform initial probe
	var lift_speed = self.Get_lift_speed(gcmd.(*GCodeCommand))
	var curpos = self.Run_probe(gcmd.(*GCodeCommand))
	// Move away from the bed
	self.Probe_calibrate_z = curpos[2]
	curpos[2] += 5.
	curpos_interface := []interface{}{}
	for _, c := range curpos {
		curpos_interface = append(curpos_interface, c)
	}
	self.Move(curpos_interface, lift_speed)
	// Move the nozzle over the probe point
	curpos[0] += self.X_offset
	curpos[1] += self.Y_offset
	for _, c := range curpos {
		curpos_interface = append(curpos_interface, c)
	}
	self.Move(curpos_interface, self.Speed)
	// Start manual probe
	NewManualProbeHelper(self.Printer, gcmd.(*GCodeCommand),
		self.Probe_calibrate_finalize)
	return nil
}

func (self *PrinterProbe) Cmd_Z_OFFSET_APPLY_PROBE(argv interface{}) error {
	var offset = self.Gcode_move.Get_status(0)["homing_origin"].([]float64)[2]
	var configfile = self.Printer.Lookup_object("configfile", object.Sentinel{})
	if offset == 0 {
		self.Gcode.Respond_info("Nothing to do: Z Offset is 0", true)
	} else {
		var new_calibrate = self.Z_offset - offset
		self.Gcode.Respond_info(fmt.Sprintf(
			"%s: z_offset: %.3f\n"+
				"The SAVE_CONFIG command will update the printer config file\n"+
				"with the above and restart the printer.",
			self.Name, new_calibrate), true)
		configfile.(*PrinterConfig).Set(self.Name, "z_offset", fmt.Sprintf("%.4f", new_calibrate))
	}
	return nil
}

const cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"

// Endstop wrapper that enables probe specific features
type ProbeEndstopWrapper struct {
	Printer             *Printer
	Position_endstop    float64
	Stow_on_each_sample bool
	Activate_gcode      interface{}
	Deactivate_gcode    interface{}
	Mcu_endstop         *MCU_endstop
	Get_mcu             interface{} //func
	Add_stepper         func(interface{})
	Get_steppers        func() []interface{}
	Home_start          interface{} //func
	Home_wait           interface{} //func
	Query_endstop       func(float64) int
	Multi               string
}

func NewProbeEndstopWrapper(config *ConfigWrapper) *ProbeEndstopWrapper {
	self := &ProbeEndstopWrapper{}
	self.Printer = config.Get_printer()
	self.Position_endstop = config.Getfloat("z_offset", 0, 0, 0, 0, 0, true)
	self.Stow_on_each_sample = config.Getboolean(
		"deactivate_on_each_sample", nil, true)
	gcode_macro := self.Printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	self.Activate_gcode = gcode_macro.Load_template(
		config, "activate_gcode", "")
	self.Deactivate_gcode = gcode_macro.Load_template(
		config, "deactivate_gcode", "")
	//Create an "endstop" object to handle the probe pin
	var ppins = self.Printer.Lookup_object("pins", object.Sentinel{})
	var pin = config.Get("pin", object.Sentinel{}, true)
	var pin_params = ppins.(*PrinterPins).Lookup_pin(pin.(string), true, true, nil)
	var mcu = pin_params["chip"]
	self.Mcu_endstop = mcu.(*MCU).Setup_pin("endstop", pin_params).(*MCU_endstop)
	self.Printer.Register_event_handler("project:mcu_identify",
		self.Handle_mcu_identify)
	// Wrappers
	self.Get_mcu = self.Mcu_endstop.Get_mcu
	self.Add_stepper = self.Mcu_endstop.Add_stepper
	self.Get_steppers = self.Mcu_endstop.Get_steppers
	self.Home_start = self.Mcu_endstop.Home_start
	self.Home_wait = self.Mcu_endstop.Home_wait
	self.Query_endstop = self.Mcu_endstop.Query_endstop
	// multi probes state
	self.Multi = "OFF"
	return self
}

func (self *ProbeEndstopWrapper) Handle_mcu_identify([]interface{}) error {
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	var kin = toolhead.(*Toolhead).Get_kinematics().(IKinematics)
	for _, stepper := range kin.Get_steppers() {
		if stepper.(*MCU_stepper).Is_active_axis('z') != 0 {
			self.Add_stepper(stepper)
		}
	}
	return nil
}

func (self *ProbeEndstopWrapper) Raise_probe() {
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	var start_pos = toolhead.(*Toolhead).Get_position()
	self.Deactivate_gcode.(*TemplateWrapper).Run_gcode_from_command(nil)
	notEq := false
	for i := 0; i < 3; i++ {
		if toolhead.(*Toolhead).Get_position()[i] != start_pos[i] {
			notEq = true
			break
		}
	}
	if notEq {
		panic("project.Toolhead moved during probe activate_gcode script")
	}
}

func (self *ProbeEndstopWrapper) Lower_probe() {
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})

	var start_pos = toolhead.(*Toolhead).Get_position()
	self.Activate_gcode.(*TemplateWrapper).Run_gcode_from_command(nil)
	for i := 0; i < 3; i++ {
		if toolhead.(*Toolhead).Get_position()[i] != start_pos[i] {
			panic("project.Toolhead moved during probe deactivate_gcode script")
		}
	}
}

func (self *ProbeEndstopWrapper) Multi_probe_begin() {
	if self.Stow_on_each_sample {
		return
	}
	self.Multi = "FIRST"
}

func (self *ProbeEndstopWrapper) Multi_probe_end() {
	if self.Stow_on_each_sample {
		return
	}
	self.Raise_probe()
	self.Multi = "OFF"
}

func (self *ProbeEndstopWrapper) Probe_prepare(hmove interface{}) {
	if self.Multi == "OFF" || self.Multi == "FIRST" {
		self.Lower_probe()
		if self.Multi == "FIRST" {
			self.Multi = "ON"
		}
	}
}

func (self *ProbeEndstopWrapper) Probe_finish(hmove interface{}) {
	if self.Multi == "OFF" {
		self.Raise_probe()
	}
}

func (self *ProbeEndstopWrapper) Get_position_endstop() float64 {
	return self.Position_endstop
}

// Helper code that can probe a series of points and report the
// position at each point.
type ProbePointsHelper struct {
	printer              *Printer
	finalize_callback    func([]float64, [][]float64) string
	Start_probe_callback func(*GCodeCommand)
	probe_points         [][]float64
	name                 string
	gcode                *GCodeDispatch
	horizontal_move_z    float64
	speed                float64
	use_offsets          bool
	lift_speed           float64
	probe_offsets        []float64
	results              [][]float64
	posArr               []float64
}

func NewProbePointsHelper(config *ConfigWrapper, finalize_callback interface{}, default_points [][]float64) *ProbePointsHelper {
	self := &ProbePointsHelper{}
	self.printer = config.Get_printer()
	self.finalize_callback = finalize_callback.(func([]float64, [][]float64) string)
	self.Start_probe_callback = self.Start_probe
	self.probe_points = default_points
	self.name = config.Get_name()
	self.gcode = MustLookupGcode(self.printer)
	// Read config settings
	if len(default_points) == 0 || config.Get("points", value.None, true) != nil {
		self.probe_points = config.Getlists("points", nil, []string{","},
			2, reflect.Float64, true).([][]float64)
	}
	self.horizontal_move_z = config.Getfloat("horizontal_move_z", 5., 0, 0, 0, 0, true)
	self.speed = config.Getfloat("speed", 50., 0, 0, 0., 0, true)
	self.use_offsets = false
	// Internal probing state
	self.lift_speed = self.speed
	self.probe_offsets = []float64{0., 0., 0.}
	self.results = [][]float64{}
	return self
}

func (self *ProbePointsHelper) Minimum_points(n int) {
	if len(self.probe_points) < n {
		panic(fmt.Sprintf("Need at least %d probe points for %s", n, self.name))
	}
}

func (self *ProbePointsHelper) Update_probe_points(points [][]float64, min_points int) {
	self.probe_points = points
	self.Minimum_points(min_points)
}

func (self *ProbePointsHelper) Use_xy_offsets(use_offsets bool) {
	self.use_offsets = use_offsets
}

func (self *ProbePointsHelper) Get_lift_speed() float64 {
	return self.lift_speed
}

func (self *ProbePointsHelper) Move_next() bool {
	toolhead := MustLookupToolhead(self.printer)
	// Lift toolhead
	var speed = self.lift_speed
	if len(self.results) != 0 {
		// Use full speed to first probe position
		speed = self.speed
	}
	toolhead.Manual_move([]interface{}{nil, nil, self.horizontal_move_z}, speed)
	// Check if done probing
	if len(self.results) >= len(self.probe_points) {
		toolhead.Get_last_move_time()
		var res = self.finalize_callback(self.probe_offsets, self.results)
		if res != "retry" {
			return true
		}
		self.results = [][]float64{}
	}
	// Move to next XY probe point
	nextpos_back := make([]float64, len(self.probe_points[len(self.results)]))
	copy(nextpos_back, self.probe_points[len(self.results)])
	if self.use_offsets {
		nextpos_back[0] -= self.probe_offsets[0]
		nextpos_back[1] -= self.probe_offsets[1]
	}
	nextpos := make([]interface{}, len(nextpos_back))
	for i, item := range nextpos_back {
		nextpos[i] = item
	}
	toolhead.Manual_move(nextpos, self.speed)
	return false
}

func (self *ProbePointsHelper) Start_probe(gcmd *GCodeCommand) {
	Verify_no_manual_probe(self.printer)
	// Lookup objects
	var probe_obj = self.printer.Lookup_object("probe", object.Sentinel{})
	probe := probe_obj.(*PrinterProbe)
	zero := 0.
	var method = strings.ToLower(gcmd.Get("METHOD", "automatic", 0,
		&zero, &zero, &zero, &zero))
	self.results = [][]float64{}
	if probe == nil || method != "automatic" {
		// Manual probe
		self.lift_speed = self.speed
		self.probe_offsets = []float64{0., 0., 0.}
		self.Manual_probe_start()
		return
	}
	// Perform automatic probing
	self.lift_speed = probe.Get_lift_speed(gcmd)
	val1, val2, val3 := probe.Get_offsets()
	self.probe_offsets = []float64{val1, val2, val3}
	if self.horizontal_move_z < self.probe_offsets[2] {
		panic("horizontal_move_z can t be less than probe's z_offset")
	}
	probe.Multi_probe_begin()
	for {
		var done = self.Move_next()
		if done {
			break
		}
		var pos = probe.Run_probe(gcmd)
		self.results = append(self.results, pos)
	}
	probe.Multi_probe_end()
}

func (self *ProbePointsHelper) Manual_probe_start() {
	var done = self.Move_next()
	if !done {
		var gcmd = self.gcode.Create_gcode_command("", "", nil)
		NewManualProbeHelper(self.printer, gcmd, self.Manual_probe_finalize)
	}
}

func (self *ProbePointsHelper) Manual_probe_finalize(kin_pos []float64) {
	if kin_pos == nil {
		return
	}
	self.results = append(self.results, kin_pos)
	self.Manual_probe_start()
}

func Load_config_probe(config *ConfigWrapper) interface{} {
	return NewPrinterProbe(config, NewProbeEndstopWrapper(config))
}
