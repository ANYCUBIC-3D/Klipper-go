package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"math"
	"reflect"
)

func bisectLeft(nums []float64, target float64) int {
	low := 0
	high := len(nums) - 1

	for low <= high {
		mid := low + (high-low)/2
		val := nums[mid]

		if math.Abs(val-target) < 0.001 { // set threshold here
			return mid
		} else if val < target {
			low = mid + 1
		} else {
			high = mid - 1
		}
	}

	return low
}

type ManualProbe struct {
	printer            *Printer
	gcode              *GCodeDispatch
	gcode_move         *GCodeMove
	z_position_endstop float64
	status             map[string]interface{}
	a_position_endstop float64
	b_position_endstop float64
	c_position_endstop float64
}

func NewManualProbe(config *ConfigWrapper) *ManualProbe {
	self := ManualProbe{}
	self.printer = config.Get_printer()
	// register commands
	self.gcode = MustLookupGcode(self.printer)
	self.gcode_move = MustLookupGCodeMove(self.printer)
	self.gcode.Register_command("MANUAL_PROBE", self.cmd_MANUAL_PROBE,
		false, self.cmd_MANUAL_PROBE_help())
	//# Endstop values for linear delta printers with vertical A,B,C towers
	a_tower_config := config.Getsection("stepper_a")
	self.a_position_endstop = a_tower_config.Getfloat("position_endstop", object.Sentinel{}, 0., 0., 0., 0., false)

	b_tower_config := config.Getsection("stepper_b")
	self.b_position_endstop = b_tower_config.Getfloat("position_endstop", object.Sentinel{}, 0., 0., 0., 0., false)

	c_tower_config := config.Getsection("stepper_c")
	self.c_position_endstop = c_tower_config.Getfloat("position_endstop", object.Sentinel{}, 0., 0., 0., 0., false)

	//# Conditionally register appropriate commands depending on printer
	//# Cartestian printers with separate Z Axis
	zconfig := config.Getsection("stepper_z")
	self.z_position_endstop = zconfig.Getfloat("Position_endstop", object.Sentinel{}, 0, 0, 0, 0, false)
	if self.z_position_endstop != 0 {
		self.gcode.Register_command("Z_ENDSTOP_CALIBRATE",
			self.cmd_Z_ENDSTOP_CALIBRATE,
			false, self.cmd_Z_ENDSTOP_CALIBRATE_help())
		self.gcode.Register_command("Z_OFFSET_APPLY_ENDSTOP",
			self.cmd_Z_OFFSET_APPLY_ENDSTOP,
			false, self.cmd_Z_OFFSET_APPLY_ENDSTOP_help())
	}
	//# Linear delta printers with A,B,C towers
	if "delta" == config.Getsection("printer").Get("kinematics", object.Sentinel{}, true).(string) {
		self.gcode.Register_command("Z_OFFSET_APPLY_ENDSTOP",
			self.cmd_Z_OFFSET_APPLY_DELTA_ENDSTOPS,
			false, self.cmd_Z_OFFSET_APPLY_ENDSTOP_help())
	}
	self.Reset_status()

	return &self
}

func (self *ManualProbe) Manual_probe_finalize(kin_pos []float64) {
	if len(kin_pos) > 0 {
		self.gcode.Respond_info(fmt.Sprintf("Z position is %.3f", kin_pos[2]), true)
	}
}

func (self *ManualProbe) Reset_status() {
	self.status = map[string]interface{}{
		"is_active":        false,
		"z_position":       nil,
		"z_position_lower": nil,
		"z_position_upper": nil,
	}
}

func (self *ManualProbe) Get_status(eventTime int64) map[string]interface{} {
	return sys.DeepCopyMap(self.status)
}

func (self *ManualProbe) cmd_MANUAL_PROBE_help() string {
	return "Start manual probe helper script"
}

func (self *ManualProbe) cmd_MANUAL_PROBE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	NewManualProbeHelper(self.printer, gcmd, self.Manual_probe_finalize)
	return nil
}

func (self *ManualProbe) Z_endstop_finalize(kin_pos []float64) {
	if len(kin_pos) == 0 {
		return
	}

	z_pos := self.z_position_endstop - kin_pos[2]
	self.gcode.Respond_info(fmt.Sprintf("stepper_z: Position_endstop: %.3f\n"+
		"The SAVE_CONFIG command will update the printer config file\n "+
		"with the above and restart the printer.", z_pos), true)
	configfile := self.printer.Lookup_object("configfile", object.Sentinel{})
	configfile.(*PrinterConfig).Set("stepper_z", "Position_endstop", fmt.Sprintf("%.3f", z_pos))
}

func (self *ManualProbe) cmd_Z_ENDSTOP_CALIBRATE_help() string {
	return "Calibrate a Z endstop"
}

func (self *ManualProbe) cmd_Z_ENDSTOP_CALIBRATE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	NewManualProbeHelper(self.printer, gcmd, self.Z_endstop_finalize)
	return nil
}

func (self *ManualProbe) cmd_Z_OFFSET_APPLY_ENDSTOP(argv interface{}) error {
	offset := self.gcode_move.Get_status(0)["homing_origin"].([]float64)[2]
	configfile := self.printer.Lookup_object("configfile", object.Sentinel{})
	if offset == 0 {
		self.gcode.Respond_info("Nothing to do: Z Offset is 0", true)
	} else {
		new_calibrate := self.z_position_endstop - offset
		self.gcode.Respond_info(fmt.Sprintf("stepper_z: Position_endstop: %.3f\n"+
			"The SAVE_CONFIG command will update the printer config file\n "+
			"with the above and restart the printer.", new_calibrate), true)
		configfile.(*PrinterConfig).Set("stepper_z", "Position_endstop",
			fmt.Sprintf("%.3f", new_calibrate))
	}
	return nil
}

func (self *ManualProbe) cmd_Z_OFFSET_APPLY_DELTA_ENDSTOPS(argv interface{}) {
	offset := self.gcode_move.Get_status(0)["homing_origin"].([]float64)[2]
	configfile := self.printer.Lookup_object("configfile", object.Sentinel{})
	if offset == 0 {
		self.gcode.Respond_info("Nothing to do: Z Offset is 0", true)
	} else {
		new_a_calibrate := self.a_position_endstop - offset
		new_b_calibrate := self.b_position_endstop - offset
		new_c_calibrate := self.c_position_endstop - offset
		self.gcode.Respond_info(fmt.Sprintf(
			"stepper_a: position_endstop: %.3f\n"+
				"stepper_b: position_endstop: %.3f\n"+
				"stepper_c: position_endstop: %.3f\n"+
				"The SAVE_CONFIG command will update the printer config file\n "+
				"with the above and restart the printer.", new_a_calibrate, new_b_calibrate, new_c_calibrate), true)
		configfile.(*PrinterConfig).Set("stepper_a", "Position_endstop",
			fmt.Sprintf("%.3f", new_a_calibrate))
		configfile.(*PrinterConfig).Set("stepper_b", "Position_endstop",
			fmt.Sprintf("%.3f", new_b_calibrate))
		configfile.(*PrinterConfig).Set("stepper_c", "Position_endstop",
			fmt.Sprintf("%.3f", new_c_calibrate))
	}
}

func (self *ManualProbe) cmd_Z_OFFSET_APPLY_ENDSTOP_help() string {
	return "Adjust the z endstop_position"
}

// Verify that a manual probe isn't already in progress
func Verify_no_manual_probe(printer *Printer) {
	gcode := printer.Lookup_object("gcode", object.Sentinel{})
	err := gcode.(*GCodeDispatch).Register_command("ACCEPT", "dummy", false, "")
	if err != nil {
		panic("Already in a manual Z probe. Use ABORT to abort it.")
	}
	gcode.(*GCodeDispatch).Register_command("ACCEPT", nil, false, "")
}

const Z_BOB_MINIMUM = 0.500
const BISECT_MAX = 0.200

// Helper script to determine a Z height
type ManualProbeHelper struct {
	printer                                *Printer
	finalize_callback                      func([]float64)
	gcode                                  *GCodeDispatch
	toolhead                               *Toolhead
	manual_probe                           *ManualProbe
	speed                                  float64
	past_positions                         []float64
	last_toolhead_pos, last_kinematics_pos []float64
	start_position                         []float64
}

func NewManualProbeHelper(printer *Printer, gcmd *GCodeCommand, finalize_callback func([]float64)) *ManualProbeHelper {
	self := &ManualProbeHelper{}
	self.printer = printer
	self.finalize_callback = finalize_callback
	self.gcode = MustLookupGcode(self.printer)
	self.toolhead = MustLookupToolhead(self.printer)
	self.manual_probe = self.printer.Lookup_object("manualprobe", object.Sentinel{}).(*ManualProbe)
	self.speed = gcmd.Get_float("SPEED", 5., nil, nil, nil, nil)
	self.past_positions = []float64{}
	self.last_toolhead_pos = nil
	self.last_kinematics_pos = nil
	Verify_no_manual_probe(printer)
	self.gcode.Register_command("ACCEPT", self.cmd_ACCEPT, false, self.cmd_ACCEPT_help())
	self.gcode.Register_command("NEXT", self.cmd_ACCEPT, false, "")
	self.gcode.Register_command("ABORT", self.cmd_ABORT, false, self.cmd_ABORT_help())
	self.gcode.Register_command("TESTZ", self.cmd_TESTZ, false, self.cmd_TESTZ_help())
	self.gcode.Respond_info(
		"Starting manual Z probe. Use TESTZ to adjust position.\n"+
			"Finish with ACCEPT or ABORT command.", true)
	self.start_position = self.toolhead.Get_position()
	self.Report_z_status(false, 0)
	return self
}

func (self *ManualProbeHelper) Get_kinematics_pos() []float64 {
	toolhead_pos := self.toolhead.Get_position()

	if reflect.DeepEqual(toolhead_pos, self.last_toolhead_pos) {
		return self.last_kinematics_pos
	}
	self.toolhead.Flush_step_generation()
	kin := self.toolhead.Get_kinematics().(IKinematics)
	kin_pos := map[string]float64{}
	for _, s := range kin.Get_steppers() {
		kin_pos[s.(*MCU_stepper).Get_name(false)] = s.(*MCU_stepper).Get_commanded_position()
	}

	_kin_pos := kin.Calc_position(kin_pos)
	self.last_toolhead_pos = toolhead_pos
	self.last_kinematics_pos = _kin_pos
	return _kin_pos
}

func (self *ManualProbeHelper) Move_z(z_pos float64) {
	curpos := self.toolhead.Get_position()
	func() {
		defer func() {
			if err := recover(); err != nil {
				self.Finalize(false)
			}
		}()
		z_bob_pos := z_pos + Z_BOB_MINIMUM
		if curpos[2] < z_bob_pos {
			self.toolhead.Manual_move([]interface{}{nil, nil, z_bob_pos}, self.speed)
		}
		self.toolhead.Manual_move([]interface{}{nil, nil, z_pos}, self.speed)
	}()
}

func (self *ManualProbeHelper) Report_z_status(warn_no_change bool, prev_pos float64) {
	// Get position
	kin_pos := self.Get_kinematics_pos()
	z_pos := kin_pos[2]
	if warn_no_change && z_pos == prev_pos {
		self.gcode.Respond_info(
			"WARNING: No change in position (reached stepper resolution)", true)
		// Find recent positions that were tested
		pp := self.past_positions
		next_pos := bisectLeft(pp, z_pos)
		prev_pos := next_pos - 1
		if next_pos < len(pp) && pp[next_pos] == z_pos {
			next_pos++
		}
		prev_pos_val := 0.0
		next_pos_val := 0.0
		prev_str := "??????"
		next_str := "??????"
		if prev_pos >= 0 {
			prev_pos_val = pp[prev_pos]
			prev_str = fmt.Sprintf("%.3f", prev_pos_val)
		}
		if next_pos < len(pp) {
			next_pos_val = pp[next_pos]
			next_str = fmt.Sprintf("%.3f", next_pos_val)
		}
		self.manual_probe.status = map[string]interface{}{
			"isActive":       true,
			"zPosition":      z_pos,
			"zPositionLower": prev_pos_val,
			"zPositionUpper": next_pos_val,
		}
		// Find recent positions
		self.gcode.Respond_info(fmt.Sprintf("Z position: %s --> %.3f <-- %s",
			prev_str, z_pos, next_str), true)
	}
}

func (self *ManualProbeHelper) cmd_ACCEPT_help() string {
	return "Accept the current Z position"
}

func (self *ManualProbeHelper) cmd_ACCEPT(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	pos := self.toolhead.Get_position()
	start_pos := self.start_position
	if reflect.DeepEqual(pos[:2], start_pos[:2]) == false || pos[2] >= start_pos[2] {
		gcmd.Respond_info("Manual probe failed! Use TESTZ commands to position the\n"+
			"nozzle prior to running ACCEPT.", true)
		self.Finalize(false)
		return nil
	}
	self.Finalize(true)
	return nil
}

func (self *ManualProbeHelper) cmd_ABORT_help() string {
	return "Abort manual Z probing tool"
}

func (self *ManualProbeHelper) cmd_ABORT(argv interface{}) error {
	self.Finalize(false)
	return nil
}

func (self *ManualProbeHelper) cmd_TESTZ_help() string {
	return "Move to new Z height"
}

func (self *ManualProbeHelper) cmd_TESTZ(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Store current position for later reference
	kin_pos := self.Get_kinematics_pos()
	z_pos := kin_pos[2]
	pp := self.past_positions
	insert_pos := bisectLeft(pp, z_pos)
	if insert_pos >= len(pp) || (pp)[insert_pos] != z_pos {
		pp[insert_pos] = z_pos
	}
	// Determine next position to move to
	req := gcmd.Get("Z", object.Sentinel{}, nil, nil, nil, nil, nil)
	next_z_pos := 0.0
	if req == "+" || req == "++" {
		check_z := 9999999999999.9
		if insert_pos < len(self.past_positions)-1 {
			check_z = self.past_positions[insert_pos+1]
		}
		if req == "+" {
			check_z = (check_z + z_pos) / 2.
		}
		next_z_pos = math.Min(check_z, z_pos+BISECT_MAX)
	} else if req == "-" || req == "--" {
		check_z := -9999999999999.9
		if insert_pos > 0 {
			check_z = self.past_positions[insert_pos-1]
		}
		if req == "-" {
			check_z = (check_z + z_pos) / 2.
		}
		next_z_pos = math.Max(check_z, z_pos-BISECT_MAX)
	} else {
		next_z_pos = z_pos + gcmd.Get_float("Z", nil, nil, nil, nil, nil)
	}
	// Move to given position and report it
	self.Move_z(next_z_pos)
	self.Report_z_status(next_z_pos != z_pos, z_pos)
	return nil
}

func (self *ManualProbeHelper) Finalize(success bool) {
	self.manual_probe.Reset_status()
	self.gcode.Register_command("ACCEPT", nil, false, "")
	self.gcode.Register_command("NEXT", nil, false, "")
	self.gcode.Register_command("ABORT", nil, false, "")
	self.gcode.Register_command("TESTZ", nil, false, "")
	kin_pos := []float64{}
	if success {
		kin_pos = self.Get_kinematics_pos()
	}
	self.finalize_callback(kin_pos)
}

func Load_config_ManualProbe(config *ConfigWrapper) *ManualProbe {
	return NewManualProbe(config)
}
