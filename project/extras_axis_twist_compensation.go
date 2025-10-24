package project

import (
	"fmt"
	"k3c/common/utils/object"
	"math"
	"strings"
)

const (
	DEFAULT_SAMPLE_COUNT      = 3
	DEFAULT_SPEED             = 50.
	DEFAULT_HORIZONTAL_MOVE_Z = 5.
)

type AxisTwistCompensation struct {
	printer              *Printer
	gcode                *GCodeDispatch
	horizontal_move_z    float64
	speed                float64
	calibrate_start_x    float64
	calibrate_end_x      float64
	calibrate_y          float64
	z_compensations      []float64
	compensation_start_x float64
	compensation_end_x   float64
	m                    interface{}
	b                    interface{}
	calibrater           *Calibrater
}

func NewAxisTwistCompensation(config *ConfigWrapper) interface{} {
	atc := &AxisTwistCompensation{}
	atc.printer = config.Get_printer()
	atc.gcode = atc.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)

	// get values from [axis_twist_compensation] section in printer .cfg
	atc.horizontal_move_z = config.Getfloat("horizontal_move_z",
		DEFAULT_HORIZONTAL_MOVE_Z, 0, 0, 0, 0, true)
	atc.speed = config.Getfloat("speed", DEFAULT_SPEED, 0, 0, 0, 0, true)
	atc.calibrate_start_x = config.Getfloat("calibrate_start_x", object.Sentinel{}, 0, 0, 0, 0, true)
	atc.calibrate_end_x = config.Getfloat("calibrate_end_x", object.Sentinel{}, 0, 0, 0, 0, true)
	atc.calibrate_y = config.Getfloat("calibrate_y", object.Sentinel{}, 0, 0, 0, 0, true)
	atc.z_compensations = config.Getfloatlist("z_compensations", []float64{}, ",", 0, true)
	atc.compensation_start_x = config.Getfloat("compensation_start_x", nil, 0, 0, 0, 0, true)
	atc.compensation_end_x = config.Getfloat("compensation_start_y", nil, 0, 0, 0, 0, true)

	atc.m = nil
	atc.b = nil

	// setup calibrater
	atc.calibrater = NewCalibrater(atc, config)

	return atc
}

func (atc *AxisTwistCompensation) get_z_compensation_value(pos []float64) float64 {
	if len(atc.z_compensations) == 0 {
		return 0
	}

	x_coord := pos[0]
	z_compensations := atc.z_compensations
	sample_count := len(z_compensations)
	spacing := (atc.calibrate_end_x - atc.calibrate_start_x) / float64(sample_count-1)
	interpolate_t := (x_coord - atc.calibrate_start_x) / spacing
	interpolate_i := int(math.Floor(interpolate_t))
	interpolate_i = int(Constrain(math.Floor(interpolate_t), 0, float64(sample_count-2)))
	interpolate_t -= float64(interpolate_i)
	interpolated_z_compensation := Lerp(interpolate_t, z_compensations[interpolate_i], z_compensations[interpolate_i+1])
	return interpolated_z_compensation
}

func (atc *AxisTwistCompensation) clear_compensations() {
	atc.z_compensations = []float64{}
	atc.m = nil
	atc.b = nil
}

type Calibrater struct {
	compensation        *AxisTwistCompensation
	printer             *Printer
	gcode               *GCodeDispatch
	probe               *PrinterProbe
	lift_speed          float64
	probe_x_offset      float64
	probe_y_offset      float64
	speed               float64
	horizontal_move_z   float64
	start_point         []float64
	end_point           []float64
	results             []float64
	current_point_index int
	gcmd                *GCodeCommand
	configname          string
	current_measured_z  float64
}

func NewCalibrater(compensation *AxisTwistCompensation, config *ConfigWrapper) *Calibrater {
	ca := &Calibrater{}
	ca.compensation = compensation
	ca.printer = compensation.printer
	ca.gcode = ca.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	ca.probe = nil
	ca.printer.Register_event_handler("klippy:connect", ca._handle_connect)
	ca.speed = compensation.speed
	ca.horizontal_move_z = compensation.horizontal_move_z
	ca.start_point = []float64{compensation.calibrate_start_x, compensation.calibrate_y}
	ca.end_point = []float64{compensation.calibrate_end_x, compensation.calibrate_y}
	ca.results = nil
	ca.current_point_index = 0
	ca.gcmd = nil
	ca.configname = config.Get_name()

	// register gcode handlers
	ca._register_gcode_handlers()
	return ca
}

func (ca *Calibrater) _handle_connect(arg []interface{}) error {
	ca.probe = ca.printer.Lookup_object("probe", nil).(*PrinterProbe)
	if ca.probe == nil {
		// config := ca.printer.Lookup_object("configfile", object.Sentinel{}).(*project.ConfigWrapper)
		panic(Config_error{"AXIS_TWIST_COMPENSATION requires [probe] to be defined"})
	}
	ca.lift_speed = ca.probe.Get_lift_speed(nil)
	ca.probe_x_offset, ca.probe_y_offset, _ = ca.probe.Get_offsets()
	return nil
}

func (ca *Calibrater) _register_gcode_handlers() {
	// register gcode handlers
	ca.gcode = ca.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	ca.gcode.Register_command("AXIS_TWIST_COMPENSATION_CALIBRATE", ca.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE, false, "")
}

func (ca *Calibrater) cmd_AXIS_TWIST_COMPENSATION_CALIBRATE(gcmd *GCodeCommand) {
	ca.gcmd = gcmd
	sample_count := gcmd.Get_int("SAMPLE_COUNT", DEFAULT_SAMPLE_COUNT, nil, nil)

	// check for valid sample_count
	if sample_count == 0 || sample_count < 2 {
		panic("SAMPLE_COUNT to probe must be at least 2")
	}

	// clear the current config
	ca.compensation.clear_compensations()

	// calculate some values
	x_range := ca.end_point[0] - ca.start_point[0]
	interval_dist := x_range / float64(sample_count-1)
	nozzle_points := ca._calculate_nozzle_points(sample_count,
		interval_dist)
	probe_points := ca._calculate_probe_points(
		nozzle_points, ca.probe_x_offset, ca.probe_y_offset)

	// verify no other manual probe is in progress
	Verify_no_manual_probe(ca.printer)

	// begin calibration
	ca.current_point_index = 0
	ca.results = []float64{}
	ca._calibration(probe_points, nozzle_points, interval_dist)
}

func (ca *Calibrater) _calculate_nozzle_points(sample_count int, interval_dist float64) [][]float64 {
	// calculate the points to put the probe at, returned as a list of tuples
	nozzle_points := [][]float64{}
	for i := 0; i < sample_count; i++ {
		x := ca.start_point[0] + float64(i)*interval_dist
		y := ca.start_point[1]
		nozzle_points = append(nozzle_points, []float64{x, y})
	}
	return nozzle_points
}

func (ca *Calibrater) _calculate_probe_points(nozzle_points [][]float64, probe_x_offset float64, probe_y_offset float64) [][]float64 {
	// calculate the points to put the nozzle at
	// returned as a list of tuples
	probe_points := [][]float64{}
	for _, point := range nozzle_points {
		x := point[0] - probe_x_offset
		y := point[1] - probe_y_offset
		probe_points = append(probe_points, []float64{x, y})
	}
	return probe_points
}

func (ca *Calibrater) _move_helper(target_coordinates []interface{}, override_speed float64) {
	// pad target coordinates
	target_coordinates_inner := []interface{}{}
	if len(target_coordinates) == 2 {
		target_coordinates_inner = []interface{}{target_coordinates[0], target_coordinates[1], nil}
	} else {
		target_coordinates_inner = target_coordinates
	}

	toolhead := ca.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	speed := 0.0
	if target_coordinates[2] == nil {
		speed = ca.speed
	} else {
		speed = ca.lift_speed
	}
	if override_speed == 0.0 {
		speed = override_speed
	}

	toolhead.Manual_move(target_coordinates_inner, speed)
}

func (ca *Calibrater) _calibration(probe_points [][]float64, nozzle_points [][]float64, interval float64) {
	// begin the calibration process
	ca.gcmd.Respond_info(fmt.Sprintf("AXIS_TWIST_COMPENSATION_CALIBRATE: Probing point %d of %d",
		ca.current_point_index+1, len(probe_points)), true)

	// horizontal_move_z (to prevent probe trigger or hitting bed)
	ca._move_helper([]interface{}{nil, nil, ca.horizontal_move_z}, 0.0)

	// move to point to probe
	ca._move_helper([]interface{}{probe_points[ca.current_point_index][0],
		probe_points[ca.current_point_index][1], nil}, 0.0)

	// probe the point
	ca.current_measured_z = ca.probe.Run_probe(ca.gcmd)[2]

	// horizontal_move_z (to prevent probe trigger or hitting bed)
	ca._move_helper([]interface{}{nil, nil, ca.horizontal_move_z}, 0.0)

	// move the nozzle over the probe point
	ca._move_helper([]interface{}{nozzle_points[ca.current_point_index]}, 0.0)

	// start the manual (nozzle) probe
	NewManualProbeHelper(
		ca.printer, ca.gcmd,
		ca._manual_probe_callback_factory(
			probe_points, nozzle_points, interval))
}

func (ca *Calibrater) _manual_probe_callback_factory(probe_points [][]float64, nozzle_points [][]float64, interval float64) func([]float64) {
	// returns a callback function for the manual probe
	is_end := ca.current_point_index == len(probe_points)-1

	callback := func(kin_pos []float64) {
		if len(kin_pos) == 0 {
			// probe was cancelled
			ca.gcmd.Respond_info(
				"AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, "+
					"calibration aborted", true)
			return
		}
		z_offset := ca.current_measured_z - kin_pos[2]
		ca.results = append(ca.results, z_offset)
		if is_end {
			// end of calibration
			ca._finalize_calibration()
		} else {
			// move to next point
			ca.current_point_index += 1
			ca._calibration(probe_points, nozzle_points, interval)
		}
	}
	return callback
}

func (ca *Calibrater) _finalize_calibration() {
	// finalize the calibration process
	// calculate average of results
	results_sum := 0.0
	for _, v := range ca.results {
		results_sum += v
	}
	avg := results_sum / float64(len(ca.results))
	// subtract average from each result
	// so that they are independent of z_offset
	result := []float64{}
	for _, x := range ca.results {
		result = append(result, avg-x)
	}
	ca.results = result
	// save the config
	configfile := ca.printer.Lookup_object("configfile", object.Sentinel{}).(*PrinterConfig)
	values_as_arr := []string{}
	for _, x := range ca.results {
		values_as_arr = append(values_as_arr, fmt.Sprintf("{:%6f}", x))
	}
	values_as_str := strings.Join(values_as_arr, ", ")
	configfile.Set(ca.configname, "z_compensations", values_as_str)

	configfile.Set(ca.configname, "compensation_start_x",
		fmt.Sprintf("%f", ca.start_point[0]))
	configfile.Set(ca.configname, "compensation_end_x",
		fmt.Sprintf("%f", ca.end_point[0]))
	ca.compensation.z_compensations = ca.results
	ca.compensation.compensation_start_x = ca.start_point[0]
	ca.compensation.compensation_end_x = ca.end_point[0]
	ca.gcode.Respond_info(
		"AXIS_TWIST_COMPENSATION state has been saved "+
			"for the current session.  The SAVE_CONFIG command will "+
			"update the printer config file and restart the printer.", true)
	// output result
	ca.gcmd.Respond_info(fmt.Sprintf(
		"AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, "+
			"offsets: %v, mean z_offset: %f",
		ca.results, avg), true)
}

// klipper's entry point using [axis_twist_compensation] section in printer.cfg
func Load_config_AxisTwistCompensation(config *ConfigWrapper) interface{} {
	return NewAxisTwistCompensation(config)
}
