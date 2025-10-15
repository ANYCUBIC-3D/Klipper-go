package project

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/collections"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
	"path/filepath"
	"reflect"
	"runtime/debug"
	"strconv"
	"strings"
	"time"
)

type TestAxis struct {
	name    string
	vib_dir []float64
}

func NewTestAxis(axis string, vib_dir []float64) *TestAxis {
	self := new(TestAxis)
	if value.IsNone(axis) {
		self.name = fmt.Sprintf("axis=%.3f,%.3f", vib_dir[0], vib_dir[1])
	} else {
		self.name = axis
	}
	if value.IsNone(vib_dir) {
		if axis == "x" {
			self.vib_dir = []float64{1.0, 0.0}
		} else {
			self.vib_dir = []float64{0.0, 1.0}
		}
	} else {
		sum := 0.0
		for _, val := range vib_dir {
			sum += val * val
		}
		s := math.Sqrt(sum)
		vib_dir := []float64{}
		for idx, val := range vib_dir {
			vib_dir[idx] = val / s
		}
		self.vib_dir = vib_dir
	}
	return self
}

func (self *TestAxis) Matches(chip_axis string) bool {
	if self.vib_dir[0] != 0.0 && strings.Index(chip_axis, "x") != -1 {
		return true
	}
	if self.vib_dir[1] != 0.0 && strings.Index(chip_axis, "y") != -1 {
		return true
	}
	return false
}

func (self *TestAxis) Get_name() string {
	return self.name
}

func (self *TestAxis) Get_point(l float64) (float64, float64) {
	return self.vib_dir[0] * l, self.vib_dir[1] * l
}

func parse_axis(gcmd interface{}, raw_axis string) *TestAxis {
	if raw_axis == "" {
		return nil
	}
	raw_axis = strings.ToLower(raw_axis)
	if raw_axis == "x" || raw_axis == "y" {
		return NewTestAxis(raw_axis, nil)
	}
	dirs := strings.Split(raw_axis, ",")
	if len(dirs) != 2 {
		panic(fmt.Errorf("Invalid format of axis '%s'", raw_axis))
	}
	dir_x, err1 := strconv.ParseFloat(strings.TrimSpace(dirs[0]), 64)
	dir_y, err2 := strconv.ParseFloat(strings.TrimSpace(dirs[1]), 64)
	if err1 != nil || err2 != nil {
		panic(fmt.Errorf("Unable to parse axis direction '%s'", raw_axis))
	}

	return NewTestAxis("", []float64{dir_x, dir_y})
}

type VibrationPulseTest struct {
	printer      *Printer
	gcode        *GCodeDispatch
	min_freq     float64
	max_freq     float64
	accel_per_hz float64
	hz_per_sec   float64
	probe_points [][]float64
	freq_start   float64
	freq_end     float64
}

func NewVibrationPulseTest(config *ConfigWrapper) *VibrationPulseTest {
	self := new(VibrationPulseTest)

	self.printer = config.Get_printer()
	self.gcode = MustLookupGcode(self.printer)
	self.min_freq = config.Getfloat("min_freq", 5., 1., 0, 0, 0, true)
	// Defaults are such that max_freq * accel_per_hz == 10000 (max_accel)
	self.max_freq = config.Getfloat("max_freq", 10000./75.,
		self.min_freq, 200., 0, 0, true)
	self.accel_per_hz = config.Getfloat("accel_per_hz", 75., 0, 0, 0, 0., true)
	self.hz_per_sec = config.Getfloat("hz_per_sec", 1.,
		0.1, 2., 0, 0, true)
	probe_points := config.Getlists("probe_points", nil, []string{",", "\n"}, 3, reflect.Float64, true)

	for _, v := range probe_points.([][]interface{}) {
		_probe_points := []float64{}
		for _, a := range v {
			_probe_points = append(_probe_points, a.(float64))
		}
		self.probe_points = append([][]float64{}, _probe_points)
	}
	return self
}

func (self *VibrationPulseTest) Get_start_test_points() [][]float64 {
	return self.probe_points
}

func (self *VibrationPulseTest) Prepare_test(gcmd *GCodeCommand) {
	self.freq_start = gcmd.Get_float("FREQ_START", self.min_freq, cast.Float64P(1.), nil, nil, nil)
	self.freq_end = gcmd.Get_float("FREQ_END", self.max_freq,
		cast.Float64P(self.freq_start), cast.Float64P(200.), nil, nil)
	self.hz_per_sec = gcmd.Get_float("HZ_PER_SEC", self.hz_per_sec,
		nil, cast.Float64P(2.), cast.Float64P(0.), nil)
}

func (self *VibrationPulseTest) Run_test(axis TestAxis, gcmd *GCodeCommand) {
	toolhead := MustLookupToolhead(self.printer)
	pos := toolhead.Get_position()
	X, Y, Z, E := pos[0], pos[1], pos[2], pos[3]
	sign := 1.0
	freq := self.freq_start
	// Override maximum acceleration and acceleration to
	// deceleration based on the maximum test frequency
	systime := self.printer.Get_reactor().Monotonic()
	toolhead_info := toolhead.Get_status(systime)
	old_max_accel := toolhead_info["max_accel"]
	old_max_accel_to_decel := toolhead_info["max_accel_to_decel"]
	max_accel := self.freq_end * self.accel_per_hz
	self.gcode.Run_script_from_command(fmt.Sprintf(
		"SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f",
		max_accel, max_accel))

	input_shaper := self.printer.Lookup_object("input_shaper", value.None)
	if value.IsNotNone(input_shaper) && value.Not(gcmd.Get_int("INPUT_SHAPING", 0, nil, nil)) {
		input_shaper.(*InputShaper).Disable_shaping()
		logger.Debug("Disabled [input_shaper] for resonance testing")
	} else {
		input_shaper = nil
	}

	gcmd.Respond_info(fmt.Sprintf("Testing frequency %.0f Hz", freq), true)
	for freq <= self.freq_end+0.000001 {
		t_seg := .25 / freq
		accel := self.accel_per_hz * freq
		max_v := accel * t_seg
		toolhead.M204(accel)
		L := .5 * accel * math.Pow(t_seg, 2)
		dX, dY := axis.Get_point(L)
		nX := X + sign*dX
		nY := Y + sign*dY
		toolhead.Move([]float64{nX, nY, Z, E}, max_v)
		toolhead.Move([]float64{X, Y, Z, E}, max_v)
		sign = -sign
		old_freq := freq
		freq += 2. * t_seg * self.hz_per_sec
		if math.Floor(freq) > math.Floor(old_freq) {
			gcmd.Respond_info(fmt.Sprintf("Testing frequency %.0f Hz", freq), true)
		}
	}

	// Restore the original acceleration values
	self.gcode.Run_script_from_command(fmt.Sprintf(
		"SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f",
		old_max_accel, old_max_accel_to_decel))
	// Restore input shaper if it was disabled for resonance testing
	if input_shaper != nil {
		input_shaper.(*InputShaper).Enable_shaping()
		gcmd.Respond_info("Re-enabled [input_shaper]", true)
	}
}

type Accel_chip_names_value struct {
	Chip_axis string
	Chip_name string
}

type Accel_chips_value struct {
	Chip_axis string
	Chip      IAccelChip
}

type raw_value struct {
	Chip_axis interface{}
	Aclient   IAclient
	Name      string
}

type ResonanceTester struct {
	printer          *Printer
	gcode            *GCodeDispatch
	move_speed       float64
	test             *VibrationPulseTest
	accel_chip_names [][2]string
	max_smoothing    float64
	accel_chips      []Accel_chips_value
}

func NewResonanceTester(config *ConfigWrapper) *ResonanceTester {
	self := new(ResonanceTester)

	self.printer = config.Get_printer()
	self.move_speed = config.Getfloat("move_speed", 50., 0, 0, 0., 0, true)
	self.test = NewVibrationPulseTest(config)

	if config.Get("accel_chip_x", value.None, true) == nil {
		self.accel_chip_names = [][2]string{{
			"xy", strings.TrimSpace(cast.ToString(config.Get("accel_chip", value.None, true))),
		}}
	} else {
		self.accel_chip_names = [][2]string{
			{
				"x", strings.TrimSpace(cast.ToString(config.Get("accel_chip_x", value.None, true))),
			},
			{
				"y", strings.TrimSpace(cast.ToString(config.Get("accel_chip_y", value.None, true))),
			},
		}

		if self.accel_chip_names[0][1] == self.accel_chip_names[1][1] {
			self.accel_chip_names = [][2]string{{
				"xy", self.accel_chip_names[0][1],
			}}
		}
	}
	self.max_smoothing = config.Getfloat("max_smoothing", value.None, 0.05, 0, 0, 0, true)
	self.gcode = MustLookupGcode(self.printer)
	self.gcode.Register_command("MEASURE_AXES_NOISE",
		self.cmd_MEASURE_AXES_NOISE, false,
		cmd_MEASURE_AXES_NOISE_help)
	self.gcode.Register_command("TEST_RESONANCES",
		self.cmd_TEST_RESONANCES, false,
		cmd_TEST_RESONANCES_help)
	self.gcode.Register_command("SHAPER_CALIBRATE",
		self.cmd_SHAPER_CALIBRATE, false,
		cmd_SHAPER_CALIBRATE_help)
	self.printer.Register_event_handler("project:connect", self.connect)
	return self
}

func (self *ResonanceTester) connect(_ []interface{}) error {
	for _, names := range self.accel_chip_names {
		self.accel_chips = append(self.accel_chips, Accel_chips_value{
			Chip_axis: names[0],
			Chip:      self.printer.Lookup_object(names[1], nil).(IAccelChip),
		})
	}
	return nil
}

func (self *ResonanceTester) _run_test(gcmd *GCodeCommand, axes []*TestAxis, helper *ShaperCalibrate, raw_name_suffix string,
	accel_chips []IAccelChip, test_point []float64) map[*TestAxis]interface{} {
	toolhead := MustLookupToolhead(self.printer)
	calibration_data := make(map[*TestAxis]interface{})
	for _, item := range axes {
		calibration_data[item] = nil
	}
	self.test.Prepare_test(gcmd)

	var test_points [][]float64
	if value.IsNotNone(test_point) {
		test_points = [][]float64{test_point}
	} else {
		test_points = self.test.Get_start_test_points()
	}

	for _, point := range test_points {
		movePoints := make([]interface{}, 0, len(point))
		for _, p := range point {
			movePoints = append(movePoints, p)
		}
		toolhead.Manual_move(movePoints, self.move_speed)

		if len(test_points) > 1 || value.IsNotNone(test_point) {
			gcmd.Respond_info(fmt.Sprintf("Probing point (%.3f, %.3f, %.3f)", point[0], point[1], point[2]), true)
		}

		for _, axis := range axes {
			toolhead.Wait_moves()
			toolhead.Dwell(0.500)
			if len(axes) > 1 {
				gcmd.Respond_info(fmt.Sprintf("Testing axis %s", axis.Get_name()), true)
			}

			var raw_values []raw_value

			if value.IsNone(accel_chips) {
				for _, ac := range self.accel_chips {
					if axis.Matches(ac.Chip_axis) {
						aclient := ac.Chip.Start_internal_client()
						raw_values = append(raw_values, raw_value{ac.Chip_axis, aclient, ac.Chip.Get_name()})
					}
				}
			} else {
				for _, chip := range accel_chips {
					aclient := chip.Start_internal_client()
					raw_values = append(raw_values, raw_value{axis, aclient, chip.Get_name()})
				}
			}

			// Generate moves
			self.test.Run_test(*axis, gcmd)

			for _, rv := range raw_values {
				rv.Aclient.Finish_measurements()
				if raw_name_suffix != "" {
					var filename_point []float64
					if len(test_points) > 1 {
						filename_point = point
					}

					var filename_chipname string
					if value.IsNone(accel_chips) {
						filename_chipname = rv.Name
					}
					raw_name := self.get_filename("raw_data", raw_name_suffix,
						axis, filename_point, filename_chipname)
					rv.Aclient.Write_to_file(raw_name)
					gcmd.Respond_info(fmt.Sprintf("Writing raw accelerometer data to %s file", raw_name), true)
				}
			}

			if value.IsNone(helper) {
				continue
			}

			for _, rv := range raw_values {
				if value.Not(rv.Aclient.Has_valid_samples()) {
					panic(fmt.Errorf("accelerometer '%s' measured no data",
						rv.Name))
				}
				new_data := helper.Process_accelerometer_data(rv.Aclient)
				if value.IsNone(calibration_data[axis]) {
					calibration_data[axis] = new_data
				} else {
					calibration_data[axis].(*CalibrationData).Add_data(new_data)
				}
			}
		}
	}
	return calibration_data
}

const cmd_TEST_RESONANCES_help = ("Runs the resonance test for a specifed axis")

func (self *ResonanceTester) cmd_TEST_RESONANCES(argv interface{}) error {
	// Parse parameters
	gcmd := argv.(*GCodeCommand)
	axis := parse_axis(gcmd, strings.ToLower(gcmd.Get("AXIS", "", value.None, nil, nil, nil, nil)))

	accel_chips := gcmd.Get("CHIPS", "", value.None, nil, nil, nil, nil)
	test_point := gcmd.Get("POINT", "", value.None, nil, nil, nil, nil)

	var test_point2 []float64
	if value.True(test_point) {
		test_coords := strings.Split(test_point, ",")
		if len(test_coords) != 3 {
			panic(errors.New("Invalid POINT parameter, must be 'x,y,z'"))
		}

		test_point2 = make([]float64, 3)
		for i, v := range test_coords {
			vf, err := cast.ToFloat64E(v)
			if err != nil {
				panic(errors.New("Invalid POINT parameter, must be 'x,y,z'" +
					" where x, y and z are valid floating point numbers"))
			}
			test_point2[i] = vf
		}
	}
	var parsed_chips []IAccelChip
	if value.True(accel_chips) {
		var chip_lookup_name string
		for _, chip_name := range strings.Split(accel_chips, ",") {
			if strings.Contains(chip_name, "adxl345") {
				chip_lookup_name = strings.TrimSpace(chip_name)
			} else {
				chip_lookup_name = "adxl345 " + strings.TrimSpace(chip_name)
			}

			chip := self.printer.Lookup_object(chip_lookup_name, value.None)
			parsed_chips = append(parsed_chips, chip.(IAccelChip))
		}
	}

	outputs := strings.Split(strings.ToLower(gcmd.Get("OUTPUT", "resonances", value.None, nil, nil, nil, nil)), ",")

	for _, output := range outputs {
		if !collections.Contains([]string{"resonances", "raw_data"}, output) {
			panic(fmt.Errorf("Unsupported output '%+v', only 'resonances'"+
				" and 'raw_data' are supported", output))
		}
	}

	if value.Not(outputs) {
		panic(errors.New("No output specified, at least one of 'resonances'" +
			" or 'raw_data' must be set in OUTPUT parameter"))
	}

	name_suffix := gcmd.Get("NAME", time.Now().Format("20060102_150102"), nil, nil, nil, nil, nil)
	if !self.is_valid_name_suffix(name_suffix) {
		panic(errors.New("Invalid NAME parameter"))
	}
	var csv_output = collections.Contains(outputs, "resonances")
	var raw_output = collections.Contains(outputs, "raw_data")

	// Setup calculation of resonances
	var helper *ShaperCalibrate
	if csv_output {
		helper = NewShaperCalibrate(self.printer)
	}

	var raw_name_suffix string
	if raw_output {
		raw_name_suffix = name_suffix
	}
	var test_accel_chips []IAccelChip
	if value.True(accel_chips) {
		test_accel_chips = parsed_chips
	}
	data := self._run_test(gcmd, []*TestAxis{axis}, helper, raw_name_suffix, test_accel_chips, test_point2)[axis]

	if csv_output {
		csv_name := self.save_calibration_data("resonances", name_suffix,
			helper, axis, data, nil, test_point2)
		gcmd.Respond_info(fmt.Sprintf("Resonances data written to %s file", csv_name), true)
	}

	return nil
}

const cmd_SHAPER_CALIBRATE_help = ("Simular to TEST_RESONANCES but suggest input shaper config")

func (self *ResonanceTester) cmd_SHAPER_CALIBRATE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Parse parameters
	axis := gcmd.Get("AXIS", value.None, value.None, nil, nil, nil, nil)
	var calibrate_axes []*TestAxis
	if value.Not(axis) {
		calibrate_axes = []*TestAxis{NewTestAxis("x", nil), NewTestAxis("y", nil)}
	} else if !collections.Contains([]string{"x", "y"}, strings.ToLower(axis)) {
		panic(fmt.Errorf("Unsupported axis '%s'", axis))
	} else {
		calibrate_axes = []*TestAxis{NewTestAxis(strings.ToLower(axis), nil)}
	}

	max_smoothing := gcmd.Get_float(
		"MAX_SMOOTHING", self.max_smoothing, cast.Float64P(0.05), nil, nil, nil)

	name_suffix := gcmd.Get("NAME", time.Now().Format("20060102_150102"), value.None, nil, nil, nil, nil)
	if !self.is_valid_name_suffix(name_suffix) {
		panic(errors.New("Invalid NAME parameter"))
	}

	// Setup shaper calibration
	helper := NewShaperCalibrate(self.printer)
	calibration_data := self._run_test(gcmd, calibrate_axes, helper, "", nil, nil)
	configfile := MustLookupConfigfile(self.printer)
	for _, _axis := range calibrate_axes {
		axis_name := _axis.Get_name()
		gcmd.Respond_info(fmt.Sprintf("Calculating the best input shaper parameters for %s axis", axis_name), true)
		calibration_data[_axis].(*CalibrationData).Normalize_to_frequencies()
		best_shaper, all_shapers := helper.find_best_shaper(calibration_data[_axis].(*CalibrationData), max_smoothing)
		gcmd.Respond_info(fmt.Sprintf("Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz",
			axis_name, best_shaper.name, axis_name, best_shaper.freq), true)
		is_save := gcmd.Get_int("IS_SAVE", 1, nil, nil)
		if is_save == 1 {
			helper.Save_params(configfile, axis_name,
				best_shaper.name, best_shaper.freq)
		}

		csv_name := self.save_calibration_data(
			"calibration_data", name_suffix, helper, _axis,
			calibration_data[_axis], all_shapers, nil)
		gcmd.Respond_info(fmt.Sprintf("Shaper calibration data written to %s file", csv_name), true)
	}

	gcmd.Respond_info("The SAVE_CONFIG command will update the printer config file\n"+
		"with these parameters and restart the printer.", true)

	debug.FreeOSMemory()
	return nil
}

const cmd_MEASURE_AXES_NOISE_help = ("Measures noise of all enabled accelerometer chips")

func (self *ResonanceTester) cmd_MEASURE_AXES_NOISE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	meas_time := gcmd.Get_float("MEAS_TIME", 2., nil, nil, nil, nil)

	var raw_values []raw_value
	for _, ac := range self.accel_chips {
		raw_values = append(raw_values, raw_value{
			Chip_axis: ac.Chip_axis,
			Aclient:   ac.Chip.Start_internal_client(),
		})
	}

	toolhead := MustLookupToolhead(self.printer)
	toolhead.Dwell(meas_time)

	for _, rv := range raw_values {
		rv.Aclient.Finish_measurements()
	}
	helper := NewShaperCalibrate(self.printer)
	_ = helper
	for _, rv := range raw_values {
		if value.Not(rv.Aclient.Has_valid_samples()) {
			panic(fmt.Errorf("%+v-axis accelerometer measured no data", rv.Chip_axis))
		}
		data := helper.Process_accelerometer_data(rv.Aclient)

		vx := maths.Mean([][]float64{data.psd_x}, nil)
		vy := maths.Mean([][]float64{data.psd_y}, nil)
		vz := maths.Mean([][]float64{data.psd_z}, nil)

		gcmd.Respond_info(fmt.Sprintf("Axes noise for %s-axis accelerometer: "+
			"%.6f (x), %.6f (y), %.6f (z)",
			rv.Chip_axis, vx, vy, vz), true)

	}
	return nil
}

func (self *ResonanceTester) is_valid_name_suffix(name_suffix string) bool {
	name_suffix = strings.ReplaceAll(name_suffix, "-", "")
	name_suffix = strings.ReplaceAll(name_suffix, "_", "")
	return str.IsAlphanum(name_suffix)
}

func (self *ResonanceTester) get_filename(base, name_suffix string, axis *TestAxis, point []float64, chip_name string) string {
	name := base
	if value.True(axis) {
		name += "_" + axis.Get_name()
	}

	if value.True(chip_name) {
		name += "_" + strings.Replace(chip_name, " ", "_", -1)
	}

	if value.True(point) && len(point) == 3 {
		name += fmt.Sprintf("_%.3f_%.3f_%.3f", point[0], point[1], point[2])
	}
	name += "_" + name_suffix
	return filepath.Join("/tmp", name+".csv")
}

func (self *ResonanceTester) save_calibration_data(base_name, name_suffix string, shaper_calibrate *ShaperCalibrate,
	axis *TestAxis, calibration_data,
	all_shapers interface{}, point []float64) string {
	output := self.get_filename(base_name, name_suffix, axis, point, "")

	shaper_calibrate.Save_calibration_data(output, calibration_data, all_shapers)
	return output
}

func Load_config_resonanceTester(config *ConfigWrapper) interface{} {
	return NewResonanceTester(config)
}
