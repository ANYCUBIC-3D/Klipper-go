package project

import (
	"errors"
	"fmt"
	"k3c/common/collections"
	"k3c/common/utils/cast"
	"k3c/common/value"
	"k3c/project/chelper"
	"math"
	"reflect"
	"strings"
)

type InputShaperParams struct {
	axis          string
	shapers       map[string]func(shaper_freq, damping_ratio float64) ([]float64, []float64)
	shaper_type   string
	damping_ratio float64
	shaper_freq   float64
}

func NewInputShaperParams(axis string, config *ConfigWrapper) *InputShaperParams {
	self := new(InputShaperParams)
	self.axis = axis
	self.shapers = make(map[string]func(shaper_freq, damping_ratio float64) ([]float64, []float64))
	for _, shaper := range INPUT_SHAPERS {
		self.shapers[shaper.Name] = shaper.Init_func
	}

	shaper_type := config.Get("shaper_type", "mzv", true)
	self.shaper_type = config.Get("shaper_type_"+axis, shaper_type, true).(string)

	if _, ok := self.shapers[self.shaper_type]; !ok {
		panic(fmt.Errorf("supported shaper type: %s", self.shaper_type))
	}

	self.damping_ratio = config.Getfloat("damping_ratio_"+axis,
		DEFAULT_DAMPING_RATIO,
		0., 1., 0, 0, true)
	self.shaper_freq = config.Getfloat("shaper_freq_"+axis, 0., 0., 0, 0, 0, true)
	return self
}

func (self *InputShaperParams) Update(gcmd *GCodeCommand) {
	axis := strings.ToUpper(self.axis)
	self.damping_ratio = gcmd.Get_float("DAMPING_RATIO_"+axis,
		self.damping_ratio,
		cast.Float64P(0.), cast.Float64P(1.), nil, nil)
	self.shaper_freq = gcmd.Get_float("SHAPER_FREQ_"+axis,
		self.shaper_freq, cast.Float64P(0.), nil, nil, nil)
	shaper_type := gcmd.Get("SHAPER_TYPE", value.None, nil, nil, nil, nil, nil)
	if value.IsNone(shaper_type) {
		shaper_type = gcmd.Get("SHAPER_TYPE_"+axis, self.shaper_type, value.None, nil, nil, nil, nil)
	}

	if _, ok := self.shapers[strings.ToLower(shaper_type)]; !ok {
		panic(fmt.Errorf("Unsupported shaper type: %s", shaper_type))
	}

	self.shaper_type = strings.ToLower(shaper_type)
}

func (self *InputShaperParams) Get_shaper() (int, []float64, []float64) {
	var A, T []float64
	if value.Not(self.shaper_freq) {
		A, T = Get_none_shaper()
	} else {
		A, T = self.shapers[self.shaper_type](
			self.shaper_freq, self.damping_ratio)
	}

	return len(A), A, T
}

func (self *InputShaperParams) get_status() *collections.SortedMap {

	m := map[string]interface{}{
		"shaper_type":   self.shaper_type,
		"shaper_freq":   fmt.Sprintf("%.3f", self.shaper_freq),
		"damping_ratio": fmt.Sprintf("%.6f", self.damping_ratio),
	}
	return collections.NewSortedMap1([]string{"shaper_type", "shaper_freq", "damping_ratio"}, m)
}

type AxisInputShaper struct {
	axis   string
	params *InputShaperParams
	n      int
	A      []float64
	T      []float64
	saved  *AxisInputShaper_saved
}

type AxisInputShaper_saved struct {
	n int
	A []float64
	T []float64
}

func NewAxisInputShaper(axis string, config *ConfigWrapper) *AxisInputShaper {
	self := new(AxisInputShaper)
	self.axis = axis
	self.params = NewInputShaperParams(axis, config)
	self.n, self.A, self.T = self.params.Get_shaper()
	self.saved = nil
	return self
}

func (self *AxisInputShaper) Get_name() string {
	return "shaper_" + self.axis
}

func (self *AxisInputShaper) Get_shaper() (int, []float64, []float64) {
	return self.n, self.A, self.T
}

func (self *AxisInputShaper) Update(gcmd *GCodeCommand) bool {
	self.params.Update(gcmd)
	old_n, old_A, old_T := self.n, self.A, self.T
	self.n, self.A, self.T = self.params.Get_shaper()

	return old_n != self.n &&
		!reflect.DeepEqual(old_A, self.A) &&
		!reflect.DeepEqual(old_T, self.T)
}

func (self *AxisInputShaper) Set_shaper_kinematics(sk interface{}) bool {
	success := chelper.Input_shaper_set_shaper_params(
		sk, int8(self.axis[0]), self.n, self.A, self.T) == 0

	if !success {
		self.disable_shaping()
		chelper.Input_shaper_set_shaper_params(
			sk, int8(self.axis[0]), self.n, self.A, self.T)
	}
	return success
}

func (self *AxisInputShaper) Get_step_generation_window() float64 {
	if len(self.A) == 0 || len(self.T) == 0 {
		return 0.
	}
	return chelper.Input_shaper_get_step_generation_window(self.n,
		self.A, self.T)
}

func (self *AxisInputShaper) disable_shaping() {
	if value.IsNone(self.saved) && value.True(self.n) {
		self.saved = &AxisInputShaper_saved{
			self.n,
			self.A,
			self.T,
		}
	}

	A, T := Get_none_shaper()
	self.n, self.A, self.T = len(A), A, T
}

func (self *AxisInputShaper) Enable_shaping() {
	if value.IsNone(self.saved) { //   Input shaper was not disabled
		return
	}

	self.n = self.saved.n
	self.A = self.saved.A
	self.T = self.saved.T
	self.saved = nil
}

func (self *AxisInputShaper) Report(gcmd *GCodeCommand) {
	var statusItems []string
	self.params.get_status().Range(func(key string, value interface{}) bool {
		statusItems = append(statusItems, fmt.Sprintf("%s_%s:%s", key, self.axis, value))
		return true
	})

	gcmd.Respond_info(strings.Join(statusItems, " "), true)
}

type InputShaper struct {
	printer                 *Printer
	toolhead                *Toolhead
	shapers                 []*AxisInputShaper
	stepper_kinematics      []interface{}
	orig_stepper_kinematics []interface{}
	old_delay               float64
}

func NewInputShaper(config *ConfigWrapper) *InputShaper {
	self := new(InputShaper)

	self.printer = config.Get_printer()
	self.printer.Register_event_handler("project:connect", self.Connect)
	self.toolhead = nil

	self.shapers = []*AxisInputShaper{
		NewAxisInputShaper("x", config),
		NewAxisInputShaper("y", config)}
	self.stepper_kinematics = nil
	self.orig_stepper_kinematics = nil

	// Register gcode commands
	gcode := MustLookupGcode(self.printer)
	gcode.Register_command("SET_INPUT_SHAPER",
		self.Cmd_SET_INPUT_SHAPER, false,
		Cmd_SET_INPUT_SHAPER_help)

	return self
}

func (self *InputShaper) Get_shapers() []*AxisInputShaper {
	return self.shapers
}

func (self *InputShaper) Connect(_ []interface{}) error {
	self.toolhead = MustLookupToolhead(self.printer)
	kin := self.toolhead.Get_kinematics()
	// Lookup stepper kinematics
	steppers := kin.(IKinematics).Get_steppers()
	for _, s := range steppers {
		sk := chelper.Input_shaper_alloc()
		orig_sk := s.(*MCU_stepper).Set_stepper_kinematics(sk)
		res := chelper.Input_shaper_set_sk(sk, orig_sk)
		if res < 0 {
			s.(*MCU_stepper).Set_stepper_kinematics(orig_sk)
			continue
		}
		self.stepper_kinematics = append(self.stepper_kinematics, sk)
		self.orig_stepper_kinematics = append(self.orig_stepper_kinematics, orig_sk)
	}
	// Configure initial values
	self.old_delay = 0.
	self._update_input_shaping(errors.New("printer config_error"))
	return nil
}

func (self *InputShaper) _update_input_shaping(err error) {
	self.toolhead.Flush_step_generation()

	var new_delay float64
	for _, s := range self.shapers {
		new_delay = math.Max(new_delay, s.Get_step_generation_window()) // Get_step_generation_window() assert >0
	}
	self.toolhead.Note_step_generation_scan_time(new_delay, self.old_delay)
	var failed []*AxisInputShaper
	for _, sk := range self.stepper_kinematics {
		for _, shaper := range self.shapers {
			if self.contains(failed, shaper) {
				continue
			}

			if value.Not(shaper.Set_shaper_kinematics(sk)) {
				failed = append(failed, shaper)
			}
		}
	}

	if len(failed) > 0 {
		names := make([]string, 0, len(failed))
		for _, s := range failed {
			names = append(names, s.Get_name())
		}

		if err == nil {
			err = errors.New("printer command error")
		}
		panic(fmt.Errorf("Failed to configure shaper(s) %s with given parameters, error: %w", strings.Join(names, ", "), err))
	}
}

func (self *InputShaper) contains(elems []*AxisInputShaper, v *AxisInputShaper) bool {
	for _, s := range elems {
		if v == s {
			return true
		}
	}
	return false
}

func (self *InputShaper) Disable_shaping() {
	for _, shaper := range self.shapers {
		shaper.disable_shaping()
	}
	self._update_input_shaping(nil)
}

func (self *InputShaper) Enable_shaping() {
	for _, shaper := range self.shapers {
		shaper.Enable_shaping()
	}

	self._update_input_shaping(nil)
}

const Cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"

func (self *InputShaper) Cmd_SET_INPUT_SHAPER(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	var updated = false
	for _, shaper := range self.shapers {
		if shaper.Update(gcmd) { // python updated |= shaper.update(gcmd)
			updated = true
		}
	}

	if updated {
		self._update_input_shaping(nil)
	}

	for _, shaper := range self.shapers {
		shaper.Report(gcmd)
	}
}

func Load_config_InputShaper(config *ConfigWrapper) interface{} {
	return NewInputShaper(config)
}
