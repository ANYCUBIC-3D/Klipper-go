// Tracking of PWM controlled heaters and their temperature control
//
// Copyright (C) 2016-2020  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/collections"
	"k3c/common/utils/object"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
	"os"
	"path"
	"sort"
	"strings"
	"sync"
)

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Heater
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const (
	KELVIN_TO_CELSIUS = -273.15
	MAX_HEAT_TIME     = 5.0
	AMBIENT_TEMP      = 25.
	PID_PARAM_BASE    = 255.
)

type Heater struct {
	Printer          *Printer
	Name             string
	Sensor           *PrinterADCtoTemperature
	Min_temp         float64
	Max_temp         float64
	Pwm_delay        float64
	Min_extrude_temp float64
	Can_extrude      bool
	Max_power        float64
	Smooth_time      float64
	Inv_smooth_time  float64
	Lock             sync.Mutex
	Last_temp        float64
	Smoothed_temp    float64
	Target_temp      float64
	Last_temp_time   float64
	Next_pwm_time    float64
	Last_pwm_value   float64
	Control          interface{}
	Mcu_pwm          interface{}
}

func NewHeater(config *ConfigWrapper, sensor interface{}) *Heater {
	var self = &Heater{}
	self.Printer = config.Get_printer()
	self.Name = strings.Split(config.Get_name(), " ")[len(strings.Split(config.Get_name(), " "))-1]
	var kelvin_to_celsius = KELVIN_TO_CELSIUS
	// Setup sensor
	self.Sensor = sensor.(*PrinterADCtoTemperature)
	self.Min_temp = config.Getfloat("min_temp", 0, kelvin_to_celsius, 0, 0, 0, true)
	self.Max_temp = config.Getfloat("max_temp", 0, 0, 0, self.Min_temp, 0, true)
	self.Sensor.Setup_minmax(self.Min_temp, self.Max_temp)
	self.Sensor.Setup_callback(self.Temperature_callback)
	self.Pwm_delay = self.Sensor.Get_report_time_delta()
	// Setup temperature checks
	self.Min_extrude_temp = config.Getfloat(
		"min_extrude_temp", 170.,
		self.Min_temp, self.Max_temp, 0, 0, true)
	var is_fileoutput bool
	if self.Printer.Get_start_args()["debugoutput"] != nil {
		is_fileoutput = true
	} else {
		is_fileoutput = false
	}
	self.Can_extrude = self.Min_extrude_temp <= 0. || is_fileoutput
	self.Max_power = config.Getfloat("max_power", 1., 0, 1, 0., 0., true)
	self.Smooth_time = config.Getfloat("smooth_time", 1., 0, 0, 0., 0, true)
	self.Inv_smooth_time = 1. / self.Smooth_time
	self.Lock = sync.Mutex{}
	self.Last_temp, self.Smoothed_temp, self.Target_temp = 0., 0., 0.
	self.Last_temp_time = 0.
	// pwm caching
	self.Next_pwm_time = 0.
	self.Last_pwm_value = 0.
	// Setup control algorithm sub-class
	var algos = map[interface{}]interface{}{"watermark": NewControlBangBang, "pid": NewControlPID}
	var algo = config.Getchoice("control", algos, nil, true)
	self.Control = algo.(func(*Heater, *ConfigWrapper) interface{})(self, config)
	// Setup output heater pin
	var heater_pin = config.Get("heater_pin", object.Sentinel{}, true)
	var ppins = self.Printer.Lookup_object("pins", object.Sentinel{})
	self.Mcu_pwm = ppins.(*PrinterPins).Setup_pin("pwm", heater_pin.(string))
	var pwm_cycle_time = config.Getfloat("pwm_cycle_time", 0.100, 0, self.Pwm_delay, 0., 0, true)
	self.Mcu_pwm.(*MCU_pwm).Setup_cycle_time(pwm_cycle_time, false)
	self.Mcu_pwm.(*MCU_pwm).Setup_max_duration(MAX_HEAT_TIME)
	// Load additional modules
	self.Printer.Load_object(config, fmt.Sprintf("verify_heater %s", self.Name), object.Sentinel{})
	self.Printer.Load_object(config, "pid_calibrate", object.Sentinel{})
	var gcode = self.Printer.Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_mux_command("SET_HEATER_TEMPERATURE", "HEATER",
		self.Name, self.Cmd_SET_HEATER_TEMPERATURE,
		cmd_SET_HEATER_TEMPERATURE_help)
	return self
}

func (self *Heater) Set_pwm(read_time float64, value float64) {
	if self.Target_temp <= 0. {
		value = 0.
	}

	if (read_time < self.Next_pwm_time || self.Last_pwm_value == 0.) &&
		math.Abs(value-self.Last_pwm_value) < 0.05 {
		// No significant change in value - can suppress update
		return
	}
	var pwm_time = read_time + self.Pwm_delay
	self.Next_pwm_time = pwm_time + 0.75*MAX_HEAT_TIME
	self.Last_pwm_value = value
	self.Mcu_pwm.(*MCU_pwm).Set_pwm(pwm_time, value)
}

type iTemperature_update interface {
	Temperature_update(read_time float64, temp float64, target_temp float64)
}

func (self *Heater) Temperature_callback(read_time float64, temp float64) {
	self.Lock.Lock()
	defer self.Lock.Unlock()
	var time_diff = read_time - self.Last_temp_time
	self.Last_temp = temp
	self.Last_temp_time = read_time
	self.Control.(iTemperature_update).Temperature_update(read_time, temp, self.Target_temp)
	var temp_diff = temp - self.Smoothed_temp
	var adj_time = math.Min(time_diff*self.Inv_smooth_time, 1.)
	self.Smoothed_temp += temp_diff * adj_time
	self.Can_extrude = self.Smoothed_temp >= self.Min_extrude_temp
}

// External commands
func (self *Heater) Get_pwm_delay() float64 {
	return self.Pwm_delay
}

func (self *Heater) Get_max_power() float64 {
	return self.Max_power
}

func (self *Heater) Get_smooth_time() float64 {
	return self.Smooth_time
}

func (self *Heater) Set_temp(degrees float64) {
	if degrees != 0 && (degrees < self.Min_temp || degrees > self.Max_temp) {
		panic(fmt.Sprintf("Requested temperature (%.1f) out of range (%.1f:%.1f)", degrees, self.Min_temp, self.Max_temp))
	}
	self.Lock.Lock()
	defer self.Lock.Unlock()
	self.Target_temp = degrees
}

func (self *Heater) Get_temp(eventtime float64) (float64, float64) {
	print_time := self.Mcu_pwm.(*MCU_pwm).Get_mcu().Estimated_print_time(eventtime) - 5.
	self.Lock.Lock()
	defer self.Lock.Unlock()
	if self.Last_temp_time < print_time {
		return 0., self.Target_temp
	}
	return self.Smoothed_temp, self.Target_temp
}

type CheckBusyer interface {
	Check_busy(float64, float64, float64) bool
}

func (self *Heater) Check_busy(eventtime float64) interface{} {
	self.Lock.Lock()
	defer self.Lock.Unlock()
	return self.Control.(CheckBusyer).Check_busy(eventtime, self.Smoothed_temp, self.Target_temp)
}

func (self *Heater) Set_control(control interface{}) interface{} {
	self.Lock.Lock()
	defer self.Lock.Unlock()
	var old_control = self.Control
	self.Control = control
	self.Target_temp = 0.
	return old_control
}

func (self *Heater) Alter_target(target_temp float64) {
	if target_temp != 0 {
		target_temp = math.Max(self.Min_temp, math.Min(self.Max_temp, target_temp))
	}
	self.Target_temp = target_temp
}

func (self *Heater) Stats(eventtime float64) (bool, string) {
	self.Lock.Lock()
	defer self.Lock.Unlock()
	var target_temp = self.Target_temp
	var last_temp = self.Last_temp
	var last_pwm_value = self.Last_pwm_value
	var is_active = target_temp != 0 || last_temp > 50.
	return is_active, fmt.Sprintf("%s: target=%.0f temp=%.1f pwm=%.3f",
		self.Name, target_temp, last_temp, last_pwm_value)
}

func (self *Heater) Get_status(eventtime float64) map[string]float64 {
	self.Lock.Lock()
	defer self.Lock.Unlock()
	var target_temp = self.Target_temp
	var smoothed_temp = self.Smoothed_temp
	var last_pwm_value = self.Last_pwm_value
	return map[string]float64{
		"temperature": math.Round(smoothed_temp),
		"target":      target_temp,
		"power":       last_pwm_value,
	}
}

const cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"

func (self *Heater) Cmd_SET_HEATER_TEMPERATURE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	temp := gcmd.Get_float("TARGET", 0., nil, nil, nil, nil)
	pheaters := self.Printer.Lookup_object("heaters", object.Sentinel{})
	pheaters.(*PrinterHeaters).Set_temperature(self, temp, false)
	return nil
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bang-bang control algo
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

type ControlBangBang struct {
	Heater           *Heater
	Heater_max_power float64
	Max_delta        float64
	Heating          bool
}

func NewControlBangBang(heater *Heater, config *ConfigWrapper) interface{} {
	var self = &ControlBangBang{}
	self.Heater = heater
	self.Heater_max_power = heater.Get_max_power()
	self.Max_delta = config.Getfloat("max_delta", 2.0, 0, 0, 0., 0, true)
	self.Heating = false
	return self
}

func (self *ControlBangBang) Temperature_update(read_time float64, temp float64, target_temp float64) {
	if self.Heating && temp >= target_temp+self.Max_delta {
		self.Heating = false
	} else if self.Heating == false && temp <= target_temp-self.Max_delta {
		self.Heating = true
	}
	if self.Heating {
		self.Heater.Set_pwm(read_time, self.Heater_max_power)
	} else {
		self.Heater.Set_pwm(read_time, 0.)
	}
}

func (self *ControlBangBang) Check_busy(eventtime float64, smoothed_temp float64, target_temp float64) bool {
	return smoothed_temp < target_temp-self.Max_delta
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Proportional Integral Derivative (PID) control algo
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const (
	PID_SETTLE_DELTA = 1.
	PID_SETTLE_SLOPE = .1
)

type ControlPID struct {
	Heater           *Heater
	Heater_max_power float64
	Kp               float64
	Ki               float64
	Kd               float64
	Min_deriv_time   float64
	Temp_integ_max   float64
	Prev_temp        float64
	Prev_temp_time   float64
	Prev_temp_deriv  float64
	Prev_temp_integ  float64
}

func NewControlPID(heater *Heater, config *ConfigWrapper) interface{} {
	var self = &ControlPID{}
	self.Heater = heater
	self.Heater_max_power = heater.Get_max_power()
	self.Kp = config.Getfloat("pid_Kp", object.Sentinel{}, 0, 0, 0, 0, true) / PID_PARAM_BASE
	self.Ki = config.Getfloat("pid_Ki", object.Sentinel{}, 0, 0, 0, 0, true) / PID_PARAM_BASE
	self.Kd = config.Getfloat("pid_Kd", object.Sentinel{}, 0, 0, 0, 0, true) / PID_PARAM_BASE
	self.Min_deriv_time = heater.Get_smooth_time()
	self.Temp_integ_max = 0.
	if self.Ki != 0 {
		self.Temp_integ_max = self.Heater_max_power / self.Ki
	}
	self.Prev_temp = AMBIENT_TEMP
	self.Prev_temp_time = 0.
	self.Prev_temp_deriv = 0.
	self.Prev_temp_integ = 0.
	return self
}

func (self *ControlPID) Temperature_update(read_time float64, temp float64, target_temp float64) {
	var time_diff = read_time - self.Prev_temp_time
	// Calculate change of temperature
	var temp_diff = temp - self.Prev_temp
	var temp_deriv float64
	if time_diff >= self.Min_deriv_time {
		temp_deriv = temp_diff / time_diff
	} else {
		temp_deriv = (self.Prev_temp_deriv*(self.Min_deriv_time-time_diff) +
			temp_diff) / self.Min_deriv_time
	}
	// Calculate accumulated temperature "error"
	var temp_err = target_temp - temp
	var temp_integ = self.Prev_temp_integ + temp_err*time_diff
	temp_integ = math.Max(0., math.Min(self.Temp_integ_max, temp_integ))
	// Calculate output
	var co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
	var bounded_co = math.Max(0., math.Min(self.Heater_max_power, co))
	self.Heater.Set_pwm(read_time, bounded_co)
	// Store state for next measurement
	self.Prev_temp = temp
	self.Prev_temp_time = read_time
	self.Prev_temp_deriv = temp_deriv
	if co == bounded_co {
		self.Prev_temp_integ = temp_integ
	}
}

func (self *ControlPID) Check_busy(eventtime float64, smoothed_temp float64, target_temp float64) bool {
	var temp_diff = target_temp - smoothed_temp
	return math.Abs(temp_diff) > PID_SETTLE_DELTA ||
		math.Abs(self.Prev_temp_deriv) > PID_SETTLE_SLOPE
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor and heater lookup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

type PrinterHeaters struct {
	Printer            *Printer
	Sensor_factories   map[string]interface{}
	Heaters            map[string]*Heater
	Gcode_id_to_sensor map[string]interface{}
	Available_heaters  []string
	Available_sensors  []string
	Available_monitors []string
	Has_started        bool
	Have_load_sensors  bool
}

func NewPrinterHeaters(config *ConfigWrapper) *PrinterHeaters {
	var self = &PrinterHeaters{}
	self.Printer = config.Get_printer()
	self.Sensor_factories = map[string]interface{}{}
	self.Heaters = map[string]*Heater{"": nil}
	self.Gcode_id_to_sensor = map[string]interface{}{}
	self.Available_heaters = []string{}
	self.Available_sensors = []string{}
	self.Has_started, self.Have_load_sensors = false, false
	self.Printer.Register_event_handler("project:ready", self.Handle_ready)
	self.Printer.Register_event_handler("gcode:request_restart",
		self.Turn_off_all_heaters)
	// Register commands
	var gcode = self.Printer.Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_command("TURN_OFF_HEATERS", self.Cmd_TURN_OFF_HEATERS,
		true,
		cmd_TURN_OFF_HEATERS_help)
	gcode.(*GCodeDispatch).Register_command("M105", self.Cmd_M105, true, "")
	gcode.(*GCodeDispatch).Register_command("TEMPERATURE_WAIT", self.Cmd_TEMPERATURE_WAIT,
		true,
		cmd_TEMPERATURE_WAIT_help)
	return self
}

func (self *PrinterHeaters) Load_config(config *ConfigWrapper) {
	self.Have_load_sensors = true
	// Load default temperature sensors
	var pconfig = self.Printer.Lookup_object("configfile", object.Sentinel{})
	var dir_name, _ = os.Getwd()
	var filename = path.Join(dir_name, "/temperature_sensors.cfg")
	// try:
	var dconfig = pconfig.(*PrinterConfig).Read_config(filename)
	if dconfig == nil {
		logger.Errorf(fmt.Sprintf("Cannot load config %s", filename))
		return
	}
	for _, c := range dconfig.Get_prefix_sections("") {
		self.Printer.Load_object(dconfig, c.Get_name(), object.Sentinel{})
	}
}

func (self *PrinterHeaters) Add_sensor_factory(sensor_type string, sensor_factory interface{}) {
	self.Sensor_factories[sensor_type] = sensor_factory
}

func (self *PrinterHeaters) Setup_heater(config *ConfigWrapper, gcode_id string) *Heater {
	strArr := strings.Split(config.Get_name(), " ")
	var heater_name = strArr[len(strArr)-1]
	if self.Heaters[heater_name] != nil {
		panic(fmt.Sprintf("Heater %s already registered", heater_name))
	}
	// Setup sensor
	var sensor = self.Setup_sensor(config)

	// Create heater
	var heater = NewHeater(config, sensor)
	self.Heaters[heater_name] = heater
	self.Register_sensor(config, heater, gcode_id)
	self.Available_heaters = append(self.Available_heaters, config.Get_name())
	return heater
}

func (self *PrinterHeaters) Get_all_heaters() []string {
	return self.Available_heaters
}

func (self *PrinterHeaters) Lookup_heater(heater_name string) *Heater {
	if self.Heaters[heater_name] == nil {
		panic(fmt.Sprintf(
			"Unknown heater  %s", heater_name))
	}
	return self.Heaters[heater_name]
}

func (self *PrinterHeaters) Setup_sensor(config *ConfigWrapper) interface{} {
	if self.Have_load_sensors == false {
		self.Load_config(config)
	}
	var sensor_type = config.Get("sensor_type", object.Sentinel{}, true).(string)
	if self.Sensor_factories[sensor_type] == nil {
		panic(fmt.Sprintf(
			"Unknown temperature sensor  %s", sensor_type))
	}
	if sensor_type == "NTC 100K beta 3950" {
		config.Deprecate("sensor_type", "NTC 100K beta 3950")
	}
	return self.Sensor_factories[sensor_type].(func(*ConfigWrapper) interface{})(config)
}

func (self *PrinterHeaters) Register_sensor(config *ConfigWrapper, psensor interface{}, gcode_id string) {
	self.Available_sensors = append(self.Available_sensors, config.Get_name())
	if gcode_id == "" {
		gcode_id = config.Get("gcode_id", value.None, true).(string)
		if gcode_id == "" {
			return
		}
	}
	if self.Gcode_id_to_sensor[gcode_id] != nil {
		panic(fmt.Sprintf(
			"G-Code sensor id %s already registered", gcode_id))
	}
	self.Gcode_id_to_sensor[gcode_id] = psensor
}

func (self *PrinterHeaters) Register_monitor(config *ConfigWrapper) {
	self.Available_monitors = append(self.Available_monitors, config.Get_name())
}

func (self *PrinterHeaters) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"available_heaters":  self.Available_heaters,
		"available_sensors":  self.Available_sensors,
		"available_monitors": self.Available_monitors}
}

func (self *PrinterHeaters) Turn_off_all_heaters(argv []interface{}) error {
	for _, v := range self.Heaters {
		if v != nil {
			v.Set_temp(0.)
		}
	}
	return nil
}

const cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"

func (self *PrinterHeaters) Cmd_TURN_OFF_HEATERS(gcmd interface{}) error {
	self.Turn_off_all_heaters(nil)
	return nil
}

// G-Code M105 temperature reporting
func (self *PrinterHeaters) Handle_ready([]interface{}) error {
	self.Has_started = true
	return nil
}

func (self *PrinterHeaters) Get_temp(eventtime float64, heater_type string) string {
	var out = make([]string, 0, len(self.Gcode_id_to_sensor))

	var gcodeIds = str.MapStringKeys(self.Gcode_id_to_sensor)
	sort.Strings(gcodeIds)
	if self.Has_started {
		for _, gcode_id := range gcodeIds {
			if self.Gcode_id_to_sensor[gcode_id] == nil {
				logger.Errorf(fmt.Sprintf("G-Code sensor id %s must not be nil", gcode_id))
				continue
			}
			sensor, ok := self.Gcode_id_to_sensor[gcode_id].(IGetTemperature)
			if !ok {
				logger.Errorf(fmt.Sprintf("G-Code sensor id %s, %+v must implement project.IGetTemperature", gcode_id, self.Gcode_id_to_sensor[gcode_id]))
				continue
			}
			cur, target := sensor.Get_temp(eventtime)
			out = append(out, fmt.Sprintf("%s:%.1f /%.1f", gcode_id, cur, target))
		}
	}
	if len(out) == 0 {
		return "T:0"
	}
	return strings.Join(out, " ")
}

func (self *PrinterHeaters) Cmd_M105(gcmd interface{}) error {
	// Get Extruder Temperature
	var reactor = self.Printer.Get_reactor()
	var msg = self.Get_temp(reactor.Monotonic(), "extruder")

	var did_ack = gcmd.(*GCodeCommand).Ack(msg)
	if did_ack {
		gcmd.(*GCodeCommand).Respond_raw(msg)
	}
	return nil
}

func (self *PrinterHeaters) Wait_for_temperature(heater *Heater) error {
	// Helper to wait on heater.check_busy() and report M105 temperatures
	if self.Printer.Get_start_args()["debugoutput"] != nil {
		return nil
	}
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	var gcode = self.Printer.Lookup_object("gcode", object.Sentinel{})
	var reactor = self.Printer.Get_reactor()
	var eventtime = reactor.Monotonic()
	for !self.Printer.Is_shutdown() && cast.ToBool(heater.Check_busy(eventtime)) {
		_ = toolhead.(*Toolhead).Get_last_move_time()
		gcode.(*GCodeDispatch).Respond_raw(self.Get_temp(eventtime, heater.Name))
		eventtime = reactor.Pause(eventtime + 1.)
	}
	return nil
}

func (self *PrinterHeaters) Set_temperature(heater *Heater, temp float64, wait bool) error {
	var toolhead = self.Printer.Lookup_object("toolhead", object.Sentinel{})
	toolhead.(*Toolhead).Register_lookahead_callback(func(pt float64) {})
	heater.Set_temp(temp)
	if wait && temp > 0 {
		return self.Wait_for_temperature(heater)
	}
	return nil
}

const cmd_TEMPERATURE_WAIT_help = "Wait for a temperature on a sensor"

func (self *PrinterHeaters) Cmd_TEMPERATURE_WAIT(gcmd *GCodeCommand) error {
	var sensor_name = gcmd.Get("SENSOR", object.Sentinel{}, "", nil, nil, nil, nil)
	if collections.Contains(self.Available_sensors, sensor_name) == false {
		panic(fmt.Sprintf("Unknown sensor %s", sensor_name))
	}

	var min_temp = gcmd.Get_float("MINIMUM", math.Inf(-1), nil, nil, nil, nil)
	var max_temp = gcmd.Get_float("MAXIMUM", math.Inf(1), nil, &min_temp, nil, nil)
	if min_temp == math.Inf(-1) && max_temp == math.Inf(1) {
		panic("Error on TEMPERATURE_WAIT: missing MINIMUM or MAXIMUM.")
	}
	if self.Printer.Get_start_args()["debugoutput"] != nil {
		return nil
	}
	var sensor *Heater
	if self.Heaters[sensor_name] != nil {
		sensor = self.Heaters[sensor_name]
	} else {
		sensor = self.Printer.Lookup_object(sensor_name, object.Sentinel{}).(*Heater)
	}
	//var toolhead = self.Printer.Lookup_object("toolhead")
	var reactor = self.Printer.Get_reactor()
	var eventtime = reactor.Monotonic()
	var temp float64
	for {
		temp, _ = sensor.Get_temp(eventtime)
		if temp >= min_temp && temp <= max_temp {
			return nil
		}
		//var print_time = toolhead.Get_last_move_time()
		gcmd.Respond_raw(self.Get_temp(eventtime, sensor_name))
		eventtime = reactor.Pause(eventtime + 1.)

		if self.Printer.Is_shutdown() {
			break
		}
	}
	return nil
}

func Load_config_heaters(config *ConfigWrapper) interface{} {
	return NewPrinterHeaters(config)
}
