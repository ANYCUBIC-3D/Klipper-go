package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/project/chelper"
)

type cs1237 struct {
	dout_pin             interface{}
	sclk_pin             interface{}
	level_pin            interface{}
	register             int
	sensitivity          int
	mcu                  *MCU
	oid                  int
	trdispatch           interface{}
	trsyncs              []*MCU_trsync
	reset_cs_cmd         *CommandWrapper
	enable_cs_cmd        *CommandWrapper
	cs_report_cmd        *CommandWrapper
	query_cs_diff        *CommandWrapper
	cs_calibration_phase *CommandWrapper
	cs_calibration_val   *CommandWrapper
	checkself_cs1237     *CommandWrapper
	printer              *Printer
	adc_value            int64
	raw_value            int64
	sensor_state         int64
	checkself_flag       int64
	report               bool
	query_comple         *ReactorCompletion
	enable_count         int
}

func NewCs1237(config *ConfigWrapper) *cs1237 {
	c := &cs1237{}
	c.register = config.Getint("register", object.Sentinel{}, 0, 0, true)
	c.sensitivity = config.Getint("sensitivity", object.Sentinel{}, 0, 0, true)
	ppins := MustLookupPins(config.Get_printer())
	dout_pin := cast.ToString(config.Get("dout_pin", object.Sentinel{}, true))
	dout_pin_params := ppins.Lookup_pin(dout_pin, true, true, nil)
	c.dout_pin = dout_pin_params["pin"]

	sclk_pin := cast.ToString(config.Get("sclk_pin", object.Sentinel{}, true))
	sclk_pin_params := ppins.Lookup_pin(sclk_pin, true, true, nil)
	c.sclk_pin = sclk_pin_params["pin"]

	level_pin := cast.ToString(config.Get("level_pin", object.Sentinel{}, true))
	level_pin_params := ppins.Lookup_pin(level_pin, true, true, nil)
	c.level_pin = level_pin_params["pin"]

	c.mcu = sclk_pin_params["chip"].(*MCU)
	c.trdispatch = chelper.Trdispatch_alloc()
	c.trsyncs = []*MCU_trsync{NewMCU_trsync(c.mcu, c.trdispatch)}
	c.printer = config.Get_printer()
	c.mcu.Register_config_callback(c.Build_config)
	c.printer.Register_event_handler("homing:multi_probe_begin", c.enable_cs1237)
	c.printer.Register_event_handler("homing:multi_probe_end", c.disable_cs1237)

	return c
}

func (c *cs1237) Build_config() {
	c.oid = c.mcu.Create_oid()
	c.mcu.Add_config_cmd(fmt.Sprintf("config_cs1237 oid=%d level_pin=%s dout_pin=%s sclk_pin=%s register=%d sensitivity=%d",
		c.oid, c.level_pin, c.dout_pin, c.sclk_pin, c.register, c.sensitivity), false, false)
	c.reset_cs_cmd, _ = c.mcu.Lookup_command("reset_cs1237 oid=%c count=%c", c.trsyncs[0].Get_command_queue())
	c.cs_report_cmd, _ = c.mcu.Lookup_command("start_cs1237_report oid=%c enable=%c ticks=%i print_state=%c sensitivity=%i", c.trsyncs[0].Get_command_queue())
	c.enable_cs_cmd, _ = c.mcu.Lookup_command("enable_cs1237 oid=%c state=%c", c.trsyncs[0].Get_command_queue())

	c.mcu.Register_response(c.cs1237_query_handle, "cs1237_state", c.oid)

	c.query_cs_diff, _ = c.mcu.Lookup_command("query_cs1237_diff oid=%c", c.trsyncs[0].Get_command_queue())
	c.mcu.Register_response(c.cs1237_diff_handle, "cs1237_diff", c.oid)

	c.cs_calibration_phase, _ = c.mcu.Lookup_command("cs1237_calibration_phase oid=%c cali_state=%c speed_state=%c", c.trsyncs[0].Get_command_queue())
	c.cs_calibration_val, _ = c.mcu.Lookup_command("cs1237_calibration_DataProcess oid=%c", c.trsyncs[0].Get_command_queue())
	c.mcu.Register_response(c.cs1237_calibration_DataProcess_handle, "cs1237_calibration_Val", c.oid)
}

func (c *cs1237) cs1237_check_start(check_period float64, check_type int64,
	sensitivity int64, delay_send_time float64) {
	tick := c.mcu.Seconds_to_clock(check_period)
	delay_t := c.mcu.Seconds_to_clock(delay_send_time)
	c.cs_report_cmd.Send([]int64{int64(c.oid), int64(1), int64(tick),
		int64(check_type), int64(sensitivity)}, delay_t, 0)
}

func (c *cs1237) cs1237_check_stop(check_type int64) {
	c.cs_report_cmd.Send([]int64{int64(c.oid), int64(0), int64(0),
		int64(check_type), int64(0)}, 0, 0)
}

func (c *cs1237) cs1237_query_handle(params map[string]interface{}) error {
	c.adc_value = maths.Int64_conversion(params["adc"].(int64))
	c.raw_value = maths.Int64_conversion(params["raw"].(int64))
	c.sensor_state = params["state"].(int64)
	logger.Debug("params ", params, c.raw_value, c.adc_value)
	return nil
}

func (c *cs1237) cs1237_diff() (int64, int64, error) {
	defer func() {
		c.query_comple = nil
	}()
	if c.query_comple == nil {
		c.query_comple = c.printer.Get_reactor().Completion()
	}
	c.query_cs_diff.Send([]int64{int64(c.oid)}, 0, 0)
	timeout := c.printer.Get_reactor().Monotonic() + 2
	params := c.query_comple.Wait(timeout, nil)
	if params != nil {
		_params := params.(map[string]interface{})
		return _params["diff"].(int64), _params["raw"].(int64), nil
	} else {
		logger.Debug("cs1237_query response timeout")
		return -1, -1, fmt.Errorf("cs1237_query response timeout")
	}

	return 0, 0, nil
}

func (c *cs1237) cs1237_calibration(calibration_state int64, speed_state int64) {
	c.cs_calibration_phase.Send([]int64{int64(c.oid), calibration_state, speed_state}, 0, 0)
}

func (c *cs1237) cs1237_calibration_DataProcess_handle(params map[string]interface{}) error {
	if params["BlockPreVal"] == nil {
		params["BlockPreVal"] = 0
	}
	if params["TargetVal"] == nil {
		params["TargetVal"] = 0
	}
	if params["RealVal"] == nil {
		params["RealVal"] = 0
	}

	params["BlockPreVal"] = maths.Int64_conversion(params["BlockPreVal"].(int64))
	params["TargetVal"] = maths.Int64_conversion(params["TargetVal"].(int64))
	params["RealVal"] = maths.Int64_conversion(params["RealVal"].(int64))
	tc := c.query_comple
	if (tc) != nil {
		c.printer.Get_reactor().Async_complete(tc, params)
	}
	return nil
}

func (c *cs1237) cs1237_calibration_Val() (int64, int64, int64) {
	defer func() {
		c.query_comple = nil
	}()
	if c.query_comple == nil {
		c.query_comple = c.printer.Get_reactor().Completion()
	}
	c.cs_calibration_val.Send([]int64{int64(c.oid)}, 0, 0)
	timeout := c.printer.Get_reactor().Monotonic() + 2
	params := c.query_comple.Wait(timeout, nil)
	if params != nil {
		_params := params.(map[string]interface{})
		return _params["BlockPreVal"].(int64), _params["TargetVal"].(int64), _params["RealVal"].(int64)
	} else {
		logger.Error("cs1237_calibration_Val response timeout")
		return -1, -1, -1
	}

	return 0, 0, 0
}

func (c *cs1237) cs1237_diff_handle(params map[string]interface{}) error {
	params["raw"] = maths.Int64_conversion(params["raw"].(int64))
	tc := c.query_comple
	if (tc) != nil {
		c.printer.Get_reactor().Async_complete(tc, params)
	}
	return nil
}

func (c *cs1237) Get_status(eventtime float64) map[string]interface{} {
	resp := make(map[string]interface{})
	if !c.report {
		return resp
	}
	resp["adc"] = c.adc_value
	resp["raw"] = c.raw_value
	resp["state"] = c.sensor_state

	return resp
}

func (c *cs1237) Reset_cs(num int) {
	count := 3
	if num > 0 {
		count = num
	}
	c.reset_cs_cmd.Send([]int64{int64(c.oid), int64(count)}, 0, 0)
	var toolhead = c.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	toolhead.Dwell(0.1)
}

func (c *cs1237) enable_cs1237(argv []interface{}) error {
	gcode := MustLookupGcode(c.printer)
	if c.enable_cs_cmd != nil {
		if c.enable_count == 0 {
			c.enable_cs_cmd.Send([]int64{int64(c.oid), int64(1)}, 0, 0)
		}
		c.enable_count++
	}
	gcode.Run_script_from_command("G4 P500")
	return nil
}

func (c *cs1237) disable_cs1237(argv []interface{}) error {
	if c.enable_cs_cmd != nil {
		c.enable_count--
		if c.enable_count == 0 {
			c.enable_cs_cmd.Send([]int64{int64(c.oid), int64(0)}, 0, 0)
		}
	}
	return nil
}

func (c *cs1237) Stats(eventtime float64) (bool, string) {
	return true, fmt.Sprintf("cs1237:adc_value=%d raw_value=%d sensor_state=%d", c.adc_value, c.raw_value, c.sensor_state)
}

func Load_config_cs1237(config *ConfigWrapper) interface{} {
	return NewCs1237(config)
}
