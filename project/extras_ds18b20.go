package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"strings"
	"time"
)

const (
	DS18_REPORT_TIME = 3.0
	//# Temperature can be sampled at any time but conversion time is ~750ms, so
	//# setting the time too low will not make the reports come faster.
	DS18_MIN_REPORT_TIME        = 1.0
	DS18_MAX_CONSECUTIVE_ERRORS = 4
)

type DS18B20 struct {
	printer       *Printer
	name          string
	sensor_id     string
	temp          float64
	min_temp      float64
	max_temp      float64
	_report_clock int64
	report_time   float64
	_mcu          *MCU
	oid           int
	callback      func(float64, float64)
}

func NewDS18B20(config *ConfigWrapper) *DS18B20 {
	self := new(DS18B20)
	self.printer = config.Get_printer()
	name := strings.Split(config.Get_name(), " ")
	self.name = name[len(name)-1]
	self.sensor_id = config.Get("serial_no", object.Sentinel{}, true).(string)
	self.temp, self.min_temp, self.max_temp = 0.0, 0.0, 0.0
	self._report_clock = 0
	self.report_time = config.Getfloat("ds18_report_time",
		DS18_REPORT_TIME, DS18_MIN_REPORT_TIME, 0., 0., 0., true)
	self._mcu = Get_printer_mcu(self.printer, config.Get("sensor_mcu", object.Sentinel{}, true).(string))
	self.oid = self._mcu.Create_oid()
	self._mcu.Register_response(self._handle_ds18b20_response,
		"ds18b20_result", self.oid)
	self._mcu.Register_config_callback(self._build_config)

	return self
}

func (self *DS18B20) _build_config() {
	sid := fmt.Sprintf("%x", self.sensor_id)

	self._mcu.Add_config_cmd(
		fmt.Sprintf("config_ds18b20 oid=%d serial=%s max_error_count=%d",
			self.oid, sid, DS18_MAX_CONSECUTIVE_ERRORS),
		false, false)

	clock := self._mcu.Get_query_slot(self.oid)
	self._report_clock = self._mcu.Seconds_to_clock(self.report_time)
	self._mcu.Add_config_cmd(
		fmt.Sprintf("query_ds18b20 oid=%d clock=%d rest_ticks=%d min_value=%d max_value=%d",
			self.oid, clock, self._report_clock,
			int(self.min_temp*1000), int(self.max_temp*1000)),
		true, false)
}

func (self *DS18B20) _handle_ds18b20_response(params map[string]interface{}) error {
	temp := params["value"].(float64) / 1000.0

	if fault, ok := params["fault"]; ok && fault != 0 {
		logger.Infof("ds18b20 reports fault %v (temp=%.1f)", fault, temp)
		return nil
	}

	nextClock := self._mcu.Clock32_to_clock64(params["next_clock"].(int64))
	last_read_clock := nextClock - self._report_clock
	last_read_time := self._mcu.Clock_to_print_time(last_read_clock)

	if self.callback != nil {
		self.callback(last_read_time, temp)
	}
	return nil
}

func (self *DS18B20) setup_minmax(min_temp, max_temp float64) {
	self.min_temp = min_temp
	self.max_temp = max_temp
}

func (self *DS18B20) Fault(msg string) {
	self.printer.invoke_async_shutdown(msg)
}

func (self *DS18B20) get_report_time_delta() float64 {
	return self.report_time
}

func (self *DS18B20) setup_callback(cb func(float64, float64)) {
	self.callback = cb
}

func (self *DS18B20) Get_status(eventTime time.Time) map[string]interface{} {
	return map[string]interface{}{
		"temperature": maths.Round(self.temp, 2),
	}
}

func Load_config_ds18b20(config *ConfigWrapper) interface{} {
	mcu := config.Get("sensor_mcu", "", true)
	if mcu != "" {
		pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{}).(*PrinterHeaters)
		pheaters.Add_sensor_factory("DS18B20", NewDS18B20(config))
	}

	return nil
}
