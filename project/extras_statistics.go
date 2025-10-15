package project

import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/sys"
	"os"
	"reflect"
	"strconv"
	"strings"
)

type PrinterSysStats struct {
	last_process_time  float64
	total_process_time float64
	last_load_avg      float64
	last_mem_avail     int
	mem_file           *os.File
}

func NewPrinterSysStats(config *ConfigWrapper) *PrinterSysStats {
	self := new(PrinterSysStats)
	self.last_process_time = 0.
	self.total_process_time = 0.
	self.last_load_avg = 0.
	self.last_mem_avail = 0
	self.mem_file, _ = os.Open("/proc/meminfo")

	printer := config.Get_printer()
	printer.Register_event_handler("project:disconnect", self._disconnect)

	return self
}

func (self *PrinterSysStats) _disconnect(_ []interface{}) error {
	if self.mem_file != nil {
		self.mem_file.Close()
		self.mem_file = nil
	}
	return nil
}

func (self *PrinterSysStats) Stats(eventtime float64) (bool, string) {
	// Get core usage stats
	pidStat, _ := sys.PidStat()
	ptime := pidStat.Stime + pidStat.Utime
	pdiff := ptime - self.last_process_time
	self.last_process_time = ptime
	if pdiff > 0.0 {
		self.total_process_time += pdiff
	}

	// self.last_load_avg = os.getloadavg()[0]
	loadavg, _ := sys.Loadavg()
	self.last_load_avg = loadavg.Load1
	msg := fmt.Sprintf("sysload=%.2f cputime=%.3f", self.last_load_avg,
		self.total_process_time)

	// Get available system memory
	if self.mem_file != nil {
		data, err := os.ReadFile(self.mem_file.Name())
		if err == nil {
			for _, line := range strings.Split(string(data), "\n") {
				if strings.HasPrefix(line, "MemAvailable:") {
					self.last_mem_avail, _ = strconv.Atoi(strings.Fields(line)[1])
					msg = fmt.Sprintf("%s memavail=%d", msg, self.last_mem_avail)
					break
				}
			}
		}
	}
	return true, msg
}

func (self *PrinterSysStats) Get_status(eventtime float64) map[string]float64 {
	return map[string]float64{
		"sysload":  self.last_load_avg,
		"cputime":  self.total_process_time,
		"memavail": float64(self.last_mem_avail),
	}
}

type PrinterStats struct {
	printer     *Printer
	stats_cb    []reflect.Value
	stats_timer *ReactorTimer
}

func NewPrinterStats(config *ConfigWrapper) *PrinterStats {
	self := new(PrinterStats)

	self.printer = config.Get_printer()
	reactor := self.printer.Get_reactor()
	self.stats_timer = reactor.Register_timer(self.Generate_stats, constants.NEVER)
	self.stats_cb = make([]reflect.Value, 0)
	self.printer.Register_event_handler("project:ready", self.Handle_ready)
	return self
}

func (self *PrinterStats) Handle_ready(_ []interface{}) error {
	for _, obj := range self.printer.Lookup_objects("") {
		if obj == nil {
			continue
		}
		reflectValue := reflect.ValueOf(obj)
		method := reflectValue.MethodByName("Stats")
		if method.Kind() == reflect.Func {
			self.stats_cb = append(self.stats_cb, method)
		} else {
			if reflectValue.Kind() == reflect.Ptr {
				field := reflectValue.Elem().FieldByName("Stats") // method value
				if field.Kind() == reflect.Func {
					self.stats_cb = append(self.stats_cb, field)
				}
			}
		}
	}
	reactor := self.printer.Get_reactor()
	reactor.Update_timer(self.stats_timer, constants.NOW)

	return nil
}
func (self *PrinterStats) Generate_stats(eventtime float64) float64 {
	defer sys.CatchPanic()
	stats := make([]string, 0)
	for _, cb := range self.stats_cb {
		values := cb.Call([]reflect.Value{reflect.ValueOf(eventtime)})
		if values[0].Interface().(bool) {
			stats = append(stats, values[1].Interface().(string))
		}
	}

	if len(stats) != 0 && self.printer.Is_shutdown() {
		logger.Infof(fmt.Sprintf("Stats %.1f: %s", eventtime, strings.Join(stats, " ")) + "\n")
		return constants.NEVER
	}
	return eventtime + 1
}

func Load_config_PrinterStats(config *ConfigWrapper) interface{} {
	config.Get_printer().Add_object("system_stats", NewPrinterSysStats(config))
	return NewPrinterStats(config)
}
