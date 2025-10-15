package project

import (
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"math"
)

type PrintStats struct {
	gcode_move             *GCodeMove
	reactor                IReactor
	gcode                  *GCodeDispatch
	filament_used          float64
	last_epos              float64
	last_exclude_e_offset  float64
	filename               string
	print_start_time       float64
	last_pause_time        float64
	prev_pause_duration    float64
	state                  string
	error_message          string
	total_duration         float64
	init_duration          float64
	info_total_layer       int
	info_current_layer     int
	printing               bool
	Restore_total_duration float64
}

func NewPrintStats(config *ConfigWrapper) *PrintStats {
	self := new(PrintStats)
	printer := config.Get_printer()
	self.gcode_move = MustLoadGcodeMove(config)
	self.reactor = printer.Get_reactor()
	self.Reset()
	//Register commands
	self.gcode = MustLookupGcode(printer)
	self.gcode.Register_command(
		"SET_PRINT_STATS_INFO", self.cmd_SET_PRINT_STATS_INFO, false,
		cmd_SET_PRINT_STATS_INFO_help)
	return self
}

func (self *PrintStats) _update_filament_usage(eventtime float64) {
	gc_status := self.gcode_move.Get_status(eventtime)
	cur_epos := gc_status["position"].([]float64)[3]
	self.filament_used += (cur_epos - self.last_epos) /
		gc_status["extrude_factor"].(float64)
	self.last_epos = cur_epos
}

func (self *PrintStats) Set_current_file(filename string) {
	self.Reset()
	self.filename = filename
}

func (self *PrintStats) Note_start() {
	curtime := self.reactor.Monotonic()
	if self.print_start_time == 0 {
		self.print_start_time = curtime
	} else if self.last_pause_time != 0 {
		// Update pause time duration
		pause_duration := curtime - self.last_pause_time
		self.prev_pause_duration += pause_duration
		self.last_pause_time = 0
	}

	// Reset last e-position
	gc_status := self.gcode_move.Get_status(curtime)
	self.last_epos = gc_status["position"].([]float64)[3]
	self.state = "printing"
	self.error_message = ""
}

func (self *PrintStats) Note_pause() {
	if self.last_pause_time == 0 {
		curtime := self.reactor.Monotonic()
		self.last_pause_time = curtime
		// update filament usage
		self._update_filament_usage(curtime)
	}

	if self.state != "error" {
		self.state = "paused"
	}
}

func (self *PrintStats) Note_complete() {
	self._note_finish("complete", "")
}

func (self *PrintStats) Note_error(message string) {
	self._note_finish("error", message)
}

func (self *PrintStats) Note_cancel() {
	self._note_finish("cancelled", "")
}

func (self *PrintStats) _note_finish(state string, error_message string) {
	if self.print_start_time == 0 {
		return
	}
	self.state = state
	self.error_message = error_message
	eventtime := self.reactor.Monotonic()
	self.total_duration = eventtime - self.print_start_time

	if self.filament_used < 0.0000001 {
		// No positive extusion detected during print
		self.init_duration = self.total_duration - self.prev_pause_duration
	}

	self.print_start_time = 0
}

const cmd_SET_PRINT_STATS_INFO_help = "Pass slicer info like layer act and " + "total to klipper"

func (self *PrintStats) cmd_SET_PRINT_STATS_INFO(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	total_layer := gcmd.Get_int("TOTAL_LAYER", self.info_total_layer, cast.IntP(0), nil)
	current_layer := gcmd.Get_int("CURRENT_LAYER", self.info_current_layer, cast.IntP(0), nil)

	if total_layer == 0 {
		self.info_total_layer = 0
		self.info_current_layer = 0
	} else if total_layer != self.info_total_layer {
		self.info_total_layer = total_layer
		self.info_current_layer = 0
	}

	if self.info_total_layer != 0 &&
		current_layer != 0 &&
		current_layer != self.info_current_layer {
		self.info_current_layer = maths.Min(current_layer, self.info_total_layer)
	}

	return nil
}

func (self *PrintStats) Reset() {
	self.filename, self.error_message = "", ""
	self.state = "standby"
	self.prev_pause_duration, self.last_epos = 0., 0.
	self.filament_used, self.total_duration = 0., 0.
	self.print_start_time, self.last_pause_time = 0., 0.
	self.init_duration = 0.
	self.info_total_layer = 0.0
	self.info_current_layer = 0.0
}

func (self *PrintStats) Get_status(eventtime float64) map[string]interface{} {
	time_paused := self.prev_pause_duration
	if self.print_start_time != 0 {
		if self.last_pause_time != 0 {
			// Calculate the total time spent paused during the print
			time_paused += eventtime - self.last_pause_time
		} else {
			// Accumulate filament if not paused
			self._update_filament_usage(eventtime)
		}
		self.total_duration = eventtime - self.print_start_time
		if self.filament_used < 0.0000001 {
			// Track duration prior to extrusion
			self.init_duration = self.total_duration - time_paused
		}
	}

	print_duration := self.total_duration - self.init_duration - time_paused

	ret := map[string]interface{}{
		"filename":       self.filename,
		"total_duration": math.Round(self.total_duration),
		"print_duration": math.Round(print_duration),
		"filament_used":  math.Round(self.filament_used),
		"state":          self.state,
		"message":        self.error_message,
		"info": map[string]int{
			"total_layer":   self.info_total_layer,
			"current_layer": self.info_current_layer,
		},
	}
	return ret
}

func Load_config_PrintStats(config *ConfigWrapper) interface{} {
	return NewPrintStats(config)
}
