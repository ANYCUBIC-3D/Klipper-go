package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/value"
)

/*
# Pause/Resume functionality with position capture/restore
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/

type PauseResume struct {
	Printer            *Printer
	reactor            IReactor
	Gcode              *GCodeDispatch
	Recover_velocity   float64
	V_sd               *VirtualSD
	Gcode_move         *GCodeMove
	Is_paused          bool
	Sd_paused          bool
	Pause_command_sent bool
	config             *ConfigWrapper
}

func NewPauseResume(config *ConfigWrapper) *PauseResume {
	self := &PauseResume{}
	self.config = config
	self.Printer = config.Get_printer()
	self.reactor = self.Printer.Get_reactor()
	self.Gcode = self.Printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	self.Recover_velocity = config.Getfloat("recover_velocity", 50., 0., 0., 0., 0., true)
	self.V_sd = nil
	self.Is_paused = false
	self.Sd_paused = false
	self.Pause_command_sent = false

	self.Printer.Register_event_handler("project:connect", self.Handle_connect)

	self.Gcode.Register_command("PAUSE", self.Cmd_PAUSE, false, Cmd_PAUSE_help)
	self.Gcode.Register_command("RESUME", self.Cmd_RESUME, false, Cmd_RESUME_help)
	self.Gcode.Register_command("CLEAR_PAUSE", self.Cmd_CLEAR_PAUSE, false, Cmd_CLEAR_PAUSE_help)
	self.Gcode.Register_command("CANCEL_PRINT", self.Cmd_CANCEL_PRINT, false, Cmd_CANCEL_PRINT_help)

	webhooks := self.Printer.Load_object(config, "webhooks", object.Sentinel{}).(*WebHooks)
	webhooks.Register_endpoint("pause_resume/cancel", self.Handle_cancel_request)
	webhooks.Register_endpoint("pause_resume/pause", self.Handle_pause_request)
	webhooks.Register_endpoint("pause_resume/resume", self.Handle_resume_request)
	return self
}
func (self *PauseResume) Handle_connect(args []interface{}) error {
	self.V_sd = self.Printer.Lookup_object("virtual_sdcard", value.None).(*VirtualSD)
	return nil
}
func (self *PauseResume) Handle_cancel_request(web_request *WebRequest) (interface{}, error) {
	self.Gcode.Run_script("CANCEL_PRINT")
	return nil, nil
}
func (self *PauseResume) Handle_pause_request(web_request *WebRequest) (interface{}, error) {
	self.Gcode.Run_script("PAUSE")
	return nil, nil
}
func (self *PauseResume) Handle_resume_request(web_request *WebRequest) (interface{}, error) {
	self.Gcode.Run_script("RESUME")
	return nil, nil
}
func (self *PauseResume) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"is_paused": self.Is_paused,
	}
}
func (self *PauseResume) Is_sd_active() bool {
	return self.V_sd != nil && self.V_sd.Is_active()
}
func (self *PauseResume) Send_pause_command() {
	/*
		# This sends the appropriate pause command from an event.  Note
		# the difference between pause_command_sent and is_paused, the
		# module isn't officially paused until the PAUSE gcode executes.
	*/
	if !self.Pause_command_sent {
		if self.Is_sd_active() {
			// Printing from virtual sd, run pause command
			self.Sd_paused = true
			self.V_sd.Do_pause()
		} else {
			self.Sd_paused = false
			self.Gcode.Respond_info("action:paused", true)
		}
		self.Pause_command_sent = true
	}
}

const Cmd_PAUSE_help = "Pauses the current print"

func (self *PauseResume) Cmd_PAUSE(gcmd interface{}) error {
	if self.Is_paused {
		gcmd.(*GCodeCommand).Respond_info("Print already paused", true)
		return nil
	}
	self.Send_pause_command()
	self.Gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")
	self.Is_paused = true

	return nil
}
func (self *PauseResume) Send_resume_command() {
	if self.Sd_paused {
		// Printing from virtual sd, run pause command
		self.V_sd.Do_resume()
		self.Sd_paused = false
	} else {
		self.Gcode.Respond_info("action:resumed", true)
	}
	self.Pause_command_sent = false
}

const Cmd_RESUME_help = "Resumes the print from a pause"

func (self *PauseResume) Cmd_RESUME(gcmd interface{}) error {
	if !self.Is_paused {
		gcmd.(*GCodeCommand).Respond_info("Print is not paused, resume aborted", true)
		return nil
	}
	zero := 0.0
	velocity := gcmd.(*GCodeCommand).Get_float("VELOCITY", self.Recover_velocity, &zero, &zero, &zero, &zero)
	self.Gcode.Run_script_from_command(fmt.Sprintf("RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f", velocity))
	self.Send_resume_command()
	self.Is_paused = false
	return nil
}

const Cmd_CLEAR_PAUSE_help = "Clears the current paused state without resuming the print"

func (self *PauseResume) Cmd_CLEAR_PAUSE(gcmd interface{}) error {
	self.Is_paused = false
	self.Pause_command_sent = false
	return nil
}

const Cmd_CANCEL_PRINT_help = "Cancel the current print"

func (self *PauseResume) Cmd_CANCEL_PRINT(gcmd interface{}) error {
	if self.Is_sd_active() || self.Sd_paused {
		self.V_sd.Do_cancel()
	} else {
		gcmd.(*GCodeCommand).Respond_info("action:cancel", true)
	}
	self.Cmd_CLEAR_PAUSE(gcmd)
	return nil
}

func Load_config_pause_resume(config *ConfigWrapper) interface{} {
	return NewPauseResume(config)
}
