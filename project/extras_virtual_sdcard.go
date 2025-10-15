/*
# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018  Kevin O"Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"errors"
	"fmt"
	"io"
	"io/fs"
	"io/ioutil"
	"k3c/common/constants"
	kerror "k3c/common/errors"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/reflects"
	"k3c/project/util"
	"math"
	"os"
	"path/filepath"
	"runtime/debug"
	"sort"
	"strings"
)

var VALID_GCODE_EXTS = []string{"gcode", "g", "gco"}

type VirtualSD struct {
	Printer            *Printer
	Sdcard_dirname     string
	Current_file       *os.File
	File_size          int64
	Print_stats        *PrintStats
	Reactor            IReactor
	Must_pause_work    bool
	Cmd_from_sd        bool
	Next_file_position int
	Work_timer         *ReactorTimer
	On_error_gcode     interface{}
	Gcode              *GCodeDispatch
	layerCount         int
	currLayer          int
	totalTime          int
	config             *ConfigWrapper
	device_type        string
	sd_type            string
	state_path         string
	IsPrint            bool
	bedTempCfg         float64
	File_position      int
}

func NewVirtualSD(config *ConfigWrapper) *VirtualSD {
	self := &VirtualSD{}
	self.config = config
	self.Printer = config.Get_printer()
	self.Printer.Register_event_handler("project:shutdown",
		self.Handle_shutdown)
	sd := config.Get("path", "", true).(string)
	self.Sdcard_dirname = util.Normpath(util.ExpandUser(sd))
	self.Current_file = nil
	self.File_position, self.File_size = 0, 0
	// Print Stat Tracking
	self.Print_stats = MustLoadPrintStats(config)
	// Work timer
	self.Reactor = self.Printer.Get_reactor()
	self.Must_pause_work, self.Cmd_from_sd = false, false
	self.Next_file_position = 0
	self.Work_timer = nil
	// Error handling
	gcode_macro := self.Printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	self.On_error_gcode = gcode_macro.Load_template(
		config, "on_error_gcode", "")
	// Register commands
	gcode_object := self.Printer.Lookup_object("gcode", object.Sentinel{})
	self.Gcode = gcode_object.(*GCodeDispatch)
	for _, cmd := range []string{"M20", "M21", "M23", "M24", "M25", "M26", "M27"} {
		var _func = reflects.GetMethod(&self, "Cmd_"+cmd)
		self.Gcode.Register_command(cmd, _func, false, "")
	}
	for _, cmd := range []string{"M28", "M29", "M30"} {
		self.Gcode.Register_command(cmd, self.Cmd_error(nil), false, "")
	}

	self.Gcode.Register_command("SDCARD_RESET_FILE", self.Cmd_SDCARD_RESET_FILE,
		false, cmd_SDCARD_RESET_FILE_help)
	self.Gcode.Register_command("SDCARD_PRINT_FILE", self.Cmd_SDCARD_PRINT_FILE,
		false, cmd_SDCARD_PRINT_FILE_help)
	return self
}

func (self *VirtualSD) Handle_shutdown([]interface{}) error {
	if self.Work_timer != nil {
		self.Must_pause_work = true
	}
	readpos := math.Max(float64(self.File_position)-1024, 0)
	readcount := float64(self.File_position) - readpos
	self.Current_file.Seek(int64(readpos)+128, 0)
	data := make([]byte, int64(readcount+128))
	_, err := self.Current_file.Read(data)
	if err != nil {
		panic("virtual_sdcard shutdown read")
	}
	logger.Info("Virtual sdcard (%f): %s\nUpcoming (%d): %s", readpos,
		string(data[:int(readcount)]), self.File_position, string(data[int(readcount):]))
	return nil
}

func (self *VirtualSD) Stats(eventtime float64) (bool, string) {
	if self.Work_timer != nil {
		return false, ""
	}
	return true, fmt.Sprintf("sd_pos=%d", self.File_position)
}

type flistNode struct {
	r_path string
	size   int64
}

func (self *VirtualSD) Get_file_list(check_subdirs bool) []flistNode {
	if check_subdirs {
		flist := []flistNode{}
		filestrlist := []string{}
		fileMap := make(map[string]*flistNode)
		filepath.Walk(self.Sdcard_dirname, func(path string, info fs.FileInfo, err error) error {
			if err != nil {
				logger.Error(err)
				return nil
			}

			if info.IsDir() {
				return nil
			}
			name := info.Name()
			ext := name[strings.LastIndex(name, ".")+1:]
			if sort.SearchStrings(VALID_GCODE_EXTS, ext) == len(VALID_GCODE_EXTS) {
				return nil
			}
			full_path := path + string(os.PathSeparator) + name
			r_path := full_path[len(self.Sdcard_dirname)+1:]
			size := info.Size()
			fileMap[full_path] = &flistNode{strings.ToLower(r_path), size}
			filestrlist = append(filestrlist, strings.ToLower(r_path))
			return nil
		})
		sort.Strings(filestrlist)
		for _, item := range filestrlist {
			val := fileMap[item]
			if val == nil {
				continue
			}
			flist = append(flist, *val)
		}
		return flist
	} else {
		dname := self.Sdcard_dirname
		dirs, err := ioutil.ReadDir(dname)
		if err != nil {
			logger.Errorf("virtual_sdcard get_file_list")
			panic("Unable to get file list")
		}
		flist := []flistNode{}
		filenames := make(map[string]fs.FileInfo)
		filestrlist := []string{}
		for _, dir := range dirs {
			filenames[strings.ToLower(dir.Name())] = dir
			filestrlist = append(filestrlist, strings.ToLower(dir.Name()))
		}
		sort.Strings(filestrlist)
		for _, item := range filestrlist {
			fname := filenames[item]
			if fname == nil {
				continue
			}
			if !strings.HasPrefix(fname.Name(), ".") && !fname.IsDir() {
				flist = append(flist, flistNode{fname.Name(), fname.Size()})
			}
		}
		return flist
	}
}
func (self *VirtualSD) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"file_path":     self.File_path(),
		"progress":      maths.Round(self.Progress(), 4),
		"is_active":     self.Is_active(),
		"file_position": self.File_position,
		"file_size":     self.File_size,
	}
}
func (self *VirtualSD) File_path() string {
	if self.Current_file != nil {
		return self.Current_file.Name()
	}
	return ""
}
func (self *VirtualSD) Progress() float64 {
	if self.File_size != 0 {
		return float64(self.File_position) / float64(self.File_size)
	} else {
		return 0.
	}
}
func (self *VirtualSD) Is_active() bool {
	return self.Work_timer != nil
}
func (self *VirtualSD) Do_pause() {
	if self.Work_timer != nil {
		self.Must_pause_work = true
		for self.Work_timer != nil && !self.Cmd_from_sd {
			self.Reactor.Pause(self.Reactor.Monotonic() + .001)
		}
	}
}

func (self *VirtualSD) Do_resume() error {
	if self.Work_timer != nil {
		panic(errors.New("SD busy"))
	}
	self.Must_pause_work = false
	self.Work_timer = self.Reactor.Register_timer(
		self.Work_handler, constants.NOW)
	return nil
}

func (self *VirtualSD) Do_cancel() {
	if self.Current_file != nil {
		self.Do_pause()
		self.Current_file.Close()
		self.Current_file = nil
		self.Print_stats.Note_cancel()
	}
	self.File_position = 0.
	self.File_size = 0.
}

// G-Code commands
func (self *VirtualSD) Cmd_error(argv interface{}) error {
	return errors.New("SD write not supported")
}
func (self *VirtualSD) Reset_file() {
	if self.Current_file != nil {
		self.Do_pause()
		self.Current_file.Close()
		self.Current_file = nil
	}
	self.File_position = 0.
	self.File_size = 0.
	self.Print_stats.Reset()
	self.Printer.Send_event("virtual_sdcard:reset_file", nil)
}

const cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print " +
	"if necessary"

func (self *VirtualSD) Cmd_SDCARD_RESET_FILE(argv interface{}) error {
	if self.Cmd_from_sd {
		return errors.New(
			"SDCARD_RESET_FILE cannot be run from the sdcard")
	}
	self.Reset_file()
	return nil
}

const cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May " +
	"include files in subdirectories."

const cmd_RESTORE_POWER_OFF_STATE = "Power back on and continue printing" +
	"EABLE 0 continue 1 cancel"

func (self *VirtualSD) Cmd_SDCARD_PRINT_FILE(argv interface{}) error {

	gcmd := argv.(*GCodeCommand)
	if self.Work_timer != nil {
		panic("SD busy")
	}
	self.Reset_file()
	filename := gcmd.Get("FILENAME", object.Sentinel{}, "", nil, nil, nil, nil)
	err := self.Load_file(gcmd, filename, true)
	if err != nil {
		return err
	}

	err = self.Do_resume()
	if err != nil {
		logger.Error(err)
	}

	return nil
}
func (self *VirtualSD) Cmd_M20(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// List SD card
	files := self.Get_file_list(false)
	gcmd.Respond_raw("Begin file list")
	for _, item := range files {
		gcmd.Respond_raw(fmt.Sprintf("%s %d", item.r_path, item.size))
	}
	gcmd.Respond_raw("End file list")
	return nil
}
func (self *VirtualSD) Cmd_M21(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Initialize SD card
	gcmd.Respond_raw("SD card ok")
	return nil
}
func (self *VirtualSD) Cmd_M23(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Select SD file
	if self.Work_timer != nil {
		return errors.New("SD busy")
	}
	self.Reset_file()
	filename := strings.TrimSpace(gcmd.Get_raw_command_parameters())
	if strings.HasPrefix(filename, "/") {
		filename = filename[1:]
	}
	self.Load_file(gcmd, filename, true)
	return nil
}
func (self *VirtualSD) Load_file(gcmd *GCodeCommand, filename string, check_subdirs bool) error {
	filename = strings.Trim(filename, "\"")
	logger.Debug("Load gcode file: ", filename)
	if !strings.HasSuffix(filename, ".gcode") {
		return errors.New("Gcode file extension incorrect")
	}

	var file string
	if strings.Index(filename, "/") != -1 {
		file = filepath.Join("/", filename)
	} else {
		file = filepath.Join(self.Sdcard_dirname, filename)
	}
	f, err := os.Stat(file)
	if err != nil {
		return fmt.Errorf("Unable to open file: %s", file)
	}
	fsize := f.Size()
	if fsize == 0 {
		return errors.New("Gcode file is empty")
	}
	logger.Debugf("File opened:%s Size:%d", strings.Trim(filename, "\""), fsize)
	logger.Debug("File selected")
	_file, _ := os.Open(file)
	if self.Current_file != nil {
		self.Current_file.Close()
		self.Current_file = nil
	}
	self.Current_file = _file
	self.File_position = 0
	self.File_size = fsize
	self.Print_stats.Set_current_file(strings.Trim(filename, "\""))
	return nil
}
func (self *VirtualSD) Cmd_M24(argv interface{}) error {
	// Start/resume SD print
	err := self.Do_resume()
	if err != nil {
		logger.Error(err)
	}
	return err
}
func (self *VirtualSD) Cmd_M25(argv interface{}) error {
	self.Do_pause()
	return nil
}
func (self *VirtualSD) Cmd_M26(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Set SD position
	if self.Work_timer != nil {
		panic("SD busy")
	}
	minval := 0
	pos := gcmd.Get_int("S", nil, &minval, &minval)
	self.File_position = pos
	return nil
}
func (self *VirtualSD) Cmd_M27(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Report SD print status
	if self.Current_file != nil {
		gcmd.Respond_raw("Not SD printing.")
		return nil
	}
	gcmd.Respond_raw(fmt.Sprintf("SD printing byte %d/%d", self.File_position, self.File_size))
	return nil
}
func (self *VirtualSD) Get_file_position() int {
	return self.Next_file_position
}
func (self *VirtualSD) Set_file_position(pos int) {
	self.Next_file_position = pos
}
func (self *VirtualSD) Is_cmd_from_sd() bool {
	return self.Cmd_from_sd
}

// Background work timer
func (self *VirtualSD) Work_handler(eventtime float64) float64 {
	logger.Debugf("Starting SD card print (position %d)", self.File_position)
	self.Reactor.Unregister_timer(self.Work_timer)
	_, err := self.Current_file.Seek(int64(self.File_position), 0)
	if err != nil {
		logger.Error("virtual_sdcard seek", err.Error())
		self.Work_timer = nil
		if self.Current_file != nil {
			self.Current_file.Close()
			self.Current_file = nil
		}
		return constants.NEVER
	}
	error_message := ""
	self.Print_stats.Note_start()
	gcode_mutex := self.Gcode.Get_mutex()
	partial_input := ""
	lines := []string{}

	for !self.Must_pause_work {
		if len(lines) <= 0 {
			// Read more data
			data := make([]byte, 8192)
			l, err := self.Current_file.Read(data)
			if err != nil && err != io.EOF {

				logger.Error("virtual_sdcard", err)
				if self.Current_file != nil {
					self.Current_file.Close()
					self.Current_file = nil
				}
				self.Gcode.Respond_raw("Read printing file error")
				error_message = "Printing file error"
				break
			}
			if l <= 0 {
				// End of file
				self.Current_file.Close()
				self.Current_file = nil
				logger.Debug("Finished SD card print")
				self.Gcode.Respond_raw("Done printing file")
				break
			}
			lines = strings.Split(string(data), "\n")
			lines[0] = partial_input + lines[0]
			partial_input = lines[len(lines)-1]
			lines = append([]string{}, lines[:len(lines)-1]...)
			lines = append([]string{}, util.Reverse(lines)...)
			self.Reactor.Pause(constants.NOW)
			continue
		}
		// Pause if any other request is pending in the gcode class
		if gcode_mutex.Test() {
			self.Reactor.Pause(self.Reactor.Monotonic() + 0.100)
			continue
		}
		// Dispatch command
		self.Cmd_from_sd = true
		line := lines[len(lines)-1]
		lines = append([]string{}, lines[:len(lines)-1]...)
		next_file_position := self.File_position + len(line) + 1
		line = strings.TrimRight(line, "\r")
		self.Next_file_position = next_file_position

		isBreak := false

		self.tryCatchWork_handler1(line, &error_message, &isBreak)
		if isBreak {
			if self.Current_file != nil {
				self.Current_file.Close()
				self.Current_file = nil
			}
			logger.Error(error_message)
			break
		}
		self.Cmd_from_sd = false
		self.File_position = self.Next_file_position
		// Do we need to skip around?
		if self.Next_file_position != next_file_position {
			_, err := self.Current_file.Seek(int64(self.File_position), 0)
			if err != nil {
				logger.Panic("virtual_sdcard seek")
				self.Work_timer = nil
				if self.Current_file != nil {
					self.Current_file.Close()
					self.Current_file = nil
				}
				return constants.NEVER
			}
			lines = []string{}
			partial_input = ""
		}
	}

	logger.Info("Exiting SD card print (position %d)", self.File_position)
	self.Work_timer = nil
	self.Cmd_from_sd = false
	if error_message != "" {
		self.Print_stats.Note_error(error_message)
	} else if self.Current_file != nil {
		self.Print_stats.Note_pause()
	} else {
		self.Print_stats.Note_complete()
	}
	return constants.NEVER
}

func (self *VirtualSD) tryCatchWork_handler1(line string, error_message *string, isBread *bool) {
	defer func() {
		if err := recover(); err != nil {
			if err == "exit" {
				panic(err)
			}
			e, ok1 := err.(*CommandError)
			if ok1 {
				*error_message = e.E
				self.tryCatchWork_handler11()
			} else if e1, ok1 := err.(string); ok1 {
				*error_message = e1
			} else if e2, ok2 := err.(*kerror.Error); ok2 {
				*error_message = e2.Error()
			}

			logger.Error("virtual_sdcard dispatch", line, err, string(debug.Stack()))
			if *error_message == "" {
				self.Gcode.Respond_raw("print file error")
			} else {
				self.Gcode.Respond_raw(*error_message)
			}

			*isBread = true
			return
		}
	}()
	*isBread = false
	self.Gcode.Run_script(line)
}

func (self *VirtualSD) tryCatchWork_handler11() {
	defer func() {
		if err := recover(); err != nil {
			if err == "exit" {
				panic(err)
			}
			logger.Error("virtual_sdcard on_error", err, string(debug.Stack()))
		}
	}()
	s, _ := self.On_error_gcode.(*TemplateWrapper).Render(nil)
	lStr := strings.TrimSpace(s)
	if strings.HasPrefix(lStr, "M") || strings.HasPrefix(lStr, "m") || strings.HasPrefix(lStr, "G") || strings.HasPrefix(lStr, "g") {
		self.Gcode.Run_script(s)
	}
}
func Load_config_virtual_sdcard(config *ConfigWrapper) interface{} {
	return NewVirtualSD(config)
}
