// Parse gcode commands
//
// Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"container/list"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/utils/reflects"
	"reflect"
	"runtime/debug"

	//"os"
	"regexp"
	"sort"
	"strconv"
	"strings"

	"unicode"
)

func isDigitChar(c byte) bool {
	return c >= '0' && c <= '9'
}

func isDigitString(s string) bool {
	if s == "" {
		return false
	}
	for _, r := range s {
		if !unicode.IsDigit(r) {
			return false
		}
	}
	return true
}

func isQuote(c rune) bool {
	return c == '"' || c == '\''
}

func isCommandValid(cmd string) bool {
	if strings.ToUpper(cmd) != cmd {
		return false
	}

	noUnderscore := strings.ReplaceAll(cmd, "_", "A")
	hasInvalidChar := false
	for _, r := range noUnderscore {
		if !unicode.IsLetter(r) && !unicode.IsDigit(r) {
			hasInvalidChar = true
			break
		}
	}
	if hasInvalidChar {
		return false
	}

	if len(cmd) > 0 && unicode.IsDigit([]rune(cmd)[0]) {
		return false
	}

	if len(cmd) > 1 && unicode.IsDigit([]rune(cmd)[1]) {
		return false
	}

	return true
}

func isValuePart(c uint8) bool {
	return isDigitChar(c) || c == '.' || c == '-'
}

func getKey(line string, pos *int) string {
	key := []byte{}
	for *pos < len(line) {
		c := line[*pos]
		if !isValuePart(c) {
			key = append(key, c)
			*pos++
		} else {
			break
		}
	}
	return string(key)
}

func getVal(line string, pos *int) string {
	val := []byte{}
	for *pos < len(line) {
		c := line[*pos]
		if isValuePart(c) {
			val = append(val, c)
			*pos++
		} else {
			break
		}
	}
	return string(val)
}

type CommandError struct {
	E string
}

type GCodeCommand struct {
	Command      string
	Commandline  string
	Params       map[string]string
	Need_ack     bool
	Respond_info func(msg string, log bool)
	Respond_raw  func(msg string)
}

func NewGCodeCommand(gcode *GCodeDispatch, command string, commandline string, params map[string]string, need_ack bool) *GCodeCommand {
	var self = GCodeCommand{}
	self.Command = command
	self.Commandline = commandline
	self.Params = params
	self.Need_ack = need_ack
	// Method wrappers
	self.Respond_info = gcode.Respond_info
	self.Respond_raw = gcode.Respond_raw
	return &self
}

func (self *GCodeCommand) Get_command() string {
	return self.Command
}

func (self *GCodeCommand) Get_commandline() string {
	return self.Commandline
}

func (self *GCodeCommand) Get_command_parameters() map[string]string {
	return self.Params
}

func (self *GCodeCommand) Get_raw_command_parameters() string {
	command := self.Command
	origline := self.Commandline
	param_start := len(command)
	param_end := len(origline)

	if strings.ToUpper(origline[:param_start]) != command {
		cmdIdx := strings.Index(strings.ToUpper(origline), command)
		if cmdIdx >= 0 {
			param_start = cmdIdx + len(command)
		}

		end := strings.LastIndex(origline, "*")
		if end >= 0 && isDigitString(origline[end+1:]) {
			param_end = end
		}
	}

	if param_start < len(origline) && unicode.IsSpace(rune(origline[param_start])) {
		param_start++
	}

	if param_start >= len(origline) {
		return ""
	}
	if param_end > len(origline) {
		param_end = len(origline)
	}
	if param_start >= param_end {
		return ""
	}

	return origline[param_start:param_end]
}

func (self *GCodeCommand) Ack(msg string) bool {
	if self.Need_ack == false {
		return false
	}
	var ok_msg = "ok"
	if msg != "" {
		ok_msg = fmt.Sprintf("ok %s", msg)
	}
	self.Respond_raw(ok_msg)
	self.Need_ack = false
	return true
}

func (self *GCodeCommand) parse_extended_params(input string) (map[string]string, error) {

	const (
		ReadKey = iota
		ReadVal
		ReadEnd
	)

	pos := 0
	currentKey := ""
	params := strings.TrimSpace(input)
	results := make(map[string]string)

	state := ReadKey
	for state != ReadEnd {
		switch state {
		case ReadKey:
			eqIdx := strings.IndexByte(params[pos:], '=')
			if eqIdx == -1 {
				state = ReadEnd
				continue
			}
			eqIdx += pos

			key := params[pos:eqIdx]
			currentKey = strings.ToUpper(strings.TrimSpace(key))

			params = params[eqIdx+1:]
			pos = 0
			state = ReadVal

		case ReadVal:
			if len(params) == 0 {
				state = ReadEnd
				continue
			}

			firstChar := rune(params[0])
			if isQuote(firstChar) {
				endQuoteIdx := strings.IndexRune(params[1:], firstChar)
				if endQuoteIdx == -1 {
					return nil, fmt.Errorf("malformed command: missing closing quote")
				}
				endQuoteIdx++
				value := params[1:endQuoteIdx]
				results[currentKey] = value
				if endQuoteIdx+1 < len(params) {
					params = strings.TrimLeft(params[endQuoteIdx+1:], " \t")
					pos = 0
					state = ReadKey
				} else {
					state = ReadEnd
				}
			} else {
				spaceIdx := strings.IndexByte(params, ' ')
				if spaceIdx == -1 {
					results[currentKey] = strings.TrimSpace(params)
					state = ReadEnd
				} else {
					value := params[:spaceIdx]
					results[currentKey] = strings.TrimSpace(value)
					params = strings.TrimLeft(params[spaceIdx+1:], " \t")
					pos = 0
					state = ReadKey
				}
			}
		}
	}

	return results, nil
}

func (self *GCodeCommand) Get(name string, _default interface{}, parser interface{}, minval *float64, maxval *float64,
	above *float64, below *float64) string {
	var value = self.Params[name]
	if value == "" {
		if _, ok := _default.(object.Sentinel); ok {
			panic(fmt.Errorf("Error on '%s': missing %s", self.Commandline, name))
		}
		if _default != nil {
			return _default.(string)
		} else {
			return ""
		}
	}

	value = string(value)
	f, _ := strconv.ParseFloat(value, 64)
	if minval != nil && f < *minval {
		logger.Errorf("Error on '%s': %s must have minimum of %f", self.Commandline, name, *minval)
	}

	if maxval != nil && f > *maxval {
		logger.Errorf("Error on '%s': %s must have maximum of %f", self.Commandline, name, *maxval)
	}

	if above != nil && f <= *above {
		logger.Errorf("Error on '%s': %s must have above of %f", self.Commandline, name, *above)
	}

	if below != nil && f >= *below {
		logger.Errorf("Error on '%s': %s must have below of %f", self.Commandline, name, *below)
	}
	return value
}

func (self *GCodeCommand) Get_int(name string, _default interface{}, minval *int, maxval *int) int {
	var value = self.Params[name]
	if value == "" {
		if _, ok := _default.(object.Sentinel); ok {
			panic(fmt.Errorf("Error on '%s': missing %s", self.Commandline, name))
		}
		if _default != nil {
			return _default.(int)
		} else {
			return 0
		}
	}

	var val, _ = strconv.ParseInt(value, 10, 32)
	if minval != nil && int(val) < cast.Int(minval) {
		logger.Errorf("Error on '%s': %s must have minimum of %d", self.Commandline, name, cast.Int(minval))
	}

	if maxval != nil && int(val) > cast.Int(maxval) {
		logger.Errorf("Error on '%s': %s must have maximum of %d", self.Commandline, name, cast.Int(maxval))
	}

	return int(val)
}

func (self *GCodeCommand) Get_float(name string, _default interface{}, minval *float64, maxval *float64,
	above *float64, below *float64) float64 {

	var value = self.Params[name]
	if value == "" {
		if _, ok := _default.(object.Sentinel); ok {
			panic(fmt.Errorf("Error on '%s': missing %s", self.Commandline, name))
		}
		if _default != nil {
			return _default.(float64)
		} else {
			return 0
		}
	}

	var val, _ = strconv.ParseFloat(value, 64)
	if minval != nil && val < *minval {
		logger.Errorf("Error on '%s': %s must have minimum of %f", self.Commandline, name, *minval)
		return *minval
	}

	if maxval != nil && val > *maxval {
		logger.Errorf("Error on '%s': %s must have maximum of %f", self.Commandline, name, *maxval)
		return *maxval
	}

	if above != nil && val <= *above {
		logger.Errorf("Error on '%s': %s must have above of %f", self.Commandline, name, *above)
		return *above
	}

	if below != nil && val >= *below {
		logger.Errorf("Error on '%s': %s must have below of %f", self.Commandline, name, *below)
		return *below
	}
	return val
}

func (self *GCodeCommand) Get_floatP(name string, _default *float64, minval *float64, maxval *float64,
	above *float64, below *float64) *float64 {

	value, ok := self.Params[name]
	if !ok {
		return _default
	}

	var val, _ = strconv.ParseFloat(value, 64)
	if minval != nil && val < cast.Float64(minval) {
		logger.Errorf("Error on '%s': %s must have minimum of %f", self.Commandline, name, cast.Float64(minval))
	}

	if maxval != nil && val > cast.Float64(maxval) {
		logger.Errorf("Error on '%s': %s must have maximum of %f", self.Commandline, name, cast.Float64(maxval))
	}

	if above != nil && val <= cast.Float64(above) {
		logger.Errorf("Error on '%s': %s must have above of %f", self.Commandline, name, cast.Float64(above))
	}

	if below != nil && val >= cast.Float64(below) {
		logger.Errorf("Error on '%s': %s must have below of %f", self.Commandline, name, cast.Float64(above))
	}
	return &val
}

func (self *GCodeCommand) Has(name string) bool {
	_, ok := self.Params[name]
	return ok
}

// Parse and dispatch G-Code commands
type GCodeDispatch struct {
	Error                     *CommandError
	Coord                     []string
	Printer                   *Printer
	Is_fileinput              bool
	Mutex                     *ReactorMutex
	Output_callbacks          []func(string)
	Base_gcode_handlers       map[string]interface{}
	Ready_gcode_handlers      map[string]interface{}
	Mux_commands              map[string]list.List
	Gcode_help                map[string]string
	Is_printer_ready          bool
	Gcode_handlers            map[string]interface{}
	Cmd_RESTART_help          string
	Cmd_FIRMWARE_RESTART_help string
	Cmd_STATUS_help           string
	Cmd_HELP_help             string
}

func NewGCodeDispatch(printer *Printer) *GCodeDispatch {
	var self = GCodeDispatch{}
	self.Printer = printer
	var is_check = printer.Get_start_args()["debuginput"]
	if is_check != "" {
		self.Is_fileinput = true
	} else {
		self.Is_fileinput = false
	}

	printer.Register_event_handler("project:ready", self.Handle_ready)
	printer.Register_event_handler("project:shutdown", self.Handle_shutdown)
	printer.Register_event_handler("project:disconnect",
		self.Handle_disconnect)
	// Command handling
	self.Is_printer_ready = false
	self.Mutex = printer.Get_reactor().Mutex(false)
	self.Output_callbacks = []func(string){}
	self.Base_gcode_handlers = map[string]interface{}{}
	self.Gcode_handlers = self.Base_gcode_handlers
	self.Ready_gcode_handlers = map[string]interface{}{}
	self.Mux_commands = map[string]list.List{}
	self.Gcode_help = map[string]string{}
	// Register commands needed before config file is loaded
	var handlers = [...]string{"M110", "M115", "M117", "RESTART", "FIRMWARE_RESTART", "ECHO", "STATUS", "HELP"}
	self.Cmd_RESTART_help = "Reload config file and restart host software"
	self.Cmd_FIRMWARE_RESTART_help = "Restart firmware, host, and reload config"
	self.Cmd_STATUS_help = "Report the printer status"
	self.Cmd_HELP_help = "Report the list of available extended G-Code commands"

	for _, cmd := range handlers {
		var _func = reflects.GetMethod(&self, "Cmd_"+cmd)
		var desc = reflects.ReflectFieldValue(&self, "cmd_"+cmd+"_help")
		if desc == nil {
			desc = ""
		}

		self.Register_command(cmd, _func, true, desc.(string))
	}

	self.Coord = []string{"x", "y", "z", "e"}
	return &self
}

func (self *GCodeDispatch) Is_traditional_gcode(cmd string) bool {
	if cmd == "" {
		return false
	}
	cmd = strings.Split(strings.ToUpper(cmd), " ")[0]
	return (len(cmd) > 1) && unicode.IsUpper(rune(cmd[0])) && unicode.IsDigit(rune(cmd[1]))
}

func (self *GCodeDispatch) Register_command(cmd string, _func interface{}, when_not_ready bool, desc string) interface{} {
	if _func == nil {
		var old_cmd = self.Ready_gcode_handlers[cmd]
		if self.Ready_gcode_handlers[cmd] != nil {
			delete(self.Ready_gcode_handlers, cmd)
		}
		if self.Base_gcode_handlers[cmd] != nil {
			delete(self.Base_gcode_handlers, cmd)
		}
		return old_cmd
	}
	_, ok := self.Ready_gcode_handlers[cmd]
	if ok {
		panic(fmt.Sprintf("gcode command %s already registered", cmd))
	}
	if self.Is_traditional_gcode(cmd) == false {
		if !isCommandValid(cmd) {
			panic(fmt.Errorf(
				"Can't register '%s' as it is an invalid name", cmd))
		}
		var origfunc = _func
		_func = func(params interface{}) error {
			paras := self.Get_extended_params(params.(*GCodeCommand))
			m := reflect.TypeOf(origfunc).Name()
			if m == "Value" {
				reflects.ReqArgs(origfunc.(reflect.Value), map[string]interface{}{"gcmd": paras})
			} else {
				err := origfunc.(func(interface{}) error)(paras)
				if err != nil {
					logger.Error("Register_command ", cmd, " error:", err, string(debug.Stack()))
					panic(err)
				}
			}
			return nil
		}
	}
	self.Ready_gcode_handlers[cmd] = _func
	if when_not_ready {
		self.Base_gcode_handlers[cmd] = _func
	}
	if desc != "" {
		self.Gcode_help[cmd] = desc
	}
	return nil
}

func (self *GCodeDispatch) Register_mux_command(cmd string, key string, value string, _func func(interface{}) error, desc string) {
	prev, ok := self.Mux_commands[cmd]
	if !ok && prev.Len() <= 0 {
		var handler = func(gcmd interface{}) error {
			return self.Cmd_mux(cmd, gcmd)
		}
		self.Register_command(cmd, handler, false, desc)
		prev.PushBack(key)
		prev.PushBack(map[string]interface{}{})
		self.Mux_commands[cmd] = prev
	}
	var prev_key, prev_values = prev.Front(), prev.Back()
	if prev_key.Value.(string) != key {
		panic(fmt.Sprintf(
			"mux command %s %s %s may have only one key (%#v)",
			cmd, key, value, prev_key))
	}
	if prev_values.Value.(map[string]interface{})[value] != nil {
		panic(fmt.Sprintf(
			"mux command %s %s %s already registered (%#v)",
			cmd, key, value, prev_values))
	}
	prev_values.Value.(map[string]interface{})[value] = _func
}

func (self *GCodeDispatch) Get_command_help() map[string]string {
	return self.Gcode_help
}

func (self *GCodeDispatch) Register_output_handler(cb func(string)) {
	self.Output_callbacks = append(self.Output_callbacks, cb)
}

func (self *GCodeDispatch) Handle_shutdown([]interface{}) error {
	if self.Is_printer_ready == false {
		return nil
	}
	self.Is_printer_ready = false
	self.Gcode_handlers = self.Base_gcode_handlers
	self.Respond_state("Shutdown")
	return nil
}

func (self *GCodeDispatch) Handle_disconnect([]interface{}) error {
	self.Respond_state("Disconnect")
	return nil
}

func (self *GCodeDispatch) Handle_ready([]interface{}) error {
	self.Is_printer_ready = true
	self.Gcode_handlers = self.Ready_gcode_handlers
	self.Respond_state("Ready")
	return nil
}

func (self *GCodeDispatch) parseGcode(input string) []string {
	tokens := []string{""}
	if input == "" {
		return tokens
	}

	parts := strings.Fields(input)
	for _, part := range parts {
		pos := 0
		for pos < len(part) {
			tokens = append(tokens, getKey(part, &pos))
			tokens = append(tokens, getVal(part, &pos))
		}
	}

	return tokens
}

func (self *GCodeDispatch) Process_commands(commands []string, need_ack bool) {
	for _, line := range commands {
		// Ignore comments and leading/trailing spaces
		line = strings.Trim(line, " ")
		origline := line
		if cpos := strings.Index(line, ";"); cpos != -1 {
			line = line[:cpos]
		}

		parts := self.parseGcode(strings.ToUpper(line))
		cmd := ""
		if len(parts) >= 3 && parts[1] == "N" {
			cmd = strings.TrimSpace(strings.Join(parts[3:], ""))
		} else if len(parts) >= 3 {
			cmd = strings.TrimSpace(strings.Join(parts[0:3], ""))
		}

		params := make(map[string]string)
		for i := 1; i < len(parts)-1; i += 2 {
			key := parts[i]
			value := ""
			if i+1 < len(parts) {
				value = strings.TrimSpace(parts[i+1])
			}
			params[key] = value
		}
		var gcmd = NewGCodeCommand(self, cmd, origline, params, need_ack)
		// Invoke handler for command
		var handler = self.Gcode_handlers[cmd]
		if handler == nil {
			handler = self.Cmd_default
		}
		m := reflect.TypeOf(handler).Name()
		if m == "Value" {
			argv := []reflect.Value{reflect.ValueOf(gcmd)}
			res := handler.(reflect.Value).Call(argv)
			if len(res) > 1 {
				logger.Debug(res)
			}
		} else {
			var _handler = handler.(func(interface{}) error)
			err := _handler(gcmd)
			if err != nil {
				logger.Error(err)
				msg := fmt.Sprintf("Internal error on command: %s \n", cmd)
				self.Printer.Invoke_shutdown(msg)
				self.Respond_error(msg)
				panic(err)
			}
		}
		gcmd.Ack("")
	}
}

func (self *GCodeDispatch) Run_script_from_command(script string) {
	self.Process_commands(strings.Split(script, "\n"), false)
}

func (self *GCodeDispatch) Run_script(script string) {
	if script == "CANCEL_PRINT" {
		self.Printer.Send_event("project:pre_cancel", nil)
	}
	self.Mutex.Lock()
	defer func() {
		self.Mutex.Unlock()
		if script == "CANCEL_PRINT" {
			self.Printer.Send_event("project:post_cancel", nil)
		}
	}()
	self.Process_commands(strings.Split(script, "\n"), false)
}

func (self *GCodeDispatch) Get_mutex() *ReactorMutex {
	return self.Mutex
}

func (self *GCodeDispatch) Create_gcode_command(command string, commandline string, params map[string]string) *GCodeCommand {
	return NewGCodeCommand(self, command, commandline, params, false)
}

// Response handling
func (self *GCodeDispatch) Respond_raw(msg string) {
	for _, cb := range self.Output_callbacks {
		cb(msg)
	}
}

func (self *GCodeDispatch) Respond_info(msg string, _log bool) {
	if _log {
		logger.Info(msg)
		var lines []string
		for _, l := range strings.Split(strings.TrimSpace(msg), "\n") {
			lines = append(lines, strings.TrimSpace(l))
		}
		self.Respond_raw(strings.Join(lines, "\n // "))
	} else {
		logger.Info(msg)
	}
}

func (self *GCodeDispatch) Respond_error(msg string) {
	//logging.warning(msg)
	var lines = strings.Split(strings.TrimSpace(msg), "\n")
	if len(lines) > 1 {
		self.Respond_info(fmt.Sprintf("%s\n", lines), true)
	}
	self.Respond_raw(fmt.Sprintf("!! %s", strings.TrimSpace(lines[0])))
	if self.Is_fileinput == true {
		self.Printer.Request_exit("error_exit")
	}
}

func (self *GCodeDispatch) Respond_state(state string) {
	self.Respond_info(fmt.Sprintf("project state: %s", state), true)
}

func (self *GCodeDispatch) Get_extended_params(gcmd *GCodeCommand) *GCodeCommand {
	gcmd.Params, _ = gcmd.parse_extended_params(gcmd.Get_raw_command_parameters())
	return gcmd
}

// G-Code special command handlers
func (self *GCodeDispatch) Cmd_default(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	var cmd = gcmd.Get_command()
	if cmd == "M105" {
		// Don"t warn about temperature requests when not ready
		gcmd.Ack("T:0")
		return nil
	}
	if cmd == "M21" {
		// Don"t warn about sd card init when not ready
		return nil
	}
	if self.Is_printer_ready == false {
		//panic(fmt.Sprintf(self.Is_fileinput.Get_state_message()[0]))
		return nil
	}

	if strings.Contains(cmd, "") {
		//# Handle M117/M118 gcode with numeric and special characters
		realcmd := strings.Split(cmd, " ")[0]
		if realcmd == "M117" || realcmd == "M118" || realcmd == "M23" {
			handler := self.Gcode_handlers[realcmd]
			if handler != nil {
				gcmd.Command = realcmd
				handler.(func(interface{}) error)(gcmd)
				return nil
			}
		}
	} else if cmd == "M140" || cmd == "M104" && gcmd.Get_float("S", 0., nil, nil, nil, nil) == -1 {
		// Don"t warn about requests to turn off heaters when not present
		return nil
	} else if cmd == "M107" || (cmd == "M106" && (gcmd.Get_float("S", 1., nil, nil, nil, nil) == -1 || self.Is_fileinput)) {
		// Don"t warn about requests to turn off fan when fan not present
		return nil
	}
	if cmd != "" {
		logger.Warn("Unknown command:", cmd)
	}
	return nil
}

func (self *GCodeDispatch) Cmd_mux(command string, argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	var cmd_list = self.Mux_commands[command]
	var key = cmd_list.Remove(cmd_list.Front()).(string)
	var values = cmd_list.Remove(cmd_list.Back())
	var key_param string
	if _, ok := values.(map[string]interface{})[""]; ok {
		key_param = gcmd.Get(key, nil, "", nil, nil, nil, nil)
	} else {
		key_param = gcmd.Get(key, object.Sentinel{}, "", nil, nil, nil, nil)
	}
	var vals = values.(map[string]interface{})
	if vals[key_param] == nil {
		panic(fmt.Sprintf("The value %s is not valid for %s", key_param, key))
	}
	return vals[key_param].(func(interface{}) error)(argv)
}

// Low-level G-Code commands that are needed before the config file is loaded
func (self *GCodeDispatch) Cmd_M110(argv interface{}) {
	logger.Debug("Cmd_M110")
	// Set Current Line Number
}

func (self *GCodeDispatch) Cmd_M112(argv interface{}) {
	// Emergency Stop
	self.Printer.Invoke_shutdown("Shutdown due to M112 command")
}

func (self *GCodeDispatch) Cmd_M115(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	// Get Firmware Version and Capabilities
	var software_version = self.Printer.Get_start_args()["software_version"]
	var kw = map[string]string{"FIRMWARE_NAME": "project", "FIRMWARE_VERSION": software_version.(string)}
	var msg string
	for k, v := range kw {
		msg += fmt.Sprintf("%s:%s ", k, v)
	}

	var did_ack = gcmd.Ack(msg)
	if did_ack == false {
		gcmd.Respond_info(msg, true)
	}
}

func (self *GCodeDispatch) Request_restart(result string) {
	if self.Is_printer_ready == true {
		toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{})
		var print_time = toolhead.(*Toolhead).Get_last_move_time()
		if result == "exit" {
			logger.Info("Exiting (print time %.3fs)", print_time)
		}
		self.Printer.Send_event("gcode:request_restart", []interface{}{print_time})
		toolhead.(*Toolhead).Dwell(0.500)
		toolhead.(*Toolhead).Wait_moves()
	}
	self.Printer.Request_exit(result)

}

func (self *GCodeDispatch) Cmd_RESTART(argv interface{}) {
	self.Request_restart("restart")
}

func (self *GCodeDispatch) Cmd_FIRMWARE_RESTART(argv interface{}) {
	self.Request_restart("firmware_restart")
}

func (self *GCodeDispatch) Cmd_ECHO(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	gcmd.Respond_info(gcmd.Get_commandline(), true)
}

func (self *GCodeDispatch) Cmd_STATUS(argv interface{}) {
	if self.Is_printer_ready == true {
		self.Respond_state("Ready")
		return
	}
	msg, _ := self.Printer.get_state_message()
	msg = strings.TrimRight(msg, "") + "\n state: Not ready"
	panic(msg)
}

func (self *GCodeDispatch) Cmd_HELP(argv interface{}) {
	gcmd := argv.(*GCodeCommand)
	var cmdhelp []string
	if self.Is_printer_ready == false {
		cmdhelp = append(cmdhelp, "Printer is not ready - not all commands available.")
	}
	cmdhelp = append(cmdhelp, "Available extended commands:")
	var ks []string
	for k, _ := range self.Gcode_handlers {
		ks = append(ks, k)
	}

	sort.Sort(sort.StringSlice(ks))
	for _, cmd := range ks {
		if self.Gcode_help[cmd] != "" {
			cmdhelp = append(cmdhelp, fmt.Sprintf("%-10s: %s", cmd, self.Gcode_help[cmd]))
		}
	}

	gcmd.Respond_info(strings.Join(cmdhelp, "\n"), true)
}

// Support reading gcode from a pseudo-tty interface
type GCodeIO struct {
	Printer            *Printer
	Gcode              *GCodeDispatch
	Fd                 interface{}
	Gcode_mutex        *ReactorMutex
	Reactor            IReactor
	Is_printer_ready   bool
	Is_processing_data bool
	Is_fileinput       bool
	Pipe_is_active     bool
	Fd_handle          *ReactorFileHandler
	Partial_input      string
	Pending_commands   []string
	Bytes_read         int
	Input_log          list.List
	M112_r             *regexp.Regexp
}

func NewGCodeIO(printer *Printer) *GCodeIO {
	var self = GCodeIO{}
	self.Printer = printer
	printer.Register_event_handler("project:ready", self.Handle_ready)
	printer.Register_event_handler("project:shutdown", self.Handle_shutdown)
	Gcode := printer.Lookup_object("gcode", object.Sentinel{})
	self.Gcode = Gcode.(*GCodeDispatch)
	self.Gcode_mutex = self.Gcode.Get_mutex()
	self.Fd = printer.Get_start_args()["gcode_fd"]
	self.Reactor = printer.Get_reactor()
	self.Is_printer_ready = false
	self.Is_processing_data = false
	var is_check = printer.Get_start_args()["debuginput"]
	if is_check != nil {
		self.Is_fileinput = true
	} else {
		self.Is_fileinput = false
	}
	self.Pipe_is_active = true
	self.Fd_handle = nil
	if self.Is_fileinput == false {
		self.Gcode.Register_output_handler(self.Respond_raw)
		self.Fd_handle = self.Reactor.Register_fd(self.Fd.(int),
			self.Process_data, nil)
	}
	self.Partial_input = ""
	self.Pending_commands = make([]string, 0)
	self.Bytes_read = 0
	self.Input_log = list.List{}
	return &self
}

func (self *GCodeIO) Handle_ready([]interface{}) error {
	self.Is_printer_ready = true
	if self.Is_fileinput && self.Fd_handle == nil {
		self.Fd_handle = self.Reactor.Register_fd(self.Fd.(int),
			self.Process_data, nil)
	}
	return nil
}

func (self *GCodeIO) Dump_debug() {
	var out []string
	out = append(out, fmt.Sprintf("Dumping gcode input %d blocks", self.Input_log.Len()))
	for i := 0; i < self.Input_log.Len(); i++ {
		var tmp_list = self.Input_log.Remove(self.Input_log.Front()).(list.List)
		var eventtime = tmp_list.Remove(tmp_list.Front()).(float64)
		var data = tmp_list.Remove(tmp_list.Front()).(string)
		out = append(out, fmt.Sprintf("Read %f: %s", eventtime, data))
	}
	for i := 0; i < len(out); i++ {
		logger.Debug(fmt.Sprintf("%s\n", out[i]))
	}
}

func (self *GCodeIO) Handle_shutdown([]interface{}) error {
	if self.Is_printer_ready == false {
		return nil
	}
	self.Is_printer_ready = false
	self.Dump_debug()
	if self.Is_fileinput == true {
		self.Printer.Request_exit("error_exit")
	}
	self.M112_r, _ = regexp.Compile("^(?:[nN][0-9]+)?\\s*[mM]112(?:\\s|$)")
	return nil
}

func (self *GCodeIO) Process_data(eventtime float64) interface{} {
	// Read input, separate by newline, and add to pending_commands
	// try:
	//var data = string(os.read(self.Fd.(int), 4096).decode())
	var data = ""
	// except (os.error, UnicodeDecodeError):
	// 	logging.exception("Read g-code")
	// 	return
	var tmp = list.New()
	tmp.PushBack(eventtime)
	tmp.PushBack(data)
	self.Input_log.PushBack(tmp)
	self.Bytes_read += len(data)
	var lines = strings.Split(data, "\n")
	lines[0] = self.Partial_input + lines[0]
	self.Partial_input = lines[len(lines)-1]
	var pending_commands = self.Pending_commands
	pending_commands = append(pending_commands, lines...)
	self.Pipe_is_active = true
	// Special handling for debug file input EOF
	if len(data) == 0 && self.Is_fileinput == true {
		if self.Is_processing_data == false {
			//self.Reactor.Unregister_fd(self.Fd_handle)
			self.Fd_handle = nil
			self.Gcode.Request_restart("exit")
		}
		pending_commands = append(pending_commands, "")
	}
	// Handle case where multiple commands pending
	if self.Is_processing_data || len(pending_commands) > 1 {
		if len(pending_commands) < 20 {
			// Check for M112 out-of-order
			for _, line := range lines {
				if self.M112_r.Match([]byte(line)) != false {
					self.Gcode.Cmd_M112(&GCodeCommand{})
				}
			}
		}
		if self.Is_processing_data == true {
			if len(pending_commands) >= 20 {
				// Stop reading input
				self.Reactor.Unregister_fd(self.Fd_handle)
				self.Fd_handle = nil
			}
			return nil
		}
	}
	// Process commands
	self.Is_processing_data = true
	for {
		self.Pending_commands = nil
		self.Gcode_mutex.Lock()
		self.Gcode.Process_commands(pending_commands, true)
		self.Gcode_mutex.Unlock()
		pending_commands = self.Pending_commands
		if pending_commands == nil {
			break
		}
	}
	self.Is_processing_data = false
	if self.Fd_handle != nil {
		self.Fd_handle = self.Reactor.Register_fd(self.Fd.(int),
			self.Process_data, nil)
	}
	return nil
}

func (self *GCodeIO) Respond_raw(msg string) {
	if self.Pipe_is_active == true {
		// try:
		//os.write(self.Fd.(int), (msg + "\n").encode())
		// except os.error:
		// 	logging.exception("Write g-code response")
		// 	self.Pipe_is_active = false
	}
}

func (self *GCodeIO) Stats(eventtime float64) (bool, string) {
	return true, fmt.Sprintf("gcodein=%d", self.Bytes_read)
}

func Add_early_printer_objects1(printer *Printer) {
	printer.Add_object("gcode", NewGCodeDispatch(printer))
	//printer.Add_object("gcode_io", NewGCodeIO(printer))
}
