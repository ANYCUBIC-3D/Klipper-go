package project

import (
	"errors"
	"flag"
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"k3c/project/util"
	"runtime/debug"

	"os"
	"runtime"
	"strings"
	"time"
)

const (
	message_ready   = "Printer is ready"
	message_startup = "Printer is not ready\nThe project host software is attempting to connect.Please\nretry in a few moments."

	message_restart = "Once the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software.\nPrinter is halted"

	message_protocol_error1 = "This is frequently caused by running an older version of the\nfirmware on the MCU(s).Fix by recompiling and flashing the\nfirmware."

	message_protocol_error2 = "Once the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software."

	message_mcu_connect_error = "Once the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nError configuring printer"

	message_shutdown = "Once the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.Printer is shutdown"
)

type Printer struct {
	config_error      *Config_error
	Command_error     *CommandError
	Start_args        map[string]interface{}
	reactor           IReactor
	state_message     string
	in_shutdown_state bool
	run_result        string
	event_handlers    map[string][]func([]interface{}) error
	objects           map[string]interface{}
	Module            map[string]interface{}
}

func NewPrinter(main_reactor IReactor, start_args map[string]interface{}) *Printer {
	self := Printer{Start_args: map[string]interface{}{}}
	self.config_error = &Config_error{}
	self.Command_error = &CommandError{}

	self.Start_args = start_args
	self.reactor = main_reactor
	self.reactor.Register_callback(self._connect, constants.NOW)
	self.state_message = message_startup
	self.in_shutdown_state = false
	self.run_result = ""
	self.event_handlers = map[string][]func([]interface{}) error{}
	self.objects = make(map[string]interface{})
	// Init printer components that must be setup prior to config
	for _, m := range []func(*Printer){Add_early_printer_objects1,
		Add_early_printer_objects_webhooks} {
		m(&self)
	}
	self.Module = LoadMainModule()
	return &self
}
func (self *Printer) Get_start_args() map[string]interface{} {
	return self.Start_args
}
func (self *Printer) Get_reactor() IReactor {
	return self.reactor
}
func (self *Printer) get_state_message() (string, string) {
	var category string
	if self.state_message == message_ready {
		category = "ready"
	} else if self.state_message == message_startup {
		category = "startup"
	} else if self.in_shutdown_state {
		category = "shutdown"
	} else {
		category = "error"
	}
	return self.state_message, category
}
func (self *Printer) Is_shutdown() bool {
	return self.in_shutdown_state
}
func (self *Printer) _set_state(msg string) {
	if self.state_message == message_ready || self.state_message == message_startup {
		self.state_message = msg
	}
	if msg != message_ready && self.Start_args["debuginput"] != nil {
		self.Request_exit("error_exit")
	}
}
func (self *Printer) Add_object(name string, obj interface{}) error {
	_, ok := self.objects[name]
	if ok {
		return errors.New(strings.Join([]string{"Printer object '", name, "' already created"}, ""))
	}
	self.objects[name] = obj
	return nil
}
func (self *Printer) Lookup_object(name string, default1 interface{}) interface{} {
	_, ok := self.objects[name]
	if ok {
		return self.objects[name]
	}
	if _, ok := default1.(object.Sentinel); ok {
		logger.Error(strings.Join([]string{"Unknown config object '", name, "' "}, ""))
	}
	return default1
}
func (self *Printer) Lookup_objects(module string) []interface{} {
	mods := []interface{}{}
	if module == "" {
		for _, v := range self.objects {
			mods = append(mods, v)
		}
		return mods
	}
	prefix := module + " "
	for k, v := range self.objects {
		mod := map[string]interface{}{}
		if strings.HasPrefix(k, prefix) {
			mod[k] = v
			mods = append(mods, mod)
		}
	}
	obj, ok := self.objects[module]
	if ok {
		mod := map[string]interface{}{}
		mod[module] = obj
		mods = append(mods, mod)
	}
	return mods
}

func (self *Printer) load_object1(config *ConfigWrapper, section string) interface{} {
	for k, object := range self.objects {
		if k == section {
			return object
		}
	}

	if strings.HasSuffix(section, " default") || strings.HasSuffix(section, " adaptive") {
		if _, ok := self.Module[strings.Split(section, " ")[0]]; !ok {
			logger.Errorf("%s depend on %s, should loaded before", section, strings.Split(section, " ")[0])
		}
		logger.Debugf("%s only as config, don't use for load object", section)
		return self.Module[strings.Split(section, " ")[0]]
	}
	init_func, ok := self.Module[section]
	if ok {
		s := config.Getsection(section)
		self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
	} else {
		init_func, ok = self.Module[strings.Split(section, " ")[0]]
		if ok {
			s := config.Getsection(section)
			self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
		}
	}
	return self.objects[section]
}

func (self *Printer) reload_object(config *ConfigWrapper, section string) interface{} {
	module_parts := strings.Split(section, " ")
	_section := section
	if strings.HasPrefix(section, "gcode_macro") && len(module_parts) > 1 {
		_section = "gcode_macro_1"
	}
	init_func, ok := self.Module[_section]
	if ok {
		s := config.Getsection(section)
		self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
	} else {
		init_func, ok = self.Module[strings.Split(_section, " ")[0]]
		if ok {
			s := config.Getsection(_section)
			self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
		}
	}
	return self.objects[section]
}

func (self *Printer) Load_object(config *ConfigWrapper, section string, default1 interface{}) interface{} {
	obj := self.load_object1(config, section)
	if obj == nil {
		if _, ok := default1.(*object.Sentinel); ok {
			self.config_error.E = fmt.Sprintf("Unable to load module '%s'", section)
			logger.Info("moudle as ", section, " is not support")
		} else {
			if _, ok := default1.(object.Sentinel); ok {
				self.config_error.E = fmt.Sprintf("Unable to load module '%s'", section)
				logger.Info("moudle as ", section, " is not support")
			} else {
				return default1
			}

		}
	}
	return obj
}

func (self *Printer) _read_config() {
	pconfig := NewPrinterConfig(self)
	self.objects["configfile"] = pconfig

	config := pconfig.Read_main_config()
	// Create printer components
	for _, m := range []func(*ConfigWrapper){Add_printer_objects_pins,
		Add_printer_objects_mcu} {
		m(config)
	}

	for _, section_config := range config.Get_prefix_sections("") {
		self.Load_object(config, section_config.Get_name(), value.None)
	}
	for _, m := range []func(*ConfigWrapper){Add_printer_objects_toolhead, Load_config_tuning_tower} {
		m(config)
	}
	// Validate that there are no undefined parameters in the config file
	pconfig.Check_unused_options(config)
}
func (self *Printer) _build_protocol_error_message(e interface{}) string {
	host_version := self.Start_args["software_version"]
	msg_update := []string{}
	msg_updated := []string{}
	for _, m := range self.Lookup_objects("mcu") {
		mcu := m.(map[string]interface{})["mcu"].(*MCU)
		mcu_name := m.(map[string]interface{})["mcu_name"]
		mcu_version := mcu.Get_status(0)["mcu_version"]

		if mcu_version != host_version {
			msg_update = append(msg_update, fmt.Sprintf("%s: Current version %s", strings.TrimSpace(mcu_name.(string)), mcu_version))
		} else {
			msg_updated = append(msg_updated, fmt.Sprintf("%s: Current version %s", strings.TrimSpace(mcu_name.(string)), mcu_version))
		}
	}
	if len(msg_update) == 0 {
		msg_update = append(msg_update, "<none>")
	}
	if len(msg_updated) == 0 {
		msg_updated = append(msg_updated, "<none>")
	}
	msg := "MCU Protocol error"
	return strings.Join([]string{msg, "\n"}, "")
}
func (self *Printer) _connect(eventtime interface{}) interface{} {
	self.tryCatchConnect1()
	self.tryCatchConnect2()
	return nil
}
func (self *Printer) tryCatchConnect1() {
	defer func() {
		if err := recover(); err != nil {
			_, ok1 := err.(*PinError)
			_, ok2 := err.(*Config_error)
			if ok1 || ok2 {
				logger.Error("Config error", err, string(debug.Stack()))
				self._set_state(fmt.Sprintf("%s\n%s", err, message_restart))
				return
			}
			_, ok11 := err.(*MsgprotoError)
			if ok11 {
				logger.Error("Protocol error", string(debug.Stack()))
				self._set_state(self._build_protocol_error_message(err))
				util.Dump_mcu_build()
				return
			}
			e, ok22 := err.(*erro)
			if ok22 {
				logger.Error("MCU error during connect", string(debug.Stack()))
				self._set_state(fmt.Sprintf("%s%s", e.err, message_mcu_connect_error))
				util.Dump_mcu_build()
				return
			}
			logger.Errorf("Unhandled exception during connect: %v, debug stack:\n%s\n", err, string(debug.Stack()))
			self._set_state(fmt.Sprintf("Internal error during connect: %s\n%s", err, message_restart))
			panic(err)
		}
	}()

	self._read_config()
	self.Send_event("project:mcu_identify", nil)
	cbs := self.event_handlers["project:connect"]
	for _, cb := range cbs {
		if self.state_message != message_startup {
			return
		}
		err := cb(nil)
		if err != nil {
			logger.Error("Config error: ", err)
			self._set_state(fmt.Sprintf("%s\n%s", err.Error(), message_restart))
			return
		}
	}
}
func (self *Printer) tryCatchConnect2() {
	defer func() {
		if err := recover(); err != nil {
			logger.Error("Unhandled exception during ready callback", err)
			self.Invoke_shutdown(fmt.Sprintf("Internal error during ready callback: %s", err))
			return
		}
	}()
	self._set_state(message_ready)
	cbs := self.event_handlers["project:ready"]
	for _, cb := range cbs {
		if self.state_message != message_ready {
			return
		}
		err := cb(nil)
		if err != nil {
			logger.Error("Unhandled exception during ready callback")
			self._set_state(fmt.Sprintf("Internal error during ready callback:%s", err.Error()))
			return
		}
	}
}
func (self *Printer) Run() string {
	systime := float64(time.Now().UnixNano()) / 1000000000
	monotime := self.reactor.Monotonic()
	logger.Infof("Start printer at %s (%.1f %.1f)",
		time.Now().String(), systime, monotime)
	// Enter main reactor loop
	err := self.reactor.Run()
	if err != nil {
		msg := "Unhandled exception during run"
		logger.Error(msg)
		// Exception from a reactor callback - try to shutdown
		self.reactor.Register_callback(self.Invoke_shutdown, constants.NOW)
		err = self.reactor.Run()
		if err != nil {
			logger.Error("Repeat unhandled exception during run")
			//Another exception - try to exit
			self.run_result = "error_exit"
		}
	}
	// Check restart flags
	run_result := self.run_result
	if run_result == "firmware_restart" {
		_, err := self.Send_event("project:firmware_restart", nil)
		if err != nil {
			logger.Error("Unhandled exception during post run")
			return run_result
		}
	}

	_, err = self.Send_event("project:disconnect", nil)
	if err != nil {
		logger.Error("Unhandled exception during post run")
		return run_result
	}
	return run_result

}
func (self *Printer) Set_rollover_info(name, info string, isLog bool) {
	if isLog {
		logger.Debug(info)
	}
}
func (self *Printer) Invoke_shutdown(msg interface{}) interface{} {
	if self.in_shutdown_state {
		return nil
	}
	logger.Errorf("Transition to shutdown state: %s", msg)
	self.in_shutdown_state = true
	self._set_state(strings.Join([]string{msg.(string), message_shutdown}, ""))
	for _, cb := range self.event_handlers["project:shutdown"] {

		err := cb(nil)
		if err != nil {
			logger.Error("Exception during shutdown handler")
		}
		logger.Debug("Reactor garbage collection: ", self.reactor.Get_gc_stats())
	}
	return nil
}

func (self *Printer) invoke_async_shutdown(msg string) {
	_func := func(argv interface{}) interface{} {
		self.Invoke_shutdown(msg)
		return nil
	}
	self.reactor.Register_async_callback(_func, constants.NOW)
}
func (self *Printer) Register_event_handler(event string, callback func([]interface{}) error) {
	list, ok := self.event_handlers[event]
	if !ok {
		list = []func([]interface{}) error{}
	}
	self.event_handlers[event] = append(list, callback)
}
func (self *Printer) Send_event(event string, params []interface{}) ([]interface{}, error) {
	ret := []interface{}{}
	cbs, ok := self.event_handlers[event]
	if ok {
		for i := 0; i < len(cbs); i++ {
			cb := cbs[i]
			ret = append(ret, (cb)(params))
		}
	}
	return ret, nil
}
func (self *Printer) Request_exit(result string) {
	if self.run_result == "" {
		self.run_result = result
	}
	self.reactor.End()
}
func ModuleK3C() {

}

type OptionParser struct {
	Debuginput  string
	Inputtty    string
	Apiserver   string
	Logfile     string
	Verbose     bool
	Debugoutput string
	Dictionary  string
	Import_test bool
	log_level   int
}

type K3C struct {
}

func NewK3C() *K3C {
	K3C := K3C{}

	return &K3C
}

//######################################################################
//# Startup
//######################################################################

func (self K3C) Main() {
	usage := "[options] <your config file>"
	flag.Usage = func() {
		fmt.Println(os.Args[0], usage)
	}
	options := OptionParser{}
	flag.StringVar(&options.Debuginput, "i", "", "read commands from file instead of from tty port")
	flag.StringVar(&options.Inputtty, "I", "/tmp/printer", "input tty name (default is /tmp/printer)")
	flag.StringVar(&options.Apiserver, "a", "/tmp/unix_uds", "api server unix domain socket filename")
	flag.StringVar(&options.Logfile, "l", "/tmp/gklib.log", "write log to file instead of stderr")
	flag.BoolVar(&options.Verbose, "v", true, "enable debug messages")
	flag.StringVar(&options.Debugoutput, "o", "", "write output to file instead of to serial port")
	flag.StringVar(&options.Dictionary, "d", "", "file to read for mcu protocol dictionary")
	flag.BoolVar(&options.Import_test, "import-test", false, "perform an import module test")
	flag.IntVar(&options.log_level, "ll", int(logger.DebugLevel), "set logger level")

	flag.Parse()
	args := flag.Args()
	if options.Import_test {
		import_test()
	}
	start_args := map[string]interface{}{}
	if len(args) != 1 {
		flag.Usage()
		start_args["config_file"] = "./printer.cfg"
	} else {
		start_args["config_file"] = args[0]
	}

	start_args["apiserver"] = options.Apiserver
	start_args["start_reason"] = "startup"

	if options.Debuginput != "" {
		start_args["debuginput"] = options.Debuginput
		debuginput, err := os.OpenFile(options.Debuginput, os.O_RDONLY, 0644)
		if err != nil {
			logger.Error(err.Error())
			os.Exit(3)
		}
		//debuginput =io. open(options.debuginput, 'rb')
		start_args["gcode_fd"] = debuginput.Fd()
	} else {
		//start_args["gcode_fd"] = util.create_pty(options.Inputtty)
		//debuginput, err := os.OpenFile(options.Inputtty, os.O_SYNC, 0644)
		//if err != nil {
		//	logger.Error(err.Error())
		//	os.Exit(3)
		//}
		//start_args["gcode_fd"] = debuginput.Fd()
	}
	if options.Debugoutput != "" {
		start_args["debugoutput"] = options.Debugoutput
	}

	// init logger
	debuglevel := logger.DebugLevel
	if options.Logfile != "" {
		start_args["log_file"] = options.Logfile
		if options.log_level > 0 {
			debuglevel = logger.LogLevel(options.log_level)
		}
		logger.InitLogger(debuglevel,
			options.Logfile,
			logger.SUPPORT_COLOR,
			2,
			2,
			7,
		)
	}

	logger.Info("Starting K3C...")

	start_args["software_version"] = sys.GetSoftwareVersion()
	start_args["cpu_info"] = sys.GetCpuInfo()

	logger.Infof("Args: %s, Git version: %s, CPU: %s",
		strings.Join(flag.Args(), ""),
		start_args["software_version"].(string),
		start_args["cpu_info"].(string))

	runtime.GC()

	// Start Printer() class
	var res string
	for {
		main_reactor := NewEPollReactor(true)
		printer := NewPrinter(main_reactor, start_args)
		res = printer.Run()
		if res == "exit" || res == "error_exit" {
			break
		}
		time.Sleep(time.Second)
		logger.Info("Restarting printer")
		start_args["start_reason"] = res
	}
	if res == "error_exit" {
		os.Exit(-1)
	}
	os.Exit(0)
}
func import_test() {

	os.Exit(0)
}
