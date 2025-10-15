package project

import (
	"encoding/json"
	"fmt"
	"k3c/common/jinja2"
	"k3c/common/logger"
	"k3c/common/utils/LiteralEval"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"reflect"
	"strings"
)

const (
	cmd_SET_GCODE_VARIABLE_help = "Set the value of a G-Code macro variable"
)

type GetStatusWrapper struct {
	printer   *Printer
	eventtime float64
	cache     map[string]interface{}
}

func NewGetStatusWrapper(printer *Printer, eventtime float64) *GetStatusWrapper {
	self := new(GetStatusWrapper)

	self.printer = printer
	self.eventtime = eventtime
	self.cache = make(map[string]interface{})
	return self
}

func (self *GetStatusWrapper) __getitem__(val string) (interface{}, error) {
	sval := strings.TrimSpace(val)
	if _, ok := self.cache[sval]; ok {
		return self.cache[sval], nil
	}

	po := self.printer.Lookup_object(sval, nil)
	if po == nil {
		//raise KeyError(val)
		panic(val)
	}
	if self.eventtime == 0 {
		self.eventtime = self.printer.Get_reactor().Monotonic()
	}
	if reflect.ValueOf(po).MethodByName("Get_status").IsNil() {
		return nil, fmt.Errorf("GetStatusWrapper %#v not method Get_status", po)
	}
	res := reflect.ValueOf(po).MethodByName("Get_status").Call([]reflect.Value{
		reflect.ValueOf(self.eventtime),
	})

	var ret = make(map[string]interface{})
	if len(res) >= 1 && res[0].Type().Kind() == reflect.Map { // assert res[0] is map[string]float64 map[string]interface{}
		for _, key := range res[0].MapKeys() {
			ret[key.String()] = res[0].MapIndex(key).Interface()
			self.cache[key.String()] = ret[key.String()]
		}
	}

	return ret[sval], nil
}

func (self *GetStatusWrapper) __contains__(val string) bool {
	_, err := self.__getitem__(val)
	if err == nil {
		return true
	}
	return false
}

func (self *GetStatusWrapper) Iter() <-chan string {
	ch := make(chan string)
	go func() {
		defer close(ch)
		for _, obj := range self.printer.Lookup_objects("") {
			for name, _ := range obj.(map[string]interface{}) {
				if self.__contains__(name) {
					ch <- name
				}
			}
		}
	}()
	return ch
}

type TemplateWrapper struct {
	printer                 *Printer
	name                    string
	gcode                   *GCodeDispatch
	create_template_context func(interface{}) interface{}
	template                *jinja2.Template
}

func NewTemplateWrapper(printer *Printer, env *jinja2.Environment, name, script string) *TemplateWrapper {
	self := new(TemplateWrapper)
	self.printer = printer
	self.name = name
	obj := self.printer.Lookup_object("gcode", object.Sentinel{})
	self.gcode = obj.(*GCodeDispatch)

	obj = self.printer.Lookup_object("gcode_macro_1", object.Sentinel{})
	gcode_macro := obj.(*PrinterGCodeMacro)
	self.create_template_context = gcode_macro.Create_template_context

	var err error
	self.template, err = env.From_string(script)
	if err != nil {
		msg := fmt.Sprintf("Error loading template '%s': %s", name, err)
		logger.Error(msg)
		panic(msg)
	}
	return self
}

func (self *TemplateWrapper) Render(context map[string]interface{}) (string, error) {
	if context == nil {
		ctx := self.create_template_context(nil)

		context, _ = ctx.(map[string]interface{})
	}
	return self.template.Render(context)
}

func (self *TemplateWrapper) Run_gcode_from_command(context map[string]interface{}) error {
	content, err := self.Render(context)
	if err != nil {
		return err
	}
	content = strings.ReplaceAll(content, "    ", "\n")
	self.gcode.Run_script_from_command(content)
	return nil
}

type PrinterGCodeMacro struct {
	printer *Printer
	env     *jinja2.Environment
}

func NewPrinterGCodeMacro(config *ConfigWrapper) *PrinterGCodeMacro {
	self := new(PrinterGCodeMacro)
	self.printer = config.Get_printer()
	self.env = jinja2.NewEnvironment()
	return self
}

func (self *PrinterGCodeMacro) Load_template(config *ConfigWrapper, option, def string) *TemplateWrapper {
	name := fmt.Sprintf("%s:%s", config.Get_name(), option)

	var script string
	if value.IsNone(def) {
		script = cast.ToString(config.Get(option, object.Sentinel{}, true))
	} else {
		script = cast.ToString(config.Get(option, def, true))
	}
	return NewTemplateWrapper(self.printer, self.env, name, script)
}

func (self *PrinterGCodeMacro) _action_emergency_stop(arg interface{}, _ interface{}) interface{} {
	msg := cast.ToString(arg)
	if msg == "" {
		msg = "action_emergency_stop"
	}
	self.printer.Invoke_shutdown(fmt.Sprintf("Shutdown due to %s", (msg)))
	return ""
}

func (self *PrinterGCodeMacro) _action_respond_info(arg interface{}, _ interface{}) interface{} {
	msg := cast.ToString(arg)
	obj := self.printer.Lookup_object("gcode", object.Sentinel{})
	obj.(*GCodeDispatch).Respond_info(msg, true)
	return ""
}

func (self *PrinterGCodeMacro) _action_raise_error(arg interface{}, _ interface{}) interface{} {
	return fmt.Errorf("_action_raise_error: %v", arg)
}

func (self *PrinterGCodeMacro) _action_call_remote_method(arg interface{}, kwargs interface{}) interface{} {
	obj := self.printer.Lookup_object("webhooks", object.Sentinel{})
	webhooks := obj.(*WebHooks)

	method := cast.ToString(arg)
	err := webhooks.Call_remote_method(method, kwargs)
	if err != nil {
		logger.Errorf("Remote Call Error, method: %s, error: %v", method, err)
		return err
	}
	return ""
}

func (self *PrinterGCodeMacro) Create_template_context(arg interface{}) interface{} {
	return map[string]interface{}{
		"printer":                   NewGetStatusWrapper(self.printer, cast.ToFloat64(arg)),
		"action_emergency_stop":     self._action_emergency_stop,
		"action_respond_info":       self._action_respond_info,
		"action_raise_error":        self._action_raise_error,
		"action_call_remote_method": self._action_call_remote_method,
	}
}

func Load_config_printer_gcode_macro(config *ConfigWrapper) interface{} {
	return NewPrinterGCodeMacro(config)
}

type GCodeMacro struct {
	alias           string
	printer         *Printer
	template        *TemplateWrapper
	gcode           *GCodeDispatch
	rename_existing string
	cmd_desc        string
	in_script       bool
	variables       map[string]interface{}
}

func NewGCodeMacro(config *ConfigWrapper) *GCodeMacro {
	self := new(GCodeMacro)
	names := strings.Split(config.Get_name(), " ")
	if len(names) > 2 {
		panic(fmt.Errorf("Name of section '%v' contains illegal whitespace", config.Get_name()))
	}

	name := names[1]
	self.alias = strings.ToUpper(name)
	self.printer = config.Get_printer()
	printerGCodeMacro := self.printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	self.template = printerGCodeMacro.Load_template(config, "gcode", value.StringNone)

	gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	self.gcode = gcode
	self.rename_existing = cast.ToString(config.Get("rename_existing", value.None, true))
	self.cmd_desc = cast.ToString(config.Get("description", "G-Code macro", true))
	if self.rename_existing != "" {
		if self.gcode.Is_traditional_gcode(self.alias) != self.gcode.Is_traditional_gcode(self.rename_existing) {
			panic(fmt.Errorf("G-Code macro rename of different types ('%s' vs '%s')",
				self.alias, self.rename_existing))
		}

		self.printer.Register_event_handler("project:connect", self.handle_connect)
	} else {
		self.gcode.Register_command(self.alias, self.cmd, false, self.cmd_desc)
	}

	self.gcode.Register_mux_command("SET_GCODE_VARIABLE", "MACRO",
		name, self.cmd_SET_GCODE_VARIABLE,
		cmd_SET_GCODE_VARIABLE_help)

	self.in_script = false
	self.variables = make(map[string]interface{})

	prefix := "variable_"
	for _, option := range config.Get_prefix_options(prefix) {
		self.variables[option[len(prefix):]] = config.Get(option, object.Sentinel{}, true)
	}

	return self

}

func (self *GCodeMacro) handle_connect([]interface{}) error {
	prev_cmd := self.gcode.Register_command(self.alias, nil, false, "")
	if prev_cmd == nil {
		return fmt.Errorf("Existing command '%s' not found in gcode_macro rename", self.alias)
	}

	pdesc := fmt.Sprintf("Renamed builtin of '%s'", self.alias)
	self.gcode.Register_command(self.rename_existing, prev_cmd, false, pdesc)
	self.gcode.Register_command(self.alias, self.cmd, false, self.cmd_desc)
	return nil
}

func (self *GCodeMacro) Get_status(eventtime float64) map[string]interface{} {
	return sys.DeepCopyMap(self.variables)
}

func (self *GCodeMacro) cmd_SET_GCODE_VARIABLE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	variable := gcmd.Get("VARIABLE", object.Sentinel{}, "", nil, nil, nil, nil)
	value := gcmd.Get("VALUE", object.Sentinel{}, "", nil, nil, nil, nil)
	if _, ok := self.variables[variable]; !ok {
		return fmt.Errorf("Unknown gcode_macro variable '%s'", variable)
	}

	parsed, err := LiteralEval.LiteralEval(value)
	if err != nil {
		return fmt.Errorf("Unable to parse '%s' as a literal: %s", value, err)
	}

	jsonBytes, err := json.Marshal(parsed)
	if err != nil {
		return fmt.Errorf("Unable to parse '%s' as a literal: %s", value, err)
	}
	self.variables[variable] = jsonBytes
	return nil
}

func (self *GCodeMacro) cmd(argv interface{}) error {
	if self.in_script {
		return fmt.Errorf("Macro %s called recursively", self.alias)
	}

	kwparams := make(map[string]interface{})
	for k, v := range self.variables {
		kwparams[k] = v
	}

	for k, v := range self.template.create_template_context(nil).(map[string]interface{}) {
		kwparams[k] = v
	}
	gcmd := argv.(*GCodeCommand)
	kwparams["params"] = gcmd.Get_command_parameters()
	kwparams["rawparams"] = gcmd.Get_raw_command_parameters()
	self.in_script = true
	defer func() {
		self.in_script = false
	}() // printer.printer.objects.leviQ3.extru_temp
	err := self.template.Run_gcode_from_command(kwparams)
	if err != nil {
		logger.Errorf("GCodeMacro  Run_gcode_from_command error: %v\n", err)
	}

	return nil
}

func Load_config_gcode_macro(config *ConfigWrapper) interface{} {
	return NewGCodeMacro(config)
}
