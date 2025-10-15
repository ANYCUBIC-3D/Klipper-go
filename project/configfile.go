package project

import (
	"bytes"
	"errors"
	"fmt"
	"io/ioutil"
	"k3c/common/configparser"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/file"
	"k3c/common/utils/object"
	"k3c/common/value"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strconv"
	"time"

	"regexp"
	"strings"
)

const (
	AUTOSAVE_HEADER = `
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
`
)

type Config_error struct {
	E string
}
type ConfigWrapper struct {
	printer         *Printer
	fileconfig      *configparser.RawConfigParser
	access_tracking map[string]interface{}
	Section         string
}

func NewConfigWrapper(printer *Printer, fileconfig *configparser.RawConfigParser, access_tracking map[string]interface{}, section string) *ConfigWrapper {
	self := ConfigWrapper{}
	self.printer = printer
	self.fileconfig = fileconfig
	self.access_tracking = access_tracking
	self.Section = section

	return &self
}

func (self *ConfigWrapper) Fileconfig() *configparser.RawConfigParser {
	return self.fileconfig
}
func (self *ConfigWrapper) Get_printer() *Printer {
	return self.printer
}
func (self *ConfigWrapper) Get_name() string {
	return self.Section
}

func (self *ConfigWrapper) Get(option string, default1 interface{}, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Get(self.Section, option)
	if note_valid {
		key := fmt.Sprintf("%s:%s", strings.ToLower(self.Section), strings.ToLower(option))
		self.access_tracking[key] = v
	}
	return v
}

func (self *ConfigWrapper) Getint(option string, default1 interface{}, minval, maxval int,
	note_valid bool) int {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(int)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := v.(int)
	if minval != 0 && n < minval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval))
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval))
	}
	return n
}
func (self *ConfigWrapper) Getint64(option string, default1 interface{}, minval, maxval int64,
	note_valid bool) int64 {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(int64)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := v.(int64)
	if minval != 0 && n < minval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval))
		return minval
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval))
		return maxval
	}
	return n
}

func (self *ConfigWrapper) GetintNone(option string, def interface{}, minval, maxval int,
	note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(def) {
			if note_valid && def != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = def
			}
			return def
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := cast.ToInt(v)
	if minval != 0 && n < minval {
		logger.Errorf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval)
	}
	if maxval != 0 && n > maxval {
		logger.Errorf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval)
	}
	return n
}

func (self *ConfigWrapper) Getfloat(option string, default1 interface{}, minval, maxval,
	above, below float64, note_valid bool) float64 {

	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(float64)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getfloat(self.Section, option)
	n, ok := v.(float64)
	if !ok {
		panic(fmt.Sprintf("Unable to parse option '%s' in section '%s'", option, self.Section))
	}
	if note_valid {
		acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
		self.access_tracking[acc_id] = v
	}
	if minval != 0 && n < minval {
		panic(fmt.Sprintf("Option '%s' in section '%s' must have minimum of %f", option, self.Section, minval))
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Sprintf("Option '%s' in section '%s' must have maximum of %f", option, self.Section, maxval))
	}
	if above != 0 && n <= above {
		panic(fmt.Sprintf("Option '%s' in section '%s' must be above %f", option, self.Section, above))
	}
	if below != 0 && n >= below {
		panic(fmt.Sprintf("Option '%s' in section '%s' must be below %f", option, self.Section, below))
	}

	return n
}

func (self *ConfigWrapper) GetfloatNone(option string, default1 interface{}, minval, maxval,
	above, below float64, note_valid bool) interface{} {

	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getfloat64None(self.Section, option)
	if v == nil {
		return nil
	}
	if note_valid {
		acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
		self.access_tracking[acc_id] = v
	}
	n := cast.ToFloat64(v)

	if minval != 0 && n < minval {
		logger.Errorf("Option '%s' in section '%s' must have minimum of %f", option, self.Section, minval)
	}
	if maxval != 0 && n > maxval {
		logger.Errorf("Option '%s' in section '%s' must have maximum of %f", option, self.Section, maxval)
	}
	if above != 0 && n <= above {
		logger.Errorf("Option '%s' in section '%s' must be above %f", option, self.Section, above)
	}
	if below != 0 && n >= below {
		logger.Errorf("Option '%s' in section '%s' must be below %f", option, self.Section, below)
	}
	return v
}

func (self *ConfigWrapper) Getboolean(option string, default1 interface{}, note_valid bool) bool {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(bool)
			if ok {
				return ret
			} else {
				return false
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getboolean(self.Section, option)
	return v.(bool)
}

func (self *ConfigWrapper) Getchoice(option string, choices map[interface{}]interface{}, default1 interface{}, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(string)
			if ok {
				return ret
			} else {
				return ""
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	var c interface{}
	for k, _ := range choices {
		if reflect.TypeOf(k).Kind() == reflect.Int {
			c = self.Getint(option, default1, 0, 0, true)
		} else {
			c = self.Get(option, default1, true)
		}
	}
	ret, ok := choices[c]
	if !ok {
		logger.Errorf("Choice '%s' for option '%s' in section '%s' is not a valid choice", c, option, self.Section)
	}
	return ret

}
func (self *ConfigWrapper) Getlists(option string, default1 interface{}, seps []string, count int, kind reflect.Kind, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	if len(seps) == 2 && seps[0] == "," && seps[1] == "\n" {
		ret := [][]interface{}{}
		value := self.fileconfig.Get(self.Section, option)
		str, ok := value.(string)
		if ok {
			str = strings.TrimSpace(str)
			//str = strings.ReplaceAll(str, "", "")
			strs := strings.Split(str, "\n")
			for _, s := range strs {
				valArr := strings.Split(s, ",")
				item := []interface{}{}
				for _, valStr := range valArr {
					var val interface{}
					switch kind {
					case reflect.Int:
						val, _ = strconv.Atoi(strings.TrimSpace(valStr))
					case reflect.Float64:
						val, _ = strconv.ParseFloat(strings.TrimSpace(valStr), 64)
					}
					item = append(item, val)
				}
				ret = append(ret, item)
			}
		}
		return ret
	} else {
		ret := []interface{}{}
		value := self.fileconfig.Get(self.Section, option)
		for _, sep := range seps {
			str, ok := value.(string)
			if ok {
				str = strings.TrimSpace(str)
				str = strings.ReplaceAll(str, "\t", "")
				strs := strings.Split(str, sep)
				var val interface{}
				for _, s := range strs {
					switch kind {
					case reflect.Int:
						val, _ = strconv.Atoi(s)
					case reflect.Float64:
						val, _ = strconv.ParseFloat(s, 64)
					}
					ret = append(ret, val)
				}
			}
		}
		return ret
	}
}

func fcparser(section, option string) {
	//	return lparser(self.fileconfig.get(section, option), len(seps)-1)
	//}
	//return self._get_wrapper(fcparser, option, default,note_valid = note_valid)
}
func (self *ConfigWrapper) Getlist(option string, default1 interface{}, sep string, count int, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []interface{}{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			ret = append(ret, s)
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getintlist(option string, default1 interface{}, sep string, count int,
	note_valid bool) []int {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.([]int)
			if ok {
				return ret
			} else {
				return nil
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []int{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			retI, err := strconv.Atoi(strings.TrimSpace(s))
			if err != nil {
				logger.Error(err.Error())
			} else {
				ret = append(ret, retI)
			}
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getfloatlist(option string, default1 interface{}, sep string, count int,
	note_valid bool) []float64 {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.([]float64)
			if ok {
				return ret
			} else {
				return nil
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []float64{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			retI, err := strconv.ParseFloat(strings.TrimSpace(s), 64)
			if err != nil {
				logger.Error(err.Error())
			} else {
				ret = append(ret, retI)
			}
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getsection(section string) *ConfigWrapper {
	return &ConfigWrapper{printer: self.printer, fileconfig: self.fileconfig,
		access_tracking: self.access_tracking, Section: section}
}
func (self *ConfigWrapper) Has_section(section string) bool {

	return self.fileconfig.Has_section(section)
}
func (self *ConfigWrapper) Get_prefix_sections(prefix string) []*ConfigWrapper {
	configs := []*ConfigWrapper{}
	for _, s := range self.fileconfig.Sections() {
		if strings.HasPrefix(s, prefix) {
			configs = append(configs, self.Getsection(s))
		}
	}
	return configs
}

func (self *ConfigWrapper) Get_prefix_options(prefix string) []string {
	options, _ := self.fileconfig.Options(self.Section)
	prefixOpts := []string{}

	for o := range options {
		if prefix == "variable_" {
			if strings.HasPrefix(o, prefix) {
				prefixOpts = append(prefixOpts, o)
			}
		} else {

		}
	}

	return prefixOpts
}

func (self *ConfigWrapper) Deprecate(option, value string) {
	if !self.fileconfig.Has_option(self.Section, option) {
		return
	}
	msg := ""
	if value == "" {
		msg = fmt.Sprintf("Option '%s' in section '%s' is deprecated.", option, self.Section)
	} else {
		msg = fmt.Sprintf("Value '%s' in option '%s' in section '%s' is deprecated.", value, option, self.Section)
	}
	pconfig := self.printer.Lookup_object("configfile", object.Sentinel{}).(*PrinterConfig)
	pconfig.Deprecate(self.Section, option, value, msg)
}

type PrinterConfig struct {
	printer             *Printer
	autosave            *ConfigWrapper
	deprecated          map[string]interface{}
	status_raw_config   map[string]interface{}
	status_save_pending map[string]interface{}
	status_settings     map[string]interface{}

	status_warnings     []interface{}
	save_config_pending bool
	comment_r           *regexp.Regexp
	value_r             *regexp.Regexp
}

func NewPrinterConfig(printer *Printer) *PrinterConfig {
	self := PrinterConfig{}
	self.comment_r = regexp.MustCompile("[#;].*$")
	self.value_r = regexp.MustCompile("[^A-Za-z0-9_].*$")
	self.printer = printer
	self.autosave = nil
	self.deprecated = map[string]interface{}{}
	self.status_raw_config = map[string]interface{}{}
	self.status_save_pending = map[string]interface{}{}

	self.status_settings = map[string]interface{}{}

	self.status_warnings = []interface{}{}
	self.save_config_pending = false
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG, false, cmd_SAVE_CONFIG_help)
	return &self
}
func (self *PrinterConfig) Get_printer() *Printer {
	return self.printer
}
func (self *PrinterConfig) _read_config_file(filename string) (string, error) {
	bs, err := file.GetBytes(filename)
	if err != nil {
		logger.Errorf("Unable to open config file %s", filename)
		return "", err
	}
	data := string(bs)
	return strings.ReplaceAll(data, "\r\n", "\n"), nil
}

func (self *PrinterConfig) _find_autosave_data(data string) (string, string) {
	regular_data := data
	autosave_data := ""
	pos := strings.Index(data, AUTOSAVE_HEADER)
	if pos >= 0 {
		regular_data = data[:pos]
		autosave_data = strings.TrimSpace(data[pos+len(AUTOSAVE_HEADER):])
	}
	// Check for errors and strip line prefixes
	if strings.Index(regular_data, "\n#*# ") != -1 {
		logger.Debug("Can't read autosave from config file - autosave state corrupted")
		return data, ""
	}
	out := []string{""}
	lines := strings.Split(autosave_data, "\n")
	for _, line := range lines {

		if (!strings.HasPrefix(line, "#*#") ||
			(len(line) >= 4 && !strings.HasPrefix(line, "#*# "))) && autosave_data == "" {

			logger.Warn("Can't read autosave from config file - modifications after header")
			return data, ""
		}
		if len(line) > 4 {
			out = append(out, line[4:])
		} else {
			out = append(out, "")
		}
	}
	out = append(out, "")
	return regular_data, strings.Join(out, "\n")
}

func (self *PrinterConfig) _strip_duplicates(data string, config *ConfigWrapper) string {
	fileconfig := config.Fileconfig()
	//Comment out fields in 'data' that are defined in 'config'
	lines := strings.Split(data, "\n")
	section := ""
	is_dup_field := false
	for lineno, line := range lines {
		//logger.Error(lineno, line)
		pruned_line := strings.Split(line, "#")[0]
		if pruned_line == "" {
			continue
		}
		if pruned_line[0] == ' ' || pruned_line[0] == '\t' {
			if is_dup_field {
				lines[lineno] = "#" + line
			}
			continue
		}
		is_dup_field = false
		pruned_line = strings.TrimSpace(pruned_line)
		if pruned_line[0] == '[' {
			section = pruned_line[1 : len(pruned_line)-1]
			continue
		}
		field := strings.TrimSpace(strings.Split(pruned_line, "=")[0])
		if fileconfig.Has_option(section, field) {
			is_dup_field = true
			lines[lineno] = "#" + line
		}
	}
	return strings.Join(lines, "\n")
}

func (self *PrinterConfig) _parse_config_buffer(buffer []string, filename string, fileconfig *configparser.RawConfigParser) {
	if buffer == nil || len(buffer) == 0 {
		return
	}
	fileconfig.Readfp(strings.NewReader(strings.Join(buffer, "\n")), filename)
}

func (self *PrinterConfig) _resolve_include(source_filename, include_spec string, fileconfig *configparser.RawConfigParser, visited map[string]string) ([]string, error) {
	dirname := filepath.Dir(source_filename)
	include_spec = strings.TrimSpace(include_spec)
	include_glob := filepath.Join(dirname, include_spec)

	hasMagic := strings.ContainsAny(include_glob, "*?[]")

	matches, err := filepath.Glob(include_glob)
	if err != nil {
		return nil, fmt.Errorf("glob error: %v", err)
	}

	if len(matches) == 0 && !hasMagic {
		//# Empty set is OK if wildcard but not for direct file reference
		panic(fmt.Sprintf("include file '%s' does not exist", include_glob))
	}

	sort.Strings(matches)

	for _, includeFilename := range matches {
		data, err := self._read_config_file(includeFilename)
		if err != nil {
			return nil, err
		}
		self._parse_config(data, includeFilename, fileconfig, visited)
	}

	return matches, nil
}

func (self *PrinterConfig) _parse_config(data, filename string, fileconfig *configparser.RawConfigParser, visited map[string]string) {
	path, err := filepath.Abs(filename)
	if err != nil {
		logger.Error(err.Error())
	}
	_, ok := visited[path]
	if ok {
		panic(fmt.Sprintf("Recursive include of config file '%s'", filename))
	}
	visited[path] = ""
	lines := strings.Split(data, "\n")
	//	 Buffer lines between includes and parse as a unit so that overrides
	//	 in includes apply linearly as they do within a single file
	buffer := []string{}
	for _, line := range lines {
		// Strip trailing comment
		pos := strings.Index(line, "#")
		if pos >= 0 {
			line = line[:pos]
		}
		// Process include or buffer line
		header := line
		if len(header) != 0 && strings.HasPrefix(header, "include ") {
			self._parse_config_buffer(buffer, filename, fileconfig)
			include_spec := strings.TrimSpace(header[8:])
			self._resolve_include(filename, include_spec, fileconfig, visited)
		} else {
			buffer = append(buffer, line)
		}
	}
	self._parse_config_buffer(buffer, filename, fileconfig)
	delete(visited, path)
}

func (self *PrinterConfig) _build_config_wrapper(data, filename string) *ConfigWrapper {
	fileconfig := configparser.NewRawConfigParser()
	self._parse_config(data, filename, fileconfig, map[string]string{})
	return NewConfigWrapper(self.printer, fileconfig, map[string]interface{}{}, "printer")
}

func (self *PrinterConfig) _build_config_string(config *ConfigWrapper) string {
	buf := bytes.NewBuffer(nil)
	config.fileconfig.Write(buf)
	return strings.TrimSpace(buf.String())
}

func (self *PrinterConfig) Read_config(filename string) *ConfigWrapper {
	data, _ := self._read_config_file(filename)
	return self._build_config_wrapper(data,
		filename)
}
func (self *PrinterConfig) Read_main_config() *ConfigWrapper {
	filename := self.printer.Get_start_args()["config_file"]
	data, err := self._read_config_file(filename.(string))
	if err != nil {
		panic("read config _read_config_file: " + err.Error())
	}
	regular_data, autosave_data := self._find_autosave_data(data)
	regular_config := self._build_config_wrapper(regular_data, filename.(string))
	autosave_data = self._strip_duplicates(autosave_data, regular_config)
	self.autosave = self._build_config_wrapper(autosave_data, filename.(string))
	cfg := self._build_config_wrapper(regular_data+"\n"+autosave_data, filename.(string))
	return cfg
}

func (self *PrinterConfig) Check_unused_options(config *ConfigWrapper) {
	/*fileconfig := config.fileconfig
	objects := self.printer.Lookup_objects("")
	// Determine all the fields that have been accessed
	access_tracking := config.access_tracking
	for _, section := range self.autosave.fileconfig.Sections() {
		options, err := self.autosave.fileconfig.Options(section)
		if err != nil {
			panic(err)
		}
		for _, option := range options {
			key := fmt.Sprintf("%s:%s", strings.ToLower(section), strings.ToLower(option))
			access_tracking[key] = 1
		}
	}
	// Validate that there are no undefined parameters in the config file
	valid_sections := make(map[string]int)
	for s, _ := range access_tracking {
		valid_sections[s] = 1
	}
	for _, section_name := range fileconfig.Sections() {
		section := strings.ToLower(section_name)
		_, ok := valid_sections[section]
		ok1 := false
		for _, item := range objects {
			val := item.(map[string]interface{})
			if _, ok2 := val[section]; ok2 {
				ok1 = true
				break
			}
		}
		if !ok && !ok1 {
			panic(fmt.Sprintf("Section '%s' is not a valid config section", section))
		}
		options, err := fileconfig.Options(section_name)
		if err != nil {
			panic(err)
		}
		for _, option := range options {
			option = strings.ToLower(option)
			key := fmt.Sprintf("%s:%s", section, option)
			if _, ok3 := access_tracking[key]; !ok3 {
				panic(fmt.Sprintf("Option '%s' is not valid in section '%s'", option, section))
			}
		}
	}*/
	// Setup get_status()
	self._build_status(config)
}
func (self *PrinterConfig) Log_config(config *ConfigWrapper) {
	lines := []string{"===== Config file =====",
		self._build_config_string(config),
		"======================="}
	self.printer.Set_rollover_info("config", strings.Join(lines, "\n"), true)
}

// // Status reporting
func (self *PrinterConfig) Deprecate(section, option, value, msg string) {
	key := fmt.Sprintf("%s:%s:%s", section, option, value)
	self.deprecated[key] = msg
}

func (self *PrinterConfig) _build_status(config *ConfigWrapper) {
	self.status_raw_config = make(map[string]interface{})
	for _, section := range config.Get_prefix_sections("") {
		section_status := make(map[string]interface{})
		self.status_raw_config[section.Get_name()] = section_status
		for _, option := range section.Get_prefix_options("") {
			section_status[option] = section.Get(option, object.Sentinel{}, false)
		}
	}
	self.status_settings = make(map[string]interface{})
	for k, val := range config.access_tracking {
		section := strings.Split(k, ":")[0]
		option := strings.Split(k, ":")[1]
		item, ok := self.status_settings[section]
		if ok {
			item.(map[string]interface{})[option] = val
		} else {
			item = make(map[string]interface{})
			item.(map[string]interface{})[option] = val
			self.status_settings[section] = item
		}
	}
	self.status_warnings = []interface{}{}
	for k, msg := range self.deprecated {
		section := strings.Split(k, ":")[0]
		option := strings.Split(k, ":")[1]
		val := strings.Split(k, ":")[2]
		res := make(map[string]string)
		if val == "" {
			res["type"] = "deprecated_option"
		} else {
			res["type"] = "deprecated_value"
			res["value"] = "value"
		}
		res["message"] = msg.(string)
		res["section"] = section
		res["option"] = option
		self.status_warnings = append(self.status_warnings, res)
	}
}

func (self *PrinterConfig) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"config":                    self.status_raw_config,
		"settings":                  self.status_settings,
		"warnings":                  self.status_warnings,
		"save_config_pending":       self.save_config_pending,
		"save_config_pending_items": self.status_save_pending,
	}
}

// Set Autosave functions
func (self *PrinterConfig) Set(section, option, val string) {
	if !self.autosave.fileconfig.Has_section(section) {
		self.autosave.fileconfig.Add_section(section)
	}

	self.autosave.fileconfig.Set(section, option, val)
	pending := self.status_save_pending
	if _, ok := pending[section]; !ok || value.IsNone(pending[section]) {
		pending[section] = make(map[string]interface{})
	}

	opts := pending[section].(map[string]interface{})
	opts[option] = val
	pending[section] = opts
	self.status_save_pending = pending
	self.save_config_pending = true
	logger.Infof("save_config: set [%s] %s = %s", section, option, val)
}

func (self *PrinterConfig) Remove_section(section string) {
	if self.autosave.fileconfig.Has_section(section) {
		self.autosave.fileconfig.Remove_section(section)
		delete(self.status_save_pending, section)
		self.save_config_pending = true
	} else if _, ok := self.status_save_pending[section]; ok {
		delete(self.status_save_pending, section)
		self.save_config_pending = true
	}
}

func (self *PrinterConfig) _disallow_include_conflicts(regular_data, cfgname string, gcode interface{}) {
	config := self._build_config_wrapper(regular_data, cfgname)
	for _, section := range self.autosave.fileconfig.Sections() {
		options, err := self.autosave.fileconfig.Options(section)
		if err != nil {
			logger.Error(err.Error())
			continue
		}
		for _, option := range options {
			if config.fileconfig.Has_option(section, option) {
				msg := fmt.Sprintf("SAVE_CONFIG section '%s' option '%s' conflicts with included value", section, option)
				panic(errors.New(msg))
			}
		}
	}
}

const cmd_SAVE_CONFIG_help = "Overwrite config file and restart"

func (self *PrinterConfig) cmd_SAVE_CONFIG(argv interface{}) error {
	if self.autosave.fileconfig.Sections() == nil {
		return nil
	}
	gcode := MustLookupGcode(self.printer)
	// Create string containing autosave data
	autosave_data := self._build_config_string(self.autosave)
	lines := strings.Split(autosave_data, "\n")
	processedLines := make([]string, 0, len(lines)+2)
	for _, line := range lines {
		processedLine := strings.TrimSpace("#*# " + line)
		processedLines = append(processedLines, processedLine)
	}
	header := strings.TrimRight(AUTOSAVE_HEADER, "\r\n\t ") + "\n"
	processedLines = append([]string{header}, processedLines...)
	processedLines = append(processedLines, "")
	autosave_data = strings.Join(processedLines, "\n")

	//l := strings.Split(autosave_data,  "\n")
	//for i := 0; i < len(l); i++ {
	//	l[i] = strings.Join([]string{"#*# ", l[i]}, "")
	//}
	//autosave_data := strings.Join(l, "\n")

	// Read in and validate current config file
	cfgname := self.printer.Get_start_args()["config_file"].(string)

	data, err := self._read_config_file(cfgname)
	regular_data, _ := self._find_autosave_data(data)
	self._build_config_wrapper(regular_data, cfgname)
	if err != nil {
		msg := "Unable to parse existing config on SAVE_CONFIG"
		logger.Error(msg)
		return errors.New(msg)
	}
	regular_data = self._strip_duplicates(regular_data, self.autosave)
	self._disallow_include_conflicts(regular_data, cfgname, gcode)
	data = strings.TrimSpace(regular_data) + "\n" + autosave_data + "\n"

	// Determine filenames
	datestr := time.Now().Format("-20060102_150405")
	backup_name := cfgname + datestr
	temp_name := cfgname + "_autosave"

	if strings.HasSuffix(cfgname, ".cfg") {
		backup_name = cfgname[:len(cfgname)-4] + datestr + ".cfg"
		temp_name = cfgname[:len(cfgname)-4] + "_autosave.cfg"
	}
	// Create new config file with temporary name and swap with main config
	logger.Infof("SAVE_CONFIG to '%s' (backup in '%s')",
		cfgname, backup_name)
	err1 := ioutil.WriteFile(temp_name, []byte(data), 0666)
	if err1 != nil {
		msg := "Unable to write config file during SAVE_CONFIG"
		logger.Error(msg)
		return err1
	}
	os.Rename(cfgname, backup_name)
	os.Rename(temp_name, cfgname)

	// Request a restart
	gcode.Request_restart("restart")

	return nil
}
