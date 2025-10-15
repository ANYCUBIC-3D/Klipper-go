package project

import (
	"bufio"
	"encoding/json"
	"fmt"
	"k3c/common/ini"
	"k3c/common/logger"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"sync"
)

func expanduser(path string) string {
	if path[:2] == "~/" {
		home, _ := os.UserHomeDir()
		return filepath.Join(home, path[2:])
	}
	return path
}

type SaveVariables struct {
	printer      *Printer
	filename     string
	allVariables map[string]interface{}
	mu           sync.RWMutex
	statusData   map[string]interface{}
}

func NewSaveVariables(config *ConfigWrapper) *SaveVariables {
	self := new(SaveVariables)
	self.printer = config.Get_printer()
	self.filename = expanduser(config.Get("filename", "", true).(string))
	self.allVariables = map[string]interface{}{}

	//try
	func() {
		//catch
		defer func() {
			if err := recover(); err != nil {
				logger.Error(err)
			}
		}()
		if _, err := os.Stat(self.filename); os.IsNotExist(err) {
			file, err := os.Create(self.filename)
			if err != nil {
				panic(err)
			}
			file.Close()
		} else if err != nil {
			panic(err)
		}
	}()

	self.loadVariables()

	gcode := MustLookupGcode(self.printer)
	gcode.Register_command("SAVE_VARIABLE", self.cmd_SAVE_VARIABLE, false,
		"Save arbitrary variables to disk")

	return self
}

func (self *SaveVariables) loadVariables() error {
	cfg, err := ini.Load(self.filename)
	if err != nil {
		return err
	}

	if section, err := cfg.GetSection("Variables"); err == nil {
		for _, key := range section.Keys() {
			value, err := parsePythonLiteral(key.String())
			if err == nil {
				self.allVariables[key.Name()] = value
			}
		}
	}

	return nil
}

func IsFloat(s string) bool {
	if s == "" || strings.HasPrefix(s, "0x") || strings.HasPrefix(s, "0X") {
		return false
	}

	_, err := strconv.ParseFloat(s, 64)
	if err != nil {
		return false
	}

	return strings.ContainsAny(s, ".eE") ||
		strings.EqualFold(s, "inf") ||
		strings.EqualFold(s, "+inf") ||
		strings.EqualFold(s, "-inf") ||
		strings.EqualFold(s, "nan")
}

func parsePythonLiteral(s string) (interface{}, error) {
	switch s {
	case "True", "true":
		return true, nil
	case "False", "false":
		return false, nil
	case "None", "null":
		return nil, nil
	}
	if IsFloat(s) {
		if num, err := strconv.ParseFloat(s, 64); err == nil {
			return num, nil
		}
	} else {
		if num, err := strconv.ParseInt(s, 10, 64); err == nil {
			return num, nil
		}
	}

	if len(s) > 0 {
		switch s[0] {
		case '[', '{', '(':
			var result interface{}
			if err := json.Unmarshal([]byte(ReplaceString(s)), &result); err == nil {
				return result, nil
			}
		case '\'', '"':
			return strings.Trim(s, `'"`), nil
		}
	}

	return s, nil
}

func ReplaceString(s string) string {
	re := regexp.MustCompile(`'(true|false|null)'`)
	s = re.ReplaceAllString(s, `$1`)

	s = strings.ReplaceAll(s, "(", "[")
	s = strings.ReplaceAll(s, ")", "]")
	s = strings.ReplaceAll(s, "'", "\"")
	s = strings.ReplaceAll(s, "True", "true")
	s = strings.ReplaceAll(s, "False", "false")
	s = strings.ReplaceAll(s, "None", "null")

	return s
}

func (self *SaveVariables) cmd_SAVE_VARIABLE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	varname := gcmd.Get("VARIABLE", "", "", nil, nil, nil, nil)
	value := gcmd.Get("VALUE", "", "", nil, nil, nil, nil)
	val, err := parsePythonLiteral(value)
	if err != nil {
		return err
	}

	self.mu.Lock()
	self.allVariables[varname] = val
	self.mu.Unlock()

	cfg := ini.Empty()
	section := cfg.NewSection("Variables")

	self.mu.RLock()
	defer self.mu.RUnlock()
	//# Write file
	for name, value := range self.allVariables {
		valueStr := Literal(value)
		section.NewKey(name, valueStr)
	}

	file, err := os.Create(self.filename)
	if err != nil {
		return fmt.Errorf("Failed to create file: %w", err)
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	writer.WriteString(cfg.IniString())
	writer.Flush()
	return self.loadVariables()
}

func Literal(v interface{}) string {
	switch val := v.(type) {
	case string:
		return `"` + strings.ReplaceAll(val, `"`, `\"`) + `"`
	case bool:
		if val {
			return "true"
		}
		return "false"
	case nil:
		return "null"
	case int, int8, int16, int32, int64, uint, uint8, uint16, uint32, uint64:
		return fmt.Sprintf("%d", val)
	case float32, float64:
		return fmt.Sprintf("%f", val)
	case []interface{}:
		items := make([]string, len(val))
		for i, item := range val {
			items[i] = Literal(item)
		}
		return "[" + strings.Join(items, ",") + "]"
	case map[string]interface{}:
		items := make([]string, 0, len(val))
		for k, v := range val {
			items = append(items, Literal(k)+":"+Literal(v))
		}
		return "{" + strings.Join(items, ",") + "}"
	default:
		return fmt.Sprintf("%v", val)
	}
}

func (self *SaveVariables) Get_status(eventtime float64) map[string]interface{} {
	statusData := make(map[string]interface{})
	statusData["variables"] = self.allVariables
	return statusData
}

func Load_config_save_variables(config *ConfigWrapper) interface{} {
	return NewSaveVariables(config)
}
