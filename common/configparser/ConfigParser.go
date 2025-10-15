package configparser

import (
	"fmt"
	"io"
	"k3c/common/collections"
	"k3c/common/logger"
	"log"
	"os"
	"regexp"
	"strconv"
	"strings"
	"unicode"
)

const DEFAULTSECT = "DEFAULT"

type RawConfigParser struct {
	//Config     map[string]map[string]string
	Config     *collections.SortedMap
	_sections  []string
	RE_SESSION *regexp.Regexp
	RE_KV      *regexp.Regexp
	_defaults  map[string]string
}

func NewRawConfigParser() *RawConfigParser {
	self := RawConfigParser{}
	self.Config = collections.NewSortedMap()
	self.RE_SESSION = regexp.MustCompile("^\\[.+\\]$")
	self.RE_KV = regexp.MustCompile("^.+\\:.+$")
	return &self
}

const COMMENT_FLAG = "#"

func (self *RawConfigParser) exists(file string) bool {
	_, err := os.Stat(file)
	if err == nil {
		logger.Debug("File exist")
		return true
	}
	if os.IsNotExist(err) {
		logger.Error("File not exist")
		return false
	}
	logger.Error("File error", err.Error())
	return false
}
func (self *RawConfigParser) parse_line(line string) (string, interface{}) {
	comment_index := strings.Index(line, COMMENT_FLAG)
	if comment_index != -1 {
		line = line[0:comment_index]
	}
	if self.RE_SESSION.MatchString(line) {
		return "section", line[1 : len(line)-1]
	}
	if self.RE_KV.MatchString(line) {
		parts := strings.SplitN(line, ":", 2)
		return "kv", map[string]string{strings.TrimSpace(parts[0]): strings.TrimSpace(parts[1])}
	}
	if strings.TrimSpace(line) == "" {
		return "empty", nil
	}
	logger.Errorf("%s format error!", line)
	return "other", nil
}

func (self *RawConfigParser) Options(section string) (map[string]string, error) {
	_config, ok := self.Config.Get(section)
	if ok {
		return _config.(map[string]string), nil
	}
	return nil, fmt.Errorf("section %s not found", section)
}

func (self *RawConfigParser) Has_option(section, option string) bool {
	_sectionMap, ok := self.Config.Get(section)
	if ok {
		_, ok = _sectionMap.(map[string]string)[option]
		return ok

	}
	return false
}

func (self *RawConfigParser) Sections() (sections []string) {
	for _, section := range self.Config.Keys() {
		sections = append(sections, section)
	}
	return sections
}

func (self *RawConfigParser) parse_rc_content(content string) map[string]map[string]string {
	config := make(map[string]map[string]string)
	lines := strings.FieldsFunc(content, func(char rune) bool {
		return strings.ContainsRune("\r\n", char)
	})
	last_section := ""
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		line_type, line_value := self.parse_line(line)
		switch {
		case line_type == "section":
			section := line_value.(string)
			if _, ok := config[section]; ok == false {
				config[section] = make(map[string]string)
			}
			last_section = section
		case line_type == "kv" && last_section != "":
			kv := line_value.(map[string]string)
			for k, v := range kv {
				config[last_section][k] = v
			}
		}
	}
	return config
}

// func (self *RawConfigParser) Init(rc_files ...string) {
// 	for _, rc_file := range rc_files {
// 		if !self.exists(rc_file) {
// 			continue
// 		}
// 		content_bytes, err := ioutil.ReadFile(rc_file)
// 		if err != nil {
// 			log.Fatalln(err)
// 		}
// 		content := string(content_bytes)
// 		for section, configs := range self.parse_rc_content(content) {
// 			_, ok := self.Config[section]
// 			if ok == false {
// 				self.Config[section] = make(map[string]string)
// 			}
// 			for name, value := range configs {
// 				self.Config[section][name] = value
// 			}
// 		}
// 	}
// }

func (self *RawConfigParser) Write(rw io.ReadWriter) io.Reader {
	for section, options := range self.Config.Map() {
		rw.Write([]byte("[" + section + "]\n"))
		for name, val := range options.(map[string]string) {
			if strings.Index(val, "\n") != -1 {
				rw.Write([]byte(name + " = \n"))
				for _, v := range strings.Split(val, "\n") {
					line := strings.Repeat(" ", 4) + v + "\n"
					rw.Write([]byte(line))
				}
			} else {
				rw.Write([]byte(name + " = " + val + "\n"))
			}
		}
	}
	return rw
}
func (self *RawConfigParser) Has_section(section string) bool {
	_, ok := self.Config.Get(section)

	return ok
}

func (self *RawConfigParser) Get(section string, option string) interface{} {
	_sectionMap, ok := self.Config.Get(section)
	if ok {
		str, ok1 := _sectionMap.(map[string]string)[option]
		if ok1 {
			return str
		}
	}
	return ""
}
func (self *RawConfigParser) Getint(section string, option string) interface{} {
	_sectionMap, ok := self.Config.Get(section)
	if ok {
		str, ok := _sectionMap.(map[string]string)[option]
		if ok {
			n, err := strconv.Atoi(str)
			if err != nil {
				logger.Error(err.Error())
			} else {
				return n
			}
		}
	}
	return 0
}
func (self *RawConfigParser) Getfloat(section string, option string) interface{} {
	_sectionMap, ok := self.Config.Get(section)
	if ok {
		str, ok := _sectionMap.(map[string]string)[option]
		if ok {
			n, err := strconv.ParseFloat(str, 64)
			if err != nil {
				logger.Error(err.Error())
			} else {
				return n
			}
		}
	}
	return 0
}

func (self *RawConfigParser) Getfloat64None(section string, option string) interface{} {
	_sectionMap, ok := self.Config.Get(section)
	if !ok {
		return nil
	}

	str, ok := _sectionMap.(map[string]string)[option]
	if !ok {
		return nil
	}
	n, _ := strconv.ParseFloat(str, 64)
	return n
}

func (self *RawConfigParser) Getboolean(section string, option string) interface{} {
	_sectionMap, ok := self.Config.Get(section)
	if ok {
		str, ok := _sectionMap.(map[string]string)[option]
		if ok {
			n, err := strconv.ParseBool(str)
			if err != nil {
				logger.Error(err.Error())
			} else {
				return n
			}
		}
	}
	return 0
}

func (self *RawConfigParser) Set(section, option, value string) {
	var sectdict map[string]string
	if section == "" || section == DEFAULTSECT {
		sectdict = self._defaults
	} else {
		_section, ok := self.Config.Get(section)
		if ok {
			sectdict = _section.(map[string]string)
		}

	}

	sectdict[option] = value
}

func (self *RawConfigParser) Add_section(section string) {

	if section == DEFAULTSECT {
		panic(fmt.Sprintf("Invalid section name: %s", section))
	}

	if ok := self.Config.Has(section); ok {
		panic(fmt.Sprintf("Duplicate section name: %s", section))
	}

	self.Config.Insert(section, make(map[string]string))
}

func (self *RawConfigParser) Remove_section(section string) {
	//delete(self.Config, section)
	self.Config.Delete(section)
}

func (self *RawConfigParser) Readfp(r io.Reader, filename string) {
	if r != nil {
		if _, ok := r.(io.Closer); !ok {
			self.InitFromReader(io.NopCloser(r))
			return
		}
		self.InitFromReader(r.(io.ReadCloser))
		return
	}

	f, err := os.Open(filename)
	if err != nil {
		log.Fatalf("open file: %s, err: %v", filename, err)
		return
	}
	self.InitFromReader(f)
}

func (self *RawConfigParser) InitFromReader(r io.ReadCloser) {
	content_bytes, err := io.ReadAll(r)
	if err != nil {
		log.Fatalln(err)
	}
	defer r.Close()
	content := string(content_bytes)
	sortedMap := self.parseContent(content)
	//for _, section := range sortedMap.Keys() {
	//
	//	configs:=sortedMap.Map()
	//	if ok := self.Config.Has(section); !ok {
	//		//self.Config[section] = make(map[string]string)
	//		self.Config.Insert(section, make(map[string]string))
	//	}
	//
	//	optionVals := self.Config.MustGet(section).(map[string]string)
	//	for name, value := range configs {
	//		optionVals[name] = value
	//	}
	//	self.Config.Insert(section, optionVals)
	//}
	self.Config = sortedMap
}

var (
	commentRegexp = regexp.MustCompile("^\\s*[#|;].*$")
	sectionRegexp = regexp.MustCompile("^\\s*\\[([_A_Za-z].*?)\\]\\s*$")
	optionRegexp  = regexp.MustCompile("((?U)^([_A-Za-z].*)[=:])([^#]*)")
)

func (self *RawConfigParser) parseContent(content string) *collections.SortedMap {

	sorted := []string{}
	config := make(map[string]interface{})
	lines := strings.FieldsFunc(content, func(char rune) bool {
		return strings.ContainsRune("\r\n", char)
	})

	var (
		lastSection string
		lastOption  string
	)
	for _, line := range lines {
		if commentRegexp.MatchString(line) { // comment
			continue
		}

		if len(strings.TrimSpace(line)) == 0 { // empty line
			continue
		}

		if len(lastOption) > 0 {
			if !sectionRegexp.MatchString(line) && !optionRegexp.MatchString(line) {
				var end int
				end = strings.Index(line, ";")
				if end < 0 {
					end = strings.Index(line, "#")
				}
				if end >= 0 {
					line = line[:end]
				}
				if strings.TrimSpace(line) == "" {
					continue
				}
				// Multiple rows of data with at least one space before each row
				if !strings.HasPrefix(line, strings.Repeat(" ", 1)) {
					panic(fmt.Errorf("multi-line option value should has at least two space character: %v, %v,%v", lastSection, lastOption, line))
				}
				// Remove leading and trailing whitespace strings
				line = strings.TrimFunc(line, unicode.IsSpace)
				if len(config[lastSection].(map[string]string)[lastOption]) > 0 {
					config[lastSection].(map[string]string)[lastOption] += "\n" + line
				} else {
					config[lastSection].(map[string]string)[lastOption] = line
				}
				continue
			} else {
				lastOption = ""
			}
		}

		match := sectionRegexp.FindStringSubmatch(line)
		if len(match) == 2 { // section
			lastSection = strings.TrimSpace(match[1])
			if len(lastSection) == 0 {
				logger.Error("empty section")
			}
			if _, ok := config[lastSection]; !ok {
				config[lastSection] = make(map[string]string)
				sorted = append(sorted, lastSection)
			}
			continue
		}

		match = optionRegexp.FindStringSubmatch(line)
		if len(match) != 4 {
			logger.Error("invalid line, it should be option", line)
			continue
		}

		if len(lastSection) == 0 {
			logger.Error("invalid option, not found section", line)
			continue
		}

		option := strings.TrimSpace(match[2])
		val := strings.TrimSpace(match[3])
		if _, ok := config[lastSection].(map[string]string)[option]; !ok {
			config[lastSection].(map[string]string)[option] = val
		} else {
			config[lastSection].(map[string]string)[option] = val
		}

		if len(val) == 0 {
			lastOption = option
		}
	}
	ret := collections.NewSortedMap1(sorted, config)
	return ret
}
func (self *RawConfigParser) parseContent1(content string) map[string]map[string]string {
	config := make(map[string]map[string]string)
	lines := strings.FieldsFunc(content, func(char rune) bool {
		return strings.ContainsRune("\r\n", char)
	})

	var (
		lastSection string
		lastOption  string
	)
	for _, line := range lines {
		if commentRegexp.MatchString(line) { // comment
			continue
		}

		if len(strings.TrimSpace(line)) == 0 { // empty line
			continue
		}

		if len(lastOption) > 0 {
			if !sectionRegexp.MatchString(line) && !optionRegexp.MatchString(line) {
				var end int
				end = strings.Index(line, ";")
				if end < 0 {
					end = strings.Index(line, "#")
				}
				if end > 0 {
					line = strings.TrimRightFunc(line[:end], unicode.IsSpace)
				}
				config[lastSection][lastOption] += line
				continue
			} else {
				lastOption = ""
			}
		}

		match := sectionRegexp.FindStringSubmatch(line)
		if len(match) == 2 { // section
			lastSection = strings.TrimSpace(match[1])
			if len(lastSection) == 0 {
				logger.Error("empty section")
			}
			if _, ok := config[lastSection]; !ok {
				config[lastSection] = make(map[string]string)
			}
			continue
		}

		match = optionRegexp.FindStringSubmatch(line)
		if len(match) != 4 {
			logger.Error("invalid line, it should be option", line)
			continue
		}

		if len(lastSection) == 0 {
			logger.Error("invalid option, not found section", line)
			continue
		}

		option := strings.TrimSpace(match[2])
		val := strings.TrimSpace(match[3])
		if _, ok := config[lastSection][option]; !ok {
			config[lastSection][option] = val
		}

		if len(val) == 0 {
			lastOption = option
		}
	}
	return config
}
