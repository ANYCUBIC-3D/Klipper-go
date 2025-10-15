package ini

import (
	"bufio"
	"bytes"
	"errors"
	"fmt"
	"io"
	"os"
	"regexp"
	"strings"
	"unicode"
)

const (
	DEFAULT_SECTION = "default"
)

var multilineReg = regexp.MustCompile(`^([\t\f ]+)(.*)`)

type File struct {
	dataSource  io.ReadCloser
	sections    map[string]*Section
	sectionList []string
}

func Load(source interface{}) (*File, error) {
	s, err := parseSource(source)
	if err != nil {
		return nil, err
	}

	file := &File{
		dataSource: s,
		sections:   make(map[string]*Section),
	}
	if err = file.load(); err != nil {
		return nil, err
	}
	return file, nil
}

func Empty() *File {
	f, _ := Load([]byte{})
	return f
}

func parseSource(source interface{}) (io.ReadCloser, error) {
	switch s := source.(type) {
	case string:
		f, err := os.Open(s)
		if err != nil {
			return nil, err
		}
		return f, nil
	case []byte:
		return io.NopCloser(bytes.NewReader(s)), nil
	case io.ReadCloser:
		return s, nil
	case io.Reader:
		return io.NopCloser(s), nil
	default:
		return nil, fmt.Errorf("Load unknown source: %v", source)
	}
}

func (f *File) load() error {
	var (
		bufr    = bufio.NewReader(f.dataSource)
		name    = ""
		section = &Section{}
		isEOF   = false
	)
	defer f.dataSource.Close()
	for !isEOF {
		line, err := bufr.ReadString('\n')
		if err != nil && err != io.EOF {
			return err
		}

		if err == io.EOF {
			isEOF = true
		}
	single:
		line = strings.TrimLeftFunc(line, unicode.IsSpace)
		if len(line) == 0 {
			continue
		}

		if line[0] == ';' || line[0] == '#' {
			continue
		}

		if line[0] == '[' {
			lastIndex := strings.LastIndex(line, "]")
			if lastIndex < 0 || lastIndex == 1 {
				return fmt.Errorf("anomaly section: %s", line)
			}
			name = strings.TrimSpace(line[1:lastIndex])
			if len(name) == 0 {
				return fmt.Errorf("anomaly section: %s", line)
			}
			section = f.NewSection(name)
			continue
		}

		lastIndex := strings.IndexAny(line, "=:")
		if lastIndex == 0 {
			return fmt.Errorf("section " + section.Name() + "key empty")
		}

		if lastIndex < 0 {
			if i := strings.IndexAny(line, "#;"); i > -1 {
				line = line[:i]
			}
			section.NewKey(line, "")
			continue
		}

		key := line[0:lastIndex]
		val := readValue(line[lastIndex+1:])
		if len(val) == 0 {
			for !isEOF {
			multiline:
				newline, err := bufr.ReadString('\n')
				if err != nil && err != io.EOF {
					return err
				}
				if err == io.EOF {
					isEOF = true
				}

				matches := multilineReg.FindStringSubmatch(newline)
				if len(matches) != 3 {
					line = newline
					section.NewKey(key, val)
					goto single
				}
				multi := readValue(matches[0])
				if len(multi) == 0 || multi[0] == '#' || multi[0] == ';' {
					continue
				}
				if len(val) == 0 {
					val = multi
				} else {
					val += "\n" + multi
				}

				goto multiline
			}
		}
		section.NewKey(key, val)
	}

	return nil
}

func readValue(value string) string {
	if i := strings.IndexAny(value, "#;"); i > -1 {
		value = value[:i]
	}
	return strings.TrimSpace(value)
}

func (f *File) Sections() []*Section {
	sections := make([]*Section, 0, len(f.sectionList))
	for _, name := range f.sectionList {
		sec, _ := f.GetSection(name)
		sections = append(sections, sec)
	}
	return sections
}

func (f *File) GetSection(name string) (*Section, error) {
	sec, ok := f.sections[name]
	if !ok {
		return nil, errors.New(name + " section not found")
	}
	return sec, nil
}

func (f *File) HasSection(name string) bool {
	_, err := f.GetSection(name)
	if err == nil {
		return true
	}
	return false
}

func (f *File) DeleteSection(name string) error {
	if !f.HasSection(name) {
		return errors.New(name + " section not found")
	}

	lists := make([]string, 0, len(f.sectionList)-1)
	for _, s := range f.sectionList {
		if name != s {
			lists = append(lists, s)
		}
	}
	f.sectionList = lists
	delete(f.sections, name)
	return nil
}

func (f *File) SectionString() []string {
	sections := make([]string, len(f.sectionList))
	copy(sections, f.sectionList)
	return sections
}

func (f *File) IniString() string {
	b := f.writeToStringBuilder()
	return b.String()
}

func (f *File) writeToStringBuilder() strings.Builder {
	var buf strings.Builder
	for _, name := range f.sectionList {
		sec, _ := f.GetSection(name)
		buf.WriteString("[" + sec.Name() + "]")
		buf.WriteString("\n")
		for _, kv := range sec.Keys() {
			buf.WriteString(kv.Name() + " = ")
			if len(kv.String()) == 0 { // empty value
				buf.WriteString("\n")
				continue
			}
			if strings.Index(kv.String(), "\n") == -1 { // single value
				buf.WriteString(kv.String())
				buf.WriteString("\n")
				continue
			}

			buf.WriteString("\n")
			values := strings.Split(kv.String(), "\n")
			buf.WriteString("  " + values[0])
			for _, val := range values[1:] {
				buf.WriteString("\n" + "  " + val)
			}
			buf.WriteString("\n")
		}

		buf.WriteString("\n")
	}

	return buf
}

func (f *File) NewSection(name string) *Section {
	if _, ok := f.sections[name]; !ok {
		f.sections[name] = NewSection(name)
		f.sectionList = append(f.sectionList, name)
	}
	return f.sections[name]
}
