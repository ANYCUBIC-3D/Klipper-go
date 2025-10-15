package project

import (
	"bytes"
	"compress/zlib"
	"container/list"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"k3c/common/logger"
	"k3c/common/utils/reflects"
	"math"
	"reflect"
	"strconv"
	"unicode"

	"strings"
)

var DefaultMessages = map[string]int{
	"identify_response offset=%u data=%.*s": 0,
	"identify offset=%u count=%c":           1,
}

const (
	MESSAGE_MIN          = 5
	MESSAGE_MAX          = 64
	MESSAGE_HEADER_SIZE  = 2
	MESSAGE_TRAILER_SIZE = 3
	MESSAGE_POS_LEN      = 0
	MESSAGE_POS_SEQ      = 1
	MESSAGE_TRAILER_CRC  = 3
	MESSAGE_TRAILER_SYNC = 1
	MESSAGE_PAYLOAD_MAX  = MESSAGE_MAX - MESSAGE_MIN
	MESSAGE_SEQ_MASK     = 0x0f
	MESSAGE_DEST         = 0x10
	MESSAGE_SYNC         = 0x7e
)

type MsgprotoError interface {
}
type PT interface {
	Encode(out *[]int, value interface{})
	Parse(s []int, pos int) (interface{}, int)
	GetIs_dynamic_string() bool
	Get_max_length() int
}

func Crc16_ccitt(buf []int) []int {
	var _crc = 0xffff
	for data := range buf {
		data ^= _crc & 0xff
		data ^= (data & 0x0f) << 4
		_crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
	}

	return []int{_crc >> 8, _crc & 0xff}
}

type PT_uint32 struct {
	Is_int            bool
	Is_dynamic_string bool
	Max_length        int
	Signed            bool
}

func NewPT_uint32() *PT_uint32 {
	self := &PT_uint32{}
	self.Is_int = true
	self.Is_dynamic_string = false
	self.Max_length = 5
	self.Signed = false
	return self
}
func (self *PT_uint32) GetIs_dynamic_string() bool {
	return self.Is_dynamic_string
}
func (self *PT_uint32) Encode(out *[]int, value interface{}) {
	var v int64
	if _v, ok := value.(int); ok {
		v = int64(_v)
	} else {
		v = value.(int64)
	}

	if v >= 0xc000000 || v < -0x4000000 {
		val := (v>>28)&0x7f | 0x80
		*out = append(*out, int(val))
	}
	if v >= 0x180000 || v < -0x80000 {
		val := (v>>21)&0x7f | 0x80
		*out = append(*out, int(val))
	}
	if v >= 0x3000 || v < -0x1000 {
		val := (v>>14)&0x7f | 0x80
		*out = append(*out, int(val))
	}
	if v >= 0x60 || v < -0x20 {
		val := (v>>7)&0x7f | 0x80
		*out = append(*out, int(val))
	}
	*out = append(*out, int(v&0x7f))
}

func (self *PT_uint32) Parse(s []int, pos int) (interface{}, int) {
	var c = int64(s[pos])
	pos += 1
	var v = c & 0x7f
	if (c & 0x60) == 0x60 {
		v |= -0x20
	}
	for {
		if (c & 0x80) != 0x80 {
			break
		}
		c = int64(s[pos])
		pos += 1
		v = (v << 7) | (c & 0x7f)
	}
	if !self.Signed {
		v = v & 0xffffffff
	}
	return v, pos
}
func (self PT_uint32) Get_max_length() int {
	return self.Max_length
}

type PT_int32 struct {
	PT_uint32
}

func NewPT_int32() *PT_int32 {
	self := &PT_int32{}
	self.PT_uint32 = *NewPT_uint32()
	self.Signed = true
	return self
}

type PT_uint16 struct {
	PT_uint32
}

func NewPT_uint16() *PT_uint16 {
	self := &PT_uint16{}
	self.PT_uint32 = *NewPT_uint32()
	self.Max_length = 3
	return self
}

type PT_int16 struct {
	PT_int32
}

func NewPT_int16() *PT_int16 {
	self := &PT_int16{}
	self.PT_uint32 = *NewPT_uint32()
	self.Max_length = 3
	self.Signed = true
	return self
}

type PT_byte struct {
	PT_uint32
}

func NewPT_byte() *PT_byte {
	self := &PT_byte{}
	self.PT_uint32 = *NewPT_uint32()
	self.Max_length = 2
	return self
}

type PT_string struct {
	Is_int            bool
	Is_dynamic_string bool
	Max_length        int
}

func NewPT_string() *PT_string {
	self := &PT_string{}
	self.Is_int = false
	self.Is_dynamic_string = true
	self.Max_length = 64
	return self
}
func (self *PT_string) GetIs_dynamic_string() bool {
	return self.Is_dynamic_string
}
func (self *PT_string) Encode(out *[]int, value interface{}) {
	var v []int
	if _, ok := value.([]int64); ok {
		for _, vi := range value.([]int64) {
			v = append(v, int(vi))
		}
	} else {
		v = value.([]int)
	}

	*out = append(*out, len(v))
	for _, val := range v {
		*out = append(*out, val)
	}
}

func (self *PT_string) Parse(s []int, pos int) (interface{}, int) {
	l := s[pos]
	if pos+l+1 >= len(s) {
		return s[0:0], 0
	}
	return s[pos+1 : pos+l+1], pos + l + 1
}

func (self *PT_string) Get_max_length() int {
	return self.Max_length
}

type PT_progmem_buffer struct {
	PT_string
}

func NewPT_progmem_buffer() *PT_progmem_buffer {
	self := &PT_progmem_buffer{}
	self.PT_string = *NewPT_string()
	return self
}
func (self *PT_progmem_buffer) Encode(out *[]int, value interface{}) {
	self.PT_string.Encode(out, value)
}
func (self *PT_progmem_buffer) Parse(s []int, pos int) (interface{}, int) {
	return self.PT_string.Parse(s, pos)
}

type PT_buffer struct {
	PT_string
}

func NewPT_buffer() *PT_buffer {
	self := &PT_buffer{}
	self.PT_string = *NewPT_string()
	return self
}
func (self *PT_buffer) Encode(out *[]int, value interface{}) {
	self.PT_string.Encode(out, value)
}
func (self *PT_buffer) Parse(s []int, pos int) (interface{}, int) {
	return self.PT_string.Parse(s, pos)
}

type Enumeration_error struct {
	error
	Enum_name string
	Value     string
}

func NewEnumeration_error(enum_name string, value string) *Enumeration_error {
	self := Enumeration_error{}
	self.Enum_name = enum_name
	self.Value = value
	return &self
}

func (self *Enumeration_error) Get_enum_params() (string, string) {
	return self.Enum_name, self.Value
}

func (self *Enumeration_error) Enumeration_error(name string, v string) {
	logger.Errorf("Unknown value '%s' in enumeration '%s'", name, v)
}

type base struct {
	Max_length int
	Name       string
}

func (self *base) Encode(out []int, v []int) {

}

func (self *base) Parse(s []int, pos int) (map[string]interface{}, int) {
	return nil, 0
}

func (self *base) Format_params(params map[string]interface{}) string {
	return ""
}

type Enumeration struct {
	Is_int            bool
	Is_dynamic_string bool
	Pt                PT
	Max_length        int
	Enum_name         string
	Enums             map[string]interface{}
	Reverse_enums     map[int]interface{}
}

func NewEnumeration(pt interface{}, enum_name string, enums map[string]interface{}) *Enumeration {
	self := Enumeration{}
	self.Is_int = false
	self.Is_dynamic_string = false
	self.Pt = pt.(PT)
	self.Max_length = pt.(PT).Get_max_length()
	self.Enum_name = enum_name
	self.Enums = enums
	self.Reverse_enums = make(map[int]interface{})
	for k, v := range enums {
		newK, ok := v.(int)
		if ok {
			self.Reverse_enums[newK] = k
		}
		newK1, ok1 := v.(float64)
		if ok1 {
			self.Reverse_enums[int(newK1)] = k
		}
	}
	return &self
}

func (self *Enumeration) Encode(out *[]int, value interface{}) {
	tv := self.Enums[value.(string)]
	if tv == nil {
		panic(NewEnumeration_error(self.Enum_name, value.(string)))
	}
	self.Pt.Encode(out, tv)
}
func (self *Enumeration) Parse(s []int, pos int) (interface{}, int) {
	v, pos := self.Pt.Parse(s, pos)
	tv := self.Reverse_enums[int(v.(int64))]
	if tv == nil {
		tv = fmt.Sprintf("?%d", v)
	}
	return tv, pos
}
func (self *Enumeration) GetIs_dynamic_string() bool { return self.Is_dynamic_string }
func (self *Enumeration) Get_max_length() int        { return self.Max_length }

var MessageTypes = map[string]interface{}{
	"%u": NewPT_uint32(), "%i": NewPT_int32(),
	"%hu": NewPT_uint16(), "%hi": NewPT_int16(),
	"%c": NewPT_byte(),
	"%s": NewPT_string(), "%.*s": NewPT_progmem_buffer(), "%*s": NewPT_buffer(),
}

// Lookup the message types for a format string
func Lookup_params(msgformat string, enumerations map[string]interface{}) []interface{} {
	var out = []interface{}{}
	argparts := [][]string{}
	for _, arg := range strings.Split(msgformat, " ")[1:] {
		if arg == "" {
			continue
		}
		s := strings.Split(arg, "=")
		argparts = append(argparts, s)
	}
	for _, arg := range argparts {
		if len(arg) == 0 {
			continue
		}
		name := arg[0]
		fmt := arg[1]
		pt := MessageTypes[fmt]
		for enum_name, enums := range enumerations {
			if name == enum_name || strings.HasSuffix(name, "_"+enum_name) {
				pt = NewEnumeration(pt, enum_name, enums.(map[string]interface{}))
				break
			}
		}
		out = append(out, []interface{}{name, pt})
	}
	return out
}

// Lookup the message types for a debugging "output()" format string
func Lookup_output_params(msgformat string) []interface{} {
	var param_types = []interface{}{}
	var args = msgformat
	var break_flag = false
	for {
		var pos = strings.Index(args, "%")
		if pos < 0 {
			break
		}
		if pos+1 >= len(args) || args[pos+1] != '%' {
			for i := 0; i < 4; i++ {
				var t = MessageTypes[args[pos:pos+1+i]]
				if t != nil {
					param_types = append(param_types, t)
					break_flag = true
					break
				}
			}
			if break_flag {
				logger.Panicf("Invalid output format for '%s'", msgformat)
			}
		}
		args = args[pos+1:]
	}
	return param_types
}

// Update the message format to be compatible with python's % operator
func Convert_msg_format(msgformat string) string {
	conds := []string{"%u", "%i", "%hu", "%hi", "%c", "%.*s", "%*s"}
	ret := msgformat
	for _, cond := range conds {
		ret = strings.ReplaceAll(ret, cond, "%s")
	}
	return ret
}

type MessageFormat struct {
	Param_types  *list.List
	Msgformat    string
	Name         string
	Param_names  []interface{}
	msgid_bytes  []int
	Debugformat  string
	Name_to_type map[string]interface{}
}

func NewMessageFormat(msgid_bytes []int, msgformat string, enumerations map[string]interface{}) *MessageFormat {
	self := MessageFormat{}
	self.msgid_bytes = append(self.msgid_bytes, msgid_bytes...)
	self.Msgformat = msgformat
	self.Debugformat = Convert_msg_format(msgformat)
	self.Name = strings.Split(msgformat, " ")[0]
	self.Param_names = Lookup_params(msgformat, enumerations)
	self.Param_types = list.New()
	self.Name_to_type = map[string]interface{}{}
	for _, row := range self.Param_names {
		name := row.([]interface{})[0].(string)
		pt := row.([]interface{})[1]
		self.Param_types.PushBack(pt)
		self.Name_to_type[name] = pt
	}
	return &self
}

func (self *MessageFormat) Encode(params interface{}) []int {
	out := []int{}
	out = append(out, self.msgid_bytes...)
	if self.Param_types.Len() != 0 {
		i := 0
		for d := self.Param_types.Front(); d != nil; d = d.Next() {
			t, ok := d.Value.(PT)
			if ok {
				if _, okk := params.([]interface{}); okk {
					if _, okk = params.([]interface{})[i].([]int); okk {
						t.Encode(&out, params.([]interface{})[i].([]int))
					} else if _, okk = params.([]interface{})[i].(int); okk {
						t.Encode(&out, params.([]interface{})[i].(int))
					} else if _, okk = params.([]interface{})[i].([]int64); okk {
						t.Encode(&out, params.([]interface{})[i].([]int64))
					} else if _, okk = params.([]interface{})[i].(int64); okk {
						t.Encode(&out, params.([]interface{})[i].(int64))
					}
				} else if _, okk = params.([]int64); okk {
					t.Encode(&out, params.([]int64)[i])
				} else {
					fmt.Print(t, params)
					panic("error ")
				}

			} else {
				logger.Debug(t)
			}
			i++
		}
	}
	return out
}

func (self *MessageFormat) Encode_by_name(params map[string]interface{}) []interface{} {
	out := []int{}
	out = append(out, self.msgid_bytes...)
	for _, param := range self.Param_names {
		p := param.([]interface{})
		name := p[0].(string)
		t := p[1]
		encode_by_name(&out, name, t, params)
	}
	ret := []interface{}{}
	for _, v := range out {
		ret = append(ret, v)
	}
	return ret
}
func encode_by_name(out *[]int, name string, t interface{}, params map[string]interface{}) {

	_, ok := t.(*PT_string)
	if ok {
		t.(*PT_string).Encode(out, params[name].([]int))
	}
	_, ok = t.(*PT_uint32)
	if ok {
		t.(*PT_uint32).Encode(out, params[name])
	}
	_, ok = t.(*PT_byte)
	if ok {
		t.(*PT_byte).Encode(out, params[name])
	}
	_, ok = t.(*PT_buffer)
	if ok {
		t.(*PT_buffer).Encode(out, params[name].([]int))
	}
	_, ok = t.(*PT_int16)
	if ok {
		t.(*PT_int16).Encode(out, params[name])
	}
	_, ok = t.(*PT_int32)
	if ok {
		t.(*PT_int32).Encode(out, params[name])
	}
	_, ok = t.(*PT_progmem_buffer)
	if ok {
		t.(*PT_progmem_buffer).Encode(out, params[name].([]int))
	}
	_, ok = t.(*PT_uint16)
	if ok {
		t.(*PT_uint16).Encode(out, params[name])
	}
	_, ok = t.(*Enumeration)
	if ok {
		t.(*Enumeration).Encode(out, params[name])
	}

}

func (self *MessageFormat) Parse(s []int, pos int) (map[string]interface{}, int) {
	pos += len(self.msgid_bytes)
	out := map[string]interface{}{}
	for _, param := range self.Param_names {
		p := param.([]interface{})
		name := p[0].(string)
		t := p[1].(PT)
		v, pos1 := t.Parse(s, pos)
		pos = pos1
		out[name] = v
	}
	return out, pos
}

func (self *MessageFormat) Format_params(params map[string]string) string {
	out := []interface{}{}
	for _, row := range self.Param_names {
		l := row.([]interface{})
		name := l[0].(string)
		t := l[1]
		v := params[name]
		if t.(*PT_string).Is_dynamic_string {
			out = append(out, []byte(v))
		}
		out = append(out, v)
	}
	return fmt.Sprintf(self.Debugformat, out)

}
func (self *MessageFormat) GetName() string {
	return self.Name
}

type OutputFormat struct {
	Name        string
	msgid_bytes []int
	Debugformat string
	Param_types []interface{}
	Msgformat   string
}

func NewOutputFormat(msgid_bytes []int, msgformat string) *OutputFormat {
	self := &OutputFormat{}
	self.msgid_bytes = append(self.msgid_bytes, msgid_bytes...)
	self.Msgformat = msgformat
	self.Debugformat = Convert_msg_format(msgformat)
	self.Param_types = Lookup_output_params(msgformat)
	self.Name = "#output"
	return self
}

func (self *OutputFormat) Parse(s []int, pos int) (map[string]interface{}, int) {
	pos += len(self.msgid_bytes)
	out := []interface{}{}
	for _, t := range self.Param_types {
		v, pos1 := t.(PT).Parse(s, pos)
		if t.(PT).GetIs_dynamic_string() {
			v = v.(string)
		}
		out = append(out, v)
		pos = pos1
	}
	return map[string]interface{}{"#msg": out}, pos
}

func (self *OutputFormat) Format_params(params map[string]interface{}) string {
	return fmt.Sprintf("#output %s", params["#msg"])
}
func (self *OutputFormat) GetName() string {
	return self.Name
}

type UnknownFormat struct {
	Name string
}

func NewUnknownFormat() *UnknownFormat {
	self := &UnknownFormat{}
	self.Name = "#unknown"
	return self
}

func (self *UnknownFormat) Parse(s []int, pos int) (map[string]interface{}, int) {
	pt_int32 := PT_int32{}
	msgid, _ := pt_int32.Parse(s, pos)
	if len(s)-1 == pos {
		msgid = s[pos]
	}
	msg := s
	return map[string]interface{}{"#msgid": msgid, "#msg": msg}, len(s) - MESSAGE_TRAILER_SIZE
}
func (self *UnknownFormat) Format_params(params map[string]string) string {
	return fmt.Sprintf("#unknown %s", string(params["#msg"]))
}
func (self *UnknownFormat) GetName() string {
	return self.Name
}

type IMessageFormat interface {
	Parse(s []int, pos int) (map[string]interface{}, int)
	Format_params(params map[string]string) string
	GetName() string
}

type MessageParser struct {
	Error             func(msg string, params interface{})
	Warn_prefix       string
	Unknown           *UnknownFormat
	Enumerations      map[string]interface{}
	Messages          []interface{}
	Messages_by_id    map[int]interface{}
	Messages_by_name  map[string]*MessageFormat
	Config            map[string]interface{}
	Version           string
	Build_versions    string
	Raw_identify_data []byte
	msgid_by_format   map[string]int
	msgid_parser      *PT_int32
}

func NewMessageParser(warn_prefix string) *MessageParser {
	self := MessageParser{}
	self.Warn_prefix = warn_prefix
	self.Unknown = NewUnknownFormat()
	self.Enumerations = map[string]interface{}{}
	self.Messages = []interface{}{}
	self.Messages_by_id = map[int]interface{}{}
	self.Messages_by_name = map[string]*MessageFormat{}
	self.msgid_by_format = map[string]int{}
	self.msgid_parser = NewPT_int32()
	self.Config = map[string]interface{}{}
	self.Version, self.Build_versions = "", ""
	self.Raw_identify_data = []byte{}
	self.Error = self.mpError
	self.Init_messages(DefaultMessages, []int{}, []int{})
	return &self
}

func (self *MessageParser) mpError(msg string, params interface{}) {
	logger.Warnf(self.Warn_prefix+msg, params)
}

func (self *MessageParser) Check_packet(s []int) int {
	if len(s) < MESSAGE_MIN {
		return 0
	}
	msglen := s[MESSAGE_POS_LEN]
	if msglen < MESSAGE_MIN || msglen > MESSAGE_MAX {
		return -1
	}
	msgseq := s[MESSAGE_POS_SEQ]

	if (int(msgseq) & (^MESSAGE_SEQ_MASK)) != MESSAGE_DEST {
		return -1
	}
	if len(s) < int(msglen) {
		// Need more data
		return 0
	}
	if s[msglen-MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
		return -1
	}
	msgcrc := s[msglen-MESSAGE_TRAILER_CRC : msglen-MESSAGE_TRAILER_CRC+2]
	crc := Crc16_ccitt(s[:msglen-MESSAGE_TRAILER_SIZE])
	if !reflect.DeepEqual(crc, msgcrc) {
		//logging.debug("got crc %s vs %s", repr(crc), repr(msgcrc))
		return -1
	}
	return int(msglen)
}
func (self *MessageParser) Dump(s []int) []string {
	msgseq := s[MESSAGE_POS_SEQ]
	out := []string{fmt.Sprintf("seq: %02x", msgseq)}
	pos := MESSAGE_HEADER_SIZE
	for {
		msgid := s[pos]
		mid := self.Messages_by_id[int(msgid)].(base)
		params, pos := mid.Parse(s, pos)
		out = append(out, mid.Format_params(params))
		if pos >= len(s)-MESSAGE_TRAILER_SIZE {
			break
		}
	}
	return out
}

func (self *MessageParser) Format_params(params map[string]string) string {
	name := params["#name"]
	mid := self.Messages_by_name[name]
	if mid != nil {
		return mid.Format_params(params)
	}
	msg := params["msg"]
	if msg != "" {
		return fmt.Sprintf("%s %s", name, msg)
	}
	dataType, _ := json.Marshal(params)
	return string(dataType)
}

func (self *MessageParser) Parse(s []int) map[string]interface{} {
	var mid IMessageFormat
	mid = self.Unknown
	msgid, _ := self.msgid_parser.Parse(s, MESSAGE_HEADER_SIZE)
	t, ok := self.Messages_by_id[int(msgid.(int64))]
	if ok {
		mid = t.(IMessageFormat)
	}
	params, pos := mid.Parse(s, MESSAGE_HEADER_SIZE)
	if pos != len(s)-MESSAGE_TRAILER_SIZE {
		logger.Panicf("Extra data at end of message")
	}
	params["#name"] = mid.(IMessageFormat).GetName()
	return params
}

func (self *MessageParser) Encode_msgblock(seq int, cmd []int) []int {
	msglen := MESSAGE_MIN + len(cmd)
	seq = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST
	out := []int{msglen, seq}
	out = append(out, cmd...)
	out = append(out, Crc16_ccitt(out)...)
	out = append(out, MESSAGE_SYNC)
	return out
}

func (self *MessageParser) Parse_buffer(value string) []int {
	if value == "" {
		return []int{}
	}
	tval, _ := strconv.Atoi(value)

	var out []int
	for i := 0; i < int(math.Floor(float64(len(value)/2))); i++ {
		out = append(out, tval&0xff)
		tval >>= 8
	}
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

func (self *MessageParser) Lookup_command(msgformat string) (interface{}, error) {

	parts := strings.Split(strings.TrimSpace(msgformat), " ")
	msgname := parts[0]
	mp := self.Messages_by_name[msgname]
	if mp == nil {
		return nil, errors.New(fmt.Sprintf("Unknown command: %s", msgname))
	}
	if msgformat != mp.Msgformat {
		self.Error("Command format mismatch: %s vs %s", []string{msgformat, mp.Msgformat})
		return nil, errors.New(fmt.Sprintf("Command format mismatch: %s vs %s", msgformat, mp.Msgformat))
	}
	return mp, nil
}

func (self *MessageParser) lookup_msgid(msgformat string) int {
	msgid := -1
	ok := false
	if msgid, ok = self.msgid_by_format[msgformat]; !ok {
		panic(fmt.Errorf("Unknown command: %s", msgformat))
	}
	return msgid
}

func (self *MessageParser) Create_command(msg string) []int {
	parts := strings.Split(strings.TrimSpace(msg), " ")
	if len(parts) == 0 {
		return []int{}
	}
	msgname := parts[0]
	mp := self.Messages_by_name[msgname]
	if mp == nil {
		logger.Errorf("Unknown command: %s", msgname)
		return []int{}
	}
	argparts := map[string]interface{}{}
	for _, arg := range parts[1:] {
		args := strings.Split(arg, "=")
		argparts[args[0]] = args[1]
	}
	for name, value := range argparts {
		t := mp.Name_to_type[name]
		var tval interface{}
		if t == nil {
			logger.Errorf("Unknown parameter type for %s", name)
			return []int{}
		}
		ret := reflects.ReflectFieldValue(t, "Is_dynamic_string")
		Is_dynamic_string := ret.(bool)
		ret = reflects.ReflectFieldValue(t, "Is_int")
		Is_int := ret.(bool)
		if Is_dynamic_string {
			tval = self.Parse_buffer(value.(string))
		} else if Is_int {
			tval, _ = strconv.ParseInt(value.(string), 10, 64)
		} else {
			tval = value
		}
		argparts[name] = tval
	}
	cmd := mp.Encode_by_name(argparts)
	ret := []int{}
	for _, c := range cmd {
		ret = append(ret, c.(int))
	}
	return ret
}

func (self *MessageParser) Fill_enumerations(enumerations map[string]interface{}) {
	for add_name, add_enums := range enumerations {
		enums := map[string]interface{}{}
		self.Enumerations[add_name] = enums
		enumsList := add_enums.(map[string]interface{})
		for enum, val := range enumsList {
			data, ok := val.(float64)
			if ok {
				//# Simple enumeration
				enums[enum] = data
				continue
			}
			//# Enumeration range
			enum_root := enum
			for enum_root != "" && unicode.IsNumber(rune(enum_root[len(enum_root)-1])) {
				enum_root = enum_root[:len(enum_root)-1]
			}

			start_enum := 0
			if len(enum_root) != len(enum) {
				num, _ := strconv.Atoi(enum[len(enum_root):])
				start_enum = num
			}
			start_value := int(val.([]interface{})[0].(float64))
			count := int(val.([]interface{})[1].(float64))
			for i := 0; i < count; i++ {
				enums[enum_root+strconv.Itoa(start_enum+i)] = start_value + i
			}
		}
	}
}

func (self *MessageParser) Init_messages(messages map[string]int, command_ids []int, output_tags []int) {
	for msgformat, msgid := range messages {
		msgtype := "response"
		for msgid := range command_ids {
			if msgid == msgid {
				msgtype = "command"
			}
		}
		for msgid := range output_tags {
			if msgid == msgid {
				msgtype = "output"
			}
		}
		self.Messages = append(self.Messages, []interface{}{msgid, msgtype, msgformat})
		self.msgid_by_format[msgformat] = msgid
		msgid_bytes := []int{}
		self.msgid_parser.Encode(&msgid_bytes, msgid)
		if msgtype == "output" {
			self.Messages_by_id[msgid] = NewOutputFormat(msgid_bytes, msgformat)
		} else {
			msg := NewMessageFormat(msgid_bytes, msgformat, self.Enumerations)
			self.Messages_by_id[msgid] = msg
			self.Messages_by_name[msg.Name] = msg
		}
	}
}

func (self *MessageParser) Process_identify(_data []byte, decompress bool) {

	if decompress {
		reader := bytes.NewReader(_data)
		var out bytes.Buffer
		r, err := zlib.NewReader(reader)
		if err != nil {
			logger.Error(err.Error())
		} else {
			defer r.Close()
			io.Copy(&out, r)
			_data = out.Bytes()
		}
	}
	self.Raw_identify_data = _data
	data := map[string]interface{}{}
	err := json.Unmarshal(_data, &data)
	if err != nil {
		logger.Error(err.Error())
	}
	temp, ok := data["enumerations"]
	if !ok {
		temp = map[string]interface{}{}
	}
	self.Fill_enumerations(temp.(map[string]interface{}))
	commands := data["commands"].(map[string]interface{})
	responses := data["responses"].(map[string]interface{})
	temp, ok = data["output"]
	if !ok {
		temp = map[string]int{}
	}
	output := temp.(map[string]int)
	all_messages := map[string]int{}
	command_tags := []int{}
	for k, v := range commands {
		command_tags = append(command_tags, int(v.(float64)))
		all_messages[k] = int(v.(float64))
	}
	for k, v := range responses {
		all_messages[k] = int(v.(float64))
	}
	output_tags := []int{}
	for k, v := range output {
		output_tags = append(output_tags, v)
		all_messages[k] = v
	}

	self.Init_messages(all_messages, command_tags,
		output_tags)
	temp, ok = data["config"]
	if ok {
		for k, v := range temp.(map[string]interface{}) {
			self.Config[k] = v
		}
	}
	self.Version = data["version"].(string)
	self.Build_versions = data["build_versions"].(string)

}

func (self *MessageParser) Get_raw_data_dictionary() []byte {
	return self.Raw_identify_data
}

func (self *MessageParser) Get_version_info() (string, string) {
	return self.Version, self.Build_versions
}

func (self *MessageParser) Get_messages() []interface{} {
	messages_back := make([]interface{}, len(self.Messages))
	copy(messages_back, self.Messages)
	return messages_back
}

func (self *MessageParser) Get_enumerations() map[string]interface{} {
	return self.Enumerations
}

func (self *MessageParser) Get_constants() map[string]interface{} {
	return self.Config
}

//type sentinel struct {}

func (self *MessageParser) Get_constant(name string, _default interface{}, kind reflect.Kind) interface{} {
	v, ok := self.Config[name]
	if !ok {
		if _default != nil {
			return _default
		}
		logger.Errorf("Firmware constant '%s' not found", name)
		return nil
	}
	return self.get_constant4kind(v, reflect.TypeOf(v).Kind(), kind)
}
func (self *MessageParser) get_constant4kind(v interface{}, kind1 reflect.Kind, kind2 reflect.Kind) interface{} {

	if kind1 == kind2 {
		return v
	} else {
		//
		if kind1 == reflect.String {
			value := v.(string)
			if kind2 == reflect.String {
				return value
			} else if kind2 == reflect.Int {
				ret, err := strconv.Atoi(value)
				if err != nil {
					logger.Error(err.Error())
					return 0
				}
				return ret
			} else if kind2 == reflect.Float64 {
				ret, err := strconv.ParseFloat(value, 64)
				if err != nil {
					logger.Error(err.Error())
					return 0
				}
				return ret
			}
		} else if kind1 == reflect.Int {
			value := v.(int)
			if kind2 == reflect.String {
				ret := strconv.Itoa(value)
				return ret
			} else if kind2 == reflect.Int {

				return value
			} else if kind2 == reflect.Float64 {

				return float64(value)
			}
		} else if kind1 == reflect.Float64 {
			value := v.(float64)
			if kind2 == reflect.String {
				ret := strconv.FormatFloat(value, 0, 0, 64)
				return ret
			} else if kind2 == reflect.Int {

				return int(value)
			} else if kind2 == reflect.Float64 {

				return value
			}
		}
	}
	return nil
}
func (self *MessageParser) Get_constant_float(name string, _default interface{}) float64 {
	v := self.Get_constant(name, _default, reflect.Float64)
	if v == nil {
		return 0
	} else {
		ret, ok := v.(float64)
		if ok {
			return ret
		} else {
			return 0
		}
	}

}

func (self *MessageParser) Get_constant_int(name string, _default interface{}) int {

	v := self.Get_constant(name, _default, reflect.Int)
	if v == nil {
		return 0
	} else {
		return v.(int)
	}
}
