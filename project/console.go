package project

import (
	"encoding/json"
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/cast"
	"k3c/project/util"
	"os"
	"reflect"
	"regexp"
	"strconv"
	"strings"
)

const help_txt = `
  This is a debugging console for the micro-controller.
  In addition to mcu commands, the following artificial commands are
  available:
    DELAY : Send a command at a clock time (eg, "DELAY 9999 get_uptime")
    FLOOD : Send a command many times (eg, "FLOOD 22 .01 get_uptime")
    SUPPRESS : Suppress a response message (eg, "SUPPRESS analog_in_state 4")
    SET   : Create a local variable (eg, "SET myvar 123.4")
    DUMP  : Dump memory (eg, "DUMP 0x12345678 100 32")
    FILEDUMP : Dump to file (eg, "FILEDUMP data.bin 0x12345678 100 32")
    STATS : Report serial statistics
    LIST  : List available mcu commands, local commands, and local variables
    HELP  : Show this text
  All commands also support evaluation by enclosing an expression in { }.
  For example, "reset_step_clock oid=4 clock={clock + freq}".  In addition
  to user defined variables (via the SET command) the following builtin
  variables may be used in expressions:
    clock : The current mcu clock time (as estimated by the host)
    freq  : The mcu clock frequency
`

var re_eval = regexp.MustCompile("{(?P<eval>[^}]*)}")

type KeyboardReader struct {
	serialport    string
	baud          int
	canbus_iface  string
	canbus_nodeid int

	ser            *SerialReader
	reactor        IReactor
	start_time     float64
	clocksync      *ClockSync
	fd             int
	mcu_freq       int
	pins           *PinResolver
	data           string
	local_commands map[string]interface{}
	eval_globals   map[string]interface{}
}

func NewKeyboardReader(reactor IReactor, serialport string, baud int, canbus_iface string, canbus_nodeid int) *KeyboardReader {
	self := new(KeyboardReader)
	self.baud = baud
	self.canbus_iface = canbus_iface
	self.canbus_nodeid = canbus_nodeid
	self.ser = NewSerialReader(reactor, "")
	self.reactor = reactor
	self.start_time = reactor.Monotonic()
	self.clocksync = NewClockSync(self.reactor)
	self.fd = int(os.Stdin.Fd())
	util.Set_nonblock(self.fd)

	self.mcu_freq = 0
	self.pins = NewPinResolver(false)
	self.data = ""

	reactor.Register_fd(self.fd, self.process_kbd, nil)
	reactor.Register_callback(self.connect, 0)

	self.eval_globals = make(map[string]interface{}, 8)
	self.local_commands = map[string]interface{}{
		"SET":      self.command_SET,
		"DUMP":     self.command_DUMP,
		"FILEDUMP": self.command_FILEDUMP,
		"DELAY":    self.command_DELAY,
		"FLOOD":    self.command_FLOOD,
		"SUPPRESS": self.command_SUPPRESS,
		"STATS":    self.command_STATS,
		"LIST":     self.command_LIST,
		"HELP":     self.command_HELP,
	}
	return nil
}

func (self *KeyboardReader) connect(eventtime interface{}) interface{} {
	self.output(help_txt)
	self.output(strings.Repeat("=", 20) + " attempting to connect " + strings.Repeat("=", 20))
	if len(self.canbus_iface) != 0 {
		iface := self.canbus_iface
		if len(iface) == 0 {
			iface = "can0"
		}
		self.ser.connect_canbus(self.serialport, cast.ToString(self.canbus_nodeid), iface)
	} else if self.baud > 0 {
		self.ser.Connect_uart(self.serialport, self.baud, true)
	} else {
		self.ser.Connect_pipe(self.serialport)
	}

	msgparser := self.ser.Get_msgparser()
	message_count := len(msgparser.Get_messages())
	version, build_version := msgparser.Get_version_info()
	self.output(fmt.Sprintf("Loaded %d commands (%s / %s)", message_count, version, build_version))

	var configItems []string
	for k, v := range msgparser.Get_constants() {
		configItems = append(configItems, fmt.Sprintf("%s=%s", k, v))
	}

	self.output(fmt.Sprintf("MCU config: %s", strings.Join(configItems, " ")))
	self.clocksync.Connect(self.ser)

	self.ser.Register_response(self.handle_output, "#output", nil)
	self.mcu_freq = int(msgparser.Get_constant_float("CLOCK_FREQ", "sentinel"))
	self.output(strings.Repeat("=", 20) + "       connected       " + strings.Repeat("=", 20))
	return constants.NEVER
}

func (self *KeyboardReader) output(msg string) {
	os.Stdout.WriteString(msg + "\n")
	os.Stdout.Sync()
}

// handle_default(params map[string]interface{})
func (self *KeyboardReader) handle_default(params map[string]interface{}) {
	receive_time, _ := strconv.ParseFloat(cast.ToString(params["#receive_time"]), 64)
	tdiff := receive_time - self.start_time

	formatedParams := make(map[string]string)
	for k, v := range params {
		formatedParams[k] = cast.ToString(v)
	}
	msg := self.ser.Get_msgparser().Format_params(formatedParams)
	self.output(fmt.Sprintf("%07.3f: %s", tdiff, msg))
}

func (self *KeyboardReader) handle_output(params map[string]string) {
	receive_time, _ := strconv.ParseFloat(params["#receive_time"], 64)
	tdiff := receive_time - self.start_time
	self.output(fmt.Sprintf("%07.3f: %s: %s", tdiff, params["#name"], params["#msg"]))
}

func (self *KeyboardReader) handle_suppress(params map[string]string) {
	return
}

func (self *KeyboardReader) update_evals(eventtime float64) {
	self.eval_globals["freq"] = self.mcu_freq
	self.eval_globals["clock"] = self.clocksync.Get_clock(eventtime)
}

func (self *KeyboardReader) command_SET(parts []interface{}) {
	if len(parts) < 2 {
		return
	}

	parts1, ok := parts[1].(string)
	if !ok {
		return
	}
	self.eval_globals[parts1] = cast.ToFloat64(parts[2])
}

func (self *KeyboardReader) command_DUMP(parts []interface{}, filename string) {
	if len(parts) < 2 {
		self.output(fmt.Sprintf("Error: %s", "parts length should greater than 2"))
		return
	}
	addr := cast.ToInt(parts[1])
	count := cast.ToInt(parts[2])
	order := [4]int{2, 0, 1, 0}[(addr|count)&3]
	if len(parts) > 3 {
		orderMap := map[string]int{
			"32": 2,
			"16": 1,
			"8":  0,
		}
		order = orderMap[cast.ToString(parts[3])]
	}

	bsize := 1 << order
	vals := make([]int, 0)

	rangeCh := rangeInt(0, (count+bsize-1)>>order)
	for i := range rangeCh {
		caddr := addr + (i << order)
		cmd := fmt.Sprintf("debug_read order=%d addr=%d", order, caddr)
		params, _ := self.ser.Send_with_response(cmd, "debug_result")
		vals = append(vals, cast.ToInt(params["val"]))
	}

	if filename == "" && order == 2 {
		for i := range rangeInt(0, floorDiv((len(vals)+3), 4)) {
			p := i * 4
			hexvals := joinIntWithFormat(vals[p:p+4], " ", "%08x")
			self.output(fmt.Sprintf("%08x  %s", addr+p*4, hexvals))
		}
		return
	}

	data := make([]byte, 8)
	for _, val := range vals {
		for b := range rangeInt(0, bsize) {
			data = append(data, byte(val>>(8*b)&0xff))
		}
	}

	data = data[:count]
	if filename != "" {
		f, err := os.Open(filename)
		if err != nil {
			self.output(fmt.Sprintf("Error: open file %s, %v", filename, err))
			return
		}
		defer f.Close()
		n, err := f.Write(data)
		if err != nil {
			self.output(fmt.Sprintf("Error: write file %s, %v", filename, err))
			return
		}
		self.output(fmt.Sprintf("Wrote %d bytes to '%s'", n, filename))
		return
	}

	for i := range rangeInt(0, floorDiv((count+15), 16)) {
		p := i * 16
		paddr := addr + p
		d := data[p : p+16]
		var hexbytes = joinByteWithFormat(d, " ", "%02x")
		var filtered = make([]interface{}, 0, len(d))
		for _, v := range d {
			if v > 0x20 && v < 0x7f {
				filtered = append(filtered, v)
			} else {
				filtered = append(filtered, '.')
			}
		}
		pb := joinWithFormat(filtered, " ", "%02x")
		o := fmt.Sprintf("%08x  %-47s  |%s|", paddr, hexbytes, pb)
		self.output(fmt.Sprintf("%s %s", o[:34], o[34:]))
	}
}

func (self *KeyboardReader) command_FILEDUMP(parts []interface{}) {
	self.command_DUMP(parts[1:], cast.ToString(parts[1]))
}

func (self *KeyboardReader) command_DELAY(parts []interface{}) {
	val, err := cast.ToInt64E(parts[1])
	if err != nil {
		self.output(fmt.Sprintf("Error: %s", err))
		return
	}

	// @todo why not return erorr
	self.ser.Send(joinWithFormat(parts[2:], " ", "%v"), val, 0)
}

func (self *KeyboardReader) command_FLOOD(parts []interface{}) {
	count, err := cast.ToIntE(parts[1])
	if err != nil {
		self.output(fmt.Sprintf("Error: %s", err))
	}
	delay, err := cast.ToFloat64E(parts[2])
	if err != nil {
		self.output(fmt.Sprintf("Error: %s", err))
	}

	msg := joinWithFormat(parts[3:], " ", "%v")
	delay_clock := int64(delay * float64(self.mcu_freq))
	msg_clock := self.clocksync.Get_clock(self.reactor.Monotonic()) + int64(float64(self.mcu_freq)*.200)

	for range rangeInt(0, count) {
		next_clock := msg_clock + delay_clock
		self.ser.Send(msg, msg_clock, next_clock) // @todo should return error
		msg_clock = next_clock
	}
}

func (self *KeyboardReader) command_SUPPRESS(parts []interface{}) {
	var oid int
	if len(parts) == 0 {
		self.output("Error: parts is empty")
	}
	name := cast.ToString(parts[1])
	if len(parts) > 2 {
		var err error
		oid, err = cast.ToIntE(parts[2])
		if err != nil {
			self.output(fmt.Sprintf("Error: %s", err))
			return
		}
	}

	self.ser.Register_response(self.handle_suppress, name, oid)
}

func (self *KeyboardReader) command_STATS(parts []interface{}) {
	curtime := self.reactor.Monotonic()
	self.output(strings.Join([]string{
		self.ser.stats(curtime),
		self.clocksync.Stats(curtime),
	}, " "))
}

// @todo
func (self *KeyboardReader) command_LIST(parts []interface{}) {
	self.update_evals(self.reactor.Monotonic())
	mp := self.ser.Get_msgparser()
	mp.Get_messages()

}

func (self *KeyboardReader) command_HELP(parts []interface{}) {
	self.output(help_txt)
}

func (self *KeyboardReader) translate(line string, eventtime float64) string {
	evalparts := re_eval.Split(line, -1)

	if len(evalparts) > 1 {
		self.update_evals(eventtime)
		for i := range rangeInt(1, len(evalparts), 2) {
			dict := make(map[string]interface{})
			err := json.Unmarshal([]byte(evalparts[1]), &dict)
			if err != nil {
				self.output(fmt.Sprintf("Unable to evaluate: %v", err))
				return ""
			}
			for k, v := range self.eval_globals {
				if _, ok := dict[k]; !ok {
					evalparts[i] = cast.ToString(v)
				}
			}
		}

		line = strings.Join(evalparts, "")
		self.output(fmt.Sprintf("Eval: %s", line))
	}
	line = strings.TrimSpace(self.pins.Update_command(line))
	if line != "" {
		parts := strings.Split(line, " ")
		if _, ok := self.local_commands[parts[0]]; ok {
			var param []reflect.Value
			for _, p := range parts {
				param = append(param, reflect.ValueOf(p))
			}
			reflect.ValueOf(self.local_commands[parts[0]]).Call(param)
		}
	}
	return line
}

func (self *KeyboardReader) process_kbd(eventtime float64) interface{} {
	f := os.NewFile(uintptr(self.fd), "stdin")
	var data [4096]byte
	n, _ := f.Read(data[:])
	self.data += string(data[:n])
	kbdlines := strings.Split(self.data, "\n")

	for _, line := range kbdlines {
		line = strings.TrimSpace(line)
		idx := strings.Index(line, "#")
		if idx >= 0 {
			line = line[:idx]
			if len(line) == 0 {
				continue
			}
		}

		msg := self.translate(strings.TrimSpace(line), eventtime)
		if len(msg) == 0 {
			continue
		}
		self.ser.Send(msg, 0, 0)
	}

	self.data = kbdlines[len(kbdlines)-1] // last elem
	return nil
}

func rangeInt(min, max int, step ...int) <-chan int {
	if min > max {
		panic("min should less than max")
	}
	ch := make(chan int)
	var stepi = 1
	if len(step) > 0 {
		stepi = step[stepi]
	}
	go func() {
		defer close(ch)
		for i := min; i < max; i = i + stepi {
			ch <- i
		}
	}()
	return ch
}

func floorDiv(a, b int) int {
	return int(a / b)
}

func joinIntWithFormat(elem []int, sep, format string) string {
	elems := make([]interface{}, len(elem), len(elem))
	for idx := range elem {
		elems[idx] = elem[idx]
	}
	return joinWithFormat(elems, sep, format)
}

func joinByteWithFormat(elem []byte, sep, format string) string {
	elems := make([]interface{}, len(elem), len(elem))
	for idx := range elem {
		elems[idx] = elem[idx]
	}
	return joinWithFormat(elems, sep, format)
}

func joinWithFormat(elems []interface{}, sep, format string) string {
	switch len(elems) {
	case 0:
		return ""
	case 1:
		return fmt.Sprintf(format, elems[0])
	}
	var strs = fmt.Sprintf(format, elems[0])
	for _, elem := range elems[1:] {
		strs = sep + fmt.Sprintf(format, elem)
	}
	return strs
}
