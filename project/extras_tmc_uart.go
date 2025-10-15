package project

import (
	"errors"
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math/big"
	"reflect"
	"strconv"
	"strings"
)

/**
######################################################################
# TMC uart analog mux support
######################################################################
*/

type MCU_analog_mux struct {
	mcu            *MCU
	cmd_queue      interface{}
	oids           []int
	pins           []interface{}
	pin_values     []int64
	update_pin_cmd *CommandWrapper
}

func NewMCU_analog_mux(mcu *MCU, cmd_queue interface{}, select_pins_desc []string) *MCU_analog_mux {
	self := new(MCU_analog_mux)
	self.mcu = mcu
	self.cmd_queue = cmd_queue

	ppins := MustLookupPins(mcu.Get_printer())
	var select_pin_params = make([]map[string]interface{}, 0)

	for _, spd := range select_pins_desc {
		select_pin_params = append(select_pin_params, ppins.Lookup_pin(spd, true, false, nil))
	}

	self.oids = make([]int, 0, len(select_pin_params))
	self.pins = make([]interface{}, 0, len(select_pin_params))
	self.pin_values = make([]int64, 0, len(select_pin_params))
	for _, pp := range select_pin_params {
		self.oids = append(self.oids, self.mcu.Create_oid())
		self.pins = append(self.pins, pp["pin"])
		self.pin_values = append(self.pin_values, -1)
	}

	for i := 0; i < len(self.oids); i++ {
		self.mcu.Add_config_cmd(fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=0",
			self.oids[i], self.pins[i]), false, false)
	}

	self.update_pin_cmd = nil
	self.mcu.Register_config_callback(self.build_config)
	return self
}

func (self *MCU_analog_mux) build_config() {
	self.update_pin_cmd, _ = self.mcu.Lookup_command("update_digital_out oid=%c value=%c", self.cmd_queue)
}

func (self *MCU_analog_mux) Get_instance_id(select_pins_desc []string) ([]int64, error) {
	ppins := MustLookupPins(self.mcu.Get_printer()) // @see ppins = self.mcu.get_printer().lookup_object("pins")

	select_pin_params := make([]map[string]interface{}, 0, len(select_pins_desc))
	for _, spd := range select_pins_desc {
		select_pin_params = append(select_pin_params, ppins.Parse_pin(spd, true, false))
	}

	for _, pin_params := range select_pin_params {
		if pin_params["chip"] != self.mcu {
			return nil, errors.New("TMC mux pins must be on the same mcu")
		}
	}

	pins := make([]interface{}, 0, len(select_pin_params))
	for _, pp := range select_pin_params {
		pins = append(pins, pp["pin"])
	}

	if !reflect.DeepEqual(pins, self.pins) {
		return nil, errors.New("All TMC mux instances must use identical pins")
	}

	ret := make([]int64, 0, len(select_pin_params))
	for _, pp := range select_pin_params {
		if cast.ToBool(pp["invert"]) == false {
			ret = append(ret, 1)
		} else {
			ret = append(ret, 0)
		}
	}
	return ret, nil
}

func (self *MCU_analog_mux) Activate(instance_id []int64) {
	for i := 0; i < len(self.oids); i++ {
		oid, old, new := self.oids[i], self.pin_values[i], instance_id[i]
		if old != new {
			self.update_pin_cmd.Send([]int64{int64(oid), int64(new)}, 0, 0)
		}
	}
	self.pin_values = instance_id
}

/**
######################################################################
# TMC uart communication
######################################################################

# Share mutexes so only one active tmc_uart command on a single mcu at
# a time. This helps limit cpu usage on slower micro-controllers.

*/

type PrinterTMCUartMutexes struct {
	mcu_to_mutex map[interface{}]interface{}
}

func NewPrinterTMCUartMutexes() *PrinterTMCUartMutexes {
	return &PrinterTMCUartMutexes{
		mcu_to_mutex: make(map[interface{}]interface{}),
	}
}

func lookup_tmc_uart_mutex(mcu *MCU) *ReactorMutex {
	printer := mcu.Get_printer()

	pmutexesObj := printer.Lookup_object("tmc_uart", nil)
	var pmutexes *PrinterTMCUartMutexes
	if value.IsNone(pmutexesObj) {
		pmutexes = NewPrinterTMCUartMutexes()
		printer.Add_object("tmc_uart", pmutexes)
	} else {
		pmutexes = pmutexesObj.(*PrinterTMCUartMutexes)
	}

	mutex := pmutexes.mcu_to_mutex[mcu]
	if value.IsNone(mutex) {
		mutex = printer.Get_reactor().Mutex(false)
		pmutexes.mcu_to_mutex[mcu] = mutex
	}

	return mutex.(*ReactorMutex)
}

const (
	TMC_BAUD_RATE     float64 = 40000
	TMC_BAUD_RATE_AVR float64 = 9000
)

// Code for sending messages on a TMC uart
type MCU_TMC_uart_bitbang struct {
	mcu              *MCU
	mutex            *ReactorMutex
	pullup           interface{}
	rx_pin           interface{}
	tx_pin           interface{}
	oid              int
	cmd_queue        interface{}
	analog_mux       *MCU_analog_mux
	instances        map[string]bool
	tmcuart_send_cmd *CommandQueryWrapper
}

func NewMCU_TMC_uart_bitbang(rx_pin_params, tx_pin_params map[string]interface{}, select_pins_desc []string) *MCU_TMC_uart_bitbang {
	self := new(MCU_TMC_uart_bitbang)
	self.mcu = rx_pin_params["chip"].(*MCU)
	self.mutex = lookup_tmc_uart_mutex(self.mcu)
	self.pullup = rx_pin_params["pullup"]
	self.rx_pin = rx_pin_params["pin"]
	self.tx_pin = tx_pin_params["pin"]

	self.oid = self.mcu.Create_oid()
	self.cmd_queue = self.mcu.Alloc_command_queue()
	self.analog_mux = nil

	if len(select_pins_desc) != 0 {
		self.analog_mux = NewMCU_analog_mux(self.mcu, self.cmd_queue,
			select_pins_desc)
	}

	self.instances = make(map[string]bool)
	self.tmcuart_send_cmd = nil
	self.mcu.Register_config_callback(self.build_config)
	return self
}

func (self *MCU_TMC_uart_bitbang) build_config() {
	baud := TMC_BAUD_RATE
	mcu_type := cast.ToString(self.mcu.Get_constants()["project.MCU"])
	if strings.HasPrefix(mcu_type, "atmega") || strings.HasPrefix(mcu_type, "at90usb") {
		baud = TMC_BAUD_RATE_AVR
	}
	bit_ticks := self.mcu.Seconds_to_clock(1. / baud)

	self.mcu.Add_config_cmd(fmt.Sprintf("config_tmcuart oid=%d rx_pin=%s pull_up=%d tx_pin=%s bit_time=%d",
		self.oid, self.rx_pin, self.pullup, self.tx_pin, bit_ticks), false, false)
	self.tmcuart_send_cmd = self.mcu.Lookup_query_command(
		"tmcuart_send oid=%c write=%*s read=%c",
		"tmcuart_response oid=%c read=%*s", self.oid,
		self.cmd_queue, true)
}

func (self *MCU_TMC_uart_bitbang) register_instance(rx_pin_params, tx_pin_params map[string]interface{},
	select_pins_desc []string, addr int) ([]int64, error) {

	if rx_pin_params["pin"] != self.rx_pin ||
		tx_pin_params["pin"] != self.tx_pin ||
		(value.IsNone(select_pins_desc) != value.IsNone(self.analog_mux)) {
		return nil, errors.New("Shared TMC uarts must use the same pins")
	}

	var instance_id []int64
	if value.IsNotNone(self.analog_mux) {
		instance_id, _ = self.analog_mux.Get_instance_id(select_pins_desc)
	}

	var uniqKey string
	uniqKey = str.JoinSliceWithFormat(instance_id, ",", "%d") + "###" + strconv.Itoa(addr)
	if _, ok := self.instances[uniqKey]; ok {
		return nil, errors.New("Shared TMC uarts need unique address or select_pins polarity")
	}

	self.instances[uniqKey] = true
	return instance_id, nil
}

func (self *MCU_TMC_uart_bitbang) _calc_crc8(data []int64) int64 {
	// Generate a CRC8-ATM value for a bytearray
	var crc int64 = 0
	for _, b := range data {
		// range iterator.RangeInt(8)
		for i := 0; i < 8; i++ {
			if (crc>>7)^(b&0x01) != 0 {
				crc = (crc << 1) ^ 0x07
			} else {
				crc = (crc << 1)
			}
			crc &= 0xff
			b >>= 1
		}
	}
	return crc
}

func (self *MCU_TMC_uart_bitbang) _add_serial_bits(data []int64) []int64 {
	// Add serial start and stop bits to a message in a bytearray
	out := big.NewInt(0)
	var pos uint = 0

	for _, d := range data {
		b := big.NewInt(d)
		b.Lsh(b, uint(1)).Or(b, big.NewInt(int64(0x200)))
		out.Or(out, b.Lsh(b, pos))
		pos += 10
	}

	res := make([]int64, 0)
	end := maths.FloorDiv(int(pos+7), 8)
	for i := 0; i < end; i++ {
		t := big.NewInt(0)
		t.Set(out)
		t = t.Rsh(t, uint(i*8)).And(t, big.NewInt(0xff))
		res = append(res, t.Int64())
	}
	return res
}

func (self *MCU_TMC_uart_bitbang) _encode_read(sync, addr, reg int64) []int64 {
	// Generate a uart read register message
	msg := []int64{sync, addr, reg}
	msg = append(msg, self._calc_crc8(msg))
	return self._add_serial_bits(msg)
}

func (self *MCU_TMC_uart_bitbang) _encode_write(sync, addr, reg, val int64) []int64 {
	// Generate a uart write register message
	msg := []int64{sync, addr, reg, (val >> 24) & 0xff,
		(val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff}
	msg = append(msg, self._calc_crc8(msg))
	return self._add_serial_bits(msg)
}

func (self *MCU_TMC_uart_bitbang) _decode_read(reg int64, data []int64) interface{} {
	// Extract a uart read response message
	if len(data) != 10 {
		return nil
	}
	// Convert data into a long integer for easy manipulation

	mval := big.NewInt(0)
	de := big.NewInt(0)
	var pos uint = 0
	for _, d := range data {
		de.SetUint64(uint64(d))
		de.Lsh(de, pos)
		mval.Or(mval, de)
		pos += 8
	}

	// Extract register value

	ff := big.NewInt(0)
	ff.SetUint64(uint64(0xff))
	t_31 := big.NewInt(0)
	t_31.Set(mval)
	t_31.Rsh(t_31, 31)
	t_31.And(t_31, ff)
	t_31.Lsh(t_31, 24)

	t_41 := big.NewInt(0)
	t_41.Set(mval)
	t_41.Rsh(t_41, 41)
	t_41.And(t_41, ff)
	t_41.Lsh(t_41, 16)

	t_51 := big.NewInt(0)
	t_51.Set(mval)
	t_51.Rsh(t_51, 51)
	t_51.And(t_51, ff)
	t_51.Lsh(t_51, 8)

	t_61 := big.NewInt(0)
	t_61.Set(mval)
	t_61.Rsh(t_61, 61)
	t_61.And(t_61, ff)

	val := t_31.Uint64() | t_41.Uint64() | t_51.Uint64() | t_61.Uint64()

	//logger.Debug(reg,data,val,mval,pos)
	// Verify start/stop bits and crc
	encoded_data := self._encode_write(0x05, 0xff, reg, int64(val))
	if !reflect.DeepEqual(data, encoded_data) {
		return nil
	}
	return val
}

func (self *MCU_TMC_uart_bitbang) reg_read(instance_id []int64, addr, reg int64) interface{} {
	if value.IsNotNone(self.analog_mux) {
		self.analog_mux.Activate(instance_id)
	}

	msg := self._encode_read(0xf5, addr, reg)
	data := []interface{}{int64(self.oid), msg, int64(10)}
	params := self.tmcuart_send_cmd.Send(data, 0, 0)
	read_params := []int64{}
	for _, x := range params.(map[string]interface{})["read"].([]int) {
		read_params = append(read_params, int64(x))
	}
	return self._decode_read(reg, read_params)
}

func (self *MCU_TMC_uart_bitbang) reg_write(instance_id []int64, addr, reg, val int64, _print_time *float64) {
	minclock := int64(0)

	if value.IsNotNone(_print_time) {
		print_time := cast.Float64(_print_time)
		minclock = self.mcu.Print_time_to_clock(print_time)
	}

	if value.IsNotNone(self.analog_mux) {
		self.analog_mux.Activate(instance_id)
	}

	msg := self._encode_write(0xf5, addr, reg|0x80, val)
	data := []interface{}{int64(self.oid), msg, int64(0)}
	self.tmcuart_send_cmd.Send(data, minclock, 0)
}

// Lookup a (possibly shared) tmc uart
func Lookup_tmc_uart_bitbang(config *ConfigWrapper, max_addr int64) ([]int64, int64, *MCU_TMC_uart_bitbang, error) {
	ppins := MustLookupPins(config.Get_printer())
	rx_pin_params := ppins.Lookup_pin(cast.ToString(config.Get("uart_pin", object.Sentinel{}, true)), false, true, "tmc_uart_rx")
	tx_pin_desc := config.Get("tx_pin", value.None, true)
	var tx_pin_params map[string]interface{}
	if value.IsNone(tx_pin_desc) {
		tx_pin_params = rx_pin_params
	} else {
		tx_pin_params = ppins.Lookup_pin(cast.ToString(tx_pin_desc), false, false, "tmc_uart_tx")
	}

	if rx_pin_params["chip"] != tx_pin_params["chip"] {
		return nil, 0, nil, errors.New("TMC uart rx and tx pins must be on the same mcu")
	}

	select_pins_desc := cast.ToStringSlice(config.Getlist("select_pins", value.None, ",", 0, true))

	addr := config.Getint("uart_address", 0, 0, cast.ForceInt(max_addr), true)
	mcu_uart, ok := rx_pin_params["class"]
	if !ok || mcu_uart == nil {
		mcu_uart = NewMCU_TMC_uart_bitbang(rx_pin_params, tx_pin_params,
			select_pins_desc)
		rx_pin_params["class"] = mcu_uart
	}
	instance_id, _ := mcu_uart.(*MCU_TMC_uart_bitbang).register_instance(rx_pin_params, tx_pin_params, select_pins_desc, addr)
	return instance_id, int64(addr), mcu_uart.(*MCU_TMC_uart_bitbang), nil
}

// Helper code for communicating via TMC uart
type MCU_TMC_uart struct {
	printer       *Printer
	name          string
	name_to_reg   map[string]int64
	fields        *FieldHelper
	ifcnt         interface{}
	instance_id   []int64
	addr          int64
	mcu_uart      *MCU_TMC_uart_bitbang
	mutex         *ReactorMutex
	tmc_frequency float64
	switch_addr   int64
}

var _ IMCU_TMC = (*MCU_TMC_uart)(nil)

func NewMCU_TMC_uart(config *ConfigWrapper, name_to_reg map[string]int64, fields *FieldHelper, max_addr int64, tmc_frequency float64) *MCU_TMC_uart {
	self := new(MCU_TMC_uart)
	self.printer = config.Get_printer()
	self.name = str.LastName(config.Get_name())
	self.name_to_reg = name_to_reg

	self.fields = fields
	self.ifcnt = value.None
	self.instance_id, self.addr, self.mcu_uart, _ = Lookup_tmc_uart_bitbang(config, max_addr)
	self.mutex = self.mcu_uart.mutex
	self.tmc_frequency = tmc_frequency
	self.switch_addr = int64(config.Getint("switch_addr", 0, 0, cast.ForceInt(max_addr), true))

	return self
}

func (self *MCU_TMC_uart) Get_fields() *FieldHelper {
	return self.fields
}

func (self *MCU_TMC_uart) _do_get_register(reg_name string) (int64, error) {
	reg := self.name_to_reg[reg_name]
	if value.IsNotNone(self.printer.Get_start_args()["debugoutput"]) {
		return 0, nil
	}

	for i := 0; i < 5; i++ {
		val := self.mcu_uart.reg_read(self.instance_id, self.addr, reg)
		if value.IsNotNone(val) {
			return cast.ToInt64(val), nil
		}
	}

	return -1, fmt.Errorf("Unable to read tmc uart '%s' register %s", self.name, reg_name)

}

func (self *MCU_TMC_uart) Get_register(reg_name string) (int64, error) {
	self.mutex.Lock()
	val, err := self._do_get_register(reg_name)
	defer self.mutex.Unlock()
	return val, err
}

func (self *MCU_TMC_uart) Set_register(reg_name string, val int64, print_time *float64) error {
	reg := self.name_to_reg[reg_name]
	if value.IsNotNone(self.printer.Get_start_args()["debugoutput"]) {
		return nil
	}

	self.mutex.Lock()
	defer self.mutex.Unlock()

	for i := 0; i < 5; i++ {
		ifcnt := self.ifcnt
		if value.IsNone(ifcnt) {
			self.ifcnt, _ = self._do_get_register("IFCNT")
			ifcnt = self.ifcnt
		}

		self.mcu_uart.reg_write(self.instance_id, self.addr, reg, val,
			print_time)

		self.ifcnt, _ = self._do_get_register("IFCNT")
		if self.ifcnt == (cast.ToInt64(ifcnt)+1)&0xff {
			return nil
		}
	}

	panic(fmt.Errorf("Unable to write tmc uart '%s' register %s", self.name, reg_name))
}

func (self *MCU_TMC_uart) get_tmc_frequency() float64 {
	return self.tmc_frequency
}
