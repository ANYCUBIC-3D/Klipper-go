package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/utils/str"
	"k3c/common/value"
	"strings"
)

func resolve_bus_name(mcu *MCU, param string, _bus *string) string {
	// Find enumerations for the given bus
	enumerations := mcu.Get_enumerations()
	bus := cast.String(_bus)
	var enums interface{}
	if _, ok := enumerations[param]; ok {
		enums = enumerations[param]
	} else if _, ok := enumerations["bus"]; ok {
		enums = enumerations["bus"]
	} else { //  if enums is None
		if _bus == nil || *_bus == "" {
			return ""
		}

		return bus
	}

	// Verify bus is a valid enumeration
	ppins := MustLookupPins(mcu.Get_printer())
	mcu_name := mcu.Get_name()
	enumsd := enums.(map[string]interface{})
	if _bus == nil || *_bus == "" {
		rev_enums := make(map[int]string, len(enumsd))
		for k, v := range enumsd {
			rev_enums[v.(int)] = k
		}

		if _, ok := rev_enums[0]; !ok {
			err := fmt.Errorf("Must specify %s on mcu '%s'", param, mcu_name)
			logger.Error(err)
			panic(err)
		}
		bus = rev_enums[0]
	}

	if _, ok := enumsd[bus]; !ok {
		err := fmt.Errorf("Unknown %s '%s'", param, bus)
		logger.Error(err)
		panic(err)
	}

	// Check for reserved bus pins
	constants := mcu.Get_constants()
	reserve_pins, ok := constants[fmt.Sprintf("BUS_PINS_%s", bus)]
	pin_resolver := ppins.Get_pin_resolver(mcu_name)
	if ok {
		for _, pin := range strings.Split(cast.ToString(reserve_pins), ",") {
			pin_resolver.Reserve_pin(pin, bus)
		}
	}

	return bus
}

/**
######################################################################
# SPI
######################################################################
*/

// Helper code for working with devices connected to an project.MCU via an SPI bus
type MCU_SPI struct {
	mcu              *MCU
	bus              string
	oid              int
	config_fmt       string
	cmd_queue        interface{}
	spi_send_cmd     *CommandWrapper
	spi_transfer_cmd *CommandQueryWrapper
}

func NewMCU_SPI(mcu *MCU, bus string, pin interface{}, mode, speed int, sw_pins []interface{},
	cs_active_high bool) *MCU_SPI {
	self := new(MCU_SPI)

	self.mcu = mcu
	self.bus = bus
	// Config SPI object (set all CS pins high before spi_set_bus commands)
	self.oid = mcu.Create_oid()

	if value.IsNone(pin) {
		mcu.Add_config_cmd(fmt.Sprintf("config_spi_without_cs oid=%d", self.oid), false, false)
	} else {
		mcu.Add_config_cmd(fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=%v", self.oid, pin, cs_active_high), false, false)
	}

	// Generate SPI bus config message
	if value.IsNotNone(sw_pins) {
		self.config_fmt = fmt.Sprintf(
			"spi_set_software_bus oid=%d miso_pin=%s mosi_pin=%s sclk_pin=%s mode=%d rate=%d", self.oid, sw_pins[0], sw_pins[1], sw_pins[2], mode, speed)
	} else {
		self.config_fmt = fmt.Sprintf("spi_set_bus oid=%d spi_bus=%%s mode=%d rate=%d", self.oid, mode, speed)
	}

	self.cmd_queue = mcu.Alloc_command_queue()
	mcu.Register_config_callback(self.build_config)
	self.spi_send_cmd = nil
	self.spi_transfer_cmd = nil
	return self
}

func (self *MCU_SPI) setup_shutdown_msg(shutdown_seq []int) {
	shutdown_msg := str.JoinSliceWithFormat(shutdown_seq, "", "%02x")
	self.mcu.Add_config_cmd(fmt.Sprintf(
		"config_spi_shutdown oid=%d spi_oid=%d shutdown_msg=%s", self.mcu.Create_oid(), self.oid, shutdown_msg), false, false)
}

func (self *MCU_SPI) Get_oid() int {
	return self.oid
}

func (self *MCU_SPI) get_mcu() *MCU {
	return self.mcu
}

func (self *MCU_SPI) get_command_queue() interface{} {
	return self.cmd_queue
}

func (self *MCU_SPI) build_config() {
	if strings.Contains(self.config_fmt, "%") {
		bus := resolve_bus_name(self.mcu, "spi_bus", cast.StringP(self.bus))
		self.config_fmt = fmt.Sprintf(self.config_fmt, bus)
	}

	self.mcu.Add_config_cmd(self.config_fmt, false, false)
	self.spi_send_cmd, _ = self.mcu.Lookup_command(
		"spi_send oid=%c data=%*s", self.cmd_queue)

	self.spi_transfer_cmd = self.mcu.Lookup_query_command(
		"spi_transfer oid=%c data=%*s",
		"spi_transfer_response oid=%c response=%*s", self.oid,
		self.cmd_queue, false)
}

func (self *MCU_SPI) Spi_send(data []int, minclock, reqclock int64) {
	if value.IsNone(self.spi_send_cmd) {
		data_msg := str.JoinSliceWithFormat(data, "", "%02x")
		self.mcu.Add_config_cmd(fmt.Sprintf("spi_send oid=%d data=%s",
			self.oid, data_msg), true, false)
		return
	}
	dat := []interface{}{self.oid, data}
	self.spi_send_cmd.Send(dat, minclock, reqclock)
}

func (self *MCU_SPI) Spi_transfer(data []int, minclock, reqclock int64) interface{} {
	dat := []interface{}{self.oid, data}
	return self.spi_transfer_cmd.Send(dat, minclock, reqclock)
}

func (self *MCU_SPI) Spi_transfer_with_preface(preface_data, data []int,
	minclock, reqclock int64) interface{} {

	return self.spi_transfer_cmd.Send_with_preface(self.spi_send_cmd,
		[]interface{}{self.oid, preface_data},
		[]interface{}{self.oid, data}, minclock, reqclock)
}

// Helper to setup an spi bus from settings in a config section

func MCU_SPI_from_config(config *ConfigWrapper, mode int, pin_option string,
	default_speed int, share_type interface{},
	cs_active_high bool) (*MCU_SPI, error) {
	ppins := MustLookupPins(config.Get_printer())
	cs_pin := cast.ToString(config.Get(pin_option, object.Sentinel{}, true))
	cs_pin_params := ppins.Lookup_pin(cs_pin, false, false, share_type)
	pin := cs_pin_params["pin"]
	if cast.ToString(pin) == "None" {
		ppins.Reset_pin_sharing(cs_pin_params)
		pin = nil
	}

	// Load bus parameters
	mcu := cs_pin_params["chip"]
	speed := config.Getint("spi_speed", default_speed, 100000, 0, true)
	var bus interface{}
	var sw_pins []interface{}
	if value.IsNotNone(config.Get("spi_software_sclk_pin", nil, true)) {
		var sw_pin_names []string
		var sw_pin_params []map[string]interface{}
		for _, name := range []string{"miso", "mosi", "sclk"} {
			sw_pin_names = append(sw_pin_names, fmt.Sprintf("spi_software_%s_pin", name))
		}
		for _, name := range sw_pin_names {
			tmp := ppins.Lookup_pin(cast.ToString(config.Get(name, object.Sentinel{}, true)), false, false, share_type)
			sw_pin_params = append(sw_pin_params, tmp)
		}

		for _, pin_params := range sw_pin_params {
			if pin_params["chip"] != mcu {
				return nil, fmt.Errorf("%s: spi pins must be on same mcu",
					config.Get_name())
			}
		}
		// sw_pins = tuple([pin_params['pin'] for pin_params in sw_pin_params])
		sw_pins = make([]interface{}, 0)
		for _, pin_params := range sw_pin_params {
			_pin, ok := pin_params["pin"].(string)
			if ok {
				sw_pins = append(sw_pins, _pin)
			} else {
				logger.Debug("pin_params[\"pin\"] type should be []string")
			}
		}
		bus = nil
	} else {
		bus = config.Get("spi_bus", value.None, true)
		sw_pins = nil
	}

	// Create MCU_SPI object

	return NewMCU_SPI(mcu.(*MCU), cast.ToString(bus), pin, mode, speed, sw_pins, cs_active_high), nil
}

/*
######################################################################
# I2C
######################################################################
*/

// Helper code for working with devices connected to an project.MCU via an I2C bus
type MCU_I2C struct {
	mcu                 *MCU
	bus                 string
	i2c_address         int
	oid                 int
	config_fmt          string
	cmd_queue           interface{}
	i2c_write_cmd       interface{}
	i2c_read_cmd        interface{}
	i2c_modify_bits_cmd interface{}
}

func NewMCU_I2C(mcu *MCU, bus string, addr, speed int) *MCU_I2C {
	self := new(MCU_I2C)
	self.mcu = mcu
	self.bus = bus
	self.i2c_address = addr

	self.oid = self.mcu.Create_oid()
	self.config_fmt = fmt.Sprintf("config_i2c oid=%d i2c_bus=%%s rate=%d address=%d", self.oid, speed, addr)
	self.cmd_queue = self.mcu.Alloc_command_queue()

	self.mcu.Register_config_callback(self.build_config)
	self.i2c_write_cmd = nil
	self.i2c_read_cmd = nil
	self.i2c_modify_bits_cmd = nil
	return self
}

func (self *MCU_I2C) Get_oid() int {
	return self.oid
}

func (self *MCU_I2C) Get_mcu() *MCU {
	return self.mcu
}

func (self *MCU_I2C) Get_i2c_address() int {
	return self.i2c_address
}

func (self *MCU_I2C) Get_command_queue() interface{} {
	return self.cmd_queue
}

func (self *MCU_I2C) build_config() {
	bus := resolve_bus_name(self.mcu, "i2c_bus", cast.StringP(self.bus))
	self.mcu.Add_config_cmd(fmt.Sprintf(self.config_fmt, bus), false, false)
	self.i2c_write_cmd, _ = self.mcu.Lookup_command(
		"i2c_write oid=%c data=%*s", self.cmd_queue)
	self.i2c_read_cmd = self.mcu.Lookup_query_command(
		"i2c_read oid=%c reg=%*s read_len=%u",
		"i2c_read_response oid=%c response=%*s", self.oid,
		self.cmd_queue, false)
	self.i2c_modify_bits_cmd, _ = self.mcu.Lookup_command(
		"i2c_modify_bits oid=%c reg=%*s clear_set_bits=%*s",
		self.cmd_queue)
}

func (self *MCU_I2C) I2c_write(data []int, minclock, reqclock int64) {
	if value.IsNone(self.i2c_write_cmd) {
		// Send setup message via mcu initialization
		data_msg := str.JoinSliceWithFormat(data, "", "%02x")
		self.mcu.Add_config_cmd(fmt.Sprintf("i2c_write oid=%d data=%s",
			self.oid, data_msg), true, false)
		return
	}

	dat := []interface{}{self.oid, data}
	self.i2c_write_cmd.(*CommandWrapper).Send(dat, minclock, reqclock)
	return
}

func (self *MCU_I2C) I2c_read(write []int, read_len int) interface{} {
	dat := []interface{}{self.oid, write, read_len}
	return self.i2c_read_cmd.(*CommandQueryWrapper).Send(dat, 0, 0)
}

func (self *MCU_I2C) I2c_modify_bits(reg, clear_bits, set_bits string,
	minclock, reqclock int64) {
	clearset := clear_bits + set_bits
	if value.IsNone(self.i2c_modify_bits_cmd) {
		// Send setup message via mcu initialization
		reg_msg := str.JoinSliceWithFormat([]rune(reg), "", "%02x")
		clearset_msg := str.JoinSliceWithFormat([]rune(clearset), "", "%02x")
		cmd := fmt.Sprintf("i2c_modify_bits oid=%d reg=%s clear_set_bits=%s", self.oid, reg_msg, clearset_msg)
		self.mcu.Add_config_cmd(cmd, true, true)
		return
	}

	self.i2c_modify_bits_cmd.(*CommandWrapper).Send([]int64{int64(self.oid), int64(len(reg)), int64(len(clearset))},
		minclock, reqclock)
}

func MCU_I2C_from_config(config *ConfigWrapper, default_addr *int, default_speed int) *MCU_I2C {
	printer := config.Get_printer()
	i2c_mcu := Get_printer_mcu(printer, cast.ToString(config.Get("i2c_mcu", "mcu", true)))
	speed := config.Getint("i2c_speed", default_speed, 100000, 0, true)
	bus := cast.ToString(config.Get("i2c_bus", nil, true))

	var addr int
	if value.IsNone(default_addr) {
		addr = config.Getint("i2c_address", 0, 0, 127, true)
	} else {
		addr = config.Getint("i2c_address", default_addr, 0, 127, true)
	}

	return NewMCU_I2C(i2c_mcu, bus, addr, speed)
}
