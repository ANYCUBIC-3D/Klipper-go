package project

import (
	"errors"
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
)

const TMC2130_TMC_FREQUENCY = 13200000.

var TMC2130_Registers = map[string]int64{
	"GCONF": 0x00, "GSTAT": 0x01, "IOIN": 0x04, "IHOLD_IRUN": 0x10,
	"TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13, "TCOOLTHRS": 0x14,
	"THIGH": 0x15, "XDIRECT": 0x2d, "MSLUT0": 0x60, "MSLUTSEL": 0x68,
	"MSLUTSTART": 0x69, "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c,
	"COOLCONF": 0x6d, "DCCTRL": 0x6e, "DRV_STATUS": 0x6f, "PWMCONF": 0x70,
	"PWM_SCALE": 0x71, "ENCM_CTRL": 0x72, "LOST_STEPS": 0x73,
}

var TMC2130_ReadRegisters = []string{
	"GCONF", "GSTAT", "IOIN", "TSTEP", "XDIRECT", "MSCNT", "MSCURACT",
	"CHOPCONF", "DRV_STATUS", "PWM_SCALE", "LOST_STEPS",
}

var TMC2130_Fields = make(map[string]map[string]int64)

func init() {
	TMC2130_Fields["GCONF"] = map[string]int64{
		"i_scale_analog": 1 << 0, "internal_rsense": 1 << 1, "en_pwm_mode": 1 << 2,
		"enc_commutation": 1 << 3, "shaft": 1 << 4, "diag0_error": 1 << 5,
		"diag0_otpw": 1 << 6, "diag0_stall": 1 << 7, "diag1_stall": 1 << 8,
		"diag1_index": 1 << 9, "diag1_onstate": 1 << 10, "diag1_steps_skipped": 1 << 11,
		"diag0_int_pushpull": 1 << 12, "diag1_pushpull": 1 << 13,
		"small_hysteresis": 1 << 14, "stop_enable": 1 << 15, "direct_mode": 1 << 16,
		"test_mode": 1 << 17,
	}

	TMC2130_Fields["GSTAT"] = map[string]int64{"reset": 1 << 0, "drv_err": 1 << 1, "uv_cp": 1 << 2}
	TMC2130_Fields["IOIN"] = map[string]int64{
		"step": 1 << 0, "dir": 1 << 1, "dcen_cfg4": 1 << 2, "dcin_cfg5": 1 << 3,
		"drv_enn_cfg6": 1 << 4, "dco": 1 << 5, "version": 0xff << 24,
	}
	TMC2130_Fields["IHOLD_IRUN"] = map[string]int64{
		"ihold": 0x1f << 0, "irun": 0x1f << 8, "iholddelay": 0x0f << 16,
	}

	TMC2130_Fields["TPOWERDOWN"] = map[string]int64{"tpowerdown": 0xff}
	TMC2130_Fields["TSTEP"] = map[string]int64{"tstep": 0xfffff}
	TMC2130_Fields["TPWMTHRS"] = map[string]int64{"tpwmthrs": 0xfffff}
	TMC2130_Fields["TCOOLTHRS"] = map[string]int64{"tcoolthrs": 0xfffff}
	TMC2130_Fields["THIGH"] = map[string]int64{"thigh": 0xfffff}
	TMC2130_Fields["MSLUT0"] = map[string]int64{"mslut0": 0xffffffff}
	TMC2130_Fields["MSLUT1"] = map[string]int64{"mslut1": 0xffffffff}
	TMC2130_Fields["MSLUT2"] = map[string]int64{"mslut2": 0xffffffff}
	TMC2130_Fields["MSLUT3"] = map[string]int64{"mslut3": 0xffffffff}
	TMC2130_Fields["MSLUT4"] = map[string]int64{"mslut4": 0xffffffff}
	TMC2130_Fields["MSLUT5"] = map[string]int64{"mslut5": 0xffffffff}
	TMC2130_Fields["MSLUT6"] = map[string]int64{"mslut6": 0xffffffff}
	TMC2130_Fields["MSLUT7"] = map[string]int64{"mslut7": 0xffffffff}
	TMC2130_Fields["MSLUTSEL"] = map[string]int64{
		"x3": 0xFF << 24,
		"x2": 0xFF << 16,
		"x1": 0xFF << 8,
		"w3": 0x03 << 6,
		"w2": 0x03 << 4,
		"w1": 0x03 << 2,
		"w0": 0x03 << 0,
	}
	TMC2130_Fields["MSLUTSTART"] = map[string]int64{
		"start_sin":   0xFF << 0,
		"start_sin90": 0xFF << 16,
	}
	TMC2130_Fields["MSCNT"] = map[string]int64{"mscnt": 0x3ff}
	TMC2130_Fields["MSCURACT"] = map[string]int64{"cur_a": 0x1ff, "cur_b": 0x1ff << 16}
	TMC2130_Fields["CHOPCONF"] = map[string]int64{
		"toff": 0x0f, "hstrt": 0x07 << 4, "hend": 0x0f << 7, "fd3": 1 << 11,
		"disfdcc": 1 << 12, "rndtf": 1 << 13, "chm": 1 << 14, "tbl": 0x03 << 15,
		"vsense": 1 << 17, "vhighfs": 1 << 18, "vhighchm": 1 << 19, "sync": 0x0f << 20,
		"mres": 0x0f << 24, "intpol": 1 << 28, "dedge": 1 << 29, "diss2g": 1 << 30,
	}
	TMC2130_Fields["COOLCONF"] = map[string]int64{
		"semin": 0x0f, "seup": 0x03 << 5, "semax": 0x0f << 8, "sedn": 0x03 << 13,
		"seimin": 1 << 15, "sgt": 0x7f << 16, "sfilt": 1 << 24,
	}
	TMC2130_Fields["DRV_STATUS"] = map[string]int64{
		"sg_result": 0x3ff, "fsactive": 1 << 15, "cs_actual": 0x1f << 16,
		"stallguard": 1 << 24, "ot": 1 << 25, "otpw": 1 << 26, "s2ga": 1 << 27,
		"s2gb": 1 << 28, "ola": 1 << 29, "olb": 1 << 30, "stst": 1 << 31,
	}
	TMC2130_Fields["PWMCONF"] = map[string]int64{
		"pwm_ampl": 0xff, "pwm_grad": 0xff << 8, "pwm_freq": 0x03 << 16,
		"pwm_autoscale": 1 << 18, "pwm_symmetric": 1 << 19, "freewheel": 0x03 << 20,
	}
	TMC2130_Fields["PWM_SCALE"] = map[string]int64{"pwm_scale": 0xff}
	TMC2130_Fields["LOST_STEPS"] = map[string]int64{"lost_steps": 0xfffff}
}

var TMC2130_SignedFields = []string{"cur_a", "cur_b", "sgt"}

var TMC2130_FieldFormatters = map[string]func(interface{}) string{
	"i_scale_analog": func(v interface{}) string {
		if value.True(v) {
			return "1(ExtVREF)"
		}
		return ""
	},
	"shaft": func(v interface{}) string {
		if value.True(v) {
			return "1(Reverse)"
		}
		return ""
	},
	"reset": func(v interface{}) string {
		if value.True(v) {
			return "1(Reset)"
		}
		return ""
	},
	"drv_err": func(v interface{}) string {
		if value.True(v) {
			return "1(ErrorShutdown!)"
		}
		return ""
	},
	"uv_cp": func(v interface{}) string {
		if value.True(v) {
			return "1(Undervoltage!)"
		}
		return ""
	},
	"version": func(v interface{}) string { return fmt.Sprintf("%#x", v) },
	"mres":    func(v interface{}) string { return fmt.Sprintf("%d(%dusteps)", v, 0x100>>cast.ToInt(v)) },
	"otpw": func(v interface{}) string {
		if value.True(v) {
			return "1(OvertempWarning!)"
		}
		return ""
	},
	"ot": func(v interface{}) string {
		if value.True(v) {
			return "1(OvertempError!)"
		}
		return ""
	},
	"s2ga": func(v interface{}) string {
		if value.True(v) {
			return "1(ShortToGND_A!)"
		}
		return ""
	},
	"s2gb": func(v interface{}) string {
		if value.True(v) {
			return "1(ShortToGND_B!)"
		}
		return ""
	},
	"ola": func(v interface{}) string {
		if value.True(v) {
			return "1(OpenLoad_A!)"
		}
		return ""
	},
	"olb": func(v interface{}) string {
		if value.True(v) {
			return "1(OpenLoad_B!)"
		}
		return ""
	},
	"cs_actual": func(v interface{}) string {
		if value.True(v) {
			return fmt.Sprintf("%d", v)
		}
		return "0(Reset?)"
	},
}

/*
######################################################################
# TMC stepper current config helper
######################################################################
*/

const MAX_CURRENT = 2.000

type TMCCurrentHelper struct {
	printer          *Printer
	name             string
	mcu_tmc          IMCU_TMC
	fields           *FieldHelper
	req_hold_current float64
	sense_resistor   float64
}

var _ ICurrentHelper = (*TMCCurrentHelper)(nil)

func NewTMCCurrentHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMCCurrentHelper {
	self := new(TMCCurrentHelper)
	self.printer = config.Get_printer()
	self.name = str.LastName(config.Get_name())
	self.mcu_tmc = mcu_tmc

	self.fields = mcu_tmc.Get_fields()
	run_current := config.Getfloat("run_current", 0, 0., MAX_CURRENT, 0, 0, true)
	hold_current := config.Getfloat("hold_current", MAX_CURRENT, 0., MAX_CURRENT, 0, 0, true)
	self.req_hold_current = hold_current
	self.sense_resistor = config.Getfloat("sense_resistor", 0.110, 0., 0, 0, 0, true)
	vsense, irun, ihold := self._calc_current(run_current, hold_current)
	self.fields.Set_field("vsense", vsense, value.None, nil)
	self.fields.Set_field("ihold", ihold, value.None, nil)
	self.fields.Set_field("irun", irun, value.None, nil)
	return self
}

func (self *TMCCurrentHelper) _calc_current_bits(current float64, vsense bool) int {
	sense_resistor := self.sense_resistor + 0.020
	vref := 0.32
	if vsense {
		vref = 0.18
	}

	cs := int(32.*sense_resistor*current*math.Sqrt(2.)/vref+.5) - 1
	return maths.Max(0, maths.Min(31, cs))
}

func (self *TMCCurrentHelper) _calc_current_from_bits(cs float64, vsense bool) float64 {
	sense_resistor := self.sense_resistor + 0.020
	vref := 0.32
	if vsense {
		vref = 0.18
	}

	return (cs + 1) * vref / (32. * sense_resistor * math.Sqrt(2.))
}

func (self *TMCCurrentHelper) _calc_current(run_current, hold_current float64) (bool, int, int) {
	vsense := true
	irun := self._calc_current_bits(run_current, true)
	if irun == 31 {
		cur := self._calc_current_from_bits(float64(irun), true)
		if cur < run_current {
			irun2 := self._calc_current_bits(run_current, false)
			cur2 := self._calc_current_from_bits(float64(irun2), false)
			if math.Abs(run_current-cur2) < math.Abs(run_current-cur) {
				vsense = false
				irun = irun2
			}
		}
	}

	ihold := self._calc_current_bits(math.Min(hold_current, run_current), vsense)
	return vsense, irun, ihold
}

func (self *TMCCurrentHelper) Get_current() []float64 {
	irun := self.fields.Get_field("irun", value.None, nil)
	ihold := self.fields.Get_field("ihold", value.None, nil)
	vsense := self.fields.Get_field("vsense", value.None, nil)
	run_current := self._calc_current_from_bits(float64(irun), cast.ToBool(vsense))
	hold_current := self._calc_current_from_bits(float64(ihold), cast.ToBool(vsense))
	return []float64{run_current, hold_current, self.req_hold_current, MAX_CURRENT}
}

func (self *TMCCurrentHelper) Set_current(run_current, hold_current, print_time float64) {
	self.req_hold_current = hold_current
	vsense, irun, ihold := self._calc_current(run_current, hold_current)
	if vsense != cast.ToBool(self.fields.Get_field("vsense", value.None, nil)) {
		val := self.fields.Set_field("vsense", vsense, value.None, nil)
		self.mcu_tmc.Set_register("CHOPCONF", val, cast.Float64P(print_time))
	}

	self.fields.Set_field("ihold", ihold, value.None, nil)
	val := self.fields.Set_field("irun", irun, value.None, nil)
	self.mcu_tmc.Set_register("IHOLD_IRUN", val, cast.Float64P(print_time))
}

/**
######################################################################
# TMC2130 SPI
######################################################################
*/

type MCU_TMC_SPI_chain struct {
	printer               *Printer
	chain_len             int64
	mutex                 *ReactorMutex
	spi                   *MCU_SPI
	taken_chain_positions []interface{}
}

func NewMCU_TMC_SPI_chain(config *ConfigWrapper, chain_len int64) *MCU_TMC_SPI_chain {
	self := new(MCU_TMC_SPI_chain)

	self.printer = config.Get_printer()
	self.chain_len = chain_len
	self.mutex = self.printer.Get_reactor().Mutex(false)
	var share = value.None
	if chain_len > 1 {
		share = "tmc_spi_cs"
	}
	self.spi, _ = MCU_SPI_from_config(config, 3, "", 4000000, share, false)
	self.taken_chain_positions = make([]interface{}, 0)
	return self
}

func (self *MCU_TMC_SPI_chain) _build_cmd(data []int64, chain_pos int64) []int {
	cmd := make([]int, 0)
	for i := 0; i < int(self.chain_len-chain_pos)*5; i++ {
		cmd = append(cmd, 0x00)
	}

	for _, d := range data {
		cmd = append(cmd, cast.ForceInt(d))
	}

	for i := 0; i < int(chain_pos-1)*5; i++ {
		cmd = append(cmd, 0x00)
	}
	return cmd
}

func (self *MCU_TMC_SPI_chain) Reg_read(reg, chain_pos int64) int64 {
	cmd := self._build_cmd([]int64{reg, 0x00, 0x00, 0x00, 0x00}, chain_pos)
	self.spi.Spi_send(cmd, 0, 0)

	if value.IsNotNone(self.printer.Get_start_args()["debugoutput"]) {
		return 0
	}

	params := self.spi.Spi_transfer(cmd, 0, 0).(map[string]interface{})

	pr := []rune(cast.ToString(params["response"]))
	pr = pr[(self.chain_len-chain_pos)*5 : (self.chain_len-chain_pos+1)*5]
	return int64((pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4])
}

func (self *MCU_TMC_SPI_chain) Reg_write(reg, val, chain_pos int64, print_time *float64) int64 {
	minclock := int64(0)

	if value.IsNone(print_time) {
		minclock = self.spi.get_mcu().Print_time_to_clock(cast.Float64(print_time))
	}
	data := []int64{(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
		(val >> 8) & 0xff, val & 0xff}

	if value.IsNotNone(self.printer.Get_start_args()["debugoutput"]) {
		self.spi.Spi_send(self._build_cmd(data, chain_pos), minclock, 0)
		return val
	}

	write_cmd := self._build_cmd(data, chain_pos)
	dummy_read := self._build_cmd([]int64{0x00, 0x00, 0x00, 0x00, 0x00}, chain_pos)
	params := self.spi.Spi_transfer_with_preface(write_cmd, dummy_read, minclock, 0)

	pr := []rune(cast.ToString(params.(map[string]interface{})["response"]))
	pr = pr[(self.chain_len-chain_pos)*5 : (self.chain_len-chain_pos+1)*5]
	return int64((pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4])
}

// Helper to setup an spi daisy chain bus from settings in a config section
func Lookup_tmc_spi_chain(config *ConfigWrapper) (*MCU_TMC_SPI_chain, int64) {
	_chain_len := config.GetintNone("chain_length", value.None, 2, 0, true)

	if value.IsNone(_chain_len) {
		// Simple, non daisy chained SPI connection
		return NewMCU_TMC_SPI_chain(config, 1), 1
	}

	chain_len := cast.ToInt64(_chain_len)
	// Shared SPI bus - lookup existing MCU_TMC_SPI_chain
	ppins := MustLookupPins(config.Get_printer())
	cs_pin_params := ppins.Lookup_pin(cast.ToString(config.Get("cs_pin", object.Sentinel{}, true)), false, false, "tmc_spi_cs")
	tmc_spi := cs_pin_params["class"]
	if value.IsNone(tmc_spi) {
		cs_pin_params["class"] = NewMCU_TMC_SPI_chain(config, chain_len)
		tmc_spi = cs_pin_params["class"]
	}

	if chain_len != tmc_spi.(*MCU_TMC_SPI_chain).chain_len {
		panic(errors.New("TMC SPI chain must have same length"))
	}

	chain_pos := config.Getint("chain_position", 0, 1, cast.ForceInt(chain_len), true)
	taken_chain_positions := tmc_spi.(*MCU_TMC_SPI_chain).taken_chain_positions
	for _, p := range taken_chain_positions {
		if cast.ToInt(p) == chain_pos { // p = "123" or p = 123
			panic(errors.New("TMC SPI chain can not have duplicate position"))
		}
	}
	tmc_spi.(*MCU_TMC_SPI_chain).taken_chain_positions = append(tmc_spi.(*MCU_TMC_SPI_chain).taken_chain_positions, chain_pos)

	return tmc_spi.(*MCU_TMC_SPI_chain), int64(chain_pos)
}

// Helper code for working with TMC devices via SPI
type MCU_TMC_SPI struct {
	printer       *Printer
	name          string
	tmc_spi       *MCU_TMC_SPI_chain
	chain_pos     int64
	name_to_reg   map[string]int64
	fields        *FieldHelper
	mutex         *ReactorMutex
	tmc_frequency float64
}

var _ IMCU_TMC = (*MCU_TMC_SPI)(nil)

func NewMCU_TMC_SPI(config *ConfigWrapper, name_to_reg map[string]int64, fields *FieldHelper, tmc_frequency float64) *MCU_TMC_SPI {
	self := new(MCU_TMC_SPI)
	self.printer = config.Get_printer()
	self.name = str.LastName(config.Get_name())
	self.tmc_spi, self.chain_pos = Lookup_tmc_spi_chain(config)
	self.mutex = self.tmc_spi.mutex
	self.name_to_reg = name_to_reg
	self.fields = fields
	self.tmc_frequency = tmc_frequency
	return self
}

func (self *MCU_TMC_SPI) Get_fields() *FieldHelper {
	return self.fields
}

func (self *MCU_TMC_SPI) Get_register(reg_name string) (int64, error) {
	reg := self.name_to_reg[reg_name]
	self.mutex.Lock()
	defer self.mutex.Unlock()
	read := self.tmc_spi.Reg_read(reg, self.chain_pos)
	return read, nil
}

func (self *MCU_TMC_SPI) Set_register(reg_name string, val int64, print_time *float64) error {
	reg := self.name_to_reg[reg_name]
	for i := 0; i < 5; i++ {
		v := self.tmc_spi.Reg_write(reg, val, self.chain_pos, print_time)
		if v == val {
			return nil
		}
	}
	return fmt.Errorf("Unable to write tmc spi '%s' register %s", self.name, reg_name)
}

func (self *MCU_TMC_SPI) Get_tmc_frequency() float64 {
	return self.tmc_frequency
}

/**
######################################################################
# TMC2130 printer object
######################################################################
*/

type TMC2130 struct {
	fields           *FieldHelper
	mcu_tmc          *MCU_TMC_SPI
	get_phase_offset func() (*int, int)
	Get_status       func(eventtime float64) map[string]interface{}
}

func NewTMC2130(config *ConfigWrapper) *TMC2130 {
	self := new(TMC2130)
	// Setup mcu communication
	self.fields = NewFieldHelper(TMC2130_Fields, TMC2130_SignedFields, TMC2130_FieldFormatters, nil)
	self.mcu_tmc = NewMCU_TMC_SPI(config, TMC2130_Registers, self.fields, TMC2130_TMC_FREQUENCY)

	// Allow virtual pins to be created
	NewTMCVirtualPinHelper(config, self.mcu_tmc)

	current_helper := NewTMCCurrentHelper(config, self.mcu_tmc)
	cmdhelper := NewTMCCommandHelper(config, self.mcu_tmc, current_helper)
	cmdhelper.Setup_register_dump(TMC2130_ReadRegisters, nil)

	self.get_phase_offset = cmdhelper.Get_phase_offset
	self.Get_status = cmdhelper.Get_status
	// Setup basic register values
	TMCStealthchopHelper(config, self.mcu_tmc, TMC2130_TMC_FREQUENCY)
	//  Allow other registers to be set from the config
	set_config_field := self.fields.Set_config_field
	//# CHOPCONF
	set_config_field(config, "toff", 4)
	set_config_field(config, "hstrt", 0)
	set_config_field(config, "hend", 7)
	set_config_field(config, "tbl", 1)
	//# COOLCONF
	set_config_field(config, "sgt", 0)
	//# IHOLDIRUN
	set_config_field(config, "iholddelay", 8)
	//# PWMCONF
	set_config_field(config, "pwm_ampl", 128)
	set_config_field(config, "pwm_grad", 4)
	set_config_field(config, "pwm_freq", 1)
	set_config_field(config, "pwm_autoscale", true)
	//# TPOWERDOWN
	set_config_field(config, "tpowerdown", 0)
	return self
}

func Load_config_TMC2130(config *ConfigWrapper) interface{} {
	return NewTMC2130(config)
}
