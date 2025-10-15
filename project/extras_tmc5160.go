package project

import (
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
)

const TMC5160_TMC_FREQUENCY = 12000000.

var TMC5160_Registers = map[string]int64{
	"GCONF":         0x00,
	"GSTAT":         0x01,
	"IFCNT":         0x02,
	"SLAVECONF":     0x03,
	"IOIN":          0x04,
	"X_COMPARE":     0x05,
	"OTP_READ":      0x07,
	"FACTORY_CONF":  0x08,
	"SHORT_CONF":    0x09,
	"DRV_CONF":      0x0A,
	"GLOBALSCALER":  0x0B,
	"OFFSET_READ":   0x0C,
	"IHOLD_IRUN":    0x10,
	"TPOWERDOWN":    0x11,
	"TSTEP":         0x12,
	"TPWMTHRS":      0x13,
	"TCOOLTHRS":     0x14,
	"THIGH":         0x15,
	"RAMPMODE":      0x20,
	"XACTUAL":       0x21,
	"VACTUAL":       0x22,
	"VSTART":        0x23,
	"A1":            0x24,
	"V1":            0x25,
	"AMAX":          0x26,
	"VMAX":          0x27,
	"DMAX":          0x28,
	"D1":            0x2A,
	"VSTOP":         0x2B,
	"TZEROWAIT":     0x2C,
	"XTARGET":       0x2D,
	"VDCMIN":        0x33,
	"SW_MODE":       0x34,
	"RAMP_STAT":     0x35,
	"XLATCH":        0x36,
	"ENCMODE":       0x38,
	"X_ENC":         0x39,
	"ENC_CONST":     0x3A,
	"ENC_STATUS":    0x3B,
	"ENC_LATCH":     0x3C,
	"ENC_DEVIATION": 0x3D,
	"MSLUT0":        0x60,
	"MSLUT1":        0x61,
	"MSLUT2":        0x62,
	"MSLUT3":        0x63,
	"MSLUT4":        0x64,
	"MSLUT5":        0x65,
	"MSLUT6":        0x66,
	"MSLUT7":        0x67,
	"MSLUTSEL":      0x68,
	"MSLUTSTART":    0x69,
	"MSCNT":         0x6A,
	"MSCURACT":      0x6B,
	"CHOPCONF":      0x6C,
	"COOLCONF":      0x6D,
	"DCCTRL":        0x6E,
	"DRV_STATUS":    0x6F,
	"PWMCONF":       0x70,
	"PWM_SCALE":     0x71,
	"PWM_AUTO":      0x72,
	"LOST_STEPS":    0x73,
}

var TMC5160_ReadRegisters = []string{
	"GCONF", "CHOPCONF", "GSTAT", "DRV_STATUS", "FACTORY_CONF", "IOIN",
	"LOST_STEPS", "MSCNT", "MSCURACT", "OTP_READ", "PWM_SCALE",
	"PWM_AUTO", "TSTEP",
}

var TMC5160_Fields = map[string]map[string]int64{
	"COOLCONF": {
		"semin":  0x0F << 0,
		"seup":   0x03 << 5,
		"semax":  0x0F << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
		"sgt":    0x7F << 16,
		"sfilt":  0x01 << 24,
	},
	"CHOPCONF": {
		"toff":     0x0F << 0,
		"hstrt":    0x07 << 4,
		"hend":     0x0F << 7,
		"fd3":      0x01 << 11,
		"disfdcc":  0x01 << 12,
		"chm":      0x01 << 14,
		"tbl":      0x03 << 15,
		"vhighfs":  0x01 << 18,
		"vhighchm": 0x01 << 19,
		"tpfd":     0x0F << 20, // midrange resonances
		"mres":     0x0F << 24,
		"intpol":   0x01 << 28,
		"dedge":    0x01 << 29,
		"diss2g":   0x01 << 30,
		"diss2vs":  0x01 << 31,
	},
	"DRV_STATUS": {
		"sg_result":  0x3FF << 0,
		"s2vsa":      0x01 << 12,
		"s2vsb":      0x01 << 13,
		"stealth":    0x01 << 14,
		"fsactive":   0x01 << 15,
		"csactual":   0xFF << 16,
		"stallguard": 0x01 << 24,
		"ot":         0x01 << 25,
		"otpw":       0x01 << 26,
		"s2ga":       0x01 << 27,
		"s2gb":       0x01 << 28,
		"ola":        0x01 << 29,
		"olb":        0x01 << 30,
		"stst":       0x01 << 31,
	},
	"FACTORY_CONF": {
		"factory_conf": 0x1F << 0,
	},
	"GCONF": {
		"recalibrate":            0x01 << 0,
		"faststandstill":         0x01 << 1,
		"en_pwm_mode":            0x01 << 2,
		"multistep_filt":         0x01 << 3,
		"shaft":                  0x01 << 4,
		"diag0_error":            0x01 << 5,
		"diag0_otpw":             0x01 << 6,
		"diag0_stall":            0x01 << 7,
		"diag1_stall":            0x01 << 8,
		"diag1_index":            0x01 << 9,
		"diag1_onstate":          0x01 << 10,
		"diag1_steps_skipped":    0x01 << 11,
		"diag0_int_pushpull":     0x01 << 12,
		"diag1_poscomp_pushpull": 0x01 << 13,
		"small_hysteresis":       0x01 << 14,
		"stop_enable":            0x01 << 15,
		"direct_mode":            0x01 << 16,
		"test_mode":              0x01 << 17,
	},
	"GSTAT": {
		"reset":   0x01 << 0,
		"drv_err": 0x01 << 1,
		"uv_cp":   0x01 << 2,
	},
	"GLOBALSCALER": {
		"globalscaler": 0xFF << 0,
	},
	"IHOLD_IRUN": {
		"ihold":      0x1F << 0,
		"irun":       0x1F << 8,
		"iholddelay": 0x0F << 16,
	},
	"IOIN": {
		"refl_step":      0x01 << 0,
		"refr_dir":       0x01 << 1,
		"encb_dcen_cfg4": 0x01 << 2,
		"enca_dcin_cfg5": 0x01 << 3,
		"drv_enn":        0x01 << 4,
		"enc_n_dco_cfg6": 0x01 << 5,
		"sd_mode":        0x01 << 6,
		"swcomp_in":      0x01 << 7,
		"version":        0xFF << 24,
	},
	"LOST_STEPS": {
		"lost_steps": 0xfffff << 0,
	},
	"MSLUT0": {"mslut0": 0xffffffff},
	"MSLUT1": {"mslut1": 0xffffffff},
	"MSLUT2": {"mslut2": 0xffffffff},
	"MSLUT3": {"mslut3": 0xffffffff},
	"MSLUT4": {"mslut4": 0xffffffff},
	"MSLUT5": {"mslut5": 0xffffffff},
	"MSLUT6": {"mslut6": 0xffffffff},
	"MSLUT7": {"mslut7": 0xffffffff},
	"MSLUTSEL": {
		"x3": 0xFF << 24,
		"x2": 0xFF << 16,
		"x1": 0xFF << 8,
		"w3": 0x03 << 6,
		"w2": 0x03 << 4,
		"w1": 0x03 << 2,
		"w0": 0x03 << 0,
	},
	"MSCNT": {
		"mscnt": 0x3ff << 0,
	},
	"MSCURACT": {
		"cur_a": 0x1ff << 0,
		"cur_b": 0x1ff << 16,
	},
	"OTP_READ": {
		"otp_fclktrim": 0x1f << 0,
		"otp_s2_level": 0x01 << 5,
		"otp_bbm":      0x01 << 6,
		"otp_tbl":      0x01 << 7,
	},
	"PWM_AUTO": {
		"pwm_ofs_auto":  0xff << 0,
		"pwm_grad_auto": 0xff << 16,
	},
	"PWMCONF": {
		"pwm_ofs":       0xFF << 0,
		"pwm_grad":      0xFF << 8,
		"pwm_freq":      0x03 << 16,
		"pwm_autoscale": 0x01 << 18,
		"pwm_autograd":  0x01 << 19,
		"freewheel":     0x03 << 20,
		"pwm_reg":       0x0F << 24,
		"pwm_lim":       0x0F << 28,
	},
	"PWM_SCALE": {
		"pwm_scale_sum":  0xff << 0,
		"pwm_scale_auto": 0x1ff << 16,
	},
	"TPOWERDOWN": {
		"tpowerdown": 0xff << 0,
	},
	"TPWMTHRS": {
		"tpwmthrs": 0xfffff << 0,
	},
	"TCOOLTHRS": {
		"tcoolthrs": 0xfffff << 0,
	},
	"TSTEP": {
		"tstep": 0xfffff << 0,
	},
}

var TMC5160_SignedFields = []string{"cur_a", "cur_b", "sgt", "xactual", "vactual", "pwm_scale_auto"}

var TMC5160_FieldFormatters = map[string]func(interface{}) string{
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
	"s2vsa": func(v interface{}) string {
		if value.True(v) {
			return "1(ShortToSupply_A!)"
		}
		return ""
	},
	"s2vsb": func(v interface{}) string {
		if value.True(v) {
			return "1(ShortToSupply_B!)"
		}
		return ""
	},
}

/**
######################################################################
# TMC stepper current config helper
######################################################################
*/

const (
	TMC5160_VREF        = 0.325
	TMC5160_MAX_CURRENT = 3.000
)

var _ ICurrentHelper = (*TMC5160CurrentHelper)(nil)

type TMC5160CurrentHelper struct {
	printer          *Printer
	name             string
	mcu_tmc          IMCU_TMC
	fields           *FieldHelper
	req_hold_current float64
	sense_resistor   float64
}

func NewTMC5160CurrentHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMC5160CurrentHelper {
	self := new(TMC5160CurrentHelper)
	self.printer = config.Get_printer()
	self.name = str.LastName(config.Get_name())
	self.mcu_tmc = mcu_tmc
	self.fields = mcu_tmc.Get_fields()
	run_current := config.Getfloat("run_current", 0, 0, MAX_CURRENT,
		0., 0, true)
	hold_current := config.Getfloat("hold_current", MAX_CURRENT, 0, MAX_CURRENT, 0., 0, true)
	self.req_hold_current = hold_current
	self.sense_resistor = config.Getfloat("sense_resistor", 0.075, 0, 0, 0., 0, true)
	gscaler, irun, ihold := self._calc_current(run_current, hold_current)
	self.fields.Set_field("globalscaler", gscaler, nil, nil)
	self.fields.Set_field("ihold", ihold, nil, nil)
	self.fields.Set_field("irun", irun, nil, nil)
	return self
}

func (self *TMC5160CurrentHelper) _calc_globalscaler(current float64) int {
	globalscaler := int((current * 256. * math.Sqrt(2.) * self.sense_resistor / TMC5160_VREF) + .5)
	globalscaler = maths.Max(32, globalscaler)
	if globalscaler >= 256 {
		globalscaler = 0
	}

	return globalscaler
}

func (self *TMC5160CurrentHelper) _calc_current_bits(current float64, globalscaler int) int {
	if globalscaler == 0 {
		globalscaler = 256
	}

	cs := int((current*256.*32.*math.Sqrt(2.)*self.sense_resistor)/
		(float64(globalscaler)*TMC5160_VREF) - 1. + .5)
	return maths.Max(0, maths.Min(31, cs))
}

func (self *TMC5160CurrentHelper) _set_globalscaler(current float64) {
	globalscaler := int((current * 256. * math.Sqrt(2.) * self.sense_resistor / TMC5160_VREF) + .5)
	globalscaler = maths.Max(32, globalscaler)
	if globalscaler >= 256 {
		globalscaler = 0
	}

	self.fields.Set_field("globalscaler", globalscaler, nil, nil)
}

func (self *TMC5160CurrentHelper) _calc_current(run_current, hold_current float64) (int, int, int) {
	gscaler := self._calc_globalscaler(run_current)
	irun := self._calc_current_bits(run_current, gscaler)
	ihold := self._calc_current_bits(math.Min(hold_current, run_current), gscaler)
	return gscaler, irun, ihold
}

func (self *TMC5160CurrentHelper) _calc_current_from_field(field_name string) float64 {
	globalscaler := float64(self.fields.Get_field("globalscaler", 0, nil))
	if value.False(globalscaler) {
		globalscaler = 256
	}

	bits := float64(self.fields.Get_field(field_name, 0, nil))
	return globalscaler * (bits + 1) * TMC5160_VREF /
		(256. * 32. * math.Sqrt(2.) * self.sense_resistor)
}

func (self *TMC5160CurrentHelper) Get_current() []float64 {
	run_current := self._calc_current_from_field("irun")
	hold_current := self._calc_current_from_field("ihold")
	return []float64{run_current, hold_current, self.req_hold_current, MAX_CURRENT}
}

func (self *TMC5160CurrentHelper) Set_current(run_current, hold_current, print_time float64) {
	self.req_hold_current = hold_current
	gscaler, irun, ihold := self._calc_current(run_current, hold_current)
	val := self.fields.Set_field("globalscaler", gscaler, nil, nil)
	self.mcu_tmc.Set_register("GLOBALSCALER", val, cast.Float64P(print_time))
	self.fields.Set_field("ihold", ihold, nil, nil)
	val = self.fields.Set_field("irun", irun, nil, nil)
	self.mcu_tmc.Set_register("IHOLD_IRUN", val, cast.Float64P(print_time))
}

/**
######################################################################
# TMC5160 printer object
######################################################################
*/

type TMC5160 struct {
	fields           *FieldHelper
	mcu_tmc          IMCU_TMC
	Get_phase_offset func() (*int, int)
	Get_status       func(float64) map[string]interface{}
}

func NewTMC5160(config *ConfigWrapper) *TMC5160 {
	self := new(TMC5160)

	// Setup mcu communication
	self.fields = NewFieldHelper(TMC5160_Fields, TMC5160_SignedFields, TMC5160_FieldFormatters, nil)
	self.mcu_tmc = NewMCU_TMC_SPI(config, TMC5160_Registers, self.fields, TMC5160_TMC_FREQUENCY)
	// Allow virtual pins to be created
	NewTMCVirtualPinHelper(config, self.mcu_tmc)
	// Register commands
	current_helper := NewTMC5160CurrentHelper(config, self.mcu_tmc)
	cmdhelper := NewTMCCommandHelper(config, self.mcu_tmc, current_helper)
	cmdhelper.Setup_register_dump(TMC5160_ReadRegisters, nil)
	self.Get_phase_offset = cmdhelper.Get_phase_offset
	self.Get_status = cmdhelper.Get_status
	// Setup basic register values
	self.fields.Set_field("multistep_filt", true, nil, nil)
	TMCStealthchopHelper(config, self.mcu_tmc, TMC5160_TMC_FREQUENCY)
	set_config_field := self.fields.Set_config_field
	//   CHOPCONF
	set_config_field(config, "toff", 3)
	set_config_field(config, "hstrt", 5)
	set_config_field(config, "hend", 2)
	set_config_field(config, "fd3", 0)
	set_config_field(config, "disfdcc", 0)
	set_config_field(config, "chm", 0)
	set_config_field(config, "tbl", 2)
	set_config_field(config, "vhighfs", 0)
	set_config_field(config, "vhighchm", 0)
	set_config_field(config, "tpfd", 4)
	set_config_field(config, "diss2g", 0)
	set_config_field(config, "diss2vs", 0)
	//   COOLCONF
	set_config_field(config, "semin", 0) // page 52
	set_config_field(config, "seup", 0)
	set_config_field(config, "semax", 0)
	set_config_field(config, "sedn", 0)
	set_config_field(config, "seimin", 0)
	set_config_field(config, "sgt", 0)
	set_config_field(config, "sfilt", 0)
	//   IHOLDIRUN
	set_config_field(config, "iholddelay", 6)
	//  PWMCONF
	set_config_field(config, "pwm_ofs", 30)
	set_config_field(config, "pwm_grad", 0)
	set_config_field(config, "pwm_freq", 0)
	set_config_field(config, "pwm_autoscale", true)
	set_config_field(config, "pwm_autograd", true)
	set_config_field(config, "freewheel", 0)
	set_config_field(config, "pwm_reg", 4)
	set_config_field(config, "pwm_lim", 12)
	//   TPOWERDOWN
	set_config_field(config, "tpowerdown", 10)
	return self
}

func Load_config_TMC5160(config *ConfigWrapper) interface{} {
	return NewTMC5160(config)
}
