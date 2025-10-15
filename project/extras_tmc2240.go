package project

import (
	"fmt"
	"k3c/common/utils/maths"
	"k3c/common/value"
	"math"
	"strings"
)

const TMC2240_TMC_FREQUENCY = 12500000.

var TMC2240_Registers = map[string]int64{
	"GCONF":           0x00,
	"GSTAT":           0x01,
	"IFCNT":           0x02,
	"NODECONF":        0x03,
	"IOIN":            0x04,
	"DRV_CONF":        0x0A,
	"GLOBALSCALER":    0x0B,
	"IHOLD_IRUN":      0x10,
	"TPOWERDOWN":      0x11,
	"TSTEP":           0x12,
	"TPWMTHRS":        0x13,
	"TCOOLTHRS":       0x14,
	"THIGH":           0x15,
	"DIRECT_MODE":     0x2D,
	"ENCMODE":         0x38,
	"X_ENC":           0x39,
	"ENC_CONST":       0x3A,
	"ENC_STATUS":      0x3B,
	"ENC_LATCH":       0x3C,
	"ADC_VSUPPLY_AIN": 0x50,
	"ADC_TEMP":        0x51,
	"OTW_OV_VTH":      0x52,
	"MSLUT0":          0x60,
	"MSLUT1":          0x61,
	"MSLUT2":          0x62,
	"MSLUT3":          0x63,
	"MSLUT4":          0x64,
	"MSLUT5":          0x65,
	"MSLUT6":          0x66,
	"MSLUT7":          0x67,
	"MSLUTSEL":        0x68,
	"MSLUTSTART":      0x69,
	"MSCNT":           0x6A,
	"MSCURACT":        0x6B,
	"CHOPCONF":        0x6C,
	"COOLCONF":        0x6D,
	"DRV_STATUS":      0x6F,
	"PWMCONF":         0x70,
	"PWM_SCALE":       0x71,
	"PWM_AUTO":        0x72,
	"SG4_THRS":        0x74,
	"SG4_RESULT":      0x75,
	"SG4_IND":         0x76,
}

var TMC2240_ReadRegisters = []string{
	"GCONF", "GSTAT", "IOIN", "DRV_CONF", "GLOBALSCALER", "IHOLD_IRUN",
	"TPOWERDOWN", "TSTEP", "TPWMTHRS", "TCOOLTHRS", "THIGH", "ADC_VSUPPLY_AIN",
	"ADC_TEMP", "MSCNT", "MSCURACT", "CHOPCONF", "COOLCONF", "DRV_STATUS",
	"PWMCONF", "PWM_SCALE", "PWM_AUTO", "SG4_THRS", "SG4_RESULT", "SG4_IND",
}

var TMC2240_Fields = make(map[string]map[string]int64)

func init() {
	TMC2240_Fields["COOLCONF"] = map[string]int64{
		"semin":  0x0F << 0,
		"seup":   0x03 << 5,
		"semax":  0x0F << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
		"sgt":    0x7F << 16,
		"sfilt":  0x01 << 24,
	}

	TMC2240_Fields["CHOPCONF"] = map[string]int64{
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
	}

	TMC2240_Fields["DRV_STATUS"] = map[string]int64{
		"sg_result":  0x3FF << 0,
		"s2vsa":      0x01 << 12,
		"s2vsb":      0x01 << 13,
		"stealth":    0x01 << 14,
		"fsactive":   0x01 << 15,
		"csactual":   0x1F << 16,
		"stallguard": 0x01 << 24,
		"ot":         0x01 << 25,
		"otpw":       0x01 << 26,
		"s2ga":       0x01 << 27,
		"s2gb":       0x01 << 28,
		"ola":        0x01 << 29,
		"olb":        0x01 << 30,
		"stst":       0x01 << 31,
	}

	TMC2240_Fields["GCONF"] = map[string]int64{
		"faststandstill":   0x01 << 1,
		"en_pwm_mode":      0x01 << 2,
		"multistep_filt":   0x01 << 3,
		"shaft":            0x01 << 4,
		"diag0_error":      0x01 << 5,
		"diag0_otpw":       0x01 << 6,
		"diag0_stall":      0x01 << 7,
		"diag1_stall":      0x01 << 8,
		"diag1_index":      0x01 << 9,
		"diag1_onstate":    0x01 << 10,
		"diag0_pushpull":   0x01 << 12,
		"diag1_pushpull":   0x01 << 13,
		"small_hysteresis": 0x01 << 14,
		"stop_enable":      0x01 << 15,
		"direct_mode":      0x01 << 16,
	}

	TMC2240_Fields["GSTAT"] = map[string]int64{
		"reset":          0x01 << 0,
		"drv_err":        0x01 << 1,
		"uv_cp":          0x01 << 2,
		"register_reset": 0x01 << 3,
		"vm_uvlo":        0x01 << 4,
	}

	TMC2240_Fields["GLOBALSCALER"] = map[string]int64{
		"globalscaler": 0xFF << 0,
	}

	TMC2240_Fields["IHOLD_IRUN"] = map[string]int64{
		"ihold":      0x1F << 0,
		"irun":       0x1F << 8,
		"iholddelay": 0x0F << 16,
		"irundelay":  0x0F << 24,
	}

	TMC2240_Fields["IOIN"] = map[string]int64{
		"step":        0x01 << 0,
		"dir":         0x01 << 1,
		"encb":        0x01 << 2,
		"enca":        0x01 << 3,
		"drv_enn":     0x01 << 4,
		"encn":        0x01 << 5,
		"uart_en":     0x01 << 6,
		"comp_a":      0x01 << 8,
		"comp_b":      0x01 << 9,
		"comp_a1_a2":  0x01 << 10,
		"comp_b1_b2":  0x01 << 11,
		"output":      0x01 << 12,
		"ext_res_det": 0x01 << 13,
		"ext_clk":     0x01 << 14,
		"adc_err":     0x01 << 15,
		"silicon_rv":  0x07 << 16,
		"version":     0xFF << 24,
	}
	TMC2240_Fields["MSLUT0"] = map[string]int64{
		"mslut0": 0xffffffff,
	}
	TMC2240_Fields["MSLUT1"] = map[string]int64{
		"mslut1": 0xffffffff,
	}
	TMC2240_Fields["MSLUT2"] = map[string]int64{
		"mslut2": 0xffffffff,
	}
	TMC2240_Fields["MSLUT3"] = map[string]int64{
		"mslut3": 0xffffffff,
	}
	TMC2240_Fields["MSLUT4"] = map[string]int64{
		"mslut4": 0xffffffff,
	}
	TMC2240_Fields["MSLUT5"] = map[string]int64{
		"mslut5": 0xffffffff,
	}
	TMC2240_Fields["MSLUT6"] = map[string]int64{
		"mslut6": 0xffffffff,
	}
	TMC2240_Fields["MSLUT7"] = map[string]int64{
		"mslut7": 0xffffffff,
	}
	TMC2240_Fields["MSLUTSEL"] = map[string]int64{
		"x3": 0xFF << 24,
		"x2": 0xFF << 16,
		"x1": 0xFF << 8,
		"w3": 0x03 << 6,
		"w2": 0x03 << 4,
		"w1": 0x03 << 2,
		"w0": 0x03 << 0,
	}
	TMC2240_Fields["MSLUTSTART"] = map[string]int64{
		"start_sin":    0xFF << 0,
		"start_sin90":  0xFF << 16,
		"offset_sin90": 0xFF << 24,
	}
	TMC2240_Fields["MSCNT"] = map[string]int64{
		"mscnt": 0x3ff << 0,
	}
	TMC2240_Fields["MSCURACT"] = map[string]int64{
		"cur_a": 0x1ff << 0,
		"cur_b": 0x1ff << 16,
	}
	TMC2240_Fields["PWM_AUTO"] = map[string]int64{
		"pwm_ofs_auto":  0xff << 0,
		"pwm_grad_auto": 0xff << 16,
	}
	TMC2240_Fields["PWMCONF"] = map[string]int64{
		"pwm_ofs":            0xFF << 0,
		"pwm_grad":           0xFF << 8,
		"pwm_freq":           0x03 << 16,
		"pwm_autoscale":      0x01 << 18,
		"pwm_autograd":       0x01 << 19,
		"freewheel":          0x03 << 20,
		"pwm_meas_sd_enable": 0x01 << 22,
		"pwm_dis_reg_stst":   0x01 << 23,
		"pwm_reg":            0x0F << 24,
		"pwm_lim":            0x0F << 28,
	}
	TMC2240_Fields["PWM_SCALE"] = map[string]int64{
		"pwm_scale_sum":  0x3ff << 0,
		"pwm_scale_auto": 0x1ff << 16,
	}
	TMC2240_Fields["TPOWERDOWN"] = map[string]int64{
		"tpowerdown": 0xff << 0,
	}
	TMC2240_Fields["TPWMTHRS"] = map[string]int64{
		"tpwmthrs": 0xfffff << 0,
	}
	TMC2240_Fields["TCOOLTHRS"] = map[string]int64{
		"tcoolthrs": 0xfffff << 0,
	}
	TMC2240_Fields["TSTEP"] = map[string]int64{
		"tstep": 0xfffff << 0,
	}
	TMC2240_Fields["THIGH"] = map[string]int64{
		"thigh": 0xfffff << 0,
	}
	TMC2240_Fields["DRV_CONF"] = map[string]int64{
		"current_range": 0x03 << 0,
		"slope_control": 0x03 << 4,
	}
	TMC2240_Fields["ADC_VSUPPLY_AIN"] = map[string]int64{
		"adc_vsupply": 0x1fff << 0,
		"adc_ain":     0x1fff << 16,
	}
	TMC2240_Fields["ADC_TEMP"] = map[string]int64{
		"adc_temp": 0x1fff << 0,
	}
	TMC2240_Fields["OTW_OV_VTH"] = map[string]int64{
		"overvoltage_vth":        0x1fff << 0,
		"overtempprewarning_vth": 0x1fff << 16,
	}
	TMC2240_Fields["SG4_THRS"] = map[string]int64{
		"sg4_thrs":         0xFF << 0,
		"sg4_filt_en":      0x01 << 8,
		"sg4_angle_offset": 0x01 << 9,
	}
	TMC2240_Fields["SG4_RESULT"] = map[string]int64{
		"sg4_result": 0x3FF << 0,
	}
	TMC2240_Fields["SG4_IND"] = map[string]int64{
		"sg4_ind_0": 0xFF << 0,
		"sg4_ind_1": 0xFF << 8,
		"sg4_ind_2": 0xFF << 16,
		"sg4_ind_3": 0xFF << 24,
	}
}

var TMC2240_SignedFields = []string{"cur_a", "cur_b", "sgt", "pwm_scale_auto", "offset_sin90"}

var TMC2240_FieldFormatters = map[string]func(interface{}) string{
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
	"adc_temp": func(v interface{}) string {
		return fmt.Sprintf("0x%04x(%.1fC)", v, float64(v.(int64)-2038)/7.7)
	},
}

/*
######################################################################
# TMC stepper current config helper
######################################################################
*/
type TMC2240CurrentHelper struct {
	printer          *Printer
	name             string
	mcu_tmc          IMCU_TMC
	fields           *FieldHelper
	Rref             float64
	req_hold_current float64
}

func NewTMC2240CurrentHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMC2240CurrentHelper {
	self := &TMC2240CurrentHelper{}
	self.printer = config.Get_printer()
	name := strings.Split(config.Get_name(), " ")
	self.name = name[len(name)-1]
	self.mcu_tmc = mcu_tmc
	self.fields = mcu_tmc.Get_fields()
	self.Rref = config.Getfloat("rref", 12000.,
		12000., 60000., 0, 0, true)
	max_cur := self._get_ifs_rms(3)
	run_current := config.Getfloat("run_current", 0., 0., max_cur, 0., 0., true)
	hold_current := config.Getfloat("hold_current", max_cur,
		0, max_cur, 0., 0., true)
	self.req_hold_current = hold_current
	current_range := self._calc_current_range(run_current)
	self.fields.Set_field("current_range", current_range, nil, nil)
	gscaler, irun, ihold := self._calc_current(run_current, hold_current)
	self.fields.Set_field("globalscaler", gscaler, nil, nil)
	self.fields.Set_field("ihold", ihold, nil, nil)
	self.fields.Set_field("irun", irun, nil, nil)
	return self
}

func (self *TMC2240CurrentHelper) _get_ifs_rms(current_range interface{}) float64 {
	if current_range == nil {
		current_range = int(self.fields.Get_field("current_range", nil, nil))
	}

	KIFS := []float64{11750., 24000., 36000., 36000.}
	return KIFS[current_range.(int)] / self.Rref / math.Sqrt(2.)
}

func (self *TMC2240CurrentHelper) _calc_current_range(current float64) int {
	current_range := 0
	for current_range = 0; current_range < 4; current_range++ {
		if current <= self._get_ifs_rms(current_range) {
			break
		}
	}

	return current_range
}

func (self *TMC2240CurrentHelper) _calc_globalscaler(current float64) int {
	ifsRms := self._get_ifs_rms(nil)
	globalscaler := int((current*256.0)/ifsRms + 0.5)
	globalscaler = maths.Max(32, globalscaler)
	if globalscaler >= 256 {
		globalscaler = 0
	}
	return globalscaler
}

func (self *TMC2240CurrentHelper) _calc_current_bits(current float64, globalscaler int) int {
	ifsRms := self._get_ifs_rms(nil)
	if globalscaler == 0 {
		globalscaler = 256
	}
	cs := int((current*256.0*32.0)/(float64(globalscaler)*ifsRms) - 1 + 0.5)
	return maths.Max(0, maths.Min(31, cs))
}

func (self *TMC2240CurrentHelper) _calc_current(run_current, hold_current float64) (int, int, int) {
	gscaler := self._calc_globalscaler(run_current)
	irun := self._calc_current_bits(run_current, gscaler)
	ihold := self._calc_current_bits(math.Min(hold_current, run_current), gscaler)
	return gscaler, irun, ihold
}

func (self *TMC2240CurrentHelper) _calc_current_from_field(_field_name string) float64 {
	ifsRms := self._get_ifs_rms(nil) // Check here for the fields parameter.
	globalscaler := self.fields.Get_field("globalscaler", nil, nil)
	if globalscaler == 0 {
		globalscaler = 256
	}
	bits := self.fields.Get_field(_field_name, nil, nil)
	return float64(globalscaler*(bits+1)) * ifsRms / (256.0 * 32.0)
}

func (self *TMC2240CurrentHelper) Get_current() []float64 {
	ifsRms := self._get_ifs_rms(nil) // Check here for the fields parameter.
	run_current := self._calc_current_from_field("irun")
	hold_current := self._calc_current_from_field("ihold")
	return []float64{run_current, hold_current, self.req_hold_current, ifsRms}
}

func (self *TMC2240CurrentHelper) Set_current(run_current, hold_current float64, print_time float64) {
	self.req_hold_current = hold_current
	gscaler, irun, ihold := self._calc_current(run_current, hold_current)
	self.fields.Set_field("globalscaler", gscaler, nil, nil)
	self.mcu_tmc.Set_register("GLOBALSCALER", int64(gscaler), &print_time)
	self.fields.Set_field("ihold", ihold, nil, nil)
	self.mcu_tmc.Set_register("IHOLD_IRUN", int64(irun), &print_time)
}

//######################################################################
//# TMC2240 printer object
//######################################################################

type TMC2240 struct {
	fields           *FieldHelper
	mcu_tmc          IMCU_TMC
	get_phase_offset func() (*int, int)
	get_status       func(eventtime float64) map[string]interface{}
}

func NewTMC2240(config *ConfigWrapper) *TMC2240 {
	self := &TMC2240{}
	self.fields = NewFieldHelper(TMC2240_Fields, TMC2240_SignedFields, TMC2240_FieldFormatters, nil)
	if config.Get("uart_pin", nil, true) != nil {
		// use UART for communication
		self.mcu_tmc = NewMCU_TMC_uart(config, TMC2240_Registers, self.fields,
			3, TMC2240_TMC_FREQUENCY)
	} else {
		// Use SPI bus for communication
		self.mcu_tmc = NewMCU_TMC_SPI(config, TMC2240_Registers, self.fields,
			TMC2240_TMC_FREQUENCY)
	}

	NewTMCVirtualPinHelper(config, self.mcu_tmc)
	current_helper := NewTMC2240CurrentHelper(config, self.mcu_tmc)
	cmdhelper := NewTMCCommandHelper(config, self.mcu_tmc, current_helper)
	cmdhelper.Setup_register_dump(TMC2240_ReadRegisters, nil)

	self.get_phase_offset = cmdhelper.Get_phase_offset
	self.get_status = cmdhelper.Get_status

	self.fields.Set_field("multistep_filt", true, nil, nil)
	TMCWaveTableHelper(config, self.mcu_tmc)
	self.fields.Set_config_field(config, "offset_sin90", 0)
	TMCStealthchopHelper(config, self.mcu_tmc, TMC2240_TMC_FREQUENCY)

	set_config_field := self.fields.Set_config_field
	// CHOPCONF
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
	// COOLCONF
	set_config_field(config, "semin", 0)
	set_config_field(config, "seup", 0)
	set_config_field(config, "semax", 0)
	set_config_field(config, "sedn", 0)
	set_config_field(config, "seimin", 0)
	set_config_field(config, "sgt", 0)
	set_config_field(config, "sfilt", 0)
	// IHOLDIRUN
	set_config_field(config, "iholddelay", 6)
	set_config_field(config, "irundelay", 4)
	// PWMCONF
	set_config_field(config, "pwm_ofs", 29)
	set_config_field(config, "pwm_grad", 0)
	set_config_field(config, "pwm_freq", 0)
	set_config_field(config, "pwm_autoscale", true)
	set_config_field(config, "pwm_autograd", true)
	set_config_field(config, "freewheel", 0)
	set_config_field(config, "pwm_reg", 4)
	set_config_field(config, "pwm_lim", 12)
	// TPOWERDOWN
	set_config_field(config, "tpowerdown", 10)
	// SG4_THRS
	set_config_field(config, "sg4_angle_offset", true)

	return self
}

func Load_config_TMC2240(config *ConfigWrapper) interface{} {
	return NewTMC2240(config)
}
