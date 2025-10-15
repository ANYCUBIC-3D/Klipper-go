package project

const TMC2209_TMC_FREQUENCY = 12000000.

var TMC2209_Registers = map[string]int64{
	// tmc2208
	"GCONF": 0x00, "GSTAT": 0x01, "IFCNT": 0x02, "SLAVECONF": 0x03,
	"OTP_PROG": 0x04, "OTP_READ": 0x05, "IOIN": 0x06, "FACTORY_CONF": 0x07,
	"IHOLD_IRUN": 0x10, "TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13,
	"VACTUAL": 0x22, "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c,
	"DRV_STATUS": 0x6f, "PWMCONF": 0x70, "PWM_SCALE": 0x71, "PWM_AUTO": 0x72,

	// tmc2209
	"TCOOLTHRS": 0x14,
	"COOLCONF":  0x42,
	"SGTHRS":    0x40,
	"SG_RESULT": 0x41,
}

var TMC2209_ReadRegisters = []string{
	// tmc2208
	"GCONF", "GSTAT", "IFCNT", "OTP_READ", "IOIN", "FACTORY_CONF", "TSTEP",
	"MSCNT", "MSCURACT", "CHOPCONF", "DRV_STATUS",
	"PWMCONF", "PWM_SCALE", "PWM_AUTO",
	// tmc2209 private
	"SG_RESULT",
}

var TMC2209_Fields = map[string]map[string]int64{
	// tmc2208
	"GCONF": {
		"i_scale_analog":   0x01,
		"internal_rsense":  0x01 << 1,
		"en_spreadcycle":   0x01 << 2, //0x00 << 2,
		"shaft":            0x01 << 3,
		"index_otpw":       0x01 << 4,
		"index_step":       0x01 << 5,
		"pdn_disable":      0x01 << 6,
		"mstep_reg_select": 0x01 << 7,
		"multistep_filt":   0x01 << 8,
		"test_mode":        0x01 << 9,
	},

	"GSTAT": {
		"reset":   0x01,
		"drv_err": 0x01 << 1,
		"uv_cp":   0x01 << 2,
	},

	"IFCNT": {
		"ifcnt": 0xff,
	},

	"SLAVECONF": {
		"senddelay": 0x0f << 8,
	},
	"OTP_PROG": {
		"otpbit":   0x07,
		"otpbyte":  0x03 << 4,
		"otpmagic": 0xff << 8,
	},

	"OTP_READ": {
		"otp_fclktrim":       0x1f,
		"otp_ottrim":         0x01 << 5,
		"otp_internalrsense": 0x01 << 6,
		"otp_tbl":            0x01 << 7,
		"otp_pwm_grad":       0x0f << 8,
		"otp_pwm_autograd":   0x01 << 12,
		"otp_tpwmthrs":       0x07 << 13,
		"otp_pwm_ofs":        0x01 << 16,
		"otp_pwm_reg":        0x01 << 17,
		"otp_pwm_freq":       0x01 << 18,
		"otp_iholddelay":     0x03 << 19,
		"otp_ihold":          0x03 << 21,
		"otp_en_spreadcycle": 0x01 << 23,
	},

	// IOIN mapping depends on the driver type (SEL_A field)
	// TMC222x (SEL_A == 0)
	"IOIN@TMC222x": {
		"pdn_uart": 0x01 << 1,
		"spread":   0x01 << 2,
		"dir":      0x01 << 3,
		"enn":      0x01 << 4,
		"step":     0x01 << 5,
		"ms1":      0x01 << 6,
		"ms2":      0x01 << 7,
		"sel_a":    0x01 << 8,
		"version":  0xff << 24,
	},

	// TMC220x (SEL_A == 1)
	"IOIN@TMC220x": {
		"enn":      0x01,
		"ms1":      0x01 << 2,
		"ms2":      0x01 << 3,
		"diag":     0x01 << 4,
		"pdn_uart": 0x01 << 6,
		"step":     0x01 << 7,
		"sel_a":    0x01 << 8,
		"dir":      0x01 << 9,
		"version":  0xff << 24,
	},

	"FACTORY_CONF": {
		"fclktrim": 0x1f,
		"ottrim":   0x03 << 8,
	},
	"IHOLD_IRUN": {
		"ihold":      0x1f,
		"irun":       0x1f << 8,
		"iholddelay": 0x0f << 16,
	},
	"TPOWERDOWN": {
		"tpowerdown": 0xff,
	},
	"TSTEP": {
		"tstep": 0xfffff,
	},
	"TPWMTHRS": {
		"tpwmthrs": 0xfffff,
	},

	"VACTUAL": {
		"vactual": 0xffffff,
	},

	"MSCNT": {
		"mscnt": 0x3ff,
	},

	"MSCURACT": {
		"cur_a": 0x1ff,
		"cur_b": 0x1ff << 16,
	},
	"CHOPCONF": {
		"toff":    0x0f,
		"hstrt":   0x07 << 4,
		"hend":    0x0f << 7,
		"tbl":     0x03 << 15,
		"vsense":  0x01 << 17,
		"mres":    0x0f << 24,
		"intpol":  0x01 << 28,
		"dedge":   0x01 << 29,
		"diss2g":  0x01 << 30,
		"diss2vs": 0x01 << 31,
	},

	"DRV_STATUS": {
		"otpw":      0x01,
		"ot":        0x01 << 1,
		"s2ga":      0x01 << 2,
		"s2gb":      0x01 << 3,
		"s2vsa":     0x01 << 4,
		"s2vsb":     0x01 << 5,
		"ola":       0x01 << 6,
		"olb":       0x01 << 7,
		"t120":      0x01 << 8,
		"t143":      0x01 << 9,
		"t150":      0x01 << 10,
		"t157":      0x01 << 11,
		"cs_actual": 0x1f << 16,
		"stealth":   0x01 << 30,
		"stst":      0x01 << 31,
	},

	"PWMCONF": {
		"pwm_ofs":       0xff,
		"pwm_grad":      0xff << 8,
		"pwm_freq":      0x03 << 16,
		"pwm_autoscale": 0x01 << 18,
		"pwm_autograd":  0x01 << 19,
		"freewheel":     0x03 << 20,
		"pwm_reg":       0xf << 24,
		"pwm_lim":       0xf << 28,
	},

	"PWM_SCALE": {
		"pwm_scale_sum":  0xff,
		"pwm_scale_auto": 0x1ff << 16,
	},

	"PWM_AUTO": {
		"pwm_ofs_auto":  0xff,
		"pwm_grad_auto": 0xff << 16,
	},

	// tmc2209 private
	"COOLCONF": {
		"semin":  0x0F << 0,
		"seup":   0x03 << 5,
		"semax":  0x0F << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
	},
	"IOIN": {
		"enn":       0x01 << 0,
		"ms1":       0x01 << 2,
		"ms2":       0x01 << 3,
		"diag":      0x01 << 4,
		"pdn_uart":  0x01 << 6,
		"step":      0x01 << 7,
		"spread_en": 0x01 << 8,
		"dir":       0x01 << 9,
		"version":   0xff << 24,
	},

	"SGTHRS": {
		"sgthrs": 0xFF << 0,
	},
	"SG_RESULT": {
		"sg_result": 0x3FF << 0,
	},
	"TCOOLTHRS": {
		"tcoolthrs": 0xfffff,
	},
}

var TMC2209_FieldFormatters = TMC2208_FieldFormatters // presume no change

/**
######################################################################
# TMC2209 printer object
######################################################################
*/

type TMC2209 struct {
	fields           *FieldHelper
	mcu_tmc          IMCU_TMC
	Get_phase_offset func() (*int, int)
	Get_status       func(float64) map[string]interface{}
}

func NewTMC2209(config *ConfigWrapper) *TMC2209 {
	self := &TMC2209{}
	// Setup mcu communication
	self.fields = NewFieldHelper(TMC2209_Fields, TMC2208_SignedFields,
		TMC2209_FieldFormatters, nil)
	self.mcu_tmc = NewMCU_TMC_uart(config, TMC2209_Registers, self.fields, 3, TMC2209_TMC_FREQUENCY)

	// Setup fields for UART
	self.fields.Set_field("pdn_disable", true, nil, nil)
	self.fields.Set_field("senddelay", 2, nil, nil) // Avoid tx errors on shared uart
	// Allow virtual pins to be created

	NewTMCVirtualPinHelper(config, self.mcu_tmc)
	// Register commands
	current_helper := NewTMCCurrentHelper(config, self.mcu_tmc)
	cmdhelper := NewTMCCommandHelper(config, self.mcu_tmc, current_helper)
	cmdhelper.Setup_register_dump(TMC2209_ReadRegisters, nil)
	self.Get_phase_offset = cmdhelper.Get_phase_offset
	self.Get_status = cmdhelper.Get_status

	// Setup basic register values
	self.fields.Set_field("mstep_reg_select", true, nil, nil)
	self.fields.Set_field("multistep_filt", true, nil, nil)
	TMCStealthchopHelper(config, self.mcu_tmc, TMC2209_TMC_FREQUENCY)
	// Allow other registers to be set from the config
	set_config_field := self.fields.Set_config_field
	//# CHOPCONF
	set_config_field(config, "toff", 3)
	set_config_field(config, "hstrt", 5)
	set_config_field(config, "hend", 0)
	set_config_field(config, "tbl", 2)
	//# IHOLDIRUN
	set_config_field(config, "iholddelay", 8)
	//# PWMCONF
	set_config_field(config, "pwm_ofs", 36)
	set_config_field(config, "pwm_grad", 14)
	set_config_field(config, "pwm_freq", 1)
	set_config_field(config, "pwm_autoscale", true)
	set_config_field(config, "pwm_autograd", true)
	set_config_field(config, "pwm_reg", 8)
	set_config_field(config, "pwm_lim", 12)
	//# TPOWERDOWN
	set_config_field(config, "tpowerdown", 20)
	//# SGTHRS
	set_config_field(config, "sgthrs", 0)

	return self
}

func Load_config_TMC2209(config *ConfigWrapper) interface{} {
	return NewTMC2209(config)
}
