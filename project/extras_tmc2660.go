package project

import (
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/str"
	"k3c/common/value"
	"math"
)

var TMC2660_Registers = map[string]int64{
	"DRVCONF": 0xE, "SGCSCONF": 0xC, "SMARTEN": 0xA,
	"CHOPCONF": 0x8, "DRVCTRL": 0x0,
}

var TMC2660_ReadRegisters = []string{"READRSP@RDSEL0", "READRSP@RDSEL1", "READRSP@RDSEL2"}

var TMC2660_Fields = map[string]map[string]int64{

	"DRVCTRL": {
		"mres":   0x0f,
		"dedge":  0x01 << 8,
		"intpol": 0x01 << 9,
	},

	"CHOPCONF": {
		"toff":  0x0f,
		"hstrt": 0x7 << 4,
		"hend":  0x0f << 7,
		"hdec":  0x03 << 11,
		"rndtf": 0x01 << 13,
		"chm":   0x01 << 14,
		"tbl":   0x03 << 15,
	},

	"SMARTEN": {
		"semin":  0x0f,
		"seup":   0x03 << 5,
		"semax":  0x0f << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
	},

	"SGCSCONF": {
		"cs":    0x1f,
		"sgt":   0x7F << 8,
		"sfilt": 0x01 << 16,
	},

	"DRVCONF": {
		"rdsel":  0x03 << 4,
		"vsense": 0x01 << 6,
		"sdoff":  0x01 << 7,
		"ts2g":   0x03 << 8,
		"diss2g": 0x01 << 10,
		"slpl":   0x03 << 12,
		"slph":   0x03 << 14,
		"tst":    0x01 << 16,
	},

	"READRSP@RDSEL0": {
		"stallguard": 0x01 << 4,
		"ot":         0x01 << 5,
		"otpw":       0x01 << 6,
		"s2ga":       0x01 << 7,
		"s2gb":       0x01 << 8,
		"ola":        0x01 << 9,
		"olb":        0x01 << 10,
		"stst":       0x01 << 11,
		"mstep":      0x3ff << 14,
	},

	"READRSP@RDSEL1": {
		"stallguard": 0x01 << 4,
		"ot":         0x01 << 5,
		"otpw":       0x01 << 6,
		"s2ga":       0x01 << 7,
		"s2gb":       0x01 << 8,
		"ola":        0x01 << 9,
		"olb":        0x01 << 10,
		"stst":       0x01 << 11,
		"sg_result":  0x3ff << 14,
	},

	"READRSP@RDSEL2": {
		"stallguard":       0x01 << 4,
		"ot":               0x01 << 5,
		"otpw":             0x01 << 6,
		"s2ga":             0x01 << 7,
		"s2gb":             0x01 << 8,
		"ola":              0x01 << 9,
		"olb":              0x01 << 10,
		"stst":             0x01 << 11,
		"se":               0x1f << 14,
		"sg_result@rdsel2": 0x1f << 19,
	},
}

var TMC2660_SignedFields = []string{"sgt"}

var TMC2660_FieldFormatters = map[string]func(interface{}) string{
	// tmc2130
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
	// tmc2660 private

	/**

		"chm": (lambda v: "1(constant toff)" if v else "0(spreadCycle)"),
	    "vsense": (lambda v: "1(165mV)" if v else "0(305mV)"),
	    "sdoff": (lambda v: "1(Step/Dir disabled!)" if v else ""),
	    "diss2g": (lambda v: "1(Short to GND disabled!)" if v else ""),
	    "se": (lambda v: ("%d" % v) if v else "0(Reset?)"),

	*/
	"chm": func(v interface{}) string {
		if value.True(v) {
			return "1(constant toff)"
		}
		return "0(spreadCycle)"
	},
	"vsense": func(v interface{}) string {
		if value.True(v) {
			return "1(165mV)"
		}
		return "0(305mV)"
	},
	"sdoff": func(v interface{}) string {
		if value.True(v) {
			return "1(Step/Dir disabled!)"
		}
		return ""
	},

	"diss2g": func(v interface{}) string {
		if value.True(v) {
			return "1(Short to GND disabled!)"
		}
		return ""
	},
	"se": func(v interface{}) string {
		if value.True(v) {
			return "(\"%d\" % v)"
		}

		return "0(Reset?)"
	},
}

/**
######################################################################
# TMC stepper current config helper
######################################################################
*/

const TMC2660_MAX_CURRENT = 2.400

type TMC2660CurrentHelper struct {
	printer                 *Printer
	name                    string
	mcu_tmc                 IMCU_TMC
	fields                  *FieldHelper
	current                 float64
	sense_resistor          float64
	idle_current_percentage int
}

func NewTMC2660CurrentHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMC2660CurrentHelper {
	self := new(TMC2660CurrentHelper)
	self.printer = config.Get_printer()
	self.name = str.LastName(config.Get_name())
	self.mcu_tmc = mcu_tmc
	self.fields = mcu_tmc.Get_fields()
	self.current = config.Getfloat("run_current", 0, 0.1, TMC2660_MAX_CURRENT, 0, 0, true)

	self.sense_resistor = config.Getfloat("sense_resistor", 0, 0, 0, 0, 0, true)
	vsense, cs := self._calc_current(self.current)
	self.fields.Set_field("cs", cs, nil, nil)
	self.fields.Set_field("vsense", vsense, nil, nil)

	// Register ready/printing handlers
	self.idle_current_percentage = config.Getint("idle_current_percent", 100, 0, 100, true)
	if self.idle_current_percentage < 100 {
		self.printer.Register_event_handler("idle_timeout:printing",
			self._handle_printing)
		self.printer.Register_event_handler("idle_timeout:ready",
			self._handle_ready)
	}

	return self
}

func (self *TMC2660CurrentHelper) _calc_current_bits(current float64, vsense bool) int {
	vref := 0.165
	if !vsense {
		vref = 0.310
	}

	sr := self.sense_resistor
	cs := int(32.*sr*current*math.Sqrt(2.)/vref+.5) - 1
	return maths.Max(0, maths.Min(31, cs))
}

func (self *TMC2660CurrentHelper) _calc_current_from_bits(cs float64, vsense bool) float64 {
	vref := 0.165
	if !vsense {
		vref = 0.310
	}

	return (cs + 1) * vref / (32. * self.sense_resistor * math.Sqrt(2.))
}

func (self *TMC2660CurrentHelper) _calc_current(run_current float64) (bool, float64) {
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

	return vsense, float64(irun)
}

func (self *TMC2660CurrentHelper) _handle_printing(argv []interface{}) error {
	print_time := cast.ToFloat64(argv[0])
	print_time -= 0.100 // Schedule slightly before deadline
	self.printer.Get_reactor().Register_callback(func(interface{}) interface{} {
		return self._update_current(self.current, print_time)
	}, 0)
	return nil
}

func (self *TMC2660CurrentHelper) _handle_ready(argv []interface{}) error {
	print_time := cast.ToFloat64(argv[0])
	current := self.current * float64(self.idle_current_percentage) / 100.
	self.printer.Get_reactor().Register_callback(
		func(i interface{}) interface{} { return self._update_current(current, print_time) }, 0)
	return nil
}

func (self *TMC2660CurrentHelper) _update_current(current, print_time float64) interface{} {
	vsense, cs := self._calc_current(current)
	val := self.fields.Set_field("cs", cs, nil, nil)
	self.mcu_tmc.Set_register("SGCSCONF", val, cast.Float64P(print_time))
	// Only update VSENSE if we need to
	if vsense != cast.ToBool(self.fields.Get_field("vsense", 0, nil)) {
		val = self.fields.Set_field("vsense", vsense, nil, nil)
		self.mcu_tmc.Set_register("DRVCONF", val, cast.Float64P(print_time))
	}

	return nil
}

func (self *TMC2660CurrentHelper) Get_current() []float64 {
	return []float64{self.current, 0, 0, MAX_CURRENT}
}

func (self *TMC2660CurrentHelper) Set_current(run_current, hold_current, print_time float64) {
	self.current = run_current
	self._update_current(run_current, print_time)
}

/**
######################################################################
# TMC2660 SPI
######################################################################

*/

// Helper code for working with TMC2660 devices via SPI

var _ IMCU_TMC = (*MCU_TMC2660_SPI)(nil)

type MCU_TMC2660_SPI struct {
	printer     *Printer
	mutex       *ReactorMutex
	spi         *MCU_SPI
	name_to_reg map[string]int64
	fields      *FieldHelper
}

func NewMCU_TMC2660_SPI(config *ConfigWrapper, name_to_reg map[string]int64, fields *FieldHelper) *MCU_TMC2660_SPI {
	self := new(MCU_TMC2660_SPI)

	self.printer = config.Get_printer()
	self.mutex = self.printer.Get_reactor().Mutex(false)
	self.spi, _ = MCU_SPI_from_config(config, 0, "", 4000000, "", false)
	self.name_to_reg = name_to_reg
	self.fields = fields
	return self
}

func (self *MCU_TMC2660_SPI) Get_fields() *FieldHelper {
	return self.fields
}

func (self *MCU_TMC2660_SPI) Get_register(reg_name string) (int64, error) {
	new_rdsel := indexOf(TMC2660_ReadRegisters, reg_name)
	reg := self.name_to_reg["DRVCONF"]
	if value.IsNotNone(self.printer.Get_start_args()["debugoutput"]) {
		return 0, nil
	}

	var params = make(map[string]interface{})
	func() {
		self.mutex.Lock()
		defer self.mutex.Unlock()
		old_rdsel := self.fields.Get_field("rdsel", 0, nil)
		val := self.fields.Set_field("rdsel", new_rdsel, nil, nil)
		msg := []int{int(((val >> 16) | reg) & 0xff), int((val >> 8) & 0xff), int(val & 0xff)}

		if int64(new_rdsel) != old_rdsel {
			//  Must set RDSEL value first
			self.spi.Spi_send(msg, 0, 0)
		}
		params = self.spi.Spi_transfer(msg, 0, 0).(map[string]interface{})
	}()

	pr := []byte(cast.ToString(params["response"]))
	return (int64(pr[0]) << 16) | (int64(pr[1]) << 8) | int64(pr[2]), nil
}

func indexOf(data []string, element string) int {
	for idx, v := range data {
		if element == v {
			return idx
		}
	}
	return -1
}

func (self *MCU_TMC2660_SPI) Set_register(reg_name string, val int64, print_time *float64) error {
	minclock := int64(0)
	if value.IsNotNone(print_time) {
		minclock = self.spi.get_mcu().Print_time_to_clock(cast.Float64(print_time))
	}
	reg := self.name_to_reg[reg_name]
	msg := []int{int(((val >> 16) | reg) & 0xff), int((val >> 8) & 0xff), int(val & 0xff)}
	self.mutex.Lock()
	defer self.mutex.Unlock()
	self.spi.Spi_send(msg, minclock, 0)
	return nil
}

/**
######################################################################
# TMC2660 printer object
######################################################################
*/

type TMC2660 struct {
	fields           *FieldHelper
	mcu_tmc          IMCU_TMC
	Get_phase_offset func() (*int, int)
	Get_status       func(float64) map[string]interface{}
}

func NewTMC2660(config *ConfigWrapper) *TMC2660 {
	self := new(TMC2660)
	// Setup mcu communication
	self.fields = NewFieldHelper(TMC2660_Fields, TMC2660_SignedFields, TMC2660_FieldFormatters, nil)
	self.fields.Set_field("sdoff", 0, nil, nil) // Access DRVCTRL in step/dir mode

	self.mcu_tmc = NewMCU_TMC2660_SPI(config, TMC2660_Registers, self.fields)
	//Register commands
	current_helper := NewTMC2660CurrentHelper(config, self.mcu_tmc)
	cmdhelper := NewTMCCommandHelper(config, self.mcu_tmc, current_helper)
	cmdhelper.Setup_register_dump(TMC2660_ReadRegisters, nil)
	self.Get_phase_offset = cmdhelper.Get_phase_offset
	self.Get_status = cmdhelper.Get_status

	// CHOPCONF
	set_config_field := self.fields.Set_config_field
	set_config_field(config, "tbl", 2)
	set_config_field(config, "rndtf", 0)
	set_config_field(config, "hdec", 0)
	set_config_field(config, "chm", 0)
	set_config_field(config, "hend", 3)
	set_config_field(config, "hstrt", 3)
	set_config_field(config, "toff", 4)

	if value.False(self.fields.Get_field("chm", 0, nil)) {
		if (self.fields.Get_field("hstrt", 0, nil) +
			self.fields.Get_field("hend", 0, nil)) > 15 {
			panic("driver_HEND + driver_HSTRT must be <= 15")
		}
	}

	// SMARTEN
	set_config_field(config, "seimin", 0)
	set_config_field(config, "sedn", 0)
	set_config_field(config, "semax", 0)
	set_config_field(config, "seup", 0)
	set_config_field(config, "semin", 0)

	// SGSCONF
	set_config_field(config, "sfilt", 0)
	set_config_field(config, "sgt", 0)

	// DRVCONF
	set_config_field(config, "slph", 0)
	set_config_field(config, "slpl", 0)
	set_config_field(config, "diss2g", 0)
	set_config_field(config, "ts2g", 3)
	return self
}

func Load_config_TMC2660(config *ConfigWrapper) interface{} {
	return NewTMC2660(config)
}
