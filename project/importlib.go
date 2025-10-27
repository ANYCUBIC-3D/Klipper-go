package project

import (
	"fmt"
	"k3c/common/logger"
)

// load main module for application
func LoadMainModule() map[string]interface{} {
	module := map[string]interface{}{
		"fan":                                    Load_config_fan,
		"safe_z_home":                            Load_config_safe_z_home,
		"homing":                                 Load_config_homing,
		"heaters":                                Load_config_heaters,
		"heater_bed":                             Load_config_Heater_bed,
		"adc_temperature":                        Load_config_adc_temperature,
		"thermistor":                             Load_config_thermistor,
		"query_adc":                              Load_config_query_adc,
		"heater_fan extruder_fan":                Load_config_heater_fan,
		"controller_fan controller_fan":          Load_config_controller_fan,
		"stepper_enable":                         Load_config_StepperEnable,
		"gcode_move":                             Load_config_gcode_move,
		"probe":                                  Load_config_probe,
		"gcode_macro_1":                          Load_config_printer_gcode_macro,
		"gcode_macro":                            Load_config_gcode_macro,
		"query_endstops":                         Load_config_query_endstops,
		"virtual_sdcard":                         Load_config_virtual_sdcard,
		"idle_timeout":                           Load_config_idletimeout,
		"encoder_sensor":                         Load_config_prefix_EncoderSensor,
		"pause_resume":                           Load_config_pause_resume,
		"bed_mesh":                               Load_config_bed_mesh,
		"adaptive_bed_mesh":                      Load_config_adaptive_bed_mesh,
		"tmc2209":                                Load_config_TMC2209,
		"tmc2240":                                Load_config_TMC2240,
		"force_move":                             Load_config_force_move,
		"gcode_arcs":                             Load_config_ArcSupport,
		"resonance_tester":                       Load_config_resonanceTester,
		"adxl345":                                Load_config_ADXL345,
		"input_shaper":                           Load_config_InputShaper,
		"print_stats":                            Load_config_PrintStats,
		"statistics":                             Load_config_PrinterStats,
		"filament_switch_sensor filament_sensor": Load_config_SwitchSensor,
		"buttons":                                Load_config_PrinterButtons,
		"lis2dw12":                               Load_config_LIS2DW12,
		"pid_calibrate":                          Load_config_PIDCalibrate,
		"verify_heater":                          Load_config_verify_heater,
		"output_pin":                             Load_config_prefix_DigitalOut,
		"cs1237":                                 Load_config_cs1237,
		"exclude_object":                         Load_config_ExcludeObject,
		"customLinear":                           Load_config_prefix_customLinear,
		"manual_stepper":                         Load_config_manual_stepper,
		"filament_tracker":                       Load_config_filament_tracker,
		"mcu_ota":                                Load_config_mcu_ota,
		"save_variables":                         Load_config_save_variables,
		"ace":                                    Load_config_ace,
		"ds18b20":                                Load_config_ds18b20,
	}
	return module
}

// load sub module for application
func Load_kinematics(kin_name string) interface{} {
	kinematics := map[string]interface{}{
		"cartesian": Load_kinematics_cartesian,
		"corexy":    Load_kinematics_corexy,
	}

	if _, ok := kinematics[kin_name]; !ok {
		logger.Error(fmt.Errorf("module about %s not support", kin_name))
		return Load_kinematics_none
	}
	return kinematics[kin_name]
}
