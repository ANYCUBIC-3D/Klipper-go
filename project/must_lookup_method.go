package project

import (
	"fmt"
	"k3c/common/utils/object"
)

func MustLookupToolhead(printer *Printer) *Toolhead {
	toolhead, ok := lookup_object(printer, "toolhead").(*Toolhead)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "toolhead", lookup_object(printer, "toolhead")))
	}
	return toolhead
}

func MustLookupStepperEnable(printer *Printer) *PrinterStepperEnable {
	stepperEnable, ok := lookup_object(printer, "stepper_enable").(*PrinterStepperEnable)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "stepper_enable", lookup_object(printer, "stepper_enable")))
	}

	return stepperEnable
}

func MustLookupPins(printer *Printer) *PrinterPins {
	pins, ok := lookup_object(printer, "pins").(*PrinterPins)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "pins", lookup_object(printer, "pins")))
	}

	return pins
}

func MustLookupHeaters(printer *Printer) *PrinterHeaters {
	heaters, ok := lookup_object(printer, "heaters").(*PrinterHeaters)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "heaters", lookup_object(printer, "heaters")))
	}

	return heaters
}

func MustLookupGcode(printer *Printer) *GCodeDispatch {
	gcode, ok := lookup_object(printer, "gcode").(*GCodeDispatch)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "gcode", lookup_object(printer, "gcode")))
	}

	return gcode
}

func MustLookupForceMove(printer *Printer) *ForceMove {
	forcemove, ok := lookup_object(printer, "force_move").(*ForceMove)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "force_move", lookup_object(printer, "force_move")))
	}

	return forcemove
}

func MustLookupGCodeMove(printer *Printer) *GCodeMove {
	gcode_move, ok := lookup_object(printer, "gcode_move").(*GCodeMove)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "gcode_move", lookup_object(printer, "gcode_move")))
	}

	return gcode_move
}

func MustLookupTuningTower(printer *Printer) *TuningTower {
	tuning_tower, ok := lookup_object(printer, "tuning_tower").(*TuningTower)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "tuning_tower", lookup_object(printer, "tuning_tower")))
	}

	return tuning_tower
}

func lookup_object(printer *Printer, module string) interface{} {
	obj := printer.Lookup_object(module, object.Sentinel{})
	//if err != nil {
	//	panic("lookup object " + module + " not found")
	//}

	return obj
}

func MustLoadGcodeMacro(config *ConfigWrapper) *PrinterGCodeMacro {
	return config.Get_printer().Load_object(config, "gcode_macro", object.Sentinel{}).(*PrinterGCodeMacro)
}

func MustLoadGcodeMove(config *ConfigWrapper) *GCodeMove {
	return config.Get_printer().Load_object(config, "gcode_move", object.Sentinel{}).(*GCodeMove)
}

func MustLookupConfigfile(printer *Printer) *PrinterConfig {
	config, ok := lookup_object(printer, "configfile").(*PrinterConfig)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "configfile", lookup_object(printer, "configfile")))
	}

	return config
}

func MustLookupWebhooks(printer *Printer) *WebHooks {
	webhooks, ok := lookup_object(printer, "webhooks").(*WebHooks)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "webhooks", lookup_object(printer, "webhooks")))
	}

	return webhooks
}

func MustLookupMCU(printer *Printer, s string) *MCU {
	return Get_printer_mcu(printer, s)
}

func MustLoadPrintStats(config *ConfigWrapper) *PrintStats {
	return config.Get_printer().Load_object(config, "print_stats", object.Sentinel{}).(*PrintStats)
}

func MustLookupProbe(printer *Printer) *PrinterProbe {
	probe, ok := lookup_object(printer, "probe").(*PrinterProbe)
	if !ok {
		panic(fmt.Errorf("lookup object %s type invalid: %#v", "probe", lookup_object(printer, "probe")))
	}

	return probe
}
