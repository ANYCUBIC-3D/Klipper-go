package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/object"
)

func Load_config_heater_Generic(config *ConfigWrapper) interface{} {
	pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{})
	pheater, ok := pheaters.(*PrinterHeaters)
	if !ok {
		logger.Errorf("pheaters type is %T not *PrinterHeaters\n", pheater)
		panic(fmt.Errorf("pheaters type is %T not *PrinterHeaters", pheater))
	}
	return pheater.Setup_heater(config, "")
}
