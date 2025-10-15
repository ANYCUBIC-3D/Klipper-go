package project

import (
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/collections"
	"reflect"
	"strings"
)

// Pin name handling
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// ######################################################################
// Command translation
// ######################################################################
type PinError struct {
}
type PinResolver struct {
	validate_aliases bool
	reserved         map[string]string
	aliases          map[string]string
	active_pins      map[string]string
}

func NewPinResolver(validate_aliases bool) *PinResolver {
	self := PinResolver{}

	self.validate_aliases = validate_aliases
	self.reserved = map[string]string{}
	self.aliases = map[string]string{}
	self.active_pins = map[string]string{}

	return &self
}
func (self *PinResolver) Reserve_pin(pin, reserve_name string) {
	v, ok := self.reserved[pin]
	if ok {
		if v != reserve_name {
			logger.Error(fmt.Sprintf("Pin %s reserved for %s - can't reserve for %s",
				pin, self.reserved[pin], reserve_name))
		}
	}
	self.reserved[pin] = reserve_name
}
func (self *PinResolver) alias_pin(alias, pin string) {
	v, ok := self.active_pins[alias]
	if ok {
		if v != pin {
			logger.Error(fmt.Sprintf("Alias %s mapped to %s - can't alias to %s",
				alias, self.aliases[alias], pin))
		}
	}
	if strings.Index(pin, "^~!:") != -1 || strings.TrimSpace(pin) != pin {
		logger.Errorf("Invalid pin alias '%s'\n", pin)
	}
	_, ok1 := self.active_pins[alias]
	if ok1 {
		pin = self.aliases[pin]
	}
	self.aliases[alias] = pin
	for existing_alias, existing_pin := range self.aliases {
		if existing_pin == alias {
			self.aliases[existing_alias] = pin
		}
	}
}

// 'config_endstop oid=0 pin=PB8 pull_up=0' get value of pin
func (self *PinResolver) Update_command(cmd string) string {
	//def pin_fixup(m):
	//name = m.group('name')
	//pin_id = self.aliases.get(name, name)
	//if (name != self.active_pins.setdefault(pin_id, name)
	//	and self.validate_aliases):
	//raise error("pin %s is an alias for %s" % (
	//	name, self.active_pins[pin_id]))
	//if pin_id in self.reserved:
	//raise error("pin %s is reserved for %s" % (
	//	name, self.reserved[pin_id]))
	//return m.group('prefix') + str(pin_id)
	return strings.TrimSpace(cmd)
}

//######################################################################
// Pin to chip mapping
//######################################################################

type PrinterPins struct {
	//error = error
	chips                map[string]interface{}
	active_pins          map[string]interface{}
	pin_resolvers        map[string]*PinResolver
	allow_multi_use_pins map[string]interface{}
}

func NewPrinterPins() *PrinterPins {
	self := PrinterPins{}
	self.chips = map[string]interface{}{}
	self.active_pins = map[string]interface{}{}
	self.pin_resolvers = map[string]*PinResolver{}
	self.allow_multi_use_pins = map[string]interface{}{}
	return &self
}
func (self *PrinterPins) Parse_pin(pin_desc string, can_invert bool, can_pullup bool) map[string]interface{} {
	desc := strings.TrimSpace(pin_desc)
	pullup := 0
	invert := 0
	if can_pullup && (strings.HasPrefix(desc, "^") || strings.HasPrefix(desc, "~")) {
		pullup = 1
		if strings.HasPrefix(desc, "~") {
			pullup = -1
		}
		desc = strings.TrimSpace(desc[1:])
	}
	if can_invert && strings.HasPrefix(desc, "!") {
		invert = 1
		desc = strings.TrimSpace(desc[1:])
	}
	chip_name := ""
	pin := ""
	if strings.Index(desc, ":") == -1 {
		chip_name = "mcu"
		pin = desc
	} else {
		strs := strings.Split(desc, ":")
		chip_name = strs[0]
		pin = strs[1]
	}
	chip, ok := self.chips[chip_name]
	if !ok && chip == nil {
		logger.Error(fmt.Sprintf("Unknown pin chip name %s",
			chip_name))
	}
	if strings.Index(pin, "^~!:") != -1 || strings.TrimSpace(pin) != pin {
		format := ""
		if can_pullup {
			format += "[^~] "
		}
		if can_invert {
			format += "[!] "
		}
		logger.Errorf("Invalid pin description '%s'\nFormat is: %s[chip_name:] pin_name",
			pin_desc, format)
	}
	pin_params := map[string]interface{}{"chip": self.chips[chip_name], "chip_name": chip_name,
		"pin": pin, "invert": invert, "pullup": pullup}
	return pin_params
}
func (self *PrinterPins) Lookup_pin(pin_desc string, can_invert bool, can_pullup bool,
	share_type interface{}) map[string]interface{} {
	pin_params := self.Parse_pin(pin_desc, can_invert, can_pullup)
	pin := pin_params["pin"]
	share_name := fmt.Sprintf("%s:%s", pin_params["chip_name"], pin)
	_, ok := self.active_pins[share_name]
	if ok {
		share_params := self.active_pins[share_name].(map[string]interface{})
		_, ok1 := self.allow_multi_use_pins[share_name]
		if ok1 {

		} else if share_type == nil || share_type != share_params["share_type"] {
			panic(fmt.Errorf("pin %s used multiple times in config", pin))
		} else if pin_params["invert"] != share_params["invert"] || pin_params["pullup"] != share_params["pullup"] {
			panic(fmt.Errorf("Shared pin %s must have same polarity", pin))
		}
		return share_params
	}
	pin_params["share_type"] = share_type
	self.active_pins[share_name] = pin_params
	return pin_params
}
func (self *PrinterPins) Setup_pin(pin_type, pin_desc string) interface{} {

	can_invert := collections.Contains([]string{"endstop", "digital_out", "pwm"}, pin_type)
	can_pullup := collections.Contains([]string{"endstop"}, pin_type)
	pin_params := self.Lookup_pin(pin_desc, can_invert, can_pullup, nil)

	switch reflect.TypeOf(pin_params["chip"]).Elem().Name() {
	case "MCU":
		return pin_params["chip"].(*MCU).Setup_pin(pin_type, pin_params)
	case "PrinterProbe":
		return pin_params["chip"].(*PrinterProbe).Setup_pin(pin_type, pin_params)
	case "TMCVirtualPinHelper":
		return pin_params["chip"].(*TMCVirtualPinHelper).Setup_pin(pin_type, pin_params)
	}
	return nil
}
func (self *PrinterPins) Reset_pin_sharing(pin_params map[string]interface{}) {
	share_name := fmt.Sprintf("%s:%s", pin_params["chip_name"], pin_params["pin"])
	delete(self.active_pins, share_name)
}
func (self *PrinterPins) Get_pin_resolver(chip_name string) *PinResolver {
	v, ok := self.pin_resolvers[chip_name]
	if !ok {

		logger.Error(fmt.Sprintf("Unknown chip name '%s'", chip_name))

	}

	return v
}
func (self *PrinterPins) Register_chip(chip_name string, chip interface{}) {
	chip_name = strings.TrimSpace(chip_name)
	_, ok := self.chips[chip_name]
	if ok {

		logger.Error(fmt.Sprintf("Duplicate chip name '%s'", chip_name))

	}
	self.chips[chip_name] = chip
	self.pin_resolvers[chip_name] = NewPinResolver(true)
}
func (self *PrinterPins) allow_multi_use_pin(pin_desc string) {
	pin_params := self.Parse_pin(pin_desc, false, false)
	share_name := fmt.Sprintf("%s:%s", pin_params["chip_name"], pin_params["pin"])
	self.allow_multi_use_pins[share_name] = true
}
func Add_printer_objects_pins(config *ConfigWrapper) {
	config.Get_printer().Add_object("pins", NewPrinterPins())
}
