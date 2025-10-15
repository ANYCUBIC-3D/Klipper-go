package ini

import (
	"errors"
	"strconv"
	"strings"
)

type Section struct {
	name    string
	keys    map[string]*Key
	keyList []string
}

type Key struct {
	section *Section
	name    string
	value   string
}

func NewSection(name string) *Section {
	return &Section{
		name:    name,
		keys:    make(map[string]*Key),
		keyList: make([]string, 0),
	}
}

func (sec *Section) Name() string {
	if sec == nil {
		panic("nil section")
	}
	return sec.name
}

func (sec *Section) HasKey(name string) bool {
	_, err := sec.GetKey(name)
	if err != nil {
		return false
	}
	return true
}

func (sec *Section) GetKey(name string) (*Key, error) {
	kv, ok := sec.keys[name]
	if !ok {
		return nil, errors.New(name + " not exists")
	}

	return kv, nil
}

func (sec *Section) Keys() []*Key {
	var keys = make([]*Key, 0, len(sec.keys))
	for _, name := range sec.keyList {
		keys = append(keys, sec.keys[name])
	}
	return keys
}

func (sec *Section) Key(name string) *Key {
	key, err := sec.GetKey(name)
	if err != nil {
		key, _ = sec.NewKey(name, "")

	}
	return key
}

func (sec *Section) NewKey(name string, value string) (*Key, error) {
	name = strings.TrimSpace(name)
	if len(name) == 0 {
		return nil, errors.New(name + " empty key name")
	}

	key, ok := sec.keys[name]
	if !ok {
		key = newKey(sec, name, value)
		sec.keys[name] = key
		sec.keyList = append(sec.keyList, name)
	}
	return key, nil
}

func newKey(sec *Section, name string, value string) *Key {
	return &Key{
		section: sec,
		name:    name,
		value:   value,
	}
}

func (k *Key) Name() string {
	return k.name
}

func (k *Key) SetValue(value string) {
	k.value = value
}

func (k *Key) String() string {
	return k.value
}

func (k *Key) Int() (int, error) {
	return strconv.Atoi(k.value)
}

func (k *Key) MustInt() int {
	v, err := k.Int()
	if err != nil {
		panic(k.section.Name() + " key: " + k.name + " can't convert to int")
	}
	return v
}

func (k *Key) Float64() (float64, error) {
	return strconv.ParseFloat(k.value, 64)
}

func (k *Key) MustFloat64() float64 {
	v, err := strconv.ParseFloat(k.value, 64)

	if err != nil {
		panic(k.section.Name() + " key: " + k.name + " can't convert to float64")
	}
	return v
}

func (k *Key) Bool() (bool, error) {
	return strconv.ParseBool(k.value)
}

func (k *Key) MustBool() bool {
	v, err := k.Bool()
	if err != nil {
		panic(k.section.Name() + " key: " + k.name + " can't convert to bool")
	}
	return v
}
