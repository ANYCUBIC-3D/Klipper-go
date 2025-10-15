package value

import (
	"math"
	"reflect"
)

var (
	Float64None float64     = math.NaN()     // float64 none placeholder
	StringNone  string      = "____none____" // string none placeholder
	None        interface{} = nil            // none
)

func IsNone(i interface{}) bool {
	if i == nil {
		return true
	}
	switch v := i.(type) {
	case float64:
		return math.IsNaN(v)
	case string:
		return v == StringNone
	}
	switch reflect.TypeOf(i).Kind() {
	case reflect.Chan,
		reflect.Func,
		reflect.Map,
		reflect.Ptr,
		reflect.UnsafePointer,
		reflect.Interface,
		reflect.Slice:
		return reflect.ValueOf(i).IsNil()
	}

	return false
}

func IsNotNone(i interface{}) bool {
	return !IsNone(i)
}

func True(i interface{}) bool {
	return !False(i)
}

func False(i interface{}) bool {
	switch v := i.(type) {
	case bool:
		return v == false
	case string:
		return len(v) == 0
	case int8, int16, int32, int64, int:
		return reflect.ValueOf(i).Int() == 0
	case uint8, uint16, uint32, uint64, uint:
		return reflect.ValueOf(i).Uint() == 0
	case float32, float64:
		return reflect.ValueOf(i).Float() == 0
	}
	return false
}

func Not(i interface{}) bool {
	return False(i)
}
