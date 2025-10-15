package collections

import "reflect"

func Contains(elems []string, v string) bool {
	for _, s := range elems {
		if v == s {
			return true
		}
	}
	return false
}
func ContainsKind(elems []reflect.Kind, v interface{}) bool {
	for _, s := range elems {
		if reflect.ValueOf(v).Kind() == s {
			return true
		}
	}
	return false
}
func ContainsStringMap(elems map[string]interface{}, v string) bool {
	_, ok := elems[v]
	return ok
}

func NotInKind(elems []reflect.Kind, v interface{}) bool {
	return !ContainsKind(elems, v)
}
func InStringMap(elems map[string]interface{}, v string) bool {
	return ContainsStringMap(elems, v)
}

func InInt(v int, elems []int) bool {
	for _, s := range elems {
		if v == s {
			return true
		}
	}
	return false
}
func FloatInterface(elems []float64) []interface{} {
	ret:=make([]interface{},len(elems))
	for _, s := range elems {
		ret=append(ret,s)
	}
	return ret
}