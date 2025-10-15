package object

import "k3c/common/utils/reflects"

type Sentinel struct {
}

func IsNotSentinel(elm interface{}) bool {
	return !IsSentinel(elm)
}

func IsSentinel(elm interface{}) bool {
	_, ok := elm.(*Sentinel)
	if !ok {
		_, ok = elm.(Sentinel)
		return ok
	}
	return ok
}
func IsNil(elm interface{}) bool {
	return reflects.IsNil(elm)
}
