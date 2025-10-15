package cmath

/*
#cgo CFLAGS: -g -Wall
#cgo LDFLAGS: -lm
#include <math.h>
*/
import "C"
import (
	"math"
	"unsafe"
)

func CCosf(v float32) float32 {
	r := C.cosf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoCosf(v float32) float32 {
	return (float32)(math.Cos(float64(v)))
}

func Cos(v float64) float64 {
	r := C.cos(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r))
}
