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

func CExpf(v float32) float32 {
	r := C.expf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoExpf(v float32) float32 {
	return (float32)(math.Exp(float64(v)))
}

func Exp(v float64) float64 {
	r := C.exp(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r))
}
