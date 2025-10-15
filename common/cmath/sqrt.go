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

func CSqrtf(v float32) float32 {
	r := C.sqrtf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoSqrtf(v float32) float32 {
	return (float32)(math.Sqrt(float64(v)))
}

func Sqrt(v float64) float64 {
	r := C.sqrt(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r))
}
