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

func CRoundf(v float32) float32 {
	r := C.roundf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoRoundf(v float32) float32 {
	return (float32)(math.Round(float64(v)))
}

func Round(v float64) float64 {
	r := C.round(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r))
}
