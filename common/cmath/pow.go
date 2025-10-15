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

func CPowf(x, y float32) float32 {
	r := C.powf(C.float(x), C.float(y))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoPowf(x, y float32) float32 {
	return (float32)(math.Pow(float64(x), float64(y)))
}

func Pow(x, y float64) float64 {
	r := C.pow(C.double(x), C.double(y))
	return *(*float64)((unsafe.Pointer)(&r))
}
