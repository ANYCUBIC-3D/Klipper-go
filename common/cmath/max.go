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

func CMaxf(x, y float32) float32 {
	r := C.fmaxf(C.float(x), C.float(y))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoMaxf(x, y float32) float32 {
	return (float32)(math.Max(float64(x), float64(y)))
}

func Max(x, y float64) float64 {
	r := C.fmax(C.double(x), C.double(y))
	return *(*float64)((unsafe.Pointer)(&r))
}
