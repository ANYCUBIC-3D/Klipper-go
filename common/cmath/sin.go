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

func CSinf(v float32) float32 {
	r := C.sinf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r))
}

func GoSinf(v float32) float32 {
	return (float32)(math.Sin(float64(v)))
}

func Sin(v float64) float64 {
	r := C.sin(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r))
}
