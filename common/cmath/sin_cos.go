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

func CSincosf(v float32) (float32, float32) {
	r1 := C.sinf(C.float(v))
	r2 := C.cosf(C.float(v))
	return *(*float32)((unsafe.Pointer)(&r1)), *(*float32)((unsafe.Pointer)(&r2))
}

func GoSincosf(v float32) (float32, float32) {
	r1, r2 := math.Sincos(float64(v))
	return (float32)(r1), (float32)(r2)
}

func Sincos(v float64) (float64, float64) {
	r1 := C.sin(C.double(v))
	r2 := C.cos(C.double(v))
	return *(*float64)((unsafe.Pointer)(&r1)), *(*float64)((unsafe.Pointer)(&r2))
}
