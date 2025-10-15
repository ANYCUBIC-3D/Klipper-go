package maths

import (
	"k3c/common/cmath"
	"math"
)

func Round(num float64, decimalPlaces int) float64 {
	multiplier := cmath.Pow(10, float64(decimalPlaces))
	return math.Round(num*multiplier) / multiplier
}
