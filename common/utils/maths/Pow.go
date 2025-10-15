package maths

import (
	math "k3c/common/cmath"
)

func Pow1D(data []float64, pow float64) []float64 {
	res := make([]float64, len(data))
	for i, val := range data {
		res[i] = math.Pow(val, pow)
	}
	return res
}
