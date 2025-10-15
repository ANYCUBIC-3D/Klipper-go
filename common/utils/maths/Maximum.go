package maths

import (
	math "k3c/common/cmath"
)

func Maximum(arr1, arr2 []float64) []float64 {
	if len(arr1) != len(arr2) || (len(arr1) == 0 || len(arr2) == 0) {
		panic("is error!")
	}
	result := make([]float64, len(arr1))
	for i := 0; i < len(arr1); i++ {
		result[i] = math.Max(arr1[i], arr2[i])
	}
	return result
}
