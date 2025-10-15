package maths

import (
	math "k3c/common/cmath"
)

func Sqrt1(arr []float64) []float64 {
	result := make([]float64, len(arr))
	for i := 0; i < len(arr); i++ {
		result[i] = math.Sqrt(arr[i])
	}
	return result
}

func Sqrt2(arr [][]float64) [][]float64 {
	result := make([][]float64, len(arr))
	for i := 0; i < len(arr); i++ {
		data := make([]float64, len(arr[i]))
		for j := 0; j < len(arr[i]); j++ {
			data[j] = math.Sqrt(arr[i][j])
		}
		result[i] = data
	}
	return result
}
