package maths

import (
	math "k3c/common/cmath"
)

func Exp(arr [][]float64) [][]float64 {
	result := make([][]float64, len(arr))
	for i := 0; i < len(arr); i++ {
		data := make([]float64, len(arr[i]))
		for j := 0; j < len(arr[i]); j++ {
			data[j] = math.Exp(arr[i][j])
		}
		result[i] = data
	}
	return result
}
