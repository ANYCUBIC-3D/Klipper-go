package maths

import (
	math "k3c/common/cmath"
)

//func Sin(arr [][]float64) [][]float64 {
//	var result [][]float64
//	for i := 0; i < len(arr); i++ {
//		var data []float64
//		for j := 0; j < len(arr[i]); j++ {
//			data = append(data, math.Sin(arr[i][j]))
//		}
//		result = append(result, data)
//	}
//	return result
//}
func Sin(arr [][]float64) [][]float64 {
	for i := 0; i < len(arr); i++ {
		for j := 0; j < len(arr[i]); j++ {
			arr[i][j] = math.Sin(arr[i][j])
		}
	}
	return arr
}
