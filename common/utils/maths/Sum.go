package maths

func Sum1(arr []float64) float64 {
	sum := 0.
	for i := 0; i < len(arr); i++ {
		sum += arr[i]
	}
	return sum
}

func Sum2(arr [][]float64, axis int) []float64 {
	var result []float64
	if axis == 1 {
		for i := 0; i < len(arr); i++ {
			sum := 0.
			for j := 0; j < len(arr[i]); j++ {
				sum += arr[i][j]
			}
			result = append(result, sum)
		}
	} else if axis == 0 {
		for j := 0; j < len(arr[0]); j++ {
			sum := 0.
			for i := 0; i < len(arr); i++ {
				sum += arr[i][j]
			}
			result = append(result, sum)
		}
	}

	return result
}
