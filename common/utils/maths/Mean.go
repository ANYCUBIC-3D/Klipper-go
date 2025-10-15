package maths

func Mean(arr [][]float64, axis interface{}) []float64 {
	var means []float64
	// If axis is nil, calculate the average value of all elements.
	if axis == nil {
		var sum float64
		var count int
		for _, row := range arr {
			for _, val := range row {
				sum += val
				count++
			}
		}

		means = append(means, sum/float64(count))
	} else {
		var axisInt int
		switch axis.(type) {
		case int:
			axisInt = axis.(int)
		}

		if axisInt == 0 {
			for j := 0; j < len(arr[0]); j++ {
				var sum float64
				for i := 0; i < len(arr); i++ {
					sum += arr[i][j]
				}

				means = append(means, sum/float64(len(arr)))
			}
		} else if axisInt == 1 {
			for i := 0; i < len(arr); i++ {
				var sum float64

				for j := 0; j < len(arr[i]); j++ {
					sum += arr[i][j]
				}
				means = append(means, sum/float64(len(arr[i])))
			}
		}
	}
	return means
}
