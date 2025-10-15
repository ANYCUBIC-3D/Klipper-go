package maths

func Arange(start, end, step float64) []float64 {
	var result []float64
	if start <= end {
		for i := start; i < end; i += step {
			result = append(result, i)
		}
	} else {
		for i := start; i > end; i += step {
			result = append(result, i)
		}
	}

	return result
}
