package maths

func Rfftfreq(n int, d float64) []float64 {
	result := make([]float64, n/2+1)
	for i := 0; i <= n/2; i++ {
		result[i] = float64(i) / (d * float64(n))

	}
	return result
}
