package maths

import (
	math "k3c/common/cmath"
)

func factorial(n int) int {
	sum := 1
	for i := 1; i <= n; i++ {
		sum *= i

	}
	return sum
}

func i0(n int, x float64) float64 {
	I0_x := 1.0

	for i := 1; i <= n; i++ {
		I0_x += math.Pow(math.Pow(x/2, float64(i))/float64(factorial(i)), 2)
	}
	return I0_x
}

func Kaiser(m int, beta float64) []float64 {
	result := make([]float64, m)
	for j := 0; j < m; j++ {
		val := i0(20, beta*math.Sqrt(1-math.Pow(2*float64(j)/float64(m-1)-1, 2))) / i0(20, beta)
		result[j] = val
	}
	return result
}
