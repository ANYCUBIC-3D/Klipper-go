package maths

func LinearRegression(X [][]float64, y []float64) (float64, float64) {
	n := len(X)
	sumX, sumY := 0.0, 0.0
	sumXY, sumXSquare := 0.0, 0.0

	for i := 0; i < n; i++ {
		sumX += X[i][0]
		sumY += y[i]
		sumXY += X[i][0] * y[i]
		sumXSquare += X[i][0] * X[i][0]
	}

	k := (float64(n)*sumXY - sumX*sumY) / (float64(n)*sumXSquare - sumX*sumX)
	b := (sumY - k*sumX) / float64(n)

	return k, b
}
