package maths

import (
	"fmt"
	"k3c/common/logger"
	"math"
	"sort"
)

func Saturate(val float64, min float64, max float64) float64 {
	if val < min {
		return min
	} else if val > max {
		return max
	} else {
		return val
	}
}

func LinearInterpolate(x1, y1, x2, y2, x float64) float64 {
	if y1 < y2 {
		return y1
	}
	return y1 + (y2-y1)*(x-x1)/(x2-x1)
}

func Float_binarySearch(xs []float64, target float64) int {
	low, high := 0, len(xs)-1
	for low <= high {
		mid := low + (high-low)/2
		if xs[mid] < target {
			low = mid + 1
		} else if xs[mid] > target {
			high = mid - 1
		} else {
			return mid
		}
	}
	return low
}

func InterpolateWithBinarySearch(xs, ys []float64, newX float64) float64 {
	var interpolatedY float64

	if newX <= xs[0] {
		interpolatedY = LinearInterpolate(xs[0], ys[0], xs[1], ys[1], newX)
	} else if newX >= xs[len(xs)-1] {
		interpolatedY = LinearInterpolate(xs[len(xs)-2], ys[len(xs)-2], xs[len(xs)-1], ys[len(xs)-1], newX)
	} else {
		idx := Float_binarySearch(xs, newX)
		if idx == 0 || idx >= len(xs) {
			logger.Debugf("Unexpected index %d for value %.2f\n", idx, newX)
		}
		interpolatedY = LinearInterpolate(xs[idx-1], ys[idx-1], xs[idx], ys[idx], newX)
	}

	return interpolatedY
}

func Economyqr_Decomposition(A [][]float64) ([][]float64, [][]float64, error) {
	m := len(A)
	if m == 0 {
		return nil, nil, fmt.Errorf("is null")
	}
	n := len(A[0])

	Q := make([][]float64, m)
	for i := range Q {
		Q[i] = make([]float64, m)
		if i < len(Q[i]) {
			Q[i][i] = 1
		}
	}

	R := make([][]float64, m)
	for i := range R {
		R[i] = append([]float64{}, A[i]...)
	}

	for k := 0; k < n; k++ {
		normX := 0.0
		for i := k; i < m; i++ {
			normX += R[i][k] * R[i][k]
		}
		normX = math.Sqrt(normX)
		if R[k][k] > 0 {
			normX = -normX
		}

		v := make([]float64, m)
		for i := k; i < m; i++ {
			if i == k {
				v[i] = R[i][k] - normX
			} else {
				v[i] = R[i][k]
			}
		}

		normV := 0.0
		for i := k; i < m; i++ {
			normV += v[i] * v[i]
		}
		normV = math.Sqrt(normV)
		if normV == 0 {
			continue
		}
		for i := k; i < m; i++ {
			v[i] /= normV
		}

		for j := k; j < n; j++ {
			dot := 0.0
			for i := k; i < m; i++ {
				dot += v[i] * R[i][j]
			}
			for i := k; i < m; i++ {
				R[i][j] -= 2 * v[i] * dot
			}
		}

		for i := 0; i < m; i++ {
			dot := 0.0
			for j := k; j < m; j++ {
				dot += v[j] * Q[i][j]
			}
			for j := k; j < m; j++ {
				Q[i][j] -= 2 * v[j] * dot
			}
		}
	}

	QTrim := make([][]float64, m)
	for i := range QTrim {
		QTrim[i] = make([]float64, n)
		copy(QTrim[i], Q[i][:n])
	}

	RTrim := make([][]float64, n)
	for i := range RTrim {
		RTrim[i] = make([]float64, n)
		copy(RTrim[i], R[i][:n])
	}

	return QTrim, RTrim, nil
}

func transposeMatrix(A [][]float64) [][]float64 {
	m, n := len(A), len(A[0])
	At := make([][]float64, n)
	for i := 0; i < n; i++ {
		At[i] = make([]float64, m)
		for j := 0; j < m; j++ {
			At[i][j] = A[j][i]
		}
	}
	return At
}

func multiplyMatrixVector(A [][]float64, x []float64) []float64 {
	m, n := len(A), len(A[0])
	if len(x) != n {
		panic("Matrix and vector dimension mismatch")
	}
	result := make([]float64, m)
	for i := 0; i < m; i++ {
		for j := 0; j < n; j++ {
			result[i] += A[i][j] * x[j]
		}
	}
	return result
}

func subtractVectors(a, b []float64) []float64 {
	if len(a) != len(b) {
		panic("vector dimension mismatch")
	}
	result := make([]float64, len(a))
	for i := 0; i < len(a); i++ {
		result[i] = a[i] - b[i]
	}
	return result
}

func solveUpperTriangular(R [][]float64, b []float64) ([]float64, error) {
	n := len(R)
	if len(b) != n {
		return nil, fmt.Errorf("Matrix and vector dimension mismatch")
	}

	x := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		if math.Abs(R[i][i]) < 1e-10 {
			return nil, fmt.Errorf("Matrix singularity, unsolvable")
		}
		sum := 0.0
		for j := i + 1; j < n; j++ {
			sum += R[i][j] * x[j]
		}
		x[i] = (b[i] - sum) / R[i][i]
	}
	return x, nil
}

func solveQR(R, Q [][]float64, Dy []float64) ([]float64, error) {
	Qt := transposeMatrix(Q)

	QtDy := multiplyMatrixVector(Qt, Dy)

	p, err := solveUpperTriangular(R, QtDy)
	if err != nil {
		return nil, fmt.Errorf("QR Decomposition solver failure: %v", err)
	}

	return p, nil
}

func computeLeverage(Q [][]float64) []float64 {
	rows := len(Q)
	if rows == 0 {
		return nil
	}

	leverage := make([]float64, rows)

	for i := 0; i < rows; i++ {
		sum := 0.0
		for _, value := range Q[i] {
			sum += value * value
		}
		leverage[i] = sum
	}

	return leverage
}

func LinearFit(X [][]float64, Y []float64, W []float64, leverage_flag bool) ([]float64, []float64, []float64) {
	J := make([][]float64, len(X))
	Dy := make([]float64, len(Y))
	leverage := make([]float64, len(Y))

	if len(W) > 0 {
		sqrtw := make([]float64, len(W))
		for i := range W {
			sqrtw[i] = math.Sqrt(W[i])
		}
		n := len(X[0])
		for i := range X {
			J[i] = make([]float64, n)
			for j := 0; j < n; j++ {
				J[i][j] = sqrtw[i] * X[i][j]
			}
		}

		for i := range Y {
			Dy[i] = sqrtw[i] * Y[i]
		}
	} else {
		J = X
		Dy = Y
	}

	Q, R, _ := Economyqr_Decomposition(J)
	p, _ := solveQR(R, Q, Dy)
	Jp := multiplyMatrixVector(J, p)
	residual := subtractVectors(Dy, Jp)

	if false == leverage_flag {
		//leverage_flag = true
		leverage = computeLeverage(Q)
	}

	return p, leverage, residual
}

func computeMedian(data []float64) float64 {
	n := len(data)
	if n == 0 {
		return 0
	}
	if n%2 == 1 {
		return data[n/2]
	}
	return (data[n/2-1] + data[n/2]) / 2.0
}

func updateWeights(r []float64) []float64 {
	n := len(r)
	w := make([]float64, n)
	allZero := true

	for i, v := range r {
		absVal := math.Abs(v)
		if absVal < 1 {
			w[i] = math.Pow(1-math.Pow(v, 2), 2)
			allZero = false
		} else {
			w[i] = 0
		}
	}

	if allZero {
		for i := range w {
			w[i] = 1
		}
	}

	return w
}

func RobustFit(X [][]float64, Y []float64) []float64 {
	W := []float64{}

	p, leverage, residual := LinearFit(X, Y, W, false)

	p0 := make([]float64, len(p))
	P := len(p0)
	LenY := len(Y)
	h := make([]float64, LenY)
	for i, v := range leverage {
		if v > 0.9999 {
			h[i] = 0.9999
		} else {
			h[i] = v
		}
	}
	adjFactor := make([]float64, LenY)
	for i, v := range h {
		if v >= 1.0 {
			fmt.Errorf("value in h is too close to or greater than 1, causing division by zero")
		}
		adjFactor[i] = 1.0 / math.Sqrt(1.0-v)
	}
	//dfe := N-P
	//ols_s := snormr / math.Sqrt(float64(dfe))
	mean := 0.0
	for _, v := range Y {
		mean += v
	}
	mean /= float64(LenY)

	variance := 0.0
	for _, v := range Y {
		diff := v - mean
		variance += diff * diff
	}
	variance /= float64(LenY - 1)

	std := math.Sqrt(variance)

	tiny_s := 1e-6 * std

	D := 1e-6
	iter := 0
	iterlim := 50

	for iter < iterlim {
		iter += 1
		radj := make([]float64, LenY)
		for i := range residual {
			radj[i] = residual[i] * adjFactor[i]
		}

		rs := make([]float64, LenY)
		for i, v := range radj {
			rs[i] = math.Abs(v)
		}
		sort.Float64s(rs)
		subResiduals := rs[P-1:]
		median := computeMedian(subResiduals)
		sigma := median / 0.6745
		tune := 4.685

		normFactor := math.Max(tiny_s, sigma) * tune
		r := make([]float64, LenY)
		for i, v := range radj {
			r[i] = v / normFactor
		}
		bw := updateWeights(r)
		p0 = p

		p, _, _ = LinearFit(X, Y, bw, true)

		for i := 0; i < len(X); i++ {
			predicted := 0.0
			for j := 0; j < len(p); j++ {
				predicted += X[i][j] * p[j]
			}
			residual[i] = Y[i] - predicted
		}

		CheckFlagCnt := 0
		for i := range p {
			diff := math.Abs(p[i] - p0[i])
			maxAbs := math.Max(math.Abs(p[i]), math.Abs(p0[i]))
			if diff <= D*maxAbs {
				CheckFlagCnt++
			}
		}
		if CheckFlagCnt == len(p) {
			break
		}

	}

	return p
}

func BilinearInterpolation(x_target, y_target float64, x1, x2, y1, y2 float64, Q11, Q21, Q12, Q22 float64) float64 {
	R1 := Q11 + (x_target-x1)*(Q21-Q11)/(x2-x1)
	R2 := Q12 + (x_target-x1)*(Q22-Q12)/(x2-x1)

	P := R1 + (y_target-y1)*(R2-R1)/(y2-y1)

	return P
}

func Interpolate2D(x_target, y_target float64, x []float64, y []float64, grid [][]float64) float64 {
	var x1, x2, y1, y2 int

	for i := 0; i < len(x)-1; i++ {
		if x_target >= x[i] && x_target <= x[i+1] {
			x1 = i
			x2 = i + 1
			break
		}
	}

	for j := 0; j < len(y)-1; j++ {
		if y_target >= y[j] && y_target <= y[j+1] {
			y1 = j
			y2 = j + 1
			break
		}
	}

	Q11 := grid[x1][y1]
	Q21 := grid[x2][y1]
	Q12 := grid[x1][y2]
	Q22 := grid[x2][y2]

	return BilinearInterpolation(x_target, y_target, x[x1], x[x2], y[y1], y[y2], Q11, Q21, Q12, Q22)
}
