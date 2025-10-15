package maths

import (
	"k3c/common/cmath"
)

func Rfft(data [][]float64, n int, axis int) [][]complex128 {
	// Check if n is given
	if n == 0 {
		n = len(data[axis])
	}

	// Check if axis is given
	if axis == 0 {
		// Transpose the data to make the last axis the transformation axis
		data = transpose(data)
		axis = len(data) - 1
	}

	// Check if n is valid
	if n < len(data[axis]) {
		// Crop the input
		data[axis] = data[axis][:n]
	} else if n > len(data[axis]) {
		// Pad the input with zeros
		padding := make([]float64, n-len(data[axis]))
		data[axis] = append(data[axis], padding...)
	}

	// Perform the FFT on the last axis
	result := make([][]complex128, len(data))
	for i := range result {
		result[i] = make([]complex128, n)
		for j := range result[i] {
			result[i][j] = complex(data[i][j], 0)
		}
		fft(result[i])
		result[i] = result[i][:n/2+1]
	}
	// Transpose the data to make the last axis the transformation axis
	res := make([][]complex128, len(result[0]))
	for i := range res {
		res[i] = make([]complex128, len(result))
		for j := range res[i] {
			res[i][j] = result[j][i]
		}
	}
	return res
}

func fft2(x []complex128) {
	n := len(x)
	if n == 1 {
		return
	}

	// Split the input into even and odd parts
	even := make([]complex128, n/2)
	odd := make([]complex128, n/2)
	for i := 0; i < n/2; i++ {
		even[i] = x[2*i]
		odd[i] = x[2*i+1]
	}

	// Recursively compute the FFT for even and odd parts
	fft(even)
	fft(odd)

	// Combine the even and odd parts using the butterfly operation
	for k := 0; k < n/2; k++ {
		t := cmath.CmplxExp(complex(0, -2*cmath.Pi*float64(k)/float64(n))) * odd[k]
		x[k] = even[k] + t
		x[k+n/2] = even[k] - t
	}
}

func transpose(data [][]float64) [][]float64 {
	result := make([][]float64, len(data[0]))
	for i := range result {
		result[i] = make([]float64, len(data))
		for j := range result[i] {
			result[i][j] = data[j][i]
		}
	}
	return result
}

// fft
// @see https://github.com/takatoh/fft/blob/main/fft.go#L8
// @see https://github.com/mjibson/go-dsp/blob/master/fft/fft.go#L72
func fft(a []complex128) {
	l := len(a)
	n := roundUpPowOfTwo(l)
	x := make([]complex128, n)
	copy(x, a)

	j := 0
	for i := 0; i < n; i++ {
		if i < j {
			x[i], x[j] = x[j], x[i]
		}
		m := n / 2
		for {
			if j < m {
				break
			}
			j = j - m
			m = m / 2
			if m < 2 {
				break
			}
		}
		j = j + m
	}
	kmax := 1
	for {
		if kmax >= n {
			copy(a, x)
			return
		}
		istep := kmax * 2
		for k := 0; k < kmax; k++ {
			theta := complex(0.0, -1.0*cmath.Pi*float64(k)/float64(kmax))
			for i := k; i < n; i += istep {
				j := i + kmax
				temp := x[j] * cmath.CmplxExp(theta)
				x[j] = x[i] - temp
				x[i] = x[i] + temp
			}
		}
		kmax = istep
	}
}

func roundUpPowOfTwo(n int) int {
	if n&(n-1) == 0 {
		return n
	}
	n--
	n |= n >> 1
	n |= n >> 2
	n |= n >> 4
	n |= n >> 8
	n |= n >> 16
	n |= n >> 32
	n++
	return n
}
