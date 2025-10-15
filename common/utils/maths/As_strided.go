package maths

func AsStrided(arr []float64, shape []int, strides []int) *Ndarray {
	rows := shape[0]
	cols := shape[1]
	data := make([][]float64, rows)
	for i := 0; i < rows; i++ {
		data[i] = make([]float64, cols)
		for j := 0; j < cols; j++ {
			index := i*strides[0] + j*strides[1]
			if index >= len(arr) {
				panic("overflow")
			} else {
				data[i][j] = arr[index]
			}

		}
	}
	return &Ndarray{data, shape, strides}
}
