package maths

func Conjugate(data [][]complex128) [][]complex128 {
	result := make([][]complex128, len(data))
	for i := 0; i < len(data); i++ {
		arr := make([]complex128, len(data[i]))
		for j := 0; j < len(data[i]); j++ {
			arr[j] = complex(real(data[i][j]), -imag(data[i][j]))
		}
		result[i] = arr
	}
	return result
}
