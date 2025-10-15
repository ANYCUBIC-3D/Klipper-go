package maths

func Outer(u, v []float64) [][]float64 {
	m := len(u)
	n := len(v)
	res := make([][]float64, m)
	for i := 0; i < m; i++ {
		res[i] = make([]float64, n)
		for j := 0; j < n; j++ {
			res[i][j] = u[i] * v[j]
		}
	}
	return res
}

//func main() {
//	u := []float64{0, 11.405586834580497,22.811173669160993,34.21676050374149,45.62234733832199,57.027934172902484,
//		68.43352100748298,79.83910784206347,91.24469467664397,102.65028151122448}
//	v := []float64{0, 0.0033545}
//	res := outer(u, v)
//	fmt.Println(res)
//}
