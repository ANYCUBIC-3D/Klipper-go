package maths

import "sort"

func Interp(x, xp, fp []float64) []float64 {
	var y []float64

	for _, v := range x {
		idx := sort.SearchFloat64s(xp, v)
		if idx == 0 {
			y = append(y, fp[0])
			continue
		}
		if idx == len(xp) {
			y = append(y, fp[len(fp)-1])
			continue
		}

		a := fp[idx-1] + (fp[idx]-fp[idx-1])*(v-xp[idx-1])/(xp[idx]-xp[idx-1])
		y = append(y, a)
	}

	return y
}
