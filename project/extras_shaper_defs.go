package project

import (
	math "k3c/common/cmath"
)

const (
	SHAPER_VIBRATION_REDUCTION = 20.0
	DEFAULT_DAMPING_RATIO      = 0.1
)

func Get_none_shaper() ([]float64, []float64) {
	return []float64{}, []float64{}
}

func Get_zv_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - math.Pow(damping_ratio, 2))
	K := math.Exp(-damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)
	A := []float64{1.0, K}
	T := []float64{0., 0.5 * t_d}
	return A, T
}

func Get_zvd_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - math.Pow(damping_ratio, 2))
	K := math.Exp(-damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)
	A := []float64{1.0, 2.0 * K, math.Pow(K, 2)}
	T := []float64{0., 0.5 * t_d, t_d}
	return A, T
}

func Get_mzv_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - math.Pow(damping_ratio, 2))
	K := math.Exp(-0.75 * damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)

	a1 := 1.0 - 1.0/math.Sqrt(2.0)
	a2 := (math.Sqrt(2.0) - 1.0) * K
	a3 := a1 * K * K

	A := []float64{a1, a2, a3}
	T := []float64{0., .375 * t_d, .75 * t_d}
	return A, T
}

func Get_ei_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	// vibration tolerance
	v_tol := 1.0 / SHAPER_VIBRATION_REDUCTION
	df := math.Sqrt(1. - math.Pow(damping_ratio, 2))
	K := math.Exp(-damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)

	a1 := 0.25 * (1.0 + v_tol)
	a2 := 0.5 * (1.0 - v_tol) * K
	a3 := a1 * K * K

	A := []float64{a1, a2, a3}
	T := []float64{0., .5 * t_d, t_d}
	return A, T
}

func Get_2hump_ei_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	// vibration tolerance
	v_tol := 1.0 / SHAPER_VIBRATION_REDUCTION
	df := math.Sqrt(1.0 - math.Pow(damping_ratio, 2))
	K := math.Exp(-damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)

	V2 := math.Pow(v_tol, 2)
	X := math.Pow(V2*(math.Sqrt(1.0-V2)+1.0), 1.0/3.0)
	a1 := (3.0*X*X + 2.0*X + 3.0*V2) / (16.0 * X)
	a2 := (0.5 - a1) * K
	a3 := a2 * K
	a4 := a1 * K * K * K

	A := []float64{a1, a2, a3, a4}
	T := []float64{0., .5 * t_d, t_d, 1.5 * t_d}
	return A, T
}

func Get_3hump_ei_shaper(shaper_freq, damping_ratio float64) ([]float64, []float64) {
	// vibration tolerance
	v_tol := 1.0 / SHAPER_VIBRATION_REDUCTION
	df := math.Sqrt(1.0 - math.Pow(damping_ratio, 2))
	K := math.Exp(-damping_ratio * math.Pi / df)
	t_d := 1.0 / (shaper_freq * df)

	K2 := K * K
	a1 := 0.0625 * (1.0 + 3.0*v_tol + 2.0*math.Sqrt(2.0*(v_tol+1.0)*v_tol))
	a2 := 0.25 * (1.0 - v_tol) * K
	a3 := (0.5*(1.0+v_tol) - 2.0*a1) * K2
	a4 := a2 * K2
	a5 := a1 * K2 * K2

	A := []float64{a1, a2, a3, a4, a5}
	T := []float64{0.0, 0.5 * t_d, t_d, 1.5 * t_d, 2.0 * t_d}
	return A, T
}

type InputShaperCfg struct {
	Name      string
	Init_func func(shaper_freq, damping_ratio float64) ([]float64, []float64)
	Min_freq  float64
}

// min_freq for each shaper is chosen to have projected max_accel ~= 1500
var INPUT_SHAPERS = []InputShaperCfg{
	{"zv", Get_zv_shaper, 21.0},
	{"mzv", Get_mzv_shaper, 23.0},
	{"zvd", Get_zvd_shaper, 29.0},
	{"ei", Get_ei_shaper, 29.0},
	{"2hump_ei", Get_2hump_ei_shaper, 39.0},
	{"3hump_ei", Get_3hump_ei_shaper, 48.0},
}
