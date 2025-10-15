package cmath

import "math"

func CmplxExp(x complex128) complex128 {
	switch re, im := real(x), imag(x); {
	case math.IsInf(re, 0):
		switch {
		case re > 0 && im == 0:
			return x
		case math.IsInf(im, 0) || math.IsNaN(im):
			if re < 0 {
				return complex(0, math.Copysign(0, im))
			} else {
				return complex(math.Inf(1.0), math.NaN())
			}
		}
	case math.IsNaN(re):
		if im == 0 {
			return complex(math.NaN(), im)
		}
	}
	r := Exp(real(x))
	s, c := Sincos(imag(x))
	return complex(r*c, r*s)
}
