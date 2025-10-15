package maths

import "math"

func Max(x, y int) int {
	if x > y {
		return x
	} else {
		return y
	}
}

func Max64(x, y int64) int64 {
	if x > y {
		return x
	} else {
		return y
	}
}

func Max1(x, y, z int) int {
	t := Max(x, y)
	ret := Max(t, z)
	return ret
}

func Min(x, y int) int {
	if x > y {
		return y
	}
	return x
}

func Radians(a float64) float64 {
	return 4 * math.Atan(1.0) / 180 * a
}

// Round returns x rounded to the given unit.
// Tip: x is "arbitrary", maybe greater than 1.
// For example:
//     Round(0.363636, 0.001) // 0.364
//     Round(0.363636, 0.01)  // 0.36
//     Round(0.363636, 0.1)   // 0.4
//     Round(0.363636, 0.05)  // 0.35
//     Round(3.2, 1)          // 3
//     Round(32, 5)           // 30
//     Round(33, 5)           // 35
//     Round(32, 10)          // 30
//
// For details, see https://stackoverflow.com/a/39544897/1705598
//func Round(x, unit float64) float64 {
//	return math.Round(x/unit) * unit
//}

func FloorDiv(a, b int) int {
	return a / b
}

func PyMod(a, b int) int {
	return (a%b + b) % b
}

func Check_above_limit(a, b float64) bool {
	if a > b && math.Abs(a-b) > 0.0000001 {
		return true
	}
	return false
}

func Check_below_limit(a, b float64) bool {
	if a < b && math.Abs(b-a) > 0.0000001 {
		return true
	}
	return false
}

func Int64_conversion(data int64) int64 {
	if (data & 0xf0000000) == 0xf0000000 {
		return -(data ^ 0xffffffff + 1)
	}

	return data
}
