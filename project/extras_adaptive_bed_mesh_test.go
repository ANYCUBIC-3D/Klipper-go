package project

import (
	"math"
	"testing"
)

func nearlyEqual(a, b, tol float64) bool {
	return math.Abs(a-b) <= tol
}

func TestParseVec2CSV(t *testing.T) {
	v, err := parseVec2CSV("10.5,20")
	if err != nil {
		t.Fatalf("expected no error, got %v", err)
	}
	if !nearlyEqual(v.X, 10.5, 1e-6) || !nearlyEqual(v.Y, 20, 1e-6) {
		t.Fatalf("unexpected vector: %+v", v)
	}

	if _, err := parseVec2CSV("invalid"); err == nil {
		t.Fatalf("expected error for invalid input")
	}
}

func TestGetPolygonMinMax(t *testing.T) {
	min, max, err := getPolygonMinMax([]vec2{
		{X: 3, Y: 4},
		{X: -1, Y: 5},
		{X: 2, Y: -3},
	})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if min.X != -1 || min.Y != -3 {
		t.Fatalf("unexpected min: %+v", min)
	}
	if max.X != 3 || max.Y != 5 {
		t.Fatalf("unexpected max: %+v", max)
	}
}

func TestApplyMinMaxMargin(t *testing.T) {
	min, max := applyMinMaxMargin(vec2{X: 10, Y: 20}, vec2{X: 30, Y: 40}, 5)
	if min.X != 5 || min.Y != 15 {
		t.Fatalf("unexpected min with margin: %+v", min)
	}
	if max.X != 35 || max.Y != 45 {
		t.Fatalf("unexpected max with margin: %+v", max)
	}
}

func TestLinspace(t *testing.T) {
	vals := linspace(0, 10, 5)
	if len(vals) != 5 {
		t.Fatalf("expected 5 values, got %d", len(vals))
	}
	if vals[0] != 0 || vals[len(vals)-1] != 10 {
		t.Fatalf("unexpected endpoints: %v", vals)
	}
	if !nearlyEqual(vals[1], 2.5, 1e-6) || !nearlyEqual(vals[2], 5, 1e-6) || !nearlyEqual(vals[3], 7.5, 1e-6) {
		t.Fatalf("unexpected interior values: %v", vals)
	}

	single := linspace(3, 9, 1)
	if len(single) != 1 || single[0] != 3 {
		t.Fatalf("expected single value equal to start, got %v", single)
	}
}

func TestGetMoveMinMax(t *testing.T) {
	min, max, err := getMoveMinMax([]vec2{{1, 2}, {-4, 5}, {3, -1}})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if min.X != -4 || min.Y != -1 {
		t.Fatalf("unexpected min: %+v", min)
	}
	if max.X != 3 || max.Y != 5 {
		t.Fatalf("unexpected max: %+v", max)
	}
}

func TestGetProbePointsSerpentine(t *testing.T) {
	abm := &AdaptiveBedMesh{
		maxProbeHorizontalDistance: 35,
		maxProbeVerticalDistance:   35,
		minimumAxisProbeCounts:     3,
		bedMeshConfigAlgorithm:     "lagrange",
	}
	min := vec2{X: 0, Y: 0}
	max := vec2{X: 100, Y: 100}

	xCount, yCount, pts, relIdx := abm.getProbePoints(min, max)
	if xCount < 3 || yCount < 3 {
		t.Fatalf("expected minimum probe counts of 3, got %d x %d", xCount, yCount)
	}
	if xCount > 6 || yCount > 6 {
		t.Fatalf("lagrange algorithm should cap probe counts at 6, got %d x %d", xCount, yCount)
	}

	expectedLen := xCount * yCount
	if len(pts) != expectedLen {
		t.Fatalf("unexpected number of probe points: %d", len(pts))
	}
	if relIdx < 0 || relIdx >= expectedLen {
		t.Fatalf("relative index out of range: %d (len=%d)", relIdx, expectedLen)
	}

	if xCount >= 2 && yCount >= 2 {
		firstRow := pts[:xCount]
		secondRow := pts[xCount : 2*xCount]
		if !nearlyEqual(firstRow[0].X, min.X, 1e-6) || !nearlyEqual(firstRow[xCount-1].X, max.X, 1e-6) {
			t.Fatalf("unexpected first row ordering: %v", firstRow)
		}
		if !nearlyEqual(secondRow[0].X, firstRow[xCount-1].X, 1e-6) || !nearlyEqual(secondRow[xCount-1].X, firstRow[0].X, 1e-6) {
			t.Fatalf("expected serpentine ordering, got %v", secondRow)
		}
	}
}

func TestApplyProbePointLimits(t *testing.T) {
	abm := &AdaptiveBedMesh{
		minimumAxisProbeCounts: 3,
		bedMeshConfigAlgorithm: "lagrange",
	}
	x, y := abm.applyProbePointLimits(10, 8)
	if x != 6 || y != 6 {
		t.Fatalf("lagrange limits should clamp counts to 6, got %d x %d", x, y)
	}

	abm.bedMeshConfigAlgorithm = "bicubic"
	x, y = abm.applyProbePointLimits(2, 8)
	if x != 4 || y != 8 {
		t.Fatalf("bicubic should raise shorter side when opposite exceeds 6, got %d x %d", x, y)
	}
}

func TestParseLinearMove(t *testing.T) {
	mv, err := parseLinearMove([]string{"G1", "X10", "Y20", "Z5", "E1.2", "F1500"})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !mv.HasX || mv.X != 10 || !mv.HasY || mv.Y != 20 || !mv.HasZ || mv.Z != 5 {
		t.Fatalf("unexpected move values: %+v", mv)
	}
	if !mv.HasE || mv.E != 1.2 || !mv.HasF || mv.F != 1500 {
		t.Fatalf("unexpected extrusion/feedrate flags: %+v", mv)
	}
}

func TestIsEven(t *testing.T) {
	if !isEven(2) || isEven(3) {
		t.Fatalf("isEven should detect even numbers correctly")
	}
}
