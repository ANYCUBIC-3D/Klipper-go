package project

import (
	"k3c/common/constants"
	"k3c/common/utils/cast"
	"math"
)

/**
# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical
*/

// Enum
var (
	ARC_PLANE_X_Y = 0
	ARC_PLANE_X_Z = 1
	ARC_PLANE_Y_Z = 2
)

// Enum
var (
	X_AXIS = 0
	Y_AXIS = 1
	Z_AXIS = 2
	E_AXIS = 3
)

type ArcSupport struct {
	printer            *Printer
	mm_per_arc_segment float64
	gcode_move         *GCodeMove
	gcode              *GCodeDispatch
	Coord              []string
	plane              int
}

func NewArcSupport(config *ConfigWrapper) ArcSupport {
	var self = ArcSupport{}
	self.printer = config.Get_printer()
	self.mm_per_arc_segment = config.Getfloat("resolution", 1., 0, 0, 0, 0.0, true)

	self.gcode_move = MustLoadGcodeMove(config)
	self.gcode = MustLookupGcode(self.printer)
	self.gcode.Register_command("G2", self.cmd_G2, false, "")
	self.gcode.Register_command("G3", self.cmd_G3, false, "")

	self.gcode.Register_command("G17", self.cmd_G17, false, "")
	self.gcode.Register_command("G18", self.cmd_G18, false, "")
	self.gcode.Register_command("G19", self.cmd_G19, false, "")

	self.Coord = self.gcode.Coord

	// backwards compatibility, prior implementation only supported XY
	self.plane = ARC_PLANE_X_Y
	return self
}

func (self *ArcSupport) cmd_G2(argv interface{}) error {
	self._cmd_inner(argv.(*GCodeCommand), true)
	return nil
}

func (self *ArcSupport) cmd_G3(argv interface{}) error {
	self._cmd_inner(argv.(*GCodeCommand), false)
	return nil
}

func (self *ArcSupport) cmd_G17(argv interface{}) error {
	self.plane = ARC_PLANE_X_Y
	return nil
}

func (self *ArcSupport) cmd_G18(argv interface{}) error {
	self.plane = ARC_PLANE_X_Z
	return nil
}

func (self *ArcSupport) cmd_G19(argv interface{}) error {
	self.plane = ARC_PLANE_Y_Z
	return nil
}

func (self *ArcSupport) _cmd_inner(gcmd *GCodeCommand, clockwise bool) {
	gcodestatus := self.gcode_move.Get_status(constants.NOW)
	currentPos := gcodestatus["gcode_position"].([]float64)
	absolute_coordinates := gcodestatus["absolute_coordinates"].(bool)
	if !absolute_coordinates {
		panic("G2/G3 does not support relative move mode")
	}
	absolute_extrude := gcodestatus["absolute_extrude"].(bool)
	// Parse parameters
	x := gcmd.Get_float("X", currentPos[0], nil, nil, nil, nil)
	y := gcmd.Get_float("Y", currentPos[1], nil, nil, nil, nil)
	z := gcmd.Get_float("Z", currentPos[2], nil, nil, nil, nil)
	targetPos := []float64{x, y, z}

	_R := gcmd.Get_floatP("R", nil, nil, nil, nil, nil)
	if _R != nil {
		panic("G2/G3 does not support R moves")
	}

	// determine the plane coordinates and the helical axis
	asPlanar := make([]float64, 2)
	for i, a := range []string{"I", "J"} {
		asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
	}

	axes := []int{X_AXIS, Y_AXIS, Z_AXIS}
	if self.plane == ARC_PLANE_X_Z {
		for i, a := range []string{"I", "K"} {
			asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
		}
		axes = []int{X_AXIS, Z_AXIS, Y_AXIS}
	} else if self.plane == ARC_PLANE_Y_Z {
		for i, a := range []string{"J", "K"} {
			asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
			axes = []int{Y_AXIS, Z_AXIS, X_AXIS}
		}
	}

	if !(asPlanar[0] != 0 || asPlanar[1] != 0) {
		panic("G2/G3 requires IJ, IK or JK parameters")
	}

	self.planArc(currentPos, targetPos, asPlanar, clockwise, gcmd, absolute_extrude, axes[0], axes[1], axes[2])
}

/*
# function planArc() originates from marlin plan_arc()
# https://github.com/MarlinFirmware/Marlin
#
# The arc is approximated by generating many small linear segments.
# The length of each segment is configured in MM_PER_ARC_SEGMENT
# Arcs smaller then this value, will be a Line only
#
# alpha and beta axes are the current plane, helical axis is linear travel
*/
func (self *ArcSupport) planArc(currentPos, targetPos, offset []float64, clockwise bool, gcmd *GCodeCommand, AbsoluteExtrude bool, alpha_axis, beta_axis, helical_axis int) {
	rP := -offset[0]
	rQ := -offset[1]

	centerP := currentPos[alpha_axis] - rP
	centerQ := currentPos[beta_axis] - rQ

	rtAlpha := targetPos[alpha_axis] - centerP
	rtBeta := targetPos[beta_axis] - centerQ

	angularTravel := math.Atan2(rP*rtBeta-rQ*rtAlpha, rP*rtAlpha+rQ*rtBeta)
	if angularTravel < 0 {
		angularTravel += 2 * math.Pi
	}
	if clockwise {
		angularTravel -= 2 * math.Pi
	}

	if angularTravel == 0 &&
		currentPos[alpha_axis] == targetPos[alpha_axis] &&
		currentPos[beta_axis] == targetPos[beta_axis] {
		angularTravel = 2 * math.Pi
	}

	linearTravel := targetPos[helical_axis] - currentPos[helical_axis]
	radius := math.Hypot(rP, rQ)
	flatMM := radius * angularTravel
	var mmOfTravel float64
	if linearTravel != 0 {
		mmOfTravel = math.Hypot(flatMM, linearTravel)
	} else {
		mmOfTravel = math.Abs(flatMM)
	}
	segments := math.Max(1, math.Floor(mmOfTravel/self.mm_per_arc_segment))

	thetaPerSegment := angularTravel / segments
	linearPerSegment := linearTravel / segments

	ePerMove, eBase := 0.0, 0.0

	asE := gcmd.Get_floatP("E", nil, nil, nil, nil, nil)
	asF := gcmd.Get_floatP("F", nil, nil, nil, nil, nil)

	if asE != nil {
		if AbsoluteExtrude {
			eBase = currentPos[E_AXIS]
		}
		ePerMove = (*asE - eBase) / float64(segments)
	}

	for i := 1; i <= int(segments); i++ {
		distHelical := float64(i) * linearPerSegment
		cTheta := float64(i) * thetaPerSegment
		cosTi := math.Cos(cTheta)
		sinTi := math.Sin(cTheta)
		rP = -offset[0]*cosTi + offset[1]*sinTi
		rQ = -offset[0]*sinTi - offset[1]*cosTi

		coord := []float64{
			centerP + rP,
			centerQ + rQ,
			currentPos[helical_axis] + distHelical,
		}

		if i == int(segments) {
			coord = targetPos
		}

		g1Params := make(map[string]string)
		g1Params["X"] = cast.ToString(coord[0])
		g1Params["Y"] = cast.ToString(coord[1])
		g1Params["Z"] = cast.ToString(coord[2])

		if ePerMove != 0 {
			g1Params["E"] = cast.ToString(eBase + ePerMove)
			if AbsoluteExtrude {
				eBase += ePerMove
			}
		}

		if asF != nil {
			g1Params["F"] = cast.ToString(*asF)
		}

		g1_gcmd := self.gcode.Create_gcode_command("G1", "G1", g1Params)
		self.gcode_move.Cmd_G1(g1_gcmd)
	}
}

func Load_config_ArcSupport(config *ConfigWrapper) interface{} {
	return NewArcSupport(config)
}
