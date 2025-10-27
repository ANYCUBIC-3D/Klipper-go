package project

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"k3c/common/logger"
	"k3c/common/utils/object"
	"k3c/project/util"
)

type AdaptiveBedMesh struct {
	printer                    *Printer
	gcode                      *GCodeDispatch
	excludeObject              *ExcludeObject
	printStats                 *PrintStats
	bedMesh                    *BedMesh
	virtualSDPath              string
	arcSegments                int
	meshAreaClearance          float64
	maxProbeHorizontalDistance float64
	maxProbeVerticalDistance   float64
	useRelativeReferenceIndex  bool
	disableSlicerBoundary      bool
	disableExcludeBoundary     bool
	disableGcodeBoundary       bool
	debugMode                  bool
	minimumAxisProbeCounts     int
	bedMeshConfigMeshMin       []float64
	bedMeshConfigMeshMax       []float64
	bedMeshConfigFadeEnd       float64
	bedMeshConfigAlgorithm     string
}

type vec2 struct {
	X float64
	Y float64
}

type moveState struct {
	X float64
	Y float64
	Z float64
}

type moveCommand struct {
	X    float64
	Y    float64
	Z    float64
	E    float64
	F    float64
	HasX bool
	HasY bool
	HasZ bool
	HasE bool
	HasF bool
}

func NewAdaptiveBedMesh(config *ConfigWrapper) *AdaptiveBedMesh {
	self := &AdaptiveBedMesh{}
	self.printer = config.Get_printer()
	self.arcSegments = config.Getint("arc_segments", 80, 1, 0, true)
	self.meshAreaClearance = config.Getfloat("mesh_area_clearance", 5., 0, 0, 0, 0, true)
	self.maxProbeHorizontalDistance = config.Getfloat("max_probe_horizontal_distance", 50., 0, 0, 0, 0, true)
	self.maxProbeVerticalDistance = config.Getfloat("max_probe_vertical_distance", 50., 0, 0, 0, 0, true)
	self.useRelativeReferenceIndex = config.Getboolean("use_relative_reference_index", false, true)

	self.disableSlicerBoundary = config.Getboolean("disable_slicer_min_max_boundary_detection", false, true)
	self.disableExcludeBoundary = config.Getboolean("disable_exclude_object_boundary_detection", false, true)
	self.disableGcodeBoundary = config.Getboolean("disable_gcode_analysis_boundary_detection", false, true)
	self.debugMode = config.Getboolean("debug_mode", false, true)
	self.minimumAxisProbeCounts = 3

	self.gcode = MustLookupGcode(self.printer)

	excludeObj, ok := self.printer.Lookup_object("exclude_object", object.Sentinel{}).(*ExcludeObject)
	if !ok || excludeObj == nil {
		panic(fmt.Sprintf("[adaptive_bed_mesh] requires [exclude_object] to be configured before %s", config.Section))
	}
	self.excludeObject = excludeObj

	printStats, ok := self.printer.Lookup_object("print_stats", object.Sentinel{}).(*PrintStats)
	if !ok || printStats == nil {
		panic(fmt.Sprintf("[adaptive_bed_mesh] requires [print_stats] to be configured before %s", config.Section))
	}
	self.printStats = printStats

	bedMesh, ok := self.printer.Lookup_object("bed_mesh", object.Sentinel{}).(*BedMesh)
	if !ok || bedMesh == nil {
		panic(fmt.Sprintf("[adaptive_bed_mesh] requires [bed_mesh] to be configured before %s", config.Section))
	}
	self.bedMesh = bedMesh

	fileConfig := config.Fileconfig()
	if !fileConfig.Has_section("bed_mesh") {
		panic("[adaptive_bed_mesh] missing required [bed_mesh] section")
	}
	bedMeshSection := config.Getsection("bed_mesh")
	self.bedMeshConfigMeshMin = bedMeshSection.Getfloatlist("mesh_min", nil, ",", 2, true)
	self.bedMeshConfigMeshMax = bedMeshSection.Getfloatlist("mesh_max", nil, ",", 2, true)
	self.bedMeshConfigFadeEnd = bedMeshSection.Getfloat("fade_end", 0., 0, 0, 0, 0, true)
	alg := bedMeshSection.Get("algorithm", "lagrange", true).(string)
	self.bedMeshConfigAlgorithm = strings.ToLower(strings.TrimSpace(alg))

	if !fileConfig.Has_section("virtual_sdcard") {
		panic("[adaptive_bed_mesh] missing required [virtual_sdcard] section")
	}
	virtualSection := config.Getsection("virtual_sdcard")
	pathVal := virtualSection.Get("path", object.Sentinel{}, true)
	sdPath, ok := pathVal.(string)
	if !ok {
		panic("[adaptive_bed_mesh] virtual_sdcard.path must be a string")
	}
	self.virtualSDPath = util.Normpath(util.ExpandUser(sdPath))

	self.gcode.Register_command("ADAPTIVE_BED_MESH_CALIBRATE", self.Cmd_ADAPTIVE_BED_MESH_CALIBRATE, false,
		"Run adaptive bed mesh calibration using detected print area")

	return self
}

func (self *AdaptiveBedMesh) Cmd_ADAPTIVE_BED_MESH_CALIBRATE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	if err := self.cmdAdaptiveBedMeshCalibrate(gcmd); err != nil {
		gcmd.Respond_info(fmt.Sprintf("AdaptiveBedMesh: %v", err), true)
		if !self.debugMode {
			return err
		}
	}
	return nil
}

func (self *AdaptiveBedMesh) cmdAdaptiveBedMeshCalibrate(gcmd *GCodeCommand) error {
	defer func() {
		if r := recover(); r != nil {
			err := fmt.Errorf("panic recovered: %v", r)
			self.logToGcode(gcmd, err.Error())
			if !self.debugMode {
				panic(err)
			}
		}
	}()

	meshMin, meshMax, err := self.detectMeshArea(gcmd)
	if err != nil {
		return err
	}

	self.logToGcode(gcmd, fmt.Sprintf("Selected mesh area: min(%.3f, %.3f) max(%.3f, %.3f)", meshMin.X, meshMin.Y, meshMax.X, meshMax.Y))

	return self.executeBedMeshCalibrate(meshMin, meshMax, gcmd)
}

func (self *AdaptiveBedMesh) detectMeshArea(gcmd *GCodeCommand) (vec2, vec2, error) {
	defaultMin := vec2{X: self.bedMeshConfigMeshMin[0], Y: self.bedMeshConfigMeshMin[1]}
	defaultMax := vec2{X: self.bedMeshConfigMeshMax[0], Y: self.bedMeshConfigMeshMax[1]}

	if !self.disableSlicerBoundary {
		self.logToGcode(gcmd, "Attempting to detect boundary by slicer min max")
		areaStart := strings.TrimSpace(gcmd.Params["AREA_START"])
		areaEnd := strings.TrimSpace(gcmd.Params["AREA_END"])
		if areaStart != "" && areaEnd != "" {
			start, errStart := parseVec2CSV(areaStart)
			end, errEnd := parseVec2CSV(areaEnd)
			if errStart == nil && errEnd == nil {
				self.logToGcode(gcmd, "Use min max boundary detection")
				return start, end, nil
			}
			self.logToGcode(gcmd, fmt.Sprintf("Failed to parse slicer AREA_* parameters: %v %v", errStart, errEnd))
		} else {
			self.logToGcode(gcmd, "Failed to run slicer min max: No information available")
		}
	}

	if !self.disableExcludeBoundary {
		self.logToGcode(gcmd, "Attempting to detect boundary by exclude boundary")
		if self.debugMode {
			self.logToGcode(gcmd, fmt.Sprintf("Exclude objects count: %d", len(self.excludeObject.objects)))
		}
		if len(self.excludeObject.objects) > 0 {
			minPt, maxPt, err := self.generateMeshWithExcludeObject(self.excludeObject.objects)
			if err != nil {
				self.logToGcode(gcmd, fmt.Sprintf("Failed to run exclude object analysis: %v", err))
			} else {
				self.logToGcode(gcmd, "Use exclude object boundary detection")
				return minPt, maxPt, nil
			}
		} else {
			self.logToGcode(gcmd, "Failed to run exclude object analysis: No exclude object information available")
		}
	}

	if !self.disableGcodeBoundary {
		self.logToGcode(gcmd, "Attempting to detect boundary by Gcode analysis")
		gcodePath := strings.TrimSpace(gcmd.Params["GCODE_FILEPATH"])
		minPt, maxPt, err := self.generateMeshWithGcodeAnalysis(gcodePath)
		if err != nil {
			self.logToGcode(gcmd, fmt.Sprintf("Failed to run Gcode analysis: %v", err))
		} else {
			self.logToGcode(gcmd, "Use Gcode analysis boundary detection")
			return minPt, maxPt, nil
		}
	}

	self.logToGcode(gcmd, "Fallback to default bed mesh")
	return defaultMin, defaultMax, nil
}

func (self *AdaptiveBedMesh) generateMeshWithExcludeObject(objects []map[string]interface{}) (vec2, vec2, error) {
	if len(objects) == 0 {
		return vec2{}, vec2{}, fmt.Errorf("no exclude object data available")
	}
	points := make([]vec2, 0, len(objects)*2)
	for _, obj := range objects {
		poly, ok := obj["polygon"].([][]float64)
		if !ok {
			return vec2{}, vec2{}, fmt.Errorf("invalid polygon data in exclude object")
		}
		for _, pt := range poly {
			if len(pt) < 2 {
				continue
			}
			points = append(points, vec2{X: pt[0], Y: pt[1]})
		}
	}
	if len(points) == 0 {
		return vec2{}, vec2{}, fmt.Errorf("no polygon vertices found in exclude object data")
	}
	return getPolygonMinMax(points)
}

func (self *AdaptiveBedMesh) generateMeshWithGcodeAnalysis(gcodePath string) (vec2, vec2, error) {
	resolvedPath := gcodePath
	if resolvedPath == "" {
		if self.printStats.filename == "" {
			return vec2{}, vec2{}, fmt.Errorf("no gcode filepath provided and no active print file")
		}
		resolvedPath = filepath.Join(self.virtualSDPath, self.printStats.filename)
	} else if !filepath.IsAbs(resolvedPath) {
		resolvedPath = filepath.Join(self.virtualSDPath, resolvedPath)
	}

	if _, err := os.Stat(resolvedPath); err != nil {
		return vec2{}, vec2{}, fmt.Errorf("unable to access gcode file: %w", err)
	}

	layers, err := self.getLayerVertices(resolvedPath)
	if err != nil {
		return vec2{}, vec2{}, err
	}

	return self.getLayerMinMaxBeforeFade(layers, self.bedMeshConfigFadeEnd)
}

func (self *AdaptiveBedMesh) logToGcode(gcmd *GCodeCommand, msg string) {
	if gcmd != nil {
		gcmd.Respond_info("AdaptiveBedMesh: "+msg, true)
	} else {
		logger.Debug("AdaptiveBedMesh: " + msg)
	}
}

func parseVec2CSV(input string) (vec2, error) {
	parts := strings.Split(input, ",")
	if len(parts) != 2 {
		return vec2{}, fmt.Errorf("expected two comma separated values")
	}
	var values [2]float64
	for i := range parts {
		val, err := strconv.ParseFloat(strings.TrimSpace(parts[i]), 64)
		if err != nil {
			return vec2{}, err
		}
		values[i] = val
	}
	return vec2{X: values[0], Y: values[1]}, nil
}

func getPolygonMinMax(points []vec2) (vec2, vec2, error) {
	if len(points) == 0 {
		return vec2{}, vec2{}, fmt.Errorf("no points provided")
	}
	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, pt := range points {
		if pt.X < minX {
			minX = pt.X
		}
		if pt.X > maxX {
			maxX = pt.X
		}
		if pt.Y < minY {
			minY = pt.Y
		}
		if pt.Y > maxY {
			maxY = pt.Y
		}
	}
	return vec2{X: minX, Y: minY}, vec2{X: maxX, Y: maxY}, nil
}

func (self *AdaptiveBedMesh) getLayerVertices(filePath string) (map[float64][]vec2, error) {
	file, err := os.Open(filePath)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	scanner.Buffer(make([]byte, 1024), 1024*1024)

	state := moveState{}
	absoluteMove := true
	extrudeLayers := make(map[float64][]vec2)

	for scanner.Scan() {
		line := scanner.Text()
		if idx := strings.Index(line, ";"); idx >= 0 {
			line = line[:idx]
		}
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		tokens := strings.Fields(line)
		if len(tokens) == 0 {
			continue
		}
		cmd := strings.ToUpper(tokens[0])

		switch cmd {
		case "G90":
			absoluteMove = true
			continue
		case "G91":
			absoluteMove = false
			continue
		}

		moves, err := self.decodeMoves(cmd, tokens, state)
		if err != nil {
			return nil, err
		}
		if len(moves) == 0 {
			continue
		}

		for _, mv := range moves {
			newState := applyMove(state, mv, absoluteMove)

			// Ignore pure extrusion moves
			if !(mv.HasX || mv.HasY || mv.HasZ) {
				state = newState
				continue
			}

			if newState.Z == 0 {
				state = newState
				continue
			}

			if mv.HasE && mv.E > 0 {
				extrudeLayers[newState.Z] = append(extrudeLayers[newState.Z], vec2{X: newState.X, Y: newState.Y})
			}

			state = newState
		}
	}

	if err := scanner.Err(); err != nil {
		return nil, err
	}

	for z, vertices := range extrudeLayers {
		if len(vertices) == 0 {
			delete(extrudeLayers, z)
		}
	}

	if len(extrudeLayers) == 0 {
		return nil, fmt.Errorf("no extrude layers detected in gcode")
	}

	return extrudeLayers, nil
}

func (self *AdaptiveBedMesh) decodeMoves(cmd string, tokens []string, state moveState) ([]moveCommand, error) {
	switch cmd {
	case "G0", "G1":
		mv, err := parseLinearMove(tokens)
		if err != nil {
			return nil, err
		}
		return []moveCommand{mv}, nil
	case "G2", "G3":
		return self.parseArcMoves(cmd, tokens, state)
	default:
		return nil, nil
	}
}

func parseLinearMove(tokens []string) (moveCommand, error) {
	var mv moveCommand
	for _, token := range tokens[1:] {
		if len(token) < 2 {
			continue
		}
		axis := strings.ToUpper(token[:1])
		val, err := strconv.ParseFloat(token[1:], 64)
		if err != nil {
			return moveCommand{}, err
		}
		switch axis {
		case "X":
			mv.X = val
			mv.HasX = true
		case "Y":
			mv.Y = val
			mv.HasY = true
		case "Z":
			mv.Z = val
			mv.HasZ = true
		case "E":
			mv.E = val
			mv.HasE = true
		case "F":
			mv.F = val
			mv.HasF = true
		}
	}
	return mv, nil
}

func (self *AdaptiveBedMesh) parseArcMoves(cmd string, tokens []string, state moveState) ([]moveCommand, error) {
	params := make(map[string]float64)
	flags := make(map[string]bool)
	for _, token := range tokens[1:] {
		if len(token) < 2 {
			continue
		}
		axis := strings.ToUpper(token[:1])
		val, err := strconv.ParseFloat(token[1:], 64)
		if err != nil {
			return nil, err
		}
		params[axis] = val
		flags[axis] = true
	}

	start := vec2{X: state.X, Y: state.Y}
	end := start
	if flags["X"] {
		end.X = params["X"]
	}
	if flags["Y"] {
		end.Y = params["Y"]
	}

	center := vec2{X: start.X + params["I"], Y: start.Y + params["J"]}

	radius := math.Hypot(params["I"], params["J"])
	if flags["R"] {
		radius = params["R"]
	}
	if radius == 0 {
		return nil, fmt.Errorf("invalid arc radius")
	}

	startAngle := math.Atan2(start.Y-center.Y, start.X-center.X)
	endAngle := math.Atan2(end.Y-center.Y, end.X-center.X)
	angleDelta := endAngle - startAngle
	if cmd == "G3" {
		if angleDelta < 0 {
			angleDelta += 2 * math.Pi
		}
	} else if cmd == "G2" {
		if angleDelta > 0 {
			angleDelta -= 2 * math.Pi
		}
	}

	segments := self.arcSegments
	if segments <= 0 {
		segments = 1
	}
	angleIncrement := angleDelta / float64(segments)
	moves := make([]moveCommand, 0, segments+1)

	for i := 0; i <= segments; i++ {
		angle := startAngle + float64(i)*angleIncrement
		x := center.X + radius*math.Cos(angle)
		y := center.Y + radius*math.Sin(angle)
		mv := moveCommand{X: x, Y: y, HasX: true, HasY: true}
		if flags["Z"] {
			mv.Z = params["Z"]
			mv.HasZ = true
		}
		if flags["E"] {
			mv.E = params["E"]
			mv.HasE = true
		}
		if flags["F"] {
			mv.F = params["F"]
			mv.HasF = true
		}
		moves = append(moves, mv)
	}
	return moves, nil
}

func applyMove(state moveState, move moveCommand, absolute bool) moveState {
	newState := state
	if move.HasX {
		if absolute {
			newState.X = move.X
		} else {
			newState.X += move.X
		}
	}
	if move.HasY {
		if absolute {
			newState.Y = move.Y
		} else {
			newState.Y += move.Y
		}
	}
	if move.HasZ {
		if absolute {
			newState.Z = move.Z
		} else {
			newState.Z += move.Z
		}
	}
	return newState
}

func (self *AdaptiveBedMesh) getLayerMinMaxBeforeFade(layers map[float64][]vec2, fadeEnd float64) (vec2, vec2, error) {
	fadeLimit := fadeEnd
	if fadeLimit == 0 {
		fadeLimit = math.Inf(1)
	}

	points := make([]vec2, 0, len(layers)*2)
	for height, vertices := range layers {
		if height >= fadeLimit {
			continue
		}
		if len(vertices) == 0 {
			continue
		}
		minPt, maxPt, err := getMoveMinMax(vertices)
		if err != nil {
			return vec2{}, vec2{}, err
		}
		points = append(points, minPt, maxPt)
	}

	if len(points) == 0 {
		return vec2{}, vec2{}, fmt.Errorf("no printable layers within fade window")
	}

	return getPolygonMinMax(points)
}

func getMoveMinMax(vertices []vec2) (vec2, vec2, error) {
	if len(vertices) == 0 {
		return vec2{}, vec2{}, fmt.Errorf("no vertices provided")
	}
	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, v := range vertices {
		if v.X < minX {
			minX = v.X
		}
		if v.X > maxX {
			maxX = v.X
		}
		if v.Y < minY {
			minY = v.Y
		}
		if v.Y > maxY {
			maxY = v.Y
		}
	}
	return vec2{X: minX, Y: minY}, vec2{X: maxX, Y: maxY}, nil
}

func (self *AdaptiveBedMesh) executeBedMeshCalibrate(meshMin, meshMax vec2, gcmd *GCodeCommand) error {
	minWithMargin, maxWithMargin := applyMinMaxMargin(meshMin, meshMax, self.meshAreaClearance)
	limitedMin, limitedMax := self.applyMinMaxLimit(minWithMargin, maxWithMargin)
	if limitedMax.X <= limitedMin.X || limitedMax.Y <= limitedMin.Y {
		return fmt.Errorf("invalid mesh bounds after applying margin and limits")
	}

	xCount, yCount, probePoints, relIdx := self.getProbePoints(limitedMin, limitedMax)
	if len(probePoints) == 0 {
		return fmt.Errorf("unable to generate probe coordinates")
	}
	if relIdx < 0 || relIdx >= len(probePoints) {
		relIdx = len(probePoints) / 2
	}
	zeroRef := probePoints[relIdx]

	cmd := fmt.Sprintf(
		"BED_MESH_CALIBRATE MESH_MIN=%.3f,%.3f MESH_MAX=%.3f,%.3f PROBE_COUNT=%d,%d",
		limitedMin.X, limitedMin.Y, limitedMax.X, limitedMax.Y, xCount, yCount,
	)

	if self.useRelativeReferenceIndex {
		cmd += fmt.Sprintf(" RELATIVE_REFERENCE_INDEX=%d", relIdx)
		self.bedMesh.Zero_ref_pos = nil
	} else {
		self.bedMesh.Zero_ref_pos = []float64{zeroRef.X, zeroRef.Y}
	}

	self.logToGcode(gcmd, cmd)
	self.gcode.Run_script_from_command(cmd)
	return nil
}

func applyMinMaxMargin(minPt, maxPt vec2, margin float64) (vec2, vec2) {
	return vec2{X: minPt.X - margin, Y: minPt.Y - margin}, vec2{X: maxPt.X + margin, Y: maxPt.Y + margin}
}

func (self *AdaptiveBedMesh) applyMinMaxLimit(minPt, maxPt vec2) (vec2, vec2) {
	minBounds := vec2{X: self.bedMeshConfigMeshMin[0], Y: self.bedMeshConfigMeshMin[1]}
	maxBounds := vec2{X: self.bedMeshConfigMeshMax[0], Y: self.bedMeshConfigMeshMax[1]}

	limitedMin := vec2{X: math.Max(minPt.X, minBounds.X), Y: math.Max(minPt.Y, minBounds.Y)}
	limitedMax := vec2{X: math.Min(maxPt.X, maxBounds.X), Y: math.Min(maxPt.Y, maxBounds.Y)}
	return limitedMin, limitedMax
}

func (self *AdaptiveBedMesh) getProbePoints(minPt, maxPt vec2) (int, int, []vec2, int) {
	hDistance := maxPt.X - minPt.X
	vDistance := maxPt.Y - minPt.Y

	hSpan := self.maxProbeHorizontalDistance
	if hSpan <= 0 {
		hSpan = hDistance
	}
	vSpan := self.maxProbeVerticalDistance
	if vSpan <= 0 {
		vSpan = vDistance
	}

	if hSpan <= 0 {
		hSpan = 1
	}
	if vSpan <= 0 {
		vSpan = 1
	}

	xCount := int(math.Ceil(hDistance / hSpan))
	yCount := int(math.Ceil(vDistance / vSpan))
	if xCount < 1 {
		xCount = 1
	}
	if yCount < 1 {
		yCount = 1
	}

	xCount, yCount = self.applyProbePointLimits(xCount, yCount)

	xPoints := linspace(minPt.X, maxPt.X, xCount)
	yPoints := linspace(minPt.Y, maxPt.Y, yCount)

	probePoints := make([]vec2, 0, len(xPoints)*len(yPoints))
	for yIdx, y := range yPoints {
		if isEven(yIdx) {
			for _, x := range xPoints {
				probePoints = append(probePoints, vec2{X: x, Y: y})
			}
		} else {
			for i := len(xPoints) - 1; i >= 0; i-- {
				probePoints = append(probePoints, vec2{X: xPoints[i], Y: y})
			}
		}
	}

	relIdx := int(math.Round(float64(xCount*yCount) / 2.0))
	if relIdx >= len(probePoints) {
		relIdx = len(probePoints) - 1
	}

	return xCount, yCount, probePoints, relIdx
}

func (self *AdaptiveBedMesh) applyProbePointLimits(xCount, yCount int) (int, int) {
	if xCount < self.minimumAxisProbeCounts {
		xCount = self.minimumAxisProbeCounts
	}
	if yCount < self.minimumAxisProbeCounts {
		yCount = self.minimumAxisProbeCounts
	}

	switch self.bedMeshConfigAlgorithm {
	case "lagrange":
		if xCount > 6 {
			xCount = 6
		}
		if yCount > 6 {
			yCount = 6
		}
	case "bicubic":
		minCnt := minInt(xCount, yCount)
		maxCnt := maxInt(xCount, yCount)
		if minCnt < 4 && maxCnt > 6 {
			if xCount < 4 {
				xCount = 4
			}
			if yCount < 4 {
				yCount = 4
			}
		}
	default:
		// leave counts as-is for unknown algorithms
	}

	return xCount, yCount
}

func linspace(start, end float64, count int) []float64 {
	if count <= 1 {
		return []float64{start}
	}
	step := (end - start) / float64(count-1)
	vals := make([]float64, count)
	for i := 0; i < count; i++ {
		vals[i] = start + float64(i)*step
	}
	return vals
}

func isEven(val int) bool {
	return val%2 == 0
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func Load_config_adaptive_bed_mesh(config *ConfigWrapper) interface{} {
	return NewAdaptiveBedMesh(config)
}
