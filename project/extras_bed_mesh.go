package project

import (
	"encoding/json"
	"fmt"
	"k3c/common/lock"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"math"
	"reflect"
	"strconv"
	"strings"
)

const PROFILE_VERSION = 1

var PROFILE_OPTIONS = map[string]reflect.Kind{
	"min_x": reflect.Float64, "max_x": reflect.Float64, "min_y": reflect.Float64, "max_y": reflect.Float64,
	"x_count": reflect.Int, "y_count": reflect.Int, "mesh_x_pps": reflect.Int, "mesh_y_pps": reflect.Int,
	"algo": reflect.String, "tension": reflect.Float64}

type BedMeshError struct {
	msg string
}

func (b *BedMeshError) Error() string {
	return b.msg
}

// PEP 485 isclose()
func Isclose(a float64, b float64, relTol float64, absTol float64) bool {
	return math.Abs(a-b) <= math.Max(relTol*math.Max(math.Abs(a), math.Abs(b)), absTol)
}

// return true if a coordinate is within the region
// specified by min_c and max_c
func Within(coord []float64, minC, maxC []float64, tol float64) bool {
	return (maxC[0]+tol) >= coord[0] && coord[0] >= (minC[0]-tol) &&
		(maxC[1]+tol) >= coord[1] && coord[1] >= (minC[1]-tol)
}

// Constrain value between min and max
func Constrain(val float64, minVal, maxVal float64) float64 {
	return math.Min(maxVal, math.Max(minVal, val))
}

// Linear interpolation between two values
func Lerp(t, v0, v1 float64) float64 {
	return (1-t)*v0 + t*v1
}

// retreive commma separated pair from config
func Parse_config_pair(config *ConfigWrapper, option string, default_ interface{}, minval float64, maxval float64) []int {
	pair := config.Getintlist(option, []interface{}{default_, default_}, ",", 0, true)
	if len(pair) != 2 {
		if len(pair) != 1 {
			panic(fmt.Sprintf("bed_mesh: malformed '%s' value: %s",
				option,
				config.Get(option, object.Sentinel{}, true)))
		}

		// pair = (pair[0], pair[0])
		pair[1] = pair[0]
	}
	if minval != 0 {
		if float64(pair[0]) < minval || float64(pair[1]) < minval {
			panic(fmt.Sprintf("Option '%s' in section bed_mesh must have a minimum of %s",
				option, strconv.FormatFloat(minval, 'f', -1, 64)))
		}
	}
	if maxval != 0 {
		if float64(pair[0]) > maxval || float64(pair[1]) > maxval {
			panic(fmt.Sprintf("Option '%s' in section bed_mesh must have a maximum of %s",
				option, strconv.FormatFloat(maxval, 'f', -1, 64)))
		}
	}
	return pair

}

// retreive commma separated pair from a g-code command
func Parse_gcmd_pair(gcmd *GCodeCommand, name string, minval *float64, maxval *float64) []int {
	var pair []int
	name_arr := strings.Split(gcmd.Get(name, nil, "", nil,
		nil, nil, nil), ",")
	for _, v := range name_arr {
		pair_val, err := strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			panic(fmt.Sprintf("Unable to parse parameter '%s'", name))
		}
		pair = append(pair, pair_val)
	}
	if len(pair) != 2 {
		if len(pair) != 1 {
			panic(fmt.Sprintf("Unable to parse parameter '%s'", name))
		}
		// pair = (pair[0], pair[0])
		pair[1] = pair[0]
	}
	if minval != nil {
		if float64(pair[0]) < *minval || float64(pair[1]) < *minval {
			panic(fmt.Sprintf("Parameter '%s' must have a minimum of %d",
				name, minval))
		}
	}
	if maxval != nil {
		if float64(pair[0]) > *maxval || float64(pair[1]) > *maxval {
			panic(fmt.Sprintf("Parameter '%s' must have a maximum of %d", name, maxval))
		}
	}
	return pair
}

// retreive commma separated coordinate from a g-code command
func Parse_gcmd_coord(gcmd *GCodeCommand, name string) (float64, float64) {
	name_arr := strings.Split(gcmd.Get(name, nil, "", nil, nil, nil, nil), ",")
	v1, err := strconv.ParseFloat(strings.TrimSpace(name_arr[0]), 64)
	v2, err1 := strconv.ParseFloat(strings.TrimSpace(name_arr[1]), 64)
	if err != nil || err1 != nil {
		panic(fmt.Sprintf("Unable to parse parameter '%s'", name))
	}
	return v1, v2

}

func customMin(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	minValue := values[0]
	for _, v := range values[1:] {
		if v < minValue {
			minValue = v
		}
	}
	return minValue
}

func customMax(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	maxValue := values[0]
	for _, v := range values[1:] {
		if v > maxValue {
			maxValue = v
		}
	}
	return maxValue
}

const FADE_DISABLE = 0x7FFFFFFF

type BedMesh struct {
	Printer           *Printer
	Last_position     []float64
	Target_position   []float64
	Bmc               *BedMeshCalibrate
	Z_mesh            *ZMesh
	Toolhead          *Toolhead
	Horizontal_move_z float64
	Fade_start        float64
	Fade_end          float64
	Fade_dist         float64
	Log_fade_complete bool
	Base_fade_target  float64
	Fade_target       float64
	Gcode             *GCodeDispatch
	Splitter          *MoveSplitter
	Pmgr              *ProfileManager
	Save_profile      func(string) error
	Status            map[string]interface{}
	Sl                lock.SpinLock
	move_transform    Itransform
}

func NewBedMesh(config *ConfigWrapper) *BedMesh {
	self := &BedMesh{}
	self.Printer = config.Get_printer()
	self.Printer.Register_event_handler("project:connect", self.Handle_connect)
	self.Last_position = []float64{0., 0., 0., 0.}
	self.Bmc = NewBedMeshCalibrate(config, self)
	self.Toolhead = nil
	self.Z_mesh = nil
	self.Horizontal_move_z = config.Getfloat("horizontal_move_z", 5., 0, 0, 0, 0, true)
	self.Fade_start = config.Getfloat("fade_start", 1., 0, 0, 0, 0, true)
	self.Fade_end = config.Getfloat("fade_end", 0., 0, 0, 0, 0, true)
	self.Fade_dist = self.Fade_end - self.Fade_start
	if self.Fade_dist <= 0. {
		self.Fade_start, self.Fade_end = FADE_DISABLE, FADE_DISABLE
	}
	self.Log_fade_complete = false
	self.Base_fade_target = config.Getfloat("fade_target", 0., 0, 0, 0, 0, true)
	self.Fade_target = 0.
	gcode_obj := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	logger.Error(err)
	//}
	self.Gcode = gcode_obj.(*GCodeDispatch)
	self.Splitter = NewMoveSplitter(config, self.Gcode)
	// setup persistent storage
	self.Pmgr = NewProfileManager(config, self)
	self.Save_profile = self.Pmgr.Save_profile
	// register gcodes
	self.Gcode.Register_command(
		"BED_MESH_OUTPUT", self.Cmd_BED_MESH_OUTPUT,
		false, cmd_BED_MESH_OUTPUT_help)
	self.Gcode.Register_command("BED_MESH_MAP", self.Cmd_BED_MESH_MAP,
		false, cmd_BED_MESH_MAP_help)
	self.Gcode.Register_command(
		"BED_MESH_CLEAR", self.Cmd_BED_MESH_CLEAR,
		false, cmd_BED_MESH_CLEAR_help)
	self.Gcode.Register_command(
		"BED_MESH_OFFSET", self.Cmd_BED_MESH_OFFSET,
		false, cmd_BED_MESH_OFFSET_help)
	// Register transform
	gcode_move := self.Printer.Load_object(config, "gcode_move", object.Sentinel{})
	gcode_move.(*GCodeMove).Set_move_transform(self, false)
	// initialize status dict
	self.Update_status()
	return self
}

func (self *BedMesh) Handle_connect(event []interface{}) error {
	toolhead_obj := self.Printer.Lookup_object("toolhead", object.Sentinel{})
	self.Toolhead = toolhead_obj.(*Toolhead)
	// self.Bmc.Print_generated_points(self.Gcode.Respond_info)
	return nil
}
func (self *BedMesh) Set_mesh(mesh *ZMesh) {
	if mesh != nil && self.Fade_end != FADE_DISABLE {
		self.Log_fade_complete = true
		if self.Base_fade_target == 0.0 {
			self.Fade_target = mesh.Avg_z
		} else {
			self.Fade_target = self.Base_fade_target
			minZ, maxZ := mesh.Get_z_range()
			if !(minZ <= self.Fade_target && self.Fade_target <= maxZ) && self.Fade_target != 0.0 {
				// fade target is non-zero, out of mesh range
				errTarget := self.Fade_target
				self.Z_mesh = nil
				self.Fade_target = 0.0
				panic(fmt.Sprintf("bed_mesh: ERROR, fade_target lies outside of mesh z range\nmin: %.4f, max: %.4f, fade_target: %.4f", minZ, maxZ, errTarget))
			}
		}
		minZ, maxZ := mesh.Get_z_range()
		if self.Fade_dist <= math.Max(math.Abs(minZ), math.Abs(maxZ)) {
			self.Z_mesh = nil
			self.Fade_target = 0.0
			panic(fmt.Sprintf("bed_mesh:  Mesh extends outside of the fade range, please see the fade_start and fade_end options in example-extras.cfg. fade distance: %.2f mesh min: %.4f mesh max: %.4f",
				self.Fade_dist, minZ, maxZ))
		}
	} else {
		self.Fade_target = 0.0
	}
	self.Z_mesh = mesh
	self.Splitter.Initialize(mesh, self.Fade_target)
	// cache the current position before a transform takes place
	gcode_move := self.Printer.Lookup_object("gcode_move", object.Sentinel{})
	gcode_move.(*GCodeMove).Reset_last_position(nil)
	self.Update_status()
}
func (self *BedMesh) Get_z_factor(z_pos float64) float64 {
	if z_pos >= self.Fade_end {
		return 0.
	} else if z_pos >= self.Fade_start {
		return (self.Fade_end - z_pos) / self.Fade_dist
	} else {
		return 1.
	}
}
func (self *BedMesh) Get_position() []float64 {
	// Return last, non-transformed position
	if self.Z_mesh == nil {
		// No mesh calibrated, so send toolhead position
		self.Last_position = append([]float64{}, self.Toolhead.Get_position()...)
		self.Last_position[2] -= self.Fade_target
	} else {
		// return current position minus the current z-adjustment
		x := self.Toolhead.Get_position()[0]
		y := self.Toolhead.Get_position()[1]
		z := self.Toolhead.Get_position()[2]
		e := self.Toolhead.Get_position()[3]
		max_adj := self.Z_mesh.Calc_z(x, y)
		factor := 1.
		z_adj := max_adj - self.Fade_target
		if math.Min(z, z-max_adj) >= self.Fade_end {
			// Fade out is complete, no factor
			factor = 0.
		} else if math.Max(z, z-max_adj) >= self.Fade_start {
			// Likely in the process of fading out adjustment.
			// Because we don't yet know the gcode z position, use
			// algebra to calculate the factor from the toolhead pos
			factor = (self.Fade_end + self.Fade_target - z) /
				(self.Fade_dist - z_adj)
			factor = Constrain(factor, 0., 1.)
		}
		final_z_adj := factor*z_adj + self.Fade_target
		self.Last_position = []float64{x, y, z - final_z_adj, e}
	}
	last_position := make([]float64, len(self.Last_position))
	copy(last_position, self.Last_position)
	return last_position
}
func (self *BedMesh) Move(newpos []float64, speed float64) {
	// self.Sl.Lock()
	// defer self.Sl.UnLock()
	target_position := make([]float64, len(newpos))
	copy(target_position, newpos)
	self.Target_position = target_position
	factor := self.Get_z_factor(newpos[2])
	if self.Z_mesh == nil || factor == 0. {
		// No mesh calibrated, or mesh leveling phased out.
		x := newpos[0]
		y := newpos[1]
		z := newpos[2]
		e := newpos[3]
		if self.Log_fade_complete {
			self.Log_fade_complete = false
			logger.Debugf("bed_mesh fade complete: Current Z: %.4f fade_target: %.4f ", z, self.Fade_target)
		}
		self.Toolhead.Move([]float64{x, y, z + self.Fade_target, e}, speed)
	} else {
		self.Splitter.Build_move(self.Last_position, newpos, factor)
		for !self.Splitter.Traverse_complete {
			split_move := self.Splitter.Split()
			if len(split_move) > 0 {
				self.Toolhead.Move(split_move, speed)
			} else {
				panic("Mesh Leveling: Error splitting move ")
			}
		}

	}
	self.Last_position = append([]float64{}, newpos...)
}
func (self *BedMesh) Get_status(eventtime float64) map[string]interface{} {
	return sys.DeepCopyMap(self.Status)
}

func (self *BedMesh) Update_status() {
	self.Status = map[string]interface{}{
		"profile_name":  "",
		"mesh_min":      []float64{0., 0.},
		"mesh_max":      []float64{0., 0.},
		"probed_matrix": [][]float64{},
		"mesh_matrix":   [][]float64{},
		"profiles":      self.Pmgr.Get_profiles(),
	}
	if self.Z_mesh != nil {
		params := self.Z_mesh.Get_mesh_params()
		mesh_min := []float64{params["min_x"].(float64), params["min_y"].(float64)}
		mesh_max := []float64{params["max_x"].(float64), params["max_y"].(float64)}
		probed_matrix := self.Z_mesh.Get_probed_matrix()
		mesh_matrix := self.Z_mesh.Get_mesh_matrix()
		self.Status["profile_name"] = self.Pmgr.Get_current_profile()
		self.Status["mesh_min"] = mesh_min
		self.Status["mesh_max"] = mesh_max
		self.Status["probed_matrix"] = probed_matrix
		self.Status["mesh_matrix"] = mesh_matrix
	}
}
func (self *BedMesh) Get_mesh() *ZMesh {
	return self.Z_mesh
}

const cmd_BED_MESH_OUTPUT_help = "Retrieve interpolated grid of probed z-points"

func (self *BedMesh) Cmd_BED_MESH_OUTPUT(gcmd *GCodeCommand) {
	if gcmd.Get_int("PGP", 0, nil, nil) > 0 {
		// Print Generated Points instead of mesh
		self.Bmc.Print_generated_points(gcmd.Respond_info)
	} else if self.Z_mesh == nil {
		gcmd.Respond_info("Bed has not been probed", true)
	} else {
		self.Z_mesh.Print_probed_matrix(gcmd.Respond_info)
		horizontal_move_z := int(self.Horizontal_move_z)
		self.Z_mesh.Print_mesh(logger.Debug, &horizontal_move_z)
	}
}

const cmd_BED_MESH_MAP_help = "Serialize mesh and output to terminal"

func (self *BedMesh) Cmd_BED_MESH_MAP(gcmd *GCodeCommand) error {
	if self.Z_mesh != nil {
		params := self.Z_mesh.Get_mesh_params()
		outdict := map[string]interface{}{
			"mesh_min":    []float64{params["min_x"].(float64), params["min_y"].(float64)},
			"mesh_max":    []float64{params["max_x"].(float64), params["max_y"].(float64)},
			"z_positions": self.Z_mesh.Get_probed_matrix()}
		jsonStr, _ := json.Marshal(outdict)
		gcmd.Respond_raw("mesh_map_output " + string(jsonStr))
	} else {
		gcmd.Respond_info("Bed has not been probed", true)
	}
	return nil
}

const cmd_BED_MESH_CLEAR_help = "Clear the Mesh so no z-adjustment is made"

func (self *BedMesh) Cmd_BED_MESH_CLEAR(gcmd interface{}) error {
	self.Set_mesh(nil)
	return nil
}

const cmd_BED_MESH_OFFSET_help = "Add X/Y offsets to the mesh lookup"

func (self *BedMesh) Cmd_BED_MESH_OFFSET(gcmd *GCodeCommand) error {
	if self.Z_mesh != nil {
		var offsets []float64
		for i, Axis := range []string{"x", "y"} {
			offsets[i] = gcmd.Get_float(Axis, nil, nil, nil, nil, nil)
		}
		self.Z_mesh.Set_mesh_offsets(offsets)
		gcode_move_obj := self.Printer.Lookup_object("gcode_move", object.Sentinel{})
		//if err != nil {
		//	logger.Error(err)
		//}
		gcode_move := gcode_move_obj.(*GCodeMove)
		gcode_move.Reset_last_position(nil)
	} else {
		gcmd.Respond_info("No mesh loaded to offset", true)
	}
	return nil
}

var ALGOS = []string{"lagrange", "bicubic"}

type BedMeshCalibrate struct {
	Printer                  *Printer
	Orig_config              map[string]interface{}
	Radius                   *float64
	Origin                   []float64
	Mesh_min                 []float64
	Mesh_max                 []float64
	Relative_reference_index *int
	Faulty_regions           [][][]float64
	Substituted_indices      [][][]float64
	Bedmesh                  *BedMesh
	Mesh_config              map[string]interface{}
	Orig_points              [][]float64
	Points                   [][]float64
	Profile_name             string
	Probe_helper             *ProbePointsHelper
	Gcode                    *GCodeDispatch
	config                   *ConfigWrapper
	adaptive_margin          float64
	min_bedmesh_area_size    []float64
}

func NewBedMeshCalibrate(config *ConfigWrapper, bedmesh *BedMesh) *BedMeshCalibrate {
	self := &BedMeshCalibrate{}
	self.config = config
	self.Printer = config.Get_printer()
	self.Orig_config = map[string]interface{}{}
	self.Orig_config["radius"] = nil
	self.Orig_config["origin"] = nil
	self.Radius = nil
	self.Origin = nil
	self.Mesh_min = []float64{0., 0.}
	self.Mesh_max = []float64{0., 0.}
	self.Relative_reference_index = get_relative_reference_index(config, self)
	self.Faulty_regions = [][][]float64{}
	self.Substituted_indices = [][][]float64{}
	self.Orig_config["rri"] = self.Relative_reference_index
	self.adaptive_margin = config.Getfloat("adaptive_margin", 0.0, 0., 0., 0., 0., true)
	self.Bedmesh = bedmesh
	self.min_bedmesh_area_size = config.Getfloatlist("min_bedmesh_area_size", []float64{100.0, 100.0}, ",", 2, true)
	self.Mesh_config = map[string]interface{}{}
	self.Init_mesh_config(config)
	self.Generate_points()
	self.Profile_name = ""
	self.Orig_points = self.Points
	self.Probe_helper = NewProbePointsHelper(
		config, self.Probe_finalize, self.Get_adjusted_points())
	self.Probe_helper.Minimum_points(3)
	self.Probe_helper.Use_xy_offsets(true)
	gcode_obj := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	logger.Error(err)
	//}
	self.Gcode = gcode_obj.(*GCodeDispatch)
	self.Gcode.Register_command(
		"BED_MESH_CALIBRATE", self.Cmd_BED_MESH_CALIBRATE,
		false, cmd_BED_MESH_CALIBRATE_help)
	/* self.Gcode.Register_command(
	"BED_MESH_CALIBRATE_BY_LEIVQ", self.Cmd_BED_MESH_CALIBRATE_BY_LEIVQ,
	false, cmd_BED_MESH_CALIBRATE_help) */
	return self
}

func get_relative_reference_index(config *ConfigWrapper, self *BedMeshCalibrate) *int {
	if !config.Fileconfig().Has_option(config.Section, "relative_reference_index") {
		return nil
	} else {
		v := config.Fileconfig().Getint(config.Section, "relative_reference_index")
		relative_reference_index := v.(int)
		self.Relative_reference_index = &relative_reference_index
		return &relative_reference_index
	}
}
func (self *BedMeshCalibrate) Generate_points() {
	x_cnt := self.Mesh_config["x_count"]
	y_cnt := self.Mesh_config["y_count"]
	min_x := self.Mesh_min[0]
	min_y := self.Mesh_min[1]
	max_x := self.Mesh_max[0]
	max_y := self.Mesh_max[1]
	x_dist := (max_x - min_x) / (float64(x_cnt.(int)) - 1)
	y_dist := (max_y - min_y) / (float64(y_cnt.(int)) - 1)
	// floor distances down to next hundredth
	x_dist = math.Floor(x_dist*100) / 100
	y_dist = math.Floor(y_dist*100) / 100
	if x_dist < 1. || y_dist < 1. {
		panic("bed_mesh: min/max points too close together")
	}
	if *self.Radius != 0 {
		// round bed, min/max needs to be recalculated
		y_dist = x_dist
		new_r := float64(x_cnt.(int)) / 2.0 * x_dist
		min_y = -new_r
		min_x = min_y
		max_y = new_r
		max_x = max_y
	} else {
		// rectangular bed, only re-calc max_x
		max_x = min_x + x_dist*(float64(x_cnt.(int))-1)
	}
	pos_y := min_y
	points := [][]float64{}
	pos_x := 0.0
	for i := 0; i < y_cnt.(int); i++ {
		for j := 0; j < x_cnt.(int); j++ {
			if i%2 == 0 {
				// move in positive directon
				pos_x = min_x + float64(j)*x_dist
			} else {
				// move in negative direction
				pos_x = max_x - float64(j)*x_dist
			}
			if *self.Radius == 0 {
				// rectangular bed, append
				points = append(points, []float64{pos_x, pos_y})
			} else {
				// round bed, check distance from origin
				dist_from_origin := math.Sqrt(pos_x*pos_x + pos_y*pos_y)
				if dist_from_origin <= *self.Radius {
					points = append(points,
						[]float64{self.Origin[0] + pos_x, self.Origin[1] + pos_y})
				}
			}
		}
		pos_y += y_dist
	}
	self.Points = points
	if len(self.Faulty_regions) == 0 {
		return
	}
	// Check to see if any points fall within faulty regions
	last_y := self.Points[0][1]
	is_reversed := false
	for i, coord := range self.Points {
		if !Isclose(coord[1], last_y, 1e-09, 0.0) {
			is_reversed = !is_reversed
		}
		last_y = coord[1]
		adj_coords := [][]float64{}
		for _, val := range self.Faulty_regions {
			min_c := val[0]
			max_c := val[1]
			if Within(coord, min_c, max_c, .00001) {
				// Point lies within a faulty region
				adj_coords = [][]float64{
					{min_c[0], coord[1]}, {coord[0], min_c[1]},
					{coord[0], max_c[1]}, {max_c[0], coord[1]}}
				if is_reversed {
					// Swap first and last points for zig-zag pattern
					first := adj_coords[0]
					adj_coords[0] = adj_coords[len(adj_coords)-1]
					adj_coords[len(adj_coords)-1] = first
				}
				break
			}

		}
		if len(adj_coords) == 0 {
			// coord is not located within a faulty region
			continue
		}
		valid_coords := [][]float64{}
		for _, ac := range adj_coords {
			// make sure that coordinates are within the mesh boundary
			if self.Radius == nil {
				if Within(ac, []float64{min_x, min_y}, []float64{max_x, max_y}, .000001) {
					valid_coords = append(valid_coords, ac)
				}
			} else {
				dist_from_origin := math.Sqrt(ac[0]*ac[0] + ac[1]*ac[1])
				if dist_from_origin <= *self.Radius {
					valid_coords = append(valid_coords, ac)
				}
			}
		}
		if len(valid_coords) == 0 {
			panic(fmt.Sprintf("bed_mesh: Unable to generate coordinates"+
				" for faulty region at index: %d", i))
		}
		self.Substituted_indices[i] = valid_coords
	}
	return
}
func (self *BedMeshCalibrate) Print_generated_points(print_func func(msg string, log bool)) {
	xOffset, yOffset := 0., 0.
	probe_obj := self.Printer.Lookup_object("probe", nil)
	if probe_obj != nil {
		xOffset, yOffset, _ = probe_obj.(*PrinterProbe).Get_offsets()
	}
	print_func("bed_mesh: generated points\nIndex| Tool Adjusted | Probe", true)
	for i, v := range self.Points {
		adjPt := fmt.Sprintf("(%.1f, %.1f)", v[0]-xOffset, v[1]-yOffset)
		meshPt := fmt.Sprintf("(%.1f, %.1f)", v[0], v[1])
		print_func(fmt.Sprintf("%-4d| %-16s | %s", i, adjPt, meshPt), true)
	}
	if self.Relative_reference_index != nil {
		rri := *self.Relative_reference_index
		pt := self.Points[rri]
		print_func(fmt.Sprintf("bed_mesh: relative_reference_index %d is (%.2f, %.2f)", rri, pt[0], pt[1]), true)
	}
	if len(self.Substituted_indices) != 0 {
		print_func("bed_mesh: faulty region points", true)
		for k, v := range self.Substituted_indices {
			pt := self.Points[k]
			print_func(fmt.Sprintf("%d (%.2f, %.2f), substituted points: %v", k, pt[0], pt[1], v), true)
		}
	}
}
func (self *BedMeshCalibrate) Init_mesh_config(config *ConfigWrapper) {
	mesh_cfg := self.Mesh_config
	orig_cfg := self.Orig_config
	radius := config.Getfloat("mesh_radius", 0, 0, 0, 0., 0, true)
	self.Radius = &radius
	min_x, min_y, max_x, max_y := 0., 0., 0., 0.
	x_cnt, y_cnt := 0, 0
	if *self.Radius != 0 {
		origin := config.Getfloatlist("mesh_radius", 0, ",", 2, true)
		self.Origin = origin
		x_cnt = config.Getint("round_probe_count", 5, 3, 0, true)
		y_cnt = x_cnt
		// round beds must have an odd number of points along each axis
		if x_cnt&1 == 0 {
			panic("bed_mesh: probe_count must be odd for round beds")
		}
		// radius may have precision to .1mm
		radius := math.Floor(*self.Radius*10) / 10
		self.Radius = &radius
		orig_cfg["radius"] = radius
		orig_cfg["origin"] = self.Origin
		min_x = -*self.Radius
		min_y = -*self.Radius
		max_x = *self.Radius
		max_y = *self.Radius
	} else {
		// rectangular
		val1, val2 := 3., 0.
		pps := Parse_config_pair(config, "probe_count", 3, val1, val2)
		x_cnt = pps[0]
		y_cnt = pps[1]
		float_arr1 := config.Getfloatlist("mesh_min", nil, ",", 2, true)
		min_x = float_arr1[0]
		min_y = float_arr1[1]
		float_arr2 := config.Getfloatlist("mesh_max", nil, ",", 2, true)
		max_x = float_arr2[0]
		max_y = float_arr2[1]
		if max_x <= min_x || max_y <= min_y {
			panic("bed_mesh: invalid min/max points")
		}
	}
	orig_cfg["x_count"] = x_cnt
	mesh_cfg["x_count"] = x_cnt
	orig_cfg["y_count"] = y_cnt
	mesh_cfg["y_count"] = y_cnt
	orig_cfg["mesh_min"] = []float64{min_x, min_y}
	self.Mesh_min = []float64{min_x, min_y}
	orig_cfg["mesh_max"] = []float64{max_x, max_y}
	self.Mesh_max = []float64{max_x, max_y}

	pps := Parse_config_pair(config, "mesh_pps", 2, 0., 0.)
	orig_cfg["mesh_x_pps"] = pps[0]
	mesh_cfg["mesh_x_pps"] = pps[0]
	orig_cfg["mesh_y_pps"] = pps[1]
	mesh_cfg["mesh_y_pps"] = pps[1]
	orig_cfg["algo"] = strings.ToLower(strings.TrimSpace(config.Get("algorithm", "lagrange", true).(string)))
	mesh_cfg["algo"] = orig_cfg["algo"]
	orig_cfg["tension"] = config.Getfloat("bicubic_tension", .2, 0., 2., 0, 0, true)
	mesh_cfg["tension"] = orig_cfg["tension"]
	for i := 1; i < 100; i++ {
		start := config.Getfloatlist(fmt.Sprintf("faulty_region_%d_min", i), nil, ",", 2, true)
		if len(start) == 0 {
			break
		}
		end := config.Getfloatlist(fmt.Sprintf("faulty_region_%d_max", i), nil, ",", 2, true)
		// Validate the corners.  If necessary reorganize them.
		// c1 = min point, c3 = max point
		//  c4 ---- c3
		//  |        |
		//  c1 ---- c2
		var c1 []float64
		length := math.Min(float64(len(start)), float64(len(end)))
		for j := 0; j < int(length); j++ {
			c1 = append(c1, math.Min(start[j], end[j]))
		}
		var c3 []float64
		for j := 0; j < int(length); j++ {
			c3 = append(c3, math.Max(start[j], end[j]))
		}
		c2 := []float64{c1[0], c3[1]}
		c4 := []float64{c3[0], c1[1]}
		// Check for overlapping regions
		for j, item := range self.Faulty_regions {
			prev_c1 := item[0]
			prev_c3 := item[1]
			prev_c2 := []float64{prev_c1[0], prev_c3[1]}
			prev_c4 := []float64{prev_c3[0], prev_c1[1]}
			//  Validate that no existing corner is within the new region
			for _, coord := range [][]float64{prev_c1, prev_c2, prev_c3, prev_c4} {
				if Within(coord, c1, c3, 0.0) {
					panic(
						fmt.Sprintf("bed_mesh: Existing faulty_region_%d %v overlaps "+
							"added faulty_region_%d %v",
							j+1, [][]float64{prev_c1, prev_c3},
							i, [][]float64{c1, c3}))
				}
			}
			// Validate that no new corner is within an existing region
			for _, coord := range [][]float64{c1, c2, c3, c4} {
				if Within(coord, prev_c1, prev_c3, 0.0) {
					panic(
						fmt.Sprintf("bed_mesh: Added faulty_region_%d %v overlaps "+
							"existing faulty_region_%d %v",
							i, [][]float64{c1, c3},
							j+1, [][]float64{prev_c1, prev_c3}))
				}
			}
		}
		self.Faulty_regions = append(self.Faulty_regions, [][]float64{c1, c3})
	}
	self.Verify_algorithm()
}

func (self *BedMeshCalibrate) Verify_algorithm() {
	params := self.Mesh_config
	x_pps := params["mesh_x_pps"]
	y_pps := params["mesh_y_pps"]
	notIn := true
	for i := 0; i < len(ALGOS); i++ {
		if ALGOS[i] == params["algo"].(string) {
			notIn = false
			break
		}
	}
	if notIn {
		panic(fmt.Sprintf("bed_mesh: Unknown algorithm <%s>", self.Mesh_config["algo"]))
	}
	// Check the algorithm against the current configuration
	max_probe_cnt := math.Max(float64(params["x_count"].(int)), float64(params["y_count"].(int)))
	min_probe_cnt := math.Min(float64(params["x_count"].(int)), float64(params["y_count"].(int)))
	if math.Max(float64(x_pps.(int)), float64(y_pps.(int))) == 0 {
		// Interpolation disabled
		self.Mesh_config["algo"] = "direct"
	} else if params["algo"] == "lagrange" && max_probe_cnt > 6 {
		// Lagrange interpolation tends to oscillate when using more
		// than 6 samples
		panic(fmt.Sprintf("bed_mesh: cannot exceed a probe_count of 6 when using "+
			"lagrange interpolation. Configured Probe Count: %d, %d",
			self.Mesh_config["x_count"], self.Mesh_config["y_count"]))
	} else if params["algo"].(string) == "bicubic" && min_probe_cnt < 4 {
		if max_probe_cnt > 6 {
			panic(
				fmt.Sprintf("bed_mesh: invalid probe_count option when using bicubic "+
					"interpolation.  Combination of 3 points on one axis with "+
					"more than 6 on another is not permitted. "+
					"Configured Probe Count: %d, %d",
					self.Mesh_config["x_count"], self.Mesh_config["y_count"]))
		} else {
			logger.Debugf(
				"bed_mesh: bicubic interpolation with a probe_count of "+
					"less than 4 points detected.  Forcing lagrange "+
					"interpolation. Configured Probe Count: %d, %d",
				self.Mesh_config["x_count"], self.Mesh_config["y_count"])
			params["algo"] = "lagrange"
		}
	}
}
func (self *BedMeshCalibrate) set_adaptive_mesh(gcmd *GCodeCommand) bool {
	if gcmd.Get_int("ADAPTIVE", 0, nil, nil) == 0 {
		return false
	}
	exclude_objects := self.Printer.Lookup_object("exclude_object", nil).(*ExcludeObject)
	if exclude_objects == nil {
		gcmd.Respond_info("Exclude objects not enabled. Using full mesh...", true)
		return false
	}
	objects := exclude_objects.Get_status(0)["objects"]
	if objects == nil {
		return false
	}

	margin := gcmd.Get_float("ADAPTIVE_MARGIN", self.adaptive_margin, nil, nil, nil, nil)
	//# List all exclude_object points by axis and iterate over
	//# all polygon points, and pick the min and max or each axis
	var list_of_xs, list_of_ys []float64
	gcmd.Respond_info(fmt.Sprintf("Found %d objects", len(objects.([]map[string]interface{}))), true)
	for _, obj := range objects.([]map[string]interface{}) {
		for _, point := range obj["polygon"].([][]float64) {
			list_of_xs = append(list_of_xs, point[0])
			list_of_ys = append(list_of_ys, point[1])
		}
	}
	x_min := customMin(list_of_xs)
	y_min := customMin(list_of_ys)
	x_max := customMax(list_of_xs)
	y_max := customMax(list_of_ys)

	mesh_min := [2]float64{math.Min(x_min, x_max), math.Min(y_min, y_max)}
	mesh_max := [2]float64{math.Max(x_min, x_max), math.Max(y_min, y_max)}

	adjusted_mesh_min := []float64{mesh_min[0] - margin, mesh_min[1] - margin}
	adjusted_mesh_max := []float64{mesh_max[0] + margin, mesh_max[1] + margin}
	logger.Debug(fmt.Sprintf("Adapted mesh bounds: (%f,%f)", adjusted_mesh_min, adjusted_mesh_max))

	adjusted_mesh_min[0] = math.Max(adjusted_mesh_min[0], self.Orig_config["mesh_min"].([]float64)[0])
	adjusted_mesh_min[1] = math.Max(adjusted_mesh_min[1], self.Orig_config["mesh_min"].([]float64)[1])
	adjusted_mesh_max[0] = math.Min(adjusted_mesh_max[0], self.Orig_config["mesh_max"].([]float64)[0])
	adjusted_mesh_max[1] = math.Min(adjusted_mesh_max[1], self.Orig_config["mesh_max"].([]float64)[1])
	var adjusted_mesh_size = []float64{adjusted_mesh_max[0] - adjusted_mesh_min[0],
		adjusted_mesh_max[1] - adjusted_mesh_min[1]}

	var ratio = []float64{adjusted_mesh_size[0] / (self.Orig_config["mesh_max"].([]float64)[0] - self.Orig_config["mesh_min"].([]float64)[0]),
		adjusted_mesh_size[1] / (self.Orig_config["mesh_max"].([]float64)[1] - self.Orig_config["mesh_min"].([]float64)[1])}
	logger.Debug(fmt.Sprintf("Original mesh bounds: (%v,%v)", self.Orig_config["mesh_min"], self.Orig_config["mesh_max"]))
	logger.Debug(fmt.Sprintf("Original probe count: (%v,%v)", self.Mesh_config["x_count"], self.Mesh_config["y_count"]))
	logger.Debug(fmt.Sprintf("Adapted mesh bounds: (%f,%f)", adjusted_mesh_min, adjusted_mesh_max))
	logger.Debug(fmt.Sprintf("Ratio: (%f, %f)", ratio[0], ratio[1]))

	new_x_probe_count := int(
		math.Ceil(float64(self.Mesh_config["x_count"].(int)) * ratio[0]))
	new_y_probe_count := int(
		math.Ceil(float64(self.Mesh_config["y_count"].(int)) * ratio[1]))
	//# There is one case, where we may have to adjust the probe counts:
	//# axis0 < 4 and axis1 > 6 (see _verify_algorithm).
	min_num_of_probes := 3
	if maths.Max(new_x_probe_count, new_y_probe_count) > 6 &&
		maths.Min(new_x_probe_count, new_y_probe_count) < 4 {
		min_num_of_probes = 4
	}
	new_x_probe_count = maths.Max(min_num_of_probes, new_x_probe_count)
	new_y_probe_count = maths.Max(min_num_of_probes, new_y_probe_count)

	gcmd.Respond_info(fmt.Sprintf("Adapted probe count: (%v,%v)",
		new_x_probe_count, new_y_probe_count), true)
	if *self.Radius != 0 {
		adaptedRadius := math.Sqrt(math.Pow(adjusted_mesh_max[0]-adjusted_mesh_min[0], 2)+math.Pow(adjusted_mesh_max[1]-adjusted_mesh_min[1], 2)) / 2
		adaptedOrigin := []float64{
			adjusted_mesh_min[0] + (adjusted_mesh_max[0]-adjusted_mesh_min[0])/2,
			adjusted_mesh_min[1] + (adjusted_mesh_max[1]-adjusted_mesh_min[1])/2,
		}
		if adaptedRadius+math.Sqrt(adaptedOrigin[0]*adaptedOrigin[0]+adaptedOrigin[1]*adaptedOrigin[1]) < *self.Radius {
			self.Radius = &adaptedRadius
			self.Origin = adaptedOrigin
			self.Mesh_min = []float64{-adaptedRadius, -adaptedRadius}
			self.Mesh_max = []float64{adaptedRadius, adaptedRadius}
			new_probe_count := maths.Max(new_x_probe_count, new_y_probe_count)
			new_probe_count += 1 - (new_probe_count % 2)
			self.Mesh_config["x_count"] = new_probe_count
			self.Mesh_config["y_count"] = new_probe_count
		}
	} else {
		if adjusted_mesh_size[0] < self.min_bedmesh_area_size[0] &&
			adjusted_mesh_size[1] < self.min_bedmesh_area_size[1] {

			adjusted_mesh_size = self.min_bedmesh_area_size
			x_center := (adjusted_mesh_min[0] + adjusted_mesh_max[0]) / 2
			y_center := (adjusted_mesh_min[1] + adjusted_mesh_max[1]) / 2

			adjusted_mesh_min = []float64{x_center - self.min_bedmesh_area_size[0]/2, y_center - self.min_bedmesh_area_size[1]/2}
			adjusted_mesh_max = []float64{x_center + self.min_bedmesh_area_size[0]/2, y_center + self.min_bedmesh_area_size[1]/2}

			adjusted_mesh_min[0] = math.Max(adjusted_mesh_min[0], self.Orig_config["mesh_min"].([]float64)[0])
			adjusted_mesh_min[1] = math.Max(adjusted_mesh_min[1], self.Orig_config["mesh_min"].([]float64)[1])
			adjusted_mesh_max[0] = math.Min(adjusted_mesh_max[0], self.Orig_config["mesh_max"].([]float64)[0])
			adjusted_mesh_max[1] = math.Min(adjusted_mesh_max[1], self.Orig_config["mesh_max"].([]float64)[1])
			logger.Debug(fmt.Sprintf("Adaptive mesh bed area too small, readjust area: (%f,%f)", adjusted_mesh_min, adjusted_mesh_max), true)

		}

		self.Mesh_min = adjusted_mesh_min
		self.Mesh_max = adjusted_mesh_max
		self.Mesh_config["x_count"] = new_x_probe_count
		self.Mesh_config["y_count"] = new_y_probe_count
	}

	if Isclose(ratio[0], 1.0, 1e-4, 1e-9) &&
		Isclose(ratio[1], 1.0, 1e-4, 1e-9) {

		self.Profile_name = "default"
	} else {
		self.Profile_name = "adaptive"
	}

	return true
}

func (self *BedMeshCalibrate) Update_config(gcmd *GCodeCommand) {
	// reset default configuration
	if self.Orig_config["radius"] != nil {
		radius := self.Orig_config["radius"].(float64)
		self.Radius = &radius
	}
	if self.Orig_config["origin"] != nil {
		origin := self.Orig_config["origin"].([]float64)
		self.Origin = origin
	}
	// relative_reference_index := self.Orig_config["rri"].(int)
	self.Relative_reference_index = self.Orig_config["rri"].(*int)
	self.Mesh_min = self.Orig_config["mesh_min"].([]float64)
	self.Mesh_max = self.Orig_config["mesh_max"].([]float64)
	for key, _ := range self.Mesh_config {
		self.Mesh_config[key] = self.Orig_config[key]
	}
	params := gcmd.Get_command_parameters()
	need_cfg_update := false

	for key, _ := range params {
		if key == "RELATIVE_REFERENCE_INDEX" {
			if gcmd.Params["RELATIVE_REFERENCE_INDEX"] == "" {
				self.Relative_reference_index = nil
			} else {
				var val, _ = strconv.ParseInt(gcmd.Params["RELATIVE_REFERENCE_INDEX"], 10, 32)
				relative_reference_index := int(val)
				self.Relative_reference_index = &relative_reference_index

			}
			if *self.Relative_reference_index < 0 {
				self.Relative_reference_index = nil
			}
			need_cfg_update = true
			break
		}
	}

	if self.Radius != nil {
		for key, _ := range params {
			if key == "MESH_RADIUS" {
				radius := gcmd.Get_float("MESH_RADIUS", nil, nil, nil, nil, nil)
				self.Radius = &radius
				radius = math.Floor(float64(*self.Radius*10)) / 10
				self.Radius = &radius
				self.Mesh_min = []float64{-*self.Radius, -*self.Radius}
				self.Mesh_max = []float64{*self.Radius, *self.Radius}
				need_cfg_update = true
				break
			}
		}

		for key, _ := range params {
			if key == "MESH_ORIGIN" {
				v1, v2 := Parse_gcmd_coord(gcmd, "MESH_ORIGIN")
				self.Origin = []float64{v1, v2}
				need_cfg_update = true
				break
			}
		}

		for key, _ := range params {
			if key == "ROUND_PROBE_COUNT" {
				minval := 3.
				minval_int := int(minval)
				cnt := gcmd.Get_int("ROUND_PROBE_COUNT", nil, &minval_int, nil)
				self.Mesh_config["x_count"] = cnt
				self.Mesh_config["y_count"] = cnt
				need_cfg_update = true
				break
			}
		}

	} else {
		for key, _ := range params {
			if key == "MESH_MIN" {
				v1, v2 := Parse_gcmd_coord(gcmd, "MESH_MIN")

				self.Mesh_min = []float64{v1, v2}
				need_cfg_update = true
				break
			}
		}

		for key, _ := range params {
			if key == "MESH_MAX" {
				v1, v2 := Parse_gcmd_coord(gcmd, "MESH_MAX")
				self.Mesh_max = []float64{v1, v2}
				need_cfg_update = true
				break
			}
		}

		for key, _ := range params {
			if key == "PROBE_COUNT" {
				minval := 3.
				arr := Parse_gcmd_pair(gcmd, "PROBE_COUNT", &minval, nil)
				self.Mesh_config["x_count"] = arr[0]
				self.Mesh_config["y_count"] = arr[1]
				need_cfg_update = true
				break
			}
		}
	}
	for key, _ := range params {
		if key == "ALGORITHM" {
			self.Mesh_config["algo"] = strings.ToLower(strings.TrimSpace(gcmd.Get("ALGORITHM", nil, "", nil, nil, nil, nil)))
			need_cfg_update = true
			break
		}
	}

	need_mesh_update := self.set_adaptive_mesh(gcmd)
	if need_cfg_update || need_mesh_update {
		self.Verify_algorithm()
		self.Generate_points()
		// gcmd.Respond_info("Generating new points...", true)
		// self.Print_generated_points(gcmd.Respond_info)
		pts := self.Get_adjusted_points()
		self.Probe_helper.Update_probe_points(pts, 3)
		var mesh_config_str_arr []string
		for key, item := range self.Mesh_config {
			mesh_config_str_arr = append(mesh_config_str_arr,
				fmt.Sprintf("%s: %v", key, item))
		}
		msg := strings.Join(mesh_config_str_arr, "\n")
		logger.Debugf("Updated Mesh Configuration:" + msg)
	} else {
		self.Points = self.Orig_points
		pts := self.Get_adjusted_points()
		self.Probe_helper.Update_probe_points(pts, 3)
	}
}

func (self *BedMeshCalibrate) Get_adjusted_points() [][]float64 {
	if len(self.Substituted_indices) == 0 {
		return self.Points
	}
	adj_pts := [][]float64{}
	last_index := 0
	for i, pts := range self.Substituted_indices {
		adj_pts = append(adj_pts, self.Points[last_index:i]...)
		adj_pts = append(adj_pts, pts...)
		// Add one to the last index to skip the point
		// we are replacing
		last_index = i + 1
	}
	adj_pts = append(adj_pts, self.Points[last_index:]...)
	return adj_pts
}

const cmd_BED_MESH_CALIBRATE_help = "Perform Mesh Bed Leveling"

func (self *BedMeshCalibrate) Cmd_BED_MESH_CALIBRATE(gcmd interface{}) error {
	self.Profile_name = gcmd.(*GCodeCommand).Get("PROFILE", "default", "", nil, nil, nil, nil)
	if strings.TrimSpace(self.Profile_name) == "" {
		panic("Value for parameter 'PROFILE' must be specified")
	}
	self.Bedmesh.Set_mesh(nil)
	self.Update_config(gcmd.(*GCodeCommand))
	self.Probe_helper.Start_probe_callback(gcmd.(*GCodeCommand))
	return nil
}

func (self *BedMeshCalibrate) Probe_finalize(offsets []float64, positions [][]float64) string {
	x_offset := offsets[0]
	y_offset := offsets[1]
	z_offset := offsets[2]
	positions_back := [][]float64{}
	for _, p := range positions {
		arr := []float64{
			math.Round(p[0]*100) / 100,
			math.Round(p[1]*100) / 100,
			p[2],
		}
		positions_back = append(positions_back, arr)
	}
	positions = positions_back

	var params = make(map[string]interface{})
	for key, item := range self.Mesh_config {
		params[key] = item
	}
	params["min_x"] = min(0, positions)[0] + x_offset
	params["max_x"] = max(0, positions)[0] + x_offset
	params["min_y"] = min(1, positions)[1] + y_offset
	params["max_y"] = max(1, positions)[1] + y_offset
	x_cnt := params["x_count"]
	y_cnt := params["y_count"]

	if len(self.Substituted_indices) > 0 {
		// Replace substituted points with the original generated
		// point.  Its Z Value is the average probed Z of the
		// substituted points.
		var corrected_pts [][]float64
		idx_offset := 0
		start_idx := 0
		var fpt []float64
		for i, pts := range self.Substituted_indices {
			length := int(math.Min(float64(len(self.Points[i])), 2))
			for j := 0; j < length; j++ {
				p := self.Points[i][j]
				o := offsets[j]
				fpt = append(fpt, p-o)
			}
			// offset the index to account for additional samples
			idx := i + idx_offset
			// Add "normal" points
			corrected_pts = append(corrected_pts, positions[start_idx:idx]...)
			var avg_z float64
			for _, p := range positions[idx : idx+len(pts)] {
				avg_z += p[2]
			}
			avg_z = avg_z / float64(len(pts))
			idx_offset += len(pts) - 1
			start_idx = idx + len(pts)
			fpt = append(fpt, avg_z)
			logger.Debug(
				"bed_mesh: Replacing value at faulty index %d"+
					" (%.4f, %.4f): avg value = %.6f, avg w/ z_offset = %.6f",
				i, fpt[0], fpt[1], avg_z, avg_z-z_offset)
			corrected_pts = append(corrected_pts, fpt)
		}
		corrected_pts = append(corrected_pts, positions[start_idx:]...)
		// validate corrected positions
		if len(self.Points) != len(corrected_pts) {
			self.Dump_points(positions, corrected_pts, offsets)
			panic(
				fmt.Sprintf("bed_mesh: invalid position list size, "+
					"generated count: %d, probed count: %d",
					len(self.Points), len(corrected_pts)))
		}
		length := int(math.Min(float64(len(self.Points)), float64(len(corrected_pts))))
		for k := 0; k < length; k++ {
			gen_pt := self.Points[k]
			probed := corrected_pts[k]
			var off_pt []float64
			innerLength := int(math.Min(float64(len(gen_pt)), 2.))
			for l := 0; l < innerLength; l++ {
				p := gen_pt[l]
				o := offsets[l]
				off_pt = append(off_pt, p-o)
			}
			if !Isclose(off_pt[0], probed[0], 1e-09, .1) ||
				!Isclose(off_pt[1], probed[1], 1e-09, .1) {
				self.Dump_points(positions, corrected_pts, offsets)
				panic(fmt.Sprintf(
					"bed_mesh: point mismatch, orig = (%.2f, %.2f)"+
						", probed = (%.2f, %.2f)",
					off_pt[0], off_pt[1], probed[0], probed[1]))
			}
		}
		positions = corrected_pts
	}

	if self.Relative_reference_index != nil {
		// zero out probe z offset and
		// set offset relative to reference index
		z_offset = positions[*self.Relative_reference_index][2]
	}
	var probed_matrix [][]float64
	var row []float64
	prev_pos := positions[0]
	for _, pos := range positions {
		// y has changed, append row and start new
		// logger.Debugf("probe_finalize pos:%v\n", pos)
		if !Isclose(pos[1], prev_pos[1], 1e-09, .1) {
			probed_matrix = append(probed_matrix, row)
			row = []float64{}
		}
		if pos[0] > prev_pos[0] {
			// probed in the positive direction
			row = append(row, pos[2]-z_offset)
		} else {
			// probed in the negative direction
			row = append([]float64{pos[2] - z_offset}, row...)
		}
		prev_pos = pos
	}
	// append last row
	probed_matrix = append(probed_matrix, row)
	// logger.Debugf("probe_finalize source probed_matrix:%v\n", probed_matrix)
	// make sure the y-axis is the correct length
	if len(probed_matrix) != y_cnt {
		panic(
			fmt.Sprintf("bed_mesh: Invalid y-axis table length\n"+
				"Probed table length: %d Probed Table:\n%v",
				len(probed_matrix), probed_matrix))
	}

	if self.Radius != nil && *self.Radius > 0 {
		// round bed, extrapolate probed values to create a square mesh
		for _, row := range probed_matrix {
			row_size := len(row)
			if row_size&1 == 0 {
				// an even number of points in a row shouldn't be possible
				msg := "bed_mesh: incorrect number of points sampled on X\n"
				msg += "Probed Table:\n"
				msg += fmt.Sprintf("%v", probed_matrix)
				panic(msg)
			}
			buf_cnt := x_cnt.(int) - row_size/2
			if buf_cnt == 0 {
				continue
			}
			var left_buffer []float64
			var right_buffer []float64
			for i := 0; i < buf_cnt; i++ {
				left_buffer = append(left_buffer, row[0])
				right_buffer = append(right_buffer, row[row_size-1])
			}
			row = append(left_buffer, row...)
			row = append(row, right_buffer...)
		}
	}
	// logger.Debugf("probe_finalize probed_matrix:%v\n", probed_matrix)
	// make sure that the x-axis is the correct length
	for _, row := range probed_matrix {
		if len(row) != x_cnt {
			panic(
				fmt.Sprintf("bed_mesh: invalid x-axis table length\n"+
					"Probed table length: %d Probed Table:\n%v",
					len(probed_matrix), probed_matrix))
		}
	}
	z_mesh := NewZMesh(params)
	z_mesh.Build_mesh(probed_matrix)
	self.Bedmesh.Set_mesh(z_mesh)
	logger.Debug("Mesh Bed Leveling Complete")
	self.Bedmesh.Save_profile(self.Profile_name)
	return ""
}
func max(index int, arr [][]float64) []float64 {
	maxArr := arr[0]
	for _, val := range arr {
		if maxArr[index] < val[index] {
			maxArr = val
		}
	}
	return maxArr
}
func min(index int, arr [][]float64) []float64 {
	minArr := arr[0]
	for _, val := range arr {
		if val[index] < minArr[index] {
			minArr = val
		}
	}
	return minArr
}
func (self *BedMeshCalibrate) Dump_points(probed_pts [][]float64, corrected_pts [][]float64, offsets []float64) {
	// logs generated points with offset applied, points received
	// from the finalize callback, and the list of corrected points
	max_len := int(math.Max(float64(len(self.Points)), math.Max(float64(len(probed_pts)), float64(len(corrected_pts)))))
	logger.Debugf("bed_mesh: calibration point dump\nIndex | %-17s| %-25s|"+
		" Corrected Point", "Generated Point", "Probed Point")
	for i := 0; i < max_len; i++ {
		gen_pt := ""
		probed_pt := ""
		corr_pt := ""
		if i < len(self.Points) {
			length := math.Min(float64(len(self.Points[i])), 2)
			var off_pt []float64
			for j := 0; j < int(length); j++ {
				p := self.Points[i][j]
				o := offsets[j]
				off_pt = append(off_pt, p-o)
			}
			gen_pt = fmt.Sprintf("(%.2f, %.2f)", off_pt[0], off_pt[1])

		}
		if i < len(probed_pts) {
			probed_pt = fmt.Sprintf("(%.2f, %.2f, %.4f)", probed_pts[i][0], probed_pts[i][1], probed_pts[i][2])
		}
		if i < len(corrected_pts) {
			corr_pt = fmt.Sprintf("(%.2f, %.2f, %.4f)", corrected_pts[i][0], corrected_pts[i][1], corrected_pts[i][2])
		}
		logger.Debugf(fmt.Sprintf("  %-4d| %-17s| %-25s| %s", i, gen_pt, probed_pt, corr_pt))
	}

}

type MoveSplitter struct {
	Split_delta_z       float64
	Move_check_distance float64
	Z_mesh              *ZMesh
	Fade_offset         float64
	Gcode               *GCodeDispatch
	Pre_pos             []float64
	Next_pos            []float64
	Current_pos         []float64
	Z_factor            float64
	Z_offset            float64
	Traverse_complete   bool
	Distance_checked    float64
	Total_move_length   float64
	Axis_move           []bool
}

func NewMoveSplitter(config *ConfigWrapper, gcode *GCodeDispatch) *MoveSplitter {
	self := &MoveSplitter{}
	self.Split_delta_z = config.Getfloat("split_delta_z", .025, 0.01, 0, 0, 0, true)
	self.Move_check_distance = config.Getfloat("move_check_distance", 5., 1., 0, 0, 0, true)
	self.Z_mesh = nil
	self.Fade_offset = 0.
	self.Gcode = gcode
	return self
}
func (self *MoveSplitter) Initialize(mesh *ZMesh, fade_offset float64) {
	self.Z_mesh = mesh
	self.Fade_offset = fade_offset
}
func (self *MoveSplitter) Build_move(prev_pos []float64, next_pos []float64, factor float64) {

	self.Pre_pos = append([]float64{}, prev_pos...)
	self.Next_pos = append([]float64{}, next_pos...)
	self.Current_pos = append([]float64{}, prev_pos...)
	self.Z_factor = factor
	self.Z_offset = self.Calc_z_offset(prev_pos)
	self.Traverse_complete = false
	self.Distance_checked = 0.
	axes_d := []float64{}
	for i := 0; i < 4; i++ {
		axes_d = append(axes_d, self.Next_pos[i]-self.Pre_pos[i])
	}
	sum_val := 0.
	for i := 0; i < 3; i++ {
		sum_val += axes_d[i] * axes_d[i]
	}
	self.Total_move_length = math.Sqrt(sum_val)
	var axis_move []bool
	for _, d := range axes_d {
		axis_move = append(axis_move, !Isclose(d, 0, 1e-10, 0))
	}
	self.Axis_move = axis_move
}
func (self *MoveSplitter) Calc_z_offset(pos []float64) float64 {
	z := self.Z_mesh.Calc_z(pos[0], pos[1])
	offset := self.Fade_offset
	return self.Z_factor*(z-offset) + offset
}
func (self *MoveSplitter) Set_next_move(distance_from_prev float64) error {
	t := distance_from_prev / self.Total_move_length
	if t > 1. || t < 0 {
		panic(
			"bed_mesh: Slice distance is negative " +
				"or greater than entire move length")
	}
	for i := 0; i < 4; i++ {
		if self.Axis_move[i] {
			self.Current_pos[i] = Lerp(
				t, self.Pre_pos[i], self.Next_pos[i])
		}
	}
	return nil
}
func (self *MoveSplitter) Split() []float64 {
	if !self.Traverse_complete {
		if self.Axis_move[0] || self.Axis_move[1] {
			// X and/or Y axis move, traverse if necessary
			for self.Distance_checked+self.Move_check_distance < self.Total_move_length {
				self.Distance_checked += self.Move_check_distance
				self.Set_next_move(self.Distance_checked)
				next_z := self.Calc_z_offset(self.Current_pos)
				if math.Abs(next_z-self.Z_offset) >= self.Split_delta_z {
					self.Z_offset = next_z
					sum_val := self.Current_pos[2] + self.Z_offset
					return []float64{self.Current_pos[0], self.Current_pos[1],
						sum_val, self.Current_pos[3]}
				}
			}
		}
		// end of move reached
		self.Current_pos = append([]float64{}, self.Next_pos...)
		self.Z_offset = self.Calc_z_offset(self.Current_pos)
		// Its okay to add Z-Offset to the final move, since it will not be
		// used again.
		self.Current_pos[2] += self.Z_offset
		self.Traverse_complete = true
		return self.Current_pos
	} else {
		// Traverse complete
		return nil
	}
}

type ZMesh struct {
	Probed_matrix [][]float64
	Mesh_matrix   [][]float64
	Mesh_params   map[string]interface{}
	Avg_z         float64
	Mesh_offsets  []float64
	Mesh_x_min    float64
	Mesh_x_max    float64
	Mesh_y_min    float64
	Mesh_y_max    float64
	Mesh_x_count  int
	Mesh_y_count  int
	X_mult        int
	Y_mult        int
	Mesh_x_dist   float64
	Mesh_y_dist   float64
	Sample        func([][]float64)
}

func NewZMesh(params map[string]interface{}) *ZMesh {
	self := &ZMesh{}
	self.Mesh_params = params
	self.Mesh_offsets = []float64{0., 0.}
	logger.Debug("bed_mesh: probe/mesh parameters:")
	for key, val := range self.Mesh_params {
		logger.Debugf("%s :  %v", key, val)
	}
	self.Mesh_x_min = params["min_x"].(float64)
	self.Mesh_x_max = params["max_x"].(float64)
	self.Mesh_y_min = params["min_y"].(float64)
	self.Mesh_y_max = params["max_y"].(float64)

	logger.Debugf("bed_mesh: Mesh Min: (%.2f,%.2f) Mesh Max: (%.2f,%.2f)",
		self.Mesh_x_min, self.Mesh_x_max, self.Mesh_y_min, self.Mesh_y_max)

	// Set the interpolation algorithm
	interpolationAlgos := map[string]func([][]float64){
		"lagrange": self.Sample_lagrange,
		"bicubic":  self.Sample_bicubic,
		"direct":   self.Sample_direct,
	}
	self.Sample = interpolationAlgos[params["algo"].(string)]

	// Number of points to interpolate per segment
	mesh_x_pps := params["mesh_x_pps"].(int)
	mesh_y_pps := params["mesh_y_pps"].(int)
	px_cnt := params["x_count"].(int)
	py_cnt := params["y_count"].(int)
	self.Mesh_x_count = (px_cnt-1)*mesh_x_pps + px_cnt
	self.Mesh_y_count = (py_cnt-1)*mesh_y_pps + py_cnt
	self.X_mult = mesh_x_pps + 1
	self.Y_mult = mesh_y_pps + 1

	logger.Debugf("bed_mesh: Mesh grid size - X:%d, Y:%d", self.Mesh_x_count, self.Mesh_y_count)
	self.Mesh_x_dist = (self.Mesh_x_max - self.Mesh_x_min) / float64(self.Mesh_x_count-1)
	self.Mesh_y_dist = (self.Mesh_y_max - self.Mesh_y_min) / float64(self.Mesh_y_count-1)
	return self
}
func (self *ZMesh) Get_mesh_matrix() [][]float64 {
	if self.Mesh_matrix != nil {
		var arr [][]float64
		for _, line := range self.Mesh_matrix {
			var round_arr []float64
			for _, z := range line {
				round_arr = append(round_arr, math.Round(z*1000000)/1000000)
			}
			arr = append(arr, round_arr)
		}
		return arr
	}
	return nil
}
func (self *ZMesh) Get_probed_matrix() [][]float64 {
	if self.Probed_matrix != nil {
		var arr [][]float64
		for _, line := range self.Probed_matrix {
			var round_arr []float64
			for _, z := range line {
				round_arr = append(round_arr, math.Round(z*1000000)/1000000)
			}
			arr = append(arr, round_arr)
		}
		return arr
	}
	return nil
}
func (self *ZMesh) Get_mesh_params() map[string]interface{} {
	return self.Mesh_params
}
func (self *ZMesh) Print_probed_matrix(print_func func(string, bool)) {
	if self.Probed_matrix != nil {
		msg := "Mesh Leveling Probed Z positions:\n"
		for _, line := range self.Probed_matrix {
			for _, x := range line {
				msg += fmt.Sprintf(" %f", x)
			}
			msg += "\n"
		}
		print_func(msg, true)
	} else {
		print_func("bed_mesh: bed has not been probed", true)
	}
}
func (self *ZMesh) Print_mesh(print_func func(v ...interface{}), move_z *int) {
	matrix := self.Get_mesh_matrix()
	if matrix != nil {
		var msg strings.Builder
		msg.WriteString(fmt.Sprintf("Mesh X,Y: %d,%d\n", self.Mesh_x_count, self.Mesh_y_count))
		if move_z != nil {
			msg.WriteString(fmt.Sprintf("Search Height: %d\n", move_z))
		}
		msg.WriteString(fmt.Sprintf("Mesh Offsets: X=%.4f, Y=%.4f\n",
			self.Mesh_offsets[0], self.Mesh_offsets[1]))
		msg.WriteString(fmt.Sprintf("Mesh Average: %.2f\n", self.Avg_z))
		mesh_min, mesh_max := self.Get_z_range()
		rng := []float64{mesh_min, mesh_max}
		msg.WriteString(fmt.Sprintf("Mesh Range: min=%.4f max=%.4f\n", rng[0], rng[1]))
		msg.WriteString(fmt.Sprintf("Interpolation Algorithm: %s\n",
			self.Mesh_params["algo"]))
		msg.WriteString("Measured points:\n")
		for y_line := self.Mesh_y_count - 1; y_line >= 0; y_line-- {
			for _, z := range matrix[y_line] {
				msg.WriteString(fmt.Sprintf(" %f", z))
			}
			msg.WriteString("\n")
		}
		print_func(msg.String())
	} else {
		print_func("bed_mesh: Z Mesh not generated")
	}
}
func (self *ZMesh) Build_mesh(z_matrix [][]float64) {
	self.Probed_matrix = z_matrix
	self.Sample(z_matrix)
	sum_val := 0.
	len_val := 0
	for _, x := range self.Mesh_matrix {
		sum := 0.
		for _, item := range x {
			sum += item
		}
		sum_val += sum
		len_val += len(x)
	}
	self.Avg_z = sum_val / float64(len_val)
	// Round average to the nearest 100th.  This
	// should produce an offset that is divisible by common
	// z step distances
	self.Avg_z = math.Round(self.Avg_z*1000) / 1000
	// self.Print_mesh(logger.Debug, nil)
}
func (self *ZMesh) Set_mesh_offsets(offsets []float64) {
	for i, o := range offsets {
		if o != 0.0 {
			self.Mesh_offsets[i] = o
		}
	}
}
func (self *ZMesh) Get_x_coordinate(index int) float64 {
	return self.Mesh_x_min + self.Mesh_x_dist*float64(index)
}
func (self *ZMesh) Get_y_coordinate(index int) float64 {
	return self.Mesh_y_min + self.Mesh_y_dist*float64(index)
}

func (self *ZMesh) Calc_z(x, y float64) float64 {
	if self.Mesh_matrix != nil {
		tbl := self.Mesh_matrix
		tx, xidx := self.Get_linear_index(x+self.Mesh_offsets[0], 0)
		ty, yidx := self.Get_linear_index(y+self.Mesh_offsets[1], 1)
		z0 := Lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx+1])
		z1 := Lerp(tx, tbl[yidx+1][xidx], tbl[yidx+1][xidx+1])
		return Lerp(ty, z0, z1)
	} else {
		// No mesh table generated, no z-adjustment
		return 0.
	}
}

/*
	 func (self *ZMesh) Calc_z(x, y float64) float64 {
		if self.Mesh_matrix != nil {
			tbl := self.Mesh_matrix
			tx, xidx := self.Get_linear_index(x+self.Mesh_offsets[0], 0)
			ty, yidx := self.Get_linear_index(y+self.Mesh_offsets[1], 1)
			z1 := tbl[yidx][xidx]
			d2 := tbl[yidx+1][xidx] - z1
			z3 := tbl[yidx][xidx+1]
			d4 := tbl[yidx+1][xidx+1] - z3
			L := z1 + d2*ty
			R := z3 + d4*ty
			D := R - L
			return L + tx*D
		} else {
			// No mesh table generated, no z-adjustment
			return 0.
		}
	}
*/
func (self *ZMesh) Get_z_range() (float64, float64) {
	if self.Mesh_matrix != nil {
		mesh_min := 0.
		mesh_max := 0.
		for _, x := range self.Mesh_matrix {
			min := 0.
			max := 0.
			for _, item := range x {
				if item < min {
					min = item
				}
				if item > max {
					max = item
				}
			}
			if min < mesh_min {
				mesh_min = min
			}
			if max > mesh_max {
				mesh_max = max
			}
		}
		return mesh_min, mesh_max
	}
	return 0., 0.
}
func (self *ZMesh) Get_linear_index(coord float64, axis int) (float64, int) {
	var meshMin float64
	var meshCnt int
	var meshDist float64
	var cfunc func(int) float64

	if axis == 0 {
		// X-axis
		meshMin = self.Mesh_x_min
		meshCnt = self.Mesh_x_count
		meshDist = self.Mesh_x_dist
		cfunc = self.Get_x_coordinate
	} else {
		// Y-axis
		meshMin = self.Mesh_y_min
		meshCnt = self.Mesh_y_count
		meshDist = self.Mesh_y_dist
		cfunc = self.Get_y_coordinate
	}

	t := 0.
	idx := int(math.Floor((coord - meshMin) / meshDist))
	idx = int(Constrain(float64(idx), 0., float64(meshCnt-2)))
	t = (coord - cfunc(idx)) / meshDist
	return Constrain(t, 0., 1.), idx
}
func (self *ZMesh) Sample_direct(zMatrix [][]float64) {
	self.Mesh_matrix = zMatrix
}
func (self *ZMesh) Sample_lagrange(z_matrix [][]float64) {
	x_mult := self.X_mult
	y_mult := self.Y_mult
	for j := 0; j < self.Mesh_y_count; j++ {
		var arr []float64
		for i := 0; i < self.Mesh_x_count; i++ {
			val := 0.
			if (i%x_mult) != 0 || j%y_mult != 0 {
				val = 0
			} else {
				val = z_matrix[j/y_mult][i/x_mult]
			}
			arr = append(arr, val)
		}
		self.Mesh_matrix = append(self.Mesh_matrix, arr)
	}
	xpts, ypts := self.Get_lagrange_coords()
	// Interpolate X coordinates
	for i := 0; i < self.Mesh_y_count; i++ {
		// only interpolate X-rows that have probed coordinates
		if i%y_mult != 0 {
			continue
		}
		for j := 0; j < self.Mesh_x_count; j++ {
			if j%x_mult == 0 {
				continue
			}
			x := self.Get_x_coordinate(j)
			self.Mesh_matrix[i][j] = self.Calc_lagrange(xpts, x, i, 0)
		}
	}
	// Interpolate Y coordinates
	for i := 0; i < self.Mesh_x_count; i++ {
		for j := 0; j < self.Mesh_y_count; j++ {
			if j%y_mult == 0 {
				continue
			}
			y := self.Get_y_coordinate(j)
			self.Mesh_matrix[j][i] = self.Calc_lagrange(ypts, y, i, 1)
		}
	}
}
func (self *ZMesh) Get_lagrange_coords() ([]float64, []float64) {
	var xpts, ypts []float64
	for i := 0; i < self.Mesh_params["x_count"].(int); i++ {
		xpts = append(xpts, self.Get_x_coordinate(i*self.X_mult))
	}
	for j := 0; j < self.Mesh_params["y_count"].(int); j++ {
		ypts = append(ypts, self.Get_y_coordinate(j*self.Y_mult))
	}
	return xpts, ypts
}
func (self *ZMesh) Calc_lagrange(lpts []float64, c float64, vec int, axis int) float64 {
	pt_cnt := len(lpts)
	total := 0.
	for i := 0; i < pt_cnt; i++ {
		n := 1.
		d := 1.
		for j := 0; j < pt_cnt; j++ {
			if j == i {
				continue
			}
			n *= c - lpts[j]
			d *= lpts[i] - lpts[j]
		}
		var z float64
		if axis == 0 {
			// Calc X-Axis
			z = self.Mesh_matrix[vec][i*self.X_mult]
		} else {
			// Calc Y-Axis
			z = self.Mesh_matrix[i*self.Y_mult][vec]
		}
		total += z * n / d
	}
	return total
}
func (self *ZMesh) Sample_bicubic(z_matrix [][]float64) {
	// should work for any number of probe points above 3x3
	x_mult := self.X_mult
	y_mult := self.Y_mult
	c := self.Mesh_params["tension"]
	for j := 0; j < self.Mesh_y_count; j++ {
		var arr []float64
		for i := 0; i < self.Mesh_x_count; i++ {
			val := 0.
			if i%x_mult != 0 || j%y_mult != 0 {
				val = 0.
			} else {
				val = z_matrix[j/y_mult][i/x_mult]
			}
			arr = append(arr, val)
		}
		self.Mesh_matrix = append(self.Mesh_matrix, arr)
	}
	// Interpolate X values
	for y := 0; y < self.Mesh_y_count; y++ {
		if y%y_mult != 0 {
			continue
		}
		for x := 0; x < self.Mesh_x_count; x++ {
			if x%x_mult == 0 {
				continue
			}
			pts := self.Get_x_ctl_pts(x, y)

			self.Mesh_matrix[y][x] = self.Cardinal_spline(pts, c.(float64))
		}
	}
	// Interpolate Y values
	for x := 0; x < self.Mesh_x_count; x++ {
		for y := 0; y < self.Mesh_y_count; y++ {
			if y%y_mult == 0 {
				continue
			}
			pts := self.Get_y_ctl_pts(x, y)

			self.Mesh_matrix[y][x] = self.Cardinal_spline(pts, c.(float64))
		}
	}
}
func (self *ZMesh) Get_x_ctl_pts(x, y int) []float64 {
	// Fetch control points and t for a X value in the mesh
	x_mult := self.X_mult
	x_row := self.Mesh_matrix[y]
	last_pt := self.Mesh_x_count - 1 - x_mult
	var p0, p1, p2, p3, t float64
	if x < x_mult {
		p0 = x_row[0]
		p1 = x_row[0]
		p2 = x_row[x_mult]
		p3 = x_row[2*x_mult]
		t = float64(x) / float64(x_mult)
	} else if x > last_pt {
		p0 = x_row[last_pt-x_mult]
		p1 = x_row[last_pt]
		p2 = x_row[last_pt+x_mult]
		p3 = x_row[last_pt+x_mult]
		t = float64(x-last_pt) / float64(x_mult)
	} else {
		found := false
		for i := x_mult; i < last_pt; i += x_mult {
			if x > i && x < i+x_mult {
				p0 = x_row[i-x_mult]
				p1 = x_row[i]
				p2 = x_row[i+x_mult]
				p3 = x_row[i+2*x_mult]
				t = float64(x-i) / float64(x_mult)
				found = true
				break
			}
		}
		if !found {
			panic(&BedMeshError{"bed_mesh: Error finding x control points"})
		}
	}
	return []float64{p0, p1, p2, p3, t}
}
func (self *ZMesh) Get_y_ctl_pts(x, y int) []float64 {
	// Fetch control points and t for a Y value in the mesh
	y_mult := self.Y_mult
	last_pt := self.Mesh_y_count - 1 - y_mult
	y_col := self.Mesh_matrix
	var p0, p1, p2, p3, t float64
	if y < y_mult {
		p0 = y_col[0][x]
		p1 = y_col[0][x]
		p2 = y_col[y_mult][x]
		p3 = y_col[2*y_mult][x]
		t = float64(y) / float64(y_mult)
	} else if y > last_pt {
		p0 = y_col[last_pt-y_mult][x]
		p1 = y_col[last_pt][x]
		p2 = y_col[last_pt+y_mult][x]
		p3 = y_col[last_pt+y_mult][x]
		t = float64(y-last_pt) / float64(y_mult)
	} else {
		found := false
		for i := y_mult; i < last_pt; i += y_mult {
			if y > 1 && y < i+y_mult {
				p0 = y_col[i-y_mult][x]
				p1 = y_col[i][x]
				p2 = y_col[i+y_mult][x]
				p3 = y_col[i+2*y_mult][x]
				t = float64(y-i) / float64(y_mult)
				found = true
				break
			}
		}
		if !found {
			panic(&BedMeshError{"bed_mesh: Error finding y control points"})
		}
	}
	return []float64{p0, p1, p2, p3, t}
}
func (self *ZMesh) Cardinal_spline(p []float64, tension float64) float64 {
	t := p[4]
	t2 := t * t
	t3 := t2 * t
	m1 := tension * (p[2] - p[0])
	m2 := tension * (p[3] - p[1])
	a := p[1] * (2*t3 - 3*t2 + 1)
	b := p[2] * (-2*t3 + 3*t2)
	c := m1 * (t3 - 2*t2 + t)
	d := m2 * (t3 - t2)
	return a + b + c + d
}

type ProfileManager struct {
	Name                  string
	Printer               *Printer
	Gcode                 *GCodeDispatch
	Bedmesh               *BedMesh
	Profiles              map[string]interface{}
	Current_profile       string
	Incompatible_profiles []string
}

func NewProfileManager(config *ConfigWrapper, bedmesh *BedMesh) *ProfileManager {
	self := &ProfileManager{}
	self.Name = config.Get_name()
	self.Printer = config.Get_printer()
	gcode_obj := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	logger.Error(err)
	//}
	self.Gcode = gcode_obj.(*GCodeDispatch)
	self.Bedmesh = bedmesh
	self.Profiles = map[string]interface{}{}
	self.Current_profile = ""
	self.Incompatible_profiles = []string{}
	// Fetch stored profiles from Config
	stored_profs := config.Get_prefix_sections(self.Name)
	stored_profs_back := []*ConfigWrapper{}
	for _, s := range stored_profs {
		if s.Get_name() != self.Name {
			stored_profs_back = append(stored_profs_back, s)
		}
	}
	stored_profs = stored_profs_back
	for _, profile := range stored_profs {
		name := strings.Join(strings.Split(profile.Get_name(), " ")[1:], "")
		version := profile.Getint("version", 0, 0, 0, true)
		if version != PROFILE_VERSION {
			logger.Errorf("bed_mesh: Profile [%s] not compatible with this version\n"+
				"of bed_mesh.  Profile Version: %d Current Version: %d ",
				name, version, PROFILE_VERSION)
			self.Incompatible_profiles = append(self.Incompatible_profiles, name)
			continue
		}
		self.Profiles[name] = map[string]interface{}{}
		zvals := profile.Getlists("points", nil, []string{",", "\n"}, 0, reflect.Float64, true)
		self.Profiles[name].(map[string]interface{})["points"] = zvals
		params := map[string]interface{}{}
		self.Profiles[name].(map[string]interface{})["mesh_params"] = params
		for key, t := range PROFILE_OPTIONS {
			if t == reflect.Int {
				params[key] = profile.Getint(key, object.Sentinel{}, 0, 0, true)
			} else if t == reflect.Float64 {
				params[key] = profile.Getfloat(key, object.Sentinel{}, 0, 0, 0, 0, true)
			} else if t == reflect.String {
				params[key] = profile.Get(key, object.Sentinel{}, true)
			}
		}

	}
	// Register GCode
	self.Gcode.Register_command("BED_MESH_PROFILE", self.Cmd_BED_MESH_PROFILE,
		false,
		cmd_BED_MESH_PROFILE_help)
	return self
}
func (self *ProfileManager) Get_profiles() map[string]interface{} {
	return self.Profiles
}
func (self *ProfileManager) Get_current_profile() string {
	return self.Current_profile
}
func (self *ProfileManager) Check_incompatible_profiles() {
	if len(self.Incompatible_profiles) != 0 {
		configfile_obj := self.Printer.Lookup_object("configfile", object.Sentinel{})
		//if err != nil {
		//	logger.Error(err)
		//}
		configfile := configfile_obj.(*PrinterConfig)
		for _, profile := range self.Incompatible_profiles {
			configfile.Remove_section("bed_mesh " + profile)
		}
		self.Gcode.Respond_info(fmt.Sprintf(
			"The following incompatible profiles have been detected\n"+
				"and are scheduled for removal:\n%s\n"+
				"The SAVE_CONFIG command will update the printer config\n"+
				"file and restart the printer", strings.Join(self.Incompatible_profiles, "\n")), true)

	}
}

func (self *ProfileManager) Save_profile(prof_name string) error {
	z_mesh := self.Bedmesh.Get_mesh()
	if z_mesh == nil {
		self.Gcode.Respond_info(fmt.Sprintf("Unable to save to profile [%s], the bed has not been probed",
			prof_name), true)
		return nil
	}
	probed_matrix := z_mesh.Get_probed_matrix()
	mesh_params := z_mesh.Get_mesh_params()
	configfile_obj := self.Printer.Lookup_object("configfile", object.Sentinel{})
	configfile := configfile_obj.(*PrinterConfig)
	//if err != nil {
	//	logger.Error(err)
	//}
	cfg_name := self.Name + " " + prof_name
	// set params
	var z_values strings.Builder
	for idx, line := range probed_matrix {
		if idx != 0 {
			z_values.WriteString("\n")
		}

		zvalTmp := make([]string, 0, len(line))
		for _, p := range line {
			// z_values.WriteString(fmt.Sprintf("%.6f, ", p))
			zvalTmp = append(zvalTmp, fmt.Sprintf("%.6f", p))
		}
		z_values.WriteString(strings.Join(zvalTmp, ", "))
	}
	configfile.Set(cfg_name, "version", strconv.Itoa(PROFILE_VERSION))
	configfile.Set(cfg_name, "points", z_values.String())

	for key, value := range mesh_params {
		if _, ok := value.(string); ok {
			configfile.Set(cfg_name, key, value.(string))
		} else if _, ok := value.(int); ok {
			configfile.Set(cfg_name, key, strconv.Itoa(value.(int)))
		} else if _, ok := value.(float64); ok {
			configfile.Set(cfg_name, key, strconv.FormatFloat(value.(float64), 'f', -1, 64))
		}
	}
	// save copy in local storage
	// ensure any self.profiles returned as status remains immutable
	profiles := make(map[string]interface{})
	for key, val := range self.Profiles {
		profiles[key] = val
	}
	profile := make(map[string]interface{})
	profiles[prof_name] = profile
	profile["points"] = probed_matrix
	mesh_params_copy := make(map[string]interface{})
	for key, val := range mesh_params {
		mesh_params_copy[key] = val
	}
	profile["mesh_params"] = mesh_params_copy
	self.Profiles = profiles
	self.Current_profile = prof_name
	self.Bedmesh.Update_status()
	logger.Debugf("Bed Mesh state has been saved to profile [%s]\n"+
		"for the current session.  The SAVE_CONFIG command will\n"+
		"update the printer config file and restart the printer.",
		prof_name)
	return nil
}
func (self *ProfileManager) Load_profile(prof_name string) error {
	profile := self.Profiles[prof_name]
	if profile == nil {
		logger.Errorf("bed_mesh: Unknown profile [%s]", prof_name)
		return nil
	}
	var probed_matrix [][]float64
	if _, ok := profile.(map[string]interface{})["points"].([][]interface{}); ok {
		arr := profile.(map[string]interface{})["points"].([][]interface{})
		for i := 0; i < len(arr); i++ {
			item := arr[i]
			var probed_matrix_item []float64
			for j := 0; j < len(item); j++ {
				probed_matrix_item = append(probed_matrix_item, item[j].(float64))
			}
			probed_matrix = append(probed_matrix, probed_matrix_item)
		}
	} else {
		arr := profile.(map[string]interface{})["points"].([][]float64)
		for i := 0; i < len(arr); i++ {
			item := arr[i]
			var probed_matrix_item []float64
			for j := 0; j < len(item); j++ {
				probed_matrix_item = append(probed_matrix_item, item[j])
			}
			probed_matrix = append(probed_matrix, probed_matrix_item)
		}
	}
	mesh_params := profile.(map[string]interface{})["mesh_params"].(map[string]interface{})
	z_mesh := NewZMesh(mesh_params)
	if err := self.Build_mesh_catch(z_mesh, probed_matrix); err != nil {
		return fmt.Errorf("%w", err)
	}
	self.Current_profile = prof_name
	self.Bedmesh.Set_mesh(z_mesh)
	return nil
}

func (pm *ProfileManager) Build_mesh_catch(z_mesh *ZMesh, probed_matrix [][]float64) (err error) {
	defer func() {
		if r := recover(); r != nil {
			err = fmt.Errorf("error: %v", r)
		}
	}()
	z_mesh.Build_mesh(probed_matrix)
	return nil
}

func (self *ProfileManager) Remove_profile(prof_name string) error {
	isIn := false
	for key, _ := range self.Profiles {
		if prof_name == key {
			isIn = true
			break
		}
	}
	if isIn {
		configfile_obj := self.Printer.Lookup_object("configfile", object.Sentinel{})
		//if err != nil {
		//	logger.Error(err)
		//}
		configfile := configfile_obj.(*PrinterConfig)
		configfile.Remove_section("bed_mesh " + prof_name)
		profiles := make(map[string]interface{})
		for key, val := range self.Profiles {
			profiles[key] = val
		}
		delete(profiles, prof_name)
		self.Bedmesh.Update_status()
		self.Gcode.Respond_info(fmt.Sprintf(
			"Profile [%s] removed from storage for this session.\n"+
				"The SAVE_CONFIG command will update the printer\n"+
				"configuration and restart the printer", prof_name), true)
	} else {
		self.Gcode.Respond_info(fmt.Sprintf(
			"No profile named [%s] to remove", prof_name), true)

	}
	return nil
}

const cmd_BED_MESH_PROFILE_help = "Bed Mesh Persistent Storage management"

func (self *ProfileManager) Cmd_BED_MESH_PROFILE(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	options := map[string]func(string) error{
		"LOAD":   self.Load_profile,
		"SAVE":   self.Save_profile,
		"REMOVE": self.Remove_profile,
	}
	for key, _ := range options {
		name := gcmd.Get(key, nil, "", nil, nil, nil, nil)
		if name != "" {
			if strings.TrimSpace(name) == "" {
				panic(fmt.Sprintf("Value for parameter '%s' must be specified", key))
			}
			if name == "default" && key == "SAVE" {
				gcmd.Respond_info(
					"Profile 'default' is reserved, please choose"+
						" another profile name.", true)
			} else {
				options[key](name)
				return nil
			}
		}
	}
	gcmd.Respond_info(fmt.Sprintf("Invalid syntax '%s'", gcmd.Get_commandline()), true)
	return nil
}
func Load_config_bed_mesh(config *ConfigWrapper) interface{} {
	return NewBedMesh(config)
}
