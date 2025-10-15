package project

import (
	"encoding/json"
	"fmt"
	"k3c/common/logger"
	"k3c/common/utils/object"
	"log"
	"math"
	"sort"
	"strconv"
	"strings"
)

type ExcludeObject struct {
	printer                 *Printer
	gcode                   *GCodeDispatch
	gcode_move              *GCodeMove
	toolhead                *Toolhead
	next_transform          Itransform
	last_position_extruded  []float64
	last_position_excluded  []float64
	objects                 []map[string]interface{}
	excluded_objects        []string
	current_object          string
	in_excluded_region      bool
	last_position           []float64
	extrusion_offsets       map[string][]float64
	max_position_extruded   float64
	max_position_excluded   float64
	extruder_adj            float64
	initial_extrusion_moves int
	last_speed              float64
	was_excluded_at_start   bool
}

func NewExcludeObject(config *ConfigWrapper) *ExcludeObject {
	self := new(ExcludeObject)

	self.printer = config.Get_printer()
	self.gcode = MustLookupGcode(self.printer)
	self.gcode_move = MustLookupGCodeMove(self.printer)

	self.printer.Register_event_handler("project:connect", self._handle_connect)
	self.printer.Register_event_handler("virtual_sdcard:reset_file", self._reset_file)
	self.next_transform = nil
	self.last_position_extruded = []float64{0., 0., 0., 0.}
	self.last_position_excluded = []float64{0., 0., 0., 0.}
	self.extrusion_offsets = map[string][]float64{}
	self._reset_state()

	self.gcode.Register_command("EXCLUDE_OBJECT_START", self.cmd_EXCLUDE_OBJECT_START,
		true, "Marks the beginning the current object as labeled")
	self.gcode.Register_command("EXCLUDE_OBJECT_END", self.cmd_EXCLUDE_OBJECT_END,
		true, "Marks the end the current object")
	self.gcode.Register_command("EXCLUDE_OBJECT", self.cmd_EXCLUDE_OBJECT,
		true, "Cancel moves inside a specified objects")
	self.gcode.Register_command("EXCLUDE_OBJECT_DEFINE", self.cmd_EXCLUDE_OBJECT_DEFINE,
		true, "Provides a summary of an object")
	self.gcode.Register_command("EXCLUDE_OBJECT_END_NO_OBJ", self.cmd_EXCLUDE_OBJECT_END_NO_OBJ,
		true, "Indicate a non-object, purge tower, or other global feature")
	self.gcode.Register_command("EXCLUDE_OBJECT_SET_OBJ", self.cmd_EXCLUDE_OBJECT_SET_OBJ,
		true, "Set the number of objects")

	return self
}

func (self *ExcludeObject) registerTransform() {
	if self.next_transform == nil {
		tuningTower := MustLookupTuningTower(self.printer)
		if tuningTower.Is_active() {
			log.Println("The exclude_object move transform is not being " +
				"loaded due to Tuning tower being Active")
			return
		}
		self.next_transform = self.gcode_move.Set_move_transform(self, true)
		self.max_position_extruded = 0
		self.max_position_excluded = 0
		self.extruder_adj = 0
		self.initial_extrusion_moves = 5
		self.last_position = []float64{0., 0., 0., 0.}

		self.Get_position()
		copy(self.last_position_extruded[:], self.last_position[:])
		copy(self.last_position_excluded[:], self.last_position[:])
	}
}

func (self *ExcludeObject) _handle_connect([]interface{}) error {
	self.toolhead = MustLookupToolhead(self.printer)
	return nil
}

func (self *ExcludeObject) _unregister_transform() error {
	if self.next_transform != nil {
		tuningTower := MustLookupTuningTower(self.printer)
		if tuningTower.Is_active() {
			log.Println("The Exclude Object move transform was not " +
				"unregistered because it is not at the head of the transform chain.")
			return fmt.Errorf("The Exclude Object move transform was not " +
				"unregistered because it is not at the head of the transform chain.")
		}
		self.gcode_move.Set_move_transform(self.next_transform, true)
		self.next_transform = nil
		self.gcode_move.Reset_last_position(nil)
	}
	return nil
}

func (self *ExcludeObject) _reset_state() {
	self.objects = []map[string]interface{}{}
	self.excluded_objects = []string{}
	self.current_object = ""
	self.in_excluded_region = false
}

func (self *ExcludeObject) _reset_file([]interface{}) error {
	self._reset_state()
	self._unregister_transform()
	return nil
}

func (self *ExcludeObject) _get_extrusion_offsets() []float64 {
	extruderName := self.toolhead.Get_extruder().Get_name()
	offset, ok := self.extrusion_offsets[extruderName]
	if !ok {
		offset = []float64{0., 0., 0., 0.}
		self.extrusion_offsets[extruderName] = offset
	}
	return offset
}

func (self *ExcludeObject) Get_position() []float64 {
	offset := self._get_extrusion_offsets()
	pos := self.next_transform.Get_position()
	for i := 0; i < 4; i++ {
		self.last_position[i] = pos[i] + offset[i]
	}
	last_position := []float64{}
	copy(last_position, self.last_position)
	return last_position
}

func (self *ExcludeObject) _normal_move(newpos []float64, speed float64) {
	offset := self._get_extrusion_offsets()

	if self.initial_extrusion_moves > 0 &&
		self.last_position[3] != newpos[3] {
		// Since the transform is not loaded until there is a request to
		// exclude an object, the transform needs to track a few extrusions
		// to get the state of the extruder
		self.initial_extrusion_moves--
	}

	copy(self.last_position, newpos)
	copy(self.last_position_extruded, self.last_position)
	self.max_position_extruded = math.Max(self.max_position_extruded, newpos[3])

	// These next few conditionals handle the moves immediately after leaving
	// and excluded object.  The toolhead is at the end of the last printed
	// object and the gcode is at the end of the last excluded object.
	//
	// Ideally, there will be Z and E moves right away to adjust any offsets
	// before moving away from the last position.  Any remaining corrections
	// will be made on the firs XY move.
	if (offset[0] != 0 || offset[1] != 0) &&
		(newpos[0] != self.last_position_excluded[0] ||
			newpos[1] != self.last_position_excluded[1]) {

		offset[0] = 0
		offset[1] = 0
		offset[2] = 0
		offset[3] += self.extruder_adj
		self.extruder_adj = 0
	}

	if offset[2] != 0 && newpos[2] != self.last_position_excluded[2] {
		offset[2] = 0
	}

	if self.extruder_adj != 0 &&
		newpos[3] != self.last_position_excluded[3] {
		offset[3] += self.extruder_adj
		self.extruder_adj = 0
	}

	tx_pos := newpos
	for i := 0; i < 4; i++ {
		tx_pos[i] = newpos[i] - offset[i]
	}
	self.next_transform.Move(tx_pos, speed)
}

func (self *ExcludeObject) _ignore_move(newpos []float64, speed float64) {
	offset := self._get_extrusion_offsets()
	for i := 0; i < 3; i++ {
		offset[i] = newpos[i] - self.last_position_extruded[i]
	}
	offset[3] = offset[3] + newpos[3] - self.last_position[3]
	copy(self.last_position, newpos)
	copy(self.last_position_excluded, self.last_position)
	self.max_position_excluded = math.Max(self.max_position_excluded, newpos[3])
}

func (self *ExcludeObject) _move_into_excluded_region(newpos []float64, speed float64) {
	self.in_excluded_region = true
	self._ignore_move(newpos, speed)
}

func (self *ExcludeObject) _move_from_excluded_region(newpos []float64, speed float64) {
	self.in_excluded_region = false

	self.extruder_adj = self.max_position_excluded - self.last_position_excluded[3] - (self.max_position_extruded - self.last_position_extruded[3])
	self._normal_move(newpos, speed)
}

func (self *ExcludeObject) _test_in_excluded_region() bool {
	return self.current_object != "" && self.initial_extrusion_moves == 0
}

func (self *ExcludeObject) Get_status(eventtime float64) map[string]interface{} {
	status := map[string]interface{}{
		"objects":          self.objects,
		"excluded_objects": self.excluded_objects,
		"current_object":   self.current_object,
	}
	return status
}

func (self *ExcludeObject) Move(newpos []float64, speed float64) {
	moveInExcludedRegion := self._test_in_excluded_region()
	self.last_speed = speed

	if moveInExcludedRegion {
		if self.in_excluded_region {
			self._ignore_move(newpos, speed)
		} else {
			self._move_into_excluded_region(newpos, speed)
		}
	} else {
		if self.in_excluded_region {
			self._move_from_excluded_region(newpos, speed)
		} else {
			self._normal_move(newpos, speed)
		}
	}
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT_START(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	name := gcmd.Get("NAME", object.Sentinel{}, "", nil, nil, nil, nil)
	name = strings.ToUpper(name)
	if !self.object_exists(name) {
		self._add_object_definition(map[string]interface{}{"name": name})
	}
	self.current_object = name
	self.was_excluded_at_start = self._test_in_excluded_region()
	return nil
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT_END(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	if self.current_object == "" && self.next_transform != nil {
		gcmd.Respond_info("EXCLUDE_OBJECT_END called, but no "+
			"object is currently active", true)
		return nil
	}
	name := gcmd.Get("NAME", object.Sentinel{}, "", nil, nil, nil, nil)
	name = strings.ToUpper(name)
	if name != "" && name != self.current_object {
		gcmd.Respond_info(fmt.Sprintf("EXCLUDE_OBJECT_END NAME=%s does not match"+
			" the current object NAME=%s", name, self.current_object), true)
	}
	self.current_object = ""
	return nil
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	reset := gcmd.Get("RESET", nil, "", nil, nil, nil, nil)
	current := gcmd.Get("CURRENT", nil, "", nil, nil, nil, nil)
	name := gcmd.Get("NAME", "", "", nil, nil, nil, nil)
	name = strings.ToUpper(name)
	if reset != "" {
		if name != "" {
			self._unexclude_object(name)
		} else {
			self.excluded_objects = []string{}
		}
	} else if name != "" {
		if !self.object_is_excluded(name) {
			self._exclude_object(name)
		}
	} else if current != "" {
		if self.current_object == "" {
			gcmd.Respond_info("There is no current object to cancel", true)
		} else {
			self._exclude_object(self.current_object)
		}
	} else {
		self._list_excluded_objects(gcmd)
	}
	return nil
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT_DEFINE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	reset := gcmd.Get("RESET", nil, "", nil, nil, nil, nil)
	name := gcmd.Get("NAME", "", "", nil, nil, nil, nil)
	// name = strings.ToUpper(name)
	if reset != "" {
		self._reset_file(nil)
	} else if name != "" {
		parameters := gcmd.Get_command_parameters()

		center := parameters["CENTER"]

		polygon := parameters["POLYGON"]

		obj := map[string]interface{}{"name": name}

		if center != "" {
			var centerArr map[string]interface{}
			c := `{"center":[%s]}`
			err := json.Unmarshal([]byte(fmt.Sprintf(c, center)), &centerArr)
			if err != nil {
				logger.Panic(err)
			}
			c_ := centerArr["center"].([]interface{})

			obj["center"] = []float64{c_[0].(float64), c_[1].(float64)}
		}
		if polygon != "" {
			var polygonArr map[string]interface{}
			c := `{"polygon":%s}`
			err := json.Unmarshal([]byte(fmt.Sprintf(c, polygon)), &polygonArr)
			if err != nil {
				logger.Panic(err)
			}
			var p_arr [][]float64
			for _, p := range polygonArr["polygon"].([]interface{}) {

				p_arr = append(p_arr, []float64{p.([]interface{})[0].(float64), p.([]interface{})[1].(float64)})
			}
			obj["polygon"] = p_arr
		}
		self._add_object_definition(obj)
	} else {
		self._list_objects(gcmd)
	}
	return nil
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT_END_NO_OBJ(argv interface{}) error {
	if self.current_object != "" {
		self.gcode.Run_script_from_command(fmt.Sprintf("EXCLUDE_OBJECT_END NAME=%s", self.current_object))
	}
	return nil
}

func (self *ExcludeObject) cmd_EXCLUDE_OBJECT_SET_OBJ(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	t := gcmd.Get("T", nil, "", nil, nil, nil, nil)

	if t != "" {
		_t, _ := strconv.ParseInt(t, 10, 64)
		for i := 0; i < int(_t); i++ {
			self.gcode.Run_script_from_command(fmt.Sprintf("EXCLUDE_OBJECT_DEFINE NAME=%d", i))
		}
		return nil
	}
	return nil
}
func (self *ExcludeObject) _add_object_definition(definition map[string]interface{}) {
	self.objects = append(self.objects, definition)
	sort.Slice(self.objects, func(i, j int) bool {
		return self.objects[i]["name"].(string) < self.objects[j]["name"].(string)
	})
}

func (self *ExcludeObject) _exclude_object(name string) {
	self.registerTransform()
	self.gcode.Respond_info(fmt.Sprintf("Excluding object %s", name), true)
	if !self.object_is_excluded(name) {
		self.excluded_objects = append(self.excluded_objects, name)
		sort.Strings(self.excluded_objects)
	}
}

func (self *ExcludeObject) _unexclude_object(name string) {
	self.gcode.Respond_info(fmt.Sprintf("Unexcluding object %s", name), true)

	for i, obj := range self.excluded_objects {
		if obj == name {
			self.excluded_objects = append(self.excluded_objects[:i], self.excluded_objects[i+1:]...)
			sort.Strings(self.excluded_objects)
			break
		}
	}
}

func (self *ExcludeObject) _list_objects(gcmd *GCodeCommand) {
	if gcmd.Get("JSON", nil, "", nil, nil, nil, nil) != "" {
		objectList, _ := json.Marshal(self.objects)
		gcmd.Respond_info(fmt.Sprintf("Known objects: %s", string(objectList)), true)
	} else {
		var objectNames []string
		for _, obj := range self.objects {
			objectNames = append(objectNames, obj["name"].(string))
		}
		gcmd.Respond_info(fmt.Sprintf("Known objects: %s", objectNames), true)
	}
}

func (self *ExcludeObject) _list_excluded_objects(gcmd *GCodeCommand) {
	gcmd.Respond_info(fmt.Sprintf("Excluded objects: %s", self.excluded_objects), true)
}

func (self *ExcludeObject) object_exists(name string) bool {
	for _, obj := range self.objects {
		if obj["name"] == name {
			return true
		}
	}
	return false
}

func (self *ExcludeObject) object_is_excluded(name string) bool {
	for _, obj := range self.excluded_objects {
		if obj == name {
			return true
		}
	}
	return false
}

func Load_config_ExcludeObject(config *ConfigWrapper) interface{} {
	return NewExcludeObject(config)
}
