package project

type IKinematics interface {
	Set_position(newpos []float64, homing_axes []int)
	Check_move(move *Move)
	Get_status(eventtime float64) map[string]interface{}
	Get_steppers() []interface{}
	Note_z_not_homed()
	Calc_position(stepper_positions map[string]float64) []float64
	Home(homing_state *Homing)
}

type IExtruder interface {
	Update_move_time(flush_time float64, clear_history_time float64)
	Check_move(move *Move) error
	Find_past_position(print_time float64) float64
	Calc_junction(prev_move, move *Move) float64
	Get_name() string
	Get_heater() interface{}
	Get_trapq() interface{}
	Get_extruder_stepper() *ExtruderStepper
}
