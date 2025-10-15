/*
# Automatic calibration of input shapers
#
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"fmt"
	math "k3c/common/cmath"
	"k3c/common/logger"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"math/bits"
	"os"
	"reflect"
)

const (
	MIN_FREQ        = 5.0
	MAX_FREQ        = 200.
	WINDOW_T_SEC    = 0.5
	MAX_SHAPER_FREQ = 150.
)

var TEST_DAMPING_RATIOS = []float64{0.075, 0.1, 0.15}
var AUTOTUNE_SHAPERS = []string{"zv", "mzv", "ei", "2hump_ei", "3hump_ei"}

/*
######################################################################
# Frequency response calculation and shaper auto-tuning
######################################################################
*/

type CalibrationData struct {
	freq_bins []float64
	psd_sum   []float64
	psd_x     []float64
	psd_y     []float64
	psd_z     []float64
	psd_list  [][]float64
	psd_map   map[string]*[]float64
	data_sets int
}

func NewCalibrationData(freq_bins []float64, psd_sum []float64, psd_x []float64, psd_y []float64, psd_z []float64) *CalibrationData {
	self := &CalibrationData{}
	self.freq_bins = freq_bins
	self.psd_sum = psd_sum
	self.psd_x = psd_x
	self.psd_y = psd_y
	self.psd_z = psd_z
	self.psd_list = [][]float64{self.psd_sum, self.psd_x, self.psd_y, self.psd_z}
	self.psd_map = map[string]*[]float64{"x": &psd_x, "y": &psd_y, "z": &psd_z, "all": &psd_sum}
	self.data_sets = 1
	return self
}

func (self *CalibrationData) Add_data(other *CalibrationData) {
	joined_data_sets := self.data_sets + other.data_sets
	for i, psd := range self.psd_list {
		other_psd := other.psd_list[i]
		// `other` data may be defined at different frequency bins,
		// interpolating to fix that.
		other_normalized := make([]float64, len(self.freq_bins))
		interp := maths.Interp(self.freq_bins, other.freq_bins, other_psd)
		for j := 0; j < len(self.freq_bins); j++ {
			other_normalized[j] = float64(other.data_sets) * interp[j]
		}
		for j := 0; j < len(psd); j++ {
			psd[j] *= float64(self.data_sets)
			psd[j] = (psd[j] + other_normalized[j]) * (1.0 / float64(joined_data_sets))
		}
	}
	self.data_sets = joined_data_sets
}

func (self *CalibrationData) Normalize_to_frequencies() {
	for j, psd := range self.psd_list {
		// Avoid division by zero errors
		for i := 0; i < len(self.freq_bins); i++ {
			psd[i] /= self.freq_bins[i] + 0.1
			// Remove low-frequency noise
			if self.freq_bins[i] < MIN_FREQ {
				psd[i] = 0.0
			} else if self.freq_bins[i] > MAX_FREQ {
				self.freq_bins = append([]float64{}, self.freq_bins[:i+1]...)
				self.psd_sum = append([]float64{}, self.psd_sum[:i+1]...)
				self.psd_list[j] = append([]float64{}, psd[:i+1]...)
				break
			}
		}
	}
}

func (self *CalibrationData) Get_psd(axis string) *[]float64 {
	return self.psd_map[axis]
}

type CalibrationResult struct {
	name      string
	freq      float64
	vals      []float64
	vibrs     float64
	smoothing float64
	score     float64
	max_accel float64
}

type ShaperCalibrate struct {
	printer *Printer
	Error   error
	numpy   interface{}
}

func NewShaperCalibrate(printer *Printer) *ShaperCalibrate {
	self := &ShaperCalibrate{}
	self.printer = printer
	return self
}

func (self *ShaperCalibrate) Background_process_exec(method interface{}, args []reflect.Value) interface{} {
	if self.printer == nil {
		return reflect.ValueOf(method).Call(args)
	}

	res_chan := make(chan []interface{})
	is_live_chan := make(chan bool)
	defer close(res_chan)
	defer close(is_live_chan)
	wrapper := func() {
		var res interface{}
		erro := func() (err error) {
			res = reflect.ValueOf(method).Call(args)[0].Interface()
			defer func() {
				if r := recover(); r != nil {
					err = fmt.Errorf("error: %v", r)
				}
			}()
			return nil
		}()
		if erro != nil {
			res_chan <- []interface{}{true, erro}
		}
		is_live_chan <- false
		res_chan <- []interface{}{false, res}

	}

	// Start a process to perform the calculation
	go wrapper()
	// Wait for the process to finish
	reactor := self.printer.Get_reactor()
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	eventtime := reactor.Monotonic()
	last_report_time := eventtime
	is_live := true
	for is_live {
		select {
		case is_live = <-is_live_chan:
		default:
			if eventtime > last_report_time+5. {
				last_report_time = eventtime
				gcode.Respond_info("Wait for calculations..", true)
			}
			eventtime = reactor.Pause(eventtime + .1)
		}

	}
	// Return results
	res := <-res_chan
	if res[0].(bool) {
		panic(fmt.Errorf("Error in remote calculation: %v", res))
	}
	return res[1]
}

func (self *ShaperCalibrate) _split_into_windows(x *maths.Ndarray, window_size, overlap int) *maths.Ndarray {
	// Memory-efficient algorithm to split an input 'x' into a series
	// of overlapping windows
	step_between_windows := window_size - overlap

	n_windows := (x.Shape[len(x.Shape)-1] - overlap) / step_between_windows
	shape := []int{window_size, n_windows}
	strides := []int{x.Strides[len(x.Strides)-1] / 32, step_between_windows * x.Strides[len(x.Strides)-1] / 32}

	return maths.AsStrided(x.Data[0], shape, strides)
}

func (self *ShaperCalibrate) _psd(x *maths.Ndarray, fs float64, nfft int) []float64 {
	// Calculate power spectral density (PSD) using Welch's algorithm
	window := maths.Kaiser(nfft, 6.)
	// Compensation for windowing loss
	var scale float64 = 0.0
	for i := 0; i < nfft; i++ {
		scale += math.Pow(window[i], 2)
	}
	scale = 1.0 / scale

	// Split into overlapping windows of size nfft
	overlap := nfft / 2
	x = self._split_into_windows(x, nfft, overlap)

	// First detrend, then apply windowing function
	x_mean := maths.Mean(x.Data, 0)
	for i := 0; i < len(x.Data); i++ {
		for j := 0; j < len(x.Data[0]); j++ {
			x.Data[i][j] = window[i] * (x.Data[i][j] - x_mean[j])
		}
	}
	// Calculate frequency response for each window using FFT
	result := maths.Rfft(x.Data, nfft, 0)
	conjugate := maths.Conjugate(result)
	for i := 0; i < len(result); i++ {
		for j := 0; j < len(result[i]); j++ {
			result[i][j] = conjugate[i][j] * result[i][j] * complex(scale/fs, 0)
		}
	}

	// For one-sided FFT output the response must be doubled, except
	// the last point for unpaired Nyquist frequency (assuming even nfft)
	// and the 'DC' term (0 Hz)
	for i := 1; i < len(result)-1; i++ {
		for j := 0; j < len(result[i]); j++ {
			result[i][j] *= 2
		}
	}

	// Welch's algorithm: average response over windows
	psd := make([]float64, nfft/2+1)
	for i := 0; i < len(result); i++ {
		for j := 0; j < len(result[i]); j++ {
			psd[i] += real(result[i][j])
		}
		psd[i] /= float64(len(result[i]))
	}

	return psd
}

func (self *ShaperCalibrate) Calc_freq_response(raw_values interface{}) *CalibrationData {
	var data *maths.Ndarray
	if raw_values == nil {
		return nil
	}
	//if isinstance(raw_values, np.ndarray):
	if reflect.TypeOf(raw_values) == reflect.TypeOf(maths.Ndarray{}) {
		data = raw_values.(*maths.Ndarray)
	} else {
		samples := raw_values.(IAclient).Get_samples()
		if samples == nil {
			return nil
		}
		data = maths.NewNdarray(samples)
	}

	N := data.Shape[0]
	T := data.Data[data.Len()-1][0] - data.Data[0][0]
	SAMPLING_FREQ := float64(N) / T
	// Round up to the nearest power of 2 for faster FFT
	M := 1 << bits.Len(uint(SAMPLING_FREQ*WINDOW_T_SEC-1))

	if N <= M {
		return nil
	}

	fx := maths.Rfftfreq(M, 1./SAMPLING_FREQ)

	dx := data.Slice(nil, 1)
	dy := data.Slice(nil, 2)
	dz := data.Slice(nil, 3)

	data = nil

	px := self._psd(dx, SAMPLING_FREQ, M)
	dx = nil

	py := self._psd(dy, SAMPLING_FREQ, M)
	dy = nil

	pz := self._psd(dz, SAMPLING_FREQ, M)
	dz = nil

	pxyz := make([]float64, len(px))
	for i := 0; i < len(px); i++ {
		pxyz[i] = px[i] + py[i] + pz[i]
	}
	return NewCalibrationData(fx, pxyz, px, py, pz)
}

func (self *ShaperCalibrate) Process_accelerometer_data(data IAclient) *CalibrationData {
	res := self.Background_process_exec(self.Calc_freq_response, []reflect.Value{reflect.ValueOf(data)})
	if res == nil {
		panic(fmt.Sprintf("Internal error processing accelerometer data %v", data))
	}
	return res.(*CalibrationData)
}

func (self *ShaperCalibrate) _estimate_shaper(shaper [2][]float64, test_damping_ratio float64, test_freqs []float64) []float64 {

	A, T := shaper[0], shaper[1]

	A_sum := 0.
	for _, val := range A {
		A_sum += val
	}
	inv_D := 1.0 / A_sum

	omega := make([]float64, len(test_freqs))
	damping := make([]float64, len(omega))
	omega_d := make([]float64, len(omega))
	for i, val := range test_freqs {
		omega[i] = 2.0 * math.Pi * val
		damping[i] = -(test_damping_ratio * omega[i])
		omega_d[i] = omega[i] * math.Sqrt(1.0-test_damping_ratio*test_damping_ratio)
	}
	_T := make([]float64, len(T))
	for i, val := range T {
		_T[i] = T[len(T)-1] - val
	}

	//W = A * np.exp(np.outer(-damping, (T[-1] - T)))
	W := maths.Exp(maths.Outer(damping, _T))
	for i := 0; i < len(W); i++ {
		for j := 0; j < len(W[i]); j++ {
			W[i][j] = A[j] * W[i][j]
		}
	}

	//S = W * np.sin(np.outer(omega_d, T))
	S := maths.Sin(maths.Outer(omega_d, T))
	for i := 0; i < len(S); i++ {
		for j := 0; j < len(S[i]); j++ {
			S[i][j] = W[i][j] * S[i][j]
		}
	}
	s_sum := maths.Sum2(S, 1)

	//C = W * np.cos(np.outer(omega_d, T))
	C := maths.Cos(maths.Outer(omega_d, T))
	for i := 0; i < len(C); i++ {
		for j := 0; j < len(C[i]); j++ {
			C[i][j] = W[i][j] * C[i][j]
		}
	}

	vals := []float64{}
	c_sum := maths.Sum2(C, 1)

	for i, v := range s_sum {
		vals = append(vals, math.Sqrt(math.Pow(c_sum[i], 2)+math.Pow(v, 2))*inv_D)
	}
	return vals
}

func (self *ShaperCalibrate) _estimate_remaining_vibrations(shaper [2][]float64, test_damping_ratio float64, freq_bins []float64, psd []float64) (float64, []float64) {
	vals := self._estimate_shaper(shaper, test_damping_ratio, freq_bins)
	// The input shaper can only reduce the amplitude of vibrations by
	// SHAPER_VIBRATION_REDUCTION times, so all vibrations below that
	// threshold can be igonred
	max := 0.
	for _, v := range psd {
		if v > max {
			max = v
		}
	}

	vibr_threshold := max / SHAPER_VIBRATION_REDUCTION

	remainingVibrations := 0.
	allVibrations := 0.
	for i, v := range psd {
		r := vals[i]*v - vibr_threshold
		if r <= 0 {
			r = 0
		}
		remainingVibrations += r

		a := v - vibr_threshold
		if a <= 0 {
			a = 0
		}
		allVibrations += a
	}

	return remainingVibrations / allVibrations, vals
}

func (self *ShaperCalibrate) _get_shaper_smoothing(shaper [2][]float64, accel float64, scv float64) float64 {
	halfAccel := accel * 0.5

	A, T := shaper[0], shaper[1]
	inv_D := 1.0 / maths.Sum1(A)
	n := len(T)
	// Calculate input shaper shift
	//ts = sum([A[i] * T[i] for i in range(n)]) * inv_D
	ts := 0.0
	for i := 0; i < n; i++ {
		ts += A[i] * T[i]
	}
	ts *= inv_D
	// Calculate offset for 90 and 180 degrees turn
	offset_90, offset_180 := 0.0, 0.0
	for i := 0; i < n; i++ {
		if T[i] >= ts {
			offset_90 += A[i] * (scv + halfAccel*(T[i]-ts)) * (T[i] - ts)
		}
		offset_180 += A[i] * halfAccel * math.Pow(T[i]-ts, 2)
	}
	offset_90 *= inv_D * math.Sqrt(2)
	offset_180 *= inv_D
	return math.Max(offset_90, offset_180)
}

func (self *ShaperCalibrate) Fit_shaper(shaper_cfg InputShaperCfg, offset_calibration_data *CalibrationData, max_smoothing float64) CalibrationResult {
	test_freqs := maths.Arange(shaper_cfg.Min_freq, MAX_SHAPER_FREQ, .2)
	freq_bins := offset_calibration_data.freq_bins
	psd := offset_calibration_data.psd_sum
	for i, v := range freq_bins {
		if v > MAX_FREQ {
			psd = psd[:i]
			freq_bins = freq_bins[:i]
			break
		}
	}

	var best_res CalibrationResult
	var res CalibrationResult
	var results []CalibrationResult
	for i := len(test_freqs) - 1; i >= 0; i-- {
		test_freq := test_freqs[i]
		shaper_vibrations := 0.0
		shaperVals := make([]float64, len(freq_bins))
		shaper := [2][]float64{}
		shaper[0], shaper[1] = shaper_cfg.Init_func(test_freq, DEFAULT_DAMPING_RATIO)
		shaperSmoothing := self._get_shaper_smoothing(shaper, 5000, 5.)
		if max_smoothing != 0 && shaperSmoothing > max_smoothing && best_res.name != "" {
			return best_res
		}
		// Exact damping ratio of the printer is unknown, pessimizing
		// remaining vibrations over possible damping values
		for _, dr := range TEST_DAMPING_RATIOS {
			vibrations, vals := self._estimate_remaining_vibrations(shaper, dr, freq_bins, psd)
			shaperVals = maths.Maximum(shaperVals, vals)
			if vibrations > shaper_vibrations {
				shaper_vibrations = vibrations
			}
		}
		maxAccel := self.find_shaper_max_accel(shaper)
		// The score trying to minimize vibrations, but also accounting
		// the growth of smoothing. The formula itself does not have any
		// special meaning, it simply shows good results on real user data
		shaperScore := shaperSmoothing * (math.Pow(shaper_vibrations, 1.5) + shaper_vibrations*0.2 + 0.01) // Formula for shaperScore

		res.name = shaper_cfg.Name
		res.freq = test_freq
		res.vals = shaperVals
		res.vibrs = shaper_vibrations
		res.smoothing = shaperSmoothing
		res.score = shaperScore
		res.max_accel = maxAccel
		results = append(results, res)

		if best_res.name == "" || best_res.vibrs > results[len(results)-1].vibrs {
			best_res = results[len(results)-1]
		}
	}
	selected := best_res
	for i := len(results) - 1; i >= 0; i-- {
		res := results[i]
		if res.vibrs < best_res.vibrs*1.1 && res.score < selected.score {
			// The current frequency is better for the shaper.
			selected = res
		}
	}
	return selected
}

func (self *ShaperCalibrate) _bisect(fun_c func(float64) bool) float64 {
	left, right := 1.0, 1.0
	for !fun_c(left) {
		right = left
		left *= 0.5
	}
	if right == left {
		for fun_c(right) {
			right *= 2.0
		}
	}
	for right-left > 1e-8 {
		middle := (left + right) * 0.5
		if fun_c(middle) {
			left = middle
		} else {
			right = middle
		}
	}
	return left
}

func (self *ShaperCalibrate) find_shaper_max_accel(shaper [2][]float64) float64 {
	targetSmoothing := 0.12
	maxAccel := self._bisect(func(testAccel float64) bool {
		return self._get_shaper_smoothing(shaper, testAccel, 5.) <= targetSmoothing
	})
	return maxAccel
}

func (self *ShaperCalibrate) find_best_shaper(calibration_data *CalibrationData, max_smoothing float64) (CalibrationResult, []CalibrationResult) {
	var best_shaper CalibrationResult
	all_shapers := make([]CalibrationResult, 0)
	for _, shaper_cfg := range INPUT_SHAPERS {
		//if shaper_cfg.name not in AUTOTUNE_SHAPERS:
		//	continue
		for _, s := range AUTOTUNE_SHAPERS {
			if shaper_cfg.Name == s {
				shaper := self.Background_process_exec(self.Fit_shaper, []reflect.Value{reflect.ValueOf(shaper_cfg), reflect.ValueOf(calibration_data), reflect.ValueOf(max_smoothing)}).(CalibrationResult)
				logger.Debugf("Fitted shaper '%s' frequency = %.1f Hz (vibrations = %.1f%%, smoothing ~= %.3f)", shaper.name, shaper.freq, shaper.vibrs*100., shaper.smoothing)
				logger.Debugf("To avoid too much smoothing with '%s', suggested max_accel <= %.0f mm/sec^2", shaper.name, math.Round(shaper.max_accel/100.)*100.)
				all_shapers = append(all_shapers, shaper)

				if best_shaper.name == "" || shaper.score*1.2 < best_shaper.score ||
					(shaper.score*1.05 < best_shaper.score && shaper.smoothing*1.1 < best_shaper.smoothing) {
					// Either the shaper significantly improves the score (by 20%),
					// or it improves the score and smoothing (by 5% and 10% resp.)
					best_shaper = shaper
				}
			}
		}
	}
	return best_shaper, all_shapers
}

func (self *ShaperCalibrate) Save_params(configfile *PrinterConfig, axis string, shaperName string, shaperFreq float64) {
	if axis == "xy" {
		self.Save_params(configfile, "x", shaperName, shaperFreq)
		self.Save_params(configfile, "y", shaperName, shaperFreq)
	} else {
		configfile.Set("input_shaper", "shaper_type_"+axis, shaperName)
		configfile.Set("input_shaper", "shaper_freq_"+axis, fmt.Sprintf("%.1f", shaperFreq))
	}
}

func (self *ShaperCalibrate) Save_calibration_data(output string, calibration_data interface{}, shapers interface{}) error {
	csvfile, err := os.Create(output)
	if err != nil {
		return fmt.Errorf("error creating file '%s': %s", output, err)
	}
	defer csvfile.Close()

	_, err = csvfile.WriteString("freq,psd_x,psd_y,psd_z,psd_xyz")
	if err != nil {
		return fmt.Errorf("error writing header to file '%s': %s", output, err)
	}

	if shapers != nil {
		for _, shaper := range shapers.([]CalibrationResult) {
			_, err = csvfile.WriteString(fmt.Sprintf(",%s(%.1f)", shaper.name, shaper.freq))
			if err != nil {
				return fmt.Errorf("error writing shaper header to file '%s': %s", output, err)
			}
		}
	}

	_, err = csvfile.WriteString("\n")
	if err != nil {
		return fmt.Errorf("error writing newline to file '%s': %s", output, err)
	}
	if calibration_data != nil {
		num_freqs := len(calibration_data.(*CalibrationData).freq_bins)
		for i := 0; i < num_freqs; i++ {
			if calibration_data.(*CalibrationData).freq_bins[i] >= MAX_FREQ {
				break
			}

			_, err = csvfile.WriteString(fmt.Sprintf("%.1f,%.3e,%.3e,%.3e,%.3e",
				calibration_data.(*CalibrationData).freq_bins[i],
				calibration_data.(*CalibrationData).psd_x[i],
				calibration_data.(*CalibrationData).psd_y[i],
				calibration_data.(*CalibrationData).psd_z[i],
				calibration_data.(*CalibrationData).psd_sum[i]))
			if err != nil {
				return fmt.Errorf("error writing calibration data to file '%s': %s", output, err)
			}

			if shapers != nil {
				for _, shaper := range shapers.([]CalibrationResult) {
					_, err = csvfile.WriteString(fmt.Sprintf(",%.3f", shaper.vals[i]))
					if err != nil {
						return fmt.Errorf("error writing shaper data to file '%s': %s", output, err)
					}
				}
			}

			_, err = csvfile.WriteString("\n")
			if err != nil {
				return fmt.Errorf("error writing newline to file '%s': %s", output, err)
			}
		}
	}
	return nil
}
