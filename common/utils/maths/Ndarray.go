package maths

type Ndarray struct {
	Data    [][]float64
	Shape   []int
	Strides []int
}

func NewNdarray(data interface{}) *Ndarray {
	self := &Ndarray{}
	if _, ok := data.([][]float64); ok {
		self.Data = data.([][]float64)
	} else {
		panic("type error")
	}

	self.Shape = self.shape()
	self.Strides = self.strides()
	return self
}

func NewNdarray_Copy(data interface{}, Shape []int, Strides []int) *Ndarray {
	self := &Ndarray{}
	if _, ok := data.([][]float64); ok {
		self.Data = make([][]float64, len(data.([][]float64)))
		copy(self.Data, data.([][]float64))
	} else {
		panic("type error")
	}

	if len(Shape) == 0 {
		self.Shape = make([]int, len(self.shape()))
		self.Shape = self.shape()
	} else {
		self.Shape = make([]int, len(Shape))
		copy(self.Shape, Shape)
	}

	if len(Strides) == 0 {
		self.Strides = make([]int, len(self.strides()))
		self.Strides = self.strides()
	} else {
		self.Strides = make([]int, len(Strides))
		copy(self.Strides, Strides)
	}

	return self
}
func (self *Ndarray) Len() int {
	return len(self.Data)
}

func (self *Ndarray) Slice(row interface{}, col interface{}) *Ndarray {
	var result [][]float64

	if row == nil && col == nil {
		result = self.Data
	}
	if row == nil {
		if colIndex, ok := col.(int); ok {
			result = append(result, []float64{})
			for i := 0; i < len(self.Data); i++ {
				if colIndex < len(self.Data[i]) {
					result[0] = append(result[0], self.Data[i][colIndex])
				}
			}
		}
		self.Strides = []int{self.Strides[0]}
		self.Shape = []int{self.Shape[0]}
	} else if col == nil {
		if rowIndex, ok := row.(int); ok {
			if rowIndex < len(self.Data) {
				result = append(result, self.Data[rowIndex])
			}
		}
		self.Strides = []int{self.Strides[1]}
		self.Shape = []int{self.Shape[1]}
	} else {
		if rowIndex, ok := row.(int); ok {
			if colIndex, ok := col.(int); ok {
				if rowIndex < len(self.Data) && colIndex < len(self.Data[rowIndex]) {
					result = append(result, []float64{self.Data[rowIndex][colIndex]})
				}
			}
		}
	}
	//self.Data = append([][]float64{}, result...)
	//return self
	return NewNdarray_Copy(result, self.Shape, self.Strides)

}
func (self *Ndarray) Slice1(row interface{}, col interface{}) *Ndarray {
	var result [][]float64

	if row == nil && col == nil {
		result = self.Data
	}
	if row == nil {
		if colIndex, ok := col.(int); ok {
			result = append(result, []float64{})
			for i := 0; i < len(self.Data); i++ {
				if colIndex < len(self.Data[i]) {
					result[0] = append(result[0], self.Data[i][colIndex])
				}
			}
		}
		self.Strides = []int{self.Strides[0]}
		self.Shape = []int{self.Shape[0]}
	} else if col == nil {
		if rowIndex, ok := row.(int); ok {
			if rowIndex < len(self.Data) {
				result = append(result, self.Data[rowIndex])
			}
		}
		self.Strides = []int{self.Strides[1]}
		self.Shape = []int{self.Shape[1]}
	} else {
		if rowIndex, ok := row.(int); ok {
			if colIndex, ok := col.(int); ok {
				if rowIndex < len(self.Data) && colIndex < len(self.Data[rowIndex]) {
					result = append(result, []float64{self.Data[rowIndex][colIndex]})
				}
			}
		}
	}
	self.Data = append([][]float64{}, result...)
	return self
	//return NewNdarray_Copy(result)

}
func (self *Ndarray) shape() []int {
	shape_arr := []int{len(self.Data)}
	maxLen := 0
	for i := 0; i < len(self.Data); i++ {
		if maxLen < len(self.Data[i]) {
			maxLen = len(self.Data[i])
		}
	}
	shape_arr = append(shape_arr, maxLen)
	return shape_arr
}

func (self *Ndarray) strides() []int {
	shape_arr := self.shape()
	shape_arr[0] = 4 * shape_arr[1] * 2
	shape_arr[1] = 4 * 2
	return shape_arr
}
