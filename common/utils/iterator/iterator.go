package iterator

// RangeInt RangeInt(5), RangeInt(1, 5), RangeInt(1, 5, 2)
func RangeInt(val int, params ...int) <-chan int {
	var (
		start = 0
		stop  = 0
		step  = 1
	)
	switch len(params) {
	case 0:
		stop = val
	case 1:
		start = val
		stop = params[0]
	case 2:
		start = val
		stop = params[0]
		step = params[1]
	default:
		panic("invalid RangeInt params")
	}

	if start > stop {
		panic("begin should less than end")
	}
	if step < 1 {
		panic("step should greater than 0")
	}

	ch := make(chan int)
	go func() {
		defer close(ch)
		for i := start; i < stop; i = i + step {
			ch <- i
		}
	}()
	return ch
}
