package project

import (
	"container/list"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/reflects"
	"k3c/common/utils/sys"
	"k3c/project/chelper"
	"k3c/project/epoll"
	"k3c/project/greenlet"
	"k3c/project/queue"
	"k3c/project/util"
	"math"
	"reflect"
	"runtime"
	"sync"
	"syscall"
	"time"
	//"time"
	//"os"
)

type IReactor interface {
	Run() error
	Monotonic() float64
	End()
	Register_timer(callback func(float64) float64, waketime float64) *ReactorTimer
	Unregister_timer(timer_handler *ReactorTimer)
	Update_timer(timer_handler *ReactorTimer, waketime float64)
	Register_callback(callback func(interface{}) interface{}, eventtime float64) *ReactorCompletion
	Register_async_callback(callback func(argv interface{}) interface{}, waketime float64)
	Get_gc_stats() [3]float64

	Register_fd(fd int, read_callback, write_callback func(float64) interface{}) *ReactorFileHandler
	Unregister_fd(file_handler *ReactorFileHandler)
	Completion() *ReactorCompletion
	//Getcurrent() *greenlet.ReactorGreenlet
	Pause(float64) float64
	Set_fd_wake(file_handler *ReactorFileHandler, is_readable, is_writeable bool)
	Async_complete(completion *ReactorCompletion, result map[string]interface{})
	Mutex(is_locked bool) *ReactorMutex
}
type ReactorFileHandler struct {
	fd             int
	read_callback  func(eventtime float64) interface{}
	write_callback func(eventtime float64) interface{}
}

func NewReactorFileHandler(fd int, read_callback, write_callback func(eventtime float64) interface{}) *ReactorFileHandler {
	self := ReactorFileHandler{}
	self.fd = fd
	self.read_callback = read_callback
	self.write_callback = write_callback
	return &self
}

func (self ReactorFileHandler) fileno() interface{} {
	return self.fd
}

type ReactorTimer struct {
	callback func(eventtime float64) float64
	waketime float64
}

func NewReactorTimer(callback func(float64) float64, waketime float64) *ReactorTimer {
	self := ReactorTimer{}
	self.callback = callback
	self.waketime = waketime
	return &self
}

type ReactorCompletion struct {
	sentinel   interface{}
	reactor    IReactor
	result     interface{}
	waiting    *list.List
	createTime int64
}

func NewReactorCompletion(reactor IReactor) *ReactorCompletion {
	self := ReactorCompletion{}
	self.reactor = reactor
	self.result = self.sentinel
	self.waiting = list.New()
	self.createTime = time.Now().Unix()
	return &self
}
func (self *ReactorCompletion) Test() bool {
	return self.result != self.sentinel
	//return false
}

func (self *ReactorCompletion) Complete(result interface{}) {
	self.result = result

	for i := self.waiting.Front(); i != nil; i = i.Next() {
		wait := i.Value.(*greenlet.ReactorGreenlet)
		if wait == nil || wait.Timer == nil {
			continue
		}
		self.reactor.Update_timer(wait.Timer.(*ReactorTimer), constants.NOW)
	}
}
func (self *ReactorCompletion) Wait(waketime float64, waketime_result interface{}) interface{} {
	if self.result == nil {
		wait := greenlet.Getcurrent()
		elm := self.waiting.PushBack(wait)
		self.reactor.Pause(waketime)
		self.waiting.Remove(elm)
		if self.result == nil {
			return waketime_result
		}
	}
	return self.result
}

type ReactorCallback struct {
	reactor    IReactor
	timer      *ReactorTimer
	callback   func(interface{}) interface{}
	completion *ReactorCompletion
}

func NewReactorCallback(reactor IReactor, callback func(interface{}) interface{}, waketime float64) *ReactorCallback {
	self := ReactorCallback{}
	self.reactor = reactor
	self.timer = reactor.Register_timer(self.invoke, waketime)
	self.callback = callback
	self.completion = NewReactorCompletion(reactor)
	//logger.Debug("ReactorCallback start")
	return &self
}
func (self *ReactorCallback) invoke(eventtime float64) float64 {
	self.reactor.Unregister_timer(self.timer)
	//logger.Debug("2 ReactorCallback invoke ")
	res := self.invoke1(eventtime)
	self.completion.Complete(res)
	return constants.NEVER
}
func (self *ReactorCallback) invoke1(eventtime float64) interface{} {
	defer sys.CatchPanic()
	res := self.callback(eventtime)
	return res
}

type ReactorMutex struct {
	is_locked    bool
	reactor      IReactor
	next_pending bool
	queue        []*greenlet.ReactorGreenlet
	Lock         func()
	Unlock       func()
}

func NewReactorMutex(reactor IReactor, is_locked bool) *ReactorMutex {
	self := ReactorMutex{}
	self.reactor = reactor
	self.is_locked = is_locked
	self.next_pending = false
	self.queue = []*greenlet.ReactorGreenlet{}
	self.Lock = self.__enter__
	self.Unlock = self.__exit__
	return &self
}
func (self *ReactorMutex) Test() bool {
	return self.is_locked
}
func (self *ReactorMutex) __enter__() {
	if !self.is_locked {
		self.is_locked = true
		return
	}
	g := greenlet.Getcurrent()
	self.queue = append(self.queue, g)
	for {
		self.reactor.Pause(constants.NEVER)
		if self.next_pending && self.queue[0] == g {
			self.next_pending = false
			self.queue = self.queue[1:len(self.queue)]
			return
		}
	}
}
func (self *ReactorMutex) __exit__() {
	if len(self.queue) == 0 {
		self.is_locked = false
		return
	}
	self.next_pending = true
	self.reactor.Update_timer(self.queue[0].Timer.(*ReactorTimer), constants.NOW)
}

type SelectReactor struct {
	Gc_checking bool

	_process   bool
	monotonic1 float64

	_check_gc      bool
	_last_gc_times [3]float64

	_next_timer float64

	_pipe_fds    [2]interface{}
	_async_queue *queue.Queue

	_read_fds  *list.List
	_write_fds *list.List

	_g_dispatch    *greenlet.ReactorGreenlet
	_greenlets     *list.List
	_all_greenlets []*greenlet.ReactorGreenlet
	_timers        []*ReactorTimer
	write_fds      []interface{}
}

func NewSelectReactor(gc_checking bool) *SelectReactor {
	self := SelectReactor{}
	// Main code
	self._process = false
	self.monotonic1 = chelper.Get_ffi().Get_monotonic()
	// Python garbage collection
	self._check_gc = gc_checking
	self._last_gc_times = [3]float64{0, 0, 0}
	// Timers
	self._timers = []*ReactorTimer{}
	self._next_timer = constants.NEVER
	// Callbacks
	self._pipe_fds = [2]interface{}{}
	self._async_queue = queue.NewQueue()
	// File descriptors
	self._read_fds = list.New()
	self._write_fds = list.New()
	// Greenlets
	self._g_dispatch = nil
	self._greenlets = list.New()
	self._all_greenlets = []*greenlet.ReactorGreenlet{}
	return &self
}
func (self *SelectReactor) Get_gc_stats() [3]float64 {
	return self._last_gc_times
}

// Timers
func (self *SelectReactor) Update_timer(timer_handler *ReactorTimer, waketime float64) {
	timer_handler.waketime = waketime
	self._next_timer = math.Min(self._next_timer, waketime)
}
func (self *SelectReactor) Register_timer(callback func(float64) float64, waketime float64) *ReactorTimer {
	timer_handler := NewReactorTimer(callback, waketime)
	timers := make([]*ReactorTimer, len(self._timers))
	copy(timers, self._timers)
	timers = append(timers, timer_handler)
	self._timers = timers
	self._next_timer = math.Min(self._next_timer, waketime)
	return timer_handler
}
func (self *SelectReactor) Unregister_timer(timer_handler *ReactorTimer) {
	timer_handler.waketime = constants.NEVER
	timers := []*ReactorTimer{}
	for _, time := range self._timers {
		if time == timer_handler {
			continue
		}
		timers = append(timers, time)
	}
	self._timers = timers
}
func (self *SelectReactor) _check_timers(eventtime float64, busy bool) float64 {
	//defer sys.CatchPanic()
	if eventtime < self._next_timer {
		if busy {

			return 0.
		}
		if self._check_gc {

			//gi := gc.get_count()
			//if gi[0] >= 700 {
			//	// Reactor looks idle and gc is due - run it
			//	gc_level := 0
			//	if gi[1] >= 10 {
			//
			//		gc_level = 1
			//		if gi[2] >= 10 {
			//			gc_level = 2
			//		}
			//	}
			//	self._last_gc_times[gc_level] = eventtime
			//	gc.collect(gc_level)
			//	return 0.
			//}

		}
		return math.Min(1., math.Max(.001, self._next_timer-eventtime))
	}
	self._next_timer = constants.NEVER

	g_dispatch := self._g_dispatch
	for _, t := range self._timers {
		waketime := t.waketime
		if eventtime >= waketime {
			t.waketime = constants.NEVER
			logger.Debug("start timer", runtime.FuncForPC(reflect.ValueOf(t.callback).Pointer()).Name())
			waketime := t.callback(eventtime)
			t.waketime = waketime
			if g_dispatch != self._g_dispatch {
				self._next_timer = math.Min(self._next_timer, waketime)
				self._end_greenlet(g_dispatch)
				return 0.
			}

		}
		self._next_timer = math.Min(self._next_timer, waketime)
	}
	return 0.
}

// Callbacks and Completions
func (self *SelectReactor) Completion() *ReactorCompletion {
	return NewReactorCompletion(self)
}
func (self *SelectReactor) Register_callback(callback func(interface{}) interface{}, waketime float64) *ReactorCompletion {
	rcb := NewReactorCallback(self, callback, waketime)
	return rcb.completion
}

// Greenlets
func (self *SelectReactor) _sys_pause(waketime float64) float64 {
	// Pause using system sleep for when reactor not running
	delay := waketime - self.Monotonic()
	if delay > 0. {
		time.Sleep(1 * time.Second)
	}
	return self.Monotonic()
}
func (self *SelectReactor) Pause(waketime float64) float64 {
	g := greenlet.Getcurrent()
	if g != self._g_dispatch {
		if self._g_dispatch == nil {
			return self._sys_pause(waketime)
		}
		// Switch to _check_timers (via g.timer.callback return)
		return self._g_dispatch.SwitchTo(waketime)

	}
	// Pausing the dispatch greenlet - prepare a new greenlet to do dispatch
	g_next := greenlet.NewReactorGreenlet(self._dispatch_loop)
	self._all_greenlets = append(self._all_greenlets, g_next)
	//g_next.parents = g.parents
	g.Timer = self.Register_timer(g.SwitchTo, waketime)
	self._next_timer = constants.NOW
	//Switch to _dispatch_loop (via _end_greenlet or direct)
	eventtime := g_next.SwitchTo(waketime)
	// This eenlet activated from g.timer.callback (via _check_timers)
	return eventtime
}
func (self *SelectReactor) End() {
	self._process = false
}

func (self *SelectReactor) Monotonic() float64 {
	lib := chelper.Get_ffi()
	return lib.Get_monotonic()
}
func (self *SelectReactor) _setup_async_callbacks() {
	// Create and configure the pipe-ends.
	b := make([]int, 2)
	err := syscall.Pipe(b)
	if err != nil {
		logger.Error(err.Error())
	}
	self._pipe_fds[0] = b[0]
	self._pipe_fds[1] = b[1]
	self.Register_fd(b[0], self._got_pipe_signal, nil)
}

// Asynchronous (from another thread) callbacks and completions
func (self *SelectReactor) Register_async_callback(callback func(argv interface{}) interface{}, waketime float64) {
	data := []interface{}{
		callback, []interface{}{},
	}
	self._async_queue.Put_nowait(data)
	_, err := syscall.Write(self._pipe_fds[1].(int), []byte{'.'})
	if err != nil {
		logger.Error(err.Error())
	}
}
func (self *SelectReactor) Async_complete(completion *ReactorCompletion, result map[string]interface{}) {
	data := []interface{}{
		completion.Complete, []interface{}{result},
	}
	self._async_queue.Put_nowait(data)
	_, err := syscall.Write(self._pipe_fds[1].(int), []byte{'.'})
	if err != nil {
		logger.Error(err.Error())
	}
}
func (self *SelectReactor) _got_pipe_signal(eventtime float64) interface{} {
	bs := make([]byte, 4096)
	n, err := syscall.Read(self._pipe_fds[0].(int), bs)
	if err != nil {
		logger.Error(err.Error())
	}
	bs = bs[:n]
	for {
		var args interface{}
		var _func func(interface{})
		datas := self._async_queue.Get_nowait()
		if datas == nil {
			break
		}
		p, ok := datas.([]interface{})
		if !ok {
			continue
		}
		if len(p) == 1 {
			_func = p[0].(func(interface{}))
			args = p[1]
		} else if len(p) == 2 {
			_func = p[0].(func(interface{}))
		} else {
			continue
		}
		if _func != nil {
			_func(args)
		}
	}
	return nil
}

func (self *SelectReactor) Mutex(is_locked bool) *ReactorMutex {
	return NewReactorMutex(self, is_locked)
}

func (self *SelectReactor) Register_fd(fd int, read_callback, write_callback func(eventtime float64) interface{}) *ReactorFileHandler {
	file_handler := NewReactorFileHandler(fd, read_callback, write_callback)
	self.Set_fd_wake(file_handler, true, false)
	return file_handler
}
func (self *SelectReactor) Unregister_fd(file_handler *ReactorFileHandler) {
	for i := self._read_fds.Front(); i != nil; i = i.Next() {

		if i.Value == file_handler {
			self._read_fds.Remove(i)
			break
		}
	}
	for i := self._write_fds.Front(); i != nil; i = i.Next() {

		if i.Value == file_handler {
			self._write_fds.Remove(i)
			break
		}
	}
}
func (self *SelectReactor) Set_fd_wake(file_handler *ReactorFileHandler, is_readable, is_writeable bool) {
	//isIn:=false
	var old *list.Element
	for i := self._read_fds.Front(); i != nil; i = i.Next() {
		if i.Value == file_handler {
			old = i
			break
		}
	}
	if old != nil {
		if !is_readable {
			self._read_fds.Remove(old)
		}
	} else {
		if is_readable {
			self._read_fds.PushBack(file_handler)
		}
	}
	old = nil
	for i := self._write_fds.Front(); i != nil; i = i.Next() {
		if i.Value == file_handler {
			old = i
			break
		}
	}
	if old != nil {
		if !is_writeable {
			self._write_fds.Remove(old)
		}
	} else {
		if !is_writeable {
			self._write_fds.PushBack(file_handler)
		}
	}
}
func (self *SelectReactor) Run() error {
	if reflects.IsNil(self._pipe_fds[0]) {
		self._setup_async_callbacks()
	}
	defer greenlet.Close()
	self._process = true
	//g_next := greenlet.NewReactorGreenlet(self._dispatch_loop)
	//self._all_greenlets = append(self._all_greenlets, g_next)
	//g_next.SwitchTo(0)
	self._dispatch_loop(constants.NOW)
	return nil
}

//func (self *SelectReactor) monotonic() float64 {
//	lib := chelper.Get_ffi()
//	return lib.Get_monotonic()
//}

// Main loop
func (self *SelectReactor) _dispatch_loop(req interface{}) interface{} {

	g_dispatch := self._g_dispatch
	busy := true
	eventtime := self.Monotonic()
	for {
		if !self._process {
			break

		}
		timeout := self._check_timers(eventtime, busy)
		//logger.Debug(timeout)
		busy = false
		res := self._dispatch_loop4select(self._read_fds, self.write_fds, nil, timeout)
		////select .select (self._read_fds, self.write_fds, [], timeout)
		eventtime = self.Monotonic()
		for _, fd := range res[0] {
			busy = true
			fd.read_callback(eventtime)
			if g_dispatch != self._g_dispatch {
				self._end_greenlet(g_dispatch)
				eventtime = self.Monotonic()
				break
			}
		}
		for _, fd := range res[1] {
			busy = true

			fd.write_callback(eventtime)
			if g_dispatch !=
				self._g_dispatch {
				self._end_greenlet(g_dispatch)
				eventtime = self.Monotonic()
				break
			}
		}
	}
	//logger.Debug(eventtime)
	self._g_dispatch = nil
	return nil
}
func (self *SelectReactor) _dispatch_loop4select(_read_fds, write_fds interface{}, data []interface{}, timeout float64) [2][]ReactorFileHandler {
	ret := [2][]ReactorFileHandler{}
	return ret
}

func (self *SelectReactor) _end_greenlet(g_old *greenlet.ReactorGreenlet) {
	// Cache this greenlet for later use

	self._greenlets.PushBack(g_old)
	self.Unregister_timer(g_old.Timer.(*ReactorTimer))
	g_old.Timer = nil
	// Switch to _check_timers (via g_old.timer.callback return)
	self._g_dispatch.SwitchTo(constants.NEVER)
	// This greenlet reactivated from pause() - return to main dispatch loop
	self._g_dispatch = g_old
}

//func (self *SelectReactor) Getcurrent() *greenlet.ReactorGreenlet {
//	return self._g_dispatch
//}

type Poll struct {
	event string
	fd    string
}

type PollReactor struct {
	SelectReactor
	_poll chan interface{}
	_fds  map[string]*ReactorFileHandler
}

func NewPollReactor(gc_checking bool) *PollReactor {
	self := PollReactor{SelectReactor: *NewSelectReactor(gc_checking)}

	self._fds = map[string]*ReactorFileHandler{}
	//reactor._poll=nil
	return &self
}

// File descriptors
func (self *PollReactor) Register_fd(fd int, read_callback, write_callback func(float64) interface{}) *ReactorFileHandler {
	file_handler := NewReactorFileHandler(fd, read_callback, write_callback)
	self._fds["df"] = file_handler

	//self._poll.register(file_handler,
	//select .POLLIN |
	//	select .POLLHUP)
	return file_handler
}
func (self *PollReactor) Unregister_fd(file_handler *ReactorFileHandler) {
	//self._poll.unregister(file_handler)
	//	delete(self._fds,file_handler )
}
func (self *PollReactor) set_fd_wake(file_handler *ReactorFileHandler, is_readable, is_writeable bool) {

	//flags := ""
	if is_readable {

		//flags |=
		//select .POLLIN
	}
	if is_writeable {
		//flags |=
		//select .POLLOUT
	}
	//self._poll.modify(file_handler, flags)
}

// Main loop
func (self *PollReactor) _dispatch_loop(req interface{}) interface{} {

	g_dispatch := greenlet.Getcurrent()
	self._g_dispatch = g_dispatch
	busy := true
	eventtime := self.Monotonic()
	for {
		if !self._process {
			break
		}
		timeout := self._check_timers(eventtime, busy)

		busy = false
		select {
		case p, ok := (<-self._poll):
			if ok {
				poll, ok := p.(Poll)
				if ok {
					if poll.event == "in" || poll.event == "hup" {
						self._fds[poll.fd].read_callback(eventtime)
						if g_dispatch != self._g_dispatch {
							self._end_greenlet(g_dispatch)
							eventtime = self.Monotonic()
						}
					}
					if poll.event == "out" {
						self._fds[poll.fd].write_callback(eventtime)
						if g_dispatch != self._g_dispatch {
							self._end_greenlet(g_dispatch)
							eventtime = self.Monotonic()
						}
					}
					self._g_dispatch = nil
				} else {
					logger.Error("illage message\n")
				}
			} else {
				logger.Error("c3 is closed\n")
			}

		default:
			t := time.Duration(math.Ceil(timeout * 1000000000))
			//logger.Debug(t)
			time.Sleep(t)
		}
	}
	self._g_dispatch = nil
	return nil
}
func (self *PollReactor) Run() error {
	if reflects.IsNil(self._pipe_fds[0]) {
		self._setup_async_callbacks()
	}
	defer greenlet.Close()
	self._process = true
	g_next := greenlet.NewReactorGreenlet(self._dispatch_loop)
	self._all_greenlets = append(self._all_greenlets, g_next)
	g_next.Switch()
	return nil
}
func (self *PollReactor) Register_callback(callback func(interface{}) interface{}, waketime float64) *ReactorCompletion {
	rcb := NewReactorCallback(self, callback, waketime)
	return rcb.completion
}
func (self *PollReactor) Monotonic() float64 {
	lib := chelper.Get_ffi()
	return lib.Get_monotonic()
}
func (self *PollReactor) End() {
	self._process = false
}
func (self *PollReactor) Register_timer(callback func(float64) float64, waketime float64) *ReactorTimer {
	timer_handler := NewReactorTimer(callback, waketime)
	self._timers = append(self._timers, timer_handler)
	self._next_timer = math.Min(self._next_timer, waketime)
	return timer_handler
}
func (self *PollReactor) Unregister_timer(timer_handler *ReactorTimer) {
	timer_handler.waketime = constants.NEVER
	for i, time := range self._timers {
		if time == timer_handler {
			self._timers = self._timers[:copy(self._timers, self._timers[i:])]
			return
		}
	}
}

func (self *PollReactor) Update_timer(timer_handler *ReactorTimer, waketime float64) {
	timer_handler.waketime = waketime
	self._next_timer = math.Min(self._next_timer, waketime)
}

func (self *PollReactor) Completion() *ReactorCompletion {
	return NewReactorCompletion(self)
}

func (self *PollReactor) Pause(waketime float64) float64 {

	g := greenlet.Getcurrent()
	if g != self._g_dispatch {
		if self._g_dispatch == nil {
			return self._sys_pause(waketime)
		}
		// Switch to _check_timers (via g.timer.callback return)
		return self._g_dispatch.SwitchTo(waketime)
	}
	// Pausing the dispatch greenlet - prepare a new greenlet to do dispatch
	g_next := greenlet.NewReactorGreenlet(self._dispatch_loop)
	self._all_greenlets = append(self._all_greenlets, g_next)
	//g_next.parents = g.parents
	g.Timer = self.Register_timer(g.SwitchTo, waketime)
	self._next_timer = constants.NOW
	//Switch to _dispatch_loop (via _end_greenlet or direct)
	eventtime := g_next.SwitchTo(waketime)
	// This eenlet activated from g.timer.callback (via _check_timers)
	return eventtime
}

type EPollReactor struct {
	SelectReactor
	_epoll *epoll.EPoll
	_fds   sync.Map
}

func NewEPollReactor(gc_checking bool) *EPollReactor {
	self := EPollReactor{SelectReactor: *NewSelectReactor(gc_checking)}
	self._epoll = epoll.NewEPoll()
	return &self
}

// File descriptors
func (self *EPollReactor) Register_fd(fd int, read_callback, write_callback func(float64) interface{}) *ReactorFileHandler {
	file_handler := NewReactorFileHandler(fd, read_callback, write_callback)
	self._fds.Store(fd, file_handler)
	self._epoll.Register(fd, epoll.EPOLLIN|epoll.EPOLLHUP)
	return file_handler
}
func (self *EPollReactor) Unregister_fd(file_handler *ReactorFileHandler) {
	self._epoll.Unregister(file_handler.fd)
	self._fds.Delete(file_handler.fd)
}

func (self *EPollReactor) Set_fd_wake(file_handler *ReactorFileHandler, is_readable, is_writeable bool) {
	flags := epoll.EPOLLHUP
	if is_readable {
		flags |= epoll.EPOLLIN
	}
	if is_writeable {
		flags |= epoll.EPOLLOUT
	}
	self._epoll.Modify(file_handler.fd, uint32(flags))
}

// Main loop
func (self *EPollReactor) _dispatch_loop(req interface{}) interface{} {
	g_dispatch := greenlet.Getcurrent()
	self._g_dispatch = g_dispatch
	busy := true
	eventtime := self.Monotonic()
	for {
		//defer sys.CatchPanic()
		if !self._process {
			break
		}
		timeout := self._check_timers(eventtime, busy)

		busy = false
		res, nevents, err := self._epoll.Epoll(timeout)
		if err != nil && err != syscall.EINTR {
			logger.Error(err.Error())
		}
		eventtime = self.Monotonic()
		for i := 0; i < nevents; i++ {
			busy = true
			event := res[i].Events
			fd := int(res[i].Fd)
			if event&(epoll.EPOLLIN|epoll.EPOLLHUP) != 0 {
				_func, ok := self._fds.Load(fd)
				if ok {
					_func.(*ReactorFileHandler).read_callback(eventtime)
				}
				if g_dispatch != self._g_dispatch {
					self._end_greenlet(g_dispatch)
					eventtime = self.Monotonic()
					break
				}
			}
			if (event & epoll.EPOLLOUT) != 0 {
				_func, ok := self._fds.Load(fd)
				if ok {
					_func.(*ReactorFileHandler).write_callback(eventtime)
				}
				if g_dispatch != self._g_dispatch {
					self._end_greenlet(g_dispatch)
					eventtime = self.Monotonic()
					break
				}
			}

		}
	}
	self._g_dispatch = nil
	return nil
}
func (self *EPollReactor) Run() error {
	if reflects.IsNil(self._pipe_fds[0]) {
		self._setup_async_callbacks()
	}
	defer greenlet.Close()
	self._process = true
	g_next := greenlet.NewReactorGreenlet(self._dispatch_loop)
	self._all_greenlets = append(self._all_greenlets, g_next)
	g_next.Switch()
	self._all_greenlets = make([]*greenlet.ReactorGreenlet, 0)
	self._greenlets = list.New()
	syscall.Close(self._pipe_fds[0].(int))
	syscall.Close(self._pipe_fds[1].(int))
	return nil
}

func (self *EPollReactor) _check_timers(eventtime float64, busy bool) float64 {
	//defer sys.CatchPanic()
	if eventtime < self._next_timer {
		if busy {

			return 0.
		}
		if self._check_gc {

			//gi := gc.get_count()
			//if gi[0] >= 700 {
			//	// Reactor looks idle and gc is due - run it
			//	gc_level := 0
			//	if gi[1] >= 10 {
			//
			//		gc_level = 1
			//		if gi[2] >= 10 {
			//			gc_level = 2
			//		}
			//	}
			//	self._last_gc_times[gc_level] = eventtime
			//	gc.collect(gc_level)
			//	return 0.
			//}

		}
		return math.Min(1., math.Max(.001, self._next_timer-eventtime))
	}
	self._next_timer = constants.NEVER

	g_dispatch := self._g_dispatch
	for _, t := range self._timers {
		waketime := t.waketime
		if eventtime >= waketime {
			t.waketime = constants.NEVER
			waketime = t.callback(eventtime)
			//logger.Debug("check_timers", eventtime,waketime,runtime.FuncForPC(reflect.ValueOf(t.callback).Pointer()).Name(), " g_dispatch ", g_dispatch.GId, " gid", sys.GetGID())
			t.waketime = waketime
			if g_dispatch != self._g_dispatch {
				self._next_timer = math.Min(self._next_timer, waketime)
				self._end_greenlet(g_dispatch)
				return 0.
			}
		}
		self._next_timer = math.Min(self._next_timer, waketime)
	}
	return 0.
}
func (self *EPollReactor) Pause(waketime float64) float64 {
	g := greenlet.Getcurrent()
	if g != self._g_dispatch {
		if self._g_dispatch == nil {
			return self._sys_pause(waketime)
		}
		// Switch to _check_timers (via g.timer.callback return)
		return self._g_dispatch.SwitchTo(waketime)
	}
	// Pausing the dispatch greenlet - prepare a new greenlet to do dispatch
	var g_next *greenlet.ReactorGreenlet
	if self._greenlets.Len() != 0 {
		elm := self._greenlets.Front()
		g_next = self._greenlets.Remove(elm).(*greenlet.ReactorGreenlet)
	} else {
		g_next = greenlet.NewReactorGreenlet(self._dispatch_loop)
		self._all_greenlets = append(self._all_greenlets, g_next)
	}
	//g_next.Parent = g
	g.Timer = self.Register_timer(g.SwitchTo, waketime)
	self._next_timer = constants.NOW
	//Switch to _dispatch_loop (via _end_greenlet or direct)
	eventtime := g_next.SwitchTo(waketime)
	// This eenlet activated from g.timer.callback (via _check_timers)
	return eventtime
}

func (self *EPollReactor) Register_timer(callback func(float64) float64, waketime float64) *ReactorTimer {
	timer_handler := NewReactorTimer(callback, waketime)
	self._timers = append(self._timers, timer_handler)
	self._next_timer = math.Min(self._next_timer, waketime)
	return timer_handler
}

// Callbacks and Completions
func (self *EPollReactor) Completion() *ReactorCompletion {
	return NewReactorCompletion(self)
}
func (self *EPollReactor) Register_callback(callback func(interface{}) interface{}, waketime float64) *ReactorCompletion {
	rcb := NewReactorCallback(self, callback, waketime)
	return rcb.completion
}
func (self *EPollReactor) _setup_async_callbacks() {
	// Create and configure the pipe-ends.
	b := make([]int, 2)
	err := syscall.Pipe(b)
	if err != nil {
		logger.Error(err.Error())
	}
	self._pipe_fds[0] = b[0]
	self._pipe_fds[1] = b[1]
	util.Set_nonblock(b[0])
	util.Set_nonblock(b[1])
	self.Register_fd(b[0], self._got_pipe_signal, nil)
	//go self.got_pipe_signal()
}
func (self *EPollReactor) got_pipe_signal() {
	for {
		self._got_pipe_signal(constants.NOW)
	}
}
func (self *EPollReactor) _got_pipe_signal(eventtime float64) interface{} {
	bs := make([]byte, 4096)
	n, err := syscall.Read(self._pipe_fds[0].(int), bs)
	if err != nil {
		logger.Error(err.Error())
	}
	bs = bs[:n]
	for {

		var args []interface{}
		var _func func(interface{})
		datas := self._async_queue.Get_nowait()
		if datas == nil {
			break
		}
		p, ok := datas.([]interface{})
		if !ok {
			continue
		}
		if len(p) == 1 {
			_func = p[0].(func(interface{}))
			_func(args[0])
		} else if len(p) == 2 {
			/*defer func() {
				if err := recover(); err != nil {
					fmt.Println(p, err)
				}
			}()*/
			if _, ok := p[0].(func(interface{}) error); ok {
				p[0].(func(interface{}) error)(p[1].([]interface{}))
			} else {
				_func = p[0].(func(interface{}))
			}
			args = p[1].([]interface{})
			_func(args[0])
		} else if len(p) == 4 {
			datas := datas.([]interface{})
			_func := datas[0].(func(IReactor, func(interface{}) interface{}, float64) *ReactorCallback)
			//logger.Debug("4 _got_pipe_signal ", n, " ", runtime.FuncForPC(reflect.ValueOf(_func).Pointer()).Name())
			_func(datas[1].(IReactor), datas[2].(func(interface{}) interface{}), datas[3].(float64))
		} else {
			continue
		}
	}
	return nil
}
func (self *EPollReactor) Async_complete(completion *ReactorCompletion, result map[string]interface{}) {
	//logger.Debug("1 EPollReactor Async_complete", result)
	data := []interface{}{
		completion.Complete, []interface{}{result},
	}
	self._async_queue.Put_nowait(data)
	_, err := syscall.Write(self._pipe_fds[1].(int), []byte{'.'})
	if err != nil {
		logger.Error(err.Error())
	}
}
func (self *EPollReactor) _end_greenlet(g_old *greenlet.ReactorGreenlet) {
	// Cache this greenlet for later use
	//
	self._greenlets.PushBack(g_old)
	self.Unregister_timer(g_old.Timer.(*ReactorTimer))
	g_old.Timer = nil
	// Switch to _check_timers (via g_old.timer.callback return)
	self._g_dispatch.SwitchTo(constants.NEVER)
	// This greenlet reactivated from pause() - return to main dispatch loop
	self._g_dispatch = g_old
	//panic("exit")
}

// Asynchronous (from another thread) callbacks and completions
func (self *EPollReactor) Register_async_callback(callback func(argv interface{}) interface{}, waketime float64) {
	data := []interface{}{
		NewReactorCallback, self, callback, waketime}
	self._async_queue.Put_nowait(data)
	_, err := syscall.Write(self._pipe_fds[1].(int), []byte{'.'})
	if err != nil {
		logger.Error(err.Error())
	}
}
func (self *EPollReactor) Mutex(is_locked bool) *ReactorMutex {
	return NewReactorMutex(self, is_locked)
}
