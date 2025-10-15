package greenlet

import (
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/sys"
	"reflect"
	"runtime"
	"runtime/debug"
	"strings"
	"sync"
)

var lock sync.Mutex
var greenlet sync.Map

type ReactorGreenlet struct {
	Name  string // thread name
	run   func(interface{}) interface{}
	Timer interface{}
	GId   uint64
	wg    sync.WaitGroup
	param interface{}
	ret   interface{}
	state string
	wgNum int
	exit  bool
}

func NewReactorGreenlet(run func(interface{}) interface{}) *ReactorGreenlet {
	self := ReactorGreenlet{}
	self.exit = false
	self.run = run
	self.Timer = nil
	self.Name = runtime.FuncForPC(reflect.ValueOf(run).Pointer()).Name()
	index := strings.Index(self.Name, ".")
	if index != -1 {
		self.Name = self.Name[index:]
	}
	self.wgNum = 0
	self.state = "start"
	return &self
}
func (self *ReactorGreenlet) goGreenlet() {
	defer func() {
		if err := recover(); err != nil {
			if !self.exit {
				s := string(debug.Stack())
				logger.Error("panic:", sys.GetGID(), err, s)
			}
		}
		lock.Lock()
		defer lock.Unlock()
		greenlet.Delete(self.GId) //delete greenlet when task end
		self.state = "end"
		self.GId = 0
	}()
	self.GId = sys.GetGID()
	greenlet.Store(self.GId, self)
	self.state = "run"
	if self.run != nil && !self.exit {
		self.ret = self.run(self.param)
	}
}
func (self *ReactorGreenlet) SwitchTo(waketime float64) float64 {
	lock.Lock()
	cSelf := Getcurrent()
	if cSelf == nil {
		//start
		cSelf = &ReactorGreenlet{}
		cSelf.GId = sys.GetGID()
		greenlet.Store(cSelf.GId, cSelf)
		cSelf.state = "run"
	}
	if self == cSelf { //to my self go out
		t := self.ret
		self.ret = waketime
		lock.Unlock()
		if t == nil {
			return constants.NEVER
		} else {
			return t.(float64)
		}
	}
	if self == nil {
		lock.Unlock()
		if cSelf.ret == nil {
			return constants.NEVER
		} else {
			return cSelf.ret.(float64)
		}
	}

	if self.GId == 0 {
		//run thread
		self.param = waketime
		go self.goGreenlet()
	} else {
		self.ret = waketime
		self.done()
	}
	cSelf.preWait()
	lock.Unlock()
	cSelf.wait()

	if cSelf.ret == nil {
		return constants.NEVER
	} else {
		return cSelf.ret.(float64)
	}
}
func (self *ReactorGreenlet) Switch() {
	self.goGreenlet()
}

func (self *ReactorGreenlet) done() {
	self.wgNum = self.wgNum - 1
	self.wg.Done()
}
func (self *ReactorGreenlet) preWait() {
	self.wgNum = self.wgNum + 1
	self.wg.Add(1)
	self.state = "wait"

	return
}
func (self *ReactorGreenlet) wait() {

	self.wg.Wait()
	if self.exit {
		//panic(fmt.Sprintf("end %d %s is ",cSelf.GId,cSelf.Name))
		panic("exit")
	}
	self.state = "run"
	//}
	return
}
func Getcurrent() *ReactorGreenlet {
	v, ok := greenlet.Load(sys.GetGID())
	if ok {
		return v.(*ReactorGreenlet)
	} else {
		return nil
	}

}

func Close() {
	lock.Lock()
	defer lock.Unlock()
	greenlet.Range(func(key, val interface{}) bool {
		greenlet.Delete(key)
		g := val.(*ReactorGreenlet)
		g.exit = true
		if g.wgNum == 1 {
			g.done()
		}
		return true
	})
	return
}
