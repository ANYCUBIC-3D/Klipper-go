package epoll

import (
	"log"
	"math"
	"runtime"
	"syscall"
)

const (
	EPOLLIN  = 1
	EPOLLHUP = 16
	EPOLLOUT = 4
)

type EPoll struct {
	epfd int
}

func NewEPoll() *EPoll {
	self := EPoll{}
	epfd, e := syscall.EpollCreate1(0)
	if e != nil {
		log.Print("epoll_create1: ", e)
	}
	self.epfd = epfd
	runtime.SetFinalizer(&self, func(data *EPoll) { data.close() })

	return &self
}

func (self *EPoll) close() {
	syscall.Close(self.epfd)
}

func (self *EPoll) Register(fd int, op uint32) error {
	event := syscall.EpollEvent{}
	event.Events = op
	event.Fd = int32(fd)
	err := syscall.EpollCtl(self.epfd, syscall.EPOLL_CTL_ADD, fd, &event)
	return err
}

func (self *EPoll) Unregister(fd int) error {
	event := syscall.EpollEvent{}
	event.Fd = int32(fd)
	err := syscall.EpollCtl(self.epfd, syscall.EPOLL_CTL_DEL, fd, &event)
	return err
}
func (self *EPoll) Modify(fd int, op uint32) error {
	event := syscall.EpollEvent{}
	event.Events = op
	event.Fd = int32(fd)
	err := syscall.EpollCtl(self.epfd, syscall.EPOLL_CTL_MOD, fd, &event)
	return err
}

func (self *EPoll) Epoll(timeout float64) ([]syscall.EpollEvent, int, error) {
	t := math.Ceil(timeout * 1000)
	events := make([]syscall.EpollEvent, 128)
	nevents, err := syscall.EpollWait(self.epfd, events, int(t))
	return events, nevents, err
}
