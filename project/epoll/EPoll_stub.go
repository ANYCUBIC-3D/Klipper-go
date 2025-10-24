//go:build !linux
// +build !linux

package epoll

import "errors"

const (
	EPOLLIN  = 1
	EPOLLHUP = 16
	EPOLLOUT = 4
)

var errUnsupported = errors.New("epoll: unsupported on this platform")

type Event struct {
	Events uint32
	Fd     int32
}

type EPoll struct{}

func NewEPoll() *EPoll {
	return &EPoll{}
}

func (e *EPoll) close() {}

func (e *EPoll) Register(fd int, op uint32) error {
	return errUnsupported
}

func (e *EPoll) Unregister(fd int) error {
	return errUnsupported
}

func (e *EPoll) Modify(fd int, op uint32) error {
	return errUnsupported
}

func (e *EPoll) Epoll(timeout float64) ([]Event, int, error) {
	return nil, 0, errUnsupported
}
