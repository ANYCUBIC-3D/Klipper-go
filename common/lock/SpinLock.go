package lock

import (
	"runtime"
	"sync/atomic"
)

type SpinLock uint32

const maxBackOff = 32

func (sl *SpinLock) Lock() {
	backoff := 1
	for !atomic.CompareAndSwapUint32((*uint32)(sl), 0, 1) {
		for i := 0; i < backoff; i++ {
			runtime.Gosched()
		}
		if backoff < maxBackOff {
			backoff <<= 1
		}

	}
}

func (sl *SpinLock) UnLock() {
	atomic.CompareAndSwapUint32((*uint32)(sl), 1, 0)
}
