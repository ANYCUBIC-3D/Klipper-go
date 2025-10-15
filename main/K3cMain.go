package main

import (
	"k3c/common/logger"
	"k3c/common/utils/sys"
	"k3c/project"
	"os"
	"time"
)

func main() {
	logger.Debugf("main thread %d running", sys.GetGID())
	project.ModuleK3C()
	k3c := project.NewK3C()
	k3c.Main()
	for {
		time.Sleep(1 * time.Second)
	}
	os.Exit(0)
	return
}
