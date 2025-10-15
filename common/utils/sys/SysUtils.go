package sys

import (
	"errors"
	"fmt"
	"k3c/common/logger"
	"os"
	"path"
	"runtime/debug"
	"strconv"
	"strings"

	"github.com/petermattis/goid"
)

func GetGID() uint64 {
	id := goid.Get()
	return uint64(id)
}

func GetCpuInfo() string {
	cpuInfoFilebytes, err := os.ReadFile("/proc/cpuinfo")
	if err != nil {
		return "?"
	}

	var coreCount int
	var modelName string
	cpuInfoLines := strings.Split(string(cpuInfoFilebytes), "\n")
	for _, line := range cpuInfoLines {
		if strings.Index(line, ":") == -1 {
			continue
		}
		lines := strings.Split(line, ":")
		fieldName := strings.TrimSpace(lines[0])
		if fieldName == "processor" {
			coreCount++
		} else if fieldName == "model name" {
			modelName = strings.TrimSpace(lines[1])
		}
	}
	return fmt.Sprintf("%d core %s", coreCount, modelName)
}

func GetSoftwareVersion() string {
	//info, _ := cpu.Info()
	//fmt.Println(info)
	return ""
}

var timeoutPrinted bool
var serialConnectClosed bool

func CatchPanic() {
	if err := recover(); err != nil {
		msg, ok := err.(string)
		s := string(debug.Stack())
		if ok {
			if "exit" == msg {
				panic(msg)
			}

			if strings.Contains(err.(string), "Timeout on wait for") {
				if !timeoutPrinted {
					logger.Error("panic:", GetGID(), err, s)
					timeoutPrinted = true
				}
				return
			} else if strings.Contains(err.(string), "Serial connection closedmap") {
				if !serialConnectClosed {
					logger.Error("panic:", GetGID(), err, s)
					serialConnectClosed = true
				}
				return
			}
		} else {
			logger.Error("panic:", GetGID(), err, s)
		}

	}
}

// PidStatInfo
// @see https://github.com/struCoder/pidusage
type PidStatInfo struct {
	Utime  float64 // CPU time spent in user code, measured in clock ticks
	Stime  float64 // CPU time spent in kernel code, measured in clock ticks
	Cutime float64 // Waited-for children's CPU time spent in user code (in clock ticks)
	Cstime float64 // Waited-for children's CPU time spent in kernel code (in clock ticks)
	Rss    float64 // Resident Set Size
}

type LoadavgInfo struct {
	Load1  float64 // the system load averages for the past 1 minute
	Load5  float64 // the system load averages for the past 5 minutes
	Load15 float64 // the system load averages for the past 15 minutes
}

func PidStat(pid ...int) (stat PidStatInfo, err error) {
	var statfile = "/proc/self/stat"
	if len(pid) > 0 {
		statfile = path.Join("/proc", strconv.Itoa(pid[0]), "stat")
	}
	procStatFileBytes, err := os.ReadFile(statfile)
	if err != nil {
		return stat, err
	}

	splitAfter := strings.SplitAfter(string(procStatFileBytes), ")")
	if len(splitAfter) == 0 || len(splitAfter) == 1 {
		return stat, errors.New("Can't find process info from " + statfile)
	}
	infos := strings.Split(splitAfter[1], " ")
	stat = PidStatInfo{
		Utime:  parseFloat(infos[12]),
		Stime:  parseFloat(infos[13]),
		Cutime: parseFloat(infos[14]),
		Cstime: parseFloat(infos[15]),
		Rss:    parseFloat(infos[22]),
	}

	return stat, nil
}

func Loadavg() (info LoadavgInfo, err error) {
	uptimeFileBytes, err := os.ReadFile(path.Join("/proc", "loadavg"))
	if err != nil {
		return
	}
	infos := strings.Split(string(uptimeFileBytes), " ")
	if len(infos) < 3 {
		return info, fmt.Errorf("loadavg info invalid: %s", string(uptimeFileBytes))
	}

	info = LoadavgInfo{
		Load1:  parseFloat(infos[0]),
		Load5:  parseFloat(infos[1]),
		Load15: parseFloat(infos[2]),
	}
	return info, nil
}

func parseFloat(val string) float64 {
	floatVal, _ := strconv.ParseFloat(val, 64)
	return floatVal
}

// _get_status_info     map[string]interface{}
func DeepCopyMap(m map[string]interface{}) map[string]interface{} {
	if m == nil {
		return nil
	}
	copyMap := make(map[string]interface{}, len(m))
	for k, v := range m {
		if nestedMap, ok := v.(map[string]interface{}); ok {
			copyMap[k] = DeepCopyMap(nestedMap)
		} else {
			copyMap[k] = v
		}
	}
	return copyMap
}
