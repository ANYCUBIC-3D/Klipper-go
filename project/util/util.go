// Low level unix utility functions
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package util

import (
	"k3c/common/logger"
	"net"
	"os"
	"os/signal"
	"strings"
	"syscall"
)

/*#####################################################################
# Low-level Unix commands
######################################################################
*/

func init() {
	fix_sigint()
	setup_python2_wrappers()
}

// Return the SIGINT interrupt handler back to the OS default
func fix_sigint() {
	signal.Reset(os.Interrupt)
}

// Set a file-descriptor as non-blocking
func Set_nonblock(fd int) {
	syscall.SetNonblock(fd, true)
}

// Clear HUPCL flag
func Clear_hupcl(fd uintptr) {
	//attrs := termios.tcgetattr(fd)
	//attrs[2] = attrs[2] &   (- ( termios.HUPCL + 1))
	//termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
}

// Support for creating a pseudo-tty for emulating a serial port
func create_pty(ptyname string) int {
	//mfd, sfd := pty.openpty()
	//err:=syscall.Unlink(ptyname)
	//if(err!=nil){
	//
	//}
	//filename := os.ttyname(sfd)
	//syscall.Chmod(filename, 0o660)
	//
	//syscall.Symlink(filename, ptyname)
	//set_nonblock(mfd)
	//old := termios.tcgetattr(mfd)
	//old[3] = old[3] &   - ( termios.ECHO + 1)
	//termios.tcsetattr(mfd, termios.TCSADRAIN, old)
	//return mfd
	return 0
}

/*#####################################################################
# Helper code for extracting mcu build info
######################################################################
*/

func Dump_file_stats(build_dir, filename string) {
	fname := strings.Join([]string{build_dir, filename}, string(os.PathSeparator))
	//fname := os.path.join(build_dir, filename)
	fileInfo, err := os.Stat(fname)
	mtime := fileInfo.ModTime()
	//mtime, err := os.path.getmtime(fname)
	fsize := fileInfo.Size()
	timestr := mtime.Format("2006-01-02 15:04:05")
	logger.Infof("Build file %s(%d): %s", fname, fsize, timestr)
	if err != nil {
		logger.Errorf("No build file %s", fname)
	}
}

// Try to log information on the last mcu build
func Dump_mcu_build() {

	//build_dir := os.path.join(os.path.dirname(__file__), "..")
	//// Try to log last mcu config
	//dump_file_stats(build_dir, ".config")
	//
	//f,err:= open(os.path.join(build_dir, ".config"), "r")
	//data := f.read(32*1024)
	//defer f.close()
	//log.Println("========= Last MCU build config =========\n%s"
	//"=======================", data)
	//if(err!=nil){
	//
	//}
	//// Try to log last mcu build version
	//dump_file_stats(build_dir, "out/klipper.dict")
	//
	//f ,err:= os.open(os.path.join(build_dir, "out/klipper.dict"), "r")
	//defer f.close()
	//data := f.read(32*1024)
	//
	//data = json.loads(data)
	//log.Println("Last MCU build version: %s", data.get("version", ""))
	//log.Println("Last MCU build tools: %s", data.get("build_versions", ""))
	//cparts := ["%s=%s" % (k, v) for k, v in data.get("config", {}).items()]
	//log.Println("Last MCU build config: %s", " ".join(cparts))
	//if(err!=nil){
	//
	//}
	//dump_file_stats(build_dir, "out/klipper.elf")
}

/*
#####################################################################
# Python2 wrapper hacks
######################################################################
*/
func setup_python2_wrappers() {
	//if sys.version_info.major >= 3 {
	//
	//	return
	//}
	//// Add module hacks so that common Python3 module imports work in Python2
	//import ConfigParser, Queue, io, StringIO, time
	//sys.modules["configparser"] = ConfigParser
	//sys.modules["queue"] = Queue
	//io.StringIO = StringIO.StringIO
	//time.process_time = time.clock
}

/*
#####################################################################
# General system and software information
######################################################################
*/
func get_cpu_info() string {

	//f ,err:= open('/proc/cpuinfo', 'r')
	//data := f.read()
	//defer f.close()
	//if(err!=nil) {
	//	log.Println("Exception on read /proc/cpuinfo: %s",
	//		traceback.format_exc())
	//	return "?"
	//}
	//
	//lines := [l.split(':', 1) for l in data.split('\n')]
	//lines = [(l[0].strip(), l[1].strip()) for l in lines if len(l) == 2]
	//core_count = [k for k, v in lines].count("processor")
	//model_name = dict(lines).get("model name", "?")
	//return fmt.Sprintf("%d core %s" ,core_count, model_name)
	return "?"
}

func get_version_from_file(K3c_src string) string {

	// f,err:= open(os.path.join(K3c_src, '.version')) as h:
	//
	//if(err!=nil){
	//
	//}
	//return h.read().rstrip()
	return "?"
}

func get_git_version(from_file bool) string {
	//K3c_src := os.path.dirname(__file__)
	//
	//// Obtain version info from "git" program
	//gitdir := os.path.join(K3c_src, "..")
	//prog := ('git', '-C', gitdir, 'describe', '--always',
	//	'--tags', '--long', '--dirty')
	//
	//process, err := subprocess.Popen(prog, stdout = subprocess.PIPE,
	//	stderr = subprocess.PIPE)
	//ver, err := process.communicate()
	//retcode := process.wait()
	//if retcode == 0 {
	//
	//	return str(ver.strip().decode())
	//} else {
	//
	//	log.Println("Error getting git version: %s", err)
	//}
	//if (err != nil) {
	//	log.Println("Exception on run: %s", traceback.format_exc())
	//}
	//if from_file {
	//
	//	return get_version_from_file(K3c_src)
	//}
	return "?"
}

func Fileno(conn_ net.Conn) int {
	conn, ok := conn_.(syscall.Conn)
	if !ok {
		panic("unsupported connection type")
	}

	rawConn, err := conn.SyscallConn()
	if err != nil {
		panic(err.Error())
	}

	var fd int
	controlErr := rawConn.Control(func(f uintptr) {
		fd = int(f)
	})

	if controlErr != nil {
		panic(controlErr.Error())
	}

	return fd
}

func Reverse(s []string) []string {
	newS := make([]string, len(s))
	for i, j := 0, len(s)-1; i <= j; i, j = i+1, j-1 {
		newS[i], newS[j] = s[j], s[i]
	}
	return newS
}

func StoreToFile(path string, data []byte) {
	go func() {
		tempname := path + ".tmp"
		f, _ := os.OpenFile(tempname, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0666)
		if f != nil {
			f.Write(data)
			// f.Sync()
			f.Close()
			os.Rename(tempname, path)
			// syscall.Sync()
		}
	}()
}
