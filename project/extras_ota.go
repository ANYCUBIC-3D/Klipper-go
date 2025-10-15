package project

import (
	"errors"
	"fmt"
	"hash/crc32"
	"io"
	"k3c/common/logger"
	"k3c/common/utils/cast"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
)

const (
	OTA_UPGRADING_START = 1 << iota
	OTA_UPGRADING_ERASEING
	OTA_UPGRADING_ERASED
	OTA_UPGRADING_WRITING
	OTA_UPGRADING_WRITED
	OTA_UPGRADING_FINISH
	OTA_UPGRADING_ERROR
	OTA_UPGRADING_RESERVED
)
const (
	EC_OD_CHECK_ERR = 20 + iota
	EC_OD_WRITE_ERR
	EC_WRITE_ERR
	EC_ERASE_ERR
	EC_CRC_ERR
)
const (
	FW_FILE_NAME = `^(?:(?:mcu|nozzle)_)?firmware_v(\d{1,3}\.\d{1,3}\.\d{1,3})_(\d{8})\.bin$` //`^firmware_v\d+\.\d+\.\d+_\d{8}\.bin$`
)

type Ota struct {
	printer              *Printer
	oid                  int
	mcu                  *MCU
	mcu_name             string
	gcode                *GCodeDispatch
	send_ota_data_cmd    *CommandWrapper
	start_ota_cmd        *CommandWrapper
	ota_erase_cmd        *CommandWrapper
	query_version_cmd    *CommandQueryWrapper
	state                string
	progress             float64
	fw_update_path       string
	fw_max_size          int
	fw_sector_size       int
	fw_ota_sector_offset int
	check_reg            *regexp.Regexp
	timeout_timer        *ReactorTimer
	is_timeout           bool
	reactor              IReactor
	local_info           interface{}
}

func NewOta(config *ConfigWrapper) *Ota {
	self := new(Ota)
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	name := config.Get_name()
	mcu := Get_printer_mcu(self.printer, name)
	if mcu != nil {
		self.mcu = mcu
		self.mcu_name = name
	}
	self.mcu.Register_config_callback(self.Build_config)

	self.fw_update_path = ""
	self.fw_max_size = config.Getint("fw_max_size", 40960, 0, 0, true)
	self.fw_sector_size = config.Getint("fw_sector_size", 8192, 0, 0, true)
	self.fw_ota_sector_offset = config.Getint("fw_ota_sector_offset", 22, 0, 0, true)

	self.check_reg = regexp.MustCompile(FW_FILE_NAME)
	self.printer.Register_event_handler("project:ready", self.handle_ready)

	self.gcode = MustLookupGcode(self.printer)
	self.gcode.Register_mux_command("OTA_START", "MCU", self.mcu_name, self.cmd_OTA_START, cmd_OTA_START_help)

	self.state = "standby"
	self.is_timeout = false
	self.timeout_timer = nil
	return self
}

func (self *Ota) Build_config() {
	self.oid = self.mcu.Create_oid()
	cmdqueue := self.mcu.Alloc_command_queue()

	//Setup config
	self.mcu.Add_config_cmd(fmt.Sprintf("config_ota oid=%d ", self.oid),
		false, false)

	self.send_ota_data_cmd, _ = self.mcu.Lookup_command(
		"ota_transfer_response oid=%c offset=%hu data=%*s", cmdqueue)
	self.ota_erase_cmd, _ = self.mcu.Lookup_command(
		"ota_erase oid=%c offset=%u is_transfer=%c", cmdqueue)
	self.mcu.Register_response(self._handle_ota_transfer, "ota_transfer", self.oid)
	self.mcu.Register_response(self._handle_ota_status, "ota_status", self.oid)
	self.mcu.Register_response(self._handle_ota_local_info, "ota_local_info", self.oid)

	self.start_ota_cmd, _ = self.mcu.Lookup_command(
		"ota_start oid=%c crc32=%u version_major=%c version_minor=%c version_patch=%c", cmdqueue)

	self.query_version_cmd = self.mcu.Lookup_query_command(
		"query_ota_local_info oid=%c",
		"ota_local_info oid=%c flag=%c crc32=%u version_major=%c version_minor=%c version_patch=%c", self.oid, nil, false)
}

func (self *Ota) handle_ready([]interface{}) error {
	version := self.mcu.Get_status(0)["mcu_version"].(string)
	update_version := self.get_update_version()
	if update_version != "" {
		if version != "" {
			if update_version == version {
				self.ota_update_state("update_success")
			} else {
				self.ota_update_state("update_failed")
			}
		}
	}
	return nil
}

func (self *Ota) _handle_timeout(float64) float64 {
	if self.state == "eraseing" || self.state == "writing" || self.state == "transfer_finish" {
		self.is_timeout = true
		self.reactor.Unregister_timer(self.timeout_timer)
		self.timeout_timer = nil
	}

	return NEVER_TIME
}

func (self *Ota) _handle_ota_transfer(params map[string]interface{}) error {
	if self.is_timeout {
		return errors.New("ota transfer timeout")
	}
	logger.Debugf("ota_transfer handle")
	offset, count := cast.ToInt(params["offset"]), cast.ToInt(params["count"])
	f, err := os.Open(self.fw_update_path)
	if err != nil {
		self.state = "open file error"
		self.send_ota_data_cmd.Send([]interface{}{int64(self.oid)}, 0, 0)
		return err
	}
	defer f.Close()

	tosend := []interface{}{int64(self.oid)}

	f.Seek(int64(offset), io.SeekStart)
	var bytesRead = make([]byte, count)
	n, err := f.Read(bytesRead)
	if err != nil && err == io.EOF {
		tosend = append(tosend, offset)
		tosend = append(tosend, []int64{})
		self.send_ota_data_cmd.Send(tosend, 0, 0)
		self.reactor.Update_timer(self.timeout_timer, self.reactor.Monotonic()+5.0)
		self.state = "transfer_finish"
		self.progress = 1
		logger.Debugf("ota transfer_finish")
		return nil
	}

	tosend = append(tosend, offset+n)
	dat := make([]int64, 0, n)
	for _, d := range bytesRead[:n] {
		dat = append(dat, int64(d))
	}
	fs, err := f.Stat()
	if err != nil {
		self.state = "file stat error"
		return err
	}

	tosend = append(tosend, dat)
	if int64(offset+n) == fs.Size() && dat[len(dat)-1] == 10 {
		dat = dat[:len(dat)-1]
	}

	self.send_ota_data_cmd.Send(tosend, 0, 0)
	self.state = "writing"

	self.reactor.Update_timer(self.timeout_timer, self.reactor.Monotonic()+5.0)

	self.progress = math.Round(float64(offset+n)/float64(fs.Size())*100) / 100

	return nil
}

func (self *Ota) _handle_ota_status(params map[string]interface{}) error {
	var status = cast.ToInt(params["status"])
	/*
		START       1
		ERASEING    2
		ERASED      4
		WRITING     8
		WRITED      16
		FINISH      32
		ERROR       64
		RESERVED    128
	*/
	logger.Debug("status:", self.state)
	var err_code = cast.ToInt(params["err_code"])
	switch status {
	case OTA_UPGRADING_FINISH:
		self.state = "upgrading_restart"
	case OTA_UPGRADING_START:
		self.state = "start"
	case OTA_UPGRADING_ERASEING:
		self.state = "eraseing"
	case OTA_UPGRADING_ERASED:
		self.state = "erase finish"
	case OTA_UPGRADING_WRITING:
		self.state = "writing"
	case OTA_UPGRADING_WRITED:
		self.state = "write finish"
	case OTA_UPGRADING_ERROR:
		switch err_code {
		case EC_CRC_ERR:
			self.state = "CRC check error"
		case EC_OD_CHECK_ERR:
			self.state = "ota data check error"
		case EC_OD_WRITE_ERR:
			self.state = "ota data write error"
		case EC_WRITE_ERR:
			self.state = "flash write error"
		case EC_ERASE_ERR:
			self.state = "flash erase error"
		}
	}
	return nil
}

func (self *Ota) _handle_ota_local_info(params map[string]interface{}) error {
	self.local_info = params
	return nil
}

func (self *Ota) handle_ota_version_dump(web_request *WebRequest) (interface{}, error) {
	web_request.Send(self.Get_Status(0))
	return nil, nil
}

const cmd_OTA_START_help = "start ota upgrade"

func (self *Ota) cmd_OTA_START(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	update_path := gcmd.Get("UPDATE_PATH", "./firmware_v0.1.1_20230712.bin", "", nil, nil, nil, nil)
	match := self.check_reg.FindStringSubmatch(filepath.Base(update_path))
	if match != nil {
		fmt.Println("verison:", match[1])
		fmt.Println("date:", match[2])
	} else {
		return fmt.Errorf("firmware file format error")
	}
	//Check if MCU upgraded wrong other mcu firmware
	if !strings.Contains(update_path, strings.Split(self.mcu_name, "_")[0]) {
		if strings.Split(update_path, "_")[0] != "firmware" {
			return fmt.Errorf("The firmware does not support this mcu")
		}
	}

	self.fw_update_path = update_path
	self.state = "ota start"
	self.is_timeout = false

	otaFile, err := os.OpenFile(self.fw_update_path, os.O_RDONLY, os.ModePerm)
	if err != nil {
		self.state = "open file error"
		logger.Error(err)
		return err
	}
	defer otaFile.Close()
	byteArr, err := io.ReadAll(otaFile)
	if err != nil {
		self.state = "read file error"
		logger.Error(err)
		return err
	}
	byteArr = byteArr[:len(byteArr)-9]
	crc32 := crc32.ChecksumIEEE(byteArr) & 0xffffffff
	version := "\"" + strings.ToLower("v"+match[1]) + "\""
	versionArr, err := self.validateVersion(version, crc32)
	if err != nil {
		self.state = "not_need_upgrad"
		logger.Error(err)
		return nil
	}
	logger.Debugf("crc32: %v version0:%v version1:%v version2:%v", crc32, versionArr[0], versionArr[1], versionArr[2])
	if self.timeout_timer == nil {
		self.timeout_timer = self.reactor.Register_timer(self._handle_timeout, self.reactor.Monotonic()+10.0)
	}
	self.start_ota_cmd.Send([]int64{int64(self.oid), int64(crc32), int64(versionArr[0]), int64(versionArr[1]), int64(versionArr[2])}, 0, 0)
	self.state = "upgrading_download"
	self.progress = 0.01

	eventtime := 0.0
	if self.ota_erase_cmd != nil {
		self.ota_erase_cmd.Send([]int64{int64(self.oid), int64(0), int64(1)}, 0, 0)
	}

	for {
		gcmd.Respond_raw(fmt.Sprintf("progress = %.1f%%", self.progress*100))
		eventtime = self.reactor.Monotonic()
		eventtime = self.reactor.Pause(eventtime + 0.1)
		if self.printer.Is_shutdown() {
			break
		} else if self.state == "upgrading_restart" {
			return nil
		} else if strings.Index(self.state, "error") != -1 {
			return errors.New(self.state)
		} else if self.state == "writing" && self.is_timeout {
			return errors.New("ota transfer timeout")
		} else if self.state == "transfer_finish" && self.is_timeout {
			return errors.New("MCU responds to ota transfer finish timeout")
		} else if self.is_timeout {
			return errors.New("wait for mcu erase flash timeout")
		}
	}
	return nil
}

func (self *Ota) Get_Status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"state":    self.state,
		"progress": self.progress,
		"version":  self.mcu._get_status_info["mcu_version"],
	}
}
func (self *Ota) out_put_version_file(newVersion string) error {
	vesion_fd, err := os.OpenFile("/tmp/version", os.O_CREATE|os.O_WRONLY|os.O_APPEND, os.ModePerm)

	if err != nil {
		return err
	}
	defer vesion_fd.Close()

	_, err = io.WriteString(vesion_fd, self.mcu._name+":"+newVersion+"\n")
	if err != nil {
		return err
	}
	return nil
}

func (self *Ota) validateVersion(newVersion string, crc32 uint32) ([]int, error) {

	if self.local_info == nil {
		self.local_info = self.query_version_cmd.Send([]interface{}{int64(self.oid)}, 0, 0)
	}

	if newVersion == self.mcu._get_status_info["mcu_version"].(string) {
		err := self.out_put_version_file(newVersion)
		if err != nil {
			return nil, err
		}
		return nil, errors.New("version match consistenly,not need upgrad")
	}

	if int64(crc32) == self.local_info.(map[string]interface{})["crc32"].(int64) {
		err := self.out_put_version_file(newVersion)
		if err != nil {
			return nil, err
		}
		return nil, errors.New("CRC32 match consistenly,not need upgrad")
	}

	err := self.out_put_version_file(newVersion)
	if err != nil {
		return nil, err
	}
	newVersion = strings.ReplaceAll(newVersion, "\"", "")
	versionStrArr := strings.Split(newVersion, "v")
	versionStrArr = strings.Split(versionStrArr[1], ".")
	v1, err := strconv.Atoi(versionStrArr[0])
	if err != nil {
		return nil, err
	}
	v2, err := strconv.Atoi(versionStrArr[1])
	if err != nil {
		return nil, err
	}
	v3, err := strconv.Atoi(versionStrArr[2])
	if err != nil {
		return nil, err
	}

	return []int{v1, v2, v3}, nil
}

func (self *Ota) get_update_version() string {
	vesion_fd, err := os.OpenFile(fmt.Sprintf("/tmp/%s_version", self.mcu_name), os.O_RDONLY, os.ModePerm)
	if err != nil {
		return ""
	}
	defer vesion_fd.Close()
	rbyte, rerr := io.ReadAll(vesion_fd)
	if rerr != nil {
		return ""
	}
	line := strings.Split(string(rbyte), "\n")
	for _, l := range line {
		split := strings.Split(l, ":")
		if len(split) == 2 {
			return split[1]
		}
	}
	return ""
}

func (self *Ota) ota_update_state(stats string) error {
	self.state = stats
	os.Truncate("/tmp/version", 0)
	if self.state == "update_success" {
		self.fw_update_path = ""
		logger.Debug("ota firmware update_success")
		return nil
	} else {
		self.fw_update_path = ""
		return fmt.Errorf("ota firmware update failed")
	}
}

func Load_config_mcu_ota(config *ConfigWrapper) interface{} {
	return NewOta(config)
}
