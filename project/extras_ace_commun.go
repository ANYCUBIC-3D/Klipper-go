package project

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"github.com/tarm/serial"
	"k3c/common/logger"
	"k3c/common/utils/reflects"
	"k3c/project/queue"
	"os"
	"strings"
	"syscall"
	"time"
)

const (
	FRAME_START_1  = 0xFF
	FRAME_START_2  = 0xAA
	FRAME_END      = 0xFE
	MIN_FRAME_SIZE = 7 // start(2) + len(2) + CRC(2) + end(1)
)

const (
	RESPOND_TIMEOUT_ERROR  = "Respond timeout with the ACE PRO"
	UNABLE_TO_COMMUN_ERROR = "Unable to communicate with the ACE PRO"
	OPEN_REMOTE_DEV_ERROR  = "Unable to open remote dev"
	OPEN_SERIAL_DEV_ERROR  = "Unable to open serial port"
	NOT_FOUND_SERIAL_ERROR = "Not found serial port"
)

func _calc_crc(buf []byte) uint16 {
	var _crc uint16 = 0xffff
	for i := 0; i < len(buf); i++ {
		data := uint16(buf[i])
		data ^= _crc & 0xff
		data ^= (data & 0x0f) << 4
		_crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
	}
	return _crc
}

type AceCommun struct {
	//serial hardware property
	name string
	dev  SerialDeviceBase
	baud int

	//serial state
	is_connected bool

	//serial send
	request_id int
	send_time  float64
	queue      *queue.Queue

	//serial read
	read_buffer  []byte
	callback_map map[int]func(map[string]interface{})
}

type RequestInfo struct {
	request  map[string]interface{}
	callback func(map[string]interface{})
}

func NewAceCommunication(name string, baud int) *AceCommun {
	self := new(AceCommun)
	self.is_connected = false
	self.dev = nil
	self.name = name
	self.baud = baud
	self.send_time = 0.
	self.read_buffer = nil
	self.callback_map = map[int]func(resp map[string]interface{}){}
	self.request_id = 0

	return self
}

func (self *AceCommun) Check_ace_port_exit(fileName string) bool {
	info, err := os.Stat(fileName)
	if os.IsNotExist(err) {
		return false
	}
	return !info.IsDir()
}

func (self *AceCommun) Connect() error {
	if strings.HasPrefix(self.name, "tcp@") {
		dev, err := NewRemoteDev(self.name)
		if err != nil {
			logger.Errorf("%s %s: %s", OPEN_REMOTE_DEV_ERROR, self.name, err)
			return fmt.Errorf("%s %s: %s", OPEN_REMOTE_DEV_ERROR, self.name, err)
		}
		self.dev = dev
	} else {
		if !self.Check_ace_port_exit(self.name) {
			return fmt.Errorf("%s %s ", NOT_FOUND_SERIAL_ERROR, self.name)
		}

		cfg := &serial.Config{Name: self.name, Baud: self.baud, ReadTimeout: time.Microsecond * 900}
		port, err := serial.OpenPort(cfg)
		if err != nil {
			logger.Errorf("%s %s: %s", OPEN_SERIAL_DEV_ERROR, self.name, err)
			return fmt.Errorf("%s %s: %s", OPEN_SERIAL_DEV_ERROR, self.name, err)
		}
		self.dev = NewSerialDev(port, cfg, reflects.GetPrivateFieldValue(port, "f").(*os.File))
	}
	self.is_connected = true
	self.queue = queue.NewQueue()
	return nil
}

func (self *AceCommun) Disconnect() {
	self.dev.Close()
	self.is_connected = false
	self.queue = nil
	self.dev = nil
	self.send_time = 0.
	self.read_buffer = nil
	self.request_id = 0
	self.callback_map = map[int]func(map[string]interface{}){}
}

func (self *AceCommun) Is_connected() bool {
	return self.is_connected
}

func (self *AceCommun) _send_request(req map[string]interface{}) error {
	if _, ok := req["id"]; !ok {
		req["id"] = self.request_id
		self.request_id++
	}
	var buf []byte
	buf = append(buf, FRAME_START_1)
	buf = append(buf, FRAME_START_2)

	bts, _ := json.Marshal(req)
	size := len(bts)

	buf = append(buf, byte(size))
	buf = append(buf, byte(size>>8))

	crc := _calc_crc(bts)

	buf = append(buf, bts...)
	buf = append(buf, byte(crc))
	buf = append(buf, byte(crc>>8))
	buf = append(buf, FRAME_END)
	fd := int(self.dev.GetFd())
	_, err := syscall.Write(fd, buf)

	return err
}

func (self *AceCommun) Writer(eventtime float64) {
	if !self.queue.Is_empty() {
		task := self.queue.Get_nowait()
		if task != nil {
			id := self.request_id
			self.request_id += 1
			self.callback_map[id] = task.(RequestInfo).callback
			task.(RequestInfo).request["id"] = id
			self._send_request(task.(RequestInfo).request)
			self.send_time = eventtime
		}
	}
}

func (self *AceCommun) Reader(eventtime float64) error {
	buffer := []byte{}
	raw_bytes := make([]byte, 4096)
	fd := int(self.dev.GetFd())
	n, err := syscall.Read(fd, raw_bytes)
	if err != nil {
		return fmt.Errorf("%s %v", UNABLE_TO_COMMUN_ERROR, err)
	}

	if n > 0 && len(raw_bytes) > 0 {
		text_buffer := append(self.read_buffer, raw_bytes[:n]...)
		i := bytes.IndexByte(text_buffer, FRAME_END)
		if i >= 0 {
			buffer = text_buffer
			self.read_buffer = []byte{}
		} else {
			self.read_buffer = append(self.read_buffer, raw_bytes[:n]...)
			return nil
		}
	} else {
		if (eventtime - self.send_time) > 2 {
			return fmt.Errorf(RESPOND_TIMEOUT_ERROR)
		}
		return nil
	}

	if len(buffer) < MIN_FRAME_SIZE {
		return nil
	}

	if buffer[0] != FRAME_START_1 || buffer[1] != FRAME_START_2 {
		logger.Error("Invalid data from ACE PRO (head bytes)", string(buffer))
		return nil
	}

	payload_len := uint16(buffer[2]) | uint16(buffer[3])<<8
	if len(buffer) < int(payload_len+MIN_FRAME_SIZE) {
		logger.Errorf("Invalid data from ACE PRO (len) {%d} {%d} %v", payload_len, len(buffer), string(buffer))
		return nil
	}

	if buffer[len(buffer)-1] != FRAME_END {
		logger.Errorf("Invalid data from ACE PRO (end bytes)", string(buffer))
		return nil
	}

	payload := buffer[4 : 4+payload_len]

	crc := binary.LittleEndian.Uint16(buffer[4+payload_len : 4+payload_len+2])

	if len(buffer) < int(4+payload_len+2+1) {
		logger.Errorf("Invalid data from ACE PRO (len) {%d} {%d} {%d} %v", payload_len, len(buffer), crc, string(buffer))
		return nil
	}

	if crc != _calc_crc(payload) {
		logger.Error("Invalid data from ACE PRO (CRC)")
		return nil
	}

	var ret map[string]interface{}
	if err := json.Unmarshal(payload, &ret); err != nil {
		logger.Errorf("json error:", err)
		return nil
	}

	id := int(ret["id"].(float64))
	if cb, ok := self.callback_map[id]; ok {
		delete(self.callback_map, id)
		cb(ret)
	}
	return nil
}

func (self *AceCommun) Push_send_queue(request map[string]interface{}, callback func(map[string]interface{})) {
	self.queue.Put_nowait(RequestInfo{request, callback})
}

func (self *AceCommun) Name() string {
	return self.name
}

func (self *AceCommun) Fd() int {
	return int(self.dev.GetFd())
}

func (self *AceCommun) Is_send_queue_empty() bool {
	return self.queue.Is_empty()
}
