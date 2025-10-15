package project

/*
#include <stdio.h>
#cgo CFLAGS: -I${SRCDIR}/chelper
#cgo LDFLAGS: -L${SRCDIR}/chelper -lc_helper -lm

#include "serialqueue.h"
*/
import "C"
import (
	"errors"
	"fmt"
	"k3c/common/constants"
	"k3c/common/logger"
	"k3c/common/utils/reflects"
	"k3c/common/utils/sys"
	"k3c/project/chelper"
	"k3c/project/util"
	"log"
	"net"
	"os"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/tarm/serial"
)

// Serial port management for firmware communication
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

type SerialReader struct {
	reactor           *EPollReactor
	warn_prefix       string
	serial_dev        SerialDeviceBase
	Msgparser         *MessageParser
	ffi_lib           interface{}
	default_cmd_queue interface{} //c struct command_queue *
	Serialqueue       interface{} //c struct serialqueue *
	stats_buf         []byte
	lock              sync.Locker
	background_thread interface{}
	handlers          map[string]interface{}
	last_notify_id    int64
	//pending_notifications map[int64]*ReactorCompletion
	pending_notifications sync.Map
	queueLock             sync.Locker
}

func NewSerialReader(reactor IReactor, warn_prefix string) *SerialReader {
	self := SerialReader{}
	self.reactor = reactor.(*EPollReactor)
	self.warn_prefix = warn_prefix
	// Serial port
	self.serial_dev = nil
	self.Msgparser = NewMessageParser(warn_prefix)
	// C interface
	self.ffi_lib = chelper.Get_ffi()
	self.Serialqueue = nil
	self.default_cmd_queue = self.Alloc_command_queue()
	//runtime.SetFinalizer(&self,func(){self._SerialReader()})
	self.stats_buf = make([]byte, 4096)
	// Threading
	self.lock = &sync.Mutex{}
	self.background_thread = nil
	// Message handlers
	self.handlers = map[string]interface{}{}
	self.Register_response(self._handle_unknown_init, "#unknown", nil)
	self.Register_response(self.handle_output, "#output", nil)
	// Sent message notification tracking
	self.last_notify_id = 0
	//self.pending_notifications = map[int64]*ReactorCompletion{}
	self.queueLock = &sync.Mutex{}
	self.pending_notifications = sync.Map{}
	return &self
}

func (self *SerialReader) _SerialReader() {
	if self.default_cmd_queue != nil {
		chelper.Serialqueue_free_commandqueue(self.default_cmd_queue)
	}
}

func (self *SerialReader) timeoutThread() {
	t := time.Now().Unix()
	for {
		now := time.Now().Unix()
		if now-t > 2 {
			//log.Println("for update",now, t, now-t)
			t = now
			self.pending_notifications.Range(func(key, value interface{}) bool {
				pn := value.(*ReactorCompletion)
				if now-pn.createTime > 60 || !self.reactor._process {
					log.Println("serial timeout by 60", pn.createTime)
					//delete(self.pending_notifications, key)
					self.pending_notifications.Delete(key)
					pn.Complete(nil)
				}
				return true
			})

		} else {
			if self.default_cmd_queue == nil {
				break
			}
			time.Sleep(time.Second)
		}
	}
}
func (self *SerialReader) bg_thread() {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for {
		if self.taskThread() {
			break
		}
	}
}
func (self *SerialReader) taskThread() bool {
	defer sys.CatchPanic()
	response := chelper.New_pull_queue_message()
	defer func() {
		runtime.SetFinalizer(response, func(t interface{}) {
			if self.default_cmd_queue != nil {
				chelper.Serialqueue_free_commandqueue(self.default_cmd_queue)
				self.default_cmd_queue = nil
			}
		})
	}()
	for {
		self.queueLock.Lock()
		if self.Serialqueue == nil {
			//finish while queue is empty
			self.queueLock.Unlock()
			return true
		}
		chelper.Serialqueue_pull(self.Serialqueue, response)
		self.queueLock.Unlock()
		count := response.Len
		if count < 0 {
			time.Sleep(time.Second)
			continue
		}
		if response.Notify_id > 0 {
			params := map[string]interface{}{"#sent_time": response.Sent_time,
				"#receive_time": response.Receive_time}
			completion, ok := self.pending_notifications.Load(int64(response.Notify_id))
			if ok {
				self.pending_notifications.Delete(int64(response.Notify_id))
				self.reactor.Async_complete(completion.(*ReactorCompletion), params)
			} else {
			}
			continue
		}
		params := map[string]interface{}{}
		msgs := []int{}
		for i := 0; i < int(count); i++ {
			msgs = append(msgs, int(response.Msg[i]))
		}
		params = self.Msgparser.Parse(msgs)
		params["#sent_time"] = response.Sent_time
		params["#receive_time"] = response.Receive_time
		hdl, ok := params["#name"]
		if ok {
			hdl1, ok1 := params["oid"]
			if ok1 {
				hdl = strings.Join([]string{hdl.(string), strconv.FormatInt(hdl1.(int64), 10)}, "")
			}
			self.lock.Lock()
			_func := self.handlers[hdl.(string)]
			var err error
			if _func != nil {
				err = _func.(func(map[string]interface{}) error)(params)
				if err != nil {
					logger.Errorf("%sException in serial callback",
						self.warn_prefix)

				}
			} else {
				logger.Errorf("%s Exception in serial callback %s",
					self.warn_prefix, hdl.(string))
			}
			self.lock.Unlock()
		}
	}
	return false
}
func (self *SerialReader) _error(msg string, params map[string]interface{}) {
	//log.Printf(self.warn_prefix, msg, params)
	panic(fmt.Sprintf("%s%s%s", self.warn_prefix, msg, params))
}
func (self *SerialReader) _get_identify_data(eventtime interface{}) interface{} {
	// Query the "data dictionary" from the micro-controller
	identify_data := []byte{}
	for {
		msg := fmt.Sprintf("identify offset=%d count=%d", len(identify_data), 40)

		params, err := self.Send_with_response(msg, "identify_response")
		if err != nil {
			logger.Error(err.Error())
			return nil
		}
		if params["offset"].(int64) == int64(len(identify_data)) {
			msgdata, ok := params["data"]
			if !ok {
				// Done
				return identify_data
			}
			bs, ok := msgdata.([]int)
			if ok && len(bs) == 0 {
				// Done
				return identify_data
			}
			for _, i := range msgdata.([]int) {
				identify_data = append(identify_data, byte(i))
			}
		}
	}
}

func (self *SerialReader) _start_session(serial_dev SerialDeviceBase, serial_fd_type byte, client_id int) bool {
	self.serial_dev = serial_dev
	self.Serialqueue = chelper.Serialqueue_alloc(serial_dev.GetFd(), serial_fd_type, client_id)
	go self.bg_thread()
	go self.timeoutThread()
	// Obtain and load the data dictionary from the firmware
	completion := self.reactor.Register_callback(self._get_identify_data, constants.NOW)
	identify_data := completion.Wait(self.reactor.Monotonic()+5., nil)
	if identify_data == nil {
		logger.Errorf("%sTimeout on connect", self.warn_prefix)
		self.Disconnect()
		time.Sleep(time.Second)
		return false
	}
	msgparser := NewMessageParser(self.warn_prefix)
	msgparser.Process_identify(identify_data.([]byte), true)
	self.Msgparser = msgparser
	self.Register_response(self.Handle_unknown, "#unknown", nil)
	// Setup baud adjust
	var wire_freq float64
	wire_freq = 2500000
	if serial_fd_type == 'c' {
		wire_freq = msgparser.Get_constant_float("CANBUS_FREQUENCY", nil)
	} else {
		wire_freq = msgparser.Get_constant_float("SERIAL_BAUD", nil)
	}
	if wire_freq != 0 {
		chelper.Serialqueue_set_wire_frequency(self.Serialqueue, wire_freq)

	}

	receive_window := msgparser.Get_constant_int("RECEIVE_WINDOW", nil)
	if receive_window != 0 {
		chelper.Serialqueue_set_receive_window(
			self.Serialqueue, receive_window)
	}
	return true
}

func (self *SerialReader) connect_canbus(canbus_uuid, canbus_nodeid, canbus_iface string) {
	//import can # XXX
	//txid = canbus_nodeid * 2 + 256
	//filters = [{"can_id": txid+1, "can_mask": 0x7ff, "extended": False}]
	//# Prep for SET_NODEID command
	//try:
	//uuid = int(canbus_uuid, 16)
	//except ValueError:
	//uuid = -1
	//if uuid < 0 or uuid > 0xffffffffffff:
	//self._error("Invalid CAN uuid")
	//uuid = [(uuid >> (40 - i*8)) & 0xff for i in range(6)]
	//CANBUS_ID_ADMIN = 0x3f0
	//CMD_SET_NODEID = 0x01
	//set_id_cmd = [CMD_SET_NODEID] + uuid + [canbus_nodeid]
	//set_id_msg = can.Message(arbitration_id=CANBUS_ID_ADMIN,
	//data=set_id_cmd, is_extended_id=False)
	//# Start connection attempt
	//logging.info("%sStarting CAN connect", self.warn_prefix)
	//start_time = self.reactor.monotonic()
	//while 1:
	//if self.reactor.monotonic() > start_time + 90.:
	//self._error("Unable to connect")
	//try:
	//bus = can.interface.Bus(channel=canbus_iface,
	//can_filters=filters,
	//bustype='socketcan')
	//bus.send(set_id_msg)
	//except can.CanError as e:
	//logging.warn("%sUnable to open CAN port: %s",
	//self.warn_prefix, e)
	//self.reactor.pause(self.reactor.monotonic() + 5.)
	//continue
	//bus.close = bus.shutdown # XXX
	//ret = self._start_session(bus, b'c', txid)
	//if not ret:
	//continue
	//# Verify correct canbus_nodeid to canbus_uuid mapping
	//try:
	//params = self.send_with_response('get_canbus_id', 'canbus_id')
	//got_uuid = bytearray(params['canbus_uuid'])
	//if got_uuid == bytearray(uuid):
	//break
	//except:
	//logging.exception("%sError in canbus_uuid check",
	//self.warn_prefix)
	//logging.info("%sFailed to match canbus_uuid - retrying..",
	//self.warn_prefix)
	//self.Disconnect()
}
func (self *SerialReader) Connect_pipe(filename string) {
	logger.Infof("%s Starting connect", self.warn_prefix)
	start_time := self.reactor.Monotonic()
	for {
		if self.reactor.Monotonic() > start_time+90. {
			self._error("Unable to connect", nil)
		}

		fd, err := syscall.Open(filename, syscall.O_RDWR|syscall.O_NOCTTY, 0777)
		if err != nil {
			logger.Errorf("%sUnable to open port: %s", self.warn_prefix, err)
			self.reactor.Pause(self.reactor.Monotonic() + 5.)
			continue
		}
		serial_dev := os.NewFile(uintptr(fd), filename)
		//serial_dev := C.fdopen(C.int(fd), C.CString("rb+"))
		serialDev := NewSerialDev(nil, nil, serial_dev)
		ret := self._start_session(serialDev, 'u', 0)
		if ret {
			break
		}
	}
}
func (self *SerialReader) Connect_uart(serialport string, baud int, rts bool) {
	// Initial connection
	logger.Infof("%sStarting serial connect", self.warn_prefix)
	start_time := self.reactor.Monotonic()
	for {
		start_time = self.reactor.Monotonic()
		if start_time > start_time+90. {
			self._error("Unable to connect", nil)
		}
		var err error

		cfg := &serial.Config{Name: serialport, Baud: baud, ReadTimeout: time.Microsecond * 900}
		serial_dev, err := serial.OpenPort(cfg)
		if err != nil {
			logger.Errorf("%sUnable to open serial port: %s",
				self.warn_prefix, err)
			time.Sleep(time.Second)
			self.reactor.Pause(self.reactor.Monotonic() + 5.)
			continue
		}
		serialDev := NewSerialDev(serial_dev, cfg, reflects.GetPrivateFieldValue(serial_dev, "f").(*os.File))
		// stk500v2_leave(serialDev, self.reactor)
		ret := self._start_session(serialDev, 'u', 0)

		if ret {
			break
		}

	}
}
func (self *SerialReader) Connect_remote(serialport string) {
	// Initial connection
	logger.Debugf("%sStarting remote connect %s", self.warn_prefix, serialport)
	var start_time float64
	for {
		start_time = self.reactor.Monotonic()
		if start_time > start_time+90. {
			self._error("Unable to connect", nil)
		}
		serialDev, err := NewRemoteDev(serialport)
		if err != nil {
			continue
		}
		if self._start_session(serialDev, 'n', 0) {
			break
		}
	}
}

//	func (self *SerialReader) connect_file(debugoutput interface{}, dictionary []byte, pace bool) {
//		self.serial_dev = debugoutput
//		self.Msgparser.Process_identify(dictionary, false)
//		//self.Serialqueue = chelper.Serialqueue_alloc(self.serial_dev.(SerialDev).file.Fd(), 'f', 0)
//	}
func (self *SerialReader) Set_clock_est(freq float64, conv_time float64, conv_clock int64, last_clock int64) {
	chelper.Serialqueue_set_clock_est(
		self.Serialqueue, freq, conv_time, uint64(conv_clock), uint64(last_clock))
}
func (self *SerialReader) Disconnect() {
	if self.Serialqueue != nil {
		//self.queueLock.Lock()
		chelper.Serialqueue_exit(self.Serialqueue)
		if self.background_thread != nil {
			self.background_thread = nil
		}
		chelper.Serialqueue_free(self.Serialqueue)
		self.Serialqueue = nil
		//self.queueLock.Unlock()
	}
	if self.serial_dev != nil {
		self.serial_dev.Close()
		self.serial_dev = nil
	}
	self.pending_notifications.Range(func(key, value interface{}) bool {
		pn := value.(*ReactorCompletion)
		pn.Complete(nil)
		return true
	})
	self.pending_notifications = sync.Map{}
}
func (self *SerialReader) stats(eventtime float64) string {
	if self.Serialqueue == nil {
		return ""
	}
	chelper.Serialqueue_get_stats(self.Serialqueue, self.stats_buf)
	return strings.TrimRight(string(self.stats_buf), "\x00")
}
func (self *SerialReader) Get_reactor() IReactor {
	return self.reactor
}
func (self *SerialReader) Get_msgparser() *MessageParser {
	return self.Msgparser
}
func (self *SerialReader) Get_default_command_queue() interface{} {
	return self.default_cmd_queue
}

// Serial response callbacks
func (self *SerialReader) Register_response(callback interface{}, name string, oid interface{}) {
	self.lock.Lock()
	defer self.lock.Unlock()
	key := ""
	if oid != nil && oid.(int) >= 0 {
		key = fmt.Sprintf("%s%d", name, oid.(int))
	} else {
		key = name
	}
	if callback == nil {
		delete(self.handlers, key)
	} else {
		self.handlers[key] = callback
	}
}

// Command sending
func (self *SerialReader) Raw_send(cmd []int, minclock, reqclock int64, cmd_queue interface{}) {
	chelper.Serialqueue_send(self.Serialqueue, cmd_queue,
		cmd, len(cmd), minclock, reqclock, 0)
}
func (self *SerialReader) Raw_send_wait_ack(cmd []int, minclock, reqclock int64, cmd_queue interface{}) map[string]interface{} {
	self.last_notify_id += 1
	nid := self.last_notify_id
	//log.Print(nid)
	completion := self.reactor.Completion()
	//self.pending_notifications[nid] = completion
	self.pending_notifications.Store(nid, completion)
	chelper.Serialqueue_send(self.Serialqueue, cmd_queue,
		cmd, len(cmd), minclock, reqclock, nid)
	params := completion.Wait(constants.NEVER, nil)
	if params == nil {
		self._error("Serial connection closed", nil)
		//		log.Print("Raw_send_wait_ack nil", nil)
		return nil
	}
	return params.(map[string]interface{})
}
func (self *SerialReader) Send(msg string, minclock, reqclock int64) {
	cmd := self.Msgparser.Create_command(msg)
	self.Raw_send(cmd, minclock, reqclock, self.default_cmd_queue)
}
func (self *SerialReader) Send_with_response(msg string, response string) (map[string]interface{}, error) {
	cmd := self.Msgparser.Create_command(msg)
	src := NewSerialRetryCommand(self, response, nil)
	return src.get_response([]interface{}{cmd}, self.default_cmd_queue, 0, 0)
}
func (self *SerialReader) Alloc_command_queue() interface{} {
	sq := chelper.Serialqueue_alloc_commandqueue()

	return sq
}

// Dumping debug lists
func (self *SerialReader) Dump_debug() string {
	out := []string{}
	out = append(out, fmt.Sprintf("Dumping serial stats: %s",
		self.stats(self.reactor.Monotonic())))
	//sdata := chelper.New_pull_queue_message()
	//rdata := chelper.New_pull_queue_message()
	//sdata = self.ffi_main.new('struct pull_queue_message[1024]')
	//rdata = self.ffi_main.new('struct pull_queue_message[1024]')
	var scount int
	//scount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 1,
	//sdata, len(sdata))
	//rcount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 0,
	//rdata, len(rdata))
	out = append(out, fmt.Sprintf("Dumping send queue %d messages", scount))
	//for i in range(scount):
	//msg = sdata[i]
	//cmds = self.Msgparser.dump(msg.msg[0:msg.len])
	//out.append("Sent %d %f %f %d: %s" % (
	//i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
	//out.append("Dumping receive queue %d messages" % (rcount,))
	//for i in range(rcount):
	//msg = rdata[i]
	//cmds = self.Msgparser.dump(msg.msg[0:msg.len])
	//out.append("Receive: %d %f %f %d: %s" % (
	//i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
	return strings.Join(out, "\n")
}

// Default message handlers
func (self *SerialReader) _handle_unknown_init(params map[string]interface{}) error {

	logger.Warnf("%sUnknown message %d (len %d) while identifying",
		self.warn_prefix, params["#msgid"], params["#msg"].([]int))
	return nil
}
func (self *SerialReader) Handle_unknown(params map[string]interface{}) error {
	logger.Warnf("%sUnknown message type %d: %s",
		self.warn_prefix, params["#msgid"], params["#msg"])
	return nil
}
func (self *SerialReader) handle_output(params map[string]interface{}) {
	logger.Warnf("%s%s: %s", self.warn_prefix,
		params["#name"], params["#msg"])
}
func (self *SerialReader) handle_default(params map[string]interface{}) {
	logger.Warnf("%sgot %s", self.warn_prefix, params)
}

type SerialDeviceBase interface {
	Close()
	GetFd() uintptr
}

type SerialDev struct {
	Port   *serial.Port
	Config *serial.Config
	file   *os.File
}

func (self *SerialDev) Close() {
	if self.Port != nil {
		self.Port.Close()
	} else {
		if self.file != nil {
			self.file.Close()
		}
	}
}

func (self *SerialDev) GetFd() uintptr {
	return self.file.Fd()
}

func NewSerialDev(Port *serial.Port, Config *serial.Config,
	file *os.File) *SerialDev {
	self := SerialDev{Port: Port, Config: Config, file: file}
	return &self
}

type RemoteDev struct {
	url  string
	conn *net.TCPConn
	file *os.File
}

func (self *RemoteDev) Close() {
	if self.conn != nil {
		self.conn.Close()
	}
	if self.file != nil {
		self.file.Close()
	}
}

func (self *RemoteDev) GetFd() uintptr {
	return self.file.Fd()
}

func NewRemoteDev(url string) (*RemoteDev, error) {
	addr := strings.Split(url, "tcp@")[1]

	tcpAddr, err := net.ResolveTCPAddr("tcp", addr)
	if err != nil {
		log.Print("ResolveTCPAddr failed:", err.Error())
		return nil, err
	}

	conn, err := net.DialTCP("tcp", nil, tcpAddr)
	if err != nil {
		println("Dial failed:", err.Error())
		return nil, err
	}
	conn.SetNoDelay(true)

	file, _ := conn.File()
	return &RemoteDev{url, conn, file}, nil
}

// Class to send a query command and return the received response
type SerialRetryCommand struct {
	serial      *SerialReader
	name        string
	oid         interface{}
	last_params map[string]interface{}
}

func NewSerialRetryCommand(serial *SerialReader, name string, oid interface{}) *SerialRetryCommand {
	self := SerialRetryCommand{}
	self.serial = serial
	self.name = name
	self.oid = oid
	self.last_params = nil
	self.serial.Register_response(self.handle_callback, name, oid)
	return &self
}
func (self *SerialRetryCommand) handle_callback(params map[string]interface{}) error {
	//log.Print("3 SerialRetryCommand handle_callback", params)
	self.last_params = params
	return nil
}
func (self *SerialRetryCommand) get_response(cmds []interface{}, cmd_queue interface{}, minclock, reqclock int64) (map[string]interface{}, error) {
	retries := 5
	retry_delay := .010
	for {
		lastCmdIndex := len(cmds) - 1
		if lastCmdIndex < 0 {
			return map[string]interface{}{}, nil
		}
		for i, cmd := range cmds {
			if i == lastCmdIndex {
				continue
			}
			self.serial.Raw_send(cmd.([]int), minclock, reqclock, cmd_queue)
		}
		self.serial.Raw_send_wait_ack(cmds[lastCmdIndex].([]int), minclock, reqclock,
			cmd_queue)
		params := self.last_params
		//log.Print(params)
		if params != nil {
			self.serial.Register_response(nil, self.name, self.oid)
			return params, nil
		}
		if retries <= 0 {
			self.serial.Register_response(nil, self.name, self.oid)
			return nil, errors.New(fmt.Sprintf("Unable to obtain '%s' response", self.name))
		}
		reactor := self.serial.reactor
		reactor.Pause(reactor.Monotonic() + retry_delay)
		retries -= 1
		retry_delay *= 2.
		//time.Sleep(500)
	}

}

// Attempt to place an AVR stk500v2 style programmer into normal mode
func stk500v2_leave(ser *SerialDev, reactor IReactor) {
	log.Print("Starting stk500v2 leave programmer sequence")
	util.Clear_hupcl(ser.file.Fd())
	origbaud := ser.Config.Baud
	// Request a dummy speed first as this seems to help reset the port
	ser.Config.Baud = 2400
	bs := make([]byte, 4096)
	_, err := ser.Port.Read(bs)
	if err != nil {
		logger.Error(err.Error())
	}
	// Send stk500v2 leave programmer sequence
	ser.Config.Baud = 115200
	reactor.Pause(reactor.Monotonic() + 0.100)
	ser.Port.Read(bs)
	writeData := []byte("\x1b\x01\x00\x01\x0e\x11\x04")
	ser.Port.Write(writeData)
	reactor.Pause(reactor.Monotonic() + 0.050)
	count, err1 := ser.Port.Read(bs)
	if err1 != nil {
		logger.Error(err1.Error())
	}
	logger.Infof("Got %X from stk500v2", string(bs[:count]))
	ser.Config.Baud = origbaud
}
func Cheetah_reset(serialport string, reactor IReactor) {
	// Fysetc Cheetah v1.2 boards have a weird stateful circuitry for
	// configuring the bootloader. This sequence takes care of disabling it for
	// sure.
	// Open the serial port with RTS asserted
	cfg := &serial.Config{Name: serialport, Baud: 2400, ReadTimeout: 0}
	ser, err := serial.OpenPort(cfg)
	if err != nil {
		logger.Error(err.Error())
	}
	defer ser.Close()

	bs := make([]byte, 4096)
	_, err1 := ser.Read(bs)
	if err1 != nil {
		logger.Error(err1.Error())
	}
	reactor.Pause(reactor.Monotonic() + 0.100)
	//Toggle DTR
	//ser.dtr = True
	reactor.Pause(reactor.Monotonic() + 0.100)
	//ser.dtr = False
	// Deassert RTS
	reactor.Pause(reactor.Monotonic() + 0.100)
	//ser.rts = False
	reactor.Pause(reactor.Monotonic() + 0.100)
	// Toggle DTR again
	//ser.dtr = True
	reactor.Pause(reactor.Monotonic() + 0.100)
	//ser.dtr = False
	reactor.Pause(reactor.Monotonic() + 0.100)

}

// Attempt an arduino style reset on a serial port
func Arduino_reset(serialport string, reactor IReactor) {
	//First try opening the port at a different baud
	cfg := &serial.Config{Name: serialport, Baud: 2400, ReadTimeout: 0}
	ser, err := serial.OpenPort(cfg)
	if err != nil {
		logger.Error(err.Error())
	}
	defer ser.Close()
	bs := make([]byte, 4096)
	_, err1 := ser.Read(bs)
	if err1 != nil {
		logger.Error(err1.Error())
	}
	reactor.Pause(reactor.Monotonic() + 0.100)
	// Then toggle DTR
	//ser.dtr = true
	reactor.Pause(reactor.Monotonic() + 0.100)
	//ser.dtr = false
	reactor.Pause(reactor.Monotonic() + 0.100)

}
