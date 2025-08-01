package ld2451

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"sync"
	"time"

	"github.com/tarm/serial"
)

type Config struct {
	SerialPort       string
	BaudRate         int
	TargetBufferSize int //Size of the channel buffer to store targets in
}

type Target struct {
	Angle     int       // Angle of the target relative to the perpendicular direction of the antenna
	Distance  int       // Distance in meters to the target
	Direction Direction // Direction of movement relative to the antenna
	Speed     int       // Speed in KM/H
	SNR       int       // Signal to Noise Ratio
}

const (
	DirectionAway   Direction = 0
	DirectionToward Direction = 1
	DirectionAll    Direction = 2
)

type Direction uint8

const (
	SerialPortBaudRate9600   SerialPortBaudRate = 0x0001
	SerialPortBaudRate19200  SerialPortBaudRate = 0x0002
	SerialPortBaudRate38400  SerialPortBaudRate = 0x0003
	SerialPortBaudRate57600  SerialPortBaudRate = 0x0004
	SerialPortBaudRate115200 SerialPortBaudRate = 0x0005 // default
	SerialPortBaudRate230400 SerialPortBaudRate = 0x0006
	SerialPortBaudRate256000 SerialPortBaudRate = 0x0007
	SerialPortBaudRate460800 SerialPortBaudRate = 0x0008
)

type SerialPortBaudRate uint8

func (d Direction) String() string {
	switch d {
	case DirectionAway:
		return "Away"
	case DirectionToward:
		return "Toward"
	case DirectionAll:
		return "All"
	default:
		return "Unknown"
	}
}

type LD2451 struct {
	config    Config
	targets   chan Target
	errors    chan error
	port      *serial.Port
	wg        sync.WaitGroup
	stop_read chan bool
}

func OpenLD2451(config Config) (*LD2451, error) {
	serialConfig := &serial.Config{
		Name:        config.SerialPort,
		Baud:        115200,
		ReadTimeout: time.Second * 2,
		Parity:      serial.ParityNone,
	}

	port, err := serial.OpenPort(serialConfig)
	if err != nil {
		return nil, err
	}

	ld2451 := &LD2451{
		config:  config,
		targets: make(chan Target, config.TargetBufferSize),
		errors:  make(chan error),
		port:    port,
	}

	{
		// Enable the configuration command
		err := ld2451.EnableConfigCmd()
		if err != nil {
			return nil, fmt.Errorf("failed to enable config command: %v", err)
		} else {
			fmt.Println("Config command enabled")
		}
	}

	{
		ver, err := ld2451.GetFirmwareVersion()
		if err != nil {
			return nil, fmt.Errorf("failed to get firmware version: %v", err)
		} else {
			fmt.Printf("Firmware version: %s\n", ver)
		}
	}

	{
		err := ld2451.EndConfigCmd()
		if err != nil {
			return nil, fmt.Errorf("failed to end config command: %v", err)
		} else {
			fmt.Println("Config command end")
		}
	}

	return ld2451, nil
}

func (ld2451 *LD2451) Close() {
	ld2451.port.Close()
}

func (ld2451 *LD2451) sendFrame(data []byte) ([]byte, error) {
	//	Radar serial port data format description:
	//	1. Frame: Each command data is called a frame; the frame consists of 4 parts:
	//	   frame header, data length in the frame, data in the frame and tail of the frame;
	//	2. Intra-frame data: The intra-frame data starts with a command, followed by the data content
	//
	//	Notice:
	//	1. The maximum data length of a single serial port command does not exceed 64 bytes
	//		(the size is subject to the actual situation, and each platform may be different.
	//		When the upper computer sends the start command, the result returned by the lower
	//		computer includes the buffer size of the command communication) , so when reading
	//		and writing multiple registers, if it exceeds 64 bytes, it needs to be divided
	//		into multiple commands and issued.
	//	2. Byte order: little endian

	var frameheader = []byte{0xfd, 0xfc, 0xfb, 0xfa}
	var framefooter = []byte{0x04, 0x03, 0x02, 0x01}
	datasize := make([]byte, 2)
	binary.LittleEndian.PutUint16(datasize, uint16(len(data)))

	//create the frame
	var frame = make([]byte, 0)
	frame = append(frame, frameheader...)
	frame = append(frame, datasize...)
	frame = append(frame, data...)
	frame = append(frame, framefooter...)

	cnt, err := ld2451.port.Write(frame)
	if err != nil || cnt != len(frame) {
		return nil, fmt.Errorf("failed to send frame (%d-%d): %v", cnt, len(frame), err)
	}

	buf := make([]byte, 1)
	// wait for the ACK sync
	for i := 0; i < 100; i++ {
		cnt, err := ld2451.port.Read(buf)
		if err != nil || cnt == 0 {
			return nil, fmt.Errorf("failed to read ACK sync: %v", err)
		}

		if buf[0] == frameheader[0] {
			break
		}
	}

	//read the response
	buf = make([]byte, 5)
	cnt, err = ld2451.port.Read(buf)
	if err != nil || cnt != len(buf) || !bytes.Equal(buf[0:3], frameheader[1:]) {
		return nil, fmt.Errorf("failed to read frame ACK: %v (%d:%v)", err, cnt, buf)
	}

	var size = binary.LittleEndian.Uint16(buf[3:5])
	if size == 0 {
		return nil, fmt.Errorf("frame size is 0")
	}

	buf = make([]byte, size+4)
	cnt, err = ld2451.port.Read(buf)

	if err != nil || cnt != len(buf) {
		return nil, fmt.Errorf("failed to read ACK frame footer: %v", err)
	}

	if !bytes.Equal(buf[len(buf)-4:], framefooter) {
		return nil, fmt.Errorf("frame footer is not valid")
	}

	return buf[:len(buf)-4], nil
}

func (ld2451 *LD2451) sendCmd(cmd byte, data []byte) ([]byte, error) {

	var frame = make([]byte, 0)
	frame = append(frame, cmd, 0x00)
	frame = append(frame, data...)

	data, err := ld2451.sendFrame(frame)
	if err != nil {
		return nil, err
	}

	if len(data) < 2 {
		return nil, fmt.Errorf("ACK frame size is too small (%d)", len(data))
	}

	if data[0] != cmd || data[1] != 0x01 {
		return nil, fmt.Errorf("ACK cmd header is not valid (%d)", data[0])
	}

	return data[2:], nil

}

func (ld2451 *LD2451) GetFirmwareVersion() (string, error) {
	// Return value: 2-byte ACK status (0 success, 1 failure) + 2-byte firmware type (0x2451) + 2-byte major version number + 4-byte minor version number
	data, err := ld2451.sendCmd(0xA0, []byte{})
	if err != nil {
		return "", err
	}

	if data[0] != 0x00 || data[1] != 0x00 {
		return "", fmt.Errorf("GEtFirmwareVersion: invalid response")
	}

	if data[2] != 0x51 || data[3] != 0x24 {
		return "", fmt.Errorf("GetFirmwareVersion: invalid firmware type %x %x", data[2], data[3])
	}

	fmt.Printf("GetFirmwareVersion: %x %x\n", data[4], data[5])

	// Convert the version number to a string
	var version = fmt.Sprintf("%d.%d.%d%d%d%d", data[5], data[4], data[9], data[8], data[7], data[6])

	// Return the version number
	return version, nil
}

// Enabling Configuration Commands
// Any other command issued to the radar must be executed after this command is issued, otherwise it will be invalid.
func (ld2451 *LD2451) enableConfigCmdInt() error {
	// Command word: 0x00FF
	// Command value: 0x0001
	// Return value: 2 bytes ACK status (0 success, 1 failure) + 2 bytes protocol version (0x0001) + 2 bytes

	data, err := ld2451.sendCmd(0xff, []byte{0x00, 0x01})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("EnableConfigCmd: unsuccessfull response")
	}

	//fmt.Printf("EnableConfigCmd proto: %x %x\n", data[2], data[3])

	return nil
}

// Enabling Configuration Commands
// Any other command issued to the radar must be executed after this command is issued, otherwise it will be invalid.
func (ld2451 *LD2451) EnableConfigCmd() error {
	// The usual method is divided into three steps:
	//    a) Send "open command mode" (because the chip may still output data,
	//       the data received by the serial port will contain waveform data, so the returned result will not be analyzed)
	//    b) Clear the serial port cache data (generally delay about 100ms, make sure the serial port data is cleared)
	//    c) Send "open command mode" again, and analyze the returned result

	ld2451.enableConfigCmdInt()
	time.Sleep(100 * time.Millisecond)
	return ld2451.enableConfigCmdInt()
}

// End configuration command
// End the configuration command, and the radar will resume working mode after execution. If you need to send other commands again, you need to send the enable configuration command first.
func (ld2451 *LD2451) EndConfigCmd() error {
	// Command word: 0x00FE
	// Command value: None
	// Return value: 2-byte ACK status (0 for success, 1 for failure)
	data, err := ld2451.sendCmd(0xfe, []byte{})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("EndConfigCmd: unsuccessfull response")
	}
	return nil
}

// This command is used to restore all configuration values to factory settings. The configuration values take effect after the module is restarted.
func (ld2451 *LD2451) RestoreFactorySettings() error {
	// Command word: 0x00A2
	// Command value: None
	// Return value: 2-byte ACK status (0 for success, 1 for failure)

	data, err := ld2451.sendCmd(0xa2, []byte{})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("RestoreFactorySettings: unsuccessfull response")
	}

	return nil
}

// When the module receives this command, it will automatically restart after the response is sent.
func (ld2451 *LD2451) RestartModule() error {
	// Command word: 0x00A3
	// Command value: None
	// Return value: 2-byte ACK status (0 for success, 1 for failure)

	data, err := ld2451.sendCmd(0xa3, []byte{})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("RestartModule: unsuccessfull response")
	}

	return nil
}

// Target detection parameter configuration commands
// Any other command issued to the radar must be executed after this command is issued, otherwise it will be invalid.
func (ld2451 *LD2451) SetTargetDetectionCfg(max_distance uint8, direction Direction, min_speed uint8, target_delay uint8) error {
	// Command word: 0x0002
	// Command value: 4 bytes
	//   1 byte - Maximum detection distance 		- 0A-FF, Unit: m
	//   1 byte - Movement direction setting		- 00: Only detect away, 01: Only detect approach, 02: All detected
	//   1 byte - Minimum movement speed setting 	- 00-0x78, Unit: km/h
	//   1 byte - No target delay time setting		- 00~FF, Unit: s
	// Return value: 2 bytes ACK status (0 success, 1 failure)

	if max_distance < 0x0A {
		max_distance = 0x0A
	}
	if min_speed > 0x78 {
		min_speed = 0x78
	}

	data, err := ld2451.sendCmd(0x02, []byte{byte(max_distance), byte(direction), byte(min_speed), byte(target_delay)})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("SetTargetDetectionCfg: unsuccessfull response")
	}
	return nil
}

// Read target detection parameter command
// This command can read the radar's current target detection parameters.
func (ld2451 *LD2451) GetTargetDetectionCfg() (max_distance uint8, direction Direction, min_speed uint8, target_delay uint8, err error) {
	// Command word: 0x0012
	// Command value: None
	// Return value: 2 bytes ACK status (0 success, 1 failure) + 4 bytes configuration value (format is the same as the setting command)

	var data []byte
	data, err = ld2451.sendCmd(0x12, []byte{})
	if err != nil {
		return
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		err = fmt.Errorf("GetTargetDetectionCfg: unsuccessfull response")
	} else {
		max_distance = uint8(data[2])
		direction = Direction(data[3])
		min_speed = uint8(data[4])
		target_delay = uint8(data[5])
	}
	return
}

// Radar sensitivity parameter configuration command
func (ld2451 *LD2451) SetRadarSensitivity(trigger_times bool, snr uint8) error {
	// Command word: 0x0003
	// Command value: 4-byte sensitivity value
	//   1 byte - Cumulative effective trigger times 		- 1-0, Default 1, The alarm information will be reported only when the number of consecutive detections is met.
	//   1 byte - Signal-to-noise ratio threshold level	- 3-8, Default 4, The larger the value, the lower the sensitivity and the more difficult it is to detect the target.
	//   1 byte - Extended Parameters 					- 00
	//   1 byte - Extended Parameters						- 00
	// Return value: 2 bytes ACK status (0 success, 1 failure)
	var trigger byte
	if trigger_times {
		trigger = 1
	}
	if snr < 3 {
		snr = 3
	}
	if snr > 8 {
		snr = 8
	}

	data, err := ld2451.sendCmd(0x03, []byte{trigger, snr, 0, 0})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("SetRadarSensitivity: unsuccessfull response")
	}

	return nil
}

// This command queries the motion sensitivity of each range gate.
func (ld2451 *LD2451) GetRadarSensitivity() (trigger_times bool, snr uint8, err error) {
	// Command word: 0x0013
	// Command value: None
	// Return value: 2-byte ACK status (0 for success, 1 for failure) + 4-byte sensitivity value (same format as setting command)

	data, err := ld2451.sendCmd(0x13, []byte{})
	if err != nil {
		return
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		err = fmt.Errorf("GetRadarSensitivity: unsuccessfull response")
	} else {
		trigger_times = uint8(data[2]) == 1
		snr = uint8(data[3])
	}
	return
}

// Setting the serial port baud rate
// This command is used to set the baud rate of the module serial port.
// The configuration value will not be lost when the power is off.
// The configuration value will take effect after the module is restarted.
func (ld2451 *LD2451) SetSerialPortBaudRate(serial_rate SerialPortBaudRate) error {
	// Command word: 0x00A1
	// Command value: 2-byte baud rate selection index
	// The factory default value is 0x0005, which is 115200
	// Baud rate selection index value 	|	Baud rate
	//							0x0001	|	  9600
	//			 				0x0002	|  	 19200
	//			 				0x0003 	| 	 38400
	//			 				0x0004 	| 	 57600
	//			 				0x0005 	|	115200
	//							0x0006 	| 	230400
	//			 				0x0007 	| 	256000
	//							0x0008 	|   460800
	// Return value: 2 bytes ACK status (0 success, 1 failure)

	data, err := ld2451.sendCmd(0xA1, []byte{byte(serial_rate), 0})
	if err != nil {
		return err
	}
	if data[0] != 0x00 || data[1] != 0x00 {
		return fmt.Errorf("SetSerialPortBaudRate: unsuccessfull response")
	}

	return nil
}

func (ld2451 *LD2451) StartReadThread() {
	ld2451.stop_read = make(chan bool)
	go ld2451.read_thread()
	return
}

func (ld2451 *LD2451) StartSyncReadThread() {
	ld2451.stop_read = make(chan bool)
	ld2451.read_thread()
	return
}

func (ld2451 *LD2451) StopReadThread() {
	ld2451.stop_read <- true
	ld2451.wg.Wait()
}

// add error to channel even if it full
func (ld2451 *LD2451) add_error(err error) {
	select {
	case ld2451.errors <- err:
		// ок
	default:
		// канал повний, обробка
		<-ld2451.errors
		ld2451.errors <- err
	}
}

// LD2451 outputs radar detection results through the serial port.
// By default, if no target is detected, the frame header, frame length, and frame tail are output.
// The radar will additionally output information such as the number, angle, distance, speed, etc. of detected targets.
// The radar data is output in the specified frame format.
//
// Reporting data frame format
//	Frame Header			- 0xF4 0xF3 0xF2 0xF1
//  Data length in frame - 2 bytes
//	Intra-frame data		- See next table
//	Frame tail (footer)	- 0xF8 0xF7 0xF6 0xF5
//
//   Intra-frame data frame format
//   1 byte Targeta quantity
//   1 byte Alarm information, 1 if there is a target approaching, 0 no aproach target
//   5 bytes - target N information
//		1 byte - angle, unit degree, Actual angle value = reported value - 0x80 (тут потрібно віднімати 0x80)
//		1 byte - distance, unit meter, 0-100
//		1 byte - direction, 0x00 - away, 0x01 - approach
//		1 byte - speed km/h, 0-120
//		1 byte - signal-to-noise, 0-255

func (ld2451 *LD2451) read_thread() {
	ld2451.wg.Add(1)
	defer ld2451.wg.Done()
	var frameheader = []byte{0xf4, 0xf3, 0xf2, 0xf1}

	for {
		buf := make([]byte, 1)
		_, err := ld2451.port.Read(buf)
		if err != nil {
			ld2451.errors <- err
			return
		}

		if buf[0] != frameheader[0] {
			continue
		}

		//check if the next 3 bytes are the frame header
		buf = make([]byte, 3)
		_, err = ld2451.port.Read(buf)
		if err != nil {
			ld2451.errors <- err
			return
		}

		if bytes.Equal(buf, frameheader[1:]) {

			//fmt.Print("NEW TARGET: ")

			//get length of the frame (next 2 bytes)
			buf = make([]byte, 2)
			_, err := ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}
			frameLength := int(buf[1])<<8 | int(buf[0])
			if frameLength == 0 {
				// restart loop if there is no more data
				// read the next 4 bytes, this is the frame footer []byte{0xf8, 0xf7, 0xf6, 0xf5}
				// skip 4 bytes
				buf = make([]byte, 4)
				_, err = ld2451.port.Read(buf)
				if err != nil {
					ld2451.errors <- err
					return
				}
				continue
			}

			//fmt.Printf("frame:%d ", frameLength)

			//read the rest of the frame
			buf = make([]byte, frameLength)
			_, err = ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}

			//get the number of targets in the frame, this is the next byte after the frame length
			numTargets := int(buf[0])
			//fmt.Printf("targets:%d ", numTargets)
			//move to the next byte AND skip alarm state
			buf = buf[2:]

			//loop over and parse each target
			for i := 0; i < numTargets; i++ {
				target := Target{}
				//get the target data
				offset := i * 5
				target.Angle = int(buf[offset+0]) - 0x80
				target.Distance = int(buf[offset+1])
				target.Direction = Direction(buf[offset+2])
				target.Speed = int(buf[offset+3])
				target.SNR = int(buf[offset+4])

				//send the target to the channel
				ld2451.targets <- target
				//move to the next target
			}
			//flush the rest of the frame
			buf = make([]byte, 4)
			_, err = ld2451.port.Read(buf)
			if err != nil {
				ld2451.errors <- err
				return
			}
			//fmt.Println()
		}
	}
}

func (ld2451 *LD2451) ReadTarget() (Target, error) {
	select {
	case target := <-ld2451.targets:
		return target, nil
	case err := <-ld2451.errors:
		return Target{}, err
	}
}

func (ld2451 *LD2451) ReadTargetNoBlock() (*Target, error) {
	select {
	case target := <-ld2451.targets:
		return &target, nil
	case err := <-ld2451.errors:
		return nil, err
	default:
		return nil, nil
	}
}
