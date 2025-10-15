package errors

import "fmt"

type Code string

var (
	// common
	UnknownCode            Code = "1000400"
	MissingArgumentCode    Code = "1000401"
	InvalidArgumentCode    Code = "1000402"
	MethodUnregisteredCode Code = "1000403"
	// printer
	K3CRunningCode               Code = "10010505"
	K3CShutdowningCode           Code = "10010506"
	PrintFileNozzleDisErrorCode  Code = "10010507"
	PrintFileXMoveOutOfRangeCode Code = "10010508"
	PrintFileYMoveOutOfRangeCode Code = "10010509"
	PrintFileZMoveOutOfRangeCode Code = "10010510"
	// homing/move
	MustHomeAxisFirstCode   Code = "10011600"
	MoveOutOfRangeCode      Code = "10011601"
	MustHomeXYAxesFirstCode Code = "10011602"
	// leviq2
	PressuareErrorCode          Code = "10011499"
	PrinterZeroingErrorCode     Code = "10011500"
	ExtrusionErrorCode          Code = "10011501"
	AutomaticLevelingFailedCode Code = "10011502"
	ButtonAdvanceTriggerCode    Code = "10011503"
	ButtonNotCalibratedCode     Code = "10011504"
	ButtonNotActivatedCode      Code = "10011505"
	ButtonErrorCode             Code = "10011506"
	ExtruderNtcErrorCode        Code = "10011507"
	HeaterBedNtcErrorCode       Code = "10011508"
	// print record
	PrintRecordIsNotExistErrorCode Code = "10011509"
	GcodeNotExistErrorCode         Code = "10011510"
	ExtruderHeatingErrorCode       Code = "10011511"
	HeaterBedHeatingErrorCode      Code = "10011512"
	// fliament hub
	InvalidFilamentIdErrorCode                Code = "10011700"
	InvalidFilamentHubIdErrorCode             Code = "10011701"
	FilamentHubNotExistErrorCode              Code = "10011702"
	UnknownFilamentInExtruderErrorCode        Code = "10011703"
	InvalidFilamentHubTempErrorCode           Code = "10011704"
	InvalidFilamentHubDurationErrorCode       Code = "10011705"
	InvalidFilamentHubFanSpeedErrorCode       Code = "10011706"
	InvalidFilamentHubTypeErrorCode           Code = "10011707"
	WaitFilamentHubTimeoutErrorCode           Code = "10011708"
	UnableToEditFilamentInfoErrorCode         Code = "10011709"
	FilamentHubFeedFilamentTimeoutErrorCode   Code = "10011710"
	FilamentHubUnwindFilamentTimeoutErrorCode Code = "10011711"
	FilamentHubWaitReadyTimeoutErrorCode      Code = "10011712"
	FilamentHubWaitActionTimeoutErrorCode     Code = "10011713"
	FilamentHubCommunicationErrorCode         Code = "10011714"
	ExtruderMustHeatUpFirstErrorCode          Code = "10011715"
	FilamentNotPresentInExtruderErrorCode     Code = "10011716"
	FilamentHubIsEmptyErrorCode               Code = "10011717"
	FilamentIsStuckErrorCode                  Code = "10011718"
	FilamentIsTangledErrorCode                Code = "10011719"
	FilamentRunoutErrorCode                   Code = "10011720"
	FilamentHubGearStuckErrorCode             Code = "10011721"
	FilamentHubDisconnectedErrorCode          Code = "10011722"
	NotEnoughFilamentsInHubErrorCode          Code = "10011723"
	GcodeFileNotExistTCmdErrorCode            Code = "10011724"

	// motor
	MotorWiringErrorCode Code = "10011801"
)

var codeMessageMaps = map[Code]string{
	UnknownCode:                    "unknown error",
	MissingArgumentCode:            "missing argument",
	InvalidArgumentCode:            "invalid argument type",
	K3CRunningCode:                 "k3c is runing script, please try later",
	K3CShutdowningCode:             "k3c is shutdowing, please try later",
	PrintFileNozzleDisErrorCode:    "The nozzle size of the gcode file is inconsistent with the nozzle size of the device",
	PrintFileXMoveOutOfRangeCode:   "The X-axis of the printing file exceeds the device move out of range",
	PrintFileYMoveOutOfRangeCode:   "The Y-axis of the printing file exceeds the device move out of range",
	PrintFileZMoveOutOfRangeCode:   "The Z-axis of the printing file exceeds the device move out of range",
	MustHomeAxisFirstCode:          "Must home axis first",
	MoveOutOfRangeCode:             "Move out of range",
	MustHomeXYAxesFirstCode:        "Must home X and Y axes first",
	PressuareErrorCode:             "Pressuare home Z error",
	PrinterZeroingErrorCode:        "printer zeroing error",
	ExtrusionErrorCode:             "extrusion error",
	AutomaticLevelingFailedCode:    "automatic leveling failed",
	ButtonAdvanceTriggerCode:       "button advance trigger",
	ButtonNotCalibratedCode:        "button not calibrated",
	ButtonNotActivatedCode:         "button not activated",
	ButtonErrorCode:                "the leivQ2 button may be damaged",
	ExtruderNtcErrorCode:           "extruder Ntc error",
	ExtruderHeatingErrorCode:       "extruder heating error",
	HeaterBedNtcErrorCode:          "heater_bed Ntc error",
	PrintRecordIsNotExistErrorCode: "Print record is not exist",
	GcodeNotExistErrorCode:         "Gcode file is not exist",
	HeaterBedHeatingErrorCode:      "heater_bed heating error",

	InvalidFilamentIdErrorCode:                "invalid filament id",
	InvalidFilamentHubIdErrorCode:             "invalid filament hub id",
	FilamentHubNotExistErrorCode:              "filament hub not exist",
	UnknownFilamentInExtruderErrorCode:        "unknown filament in extruder",
	InvalidFilamentHubTempErrorCode:           "invalid filament hub temp",
	InvalidFilamentHubDurationErrorCode:       "invalid filament hub duration",
	InvalidFilamentHubFanSpeedErrorCode:       "invalid filament hub fan speed",
	InvalidFilamentHubTypeErrorCode:           "invalid filament hub type",
	WaitFilamentHubTimeoutErrorCode:           "wait filament hub response timeout",
	UnableToEditFilamentInfoErrorCode:         "unable to edit filament info",
	FilamentHubFeedFilamentTimeoutErrorCode:   "filament hub feed filament timeout",
	FilamentHubUnwindFilamentTimeoutErrorCode: "filament hub unwind filament timeout",
	FilamentHubWaitReadyTimeoutErrorCode:      "filament hub wait ready timeout",
	FilamentHubWaitActionTimeoutErrorCode:     "filament hub wait action timeout",
	FilamentHubCommunicationErrorCode:         "filament hub communication error",
	ExtruderMustHeatUpFirstErrorCode:          "Extruder must heat up first",
	FilamentNotPresentInExtruderErrorCode:     "Filament not present in extruder",
	FilamentHubIsEmptyErrorCode:               "Filament hub is empty",
	FilamentIsStuckErrorCode:                  "Filament is stuck",
	FilamentIsTangledErrorCode:                "Filament is tangled",
	FilamentRunoutErrorCode:                   "Filament is runout",
	FilamentHubGearStuckErrorCode:             "Filament hub gear struck",
	FilamentHubDisconnectedErrorCode:          "Filament hub disconnected",
	NotEnoughFilamentsInHubErrorCode:          "Not enough filaments in filament hub",
	GcodeFileNotExistTCmdErrorCode:            "The gcode file does not contain the T command",
	MotorWiringErrorCode:                      "Motor wiring error",
}

var (
	UnknownError                  = FromCode(UnknownCode)
	MissingArgumentError          = FromCode(MissingArgumentCode)
	InvalidArgumentError          = FromCode(InvalidArgumentCode)
	MethodUnregisteredError       = FromCode(MethodUnregisteredCode)
	K3CRunningError               = FromCode(K3CRunningCode)
	K3CShutdowningError           = FromCode(K3CRunningCode)
	PrintFileNozzleDisError       = FromCode(PrintFileNozzleDisErrorCode)
	PrintFileXMoveOutOfRangeError = FromCode(PrintFileXMoveOutOfRangeCode)
	PrintFileYMoveOutOfRangeError = FromCode(PrintFileYMoveOutOfRangeCode)
	PrintFileZMoveOutOfRangeError = FromCode(PrintFileZMoveOutOfRangeCode)
	MustHomeAxisFirstError        = FromCode(MustHomeAxisFirstCode)
	MoveOutOfRangeError           = FromCode(MoveOutOfRangeCode)
	MustHomeXYAxesFirstError      = FromCode(MustHomeXYAxesFirstCode)
	PressuareZAxesError           = FromCode(PressuareErrorCode)
	PrinterZeroingError           = FromCode(PrinterZeroingErrorCode)
	ExtrusionError                = FromCode(ExtrusionErrorCode)
	AutomaticLevelingFailedError  = FromCode(AutomaticLevelingFailedCode)
	ButtonAdvanceTriggerError     = FromCode(ButtonAdvanceTriggerCode)
	ButtonNotCalibratedError      = FromCode(ButtonNotCalibratedCode)
	ButtonNotActivatedError       = FromCode(ButtonNotActivatedCode)
	ButtonError                   = FromCode(ButtonNotActivatedCode)
	ExtruderNtcError              = FromCode(ExtruderNtcErrorCode)
	ExtruderHeatingError          = FromCode(ExtruderHeatingErrorCode)
	HeaterBedNtcError             = FromCode(HeaterBedNtcErrorCode)
	PrintRecordIsNotExistError    = FromCode(PrintRecordIsNotExistErrorCode)
	GcodeIsNotExistError          = FromCode(GcodeNotExistErrorCode)
	HeaterBedHeatingError         = FromCode(HeaterBedHeatingErrorCode)

	InvalidFilamentIdError                = FromCode(InvalidFilamentIdErrorCode)
	InvalidFilamentHubIdError             = FromCode(InvalidFilamentHubIdErrorCode)
	FilamentHubNotExistError              = FromCode(FilamentHubNotExistErrorCode)
	UnknownFilamentInExtruderError        = FromCode(UnknownFilamentInExtruderErrorCode)
	InvalidFilamentHubTempError           = FromCode(InvalidFilamentHubTempErrorCode)
	InvalidFilamentHubDurationError       = FromCode(InvalidFilamentHubDurationErrorCode)
	InvalidFilamentHubFanSpeedError       = FromCode(InvalidFilamentHubFanSpeedErrorCode)
	InvalidFilamentHubTypeError           = FromCode(InvalidFilamentHubTypeErrorCode)
	WaitFilamentHubTimeoutError           = FromCode(WaitFilamentHubTimeoutErrorCode)
	UnableToEditFilamentInfoError         = FromCode(UnableToEditFilamentInfoErrorCode)
	FilamentHubFeedFilamentTimeoutError   = FromCode(FilamentHubFeedFilamentTimeoutErrorCode)
	FilamentHubUnwindFilamentTimeoutError = FromCode(FilamentHubUnwindFilamentTimeoutErrorCode)
	FilamentHubWaitReadyTimeoutError      = FromCode(FilamentHubWaitReadyTimeoutErrorCode)
	FilamentHubWaitActionTimeoutError     = FromCode(FilamentHubWaitActionTimeoutErrorCode)
	FilamentHubCommunicationError         = FromCode(FilamentHubCommunicationErrorCode)
	ExtruderMustHeatUpFirstError          = FromCode(ExtruderMustHeatUpFirstErrorCode)
	FilamentNotPresentInExtruderError     = FromCode(FilamentNotPresentInExtruderErrorCode)
	FilamentHubIsEmptyError               = FromCode(FilamentHubIsEmptyErrorCode)
	FilamentIsStuckError                  = FromCode(FilamentIsStuckErrorCode)
	FilamentIsTangledError                = FromCode(FilamentIsTangledErrorCode)
	FilamentRunoutError                   = FromCode(FilamentRunoutErrorCode)
	FilamentHubGearStuckError             = FromCode(FilamentHubGearStuckErrorCode)
	FilamentHubDisconnectedError          = FromCode(FilamentHubDisconnectedErrorCode)
	NotEnoughFilamentsInHubError          = FromCode(NotEnoughFilamentsInHubErrorCode)
	GcodeFileNotExistTCmdError            = FromCode(GcodeFileNotExistTCmdErrorCode)

	MotorWiringError = FromCode(MotorWiringErrorCode)
)

type Error struct {
	Typ     string `json:"error"`
	Code    Code   `json:"code"`
	Message string `json:"message"`
}

func New(code Code, typ, message string) *Error {
	return &Error{
		Typ:     typ,
		Code:    code,
		Message: message,
	}
}

func (e *Error) Error() string {
	return fmt.Sprintf("error: typ = %s, code = %s, message = %s", e.Typ, string(e.Code), e.Message)
}

func NewWebRequestError(code Code, message string) *Error {
	return &Error{Typ: "WebRequestError", Code: code, Message: message}
}

func FromError(err error) *Error {
	switch e := err.(type) {
	case *Error:
		return e
	default:
		return NewWebRequestError(UnknownCode, e.Error())
	}
}

func FromCode(code Code) *Error {
	return NewWebRequestError(code, code.String())
}

func (code Code) String() string {
	if msg, ok := codeMessageMaps[code]; ok {
		return msg
	}
	return "code(" + string(code) + ")"
}
