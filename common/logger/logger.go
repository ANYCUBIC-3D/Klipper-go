package logger

import (
	"fmt"
	"log"
	"os"

	"github.com/natefinch/lumberjack"
	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"
)

var (
	Logger *zap.Logger
)

type LogLevel int8

const (
	DebugLevel LogLevel = iota - 1
	InfoLevel
	WarnLevel
	ErrorLevel
)
const SUPPORT_COLOR = true

func newEncoderSupportColor() zapcore.Encoder {

	encoderConfig := zapcore.EncoderConfig{
		MessageKey:       "message",
		LevelKey:         "level",
		TimeKey:          "time",
		CallerKey:        "caller",
		EncodeLevel:      zapcore.CapitalColorLevelEncoder,
		EncodeTime:       zapcore.ISO8601TimeEncoder,
		EncodeCaller:     zapcore.ShortCallerEncoder,
		ConsoleSeparator: " ",
	}

	return zapcore.NewConsoleEncoder(encoderConfig)
}

func newEncoderSupportCapital() zapcore.Encoder {
	encoderConfig := zapcore.EncoderConfig{
		MessageKey:       "message",
		LevelKey:         "level",
		TimeKey:          "time",
		CallerKey:        "caller",
		EncodeLevel:      zapcore.CapitalLevelEncoder,
		EncodeTime:       zapcore.ISO8601TimeEncoder,
		EncodeCaller:     zapcore.ShortCallerEncoder,
		ConsoleSeparator: " ",
	}

	return zapcore.NewConsoleEncoder(encoderConfig)
}

func newConsoleCore(encoder zapcore.Encoder, level zapcore.Level) zapcore.Core {
	return zapcore.NewCore(encoder, zapcore.Lock(os.Stdout), level)
}

func newFileCore(encoder zapcore.Encoder, level zapcore.Level, logfile string, maxSize, maxBackups, maxAge int) zapcore.Core {
	logFile := &lumberjack.Logger{
		Filename:   logfile,
		MaxSize:    maxSize,
		MaxBackups: maxBackups,
		MaxAge:     maxAge,
		Compress:   false,
		LocalTime:  true,
	}

	return zapcore.NewCore(encoder, zapcore.AddSync(logFile), level)
}

func InitLogger(level LogLevel, logfile string, supportColor bool, maxSize, maxBackups, maxAge int) {
	var encoder zapcore.Encoder
	if supportColor {
		encoder = newEncoderSupportColor()
	} else {
		encoder = newEncoderSupportCapital()
	}

	consoleCore := newConsoleCore(encoder, zapcore.Level(level))
	fileCore := newFileCore(encoder, zapcore.Level(level), logfile, maxSize, maxBackups, maxAge)
	core := zapcore.NewTee(consoleCore, fileCore)
	Logger = zap.New(core, zap.AddCaller(), zap.AddCallerSkip(1))
}

func Sync() {
	if Logger != nil {
		err := Logger.Sync()
		if err != nil {
			log.Fatalf("failed to sync logger: %v", err)
		}
	}
}

func Infof(format string, args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Infof(format, args...)
	}
}

func Info(args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Info(args...)
	}
}

func Debugf(format string, args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Debugf(format, args...)
	}
}

func Debug(args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Debug(args...)
	}
}

func Warnf(format string, args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Warnf(format, args...)
	}
}

func Warn(args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Warn(args...)
	}
}

func Errorf(format string, args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Errorf(format, args...)
	}
}

func Error(args ...interface{}) {
	if Logger != nil {
		Logger.Sugar().Error(args...)
	}
}

func Panicf(format string, args ...interface{}) {
	if Logger != nil {
		message := fmt.Sprintf(format, args...)
		Logger.Panic(message)
		Logger.Sync()
		panic(message)
	}
}

func Panic(args ...interface{}) {
	if Logger != nil {
		message := fmt.Sprint(args...)
		Logger.Panic(message)
		Logger.Sync()
		panic(message)
	}
}

func Fatalf(format string, args ...interface{}) {
	if Logger != nil {
		message := fmt.Sprintf(format, args...)
		Logger.Fatal(message)
		Logger.Sync()
		os.Exit(1)
	}
}

func Fatal(args ...interface{}) {
	if Logger != nil {
		message := fmt.Sprint(args...)
		Logger.Fatal(message)
		Logger.Sync()
		os.Exit(1)
	}
}
