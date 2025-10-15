package cast

import (
	"fmt"
	"k3c/common/logger"
	"math"
	"reflect"
	"runtime"
	"strconv"
)

func ToFloat64(i interface{}) float64 {
	v, _ := ToFloat64E(i)
	return v
}

func ToFloat64E(i interface{}) (float64, error) {
	switch v := i.(type) {
	case float64:
		return v, nil
	case float32:
		return float64(v), nil
	case int8, int16, int32, int64, int:
		return float64(reflect.ValueOf(v).Int()), nil
	case uint8, uint16, uint32, uint64, uint:
		return float64(reflect.ValueOf(v).Uint()), nil
	case string:
		return strconv.ParseFloat(v, 64)
	default:
		return 0, fmt.Errorf("cast %#v of type %T to float64 fail", i, i)
	}
}

func ToInt(i interface{}) int {
	v, _ := ToIntE(i)
	return v
}

func ToIntE(i interface{}) (int, error) {
	switch v := i.(type) {
	case int:
		return v, nil
	case float32, float64:
		return int(reflect.ValueOf(v).Float()), nil
	case int8, int16, int32, int64:
		return int(reflect.ValueOf(v).Int()), nil
	case uint8, uint16, uint32, uint64:
		return int(reflect.ValueOf(v).Uint()), nil
	case string:
		return strconv.Atoi(v)
	case bool:
		if v {
			return 1, nil
		}
		return 0, nil
	default:
		return 0, fmt.Errorf("cast %#v of type %T to int fail", i, i)
	}
}

func ToString(i interface{}) string {
	v, _ := ToStringE(i)
	return v
}

func ToStringE(i interface{}) (string, error) {
	switch v := i.(type) {
	case string:
		return v, nil
	case int8, int16, int32, int64, int, uint8, uint16, uint32, uint64:
		return fmt.Sprintf("%d", v), nil
	case bool:
		return strconv.FormatBool(v), nil
	case float32:
		return strconv.FormatFloat(float64(v), 'f', -1, 32), nil
	case float64:
		return strconv.FormatFloat(v, 'f', -1, 64), nil
	case []byte:
		return string(v), nil
	case nil:
		return "", nil
	case fmt.Stringer:
		if reflect.ValueOf(i).Kind() == reflect.Ptr && reflect.ValueOf(i).IsNil() {
			return "", nil
		}
		return v.String(), nil
	case error:
		return v.Error(), nil
	default:
		return "", fmt.Errorf("cast %#v of type %T to string fail", i, i)
	}
}

func ToStringSlice(i interface{}) []string {
	v, err := ToStringSliceE(i)
	if err != nil {
		logger.Errorf("cast %#v of type %T to string fail", i, i)
	}
	return v
}

func ToStringSliceE(i interface{}) ([]string, error) {
	switch v := i.(type) {
	case []string:
		return v, nil
	case []interface{}:
		var result []string
		for _, item := range v {
			str, ok := item.(string)
			if !ok {
				return nil, fmt.Errorf("item %#v of type %T is not a string", item, item)
			}
			result = append(result, str)
		}
		return result, nil
	}

	return nil, fmt.Errorf("cast %#v of type %T to []string fail", i, i)
}

func ToBool(i interface{}) bool {
	v, _ := ToBoolE(i)
	return v
}

func ToBoolE(i interface{}) (bool, error) {
	switch v := i.(type) {
	case bool:
		return v, nil
	case int8, int16, int32, int64, int:
		return reflect.ValueOf(v).Int() == 1, nil
	case uint8, uint16, uint32, uint64, uint:
		return reflect.ValueOf(v).Uint() == 1, nil
	case string:
		return strconv.ParseBool(v)
	default:
		return false, fmt.Errorf("cast %#v of type %T to []string fail", i, i)
	}
}

func IntP(v int) *int {
	return &v
}

func Int64P(v int64) *int64 {
	return &v
}

func Float32P(v float32) *float32 {
	return &v
}

func Float64P(v float64) *float64 {
	return &v
}

func BoolP(v bool) *bool {
	return &v
}

func StringP(v string) *string {
	return &v
}

func Float64(v *float64) float64 {
	if v == nil {
		return math.NaN()
	}

	return *v
}

func Int(v *int) int {
	if v == nil {
		return 0
	}
	return *v
}

func String(v *string) string {
	if v == nil {
		return ""
	}
	return *v
}

func ToInt64(i interface{}) int64 {
	v, _ := ToInt64E(i)
	return v
}

func ToInt64E(i interface{}) (int64, error) {
	switch v := i.(type) {
	case int64:
		return v, nil
	case float32, float64:
		return int64(reflect.ValueOf(v).Float()), nil
	case int8, int16, int32, int:
		return reflect.ValueOf(v).Int(), nil
	case uint8, uint16, uint32, uint64:
		return int64(reflect.ValueOf(v).Uint()), nil
	case string:
		return strconv.ParseInt(v, 10, 64)
	case bool:
		if v {
			return 1, nil
		}
		return 0, nil
	default:
		return 0, fmt.Errorf("cast %#v of type %T to int64 fail", i, i)
	}
}

func ForceInt(v int64) int {
	if strconv.IntSize == 32 && v > math.MaxInt32 {
		pc, file, line, ok := runtime.Caller(1)
		if ok {
			funcName := runtime.FuncForPC(pc).Name()
			logger.Debugf("force convert %+v to int overflow, file: %s, line: %d, func: %s", v, file, line, funcName)
		} else {
			logger.Debugf("force convert %+v to int overflow:", v)
		}
	}
	return int(v)
}
