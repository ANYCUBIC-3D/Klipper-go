package str

import (
	"fmt"
	"reflect"
	"regexp"
	"strings"
	"unsafe"
)

func String2Bytes(str string) []byte {

	return *(*[]byte)(unsafe.Pointer(&str))
}

func Bytes2String(bs []byte) string {

	return *(*string)(unsafe.Pointer(&bs))
}

func Strip(s_ string, chars_ string) string {
	s, chars := []rune(s_), []rune(chars_)
	length := len(s)
	max := len(s) - 1
	l, r := true, true
	start, end := 0, max
	tmpEnd := 0
	charset := make(map[rune]bool)
	for i := 0; i < len(chars); i++ {
		charset[chars[i]] = true
	}
	for i := 0; i < length; i++ {
		if _, exist := charset[s[i]]; l && !exist {
			start = i
			l = false
		}
		tmpEnd = max - i
		if _, exist := charset[s[tmpEnd]]; r && !exist {
			end = tmpEnd
			r = false
		}
		if !l && !r {
			break
		}
	}
	if l && r {
		return ""
	}
	return string(s[start : end+1])
}

// LastName "hello world" => world , "hello" => hello
func LastName(name string) string {
	segments := strings.Split(strings.TrimSpace(name), " ")
	return segments[len(segments)-1]
}

func MapStringKeys(m interface{}) []string {
	if m == nil {
		return nil
	}
	typ := reflect.TypeOf(m)
	if typ.Kind() != reflect.Map {
		return nil
	}

	v := reflect.ValueOf(m)
	var keys []string
	for _, key := range v.MapKeys() {
		if key.Kind() != reflect.String {
			return nil
		}
		keys = append(keys, key.String())
	}
	return keys
}

func JoinSliceWithFormat(elems interface{}, sep, format string) string {
	if elems == nil {
		return ""
	}

	typ := reflect.TypeOf(elems)
	if typ.Kind() != reflect.Slice {
		return ""
	}

	val := reflect.ValueOf(elems)
	switch val.Len() {
	case 0:
		return ""
	case 1:
		return fmt.Sprintf(format, val.Index(0).Interface())
	}

	var strs = fmt.Sprintf(format, val.Index(0).Interface())
	for i := 1; i < val.Len(); i++ {
		strs += sep + fmt.Sprintf(format, val.Index(i).Interface())
	}

	return strs
}

func MergeSlice(s []string, elems ...[]string) []string {
	if len(elems) == 0 {
		return s
	}

	for _, elem := range elems {
		s = append(s, elem...)
	}
	return s
}

var alphanumExp = regexp.MustCompile(`^[a-zA-Z0-9]*$`)

func IsAlphanum(word string) bool {
	return alphanumExp.MatchString(word)
}
