package LiteralEval

import (
	"fmt"
	"strconv"
	"strings"
)

func LiteralEval(s string) (interface{}, error) {
	s = strings.TrimSpace(s)

	if s == "" {
		return "", nil
	}

	switch s {
	case "True", "true":
		return true, nil
	case "False", "false":
		return false, nil
	case "None", "null":
		return nil, nil
	}

	if num, err := strconv.ParseInt(s, 10, 64); err == nil {
		return num, nil
	}
	if num, err := strconv.ParseFloat(s, 64); err == nil {
		return num, nil
	}

	if len(s) >= 2 && ((s[0] == '\'' && s[len(s)-1] == '\'') ||
		(s[0] == '"' && s[len(s)-1] == '"')) {
		return s[1 : len(s)-1], nil
	}

	if s[0] == '[' && s[len(s)-1] == ']' {
		return parseList(s)
	}

	if s[0] == '{' && s[len(s)-1] == '}' {
		return parseDict(s)
	}

	if s[0] == '(' && s[len(s)-1] == ')' {
		return parseTuple(s)
	}

	return s, nil
}

func parseList(s string) ([]interface{}, error) {
	s = strings.TrimSpace(s[1 : len(s)-1])
	if s == "" {
		return []interface{}{}, nil
	}

	var result []interface{}
	items := splitByComma(s)

	for _, item := range items {
		val, err := LiteralEval(item)
		if err != nil {
			return nil, err
		}
		result = append(result, val)
	}

	return result, nil
}

func parseDict(s string) (map[string]interface{}, error) {
	s = strings.TrimSpace(s[1 : len(s)-1])
	if s == "" {
		return map[string]interface{}{}, nil
	}

	result := make(map[string]interface{})
	items := splitByComma(s)

	for _, item := range items {
		parts := strings.SplitN(item, ":", 2)
		if len(parts) != 2 {
			return nil, fmt.Errorf("invalid dictionary item: %s", item)
		}

		key, err := LiteralEval(strings.TrimSpace(parts[0]))
		if err != nil {
			return nil, err
		}

		value, err := LiteralEval(strings.TrimSpace(parts[1]))
		if err != nil {
			return nil, err
		}

		if keyStr, ok := key.(string); ok {
			result[keyStr] = value
		} else {
			return nil, fmt.Errorf("dictionary key must be string, got %T", key)
		}
	}

	return result, nil
}

func parseTuple(s string) ([]interface{}, error) {
	s = strings.TrimSpace(s[1 : len(s)-1])
	if s == "" {
		return []interface{}{}, nil
	}

	if !strings.Contains(s, ",") {
		val, err := LiteralEval(s)
		if err != nil {
			return nil, err
		}
		return []interface{}{val}, nil
	}

	return parseList("[" + s + "]")
}

func splitByComma(s string) []string {
	var result []string
	var start int
	braceLevel := 0

	for i, r := range s {
		switch r {
		case '[', '{', '(':
			braceLevel++
		case ']', '}', ')':
			braceLevel--
		case ',':
			if braceLevel == 0 {
				result = append(result, strings.TrimSpace(s[start:i]))
				start = i + 1
			}
		}
	}

	if start < len(s) {
		result = append(result, strings.TrimSpace(s[start:]))
	}

	return result
}
