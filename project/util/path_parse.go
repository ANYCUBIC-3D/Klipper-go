package util

import (
	"bufio"
	"os"
	"path/filepath"
	"strings"
)

func ExpandUser(path string) string {
	if len(path) == 0 || path[0] != '~' {
		return path
	}

	slashIndex := strings.Index(path, "/")
	if slashIndex == -1 {
		slashIndex = len(path)
	}

	username := path[1:slashIndex]
	var homedir string

	switch {
	case username == "":
		if home, err := os.UserHomeDir(); err == nil {
			homedir = home
		}
	default:
		if home := lookupLinuxUser(username); home != "" {
			homedir = home
		}
	}

	if homedir == "" {
		return path
	}

	return filepath.Join(homedir, path[slashIndex:])
}

func lookupLinuxUser(username string) string {
	file, err := os.Open("/etc/passwd")
	if err != nil {
		return ""
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		if strings.HasPrefix(line, "#") {
			continue
		}
		parts := strings.Split(line, ":")
		if len(parts) > 5 && parts[0] == username {
			return parts[5]
		}
	}
	return ""
}

func Normpath(s string) string {
	return filepath.Clean(s)
}
