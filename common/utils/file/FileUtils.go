package file

import (
	"io/ioutil"
	"os"
	"path/filepath"
	"strings"
	"time"
)

func GetBytes(fileName string) ([]byte, error) {
	bs, err := ioutil.ReadFile(fileName)

	return bs, err
}

func PathExists(path string) (bool, error) {
	_, err := os.Stat(path)
	if err == nil {
		return true, nil
	}
	if os.IsNotExist(err) {
		return false, nil
	}
	return false, err
}

func GetMtime(fileName string) (float64, error) {
	state, err := os.Stat(fileName)
	if err == nil {
		return float64(state.ModTime().Unix()), nil
	}
	if os.IsNotExist(err) {
		return 0, nil
	}
	return 0, err
}

func GetBackFileNameWhileNoExist(oldFileName string) string {
	//_, err := os.Stat(oldFileName)
	//if err != nil {
	//	return oldFileName
	//}
	dir, fileName := filepath.Split(oldFileName)
	if dir == "" {
		dir = "./"
	}
	fileName = strings.Split(fileName, ".")[0]
	files, err := ioutil.ReadDir(dir)
	if err != nil {
		return ""
	}
	var modTime time.Time
	var names []string
	for _, fi := range files {
		fn := fi.Name()
		if fi.Mode().IsRegular() && strings.Index(fn, fileName) != -1 {
			//log.Println(fn, fi.ModTime(), modTime)
			if !fi.ModTime().Before(modTime) {
				if fi.ModTime().After(modTime) {
					modTime = fi.ModTime()
					names = names[:0]
				}
				names = append(names, fn)

			}
		}
	}
	if len(names) > 0 {
		return strings.Join([]string{dir, names[0]}, "")
	}
	return ""
}
func DeleteBackFileNameWhileMore(oldFileName string, subName string, size int) error {
	dir, fileName := filepath.Split(oldFileName)
	if dir == "" {
		dir = "./"
	}
	//log.Println("oldFileName", oldFileName, dir, fileName)
	fileName = strings.Split(fileName, ".")[0]
	files, err := ioutil.ReadDir(dir)
	if err != nil {
		return err
	}
	//log.Println("DeleteBackFileNameWhileMore", dir, fileName, len(files))
	count := 0
	for _, fi := range files {
		fn := fi.Name()
		if strings.HasSuffix(fn, strings.Join([]string{fileName, ".", subName}, "")) || strings.HasSuffix(fn, strings.Join([]string{fileName, ".", subName, ".tmp"}, "")) {
			//pass while is self
			continue
		}
		if fi.Mode().IsRegular() && strings.Index(fn, fileName) != -1 {
			count++
			if count > size {
				deleteFileName := filepath.Join(dir, fn)
				//log.Println(deleteFileName)
				os.Remove(deleteFileName)
			}
		}
	}

	return nil
}
