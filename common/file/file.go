package file

import "os"

func WriteFileWithSync(file string, data []byte) error {
	f, err := os.Create(file)
	if err != nil {
		return err
	}

	if _, err = f.Write(data); err != nil {
		return err
	}
	if err = f.Sync(); err != nil {
		return err
	}

	return f.Close()
}
