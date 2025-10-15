package config

import (
	"encoding/json"
	"errors"
	"k3c/common/file"
	"log"
	"os"
)

const paraFile = "/userdata/app/gk/config/para.cfg"

func LedDetectEnable() bool {
	para := readParaFile(paraFile)
	if para != nil &&
		para["ParaConfig"] != nil {
		conf, _ := para["ParaConfig"].(map[string]interface{})
		return conf["Led_detect"] == "on"
	}
	return false
}

func SetLedDetect(enable bool) error {
	para := readParaFile(paraFile)
	if para == nil {
		return errors.New("read " + paraFile + "error")
	}

	conf, _ := para["ParaConfig"].(map[string]interface{})
	if enable {
		conf["Led_detect"] = "on"
	} else {
		conf["Led_detect"] = "off"
	}
	para["ParaConfig"] = conf
	return saveParaFile(paraFile, para)
}

func readParaFile(paraFile string) map[string]interface{} {
	content, err := os.ReadFile(paraFile)
	if err != nil {
		log.Printf("read para file error: %v", err)
		return nil
	}

	var para = map[string]interface{}{}
	err = json.Unmarshal(content, &para)
	if err != nil {
		log.Printf("unmarshal "+paraFile+" error: %v", err)
		return nil
	}

	return para
}

func saveParaFile(paraFile string, data map[string]interface{}) error {
	d, err := json.MarshalIndent(data, "", "\t")
	if err != nil {
		return err
	}

	return file.WriteFileWithSync(paraFile, d)
}
