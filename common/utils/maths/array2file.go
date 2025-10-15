package maths

import (
	"bufio"
	"os"
	"strconv"
	"strings"
)

//func main(){
//	arr := [][]float64{{1,2,3}, {4,5,6}}
//
//	file, err := os.OpenFile("data.txt", os.O_WRONLY | os.O_TRUNC | os.O_CREATE, 0666)
//	if err != nil {
//		panic(err)
//	}
//	defer file.Close()
//	bw := bufio.NewWriter(file)
//	defer bw.Flush()
//	for _,row := range arr {
//		buStr := strings.Builder{}
//		for i,item := range row {
//			buStr.WriteString(strconv.FormatFloat(item, 'f', 10, 32))
//			if i != len(row) - 1 {
//				buStr.WriteString(" ")
//			}
//		}
//		buStr.WriteString("\n")
//		bw.WriteString(buStr.String())
//	}
//}
func Write_file(name string, arr [][]float64) {
	file, err := os.OpenFile(name, os.O_WRONLY|os.O_TRUNC|os.O_CREATE, 0666)
	if err != nil {
		panic(err)
	}
	defer file.Close()
	bw := bufio.NewWriter(file)
	defer bw.Flush()
	for _, row := range arr {
		buStr := strings.Builder{}
		for i, item := range row {
			buStr.WriteString(strconv.FormatFloat(item, 'f', 10, 32))
			if i != len(row)-1 {
				buStr.WriteString(" ")
			}
		}
		buStr.WriteString("\n")
		bw.WriteString(buStr.String())
	}
}

func Write_file_complex(name string, arr [][]complex128) {
	file, err := os.OpenFile(name, os.O_WRONLY|os.O_TRUNC|os.O_CREATE, 0666)
	if err != nil {
		panic(err)
	}
	defer file.Close()
	bw := bufio.NewWriter(file)
	defer bw.Flush()
	for _, row := range arr {
		buStr := strings.Builder{}
		for i, item := range row {
			buStr.WriteString(strconv.FormatComplex(item, 'f', 10, 128))
			if i != len(row)-1 {
				buStr.WriteString(" ")
			}
		}
		buStr.WriteString("\n")
		bw.WriteString(buStr.String())
	}
}
