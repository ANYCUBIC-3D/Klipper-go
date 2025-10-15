
export CGO_ENABLED=1
export GOARCH="386"

go mod download
go mod vendor
cd ../
rm -rf ./chelper/libc_helper.so
rm -rf ./project/chelper/libc_helper.so
./chelper/build_chelper_386.sh
cd build
go build -v -x -ldflags "-s -w" -o gklib ../main/K3cMain.go
