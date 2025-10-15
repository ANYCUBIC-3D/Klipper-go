export GOOS="linux"
export CGO_ENABLED=1
export GOARCH="arm"
export CC=/home/liuxiaobo/gcc/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc

go mod download
go mod vendor
go mod tidy
cd ../
rm -rf ./chelper/libc_helper.so
rm -rf ./project/chelper/libc_helper.so
./chelper/build_chelper_arm.sh
cd build
go build -v -x -ldflags "-s -w" -o gklib ../main/K3cMain.go
