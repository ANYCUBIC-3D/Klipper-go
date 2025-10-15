#!/bin/bash
set -x
CC=gcc
opt="-Wall -g -O2 -shared -fPIC -flto -fwhole-program -fno-use-linker-plugin"
TARGET="$PWD/project/chelper/libc_helper.so"

if [ -f "$TARGET" ]; then
    ARCH=$(file "$TARGET" | grep -o 'ELF [^,]*' | awk '{print $2}')
    if [[ "$ARCH" == "32-bit" ]]; then
        echo "Is 32-bit: $TARGET"
        exit 0
    fi
fi

rm -rf ./libc_helper.so
$CC $opt -o $PWD/chelper/libc_helper.so $PWD/chelper/*.c
mv $PWD/chelper/libc_helper.so $PWD/project/chelper/