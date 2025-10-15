#!/bin/bash
set -x

opt="-Wall -g -O2 -shared -fPIC -flto -fwhole-program -fno-use-linker-plugin"
TARGET="$PWD/project/chelper/libc_helper.so"

if [ -f "$TARGET" ]; then
    ARCH=$(file "$TARGET" | grep -o 'ELF [^,]*' | awk '{print $2}')
    if [[ "$ARCH" == "ARM" ]]; then
        echo "Is ARM: $TARGET"
        exit 0
    fi
fi

rm -rf ./libc_helper.so
$ARM_GCC $opt -o $PWD/chelper/libc_helper.so $PWD/chelper/*.c
mv $PWD/chelper/libc_helper.so $PWD/project/chelper/