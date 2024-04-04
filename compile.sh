#!/bin/sh

PROJECT_DIR="/home/developer/proj/velopera/repos/velopera-ublox-firmware"
if [ ! -d "$PROJECT_DIR/build" ]; then
    mkdir -p "$PROJECT_DIR/build" && cd build
    cmake -G Ninja -DPYTHON_DEPS_CHECKED=1 -DESP_PLATFORM=1 -DIDF_TARGET=esp32 -DCCACHE_ENABLE=0 $PROJECT_DIR
else
    cd build
fi

ninja all