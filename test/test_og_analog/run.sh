#!/usr/bin/env bash
# Host build of the OG analog transfer-constant test. No hardware, no PlatformIO.
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD_DIR:-/tmp/test_og_analog}
mkdir -p "$BUILD"
${CXX:-g++} -std=gnu++17 ${CXXFLAGS:--O1} -DOG_JUMPERLESS -Isrc \
    test/test_og_analog/test_og_analog.cpp \
    src/boards/board.cpp src/boards/v5/board_v5.cpp src/boards/og/board_og.cpp \
    -o "$BUILD/test_og_analog"
"$BUILD/test_og_analog" "$@"
