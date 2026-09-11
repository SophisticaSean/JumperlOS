#!/usr/bin/env bash
# Host build of the OG router dry-run test. No hardware, no PlatformIO.
#
# The router is copied into the build dir because quote-includes resolve
# relative to the including file first: compiled in place it would pull the
# real States.h / NetManager.h (Arduino, YAMLDuino, ...) instead of the shims.
# The real MatrixState.h, NetsToChipConnections.h, JumperlessDefines.h and the
# board descriptors are used as-is; `nano` is lifted out of MatrixState.cpp.
set -euo pipefail
cd "$(dirname "$0")/../.."
BUILD=${BUILD_DIR:-/tmp/test_og_router}
mkdir -p "$BUILD"
cp src/routing/NetsToChipConnections_OG.cpp "$BUILD/NetsToChipConnections_OG.cpp"
awk '/^struct nanoStatus nano = \{/{p=1} p{print} p&&/^  \};/{exit}' src/routing/MatrixState.cpp > "$BUILD/nano_init.inc"
# -Os: the firmware's optimization level (some of the bugs this test guards
# only show under optimization, e.g. a 0xFF bool compared == true).
#
# Narrowing check (the OG stores path/net fields in int8_t/uint8_t): build
# with clang's implicit-conversion sanitizer and run the sweeps - any value
# that no longer fits its field is reported at the store:
#   CXX=clang++ CXXFLAGS="-O1 -g -fsanitize=undefined,implicit-conversion,integer \
#     -fno-sanitize=enum,unsigned-shift-base -fsanitize-recover=all" \
#     BUILD_DIR=/tmp/og_ubsan test/test_og_router/run.sh
#   /tmp/og_ubsan/test_og_router rand 1 2000
# (`enum` is excluded: clearAllNTCC memsets paths[] to -1, so nodeType[] holds
# 0xFF until the router fills it - pre-existing, and every board does it.)
#
# Routing digest (compare two builds crosspoint for crosspoint):
#   OG_ROUTER_DIGEST=1 ./test_og_router            # per fixed case
#   OG_ROUTER_DIGEST=1 ./test_og_router rand 1 2000 # per trial
# diff the outputs of the old and new tree: identical = identical routing.
${CXX:-g++} -std=gnu++17 ${CXXFLAGS:--Os} -DOG_JUMPERLESS \
    -Itest/test_og_router/shim -Isrc -Isrc/routing -I"$BUILD" \
    "$BUILD/NetsToChipConnections_OG.cpp" src/boards/board.cpp src/boards/og/board_og.cpp \
    test/test_og_router/test_og_router.cpp -o "$BUILD/test_og_router"
"$BUILD/test_og_router" "$@"
