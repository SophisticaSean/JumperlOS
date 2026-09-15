#!/bin/sh
# Build and run the MicroPython heap-rung ladder test on the host (no PlatformIO).
set -e
cd "$(dirname "$0")"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -o test_mp_rung test_mp_rung.c
./test_mp_rung
