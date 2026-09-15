#!/bin/sh
# Build and run the pair_str parser test on the host (no PlatformIO).
set -e
cd "$(dirname "$0")"
cc -std=c11 -Wall -Wextra -Werror -O1 -o test_pair_str test_pair_str.c
./test_pair_str
