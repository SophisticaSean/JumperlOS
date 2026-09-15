#!/bin/sh
# Build and run the slot auto-save gate test on the host (no PlatformIO).
set -e
cd "$(dirname "$0")"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -o test_slot_backstop test_slot_backstop.c
./test_slot_backstop
