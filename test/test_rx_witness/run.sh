#!/bin/sh
# Build and run the RX overflow-witness test on the host (no PlatformIO).
set -e
cd "$(dirname "$0")"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -o test_rx_witness test_rx_witness.c
./test_rx_witness
