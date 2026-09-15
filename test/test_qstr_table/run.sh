#!/bin/sh
# Build and run the QSTR-table check on the host (no PlatformIO).
set -e
cd "$(dirname "$0")"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -o test_qstr_table test_qstr_table.c
./test_qstr_table ../../lib/micropython/micropython_embed/genhdr/qstrdefs.generated.h
