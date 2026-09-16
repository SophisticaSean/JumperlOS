#!/bin/sh
# RAM image of the OG build: arm-none-eabi-size + readelf __end__.
#
# Two gates, both against the static end of .bss (__end__), with the C heap
# and the MicroPython GC heap carved from what is left up to 0x20040000:
#   GATE_56  = 0x20024000  the one calibrated literal: the highest __end__ at
#              which the configured 56 KB GC rung still fits after the stack
#              and the boot-time C-heap allocations (measured 2026-09-12).
#              Above it the board lands on a lower rung - a WARNING.
#   FLOOR_40 = GATE_56 + (RUNG_56 - RUNG_40) KB - above this even the 40 KB
#              rung cannot fit, so the build hard-FAILS (exit 1). The rung
#              sizes are read by name from src/snakes/MpHeapRung.h; do NOT
#              grep JumperlessDefines.h (the V5's 64 KB sibling sits next to
#              the OG's 56 under a board #if).
# Plain sh arithmetic and awk - no gawk (strtonum), works on mawk/BSD awk.
#   RAM_GATE=hard  also hard-fail above GATE_56 (the fork's own CI does this;
#                  upstream CI only warns there).
set -e
ELF=${ELF:-.pio/build/jumperless_og/firmware.elf}
BIN=$(dirname "$(ls "$HOME"/.platformio/packages/toolchain-rp2040-earlephilhower/bin/arm-none-eabi-nm 2>/dev/null || ls "$HOME"/.platformio/packages/*/bin/arm-none-eabi-nm | head -1)")
END=$("$BIN/arm-none-eabi-nm" "$ELF" | awk '$3=="__end__"{print $1}')
# readelf prints "[ 7] .data" - the space inside the brackets shifts the
# fields for indexes < 10, so normalise it before picking columns.
SECS=$("$BIN/arm-none-eabi-readelf" -S "$ELF" | sed 's/\[ *\([0-9]*\)\]/[\1]/')
DATA=$(printf '%s\n' "$SECS" | awk '$2==".data" && $4 ~ /^2000/{print $6}')
BSS=$(printf '%s\n' "$SECS" | awk '$2==".bss"{print $6}')
RUNG_56=$(sed -n 's/^#define MP_RUNG_OG_CONFIGURED_KB \([0-9]*\).*/\1/p' src/snakes/MpHeapRung.h)
RUNG_40=$(sed -n 's/^#define MP_RUNG_FLOOR_KB \([0-9]*\).*/\1/p' src/snakes/MpHeapRung.h)
for v in END DATA BSS RUNG_56 RUNG_40; do
  eval "x=\$$v"; [ -n "$x" ] || { echo "ram-report.sh: could not read $v"; exit 2; }
done
GATE=$(( 0x20024000 ))
FLOOR=$(( GATE + (RUNG_56 - RUNG_40) * 1024 ))
E=$(( 0x$END ))
FREE=$(( 0x20040000 - E ))
DATA_B=$(( 0x$DATA )); BSS_B=$(( 0x$BSS ))
if [ "$E" -le "$GATE" ]; then STATUS="under gate by $(( GATE - E )) B"; else STATUS="ABOVE gate by $(( E - GATE )) B"; fi
echo "$(git rev-parse --short HEAD) __end__=0x$END RAM.data=$DATA_B .bss=$BSS_B c_heap=$FREE  ${RUNG_56}KB-rung gate 0x20024000: $STATUS; ${RUNG_40}KB-rung floor $(printf '0x%x' $FLOOR)"
if [ "$E" -gt "$FLOOR" ]; then echo "FAIL: __end__ above the ${RUNG_40} KB rung floor - the MicroPython heap cannot fit"; exit 1; fi
if [ "$E" -gt "$GATE" ]; then
  echo "::warning::OG __end__ 0x$END is above the ${RUNG_56} KB rung gate 0x20024000 by $(( E - GATE )) B - the board will boot on a lower GC rung"
  [ "${RAM_GATE:-warn}" = hard ] && exit 1
fi
exit 0
