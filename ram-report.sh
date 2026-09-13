#!/bin/sh
# RAM image of the OG build: arm-none-eabi-size + readelf __end__ (PERF_PLAN.md G3).
# Pre-flash gate for the 56 KB MicroPython rung: __end__ <= 0x20024000.
set -e
ELF=.pio/build/jumperless_og/firmware.elf
BIN=$(dirname "$(ls ~/.platformio/packages/*/bin/arm-none-eabi-nm | head -1)")
END=$("$BIN/arm-none-eabi-nm" "$ELF" | awk '$3=="__end__"{print $1}')
DATA=$("$BIN/arm-none-eabi-readelf" -S "$ELF" | awk '$2==".data" && $4 ~ /^2000/{print strtonum("0x"$6)}')
BSS=$("$BIN/arm-none-eabi-readelf" -S "$ELF" | awk '$2==".bss"{print strtonum("0x"$6)}')
GATE=$(( 0x20024000 ))
E=$(( 0x$END ))
FREE=$(( 0x20040000 - E ))
STATUS="ABOVE gate by $(( E - GATE )) B"
[ "$E" -le "$GATE" ] && STATUS="under gate by $(( GATE - E )) B"
echo "$(git rev-parse --short HEAD) __end__=0x$END RAM.data=$DATA .bss=$BSS c_heap=$FREE  56KB-rung gate 0x20024000: $STATUS"
