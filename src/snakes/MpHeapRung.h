// SPDX-License-Identifier: MIT
// The MicroPython GC-heap rung ladder, kept free of Arduino/pico so
// test/test_mp_rung can run it on the host. Only the ladder and the
// admissibility test live here: mpAllocHeap (Python_Proper.cpp) still
// attempts the real malloc on every rung and falls through on NULL - free
// total != largest contiguous block, and that fall-through is the fix for
// the nlr_jump_fail boot loop. Do not move the malloc loop into here.
#pragma once
#include <stddef.h>

// The OG's configured rung and the lowest rung the RAM gate (ram-report.sh)
// tolerates before it hard-fails the build. Kept as plain literals so a
// shell grep can read them by name; JumperlessDefines.h is NOT the source
// (the V5's 64 KB sibling sits next to the OG's 56 under a board #if).
#define MP_RUNG_OG_CONFIGURED_KB 56
#define MP_RUNG_FLOOR_KB 40

// Rung i of the ladder for a configured size, 0 past the end. The
// configured-8K and 40 K rungs make a build a few KB short of its rung land
// one step down, never two (56 -> 48 -> 40).
static inline size_t mpNextRung( size_t configured, size_t i ) {
    const size_t fixed[] = { 48 * 1024, 40 * 1024, 32 * 1024, 24 * 1024, 16 * 1024 };
    if ( i == 0 ) return configured;
    if ( i == 1 ) return configured > 8 * 1024 ? configured - 8 * 1024 : 0;
    i -= 2;
    return i < sizeof( fixed ) / sizeof( fixed[ 0 ] ) ? fixed[ i ] : 0;
}

// May this rung be tried: never above the configured size, and it must leave
// the C-heap reserve behind. A true here still has to survive the malloc.
static inline int mpRungFits( size_t sz, size_t configured, size_t reserve, size_t freeHeap ) {
    if ( sz == 0 || sz > configured ) return 0;
    return sz + reserve <= freeHeap;
}
