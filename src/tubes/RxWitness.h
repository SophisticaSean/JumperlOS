// SPDX-License-Identifier: MIT
// The RX-ring overflow witness, kept free of pico-sdk so test/test_rx_witness
// can run it on the host. `total` is the unmasked byte count the DMA has
// written so far - from TRANS_COUNT on the RP2040, or accumulated from the
// masked write_addr delta on the RP2350 (rx_witness_total_rp2350).
#pragma once
#include <stdint.h>

// Bytes that arrived since the last call; bumps the counters when more
// arrived than the ring had free (an overflow), by whole laps when it was
// that bad.
static inline uint32_t rx_witness_step( uint32_t total, uint32_t* lastTotal, uint16_t freeBytes,
                                        uint32_t ringSize, volatile uint32_t* overflows,
                                        volatile uint32_t* laps ) {
    uint32_t arrived = total - *lastTotal;
    *lastTotal = total;
    if ( arrived > freeBytes ) {
        ( *overflows )++;
        *laps += arrived / ringSize;
    }
    return arrived;
}

// RP2350 (ENDLESS DMA, TRANS_COUNT never moves): advance `total` by the
// write pointer's masked delta. An exact whole-lap alias reads as 0.
static inline uint32_t rx_witness_total_rp2350( uint32_t lastTotal, uint32_t wa, uint32_t* lastWa,
                                                uint32_t ringMask ) {
    uint32_t d = ( wa - *lastWa ) & ringMask;
    *lastWa = wa;
    return lastTotal + d;
}
