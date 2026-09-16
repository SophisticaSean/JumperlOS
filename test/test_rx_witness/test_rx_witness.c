// SPDX-License-Identifier: MIT
// Host test of src/tubes/RxWitness.h: the RP2040 unmasked witness catches a
// whole ring lap; the RP2350 masked one is documented to miss exactly that.
#include <stdio.h>
#include <stdint.h>
#include "../../src/tubes/RxWitness.h"

#define RING 2048u
#define MASK ( RING - 1 )
static int fails = 0;
#define CHECK( c ) do { if ( !( c ) ) { printf( "FAIL %s:%d %s\n", __FILE__, __LINE__, #c ); fails++; } } while ( 0 )

int main( void ) {
    uint32_t last = 0; volatile uint32_t ov = 0, laps = 0;
    // normal traffic that fits: no overflow
    CHECK( rx_witness_step( 100, &last, 500, RING, &ov, &laps ) == 100 );
    CHECK( ov == 0 && laps == 0 );
    // more than free: one overflow, no whole lap
    CHECK( rx_witness_step( 100 + 600, &last, 500, RING, &ov, &laps ) == 600 );
    CHECK( ov == 1 && laps == 0 );
    // RP2040: exactly one full lap between syncs is SEEN (the 0a5dc31 fix)
    CHECK( rx_witness_step( 700 + RING, &last, RING - 1, RING, &ov, &laps ) == RING );
    CHECK( ov == 2 && laps == 1 );
    // three laps and change
    rx_witness_step( last + 3 * RING + 7, &last, 10, RING, &ov, &laps );
    CHECK( ov == 3 && laps == 4 );
    // TRANS_COUNT wrap: 0xFFFFFFFF - remaining wraps through 0 and the delta still holds
    last = 0xFFFFFFF0u;
    CHECK( rx_witness_step( 0x00000010u, &last, RING - 1, RING, &ov, &laps ) == 0x20 );

    // RP2350: masked write-pointer delta
    uint32_t base = 0x20010000u, lastWa = base, total = 0;
    total = rx_witness_total_rp2350( total, base + 300, &lastWa, MASK );
    CHECK( total == 300 && lastWa == base + 300 );
    // wrap of the ring address
    total = rx_witness_total_rp2350( total, base + 100, &lastWa, MASK );
    CHECK( total == 300 + ( RING - 300 + 100 ) );
    // an exact whole-lap alias reads as 0 - the documented undercount, not a regression
    uint32_t before = total;
    total = rx_witness_total_rp2350( total, base + 100, &lastWa, MASK );
    CHECK( total == before );

    printf( fails ? "%d FAILED\n" : "rx_witness: all cases pass\n", fails );
    return fails ? 1 : 0;
}
