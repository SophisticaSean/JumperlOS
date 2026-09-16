// SPDX-License-Identifier: MIT
// Host test of src/routing/SlotSaveGate.h: the quiet window, and the
// backstop that ends a polling host's ability to starve the slot save.
// Ticks every 50 ms like SlotManager::service; input every 500 ms from t=0.
#include <stdio.h>
#include <stdint.h>
#include "../../src/routing/SlotSaveGate.h"

static int fails = 0;
#define CHECK( c ) do { if ( !( c ) ) { printf( "FAIL %s:%d %s\n", __FILE__, __LINE__, #c ); fails++; } } while ( 0 )

int main( void ) {
    const uint32_t B = SLOT_SAVE_BACKSTOP_MS;
    // quiet host, one mutation at t=0 (which is also the last input)
    CHECK( !slotSaveDue( 1, 750, 750, B ) );
    CHECK( slotSaveDue( 1, 751, 751, B ) );
    // polled host: input at t = 0, 500, 1000, ... ; mutation at t = 0
    uint32_t firstDue = 0;
    for ( uint32_t t = 0; t <= B + 5000; t += 50 ) {
        uint32_t sinceInput = t % 500;
        if ( slotSaveDue( 1, t, sinceInput, B ) ) { firstDue = t; break; }
    }
    CHECK( firstDue == B + 50 );   // never inside the cap (age not > backstop at t == B, sinceInput 0), the first pass after it
    // the backstop opens only the window: a clean slot is never due
    CHECK( !slotSaveDue( 0, B + 1000, 0, B ) );
    CHECK( !slotSaveDue( 0, B + 1000, 5000, B ) );
    // once past the cap the window is 0 - any positive time since input saves
    CHECK( slotSaveQuietMs( B + 1, B ) == 0 && slotSaveQuietMs( B, B ) == SLOT_SAVE_QUIET_MS );
    CHECK( slotSaveDue( 1, B + 1, 1, B ) );
    printf( fails ? "%d FAILED\n" : "slot_backstop: all cases pass\n", fails );
    return fails ? 1 : 0;
}
