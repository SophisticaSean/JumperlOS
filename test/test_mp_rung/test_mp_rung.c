// SPDX-License-Identifier: MIT
// Host test of src/snakes/MpHeapRung.h: the ladder order and admissibility.
#include <stdio.h>
#include "../../src/snakes/MpHeapRung.h"
#define K( n ) ( (size_t)( n ) * 1024 )
static int fails = 0;
#define CHECK( c ) do { if ( !( c ) ) { printf( "FAIL %s:%d %s\n", __FILE__, __LINE__, #c ); fails++; } } while ( 0 )
int main( void ) {
    // OG: 56 -> 48 -> 48 -> 40 -> 32 -> 24 -> 16 -> 0 (configured-8K duplicates 48)
    size_t want[] = { K( 56 ), K( 48 ), K( 48 ), K( 40 ), K( 32 ), K( 24 ), K( 16 ), 0 };
    for ( size_t i = 0; i < 8; i++ ) CHECK( mpNextRung( K( 56 ), i ) == want[ i ] );
    // V5 96 K: configured-8K is 88 K
    CHECK( mpNextRung( K( 96 ), 1 ) == K( 88 ) );
    CHECK( mpNextRung( K( 4 ), 1 ) == 0 );
    // never above configured, even for the fixed rungs
    for ( size_t i = 0; mpNextRung( K( 40 ), i ); i++ ) CHECK( !mpRungFits( mpNextRung( K( 40 ), i ), K( 40 ), K( 12 ), K( 1000 ) ) || mpNextRung( K( 40 ), i ) <= K( 40 ) );
    CHECK( !mpRungFits( K( 48 ), K( 40 ), K( 12 ), K( 1000 ) ) );
    // the reserve is honoured: 56 + 12 needs 68 free
    CHECK( mpRungFits( K( 56 ), K( 56 ), K( 12 ), K( 68 ) ) );
    CHECK( !mpRungFits( K( 56 ), K( 56 ), K( 12 ), K( 67 ) ) );
    // a build 3 KB short of its rung lands exactly one step down
    size_t freeHeap = K( 56 ) + K( 12 ) - K( 3 ); size_t taken = 0;
    for ( size_t i = 0; mpNextRung( K( 56 ), i ); i++ ) if ( mpRungFits( mpNextRung( K( 56 ), i ), K( 56 ), K( 12 ), freeHeap ) ) { taken = mpNextRung( K( 56 ), i ); break; }
    CHECK( taken == K( 48 ) );
    CHECK( MP_RUNG_OG_CONFIGURED_KB == 56 && MP_RUNG_FLOOR_KB == 40 );
    printf( fails ? "%d FAILED\n" : "mp_rung: all cases pass\n", fails );
    return fails ? 1 : 0;
}
