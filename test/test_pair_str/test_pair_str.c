// SPDX-License-Identifier: MIT
// Host test of modules/jumperless/pair_str.h (PERF_PLAN.md P2.1): every
// reject case must return -1, every accept case must count and fill right.
//   test/test_pair_str/run.sh
#include <stdio.h>
#include <string.h>
#include "../../modules/jumperless/pair_str.h"

static int fails = 0;
static void reject( const char* s ) {
    int n = pair_str_count( s, strlen( s ) );
    if ( n != -1 ) { printf( "FAIL reject %-16s -> %d\n", s, n ); fails++; }
}
static void accept( const char* s, int want, const char* pairs ) {
    int n = pair_str_count( s, strlen( s ) );
    if ( n != want ) { printf( "FAIL accept %-16s count %d want %d\n", s, n, want ); fails++; return; }
    int16_t A[ 256 ], B[ 256 ];
    pair_str_fill( s, strlen( s ), A, B );
    char buf[ 2048 ] = ""; size_t at = 0;
    for ( int i = 0; i < n; i++ ) at += (size_t)snprintf( buf + at, sizeof buf - at, "%d-%d%s", A[ i ], B[ i ], i + 1 < n ? "," : "" );
    if ( strcmp( buf, pairs ) != 0 ) { printf( "FAIL accept %-16s pairs %s want %s\n", s, buf, pairs ); fails++; }
}
int main( void ) {
    // reject list from PERF_PLAN.md P2.1
    reject( "" ); reject( ":" ); reject( "1:" ); reject( ":2" ); reject( "1:2," ); reject( ";;" );
    reject( "0:1" ); reject( "1:0" ); reject( "200:1" ); reject( "1:200" ); reject( "-1:2" ); reject( "1:-2" );
    reject( "a:1" ); reject( "1:b" ); reject( "1:2;x" ); reject( "1::2" ); reject( "1:2,,3" ); reject( "1:2 " );
    reject( "1000:1" ); reject( "1:2:3" );
    // accept list
    accept( "103:4", 1, "103-4" );
    accept( "103:1,2,3", 3, "103-1,103-2,103-3" );
    accept( "103:1,2;100:3", 3, "103-1,103-2,100-3" );
    accept( "103:4;", 1, "103-4" );
    accept( "103:4;;;;;;", 1, "103-4" );
    accept( ";;103:4", 1, "103-4" );
    accept( "1:2", 1, "1-2" );
    accept( "199:199", 1, "199-199" );
    // 129 pairs count (the caller rejects > 128 / > MAX_BRIDGES)
    char big[ 1024 ] = "103:"; for ( int i = 1; i <= 129; i++ ) { char t[ 8 ]; snprintf( t, sizeof t, "%d%s", i, i < 129 ? "," : "" ); strcat( big, t ); }
    if ( pair_str_count( big, strlen( big ) ) != 129 ) { printf( "FAIL 129 pairs\n" ); fails++; }
    printf( fails ? "%d FAILED\n" : "pair_str: all cases pass\n", fails );
    return fails ? 1 : 0;
}
