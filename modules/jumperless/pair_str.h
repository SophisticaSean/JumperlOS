// SPDX-License-Identifier: MIT
// String form of a connect_many pair list, parsed in C so the host sends one
// token instead of a tuple list the compiler chews on (~40 us/byte on the
// RP2040):
//   "<node>:<p>[,<p>]*(;<node>:<p>[,<p>]*)*"   e.g. "103:1,2,3;100:4"
// with node ids as decimal ints (1..199). Trailing ';' runs are accepted (the
// host pads short literals past MicroPython's 10-byte qstr-interning limit).
// Two passes: pair_str_count validates the WHOLE string and counts pairs (the
// caller raises before anything is edited), pair_str_fill writes them out.
// Pure C, no MicroPython dependency: test/test_pair_str builds it on the host.
#ifndef JL_PAIR_STR_H
#define JL_PAIR_STR_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

static inline bool pair_str_int( const char** p, const char* end, int* out ) {
    int v = 0; int digits = 0;
    while ( *p < end && **p >= '0' && **p <= '9' ) {
        v = v * 10 + ( **p - '0' );
        if ( ++digits > 3 ) return false;
        ( *p )++;
    }
    if ( digits == 0 || v < 1 || v > 199 ) return false;
    *out = v;
    return true;
}
// -1 on any defect (an empty set included), else the pair count.
static inline int pair_str_count( const char* s, size_t len ) {
    const char* p = s; const char* end = s + len;
    int n = 0;
    while ( p < end ) {
        if ( *p == ';' ) { p++; continue; }
        int a;
        if ( !pair_str_int( &p, end, &a ) || p >= end || *p != ':' ) return -1;
        p++;
        int b;
        if ( !pair_str_int( &p, end, &b ) ) return -1;
        n++;
        while ( p < end && *p == ',' ) {
            p++;
            if ( !pair_str_int( &p, end, &b ) ) return -1;
            n++;
        }
        if ( p < end && *p != ';' ) return -1;
    }
    return n == 0 ? -1 : n;   // "" / ";;" is a defect, not "want nothing": clearing uses want=[]
}
// Only after pair_str_count >= 0; A/B must hold that many.
static inline void pair_str_fill( const char* s, size_t len, int16_t* A, int16_t* B ) {
    const char* p = s; const char* end = s + len;
    int n = 0;
    while ( p < end ) {
        if ( *p == ';' ) { p++; continue; }
        int a = 0, b = 0;
        pair_str_int( &p, end, &a ); p++;
        pair_str_int( &p, end, &b );
        A[ n ] = (int16_t)a; B[ n ] = (int16_t)b; n++;
        while ( p < end && *p == ',' ) {
            p++;
            pair_str_int( &p, end, &b );
            A[ n ] = (int16_t)a; B[ n ] = (int16_t)b; n++;
        }
    }
}
#endif
