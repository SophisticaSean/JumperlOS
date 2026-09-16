// SPDX-License-Identifier: MIT
// The hand-maintained MicroPython QSTR table: every QDEF1 line must carry
// the right hash (XOR-djb2 over the bytes, 16-bit, 0 -> 1) and length, and
// the entries must be byte-sorted by string - a misplaced or mis-hashed
// QDEF makes the name it defines unresolvable on the board.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static unsigned qhash( const char* s, size_t n ) {
    unsigned h = 5381;
    for ( size_t i = 0; i < n; i++ ) h = ( h * 33 ) ^ (unsigned char)s[ i ];
    h &= 0xffff;
    return h ? h : 1;
}
// C-escapes the table uses in strings: \" \\ \n \r \t \0 and \xNN
static size_t unescape( const char* in, char* out ) {
    size_t n = 0;
    for ( ; *in; in++ ) {
        if ( *in != '\\' ) { out[ n++ ] = *in; continue; }
        in++;
        switch ( *in ) {
            case 'n': out[ n++ ] = '\n'; break; case 'r': out[ n++ ] = '\r'; break; case 't': out[ n++ ] = '\t'; break;
            case '0': out[ n++ ] = '\0'; break; case 'x': { unsigned v; sscanf( in + 1, "%2x", &v ); out[ n++ ] = (char)v; in += 2; break; }
            default: out[ n++ ] = *in;
        }
    }
    return n;
}
int main( int argc, char** argv ) {
    const char* path = argc > 1 ? argv[ 1 ] : "lib/micropython/micropython_embed/genhdr/qstrdefs.generated.h";
    FILE* f = fopen( path, "r" ); if ( !f ) { perror( path ); return 2; }
    char line[ 1024 ], prev[ 1024 ] = ""; size_t prevn = 0; int bad = 0, count = 0;
    while ( fgets( line, sizeof line, f ) ) {
        if ( strncmp( line, "QDEF1(", 6 ) ) continue;
        unsigned hash, len; char raw[ 1024 ], s[ 1024 ];
        // QDEF1(MP_QSTR_x, <hash>, <len>, "<string>")
        char* q = strchr( line, '"' ); char* e = strrchr( line, '"' );
        if ( !q || e <= q || sscanf( line, "QDEF1(%*[^,], %u, %u,", &hash, &len ) != 2 ) { printf( "unparsable: %s", line ); bad++; continue; }
        memcpy( raw, q + 1, (size_t)( e - q - 1 ) ); raw[ e - q - 1 ] = 0;
        size_t n = unescape( raw, s );
        count++;
        if ( hash != qhash( s, n ) || len != n ) { printf( "%s: hash %u len %u, want %u/%zu\n", raw, hash, len, qhash( s, n ), n ); bad++; }
        size_t m = prevn < n ? prevn : n;
        int c = memcmp( prev, s, m ); if ( c == 0 ) c = (int)prevn - (int)n;
        if ( count > 1 && c > 0 ) { printf( "%s: out of byte order (after %s)\n", raw, prev ); bad++; }
        memcpy( prev, s, n ); prevn = n;
    }
    fclose( f );
    if ( bad ) { printf( "qstr table: %d bad entries\n", bad ); return 1; }
    printf( "qstr table: %d entries, sorted, hashes ok\n", count );
    return 0;
}
