// SPDX-License-Identifier: MIT
#include "ReplTiming.h"
#include <Arduino.h>
#include "tusb.h"
#include "Commands.h"       // ledRepaintHeld
#include "config.h"         // jumperlessConfig.debug.repl_timing

namespace {
volatile uint32_t s_done = 0;          // us, core 0
volatile uint8_t  s_itf = 0xFF;        // CDC instance the reply went out on
volatile uint32_t s_wire = 0;          // us, tx-complete for s_itf after s_done
volatile uint32_t s_txCount = 0;       // tx-completes seen on s_itf since done
volatile uint32_t s_renderStart = 0, s_renderEnd = 0;   // us, core 1
volatile uint32_t s_stage[ 6 ] = { 0 };                  // us, core 1
volatile uint32_t s_saveStart = 0, s_saveEnd = 0;        // us, core 0
volatile bool     s_armed = false;
}

extern "C" void tud_cdc_tx_complete_cb( uint8_t itf ) {
    if ( !s_armed || itf != s_itf ) return;
    s_txCount = s_txCount + 1;
    if ( s_wire == 0 ) s_wire = micros( );
}

namespace replt {

void execDone( uint8_t cdcInstance ) {
    if ( !jumperlessConfig.debug.repl_timing ) return;
    s_done = micros( );
    s_itf = cdcInstance;
    s_wire = 0; s_txCount = 0;
    s_renderStart = 0; s_renderEnd = 0;
    s_saveStart = 0; s_saveEnd = 0;
    for ( int i = 0; i < 6; i++ ) s_stage[ i ] = 0;
    __dmb( );
    s_armed = true;
}

void renderStart( void ) {
    if ( !s_armed || s_renderStart ) return;
    s_renderStart = micros( );
}

void renderEnd( uint32_t nets, uint32_t gpio, uint32_t fake, uint32_t meas, uint32_t anim, uint32_t show ) {
    if ( !s_armed || s_renderEnd ) return;
    s_renderEnd = micros( );
    s_stage[ 0 ] = nets; s_stage[ 1 ] = gpio; s_stage[ 2 ] = fake;
    s_stage[ 3 ] = meas; s_stage[ 4 ] = anim; s_stage[ 5 ] = show;
}

void saveStart( void ) { if ( s_armed && !s_saveStart ) s_saveStart = micros( ); }
void saveEnd( void )   { if ( s_armed && !s_saveEnd )   s_saveEnd = micros( ); }

static float rel( uint32_t t ) { return t ? ( (float)( (int32_t)( t - s_done ) ) / 1000.0f ) : -1.0f; }

void service( void ) {
    if ( !s_armed ) return;
    uint32_t now = micros( );
    bool wired = s_wire != 0;
    if ( !wired && ( now - s_done ) < 300000 ) return;   // wait for the wire stamp (or 300 ms)
    if ( wired && ( now - s_wire ) < 120000 ) return;    // let a render/save that follows the reply land
    s_armed = false;
    __dmb( );
    Serial.printf( "[replt] done +0 | wire %+.1f ms (tx#%lu) | render %+.1f..%+.1f ms (nets %lu gpio+fake %lu [%lu] meas %lu anim+ovl %lu show %lu us) | save %+.1f..%+.1f ms | held %d\r\n",
                   wired ? rel( s_wire ) : -1.0f, (unsigned long)s_txCount,
                   rel( s_renderStart ), rel( s_renderEnd ),
                   (unsigned long)s_stage[ 0 ], (unsigned long)s_stage[ 1 ], (unsigned long)s_stage[ 2 ],
                   (unsigned long)s_stage[ 3 ], (unsigned long)s_stage[ 4 ], (unsigned long)s_stage[ 5 ],
                   rel( s_saveStart ), rel( s_saveEnd ), ledRepaintHeld ? 1 : 0 );
}

} // namespace replt
