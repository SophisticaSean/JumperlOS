// SPDX-License-Identifier: MIT
//
// Raw-REPL reply timeline probe (debug.repl_timing). One line on port 1 per
// raw-REPL exec, stamped from both cores:
//
//   [replt] done +0 | wire +N.N ms (tx#k) | render +a..+b ms (nets x
//           gpio+fake y [0] meas w anim+ovl v show u us) | save +p..+q ms | held 0/1
//
// "done" = core 0 finished the exec and flushed the \x04\x04> markers into
// the CDC FIFO; "wire" = TinyUSB's tx-complete for that CDC interface (the
// bytes left the board); "render" = core 1's nets render (LED branch of
// loop1) that overlapped or followed; "save" = a slot auto-save that ran in
// between. Whatever sits between "done" and "wire" is what a host round trip
// waits on. Cheap: a few volatile stamps; the print happens on core 0 from
// MpRemoteService once the wire stamp lands (or 300 ms later).
#ifndef REPL_TIMING_H
#define REPL_TIMING_H

#include <stdint.h>

namespace replt {

// core 0: exec finished, reply flushed (MpRemoteService)
void execDone( uint8_t cdcInstance );
// core 0: print the line if a wire stamp (or the timeout) has landed
void service( void );
// core 1: LED-branch nets render start / end and its stage stamps (us)
void renderStart( void );
void renderEnd( uint32_t nets, uint32_t gpio, uint32_t fake, uint32_t meas, uint32_t anim, uint32_t show );
// core 0: slot auto-save start / end
void saveStart( void );
void saveEnd( void );

} // namespace replt

#endif
