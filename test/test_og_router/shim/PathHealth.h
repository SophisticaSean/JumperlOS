// SPDX-License-Identifier: MIT
//
// Per-bridge routing health, the "crossbar-truth" rule get_netlist() reports:
// a bridge is UNROUTED when it has no primary path (duplicate == 0) with its
// two nodes, when that path was refused (net < 0) or skipped, or when a
// stage the sender needs is incomplete (see pathIsClean). Duplicate paths
// are not consulted; the primary decides. Header-only and Arduino-free so the
// host harness (test/test_og_router) checks it against its crossbar model.
#ifndef PATHHEALTH_H
#define PATHHEALTH_H

// Include States.h (globalState) BEFORE this header - it deliberately does not
// include it itself, so the host harness can supply its shim States.h (a
// quote-include from here would resolve to the real, Arduino one).

// What the sender does (CH446Q.cpp sendPath): a hop is closed iff chip, x
// AND y are all set; a hop with chip set but x or y == -1 is silently
// skipped. So "every used hop resolved" is NOT the truth - the router's
// corner-through-chip-L shape legitimately leaves stage 3 as {chip, -1, y}
// (bench, 2026-09-11: GND on all 60 rows flagged GND-1/30/31/60 while the
// ADC read them connected; 1.3.22 prints the same shape for working corner
// paths). The rule validated on hardware (LEDs + ADC across corners, big
// nets and the 30-link case): stages 0 and 1 must be complete, stage 2
// must be complete when its chip is set, stage 3 is never required. The
// host harness checks it both ways against the crossbar model (a bridge
// the rule calls clean is closed by its own hops; one it calls unrouted is
// not).
// One more thing the shape tells: -2 is the router's DEFERRED sentinel (a
// hop it meant to resolve later and never did - the known-open direct
// supply->ADC1/ADC2 case ends {chip -1, x 9, y -2}: the last stage was
// planned, never resolved, and the sender skips it because its chip is
// unset). -1 in stage 3 is the unused corner stage; -2 anywhere, chip set
// or not, is a lane that was never closed.
inline bool pathIsClean( const pathStruct& p ) {
  if ( p.skip || p.net < 0 ) return false;
  for ( int h = 0; h < 2; h++ ) {
    if ( p.chip[ h ] == -1 || p.x[ h ] < 0 || p.y[ h ] < 0 ) return false;
  }
  if ( p.chip[ 2 ] != -1 && ( p.x[ 2 ] < 0 || p.y[ 2 ] < 0 ) ) return false;
  for ( int h = 0; h < 4; h++ ) {
    if ( p.x[ h ] == -2 || p.y[ h ] == -2 ) return false;
  }
  return true;
}

// numPaths: how many entries of paths[] to consult (the primary count).
inline int pathHealthBridgeUnrouted( int bridgeIdx, int numPaths ) {
  if ( bridgeIdx < 0 || bridgeIdx >= globalState.connections.numBridges ) return 1;
  int a = globalState.connections.bridges[ bridgeIdx ][ 0 ];
  int b = globalState.connections.bridges[ bridgeIdx ][ 1 ];
  if ( numPaths > MAX_BRIDGES ) numPaths = MAX_BRIDGES;
  for ( int i = 0; i < numPaths; i++ ) {
    const pathStruct& p = globalState.connections.paths[ i ];
    if ( p.duplicate != 0 ) continue;
    if ( !( ( p.node1 == a && p.node2 == b ) || ( p.node1 == b && p.node2 == a ) ) ) continue;
    return pathIsClean( p ) ? 0 : 1;
  }
  return 1;   // no path at all
}

#endif
