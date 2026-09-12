// SPDX-License-Identifier: MIT
//
// Per-bridge routing health, the "crossbar-truth" rule get_netlist() reports:
// a bridge is UNROUTED when it has no primary path (duplicate == 0) with its
// two nodes, when that path was refused (net < 0) or skipped, or when a used
// hop (chip set) has an x or y that never resolved (-1). Duplicate paths are
// not consulted; the primary decides. Header-only and Arduino-free so the
// host harness (test/test_og_router) checks it against its crossbar model.
#ifndef PATHHEALTH_H
#define PATHHEALTH_H

// Include States.h (globalState) BEFORE this header - it deliberately does not
// include it itself, so the host harness can supply its shim States.h (a
// quote-include from here would resolve to the real, Arduino one).

inline bool pathIsClean( const pathStruct& p ) {
  if ( p.skip || p.net < 0 ) return false;
  for ( int h = 0; h < 4; h++ ) {
    if ( p.chip[ h ] == -1 ) continue;
    if ( p.x[ h ] < 0 || p.y[ h ] < 0 ) return false;
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
