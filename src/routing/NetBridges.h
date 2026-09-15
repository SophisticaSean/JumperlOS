// SPDX-License-Identifier: MIT
//
// Per-net bridge lists: the (node1, node2) pairs NetManager files under each
// net, which bridgesToPaths() turns into paths in net order, bridge order.
//
// Two storages behind one API so NetManager / the routers are single-source:
//
//   V5  netStruct.bridges[MAX_NODES][2], the original inline table. Each net
//       owns MAX_NODES slots; a 0 in [k][0] ends the list.
//
//   OG  ONE shared pool of NET_BRIDGE_POOL_SIZE entries (node1, node2, next)
//       and a per-net linked list {head, tail, count} in netStruct.bridges.
//       The inline table cost 96 B x 60 nets = 5760 B for a board that can
//       hold MAX_BRIDGES = 72 bridges in total (1440 slots, ~20x); the pool
//       is 2*MAX_BRIDGES entries because a bridge between two special nets
//       is listed under BOTH (NetManager "can't combine special nets") and a
//       merge appends net B's list to net A before net B is freed. So no
//       netlist that fits MAX_BRIDGES can fill it, and the per-net BRIDGE
//       cap (the old MAX_NODES) is gone: one net may carry every bridge.
//
// Lifecycle rules the OG storage adds (no-ops on V5, but call them anyway):
//   - resetAll() whenever every net is reinitialised (initNets /
//     ConnectionState::clear), so the pool cannot leak entries whose net
//     header was overwritten by a struct assignment.
//   - clear(net) frees the net's entries. detach(net) forgets them WITHOUT
//     freeing: after shiftNets copies nets[i+1] into nets[i] the last slot
//     still holds a header that now belongs to the net below it.
//   - Iteration order is insertion order on both boards, and a merged net
//     keeps A's bridges before B's: the routing order is unchanged.
//
// Arduino-free on purpose: test/test_og_router builds this on the host.
#ifndef NETBRIDGES_H
#define NETBRIDGES_H

#include <stdint.h>
#include <string.h>
#include "JumperlessDefines.h"

#if defined(OG_JUMPERLESS)

#define NET_BRIDGE_POOL_SIZE (2 * MAX_BRIDGES)  // entries; index 0 is the null link
static_assert(NET_BRIDGE_POOL_SIZE + 1 <= 255, "pool links are uint8_t");

// The per-net header. Lives at netStruct.bridges so the positional aggregate
// initialisers ({{}} in specialFunctionNetsInit / initNets / clearAllNTCC)
// zero it like they zeroed the old table.
struct NetBridgeList {
  uint8_t head;   // pool index of the first entry, 0 = empty
  uint8_t tail;   // pool index of the last entry (valid when head != 0)
  uint8_t count;
};

struct NetBridgePool {
  struct Entry {
    int16_t node1;
    int16_t node2;
    uint8_t next;   // 0 = end of list / end of free list
  };
  Entry entries[NET_BRIDGE_POOL_SIZE + 1];  // [0] is never used (null index)
  uint8_t freeHead;
  uint8_t used;
  bool initialised;
};

// Owned by globalState.connections in spirit (there is exactly one
// JumperlessState, and it is non-copyable); kept a separate global so the
// header stays Arduino-free and the host test can define its own.
extern NetBridgePool netBridgePool;

#endif // OG_JUMPERLESS

struct netStruct;

namespace netbridges {

#if defined(OG_JUMPERLESS)

inline void resetAll() {
  NetBridgePool& p = netBridgePool;
  for (int i = 1; i < NET_BRIDGE_POOL_SIZE; i++) p.entries[i].next = (uint8_t)(i + 1);
  p.entries[NET_BRIDGE_POOL_SIZE].next = 0;
  p.entries[0].next = 0;
  p.freeHead = 1;
  p.used = 0;
  p.initialised = true;
}

inline int capacity() { return NET_BRIDGE_POOL_SIZE; }
inline int poolUsed() { return netBridgePool.used; }

struct Iter {
  uint8_t idx;
  bool valid() const { return idx != 0; }
  void next() { idx = netBridgePool.entries[idx].next; }
  int16_t node1() const { return netBridgePool.entries[idx].node1; }
  int16_t node2() const { return netBridgePool.entries[idx].node2; }
};

// Declared here, defined after netStruct is complete (MatrixState.h).
inline Iter begin(const netStruct& n);
inline int count(const netStruct& n);
inline bool append(netStruct& n, int16_t node1, int16_t node2);
inline void clear(netStruct& n);
inline void detach(netStruct& n);

#else // V5: the inline table

inline void resetAll() {}
inline int capacity() { return MAX_NODES; }
inline int poolUsed() { return 0; }

struct Iter {
  const int16_t (*table)[2];
  int k;
  bool valid() const { return k < MAX_NODES && table[k][0] != 0; }
  void next() { k++; }
  int16_t node1() const { return table[k][0]; }
  int16_t node2() const { return table[k][1]; }
};

inline Iter begin(const netStruct& n);
inline int count(const netStruct& n);
inline bool append(netStruct& n, int16_t node1, int16_t node2);
inline void clear(netStruct& n);
inline void detach(netStruct& n);

#endif

} // namespace netbridges

#endif // NETBRIDGES_H
