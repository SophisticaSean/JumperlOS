#pragma once
#include <stdint.h>

// Host shim: only what RouteSafety.cpp needs from the crossbar driver. The
// real header drags in the PIO/DMA send path.
struct chipXYBitfield {
    uint16_t connected[8]; // connected[y] & (1 << x)
};

extern chipXYBitfield lastChipXY[12];   // defined in test_og_router.cpp

inline bool getConnectionBit(const chipXYBitfield& state, int x, int y) {
    if (x < 0 || x >= 16 || y < 0 || y >= 8) return false;
    return (state.connected[y] & (1 << x)) != 0;
}

inline void setConnectionBit(chipXYBitfield& state, int x, int y, bool value) {
    if (x < 0 || x >= 16 || y < 0 || y >= 8) return;
    if (value) state.connected[y] |= (uint16_t)(1 << x);
    else state.connected[y] &= (uint16_t)~(1 << x);
}

// The harness never touches hardware: a raw send is a no-op.
inline void sendXYrawUnchecked(int, int, int, int, int = 0) {}
inline int get_core_num(void) { return 0; }
extern int ch446q_timeout_count;                 // defined in test_og_router.cpp
