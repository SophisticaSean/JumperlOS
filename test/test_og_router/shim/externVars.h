#pragma once
#include <stdint.h>

// Host shim: the few externs RouteSafety.cpp reaches for. The harness is
// single-threaded, so nothing is ever held or in flight.
static inline bool core1FramesHeld(void) { return false; }
