#pragma once
// Host shim: the harness is single-threaded, so core 1 is always idle.
namespace core1req { inline bool allIdle() { return true; } }
