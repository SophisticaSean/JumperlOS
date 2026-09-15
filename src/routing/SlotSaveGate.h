// SPDX-License-Identifier: MIT
// The slot auto-save's quiet-window rule, kept free of Arduino/pico so
// test/test_slot_backstop can run it on the host.
//
// The save is gated on systemIdleForFlush(quietMs), which (among other
// checks that stay in force: refreshInProgress, loadingFile, core1busy,
// usbMountedByHost, probe, click-menu, context) rejects while
//   (millis() - lastUserInputMs) <= quietMs            (externVars.cpp)
// Every raw-REPL batch counts as user input, so a host that polls faster
// than the 750 ms window could keep the slot dirty forever; once the slot
// has been dirty for longer than the backstop, the quiet window collapses
// to 0 and the next service pass saves regardless of input cadence. The
// other safety checks are untouched by this - only the window changes.
#pragma once
#include <stdint.h>

#define SLOT_SAVE_QUIET_MS 750u
#define SLOT_SAVE_BACKSTOP_MS 60000u

static inline uint32_t slotSaveQuietMs( uint32_t dirtyAgeMs, uint32_t backstopMs ) {
    return dirtyAgeMs > backstopMs ? 0u : SLOT_SAVE_QUIET_MS;
}

// Host-side mirror of the decision the board makes (the `>` is
// externVars.cpp's `<= quietMs` rejection, inverted).
static inline int slotSaveDue( int dirty, uint32_t dirtyAgeMs, uint32_t sinceInputMs, uint32_t backstopMs ) {
    return dirty && sinceInputMs > slotSaveQuietMs( dirtyAgeMs, backstopMs );
}
