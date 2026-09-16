#pragma once
#include <Arduino.h>
struct CurrentSenseOverlayState { int virtualWireNode1 = -1; int virtualWireNode2 = -1; };
extern CurrentSenseOverlayState currentSenseOverlayState;
extern int numberOfShownNets;
inline void updateLiveCrossbarDisplay(){}
