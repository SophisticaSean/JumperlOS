#pragma once
#include <Arduino.h>
#include "JumperlessDefines.h"
#include "config.h"
#include "MatrixState.h"
#include "LEDs.h"
struct justXY { int8_t xStatus[16]; int8_t yStatus[8]; };
struct ConnectionState {
    int16_t bridges[MAX_BRIDGES][3];
    int16_t numBridges;
    netStruct nets[MAX_NETS];
    int16_t numNets;
    uint32_t bridgeColors[MAX_BRIDGES];
    pathStruct paths[MAX_BRIDGES];
    int16_t numPaths;
    bool pathsCacheValid;
    chipStatus chipStates[12];
    struct justXY chipXY[12];
    bool chipStatesCacheValid;
};
struct DisplayState {
    bool getNetColor(int, rgbColor&, uint32_t&, char*) const { return false; }
    const char* getNetName(int) const { return nullptr; }
};
struct PowerState { float topRail = 0, bottomRail = 0, dac0 = 0, dac1 = 0; };
struct JumperlessState { ConnectionState connections; DisplayState display; PowerState power; };
extern JumperlessState globalState;
