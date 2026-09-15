#pragma once
#include "LEDs.h"
extern int gpioNet[10];
extern int gpioReading[10];
extern int gpioDef[10][3];
extern int showADCreadings[8];
extern uint32_t gpioReadingColors[10];
inline uint32_t measurementToColor(float, float = -8.0, float = 8.0) { return 0; }
extern float adcReadings[8];
extern int gpioState[50];
