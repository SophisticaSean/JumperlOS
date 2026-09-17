#pragma once
#include <stdint.h>
typedef struct rgbColor { unsigned char r; unsigned char g; unsigned char b; } rgbColor;
typedef struct hsvColor { unsigned char h; unsigned char s; unsigned char v; } hsvColor;
extern rgbColor netColors[];
extern uint8_t gpioAnimationBaseHues[10];
extern int brightenedNode;
inline char* colorToName(uint32_t, int = -1) { static char s[] = "color"; return s; }
inline char* colorToName(int, int = -1) { static char s[] = "color"; return s; }
inline char* colorToName(rgbColor, int = -1) { static char s[] = "color"; return s; }
inline int colorToVT100(uint32_t, int = 256) { return 0; }
inline int colorToAnsi(uint32_t) { return 0; }
inline rgbColor HsvToRgb(hsvColor) { return rgbColor{0,0,0}; }
inline uint32_t HsvToRaw(hsvColor) { return 0; }
inline uint32_t packRgb(rgbColor) { return 0; }
inline uint32_t packRgb(uint8_t, uint8_t, uint8_t) { return 0; }
inline hsvColor RgbToHsv(rgbColor) { return hsvColor{0,0,0}; }
inline hsvColor RgbToHsv(uint32_t) { return hsvColor{0,0,0}; }
