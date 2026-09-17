#pragma once
struct Probing { int checkProbeButtonState() { return 0; } };
extern Probing probing;
extern int blockProbeButton;
extern unsigned long blockProbeButtonTimer;
inline int encoderNetHighlight(int, int) { return -1; }
