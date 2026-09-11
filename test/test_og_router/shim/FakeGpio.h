#pragma once
#include "JumperlessDefines.h"
struct FakeGpioOutput { bool active; int currentState; int highVoltageNode; int lowVoltageNode; };
struct FakeGpioInput { bool active; int adcChannel; };
#ifndef MAX_FAKE_GP_OUT
#define MAX_FAKE_GP_OUT 8
#endif
#ifndef MAX_FAKE_GP_IN
#define MAX_FAKE_GP_IN 8
#endif
extern FakeGpioOutput fakeGpioOutputs[MAX_FAKE_GP_OUT];
extern FakeGpioInput fakeGpioInputs[MAX_FAKE_GP_IN];
extern int fakeGpioInputAdcChannel;
