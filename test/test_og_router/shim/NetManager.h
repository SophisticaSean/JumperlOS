#pragma once
#include <Arduino.h>
#include "Jerial.h"
extern int newBridgeLength;
int printNodeOrName(int node, int longOrShort = 0, int netIndex = -1, Stream *stream = &Jerial);
const char* definesToChar (int defined, int longOrShort = 0);
void printBridgeArray(Stream *stream = &Jerial);
void assignTermColor(int startIndex = 0);
