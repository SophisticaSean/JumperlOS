#pragma once
struct EEPROMClass { int read(int){return 0;} void write(int,int){} void commit(){} void begin(int){} template<typename T> T& get(int, T& t){return t;} template<typename T> const T& put(int, const T& t){return t;} };
extern EEPROMClass EEPROM;
