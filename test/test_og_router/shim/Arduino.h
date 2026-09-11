#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <stdarg.h>
typedef std::string String;
typedef bool boolean;
typedef uint8_t byte;
#define PROGMEM
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#if 0
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
inline unsigned long micros(){return 0;}
inline unsigned long millis(){return 0;}
inline void delay(unsigned long){}
inline void delayMicroseconds(unsigned int){}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
inline int analogRead(int){return 0;}
inline long random(long a){return a?0:0;}
inline long random(long,long b){return b?0:0;}
inline long map(long x,long,long,long,long){return x;}
struct Stream {
  virtual ~Stream(){}
  virtual size_t write(uint8_t c){ return fputc(c, stdout)>=0; }
  template<typename T> size_t print(T v){ std::string s = toStr(v); fputs(s.c_str(), stdout); return s.size();}
  template<typename T> size_t println(T v){ size_t n=print(v); fputs("\n", stdout); return n+1;}
  size_t println(){ fputs("\n", stdout); return 1;}
  size_t print(const char* s){ fputs(s, stdout); return strlen(s);}
  size_t print(const std::string& s){ fputs(s.c_str(), stdout); return s.size();}
  size_t print(char c){ fputc(c, stdout); return 1;}
  void print(int v, int){ printf("%d", v);}
  void print(long v, int){ printf("%ld", v);}
  void print(unsigned v, int){ printf("%u", v);}
  void print(float v, int){ printf("%f", v);}
  void print(double v, int){ printf("%f", v);}
  void println(int v, int){ printf("%d\n", v);}
  void println(float v, int){ printf("%f\n", v);}
  void printf(const char* fmt, ...){ va_list a; va_start(a,fmt); vprintf(fmt,a); va_end(a);}
  void flush(){ fflush(stdout);}
  int available(){return 0;}
  int read(){return -1;}
  int peek(){return -1;}
  void begin(long){}
  operator bool(){return true;}
private:
  static std::string toStr(int v){ return std::to_string(v);}  
  static std::string toStr(unsigned v){ return std::to_string(v);}  
  static std::string toStr(long v){ return std::to_string(v);}  
  static std::string toStr(unsigned long v){ return std::to_string(v);}  
  static std::string toStr(long long v){ return std::to_string(v);}  
  static std::string toStr(unsigned long long v){ return std::to_string(v);}  
  static std::string toStr(float v){ return std::to_string(v);}  
  static std::string toStr(double v){ return std::to_string(v);}  
  static std::string toStr(char v){ return std::string(1,v);}  
  static std::string toStr(unsigned char v){ return std::to_string((int)v);}  
  static std::string toStr(signed char v){ return std::to_string((int)v);}  
  static std::string toStr(short v){ return std::to_string((int)v);}  
  static std::string toStr(bool v){ return std::to_string((int)v);}  
  static std::string toStr(const char* v){ return std::string(v);}  
  static std::string toStr(const std::string& v){ return v;}  
};
#include <stdarg.h>
extern Stream Serial;
extern Stream Serial1;
inline void changeTerminalColor(int=0,bool=false,Stream* =nullptr){}
