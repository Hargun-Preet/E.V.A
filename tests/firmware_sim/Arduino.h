// Minimal stand-in for the Arduino core so the real firmware can be compiled and tested on a PC.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <string>
#include <strings.h>

#define PI 3.14159265358979
#define RAD_TO_DEG 57.29577951308232
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

extern unsigned long fake_ms;
inline unsigned long millis() { return fake_ms; }
inline void delay(unsigned long ms) { fake_ms += ms; }

struct SerialT {
  std::queue<char> in;
  std::string out;                               // everything the firmware printed
  void begin(int) {}
  int available() { return (int)in.size(); }
  char read() { char c = in.front(); in.pop(); return c; }
  void print(const char *s) { out += s; }
  void print(char c) { out += c; }
  void print(int v) { out += std::to_string(v); }
  void print(float v, int d = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", d, v); out += b; }
  void println(const char *s) { out += s; out += '\n'; }
  void println(int v) { print(v); out += '\n'; }
  void println(float v, int d = 2) { print(v, d); out += '\n'; }
  void feed(const std::string &s) { for (char c : s) in.push(c); }
};
extern SerialT Serial;
