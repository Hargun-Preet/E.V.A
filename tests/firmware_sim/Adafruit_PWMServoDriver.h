#pragma once
#include "Arduino.h"
extern uint16_t pwm_ticks[16];                   // last pulse written to each channel
struct Adafruit_PWMServoDriver {
  Adafruit_PWMServoDriver(int = 0x40) {}
  void begin() {}
  void setOscillatorFrequency(long) {}
  void setPWMFreq(int) {}
  void setPWM(int ch, int on, int off) { pwm_ticks[ch] = off; }
};
