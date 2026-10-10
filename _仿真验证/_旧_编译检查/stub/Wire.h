#ifndef WIRE_H_STUB
#define WIRE_H_STUB
#include <Arduino.h>

struct WireStub {
  void begin() {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(void) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 1; }
  int available() { return 1; }
  int read() { return 0; }
};
extern WireStub Wire;

#endif