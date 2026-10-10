#ifndef WIRE_H_STUB
#define WIRE_H_STUB
#include <Arduino.h>

// 可配置：仿真时把 read() 的返回值改为脚本字节
extern uint8_t g_wireByte;
// 若非 NULL，read() 返回 g_wireHook() 的值（用于按虚拟时间脚本化红外输入）
extern uint8_t (*g_wireHook)();

struct WireStub {
  void begin() {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(void) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 1; }
  int available() { return 1; }
  int read() { return g_wireHook ? (int)g_wireHook() : (int)g_wireByte; }
};
extern WireStub Wire;

#endif