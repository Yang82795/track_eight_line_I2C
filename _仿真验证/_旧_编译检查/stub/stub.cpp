// 桩实现：只为链接阶段提供符号
#include "Arduino.h"
SerialStub Serial;
WireStub Wire;
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
int  digitalRead(uint8_t) { return 0; }
void analogWrite(uint8_t, int) {}
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
unsigned long millis(void) { return 0; }
unsigned long micros(void) { return 0; }
unsigned long pulseIn(uint8_t, uint8_t, unsigned long) { return 0; }