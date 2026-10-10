// 仅用于本机语法/类型检查的 Arduino 桩头文件（不会用于实际编译小车）
#ifndef ARDUINO_H_STUB
#define ARDUINO_H_STUB

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define HIGH 1
#define LOW  0
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2

#define F(x) x

typedef uint8_t byte;
typedef bool boolean;

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int  digitalRead(uint8_t);
void analogWrite(uint8_t, int);
void delay(unsigned long);
void delayMicroseconds(unsigned int);
unsigned long millis(void);
unsigned long micros(void);
unsigned long pulseIn(uint8_t, uint8_t, unsigned long);

struct SerialStub {
  void begin(unsigned long) {}
  template <typename T> void print(T) {}
  template <typename T> void print(T, int) {}
  template <typename T> void println(T) {}
  void println() {}
};
extern SerialStub Serial;

#endif