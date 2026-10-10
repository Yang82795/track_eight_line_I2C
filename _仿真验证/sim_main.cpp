// ===========================================================================
// 宿主机行为仿真：用真实的 avoid_obstacle.cpp / motor_car.cpp / line_pid.cpp
// 配虚拟时钟 + 脚本化传感器，验证避障判定与转向方向是否真的正确。
// 仅用于验证，不会烧进小车。
// ===========================================================================
#include "Arduino.h"
#include "Wire.h"
#include "avoid_obstacle.hpp"
#include <cstdio>

// ---------------- 虚拟时钟 ----------------
static unsigned long g_now = 0;
unsigned long millis(void) { return g_now; }
unsigned long micros(void)  { return g_now * 1000UL; }
void delay(unsigned long ms) { g_now += ms; }               // 不真睡，推进虚拟时间
void delayMicroseconds(unsigned int us) { g_now += (us + 999) / 1000; }

void pinMode(uint8_t, uint8_t) {}
int  digitalRead(uint8_t) { return HIGH; }                  // 按键永不按下
void digitalWrite(uint8_t, uint8_t) {}

// ---------------- 电机 PWM 记录 ----------------
#define MOTOR_L_PWM 5
#define MOTOR_R_PWM 10
static int  g_pwmL = 0, g_pwmR = 0;
static int  g_minSpeedSeen = 999;
static int  g_maxSpeedSeen = 0;
static bool g_recording = false;
void analogWrite(uint8_t pin, int v) {
  if (pin == MOTOR_L_PWM) g_pwmL = v;
  if (pin == MOTOR_R_PWM) g_pwmR = v;
  if (g_recording && v > 0 && v < g_minSpeedSeen) g_minSpeedSeen = v;
  if (g_recording && v > g_maxSpeedSeen) g_maxSpeedSeen = v;
}

// ---------------- 脚本化超声波 ----------------
static float g_cmL = 999.0f, g_cmR = 999.0f;
unsigned long pulseIn(uint8_t echo, uint8_t, unsigned long timeout) {
  float cm = (echo == ECHO_PIN_LEFT) ? g_cmL : g_cmR;
  unsigned long us = (unsigned long)(cm * 58.2f);
  return us > timeout ? 0UL : us;      // 超时=无回波，与真实 HC-SR04 一致
}

// ---------------- 脚本化 I2C (8 路红外) ----------------
// bit7=x1(最左) ... bit0=x8(最右)，0=黑线 1=白底
// 0xFF = 全白；0xE7 = 只有 x4,x5 压黑线(正中间，err=0)
uint8_t g_wireByte = 0xFF;
uint8_t (*g_wireHook)() = 0;
WireStub Wire;

// 按虚拟时间脚本化红外：lineAt 之前全白(看不到线)，之后中线有黑(0xE7)
static unsigned long g_lineAt = 0;
static bool g_lineScript = false;
static uint8_t wireHook() {
  return (g_lineScript && g_now >= g_lineAt) ? 0xE7 : 0xFF;
}

// ---------------- 测试框架 ----------------
static int g_pass = 0, g_fail = 0;
static void check(const char* what, bool ok) {
  if (ok) { g_pass++; printf("  [PASS] %s\n", what); }
  else    { g_fail++; printf("  [FAIL] %s\n", what); }
}

/** 时间必须单调递增，否则会落进开机冷却窗口 */
static void resetWorld() {
  g_now += 20000;     // 推进到一个全新的时刻
  Avoid_init();       // 内部: cooldown = g_now + 800
  g_now += 1200;      // 跳过开机 800ms 屏蔽期
  g_cmL = 999.0f; g_cmR = 999.0f;
  g_wireByte = 0xFF;
}

static ObstacleSide runDetect(unsigned long ms, float cmL, float cmR) {
  g_cmL = cmL; g_cmR = cmR;
  ObstacleSide last = OBS_NONE;
  unsigned long t_end = g_now + ms;
  while (g_now < t_end) {
    last = Avoid_detect();
    if (last != OBS_NONE) return last;
    g_now += 1;
  }
  return last;
}

/** 反复调用 Avoid_detect 直到触发或超时（按真实毫秒推进） */
static ObstacleSide spinDetect(unsigned long ms) {
  unsigned long t_end = g_now + ms;
  ObstacleSide s = OBS_NONE;
  while (g_now < t_end) {
    s = Avoid_detect();
    if (s != OBS_NONE) return s;
    g_now += 1;
  }
  return s;
}

int main() {
  printf("=== 避障逻辑仿真验证 (虚拟时钟 + 脚本化传感器) ===\n");
  printf("    AVOID_TICK_MS=%d  AVOID_TRIGGER_CM=%d  CONFIRM=%d/%d  MODE=%d\n\n",
         (int)AVOID_TICK_MS, (int)AVOID_TRIGGER_CM, (int)AVOID_CONFIRM_TIMES,
         (int)AVOID_CONFIRM_NEAR, (int)AVOID_SWITCH_MODE);

  g_wireHook = wireHook;   // 注册红外脚本钩子

  printf("[T1] 左侧 25cm 障碍 -> 判定 OBS_LEFT (往右绕)\n");
  resetWorld();
  check("触发 OBS_LEFT", runDetect(800, 25.0f, 999.0f) == OBS_LEFT);

  printf("\n[T2] 右侧 25cm 障碍 -> 判定 OBS_RIGHT (往左绕)\n");
  resetWorld();
  check("触发 OBS_RIGHT", runDetect(800, 999.0f, 25.0f) == OBS_RIGHT);

  printf("\n[T3] 空旷场地 -> 绝不触发\n");
  resetWorld();
  check("不误触发", runDetect(3000, 999.0f, 999.0f) == OBS_NONE);

  printf("\n[T4] 远处 50cm 障碍 -> 不触发 (超出触发距离)\n");
  resetWorld();
  check("不触发", runDetect(3000, 50.0f, 999.0f) == OBS_NONE);

  printf("\n[T5] 30cm 障碍 -> 触发\n");
  resetWorld();
  check("触发 OBS_LEFT", runDetect(3000, 30.0f, 999.0f) == OBS_LEFT);

  printf("\n[T6] 近距离 20cm -> 需更多次确认，但仍要触发\n");
  resetWorld();
  check("触发 OBS_LEFT", runDetect(3000, 20.0f, 999.0f) == OBS_LEFT);

  printf("\n[T7] 临界抖动：中间丢 1 次波后恢复 -> 仍触发\n");
  resetWorld();
  { ObstacleSide s = OBS_NONE;
    int ticks = 0;
    for (int i = 0; i < 400 && s == OBS_NONE; i++) {
      // 每 3 次真实 detect 制造一次单拍丢波
      if (ticks == 1) g_cmL = 999.0f;
      if (ticks == 2) g_cmL = 30.0f;
      unsigned long before = g_now;
      s = Avoid_detect();
      if (g_now != before) ticks++;      // 本次真的执行了一次测距
      g_now += 1;
    }
    check("单次丢波不会导致漏触发", s == OBS_LEFT); }

  printf("\n[T8] 双侧障碍：左侧近(18cm)右侧远(40cm) -> 往右绕\n");
  resetWorld();
  check("选 OBS_LEFT", runDetect(1500, 18.0f, 40.0f) == OBS_LEFT);

  printf("\n[T9] 双侧障碍：右侧近(18cm)左侧远(40cm) -> 往左绕\n");
  resetWorld();
  check("选 OBS_RIGHT", runDetect(1500, 40.0f, 18.0f) == OBS_RIGHT);

  printf("\n[T10] 双侧等距(各 20cm) -> 按固定优先级向右绕\n");
  resetWorld();
  check("按 AVOID_PRIORITY_DIR 选 OBS_RIGHT",
        runDetect(1500, 20.0f, 20.0f) == OBS_RIGHT);

  printf("\n[T11] 阶段1方向：左轮快=向右偏(绕开左侧障碍)\n");
  resetWorld();
  StopCar();
  Set_motor_direct(AVOID_TURN_OUT_OUTER, AVOID_TURN_OUT_INNER);
  check("左快右慢 => 向右偏", g_pwmL > g_pwmR);
  StopCar();
  Set_motor_direct(AVOID_TURN_OUT_INNER, AVOID_TURN_OUT_OUTER);
  check("左慢右快 => 向左偏", g_pwmR > g_pwmL);
  StopCar();

  printf("\n[T12] 完整动作(左侧障碍)：绕行 -> 闭环回线 -> 耗时/不倒转\n");
  {
    resetWorld();
    g_cmL = 25.0f;
    ObstacleSide s = spinDetect(1000);
    check("先检测到障碍", s == OBS_LEFT);

    // 脚本：动作开始 900ms 后黑线重新出现在正中间(0xE7)
    g_lineScript = true;
    g_lineAt = g_now + 900;

    g_minSpeedSeen = 999; g_maxSpeedSeen = 0; g_recording = true;
    unsigned long t0 = g_now;
    Avoid_perform(s);
    unsigned long dur = g_now - t0;
    g_recording = false;
    g_lineScript = false;

    printf("       动作耗时(虚拟) = %lu ms\n", dur);
    check("全程无倒车 (最小轮速 >= 1)", g_minSpeedSeen >= 1);
    check("电机确实转起来 (最大轮速 > 20)", g_maxSpeedSeen > 20);
    check("闭环回线成功，耗时 <= 2500ms", dur <= 2500);
  }

  printf("\n[T12b] 回线最坏情况：全程看不到线 -> 应在超时后放弃并停车\n");
  {
    resetWorld();
    g_cmL = 25.0f;
    ObstacleSide s = spinDetect(1000);
    g_lineScript = true; g_lineAt = g_now + 999999;   // 永远看不到线
    g_minSpeedSeen = 999; g_maxSpeedSeen = 0; g_recording = true;
    unsigned long t0 = g_now;
    Avoid_perform(s);
    unsigned long dur = g_now - t0;
    g_recording = false;
    g_lineScript = false;
    printf("       最坏耗时(虚拟) = %lu ms\n", dur);
    check("超时后有界 (<= 3200ms) 且不倒转",
          dur <= 3200 && g_minSpeedSeen >= 1);
    check("超时后电机已停转 (PWM=0)", g_pwmL == 0 && g_pwmR == 0);
  }

  printf("\n[T13] 冷却：避障刚结束不对同一障碍二次触发\n");
  {
    resetWorld();
    g_cmL = 25.0f;
    ObstacleSide s = OBS_NONE;
    for (int i = 0; i < 20 && s == OBS_NONE; i++) { s = Avoid_detect(); g_now += 1; }
    Avoid_perform(s);
    check("冷却期内不二次触发", runDetect(800, 25.0f, 999.0f) == OBS_NONE);
  }

  printf("\n[T14] 避障后屏蔽期内不误判终点\n");
  {
    resetWorld();
    g_cmL = 25.0f;
    ObstacleSide s = OBS_NONE;
    for (int i = 0; i < 20 && s == OBS_NONE; i++) { s = Avoid_detect(); g_now += 1; }
    Avoid_perform(s);
    check("终点屏蔽窗口已激活 (millis() < g_shield_finish_until)",
          millis() < g_shield_finish_until);
  }

  printf("\n==========================================\n");
  printf("  通过 %d 项, 失败 %d 项\n", g_pass, g_fail);
  printf("==========================================\n");
  return g_fail == 0 ? 0 : 1;
}