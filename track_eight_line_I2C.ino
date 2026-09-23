/* =========================================================================
 *  重走古丝路 · 自动循迹避障小车   主程序 (融合优化版 V5)
 *
 *  硬件: 8路I2C红外循迹 + HC-SR04超声波 + 舵机扫描 + 双直流电机
 *  赛道: 6200mm x 2000mm，黑色循迹线宽约 20mm，障碍随机放在线的任意一侧。
 *
 *  计分点对应策略:
 *   赛段分40: 连续质心偏差(mm) PD + 弯道减速 + 丢线原地找回
 *   避障分40: 障碍随机放线两侧 → 超声波左右扫描判侧 + 贴边偏置绕行 + 回线
 *   时间分40: 直道快、弯道慢；先稳后提速(Speed_Line)
 *   停止分20: 终点横线"先刹后判"(电子刹车 + 慢爬测黑块厚度)，区分发夹与终点
 *
 *  本版由两套独立实现融合而来，取长补短:
 *   - 主循环/巡线/终点判定/软里程 采用 -ds 版(更完善)
 *   - I2C 连续失败兜底停车、左右电机比例校正、两侧接近时交替绕行 采用根目录版
 *   - 新增: 可选自动起步(无按键场景)、避障动作复用线感知贴边通过
 *
 *  ⚠ 上车前必看: `现场调试步骤与待定夺清单.md`(动手顺序/记录表)
 *               `调参说明.md` 与 `比赛调试与优化指南.md`(参数速查/优化顺序)
 *  ⚠ 必须先确认: line_pid.cpp 的 BLACK_IS_ZERO(0=压线) 与 SENSOR_PITCH_MM
 * ========================================================================= */

#include <Wire.h>
#include "motor_car.hpp"
#include "line_pid.hpp"
#include "avoid_obstacle.hpp"

/* ------------------------- 硬件 / 起步 ------------------------- */
#define KEY_PIN          (2)
const int Press_KEY   = 0;
const int Release_KEY = 1;

// true: 上电稳定后自动起步(无需按键，适合现场一键放行)
// false: 需按 KEY_PIN 起步(便于对齐起始线后再放行)
#define AUTO_START       (false)
#define AUTO_START_DELAY_MS (500)

extern uint8_t x1,x2,x3,x4,x5,x6,x7,x8;    // 8路红外状态(从左往右)，定义在 line_pid.cpp

/* ------------------------- 主循环与判定参数 ------------------------- */
#define CTRL_PERIOD_MS   (10)      // 控制周期 10ms = 100Hz
#define DEBUG_SERIAL     (1)       // 调参时1；正式比赛建议0(并拔掉USB线)
#define DEBUG_PERIOD_MS  (300)     // 串口调试输出间隔

#define SEARCH_LOST_MS    (60)     // 全白持续这么久 → 原地找线
#define SEARCH_TIMEOUT_MS (1300)   // 找线超时 → 停车(防止跑出赛道/逆行)
#define SEARCH_PWM        (62)     // 原地找线速度

#define I2C_FAIL_STOP_CNT (5)      // 连续 I2C 读失败达到此次数 → 兜底停车(线松脱保护)

#define CROSS_MIN_MS      (12000)  // 终点线时间闸门(起步后至少12s)
#define CROSS_MIN_MM      (4500)   // 终点线里程闸门(至少4.5m，挡住前中段发夹弯)
#define CROSS_NORMAL_MS   (400)    // 候选前必须有"正常压线"记录
#define CROSS_NORMAL_RUN  (3)      // 候选前需连续正常压线周期数
#define CROSS_ENV_MM      (32)     // 候选前偏差包络需小于此值(近似走直)；误停改小，检不到改999
#define CROSS_WIDE_CNT    (5)      // "宽黑"门限：黑点数≥5 视为横线/发夹候选
#define CROSS_WIDE_CYCLES (2)      // 宽黑需连续这么多周期才进入判定
#define BRAKE_MS          (90)     // 电子刹车时间
#define CREEP_PWM         (68)     // 判定爬行速度(必须>电机起转60)
#define CREEP_MS          (450)    // 判定爬行最长时间(≈90mm行程)
#define CREEP_CLEAR_MS    (250)    // 黑块必须在这段时间内结束才算"薄横线"；超时=发夹黑块
#define CREEP_CLEAR_CNT   (3)      // 爬行中黑点数降到≤此值 → 黑块结束(终点候选)
#define CREEP_TURN_MM     (18)     // 爬行中质心偏差超过此值 → 黑块偏在一边，判为发夹

enum { CAR_IDLE = 0, CAR_RUN, CAR_SEARCH, CAR_DONE };
static uint8_t  g_state = CAR_IDLE;
static uint32_t g_startMs = 0, g_lastNormalMs = 0, g_lostSince = 0;
static uint32_t g_searchStart = 0, g_lastDebug = 0, g_loopLast = 0;
static uint8_t  g_wideCnt = 0;          // 当前宽黑已持续周期数
static uint8_t  g_normalRun = 0;        // 当前连续正常压线周期数
static uint8_t  g_runBeforeWide = 0;    // 宽黑出现前的连续正常压线周期数
static int8_t   g_searchDir = 1;
static uint8_t  g_i2cFail = 0;          // 连续 I2C 读失败计数

int  getKeyState(uint8_t pin);
static void start_race(void);
static bool cross_gate_ok(void);
static void cross_handle(void);
static void debug_print(void);

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(KEY_PIN, INPUT_PULLUP);
  delay(200);

  avoid_init();        // 舵机回中 + 超声波引脚
  Motor_init();        // 电机初始化
  init_x_PID();        // PID/里程初始化

  Car_stop();

#if AUTO_START
  delay(AUTO_START_DELAY_MS);   // 仅上电稳定等待；此后全程无阻塞 delay(避障/刹车内部除外)
  start_race();
  Serial.println(F("=== Silk-Road Line Car: AUTO-START ==="));
#else
  Serial.println(F("=== Silk-Road Line Car Ready. Align at START line, press KEY ==="));
#endif
}

/* 起步初始化：清零所有运行时状态、复位里程、短暂屏蔽超声波 */
static void start_race(void)
{
  init_x_PID();
  line_dist_reset();
  g_startMs       = millis();
  g_lastNormalMs  = g_startMs;
  g_lostSince     = 0;
  g_wideCnt       = 0;
  g_normalRun     = 0;
  g_runBeforeWide = 0;
  g_searchDir     = 1;
  g_i2cFail       = 0;
  avoid_blank(500);
  g_state = CAR_RUN;
}

void loop()
{
  uint32_t now = millis();

  /* ---------- 100Hz 定周期调度(去掉原代码每圈的 delay(10)+Serial 阻塞) ---------- */
  if ((uint32_t)(now - g_loopLast) < CTRL_PERIOD_MS) return;
  g_loopLast = now;

  switch (g_state)
  {
  /* ---------------- 待机: 等按键 ---------------- */
  case CAR_IDLE:
    if (getKeyState(KEY_PIN) == Press_KEY) {
      delay(200);                 // 等手离开
      start_race();
      Serial.println(F("GO"));
    }
    break;

  /* ---------------- 巡线主状态 ---------------- */
  case CAR_RUN:
  {
    uint8_t m = line_read();

    /* I2C 兜底保护：连续多次读失败(线松脱/掉线) → 停车，避免带旧数据高速冲出 */
    if (!line_read_ok()) {
      if (g_i2cFail < 255) g_i2cFail++;
      if (g_i2cFail >= I2C_FAIL_STOP_CNT) {
        Car_stop();
        g_state = CAR_DONE;
        Serial.println(F("I2C-FAIL-STOP"));
        break;
      }
    } else {
      g_i2cFail = 0;
    }

    uint8_t n  = line_black_count(m);
    uint8_t ev;

    /* 记录"最近一次正常压线"的时间；并统计"宽黑出现前连续正常了多少周期"
       —— 终点横线是突然出现的，发夹是逐渐变宽的，这个量能区分二者 */
    if (n >= 1 && n <= 4) {
      g_lastNormalMs = millis();
      if (g_normalRun < 250) g_normalRun++;
    } else if (n >= CROSS_WIDE_CNT) {
      if (g_wideCnt == 0) g_runBeforeWide = g_normalRun;   // 宽黑事件刚开始，锁存历史
      if (g_wideCnt < 200) g_wideCnt++;
      g_normalRun = 0;
    } else {
      g_normalRun = 0;
      g_wideCnt = 0;
    }

    /* ---- 终点横线候选: 黑块突然变宽 ----
       正穿的横线 → 8路全黑；斜穿(本项目起/终点线) → 约5~6路全黑；
       发夹弯顶部同样会变宽变黑，所以这里只是"候选"，由 cross_handle() 先刹后判 */
    if (g_wideCnt >= CROSS_WIDE_CYCLES && g_runBeforeWide >= CROSS_NORMAL_RUN &&
        cross_gate_ok()) {
      cross_handle();             // 先刹后判(内部会切换到 CAR_DONE 或回 CAR_RUN)
      break;
    }

    /* ---- 正常巡线一步 ---- */
    ev = line_follow(m);

    /* ---- 丢线: 原地转向找回(发夹弯/急弯的保命逻辑) ---- */
    if (ev & LINE_EV_LOST) {
      if (g_lostSince == 0) g_lostSince = millis();
      if ((uint32_t)(millis() - g_lostSince) > SEARCH_LOST_MS) {
        g_searchDir = (int8_t)line_last_dir();   // 朝上次偏差方向原地转，最易找回线
        g_searchStart = millis();
        Car_pivot(g_searchDir, SEARCH_PWM);
        g_state = CAR_SEARCH;
        break;
      }
    } else {
      g_lostSince = 0;
    }

    /* ---- 避障(超声波确认后执行分步动作) ---- */
    if (avoid_need()) {
      avoid_run();
    }
    break;
  }

  /* ---------------- 丢线原地找线 ---------------- */
  case CAR_SEARCH:
    Car_pivot(g_searchDir, SEARCH_PWM);
    if (line_black_count(line_read()) > 0) {
      Car_stop();
      g_lostSince = 0;
      g_state = CAR_RUN;
    } else if ((uint32_t)(millis() - g_searchStart) > SEARCH_TIMEOUT_MS) {
      Car_stop();                 // 长时间找不到线：停车，避免跑出赛道/逆行
      g_state = CAR_DONE;
      Serial.println(F("LOST-STOP"));
    }
    break;

  /* ---------------- 已结束: 保持停车 ---------------- */
  case CAR_DONE:
  default:
    Car_stop();
    break;
  }

  debug_print();
}


/* =======================================================================
 *  终点横线判定(先刹后判)
 *
 *  难点: 发夹弯顶部两条腿会并成黑块，读数与终点横线(500x20mm)很接近。
 *  对策三步:
 *    1) 闸门: 至少跑 12s 且 4.5m；宽黑前连续正常压线≥3周期；偏差包络<32mm(直线段)；
 *    2) 先刹: 命中候选立刻电子刹车(反正终点也要停，同时保住"停止分")；
 *    3) 后判: 68PWM 慢爬最多 450ms(≈90mm):
 *             - 黑块 250ms 内结束 → 薄横线 → 确认终点，停车结束；
 *             - 爬 250ms 还黑 → 发夹黑块(沿路径150mm+) → 误报；
 *             - 黑块一直偏在一侧 → 车在转弯 → 误报。
 *     误报只损失约 0.3~0.6s，绝不会把"终点前"当终点提前停车。
 * ======================================================================= */
static bool cross_gate_ok(void)
{
  uint32_t now = millis();

  if ((uint32_t)(now - g_startMs) < CROSS_MIN_MS) return false;   // 时间闸门
  if (line_dist_mm() < CROSS_MIN_MM)              return false;   // 里程闸门
  if ((uint32_t)(now - g_lastNormalMs) > CROSS_NORMAL_MS) return false; // 宽黑前必须一直正常压线
  if (line_err_envelope_mm() > CROSS_ENV_MM)      return false;   // 不是在急弯里

  return true;
}

static void cross_handle(void)
{
  uint32_t t0;
  uint8_t  result = 0;                     // 0=未定，1=终点确认，2=发夹/黑块误报
  uint32_t elapsed;

  Car_brake(BRAKE_MS);                     // 先刹：保住"停止分"(车尾不越线)

  t0 = millis();
  for (;;) {
    uint8_t m;
    uint8_t n;
    int     e;

    elapsed = (uint32_t)(millis() - t0);

    Car_drive(CREEP_PWM, CREEP_PWM, 1);    // 慢速前爬，看黑块是否很快结束
    delay(10);

    m = line_read();
    n = line_black_count(m);
    e = line_error_mm(m);

    if (n <= CREEP_CLEAR_CNT) {
      /* 黑块结束：只有"结束得足够快"才算终点横线。
         终点横线(20mm厚)无论正/斜穿黑块沿路径 20~60mm；发夹黑块 150mm+ */
      result = (elapsed <= CREEP_CLEAR_MS) ? 1 : 2;
      break;
    }
    if (e > CREEP_TURN_MM || e < -CREEP_TURN_MM) {
      result = 2;                          // 黑块长时间偏在一侧 → 车在转弯，是发夹
      break;
    }
    if (elapsed >= CREEP_MS) {
      result = 2;                          // 爬够了还是黑的 → 发夹黑块
      break;
    }
  }
  Car_stop();

  if (result == 1) {
    g_state = CAR_DONE;                     // 确认终点线 → 停车结束
    Serial.println(F("FINISH-STOP"));
  } else {
    /* 误报(发夹顶部/斜跨黑块) → 清计数继续巡线，绝不能在此停机 */
    g_wideCnt       = 0;
    g_normalRun     = 0;
    g_runBeforeWide = 0;
    g_state = CAR_RUN;
    Serial.println(F("CROSS-FALSE"));
  }
}

/* =======================================================================
 *  调试输出(线型/偏差/里程/避障，用于标定参数)
 *  一行格式:
 *   st状态 m=掩码 n=黑点数 k=黑块形态 e=偏差mm v=偏差包络mm t=秒 d=里程cm
 *   D=前方距离mm L=左扫描mm R=右扫描mm S=避障判定侧别
 *   k: 0正常(1~4路) 1宽黑(5~7路) 2全黑(8路) 3丢线(全白)
 *   S: +1障碍在右(向左绕) -1障碍在左(向右绕)
 * ======================================================================= */
static void debug_print(void)
{
#if DEBUG_SERIAL
  uint32_t now = millis();
  uint8_t  m;
  uint8_t  n;
  uint8_t  k;

  if ((uint32_t)(now - g_lastDebug) < DEBUG_PERIOD_MS) return;
  g_lastDebug = now;

  m = (uint8_t)((x1 << 7) | (x2 << 6) | (x3 << 5) | (x4 << 4) |
                (x5 << 3) | (x6 << 2) | (x7 << 1) | (x8 << 0));
  n = line_black_count(m);

  if      (n == 0) k = 3;
  else if (n >= 8) k = 2;
  else if (n >= 5) k = 1;
  else             k = 0;

  Serial.print(F("st"));  Serial.print(g_state);
  Serial.print(F(" m=")); if (m < 0x10) Serial.print('0'); Serial.print(m, HEX);
  Serial.print(F(" n=")); Serial.print(n);
  Serial.print(F(" k=")); Serial.print(k);
  Serial.print(F(" e=")); Serial.print(line_last_error_mm());
  Serial.print(F(" v=")); Serial.print(line_err_envelope_mm());
  Serial.print(F(" t=")); Serial.print((now - g_startMs) / 1000);
  Serial.print(F(" d=")); Serial.print(line_dist_mm() / 10);
  Serial.print(F(" D=")); Serial.print(avoid_last_dist_mm());
  Serial.print(F(" L=")); Serial.print(avoid_last_left_mm());
  Serial.print(F(" R=")); Serial.print(avoid_last_right_mm());
  Serial.print(F(" S=")); Serial.println(avoid_last_side());
#else
  (void)0;
#endif
}

/**
 * @brief 获取按键状态(消抖)
 */
int getKeyState(uint8_t pin)
{
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      return Press_KEY;
    }
    return Release_KEY;
  }
  return Release_KEY;
}
