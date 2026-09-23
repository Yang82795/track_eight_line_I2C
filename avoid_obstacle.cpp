#include "avoid_obstacle.hpp"
#include <Servo.h>

/* =========================================================================
 *  超声波避障 (HC-SR04 + 舵机扫描，用 pulseIn 自己读，不依赖 HCSR04 库)
 *
 *  流程：
 *   1) 平时每 SONAR_PERIOD_MS 用"车头正前"测一次，连续 2 次都近才认障碍；
 *   2) 确认后停车，舵机左右各两点扫描，取每侧较小值比较，哪侧开阔往哪侧绕；
 *      左右净空接近时"交替绕行"，避免总往场地同一侧偏；
 *   3) 原地偏出一定角度，再用"目标偏置"贴边循迹通过(PID 目标点偏到线另一侧)；
 *   4) 通过后清除偏置，交回主循环 PD 把车拉回线上，并屏蔽超声波一段时间。
 * ========================================================================= */

/* ------------------------------ 引脚 ------------------------------ */
#define TrigPin          (8)
#define EchoPin          (7)
#define ServoPin         (6)

/* --------------------------- 超声波参数 --------------------------- */
#define SONAR_TIMEOUT_US (5000)    // pulseIn 超时(约 850mm)，越小越省时间
#define SONAR_PERIOD_MS  (60)      // 平时测距间隔
#define SONAR_TRIGGER_MM (300)     // 触发避障的距离
#define SONAR_CLEAR_MM   (280)     // 偏出后再测，大于此值认为前方已让开
#define SONAR_PASSED_MM  (450)     // 通过阶段：距离大于此值认为障碍已过
#define SONAR_MAX_MM     (850)     // 超时时的替代值

/* ---------------------------- 舵机扫描 ---------------------------- */
/* 注意：若实测左右相反，把 LEFT/RIGHT 两组宏对调即可 */
#define SERVO_L2         (60)      // 左偏大
#define SERVO_L1         (80)      // 左偏小
#define SERVO_MID        (95)      // 中位
#define SERVO_R1         (110)     // 右偏小
#define SERVO_R2         (130)     // 右偏大
#define SERVO_SETTLE_MS  (70)      // 舵机到位等待
#define SIDE_HYST_MM     (120)     // 左右差值超过此值才判定方向

/* ---------------------------- 动作参数 ---------------------------- */
#define PIVOT_PWM        (95)      // 偏出时的原地转速度
#define PIVOT_OUT_MS     (260)     // 首次偏出时长
#define PIVOT_EXTRA_MS   (130)     // 前方仍被挡时追加偏出时长
#define PASS_PWM         (85)      // 通过阶段速度
#define PASS_MIN_MS      (350)     // 通过阶段最短时间(防止立刻判"已过")
#define PASS_MAX_MS      (1600)    // 通过阶段最长时间
#define LINE_OFFSET_MM   (22)      // 贴边偏置：让线保持在障碍侧，车体让开
#define BLANK_MS         (650)     // 避障结束后屏蔽时间
#define AVOID_TIMEOUT_MS (3500)    // 整个避障过程的屏蔽时间(防止中途重复触发)

/* ------------------------------ 全局 ------------------------------ */
static Servo    g_servo;
static uint32_t g_blankUntil  = 0;
static uint32_t g_sonarLastMs = 0;
static uint16_t g_lastD       = SONAR_MAX_MM;
static int      g_lastSide    = 1;
static int      g_prevSide    = -1;   // 上一次绕行方向(用于交替)
static uint16_t g_lastDist    = 0;
static uint16_t g_lastLeft    = 0;
static uint16_t g_lastRight   = 0;

void avoid_init(void)
{
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  digitalWrite(TrigPin, LOW);
  g_servo.attach(ServoPin);
  delay(50);
  g_servo.write(SERVO_MID);
  delay(200);
  g_blankUntil = millis() + 600;
}

/**
 * @brief 单次超声波测距
 * @retval 距离(mm)；返回 0 表示超时(前方量程内无障碍)
 */
uint16_t sonar_read_mm(void)
{
  uint32_t us;

  digitalWrite(TrigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  us = pulseIn(EchoPin, HIGH, SONAR_TIMEOUT_US);
  if (us == 0) return 0;                          // 超时：没有回波
  return (uint16_t)((us * 17UL) / 100UL);         // us*0.1715 ≈ mm
}

/** @brief 屏蔽超声波一段时间(只延长，不缩短) */
void avoid_blank(uint16_t ms)
{
  uint32_t until = millis() + ms;
  if ((int32_t)(until - g_blankUntil) > 0) g_blankUntil = until;
}

/** @brief 是否需要避障(限频 + 连续 2 次确认 + 屏蔽期) */
bool avoid_need(void)
{
  uint16_t d;
  bool near_now;

  if ((int32_t)(millis() - g_blankUntil) < 0) return false;             // 仍在屏蔽期
  if ((uint32_t)(millis() - g_sonarLastMs) < SONAR_PERIOD_MS) return false;
  g_sonarLastMs = millis();

  d = sonar_read_mm();
  if (d == 0) d = SONAR_MAX_MM;      // 超时=前方开阔
  g_lastDist = d;

  near_now = (d <= SONAR_TRIGGER_MM);
  if (near_now && (g_lastD <= SONAR_TRIGGER_MM)) {
    g_lastD = d;
    return true;                     // 连续两次都近 → 确认障碍
  }
  g_lastD = d;
  return false;
}

static uint16_t sonar_scan(uint8_t angle)
{
  uint16_t d;
  g_servo.write(angle);
  delay(SERVO_SETTLE_MS);
  d = sonar_read_mm();
  if (d == 0) d = SONAR_MAX_MM;
  return d;
}

/**
 * @brief 左右扫描判断绕行方向
 * @retval +1: 障碍在右 → 向左绕；-1: 障碍在左 → 向右绕
 *
 *  两侧净空接近时(差值 < SIDE_HYST_MM)交替选择绕行方向，避免总往
 *  同一侧场地边界偏移(借鉴根目录版思路)。
 */
static int avoid_scan_sides(void)
{
  uint16_t l2 = sonar_scan(SERVO_L2);
  uint16_t l1 = sonar_scan(SERVO_L1);
  uint16_t r1 = sonar_scan(SERVO_R1);
  uint16_t r2 = sonar_scan(SERVO_R2);
  uint16_t dl, dr;
  int side;

  g_servo.write(SERVO_MID);

  dl = (l1 < l2) ? l1 : l2;
  dr = (r1 < r2) ? r1 : r2;
  g_lastLeft  = dl;
  g_lastRight = dr;

  if (dl > (uint16_t)(dr + SIDE_HYST_MM)) {
    side = +1;                          // 左侧明显开阔 → 向左绕
  } else if (dr > (uint16_t)(dl + SIDE_HYST_MM)) {
    side = -1;                          // 右侧明显开阔 → 向右绕
  } else {
    side = -g_prevSide;                 // 分不清 → 交替绕行
  }
  g_prevSide = side;
  return side;
}

/**
 * @brief 完整避障流程(内部有延时，单次约 0.6~1.8s)
 */
void avoid_run(void)
{
  uint32_t ts, sonarT;
  int side;

  Car_stop();
  delay(80);

  side = avoid_scan_sides();
  g_lastSide = side;
  avoid_blank(AVOID_TIMEOUT_MS);       // 整个动作期间不再触发

  /* 1) 向空侧原地偏出 */
  Car_pivot((int8_t)(-side), PIVOT_PWM);
  delay(PIVOT_OUT_MS);
  Car_stop();
  delay(40);

  /* 2) 前方仍很近 → 再偏一点(最多 2 次，闭环) */
  for (uint8_t i = 0; i < 2; i++) {
    uint16_t d = sonar_read_mm();
    if (d == 0 || d > SONAR_CLEAR_MM) break;
    Car_pivot((int8_t)(-side), PIVOT_PWM);
    delay(PIVOT_EXTRA_MS);
    Car_stop();
    delay(40);
  }

  /* 3) 贴边通过：目标偏置设为障碍侧，让车体整体让到线的另一侧 */
  ts = millis();
  sonarT = ts;
  while ((uint32_t)(millis() - ts) < PASS_MAX_MS) {
    uint8_t m = line_read();
    uint8_t n = line_black_count(m);

    if (n > 0) {
      line_follow_speed(m, PASS_PWM, side * LINE_OFFSET_MM);
    } else {
      Car_drive(PASS_PWM, PASS_PWM, 1);   // 线暂时看不见，保持直行
    }

    if ((uint32_t)(millis() - sonarT) >= 80) {
      uint16_t d;
      sonarT = millis();
      d = sonar_read_mm();
      if (((uint32_t)(millis() - ts) >= PASS_MIN_MS) &&
          (d == 0 || d > SONAR_PASSED_MM)) {
        break;                            // 障碍已过
      }
    }
    delay(5);
  }

  /* 4) 收尾：清除偏置，交回主循环 PD 把车拉回线 */
  line_set_offset(0);
  Car_stop();
  avoid_blank(BLANK_MS);
}

int      avoid_last_side(void)     { return g_lastSide; }
uint16_t avoid_last_dist_mm(void)  { return g_lastDist; }
uint16_t avoid_last_left_mm(void)  { return g_lastLeft; }
uint16_t avoid_last_right_mm(void) { return g_lastRight; }
