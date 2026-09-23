#include "line_pid.hpp"

/*
 * 针对 20 mm 黑线和 8 路横向传感器的参数。
 * 先只调 LINE_KP；高速过弯摆动时减小 LINE_KP 或增大 LINE_KD。
 */
#define LINE_SPEED_STRAIGHT      128
#define LINE_SPEED_TURN          92
#define LINE_KP                  15.0f
#define LINE_KI                  0.45f
#define LINE_KD                  0.16f
#define LINE_MAX_CORRECTION      125
#define LOST_LINE_TURN_PWM       130

// x1 到 x8：左负、右正。黑线落在右侧时应给出正修正，使小车向右转。
static const int8_t kSensorPosition[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };

static float lineError = 0.0f;
static float previousError = 0.0f;
static float integral = 0.0f;
static unsigned long previousPidMs = 0;
static uint8_t blackCount = 0;
static bool lineDetected = false;
static bool lineCrossing = false;
static int lastLineDirection = 1;

// x1-x8 从左至右，0 表示黑线，1 表示白色底板。
uint8_t x1, x2, x3, x4, x5, x6, x7, x8;

static int clampInt(int value, int low, int high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void ResetLineController(void)
{
  lineError = 0.0f;
  previousError = 0.0f;
  integral = 0.0f;
  previousPidMs = millis();
}

void init_x_PID(void)
{
  blackCount = 0;
  lineDetected = false;
  lineCrossing = false;
  lastLineDirection = 1;
  ResetLineController();
}

/*
 * 原程序只枚举了少量位型，弯道、边缘单点、黑线压到相邻三点时会保留旧误差。
 * 这里用所有黑色探头的加权质心计算位置，因此 256 种位型都能得到确定结果。
 */
void LineWalking(void)
{
  const uint8_t sensor[8] = { x1, x2, x3, x4, x5, x6, x7, x8 };
  int positionSum = 0;

  blackCount = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    if (sensor[i] == 0) {
      positionSum += kSensorPosition[i];
      ++blackCount;
    }
  }

  lineDetected = (blackCount > 0);
  // 20 mm 的普通循迹线不可能同时覆盖 7 个以上探头；该特征用于识别终点横线。
  lineCrossing = (blackCount >= 7);

  if (lineDetected && !lineCrossing) {
    lineError = (float)positionSum / (float)blackCount;
    if (lineError > 0.15f) {
      lastLineDirection = 1;
    } else if (lineError < -0.15f) {
      lastLineDirection = -1;
    }
  } else if (lineCrossing) {
    // 横线/起终点线没有左右误差；不要把此前的方向信息抹掉，丢线时还要用它搜索。
    lineError = 0.0f;
  }
}

int PID_count_x(void)
{
  LineWalking();

  if (!lineDetected) {
    // 失线时由 Car_line_track 原地向最后一次看到黑线的方向找线。
    return lastLineDirection * LOST_LINE_TURN_PWM;
  }

  const unsigned long now = millis();
  float dt = (float)(now - previousPidMs) / 1000.0f;
  // I2C 循环通常为数毫秒；限制 dt 可避免首次运行或调试暂停后的微分尖峰。
  if (dt < 0.010f) dt = 0.010f;
  if (dt > 0.080f) dt = 0.080f;

  if (lineCrossing) {
    integral = 0.0f;
    previousError = 0.0f;
    previousPidMs = now;
    return 0;
  }

  integral += lineError * dt;
  if (integral > 8.0f) integral = 8.0f;
  if (integral < -8.0f) integral = -8.0f;

  const float derivative = (lineError - previousError) / dt;
  const float output = LINE_KP * lineError + LINE_KI * integral + LINE_KD * derivative;

  previousError = lineError;
  previousPidMs = now;
  return clampInt((int)output, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
}

void Car_line_track(void)
{
  const int correction = PID_count_x();

  if (!lineDetected) {
    // 两轮反向转动，比带着较大前进速度盲冲更容易在 S 弯和避障回归时重新压线。
    Set_speed(0, correction);
    return;
  }

  int speed = LINE_SPEED_STRAIGHT;
  const float absError = (lineError < 0.0f) ? -lineError : lineError;
  if (lineCrossing) {
    speed = LINE_SPEED_TURN;
  } else if (absError >= 4.0f) {
    speed = LINE_SPEED_TURN;
  } else if (absError >= 2.0f) {
    speed = (LINE_SPEED_STRAIGHT + LINE_SPEED_TURN) / 2;
  }

  Set_speed(speed, correction);
}

bool IsLineDetected(void)
{
  return lineDetected;
}

bool IsLineCrossing(void)
{
  return lineCrossing;
}

uint8_t GetLineBlackCount(void)
{
  return blackCount;
}

int GetLastLineError(void)
{
  return (int)lineError;
}
