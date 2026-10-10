#include "line_pid.hpp"

// 8 路红外状态变量
uint8_t x1 = 1, x2 = 1, x3 = 1, x4 = 1, x5 = 1, x6 = 1, x7 = 1, x8 = 1;

static float err = 0.0f;
static float last_err = 0.0f;
static float prev_err = 0.0f;
static float integral_x = 0.0f;

static bool  g_is_cross = false;
static int   g_black_cnt = 0;

void init_x_PID(void) {
  err = 0.0f;
  last_err = 0.0f;
  prev_err = 0.0f;
  integral_x = 0.0f;
  g_is_cross = false;
  g_black_cnt = 0;
}

/**
 * @brief 读取 8 路红外循迹模块 (I2C)
 *        保留原版硬件厂商标准通信时序与 10ms 准备延时，稳定绝不丢帧
 */
void I2Cdata(void) {
  Wire.beginTransmission(0x12); // I2C 设备地址
  Wire.write(0x30);             // 数据寄存器
  Wire.endTransmission();

  delay(10); // 关键硬件延时：给模块内部从机单片机准备数据的时间

  Wire.requestFrom(0x12, 1);
  if (Wire.available()) {
    byte data = Wire.read();
    x1 = (data >> 7) & 0x01;
    x2 = (data >> 6) & 0x01;
    x3 = (data >> 5) & 0x01;
    x4 = (data >> 4) & 0x01;
    x5 = (data >> 3) & 0x01;
    x6 = (data >> 2) & 0x01;
    x7 = (data >> 1) & 0x01;
    x8 = (data >> 0) & 0x01;
  }

  // 统计黑线探头并计算偏差 (探头权值对称分布：-7, -5, -3, -1, 1, 3, 5, 7)
  float sum = 0.0f;
  int cnt = 0;

  if (x1 == 0) { sum += -7.0f; cnt++; }
  if (x2 == 0) { sum += -5.0f; cnt++; }
  if (x3 == 0) { sum += -3.0f; cnt++; }
  if (x4 == 0) { sum += -1.0f; cnt++; }
  if (x5 == 0) { sum +=  1.0f; cnt++; }
  if (x6 == 0) { sum +=  3.0f; cnt++; }
  if (x7 == 0) { sum +=  5.0f; cnt++; }
  if (x8 == 0) { sum +=  7.0f; cnt++; }

  g_black_cnt = cnt;

  // 1. 横线判定：有 6 路及以上同时检测到黑线，说明压在 500mm 起点/终点线上
  if (cnt >= 6) {
    g_is_cross = true;
    err = 0.0f; // 横线上保持直行
  } else {
    g_is_cross = false;
    
    // 2. 丢线处理：全白 (发夹弯急打或小车轻微冲出)
    if (cnt == 0) {
      // 沿用上一次有效偏差方向继续加大转向，快速寻回黑线
      if (last_err > 0.5f) {
        err = 6.0f;   // 保持向右大力拉回
      } else if (last_err < -0.5f) {
        err = -6.0f;  // 保持向左大力拉回
      } else {
        err = 0.0f;
      }
    } 
    // 3. 正常压线 (1~5 路黑)：连续平滑加权
    else {
      err = sum / (float)cnt;
      last_err = err;
    }
  }
}

/**
 * @brief 标准位置式 PID 计算转向差速
 */
int PID_count_x(void) {
  float d_err = err - prev_err;
  
  integral_x += err;
  if (integral_x > 30.0f)  integral_x = 30.0f;
  if (integral_x < -30.0f) integral_x = -30.0f;

  float pwm_out = (err * KPx) + (integral_x * KIx) + (d_err * KDx);
  prev_err = err;

  return (int)pwm_out;
}

/**
 * @brief 自适应弯道巡线
 *        直道全速提高时间分；急弯自适应降速保障绝对稳定性
 */
void Car_line_track(void) {
  int steer_pwm = PID_count_x();

  float abs_err = fabs(err);
  int base_speed = Speed_Line;

  if (abs_err <= 1.5f) {
    base_speed = Speed_Line;   // 直道全速
  } else if (abs_err <= 3.5f) {
    base_speed = Speed_Curve;  // 普通弯道微降速
  } else {
    base_speed = Speed_Sharp;  // 急弯/发夹弯大降速保证抓地与转弯灵敏度
  }

  Set_speed(base_speed, steer_pwm);
}

bool isCrossLine(void) {
  return g_is_cross;
}

int getBlackCount(void) {
  return g_black_cnt;
}

float getLineError(void) {
  return err;
}
