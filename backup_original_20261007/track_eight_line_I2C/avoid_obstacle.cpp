#include "Arduino.h"
#include "avoid_obstacle.hpp"

static unsigned long last_detect_time = 0;
static unsigned long cooldown_until = 0;
static uint8_t left_confirm_cnt = 0;
static uint8_t right_confirm_cnt = 0;
static ObstacleSide g_detected_side = OBS_NONE;

#if (AVOID_SWITCH_MODE == AVOID_MODE_ALL_ON)
static uint8_t g_ping_turn = 0; // 全开模式下交替采样左右探头，彻底消除声波串扰与双倍延时
#endif

unsigned long g_shield_finish_until = 0; // 避障后终点线屏蔽截止时刻

/**
 * @brief 超声波快速无锁测距函数
 * @param trigPin 发射引脚
 * @param echoPin 接收引脚
 * @return 障碍物距离 (cm)，若前方无障碍物(>60cm)则快速返回 999.0f
 * @note 超时限定为 3500us (约60cm)，当无障碍时 3.5ms 极速返回，且绝不关闭中断，不扰动电机 PWM 和 I2C
 */
static float get_ultrasonic_distance_cm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 3500UL); // 3500us ≈ 60cm 超时
  if (duration == 0) {
    return 999.0f; // 60cm 范围内无障碍
  }
  return (float)duration / 58.2f;
}

void Avoid_init(void) {
#if (AVOID_SWITCH_MODE == AVOID_MODE_LEFT_ONLY || AVOID_SWITCH_MODE == AVOID_MODE_ALL_ON)
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  digitalWrite(TRIG_PIN_LEFT, LOW);
#endif

#if (AVOID_SWITCH_MODE == AVOID_MODE_RIGHT_ONLY || AVOID_SWITCH_MODE == AVOID_MODE_ALL_ON)
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  digitalWrite(TRIG_PIN_RIGHT, LOW);
#endif

  cooldown_until = millis() + 800; // 开机前 800ms 屏蔽避障
  g_shield_finish_until = 0;
}

/**
 * @brief 实时检测两侧障碍物 (根据总开关模式执行)
 * @return ObstacleSide: OBS_NONE(0), OBS_LEFT(1), OBS_RIGHT(2), OBS_BOTH(3)
 */
ObstacleSide Avoid_detect(void) {
#if (AVOID_SWITCH_MODE == AVOID_MODE_OFF)
  return OBS_NONE; // 【全关模式】：零耗时直接返回，纯巡线绝对防误触
#else

  unsigned long now = millis();
  if (now < cooldown_until) return OBS_NONE;
  if (now - last_detect_time < 50) return OBS_NONE; // 50ms 节拍检测
  last_detect_time = now;

#if (AVOID_SWITCH_MODE == AVOID_MODE_LEFT_ONLY)
  // ----------------------------------------------------
  // 【只左模式】仅检测左侧超声波，右侧未插线完全不读取
  // ----------------------------------------------------
  float d_left = get_ultrasonic_distance_cm(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  if (d_left > 2.0f && d_left <= AVOID_TRIGGER_CM_LEFT) {
    left_confirm_cnt++;
    if (left_confirm_cnt >= AVOID_CONFIRM_TIMES) {
      left_confirm_cnt = 0;
      g_detected_side = OBS_LEFT;
      return OBS_LEFT;
    }
  } else {
    left_confirm_cnt = 0;
  }
  return OBS_NONE;

#elif (AVOID_SWITCH_MODE == AVOID_MODE_RIGHT_ONLY)
  // ----------------------------------------------------
  // 【只右模式】仅检测右侧超声波
  // ----------------------------------------------------
  float d_right = get_ultrasonic_distance_cm(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  if (d_right > 2.0f && d_right <= AVOID_TRIGGER_CM_RIGHT) {
    right_confirm_cnt++;
    if (right_confirm_cnt >= AVOID_CONFIRM_TIMES) {
      right_confirm_cnt = 0;
      g_detected_side = OBS_RIGHT;
      return OBS_RIGHT;
    }
  } else {
    right_confirm_cnt = 0;
  }
  return OBS_NONE;

#elif (AVOID_SWITCH_MODE == AVOID_MODE_ALL_ON)
  // ----------------------------------------------------
  // 【全开模式】交替采样左右超声波 (防声波串扰与双倍延迟)
  // ----------------------------------------------------
  if (g_ping_turn == 0) {
    g_ping_turn = 1;
    float d_left = get_ultrasonic_distance_cm(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    if (d_left > 2.0f && d_left <= AVOID_TRIGGER_CM_LEFT) {
      left_confirm_cnt++;
    } else {
      left_confirm_cnt = 0;
    }
  } else {
    g_ping_turn = 0;
    float d_right = get_ultrasonic_distance_cm(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    if (d_right > 2.0f && d_right <= AVOID_TRIGGER_CM_RIGHT) {
      right_confirm_cnt++;
    } else {
      right_confirm_cnt = 0;
    }
  }

  if (left_confirm_cnt >= AVOID_CONFIRM_TIMES && right_confirm_cnt >= AVOID_CONFIRM_TIMES) {
    left_confirm_cnt = 0;
    right_confirm_cnt = 0;
    g_detected_side = OBS_BOTH;
    return OBS_BOTH;
  } else if (left_confirm_cnt >= AVOID_CONFIRM_TIMES) {
    left_confirm_cnt = 0;
    right_confirm_cnt = 0;
    g_detected_side = OBS_LEFT;
    return OBS_LEFT;
  } else if (right_confirm_cnt >= AVOID_CONFIRM_TIMES) {
    left_confirm_cnt = 0;
    right_confirm_cnt = 0;
    g_detected_side = OBS_RIGHT;
    return OBS_RIGHT;
  }

  return OBS_NONE;
#endif

#endif
}

/**
 * @brief 直走切回黑线辅助函数：双轮绝对等速直行，全通道红外见线即停
 */
static bool wait_return_to_line(int speed, unsigned long max_ms) {
  Set_motor_direct(speed, speed); // 双轮绝对等速直走
  unsigned long t0 = millis();
  while (millis() - t0 < max_ms) {
    I2Cdata();
    // 8 路红外任意一路压到黑线，说明成功切回黑线！
    if (x1 == 0 || x2 == 0 || x3 == 0 || x4 == 0 || x5 == 0 || x6 == 0 || x7 == 0 || x8 == 0) {
      StopCar();
      delay(40);
      return true; // 见线即停
    }
    delay(10);
  }
  StopCar();
  delay(40);
  return false;
}

/**
 * @brief 执行避障核心动作
 * @param side: 传入检测到的障碍物侧别
 */
void Avoid_perform(ObstacleSide side) {
  StopCar();
  delay(TIME_INITIAL_STOP_MS); // 初始刹停

  if (side == OBS_NONE) {
    side = g_detected_side;
  }

  // 1. 确定绕行方向
  // bypass_left == true:  向左绕行 (用于避开右侧障碍)
  // bypass_left == false: 向右绕行 (用于避开左侧障碍)
  bool bypass_left = false;

  if (side == OBS_LEFT) {
    bypass_left = false; // 障碍在左侧 -> 向右绕行
  } else if (side == OBS_RIGHT) {
    bypass_left = true;  // 障碍在右侧 -> 向左绕行
  } else {
    // 双侧同时触发或未决 -> 依据优先方向
    bypass_left = (AVOID_PRIORITY_DIR == 1);
  }

  // ----------------------------------------------------
  // 阶段 1：向外偏出 (转角约 45 度)
  // ----------------------------------------------------
  if (bypass_left) {
    Set_motor_direct(SPEED_TURN_OUT_SLOW, SPEED_TURN_OUT_FAST); // 左慢右快 -> 向左偏
  } else {
    Set_motor_direct(SPEED_TURN_OUT_FAST, SPEED_TURN_OUT_SLOW); // 左快右慢 -> 向右偏
  }
  delay(TIME_TURN_OUT_MS);

  // 停顿缓冲 1：消除外甩惯性
  StopCar();
  delay(TIME_PAUSE_MS);

  // ----------------------------------------------------
  // 阶段 2：超越障碍物并充足回转对准黑线 (弧线收拢)
  // ----------------------------------------------------
  if (bypass_left) {
    Set_motor_direct(SPEED_PASS_FAST, SPEED_PASS_SLOW); // 左快右慢 -> 向右收拢
  } else {
    Set_motor_direct(SPEED_PASS_SLOW, SPEED_PASS_FAST); // 左慢右快 -> 向左收拢
  }
  delay(TIME_PASS_MS);

  // 停顿缓冲 2：车头已对准黑线，短暂停稳
  StopCar();
  delay(TIME_PAUSE_MS);

  // ----------------------------------------------------
  // 阶段 3：直走切回黑线到底 (双轮等速笔直前进，全通道红外见线即停)
  // ----------------------------------------------------
  wait_return_to_line(SPEED_RETURN, TIME_RETURN_MAX);

  // ----------------------------------------------------
  // 阶段 4：触线顺势微调拉直车身，与黑线平行
  // ----------------------------------------------------
  if (bypass_left) {
    Set_motor_direct(SPEED_ALIGN_SLOW, SPEED_ALIGN_FAST); // 向左微打拉直
  } else {
    Set_motor_direct(SPEED_ALIGN_FAST, SPEED_ALIGN_SLOW); // 向右微打拉直
  }
  delay(TIME_ALIGN_MS);

  // ----------------------------------------------------
  // 阶段 5：收尾停车，清空 PID 历史，激活防误判终点与冷却保护
  // ----------------------------------------------------
  StopCar();
  delay(40);
  init_x_PID(); // 清空 PID 历史误差，防止循迹接管瞬间剧烈甩尾
  
  cooldown_until = millis() + AVOID_COOLDOWN_MS;
  g_shield_finish_until = millis() + AVOID_FINISH_SHIELD_MS; // ★★★ 避障回线后 5 秒内严禁误判终点线！
}
