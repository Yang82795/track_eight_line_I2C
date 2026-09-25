#ifndef __AVOID_OBSTACLE_HPP_
#define __AVOID_OBSTACLE_HPP_

#include <Arduino.h>
#include <HCSR04.h>
#include <Servo.h>
#include "motor_car.hpp"
#include "line_pid.hpp"

// ==========================================
// 超声波与舵机引脚定义 (与原版完全一致)
// ==========================================
#define EchoPin          7
#define TrigPin          8
#define ServoPin         6

// ==========================================
// 避障动作参数 (现场可微调)
// ==========================================
#define SERVO_CENTER_DEG 95   // 舵机正前中位角度
#define SERVO_LEFT_DEG   65   // 舵机向左探照角度
#define SERVO_RIGHT_DEG  125  // 舵机向右探照角度

#define AVOID_TRIGGER_CM 25   // 触发避障的前方距离阈值 (cm)
#define AVOID_COOLDOWN_MS 1500 // 避障完成后屏蔽超声波时间，防止重复触发 (ms)

// 绕障三段动作耗时参数
#define TIME_TURN_OUT_MS 360  // 第一阶段：向开阔侧偏出时间 (ms)
#define TIME_PASS_MS     520  // 第二阶段：直行超越障碍物时间 (ms)
#define TIME_RECOVER_MAX 900  // 第三阶段：斜向切回寻线的最长超时时间 (ms)

// ==========================================
// 接口函数
// ==========================================
void Avoid_init(void);
bool Avoid_detect(void);      // 检测前方是否有障碍 (滤波防误触)
void Avoid_perform(void);     // 执行快速选向-弧线绕障-寻线闭环回正

#endif
