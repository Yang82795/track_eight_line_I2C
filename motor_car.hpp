#ifndef __MOTOR_CAR_HPP_
#define __MOTOR_CAR_HPP_

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================
// 电机控制引脚定义 (与原版 100% 一致)
// ==========================================
#define Motor_L_PWM 3  // 控制小车左边电机速度
#define Motor_L_IN1 4  // 控制小车左边电机正反转 
#define Motor_L_IN2 5

#define Motor_R_PWM 11 // 控制小车右边电机速度
#define Motor_R_IN1 9  // 控制小车右边电机正反转
#define Motor_R_IN2 10  

// ==========================================
// 左右轮机械对称性校正 (默认 1.0)
// 若小车直行明显偏右，可微调左轮 0.96 或右轮 1.04
// ==========================================
#define MOTOR_LEFT_SCALE   1.00f
#define MOTOR_RIGHT_SCALE  1.00f

// ==========================================
// 运动控制接口
// ==========================================
void Motor_init(void);
void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2, uint16_t motor_pin_pwm, int motor_speed);

int myignore_speed(int speed);
int limin_speed(int speed, int max, int min);

// speed_fb: 前进后退基准速度(PWM), speed_lr: 左右转向差速量(PWM)
void Set_speed(int speed_fb, int speed_lr);

// 直接设置左右电机差速 (用于避障绕行与平滑转向)
void Set_motor_direct(int speed_l, int speed_r);

// 正常停止小车 (断电滑行停止)
void StopCar(void);

// 强效反接电子刹车 (终点线精准停车专用，反向通电 90ms 彻底消除惯性越线)
void StopCarBrake(void);

#ifdef __cplusplus
}
#endif

#endif
