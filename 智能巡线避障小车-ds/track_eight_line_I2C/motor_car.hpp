#ifndef __MOTOR_CAR_HPP_
#define __MOTOR_CAR_HPP_

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>                     // 包含Wire(I2C)通讯库 Include Wire library



#ifdef __cplusplus
extern "C" {
#endif


// 定义电机控制引脚 Define motor control pins
#define Motor_L_PWM 3  // 控制小车左边电机速度

#define Motor_L_IN1 4  // 控制小车左边电机正反转 
#define Motor_L_IN2 5

#define Motor_R_PWM 11 //  控制小车右边电机速度

#define Motor_R_IN1 9  // 控制小车右边电机正反转
#define Motor_R_IN2 10  





void Motor_init();
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin,uint16_t motor_pin_pwm,int motor_speed);  // 设置单个电机速度 Setting the Motor Speed


int myignore_speed(int speed);
int limin_speed(int speed,int max,int min);
void Set_speed(int speed_fb,int speed_lr);

/* ---------------- 以下为循迹/避障专用运动接口 ---------------- */

// 直接给左右轮带符号速度, deadzone_en=1 时低速自动抬到起转速度(直行用)
void Car_drive(int speed_l, int speed_r, uint8_t deadzone_en);
// 与 Set_speed 相同, 但可关闭死区(原地转/内轮停转时必须关闭)
void Set_speed_ex(int speed_fb, int speed_lr, uint8_t deadzone_en);
// 立即停车(占空比0, 滑行)
void Car_stop(void);
// 电子刹车: 反向通 brake_ms 毫秒再停(终点线停车分关键)
void Car_brake(uint16_t brake_ms);
// 原地转: dir>0 右转(左轮正/右轮反), dir<0 左转
void Car_pivot(int8_t dir, uint8_t pwm);


#ifdef __cplusplus
}
#endif

#endif
