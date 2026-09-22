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


#ifdef __cplusplus
}
#endif

#endif
