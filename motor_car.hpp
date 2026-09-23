#ifndef MOTOR_CAR_HPP
#define MOTOR_CAR_HPP

#include <Arduino.h>
#include <Wire.h>

// 电机控制引脚
#define Motor_L_PWM 3
#define Motor_L_IN1 4
#define Motor_L_IN2 5
#define Motor_R_PWM 11
#define Motor_R_IN1 9
#define Motor_R_IN2 10

void Motor_init(void);
void StopCar(void);
void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2,
                   uint16_t motor_pin_pwm, int motor_speed);

int myignore_speed(int speed);
int limin_speed(int speed, int max, int min);

/*
 * speed_fb: 前进/后退基准速度；speed_lr: 左右转向修正。
 * 正 speed_lr 会使左轮更快、右轮更慢，即小车向右转。
 */
void Set_speed(int speed_fb, int speed_lr);

#endif
