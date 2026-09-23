#ifndef MOTOR_CAR_HPP
#define MOTOR_CAR_HPP

#include <Arduino.h>
#include <Wire.h>

/* =========================================================================
 *  运动控制层 (双直流电机 + L298/TB6612 类H桥)
 *
 *  融合说明：
 *   - 采用 -ds 版丰富的运动原语 (Car_drive / Car_pivot / Car_brake /
 *     Set_speed_ex)，避障与终点刹车都要用到；
 *   - 采用根目录版的左右电机比例校正 (MOTOR_LEFT_SCALE / MOTOR_RIGHT_SCALE)，
 *     用于消除机械/电气不对称造成的直行跑偏，优先于用大积分去补偿。
 * ========================================================================= */

// ---------- 电机控制引脚 ----------
#define Motor_L_PWM 3   // 左电机 PWM
#define Motor_L_IN1 4   // 左电机方向
#define Motor_L_IN2 5
#define Motor_R_PWM 11  // 右电机 PWM
#define Motor_R_IN1 9   // 右电机方向
#define Motor_R_IN2 10

void Motor_init(void);
void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2,
                   uint16_t motor_pin_pwm, int motor_speed);

int myignore_speed(int speed);   // 低速抬到电机起转门限(克服静摩擦)
int limin_speed(int speed, int max, int min);

/*
 * speed_fb: 前进/后退基准速度；speed_lr: 左右转向修正。
 * 约定：正 speed_lr → 左轮更快、右轮更慢，即小车向右转。
 */
void Set_speed(int speed_fb, int speed_lr);
// 与 Set_speed 相同，但可关闭死区(原地转/内轮停转时必须关闭)
void Set_speed_ex(int speed_fb, int speed_lr, uint8_t deadzone_en);

// 直接给左右轮带符号速度；deadzone_en=1 时低速自动抬到起转速度(直行用)
void Car_drive(int speed_l, int speed_r, uint8_t deadzone_en);
// 立即停车(占空比0，自由滑行)
void Car_stop(void);
// 兼容旧接口：等价于 Car_stop()
void StopCar(void);
// 电子刹车：反向通电 brake_ms 毫秒再停(终点停车分的关键，比断电滑行停得快)
void Car_brake(uint16_t brake_ms);
// 原地转：dir>0 右转(左轮正/右轮反)，dir<0 左转
void Car_pivot(int8_t dir, uint8_t pwm);

#endif
