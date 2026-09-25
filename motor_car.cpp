#include "Arduino.h"
#include "motor_car.hpp"

#define Ignore_speed   (60)   // 电机克服静摩擦的最小起转 PWM
#define MAX_Speed      (255)  // 最大速度限制
#define BRAKE_SPEED    (120)  // 电子刹车反接制动 PWM
#define BRAKE_MS       (90)   // 电子刹车反向脉冲时长 (毫秒)

static int speed_L = 0;
static int speed_R = 0;

void Motor_init(void) {
  pinMode(Motor_L_PWM, OUTPUT);
  pinMode(Motor_L_IN1, OUTPUT);
  pinMode(Motor_L_IN2, OUTPUT);
  pinMode(Motor_R_PWM, OUTPUT);
  pinMode(Motor_R_IN1, OUTPUT);
  pinMode(Motor_R_IN2, OUTPUT);
  
  StopCar();
  delay(100); // 上电稳定延时
}

/**
 * @brief 设置单个电机速度
 * @param motor_pin1: 方向引脚 1
 * @param motor_pin2: 方向引脚 2
 * @param motor_pin_pwm: PWM 调速引脚
 * @param motor_speed: 速度 (-255 ~ 255)
 */
void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2, uint16_t motor_pin_pwm, int motor_speed) {
  if (motor_speed > 0) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
    analogWrite(motor_pin_pwm, motor_speed);
  } else if (motor_speed < 0) {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, HIGH);
    analogWrite(motor_pin_pwm, (-motor_speed));
  } else {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, LOW);
    analogWrite(motor_pin_pwm, 0);
  }
}

int myignore_speed(int speed) {
  if (speed == 0) return 0;
  
  if (speed < 0) {
    if (speed > -Ignore_speed) {
      speed = -Ignore_speed;
    }
  } else if (speed < Ignore_speed) {
    speed = Ignore_speed;
  }
  return speed;
}

int limin_speed(int speed, int max, int min) {
  if (speed > max) return max;
  if (speed < min) return min;
  return speed;
}

/**
 * @brief 差速合成输出
 * @param speed_fb: 前进基准速度 (y 轴)
 * @param speed_lr: 左右转向差速 (x 轴，正值代表向右转，左轮加、右轮减)
 */
void Set_speed(int speed_fb, int speed_lr) {
  speed_L = speed_fb + speed_lr;
  speed_R = speed_fb - speed_lr;

  // 左右轮机械对称性修正
  speed_L = (int)(speed_L * MOTOR_LEFT_SCALE);
  speed_R = (int)(speed_R * MOTOR_RIGHT_SCALE);

  // 满足速度范围
  speed_L = limin_speed(speed_L, MAX_Speed, -MAX_Speed);
  speed_R = limin_speed(speed_R, MAX_Speed, -MAX_Speed);

  // 消除死区 (只有前进且非零时处理)
  speed_L = myignore_speed(speed_L);
  speed_R = myignore_speed(speed_R);

  // 输出到电机
  setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, speed_L);
  setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, speed_R);
}

/**
 * @brief 直接设置左右轮速度 (避障弧线专用)
 */
void Set_motor_direct(int s_left, int s_right) {
  s_left  = (int)(s_left * MOTOR_LEFT_SCALE);
  s_right = (int)(s_right * MOTOR_RIGHT_SCALE);

  s_left  = limin_speed(s_left, MAX_Speed, -MAX_Speed);
  s_right = limin_speed(s_right, MAX_Speed, -MAX_Speed);

  s_left  = myignore_speed(s_left);
  s_right = myignore_speed(s_right);

  setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, s_left);
  setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, s_right);
}

/**
 * @brief 正常停止小车 (断电滑行)
 */
void StopCar(void) {
  analogWrite(Motor_L_PWM, 0);
  analogWrite(Motor_R_PWM, 0);
  digitalWrite(Motor_L_IN1, LOW);
  digitalWrite(Motor_L_IN2, LOW);
  digitalWrite(Motor_R_IN1, LOW);
  digitalWrite(Motor_R_IN2, LOW);
}

/**
 * @brief 强效反接电子刹车 (终点线专用)
 */
void StopCarBrake(void) {
  // 反向通电微小脉冲，瞬间抵消前进惯性
  digitalWrite(Motor_L_IN1, LOW);
  digitalWrite(Motor_L_IN2, HIGH);
  analogWrite(Motor_L_PWM, BRAKE_SPEED);
  digitalWrite(Motor_R_IN1, LOW);
  digitalWrite(Motor_R_IN2, HIGH);
  analogWrite(Motor_R_PWM, BRAKE_SPEED);
  delay(BRAKE_MS);

  // 彻底切断驱动输出
  StopCar();
}
