#include "motor_car.hpp"

#define IGNORE_SPEED 60      // 电机克服静摩擦的最小有效 PWM
#define MAX_SPEED 255

/*
 * 若直行时小车持续向某一侧偏，可在此处微调（例如 0.96f）。
 * 不要靠很大的 PID 积分去补偿两电机的机械差异。
 */
#define MOTOR_LEFT_SCALE  1.00f
#define MOTOR_RIGHT_SCALE 1.00f

static int applyScale(int speed, float scale)
{
  return (int)((float)speed * scale);
}

void Motor_init(void)
{
  pinMode(Motor_L_PWM, OUTPUT);
  pinMode(Motor_L_IN1, OUTPUT);
  pinMode(Motor_L_IN2, OUTPUT);
  pinMode(Motor_R_PWM, OUTPUT);
  pinMode(Motor_R_IN1, OUTPUT);
  pinMode(Motor_R_IN2, OUTPUT);
  StopCar();
}

void StopCar(void)
{
  analogWrite(Motor_L_PWM, 0);
  analogWrite(Motor_R_PWM, 0);
  digitalWrite(Motor_L_IN1, LOW);
  digitalWrite(Motor_L_IN2, LOW);
  digitalWrite(Motor_R_IN1, LOW);
  digitalWrite(Motor_R_IN2, LOW);
}

void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2,
                   uint16_t motor_pin_pwm, int motor_speed)
{
  motor_speed = limin_speed(motor_speed, MAX_SPEED, -MAX_SPEED);

  if (motor_speed == 0) {
    analogWrite(motor_pin_pwm, 0);
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, LOW);
  } else if (motor_speed > 0) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
    analogWrite(motor_pin_pwm, motor_speed);
  } else {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, HIGH);
    analogWrite(motor_pin_pwm, -motor_speed);
  }
}

int myignore_speed(int speed)
{
  if (speed == 0) return 0;

  if (speed > 0 && speed < IGNORE_SPEED) return IGNORE_SPEED;
  if (speed < 0 && speed > -IGNORE_SPEED) return -IGNORE_SPEED;
  return speed;
}

int limin_speed(int speed, int max, int min)
{
  if (speed > max) return max;
  if (speed < min) return min;
  return speed;
}

void Set_speed(int speed_fb, int speed_lr)
{
  int speedLeft = speed_fb + speed_lr;
  int speedRight = speed_fb - speed_lr;

  speedLeft = limin_speed(speedLeft, MAX_SPEED, -MAX_SPEED);
  speedRight = limin_speed(speedRight, MAX_SPEED, -MAX_SPEED);

  speedLeft = myignore_speed(speedLeft);
  speedRight = myignore_speed(speedRight);

  speedLeft = limin_speed(applyScale(speedLeft, MOTOR_LEFT_SCALE), MAX_SPEED, -MAX_SPEED);
  speedRight = limin_speed(applyScale(speedRight, MOTOR_RIGHT_SCALE), MAX_SPEED, -MAX_SPEED);

  setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, speedLeft);
  setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, speedRight);
}
