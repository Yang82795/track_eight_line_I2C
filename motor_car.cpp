#include "Arduino.h"
#include "motor_car.hpp"


#define  Ignore_speed (60) //电机最小起转
#define  MAX_Speed (255) //最大速度设置


static int speed_L = 0;
static int speed_R = 0;

// 创建Adafruit_PWMServoDriver类的实例 Create an instance of the Adafruit_PWMServoDriver class
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(Bottom_Layer_Driver_ADDR);

void Motor_init() {
  Wire.begin();
  pinMode(Motor_L_PWM,OUTPUT);
  pinMode(Motor_L_IN1,OUTPUT);
  pinMode(Motor_L_IN2,OUTPUT);
  pinMode(Motor_R_PWM,OUTPUT);
  pinMode(Motor_R_IN1,OUTPUT);
  pinMode(Motor_R_IN2,OUTPUT);
  delay(100);                    // 如果小车功能异常，可以增加这个延时 If the function is abnormal, you can increase the delay
  digitalWrite(Motor_L_IN1, LOW); // 设置小车停止状态 Set the car to stop state
  digitalWrite(Motor_L_IN2, LOW);
  digitalWrite(Motor_R_IN1, LOW);
  digitalWrite(Motor_R_IN2, LOW);
}


/**
 * @brief 设置单个电机速度 Setting the Motor Speed
 * @param motor_forward_pin: 控制电机前进引脚 Control the motor forward pin
 * @param motor_backward_pin: 控制电机后退引脚 Control the motor backward pin
 * @param motor_speed: 设置电机速度 Setting the Motor Speed
 * @retval 无 None
 */
void setMotorSpeed(uint16_t motor_pin1, uint16_t motor_pin2,uint16_t motor_pin_pwm,int motor_speed) {
  if (motor_speed >= 0) {
    digitalWrite(motor_pin1,HIGH);
    digitalWrite(motor_pin2,LOW);
    analogWrite(motor_pin_pwm,motor_speed);
  } else if (motor_speed < 0) {
    digitalWrite(motor_pin1,LOW);
    digitalWrite(motor_pin2,HIGH);
    analogWrite(motor_pin_pwm,(-motor_speed));
  }
}


int myignore_speed(int speed)
{
   if(speed == 0)
    return 0;

    if(speed < 0)
    {
        if(speed > -Ignore_speed)
        {
            speed = -Ignore_speed;
        }
    }
    else if (speed < Ignore_speed)
    {
        speed = Ignore_speed;
    }

    return speed;

}


int limin_speed(int speed,int max,int min)
{
    if(speed > max)
    {
        return max;
    }
    else if(speed<min)
    {
        return min;
    }
    return speed;

}


//speed_fb y轴  speed_lr ：x轴
void Set_speed(int speed_fb,int speed_lr)
{
    speed_L = speed_fb + speed_lr ;
    speed_R = speed_fb - speed_lr ;

    //满足速度范围
    speed_L = limin_speed(speed_L,MAX_Speed,-MAX_Speed);
    speed_R = limin_speed(speed_R,MAX_Speed,-MAX_Speed);

    //去速度死区
    speed_L = myignore_speed(speed_L);
    speed_R = myignore_speed(speed_R);

    //直接输出pwm
    setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, speed_L);
    setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, speed_R);
}
