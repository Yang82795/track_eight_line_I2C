#include "Arduino.h"
#include "motor_car.hpp"


#define  Ignore_speed (60) //电机最小起转
#define  MAX_Speed (255) //最大速度设置
#define  Brake_speed (110) //电子刹车反向PWM


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
void Set_speed_ex(int speed_fb,int speed_lr,uint8_t deadzone_en)
{
    speed_L = speed_fb + speed_lr ;
    speed_R = speed_fb - speed_lr ;

    //满足速度范围
    speed_L = limin_speed(speed_L,MAX_Speed,-MAX_Speed);
    speed_R = limin_speed(speed_R,MAX_Speed,-MAX_Speed);

    //去速度死区
    if(deadzone_en)
    {
        speed_L = myignore_speed(speed_L);
        speed_R = myignore_speed(speed_R);
    }

    //直接输出pwm
    setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, speed_L);
    setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, speed_R);
}


void Set_speed(int speed_fb,int speed_lr)
{
    Set_speed_ex(speed_fb,speed_lr,1);
}


/**
 * @brief 直接给左右轮带符号速度
 * @param speed_l/speed_r: 左/右轮速度, 正=前进, 负=后退
 * @param deadzone_en: 1=低速时抬到电机起转速度(巡线直行用) 0=原样输出(原地转/内轮停用)
 */
void Car_drive(int speed_l, int speed_r, uint8_t deadzone_en)
{
    speed_l = limin_speed(speed_l, MAX_Speed, -MAX_Speed);
    speed_r = limin_speed(speed_r, MAX_Speed, -MAX_Speed);

    if(deadzone_en)
    {
        speed_l = myignore_speed(speed_l);
        speed_r = myignore_speed(speed_r);
    }

    setMotorSpeed(Motor_L_IN1, Motor_L_IN2, Motor_L_PWM, speed_l);
    setMotorSpeed(Motor_R_IN1, Motor_R_IN2, Motor_R_PWM, speed_r);
}


/**
 * @brief 立即停车(两轮占空比清零, 自由滑行)
 */
void Car_stop(void)
{
    digitalWrite(Motor_L_IN1,LOW);
    digitalWrite(Motor_L_IN2,LOW);
    analogWrite(Motor_L_PWM,0);
    digitalWrite(Motor_R_IN1,LOW);
    digitalWrite(Motor_R_IN2,LOW);
    analogWrite(Motor_R_PWM,0);
}


/**
 * @brief 电子刹车: 两轮反向通电 brake_ms 毫秒后停车(比单纯断电停得快)
 * @param brake_ms: 反向通电时间, 建议 80~120ms
 */
void Car_brake(uint16_t brake_ms)
{
    digitalWrite(Motor_L_IN1,LOW);
    digitalWrite(Motor_L_IN2,HIGH);
    analogWrite(Motor_L_PWM,Brake_speed);
    digitalWrite(Motor_R_IN1,LOW);
    digitalWrite(Motor_R_IN2,HIGH);
    analogWrite(Motor_R_PWM,Brake_speed);
    delay(brake_ms);
    Car_stop();
}


/**
 * @brief 原地转向(差速自转): dir>0 右转, dir<0 左转
 * @param pwm: 转向速度(两侧反向同速), 建议 60~90
 */
void Car_pivot(int8_t dir, uint8_t pwm)
{
    if(dir >= 0)
    {
        Car_drive((int)pwm, -(int)pwm, 0);   // 左轮正转 + 右轮反转 = 右转
    }
    else
    {
        Car_drive(-(int)pwm, (int)pwm, 0);   // 左轮反转 + 右轮正转 = 左转
    }
}
