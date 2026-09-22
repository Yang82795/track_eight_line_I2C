#include <HCSR04.h>
#include <Servo.h>
#include <stdio.h>
#include "motor_car.hpp"
#include "line_pid.hpp"

#define u8 uint8_t

#define EchoPin 7       //超声波传感器IO口
#define TrigPin 8
#define ServoPin 6
#define KEY_PIN 2
const int Press_KEY = 0;
const int Release_KEY = 1;
const int Distance_interval=60;
unsigned long LastDistTime = 0;

void I2Cdata(void);
Servo myservo;
HCSR04 ultrasonic(TrigPin,EchoPin);

int Distance_left = 0,Distance_median = 0, Distance_right = 0; 
extern uint8_t x1,x2,x3,x4,x5,x6,x7,x8;

void setup()
{
    init_x_PID();//PID初始化
    Serial.begin(115200);  // start serial for output
    Wire.begin();        // join i2c bus (address optional for master)
    myservo.attach(ServoPin);
    pinMode(KEY_PIN, INPUT_PULLUP);  // 设置按键KEY引脚上拉输入模式 Set the key(button) pin to pull-up input mode
    delay(200);
    myservo.write(95);
    delay(200);
    Motor_init();//电机初始化
    // while (getKeyState(KEY_PIN) != Press_KEY) ;//按下key1开始巡线
}



void loop() {
//   if (millis()-LastDistTime>Distance_interval) {
//     Distance_median=ultrasonic.dist();
//     LastDistTime=millis();
//   }
//   // Serial.println(Distance_median);
//   if (Distance_median!=0 && Distance_median<=20) {
//   avoidObstacle();
//   }else {
//   I2Cdata();
//   Car_line_track();
//   }
  I2Cdata();
  Car_line_track();
  // digitalWrite(Motor_L_IN1,HIGH);
  // digitalWrite(Motor_L_IN2,LOW);
  // analogWrite(Motor_L_PWM,90);
  // digitalWrite(Motor_R_IN1,HIGH);
  // digitalWrite(Motor_R_IN2,LOW);
  // analogWrite(Motor_R_PWM,90);
}


//i2c读取函数
void I2Cdata() 
{
  byte data = 0; // 用于存储读取的数据
 char bufbuf[50]={'\0'};
  Wire.beginTransmission(0x12); // I2C设备的地址
  Wire.write(0x30); // 寄存器地址
  Wire.endTransmission();
 
  delay(10); // 给设备足够的时间来处理请求
 
  Wire.requestFrom(0x12, 1); // 请求1个字节的数据
  while (Wire.available()) {
    data = Wire.read(); // 读取数据
  }

  x1 = (data>>7)&0x01;
  x2 = (data>>6)&0x01;
  x3 = (data>>5)&0x01;
  x4 = (data>>4)&0x01;
  x5 = (data>>3)&0x01;
  x6 = (data>>2)&0x01;
  x7 = (data>>1)&0x01;
  x8 = (data>>0)&0x01;

 sprintf(bufbuf,"x1:%d,x2:%d,x3:%d,x4:%d,x5:%d,x6:%d,x7:%d,x8:%d\r\n",x1,x2,x3,x4,x5,x6,x7,x8);
  Serial.println(bufbuf);
 
 Serial.println(data, HEX); // 在串行监视器上打印数据（十六进制）
} 


void avoidObstacle(){
  StopCar();
  delay(200);
  LeftCar();
  delay(1000);
  RightCar();
  delay(2000);
}


void StopCar(){
  digitalWrite(Motor_L_IN1,LOW);
  digitalWrite(Motor_L_IN2,LOW);
  analogWrite(Motor_L_PWM,0);
  digitalWrite(Motor_R_IN1,LOW);
  digitalWrite(Motor_R_IN2,LOW);
  analogWrite(Motor_R_PWM,0);
}


void LeftCar(){
  digitalWrite(Motor_L_IN1,HIGH);
  digitalWrite(Motor_L_IN2,LOW);
  analogWrite(Motor_L_PWM,45);
  digitalWrite(Motor_R_IN1,HIGH);
  digitalWrite(Motor_R_IN2,LOW);
  analogWrite(Motor_R_PWM,90);
}


void RightCar(){
   digitalWrite(Motor_L_IN1,HIGH);
  digitalWrite(Motor_L_IN2,LOW);
  analogWrite(Motor_L_PWM,90);
  digitalWrite(Motor_R_IN1,HIGH);
  digitalWrite(Motor_R_IN2,LOW);
  analogWrite(Motor_R_PWM,45);
}

/**
 * @brief 获取按键状态 Get key(button) status
 * @param pin: 按键控制引脚 Control key(button) pins
 * @retval 按键状态 Key(button) Status
 */
int getKeyState(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      return Press_KEY;
    }
    return Release_KEY;
  } else {
    return Release_KEY;
  }
}
