#include "avoid_obstacle.hpp"

static Servo myservo;
static HCSR04 ultrasonic(TrigPin, EchoPin);

static unsigned long last_detect_time = 0;
static unsigned long cooldown_until = 0;
static uint8_t confirm_cnt = 0;

void Avoid_init(void) {
  myservo.attach(ServoPin);
  delay(100);
  myservo.write(SERVO_CENTER_DEG); // 舵机正中
  delay(200);
  cooldown_until = millis() + 800; // 开机前 800ms 屏蔽避障
}

/**
 * @brief 检测前方障碍物 (50ms 定时采样 + 连续 2 次确认防误报)
 */
bool Avoid_detect(void) {
  unsigned long now = millis();
  
  if (now < cooldown_until) return false;
  if (now - last_detect_time < 50) return false;
  last_detect_time = now;

  float d = ultrasonic.dist(); // 测距 (单位: cm)

  if (d > 2.0f && d <= AVOID_TRIGGER_CM) {
    confirm_cnt++;
    if (confirm_cnt >= 2) { // 连续两次测得前方有障碍
      confirm_cnt = 0;
      return true;
    }
  } else {
    confirm_cnt = 0;
  }
  return false;
}

/**
 * @brief 完整避障逻辑：
 *        1. 刹停并快速左右探照 (耗时约 300ms)
 *        2. 择优斜向偏出
 *        3. 直行越过障碍物长度
 *        4. 斜向切回并开启红外闭环寻线 (见线即止，绝不盲开脱轨)
 */
void Avoid_perform(void) {
  StopCar();
  delay(50);

  // 1. 快速探照左侧
  myservo.write(SERVO_LEFT_DEG);
  delay(130);
  float dist_left = ultrasonic.dist();
  if (dist_left <= 0) dist_left = 999.0f;

  // 2. 快速探照右侧
  myservo.write(SERVO_RIGHT_DEG);
  delay(150);
  float dist_right = ultrasonic.dist();
  if (dist_right <= 0) dist_right = 999.0f;

  // 3. 舵机回正
  myservo.write(SERVO_CENTER_DEG);

  // 比较左右净空：哪边开阔就往哪边绕行
  bool bypass_left = (dist_left >= dist_right);

  // 阶段 1：向开阔侧偏出
  if (bypass_left) {
    Set_motor_direct(40, 105); // 左轮慢右轮快 -> 向左绕
  } else {
    Set_motor_direct(105, 40); // 左轮快右轮慢 -> 向右绕
  }
  delay(TIME_TURN_OUT_MS);

  // 阶段 2：超越障碍物纵向长度
  Set_motor_direct(90, 90);
  delay(TIME_PASS_MS);

  // 阶段 3：斜向切回并持续红外寻线 (关键闭环设计)
  if (bypass_left) {
    Set_motor_direct(105, 45); // 向右切回黑线
  } else {
    Set_motor_direct(45, 105); // 向左切回黑线
  }

  unsigned long t0 = millis();
  while (millis() - t0 < TIME_RECOVER_MAX) {
    I2Cdata();
    // 只要中间 4 个探头中有任意一个捕捉到黑线，说明小车已成功回到黑线上
    if (x3 == 0 || x4 == 0 || x5 == 0 || x6 == 0) {
      break;
    }
    delay(10);
  }

  // 阶段 4：回线收尾并进入冷却期，避免对同一个障碍物重复触发
  StopCar();
  delay(40);
  cooldown_until = millis() + AVOID_COOLDOWN_MS;
}
