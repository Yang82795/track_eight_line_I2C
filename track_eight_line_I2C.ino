#include <Wire.h>
#include <HCSR04.h>
#include <Servo.h>
#include "motor_car.hpp"
#include "line_pid.hpp"

/*
 * 重走古丝路赛道：6200 mm x 2000 mm，黑色循迹线宽约 20 mm。
 * 障碍物在每个赛段随机位于循迹线一侧，因此避障不能写死向左/向右绕行。
 */

// ---------- 硬件连接 ----------
const uint8_t ECHO_PIN = 7;
const uint8_t TRIG_PIN = 8;
const uint8_t SERVO_PIN = 6;
const uint8_t KEY_PIN = 2;
const uint8_t LINE_SENSOR_ADDRESS = 0x12;
const uint8_t LINE_SENSOR_REGISTER = 0x30;

// ---------- 现场标定项 ----------
// 95 为超声波正前方。请通电观察后确认 145 真的是车体左侧、45 真的是车体右侧。
const uint8_t SERVO_CENTER_ANGLE = 95;
const uint8_t SERVO_LEFT_ANGLE = 145;
const uint8_t SERVO_RIGHT_ANGLE = 45;

// 该距离从超声波模块正面量起。28~35 cm 通常能兼顾提前量和赛段标识误触发。
const int OBSTACLE_DETECT_CM = 32;
const int OBSTACLE_EMERGENCY_CM = 18;
const unsigned long ULTRASONIC_INTERVAL_MS = 55;
const unsigned long SERVO_SETTLE_MS = 280;

// 以下时间必须在实车上标定。先用 100~120 的 PWM 在胶带上调好，再逐步缩短以提速。
const int AVOID_TURN_PWM = 145;
const int AVOID_DRIVE_PWM = 115;
const int AVOID_REACQUIRE_PWM = 82;
const int AVOID_REACQUIRE_CORRECTION = 32;
const unsigned long AVOID_TURN_MS = 185;             // 约 45 度
const unsigned long AVOID_OUTWARD_MS = 700;          // 斜向离开黑线，取得横向安全间隔
const unsigned long AVOID_PASS_MS = 1350;            // 覆盖最大 150 mm 障碍物 + 车长 + 余量
const unsigned long AVOID_INWARD_MIN_MS = 260;
const unsigned long AVOID_INWARD_TIMEOUT_MS = 1300;
const unsigned long OBSTACLE_RETRIGGER_LOCKOUT_MS = 1800;

// 起点横线也会使 8 路模块几乎全黑，运行一段时间后才将宽横线视为终点。
const bool FINISH_STOP_ENABLED = true;
const unsigned long FINISH_ARM_AFTER_MS = 8000;
const unsigned long FINISH_CONFIRM_MS = 45;

// 调试时设为 1；正式比赛应保持 0，避免串口输出拖慢循迹控制周期。
#define SERIAL_DEBUG 0

Servo scanServo;
HCSR04 ultrasonic(TRIG_PIN, ECHO_PIN);

int distanceLeftCm = 0;
int distanceCenterCm = 0;
int distanceRightCm = 0;

// +1 表示从车体右侧绕行，-1 表示从车体左侧绕行。
int avoidDirection = 1;
int lastAvoidDirection = -1;

unsigned long raceStartedMs = 0;
unsigned long stateStartedMs = 0;
unsigned long lastUltrasonicMs = 0;
unsigned long lastFinishCrossMs = 0;
unsigned long obstacleLockoutUntilMs = 0;
uint8_t obstacleHits = 0;
bool lineReacquiredDuringAvoidance = false;
bool hasSeenRegularTrack = false;

enum DriveState {
  DRIVE_FOLLOW_LINE,
  DRIVE_SCAN_LEFT,
  DRIVE_SCAN_RIGHT,
  DRIVE_TURN_OUT,
  DRIVE_MOVE_OUT,
  DRIVE_SQUARE_UP,
  DRIVE_PASS_OBSTACLE,
  DRIVE_TURN_IN,
  DRIVE_MOVE_IN,
  DRIVE_ALIGN_WITH_LINE,
  DRIVE_REACQUIRE_LINE,
  DRIVE_FINISHED
};

DriveState driveState = DRIVE_FOLLOW_LINE;

static bool elapsedSince(unsigned long started, unsigned long duration)
{
  return (unsigned long)(millis() - started) >= duration;
}

static bool timeReached(unsigned long target)
{
  return (long)(millis() - target) >= 0;
}

/* 返回厘米；0 表示本次回波无效/超量程，不能把 0 当作障碍物。 */
static int readDistanceCm(void)
{
  const double distance = ultrasonic.dist();
  if (distance < 3.0 || distance > 250.0) return 0;
  return (int)(distance + 0.5);
}

/*
 * I2C 读取 8 路模块。模块约定为：bit7=x1（最左），bit0=x8（最右），黑线=0。
 * 删除了原来的 delay(10)：10 ms 的阻塞会使急弯时控制频率降到约 100 Hz 以下。
 */
static bool readLineSensor(void)
{
  Wire.beginTransmission(LINE_SENSOR_ADDRESS);
  Wire.write(LINE_SENSOR_REGISTER);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(LINE_SENSOR_ADDRESS, (uint8_t)1);
  if (received != 1 || !Wire.available()) {
    while (Wire.available()) Wire.read();
    return false;
  }

  const uint8_t data = Wire.read();
  while (Wire.available()) Wire.read();

  x1 = (data >> 7) & 0x01;
  x2 = (data >> 6) & 0x01;
  x3 = (data >> 5) & 0x01;
  x4 = (data >> 4) & 0x01;
  x5 = (data >> 3) & 0x01;
  x6 = (data >> 2) & 0x01;
  x7 = (data >> 1) & 0x01;
  x8 = data & 0x01;
  return true;
}

static void setDriveState(DriveState nextState)
{
  driveState = nextState;
  stateStartedMs = millis();

  switch (driveState) {
    case DRIVE_SCAN_LEFT:
      StopCar();
      scanServo.write(SERVO_LEFT_ANGLE);
      break;

    case DRIVE_SCAN_RIGHT:
      StopCar();
      scanServo.write(SERVO_RIGHT_ANGLE);
      break;

    case DRIVE_TURN_OUT:
    case DRIVE_MOVE_OUT:
    case DRIVE_SQUARE_UP:
    case DRIVE_PASS_OBSTACLE:
    case DRIVE_TURN_IN:
    case DRIVE_MOVE_IN:
    case DRIVE_ALIGN_WITH_LINE:
    case DRIVE_REACQUIRE_LINE:
      scanServo.write(SERVO_CENTER_ANGLE);
      break;

    case DRIVE_FOLLOW_LINE:
      scanServo.write(SERVO_CENTER_ANGLE);
      ResetLineController();
      break;

    case DRIVE_FINISHED:
      StopCar();
      break;
  }
}

/* 0 回波一般代表无遮挡；把它按远距离处理，选择两侧中空间更大的一边。 */
static int clearanceForDecision(int distanceCm)
{
  return (distanceCm == 0) ? 250 : distanceCm;
}

static void chooseAvoidDirection(void)
{
  const int leftClearance = clearanceForDecision(distanceLeftCm);
  const int rightClearance = clearanceForDecision(distanceRightCm);

  if (abs(rightClearance - leftClearance) <= 5) {
    // 正前方障碍或两边都开阔时，交替绕行，避免每次都把小车推向赛道同一边界。
    avoidDirection = -lastAvoidDirection;
  } else {
    avoidDirection = (rightClearance > leftClearance) ? 1 : -1;
  }
  lastAvoidDirection = avoidDirection;
}

static void beginObstacleAvoidance(void)
{
  obstacleHits = 0;
  lineReacquiredDuringAvoidance = false;
  setDriveState(DRIVE_SCAN_LEFT);
}

/* 仅在正常循迹状态采样，连续两次看到近距离物体才进入避障，降低赛道标识误判。 */
static void watchForObstacle(void)
{
  if (!elapsedSince(lastUltrasonicMs, ULTRASONIC_INTERVAL_MS)) return;
  lastUltrasonicMs = millis();
  distanceCenterCm = readDistanceCm();

  if (distanceCenterCm != 0 && distanceCenterCm <= OBSTACLE_DETECT_CM) {
    if (obstacleHits < 255) ++obstacleHits;
    if (distanceCenterCm <= OBSTACLE_EMERGENCY_CM || obstacleHits >= 2) {
      beginObstacleAvoidance();
    }
  } else {
    obstacleHits = 0;
  }
}

static void checkForFinish(void)
{
  // 既已沿普通循迹线行驶过、又超过最短赛程时间，才允许横线触发终点停车。
  if (!FINISH_STOP_ENABLED || !hasSeenRegularTrack ||
      !elapsedSince(raceStartedMs, FINISH_ARM_AFTER_MS)) return;

  if (IsLineCrossing()) {
    if (lastFinishCrossMs == 0) lastFinishCrossMs = millis();
    if (elapsedSince(lastFinishCrossMs, FINISH_CONFIRM_MS)) {
      setDriveState(DRIVE_FINISHED);
    }
  } else {
    lastFinishCrossMs = 0;
  }
}

static void updateDriveState(void)
{
  switch (driveState) {
    case DRIVE_FOLLOW_LINE:
      Car_line_track();
      if (IsLineDetected() && !IsLineCrossing()) hasSeenRegularTrack = true;
      checkForFinish();
      if (driveState == DRIVE_FOLLOW_LINE && timeReached(obstacleLockoutUntilMs)) {
        watchForObstacle();
      }
      break;

    case DRIVE_SCAN_LEFT:
      StopCar();
      if (elapsedSince(stateStartedMs, SERVO_SETTLE_MS)) {
        distanceLeftCm = readDistanceCm();
        setDriveState(DRIVE_SCAN_RIGHT);
      }
      break;

    case DRIVE_SCAN_RIGHT:
      StopCar();
      if (elapsedSince(stateStartedMs, SERVO_SETTLE_MS)) {
        distanceRightCm = readDistanceCm();
        chooseAvoidDirection();
#if SERIAL_DEBUG
        Serial.print(F("avoid L/R: "));
        Serial.print(distanceLeftCm);
        Serial.print('/');
        Serial.print(distanceRightCm);
        Serial.print(F(" dir: "));
        Serial.println(avoidDirection);
#endif
        setDriveState(DRIVE_TURN_OUT);
      }
      break;

    case DRIVE_TURN_OUT:
      Set_speed(0, avoidDirection * AVOID_TURN_PWM);
      if (elapsedSince(stateStartedMs, AVOID_TURN_MS)) setDriveState(DRIVE_MOVE_OUT);
      break;

    case DRIVE_MOVE_OUT:
      Set_speed(AVOID_DRIVE_PWM, 0);
      if (elapsedSince(stateStartedMs, AVOID_OUTWARD_MS)) setDriveState(DRIVE_SQUARE_UP);
      break;

    case DRIVE_SQUARE_UP:
      Set_speed(0, -avoidDirection * AVOID_TURN_PWM);
      if (elapsedSince(stateStartedMs, AVOID_TURN_MS)) setDriveState(DRIVE_PASS_OBSTACLE);
      break;

    case DRIVE_PASS_OBSTACLE:
      Set_speed(AVOID_DRIVE_PWM, 0);
      if (elapsedSince(stateStartedMs, AVOID_PASS_MS)) setDriveState(DRIVE_TURN_IN);
      break;

    case DRIVE_TURN_IN:
      Set_speed(0, -avoidDirection * AVOID_TURN_PWM);
      if (elapsedSince(stateStartedMs, AVOID_TURN_MS)) setDriveState(DRIVE_MOVE_IN);
      break;

    case DRIVE_MOVE_IN:
      Set_speed(AVOID_DRIVE_PWM, 0);
      // 回到黑线后不再按固定时间继续横穿，防止在小障碍物处越过赛道。
      if (elapsedSince(stateStartedMs, AVOID_INWARD_MIN_MS) && IsLineDetected()) {
        lineReacquiredDuringAvoidance = true;
        setDriveState(DRIVE_ALIGN_WITH_LINE);
      } else if (elapsedSince(stateStartedMs, AVOID_OUTWARD_MS)) {
        setDriveState(DRIVE_ALIGN_WITH_LINE);
      }
      break;

    case DRIVE_ALIGN_WITH_LINE:
      Set_speed(0, avoidDirection * AVOID_TURN_PWM);
      if (elapsedSince(stateStartedMs, AVOID_TURN_MS)) {
        if (lineReacquiredDuringAvoidance) {
          obstacleLockoutUntilMs = millis() + OBSTACLE_RETRIGGER_LOCKOUT_MS;
          setDriveState(DRIVE_FOLLOW_LINE);
        } else {
          setDriveState(DRIVE_REACQUIRE_LINE);
        }
      }
      break;

    case DRIVE_REACQUIRE_LINE:
      // 轻微向黑线方向弯，黑线被重新检测到后立即交回 PID，避免盲目横穿赛道。
      Set_speed(AVOID_REACQUIRE_PWM, -avoidDirection * AVOID_REACQUIRE_CORRECTION);
      if (elapsedSince(stateStartedMs, AVOID_INWARD_MIN_MS) && IsLineDetected()) {
        obstacleLockoutUntilMs = millis() + OBSTACLE_RETRIGGER_LOCKOUT_MS;
        setDriveState(DRIVE_FOLLOW_LINE);
      } else if (elapsedSince(stateStartedMs, AVOID_INWARD_TIMEOUT_MS)) {
        // 赛道弯曲很大时，交由“失线向最后方向搜索”逻辑兜底，不能一直驶向场外。
        obstacleLockoutUntilMs = millis() + OBSTACLE_RETRIGGER_LOCKOUT_MS;
        setDriveState(DRIVE_FOLLOW_LINE);
      }
      break;

    case DRIVE_FINISHED:
      StopCar();
      break;
  }
}

void setup()
{
#if SERIAL_DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  pinMode(KEY_PIN, INPUT_PULLUP);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER_ANGLE);
  Motor_init();
  init_x_PID();
  delay(250); // 仅上电稳定时等待；主循环及避障状态机均不使用阻塞 delay。

  // 如需按键起跑，可取消下一行注释；比赛时只需启动一次，之后全程不人工干预。
  // while (digitalRead(KEY_PIN) == HIGH) { StopCar(); }

  raceStartedMs = millis();
  stateStartedMs = raceStartedMs;
}

void loop()
{
  // 读失败时立即停车，防止 I2C 线松动后继续以旧数据高速冲出赛道。
  if (!readLineSensor()) {
    StopCar();
    return;
  }

  // 避障阶段也要刷新当前黑线状态，回归黑线时才有依据。
  if (driveState != DRIVE_FOLLOW_LINE) {
    LineWalking();
  }

  updateDriveState();

#if SERIAL_DEBUG
  static unsigned long lastDebugMs = 0;
  if (elapsedSince(lastDebugMs, 250)) {
    lastDebugMs = millis();
    Serial.print(F("black="));
    Serial.print(GetLineBlackCount());
    Serial.print(F(" front="));
    Serial.print(distanceCenterCm);
    Serial.print(F(" state="));
    Serial.println((int)driveState);
  }
#endif
}
