#include <Wire.h>
#include "motor_car.hpp"
#include "line_pid.hpp"
#include "avoid_obstacle.hpp"

// ==========================================
// 起跑与比赛配置
// ==========================================
#define KEY_PIN           2        // 板载启动按键引脚
const int Press_KEY   = 0;
const int Release_KEY = 1;

// 0: 按键启动 (对齐起点线后，按下 KEY 启动，推荐比赛采用)
// 1: 上电自动启动 (通电稳定 2 秒后自动发车)
#define AUTO_START        0

// 终点检测时间屏蔽 (单位: ms)
// 赛道全长 6.2 米，小车正常跑完全程约 20~30 秒。
// 起跑前 14 秒屏蔽终点横线识别，确保绝不会在起点处的 500mm 横线误刹停！
#define FINISH_SHIELD_MS  14000

// 串口调试输出开关 (1: 开启调车调试信息输出；正式比赛建议设为 0)
#define DEBUG_SERIAL      1

// ==========================================
// 运行状态定义
// ==========================================
enum CarState {
  STATE_WAIT_START, // 等待发车
  STATE_RUNNING,    // 正常循迹行驶中
  STATE_FINISHED    // 已到达终点并刹停
};

static CarState g_state = STATE_WAIT_START;
static unsigned long g_start_time = 0;
static unsigned long g_last_debug_time = 0;
static uint8_t g_finish_confirm_cnt = 0;

int getKeyState(uint8_t pin);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(KEY_PIN, INPUT_PULLUP);
  delay(100);

  // 各模块底层初始化
  Motor_init();
  init_x_PID();
  Avoid_init();
  StopCar();

#if AUTO_START
  Serial.println(F("=== System Ready: Auto-start in 2 seconds... ==="));
  delay(2000);
  g_start_time = millis();
  g_state = STATE_RUNNING;
  Serial.println(F(">>> GO!"));
#else
  Serial.println(F("=== System Ready: Align car at START line, then press KEY ==="));
#endif
}

void loop() {
  switch (g_state) {
    // ----------------------------------------------------
    // 状态 1：等待起跑按键
    // ----------------------------------------------------
    case STATE_WAIT_START: {
      if (getKeyState(KEY_PIN) == Press_KEY) {
        delay(200); // 按键防抖与等手离开
        g_start_time = millis();
        g_state = STATE_RUNNING;
        Serial.println(F(">>> GO!"));
      }
      break;
    }

    // ----------------------------------------------------
    // 状态 2：比赛运行中 (循迹 + 超声波避障 + 终点判断)
    // ----------------------------------------------------
    case STATE_RUNNING: {
      // 1. 刷新 8 路红外数据
      I2Cdata();

      // 2. 终点横线判断 (必须在屏蔽期过后，且连续 2 次确认检测到全宽横线)
      if ((millis() - g_start_time) > FINISH_SHIELD_MS) {
        if (isCrossLine()) {
          g_finish_confirm_cnt++;
          if (g_finish_confirm_cnt >= 2) {
            // 到达终点线！执行强效反接刹车，保住 20 分停止分！
            StopCarBrake();
            g_state = STATE_FINISHED;

            unsigned long race_duration = millis() - g_start_time;
            Serial.println(F("========================================"));
            Serial.print(F(">>> FINISH! Time: "));
            Serial.print(race_duration / 1000.0f, 2);
            Serial.println(F(" s"));
            Serial.println(F("========================================"));
            return;
          }
        } else {
          g_finish_confirm_cnt = 0;
        }
      }

      // 3. 动态避障检测
      if (Avoid_detect()) {
#if DEBUG_SERIAL
        Serial.println(F(">>> Obstacle detected! Executing bypass..."));
#endif
        Avoid_perform();
        return; // 避障刚结束，跳出当前循环稍作稳定
      }

      // 4. 正常自适应循迹
      Car_line_track();

      // 5. 调试信息打印 (每 250ms 输出一次，避免拖慢控制循环)
#if DEBUG_SERIAL
      if (millis() - g_last_debug_time > 250) {
        g_last_debug_time = millis();
        Serial.print(F("T: "));
        Serial.print((millis() - g_start_time) / 1000);
        Serial.print(F("s | Err: "));
        Serial.print(getLineError(), 1);
        Serial.print(F(" | Blk: "));
        Serial.print(getBlackCount());
        Serial.print(F(" | ["));
        Serial.print(x1); Serial.print(x2); Serial.print(x3); Serial.print(x4);
        Serial.print(x5); Serial.print(x6); Serial.print(x7); Serial.print(x8);
        Serial.println(F("]"));
      }
#endif
      break;
    }

    // ----------------------------------------------------
    // 状态 3：到达终点，保持停止
    // ----------------------------------------------------
    case STATE_FINISHED:
    default: {
      StopCar();
      break;
    }
  }
}

/**
 * @brief 按键消抖检测函数
 */
int getKeyState(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      return Press_KEY;
    }
    return Release_KEY;
  }
  return Release_KEY;
}
