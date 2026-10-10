#include <Wire.h>
#include "motor_car.hpp"
#include "line_pid.hpp"
#include "avoid_obstacle.hpp"

// ==========================================
// 起跑与比赛配置
// ==========================================
#define KEY_PIN           4        // 板载启动按键引脚
const int Press_KEY   = 0;
const int Release_KEY = 1;

// 终点检测起跑屏蔽时间 (单位: ms)
// 起跑前 2 秒屏蔽终点横线识别，确保离开起点 500mm 横线前不误刹停
#define FINISH_SHIELD_MS  2000

// 串口调试输出开关 (1: 开启调车调试信息输出；正式比赛建议设为 0)
#define DEBUG_SERIAL      0

// ==========================================
// 运行状态与赛道中掉电自恢复机制
// ==========================================
enum CarState {
  STATE_WAIT_START, // 等待发车
  STATE_RUNNING,    // 正常循迹行驶中
  STATE_FINISHED    // 已到达终点并刹停
};

// 利用 GCC .noinit 段保存比赛运行魔数：
// 1. 关机冷启动 (彻底断电数秒)：SRAM 掉电清空，魔数失效 -> 等待按键发车！
// 2. 赛道中掉电复位 (微秒级瞬态电压跌落)：SRAM 依然维持，魔数有效 -> 立即免按键自动继续发车跑！
#define RUN_MAGIC_TOKEN 0x5A3C96E1UL
__attribute__((section(".noinit"))) static unsigned long g_in_race_token;

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

  // ★★★ 智能识别：冷启动 vs 赛道中掉电复位 ★★★
  if (g_in_race_token == RUN_MAGIC_TOKEN) {
    // 判定为赛道中瞬态掉电复位：无需等待人工按键，立即自动恢复巡线！
    g_start_time = millis() - FINISH_SHIELD_MS; // 视同起跑屏蔽期已过
    g_state = STATE_RUNNING;
    Serial.println(F(">>> [BOR RECOVERY] In-race reset detected! Resuming line tracking immediately!"));
  } else {
    // 判定为起点首次开机冷启动：必须按键发车
    g_in_race_token = 0;
    g_state = STATE_WAIT_START;
    Serial.println(F("=== System Ready: Align car at START line, then press KEY ==="));
  }
}

void loop() {
  switch (g_state) {
    // ----------------------------------------------------
    // 状态 1：等待起跑按键 (起点线冷启动)
    // ----------------------------------------------------
    case STATE_WAIT_START: {
      if (getKeyState(KEY_PIN) == Press_KEY) {
        delay(200); // 按键防抖与等手离开
        g_in_race_token = RUN_MAGIC_TOKEN; // ★ 登记比赛进行中状态，激活掉电保护
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
      // 现场手动急停/复位按键：行驶中按下按键可随时安全刹停并恢复为等待状态
      if (getKeyState(KEY_PIN) == Press_KEY) {
        StopCar();
        g_in_race_token = 0;
        g_state = STATE_WAIT_START;
        delay(500);
        Serial.println(F(">>> STOPPED by KEY. Waiting for next start..."));
        return;
      }

      // 1. 刷新 8 路红外数据
      I2Cdata();

      // 2. 终点横线判断：
      // 条件一：起跑屏蔽期已过 (FINISH_SHIELD_MS = 2000ms)
      // 条件二：避障回线后的安全期已过 (millis() > g_shield_finish_until，避障后 5 秒内绝不误判终点！)
      // 条件三：连续 2 次确认检测到全宽横线
      if ((millis() - g_start_time) > FINISH_SHIELD_MS && millis() > g_shield_finish_until) {
        if (isCrossLine()) {
          g_finish_confirm_cnt++;
          if (g_finish_confirm_cnt >= 2) {
            // 到达终点线！执行强效反接刹车，保住 20 分停止分！
            StopCar();
            g_in_race_token = 0; // ★ 到达终点，清除比赛标记，下次开机需重新按键
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
      } else {
        g_finish_confirm_cnt = 0; // 屏蔽期内强制清零
      }

      // 3. 动态避障检测 (支持：只左 / 只右 / 全关 / 全开 4种模式，由 avoid_obstacle.hpp 总开关控制)
      ObstacleSide obs = Avoid_detect();
      if (obs != OBS_NONE) {
#if DEBUG_SERIAL
        Serial.print(F(">>> Obstacle detected on side: "));
        Serial.println((int)obs);
#endif
        Avoid_perform(obs);
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
