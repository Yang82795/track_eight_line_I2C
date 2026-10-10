#include "Arduino.h"
#include "avoid_obstacle.hpp"

/* ===========================================================================
 * 双超声波避障（左 D2/D3，右 D11/D12）
 *
 * 绕行形状：标准的 S 形
 *     ① 绕出：向外转约 45°
 *     ② 斜行：双轮等速，沿 45° 方向走一段
 *     ③ 转回：反向转回，车头重新与黑线平行
 *     ④ 找线：红外闭环把车压回线中
 *
 * 为什么用 S 形而不是"转 90° 再回来"：
 *   ① 比赛障碍是【放在黑线旁边】的，转 45° 就够，没必要大角度硬绕；
 *   ② S 形结束时车头与线平行，第 ④ 步找线最容易成功
 *      （旧方案结束时车头几乎垂直于线，找线要先转 90° 再找，难度和失败率都高得多）。
 * ======================================================================== */

// ---- 判定状态 ----
static uint16_t g_hitL, g_hitR;      // 连续命中次数
static uint16_t g_missL, g_missR;    // 连续未命中次数（迟滞用）
static bool     g_blockL, g_blockR;  // 是否判定为"有障碍"

// ---- 最近一次测距（调试用）----
static float         g_cmL, g_cmR;
static unsigned long g_usL, g_usR;

// ---- 时序 ----
static unsigned long g_last_tick, g_cooldown_until;
static ObstacleSide  g_detected_side;
static float         g_absErrAvg;    // |偏差| 平滑值，用来判断是否在弯道

unsigned long g_shield_finish_until = 0;

static const uint8_t g_mode = AVOID_SWITCH_MODE;

/* ---------------------------------------------------------------------------
 * 发一次超声波脉冲，返回 echo 脉宽(us)；0 = 没收到回波
 * ------------------------------------------------------------------------ */
static unsigned long ping_us(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);              // 触发高电平需 >=10us
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, PING_TIMEOUT_US);
}

/* ---------------------------------------------------------------------------
 * 前进型差速输出
 * 规则禁止逆行，所以两轮速度都设下限，任何极端参数下最多停转、绝不倒转。
 * ------------------------------------------------------------------------ */
static void drive_forward(int speed_l, int speed_r) {
  if (speed_l < AVOID_MIN_SPEED) speed_l = AVOID_MIN_SPEED;
  if (speed_r < AVOID_MIN_SPEED) speed_r = AVOID_MIN_SPEED;
  if (speed_l > 255) speed_l = 255;
  if (speed_r > 255) speed_r = 255;
  Set_motor_direct(speed_l, speed_r);
}

/* ---------------------------------------------------------------------------
 * 单侧迟滞判定：连续命中到阈值才算"有障碍"，
 * 连续多次读不到才算"障碍消失"，避免在临界距离上反复触发/取消。
 * ------------------------------------------------------------------------ */
static void update_side(bool is_left, float cm) {
  uint16_t &hit  = is_left ? g_hitL  : g_hitR;
  uint16_t &miss = is_left ? g_missL : g_missR;
  bool     &blk  = is_left ? g_blockL : g_blockR;

  if (cm > 2.0f && cm <= (float)AVOID_TRIGGER_CM) {
    miss = 0;
    if (!blk) {
      hit++;
      uint16_t need = (cm <= (float)AVOID_TRIGGER_CM_NEAR) ? AVOID_CONFIRM_NEAR
                                                            : AVOID_CONFIRM_TIMES;
      if (hit >= need) blk = true;
    }
  } else {
    hit = 0;
    if (blk && ++miss >= AVOID_RELEASE_TIMES) { blk = false; miss = 0; }
  }
}

/* ---------------------------------------------------------------------------
 * 决定往哪边绕：哪一侧超声波报了障碍，就往另一侧绕。
 * 比赛不会出现左右同时有障碍，因此不需要"选空间大的一侧"的优先级逻辑。
 * ------------------------------------------------------------------------ */
static ObstacleSide choose_side(void) {
  if (g_blockL) return OBS_LEFT;     // 障碍在左 -> 向右绕
  if (g_blockR) return OBS_RIGHT;    // 障碍在右 -> 向左绕
  return OBS_NONE;
}

/* ---------------------------------------------------------------------------
 * 绕出：朝远离障碍的方向转出。
 *   bypass_left = true  -> 向左偏（左轮慢、右轮快）
 *   bypass_left = false -> 向右偏（左轮快、右轮慢）
 * ------------------------------------------------------------------------ */
static void turn_out(bool bypass_left, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (bypass_left) drive_forward(AVOID_TURN_OUT_INNER, AVOID_TURN_OUT_OUTER);
    else              drive_forward(AVOID_TURN_OUT_OUTER, AVOID_TURN_OUT_INNER);
    delay(5);
  }
}

/* ---------------------------------------------------------------------------
 * 转回：turn_out 的镜像动作，用来把车头重新摆正到与黑线平行。
 * ------------------------------------------------------------------------ */
static void turn_back(bool bypass_left, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (bypass_left) drive_forward(AVOID_TURN_OUT_OUTER, AVOID_TURN_OUT_INNER);
    else              drive_forward(AVOID_TURN_OUT_INNER, AVOID_TURN_OUT_OUTER);
    delay(5);
  }
}

/* ---------------------------------------------------------------------------
 * 斜行：双轮等速直行，沿绕出时形成的角度前进，
 * 一边走一边把障碍物甩到车后方。
 * ------------------------------------------------------------------------ */
static void pass_straight(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    drive_forward(AVOID_PASS_SPEED, AVOID_PASS_SPEED);
    delay(5);
  }
}

/* ---------------------------------------------------------------------------
 * 当前是否在弯道：|偏差| 的平滑值持续偏大即为弯道。
 * ------------------------------------------------------------------------ */
static bool on_curve(void) {
  return g_absErrAvg >= AVOID_CURVE_ERR;
}

/* ---------------------------------------------------------------------------
 * 极低速度原地微动对准黑线中线 (V8.8 单向锁定版)：
 * 触线刹停后，锁定首次判断的正确转向方向，以极低速度(AVOID_ALIGN_SPEED_SLOW)
 * 持续朝该方向转动车身，直到黑线位于传感器正中央 (|err| <= AVOID_CENTER_TOLERANCE)。
 * 即使刚接线上或转动途中短暂全白 (n == 0)，也始终坚持朝首次判断的方向转，绝不掉头反转！
 * ------------------------------------------------------------------------ */
static void align_to_center(bool bypass_left, unsigned long max_ms) {
  unsigned long t0 = millis();
  // 初始默认方向：按避障绕行方向推断 (bypass_left=true -> 向右转 +1；反之向左转 -1)
  int turn_dir = bypass_left ? 1 : -1;
  bool dir_decided = false;

  while (millis() - t0 < max_ms) {
    I2Cdata();
    int n = getBlackCount();
    float e = getLineError();

    // 1. 目标达成：至少 1 路黑线且偏差绝对值 <= 容差（黑线位于中央 x4/x5 探头上）
    if (n >= 1 && fabs(e) <= AVOID_CENTER_TOLERANCE) {
      break; // 已精确对准中线，跳出！
    }

    // 2. 首次方向判定（仅在第一次判定并锁定方向，后续全程保持该方向，绝不掉头反打）
    if (!dir_decided) {
      if (n >= 1) {
        if (e > AVOID_CENTER_TOLERANCE) {
          turn_dir = 1;  // e > 0 说明黑线在右侧 -> 车头向右转拉正
        } else if (e < -AVOID_CENTER_TOLERANCE) {
          turn_dir = -1; // e < 0 说明黑线在左侧 -> 车头向左转拉正
        }
      }
      dir_decided = true;
    }

    // 3. 始终朝首次判断的方向微动转向（即使刚接线丢线 n == 0 也一直转向该方向）
    if (turn_dir == 1) {
      Set_motor_direct(AVOID_ALIGN_SPEED_SLOW, 0);
    } else {
      Set_motor_direct(0, AVOID_ALIGN_SPEED_SLOW);
    }

    delay(10);
  }
}

/* ===========================================================================
 * 公开接口
 * ======================================================================== */

/* ---------------------------------------------------------------------------
 * 初始化超声波引脚与避障状态
 * ------------------------------------------------------------------------ */
void Avoid_init(void) {
  if (g_mode == AVOID_MODE_ALL_ON || g_mode == AVOID_MODE_LEFT_ONLY) {
    pinMode(TRIG_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    digitalWrite(TRIG_PIN_LEFT, LOW);
  }
  if (g_mode == AVOID_MODE_ALL_ON || g_mode == AVOID_MODE_RIGHT_ONLY) {
    pinMode(TRIG_PIN_RIGHT, OUTPUT);
    pinMode(ECHO_PIN_RIGHT, INPUT);
    digitalWrite(TRIG_PIN_RIGHT, LOW);
  }
  Avoid_reset();
  g_cooldown_until = millis() + 800;   // 上电 800ms 内不避障，等电压稳定
}

/* ---------------------------------------------------------------------------
 * 清空全部避障状态与冷却计时
 * ------------------------------------------------------------------------ */
void Avoid_reset(void) {
  g_hitL = g_hitR = 0;
  g_missL = g_missR = 0;
  g_blockL = g_blockR = false;
  g_cmL = g_cmR = 999.0f;
  g_usL = g_usR = 0;
  g_last_tick = 0;
  g_detected_side = OBS_NONE;
  g_absErrAvg = 0.0f;
  g_shield_finish_until = 0;
}

/* ---------------------------------------------------------------------------
 * 周期性测距并判定有无障碍；返回侧别，无障碍返回 OBS_NONE
 * 内部按 AVOID_TICK_MS 节流，不会拖慢循迹主循环
 * ------------------------------------------------------------------------ */
ObstacleSide Avoid_detect(void) {
  if (g_mode == AVOID_MODE_OFF) return OBS_NONE;

  unsigned long now = millis();
  if (now - g_last_tick < AVOID_TICK_MS) return OBS_NONE;
  g_last_tick = now;

  // |偏差| 平滑值：无论是否处于冷却都要跟踪，否则避障刚做完时
  // on_curve() 拿到的是避障之前的旧值。
  g_absErrAvg = 0.7f * g_absErrAvg + 0.3f * fabs(getLineError());

  if ((long)(now - g_cooldown_until) < 0) return OBS_NONE;   // 冷却中，不做判定

  if (g_mode == AVOID_MODE_ALL_ON || g_mode == AVOID_MODE_LEFT_ONLY) {
    g_usL = ping_us(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    g_cmL = g_usL ? ((float)g_usL / 58.2f) : 999.0f;
    update_side(true, g_cmL);
  } else { g_cmL = 999.0f; g_blockL = false; }

  if (g_mode == AVOID_MODE_ALL_ON || g_mode == AVOID_MODE_RIGHT_ONLY) {
    g_usR = ping_us(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    g_cmR = g_usR ? ((float)g_usR / 58.2f) : 999.0f;
    update_side(false, g_cmR);
  } else { g_cmR = 999.0f; g_blockR = false; }

  ObstacleSide side = choose_side();
  if (side != OBS_NONE) { g_hitL = g_hitR = 0; g_missL = g_missR = 0; }
  g_detected_side = side;
  return side;
}

/* ---------------------------------------------------------------------------
 * 执行完整绕行动作：刹停 -> 绕出 -> 斜行 -> 转回 -> 找线 -> 交还 PID
 * 弯道上自动缩短绕行动作，交给 PID 收尾
 * ------------------------------------------------------------------------ */
void Avoid_perform(ObstacleSide side) {
  StopCar();
  delay(AVOID_STOP_MS);

  if (side == OBS_NONE) side = g_detected_side;

  // bypass_left = true 表示"向左绕"（用于躲开右侧障碍）
  bool bypass_left = (side == OBS_RIGHT);

  unsigned long scale = on_curve() ? AVOID_CURVE_SCALE : 100;
  unsigned long t_out = AVOID_TURN_OUT_MS * scale / 100;
  unsigned long t_pass = AVOID_PASS_MS     * scale / 100;
  unsigned long t_back = t_out * AVOID_TURN_BACK_SCALE / 100;

  // ① 绕出 ~45°
  turn_out(bypass_left, t_out);
  StopCar(); delay(AVOID_PAUSE_MS);

  // ② 斜向前进
  pass_straight(t_pass);
  StopCar(); delay(AVOID_PAUSE_MS);

  // ③ 转回朝向黑线 (约 7 成回转打角，平缓切回)
  turn_back(bypass_left, t_back);
  StopCar(); delay(AVOID_PAUSE_MS);

  // ④ 寻线切向黑线：只要有任意 1 路探头碰线，立即跳出！
  unsigned long t0 = millis();
  while (millis() - t0 < AVOID_REACQ_MAX_MS) {
    I2Cdata();
    if (getBlackCount() >= 1) {
      break; // 只要哪怕 1 路碰线，立即跳出！
    }
    int d = bypass_left ? AVOID_SEARCH_DIFF_LO : -AVOID_SEARCH_DIFF_LO;
    drive_forward(AVOID_APPROACH_SPEED + d, AVOID_APPROACH_SPEED - d);
    delay(5);
  }

  // ⑤ 触线刹车（可调参数，默认 300ms，设为 0 则不刹车直接对准）
#if (AVOID_TOUCH_BRAKE_MS > 0)
  StopCar();
  delay(AVOID_TOUCH_BRAKE_MS);
#endif

  // ⑥ 极低速度原地微动转向，直到黑线位于正中央 (|err| <= 1.0)！
  align_to_center(bypass_left, AVOID_ALIGN_TIMEOUT_MS);
  StopCar();
  delay(30);

  // ⑦ ★★★ 核心修复 1：彻底清空全部避障历史与迟滞状态，杜绝 1.2 秒后再次误触发绕障！★★★
  Avoid_reset(); // 清空 g_blockL/R、g_hit、g_miss、g_absErrAvg 等，保证状态 100% 干净干净

  // ⑧ 重新采集当前正中央探头最新读数并预热 PID
  I2Cdata();
  PID_count_x(); // 预热 PID 令 prev_err = err，消除接管微分突变

  // ⑨ ★★★ 核心修复 2：回线闭环平滑接管过渡 (默认 250ms) ★★★
  //    以温和巡线速度 (PWM 32) 提前运行闭环循迹，彻底咬住黑线后再交还主循环全速奔跑
#if (AVOID_HANDOVER_MS > 0)
  unsigned long t_handover = millis();
  while (millis() - t_handover < AVOID_HANDOVER_MS) {
    I2Cdata();
    int steer = PID_count_x();
    Set_speed(AVOID_HANDOVER_SPEED, steer);
    delay(10);
  }
#endif

  // ⑩ 启动避障后冷却期与终点线误判屏蔽期
  g_cooldown_until      = millis() + AVOID_COOLDOWN_MS;
  g_shield_finish_until = millis() + AVOID_FINISH_SHIELD_MS;
}

/* ---------------------------------------------------------------------------
 * 调试读数接口
 * ------------------------------------------------------------------------ */
float         Avoid_getLeftCm(void)     { return g_cmL; }
float         Avoid_getRightCm(void)    { return g_cmR; }
unsigned long Avoid_getLeftUs(void)     { return g_usL; }
unsigned long Avoid_getRightUs(void)    { return g_usR; }
bool          Avoid_leftBlocked(void)   { return g_blockL; }
bool          Avoid_rightBlocked(void)  { return g_blockR; }