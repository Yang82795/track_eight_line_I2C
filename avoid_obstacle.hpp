#ifndef __AVOID_OBSTACLE_HPP_
#define __AVOID_OBSTACLE_HPP_

#include <Arduino.h>
#include "motor_car.hpp"
#include "line_pid.hpp"

// =========================================================================
//  0. 避障总开关
// =========================================================================
#define AVOID_MODE_OFF        0   // 全关：纯循迹，0 耗时
#define AVOID_MODE_LEFT_ONLY  1   // 只左：单独验证左超声波
#define AVOID_MODE_RIGHT_ONLY 2   // 只右：单独验证右超声波
#define AVOID_MODE_ALL_ON     3   // 全开 ★ 比赛用这个 ★

#define AVOID_SWITCH_MODE     AVOID_MODE_ALL_ON

// =========================================================================
//  1. 硬件引脚（已按实际接线核对，本版本未改动任何引脚值）
// =========================================================================
//   左 HC-SR04 : Trig = D2 ,  Echo = D3
//   右 HC-SR04 : Trig = D11, Echo = D12
//   A4/A5 = 8 路巡线 I2C（从机 0x12，寄存器 0x30）
//   D5 D6 D7 = TB6612 左轮 PWM/IN2/IN1
//   D8 D9 D10 = TB6612 右轮 IN1/IN2/PWM
//   D4 = 启动/急停按键
#define TRIG_PIN_LEFT         2
#define ECHO_PIN_LEFT         3
#define TRIG_PIN_RIGHT        11
#define ECHO_PIN_RIGHT        12

// =========================================================================
//  2. 探测与判定
// =========================================================================
// HC-SR04: 距离(cm) ≈ 脉宽(us) / 58.2
// 2600us ≈ 44.7cm，是本模块能测到的最远距离，也决定了"无障碍时"的固定耗时
#ifndef PING_TIMEOUT_US
#define PING_TIMEOUT_US       2600   // ----
#endif
#ifndef AVOID_TRIGGER_CM
#define AVOID_TRIGGER_CM      30    // 触发距离(cm)   // ----
#endif
#define AVOID_TRIGGER_CM_NEAR 22    // 更近时需更多次确认，抑制误报
#ifndef AVOID_CONFIRM_TIMES
#define AVOID_CONFIRM_TIMES   2   // ----
#endif
#ifndef AVOID_CONFIRM_NEAR
#define AVOID_CONFIRM_NEAR    3   // ----
#endif
#ifndef AVOID_RELEASE_TIMES
#define AVOID_RELEASE_TIMES   4     // 迟滞：连续这么多次读不到才认为障碍消失   // ----
#endif
#ifndef AVOID_TICK_MS
#define AVOID_TICK_MS         55    // 超声波检测周期(ms)，同侧两次发射间隔   // ----
#endif

// =========================================================================
//  3. 绕行动作速度（PWM 0~255）
// =========================================================================
// 规则「一旦逆行本轮结束」，所以内轮由 AVOID_MIN_SPEED 兜底，任何情况都不倒转。
//
// ★ 差速大小与横移上限是死结：
//     R = (轮距/2)×(外+内)/(外−内)      而【横移上限 = R】
//     60/20(差速40) => R=130mm，横移最多 130mm，绕不开 150mm 的莫高窟
//     120/108(差速12) => R 很大但差速太小，实测角度忽大忽小、完全不可控
//     58/30(差速28) => R≈204mm，横移够用，差速又够稳   ← 当前取值
#ifndef AVOID_MIN_SPEED
#define AVOID_MIN_SPEED       12   // ----
#endif
#ifndef AVOID_TURN_OUT_OUTER
#define AVOID_TURN_OUT_OUTER  58    // 绕出：外侧轮   // ----
#endif
#ifndef AVOID_TURN_OUT_INNER
#define AVOID_TURN_OUT_INNER  30    // 绕出：内侧轮   // ----
#endif
#ifndef AVOID_PASS_SPEED
#define AVOID_PASS_SPEED      50    // 斜行段：双轮等速   // ----
#endif
#ifndef AVOID_ALIGN_SPEED
#define AVOID_ALIGN_SPEED     45    // 回线后扶正：双轮等速   // ----
#endif

// =========================================================================
//  4. 绕行动作时序
// =========================================================================
// 形状 = 标准的 S 形：转出约 45° → 沿斜向直行 → 反向转回，车头重新与线平行。
// 障碍是【放在黑线旁边】的，不需要转 90° 硬绕；结束时车头平行，
// 第 3 阶段找线就容易得多。
#ifndef AVOID_STOP_MS
#define AVOID_STOP_MS         110   // 刹停稳定   // ----
#endif
#ifndef AVOID_TURN_OUT_MS
#define AVOID_TURN_OUT_MS     420   // 绕出（≈45°，按实测标定外60/内20@500ms≈52°换算）   // ----
#endif
#ifndef AVOID_PASS_MS
#define AVOID_PASS_MS         300   // 斜行直行   // ----
#endif
#ifndef AVOID_PAUSE_MS
#define AVOID_PAUSE_MS        40    // 阶段切换缓冲   // ----
#endif
#ifndef AVOID_TURN_BACK_SCALE
#define AVOID_TURN_BACK_SCALE 60    // 回线时回转打角比例 (7成 = 70%)
#endif

// =========================================================================
//  5. 回线触线刹车与极低速中线对准拉正配置 (★ 8.6 核心调参 ★)
// =========================================================================
// 寻线逼近黑线时的巡进速度 (低速靠近，防止速度过大飞过黑线)
#ifndef AVOID_APPROACH_SPEED
#define AVOID_APPROACH_SPEED          38
#endif

// 触线立即刹车停顿时间 (单位: ms)
// - 默认 300ms：触线后立即 StopCar() 停顿 300ms 彻底消除冲刺惯性，然后极低速对准
// - 设为 0：触线不刹车停顿，直接开始极低速对准拉正
#ifndef AVOID_TOUCH_BRAKE_MS
#define AVOID_TOUCH_BRAKE_MS          300
#endif

// 极低速对准转向速度 (PWM 0~255，推荐 24~28，慢速平稳无冲击)
#ifndef AVOID_ALIGN_SPEED_SLOW
#define AVOID_ALIGN_SPEED_SLOW        26
#endif

// 对准容差：偏差绝对值 |err| <= 此值即判定黑线已在正中央 (1.0 代表黑线落在中央 x4/x5 探头上)
#ifndef AVOID_CENTER_TOLERANCE
#define AVOID_CENTER_TOLERANCE        1.0f
#endif

// 对准最大超时保护时长 (ms，防止在极端情况下无限卡住)
#ifndef AVOID_ALIGN_TIMEOUT_MS
#define AVOID_ALIGN_TIMEOUT_MS        700
#endif

#ifndef AVOID_REACQ_MAX_MS
#define AVOID_REACQ_MAX_MS            2000  // 寻线最大超时 (ms)
#endif
#ifndef AVOID_SEARCH_DIFF_LO
#define AVOID_SEARCH_DIFF_LO          12    // 寻线微差速
#endif

// =========================================================================
//  6. 弯道自适应
// =========================================================================
// 弯道本身就在帮着把车带离黑线，转太多反而容易过头；
// 急弯处回线也更难，所以弯道上把绕行动作整体缩短，交给 PID 收尾。
#ifndef AVOID_CURVE_ERR
#define AVOID_CURVE_ERR       2.5   // |偏差| 平滑值超过此值 => 判定当前在弯道   // ----
#endif
#ifndef AVOID_CURVE_SCALE
#define AVOID_CURVE_SCALE     70    // 弯道上绕行动作的时长百分比(%)   // ----
#endif

// =========================================================================
//  7. 避障回线平滑接管配置 (★ 核心修复：彻底解决回线切回巡线丢线难题 ★)
// =========================================================================
// 平滑接管过渡时间 (单位: ms)
// - 默认 250ms：中线拉正后以柔和速度闭环巡线 250ms，彻底稳在黑线上再交还全速主循环
// - 设为 0：不进行过渡，对准后直接交还主循环
#ifndef AVOID_HANDOVER_MS
#define AVOID_HANDOVER_MS             250
#endif

// 平滑接管巡线基准速度 (PWM 0~255，推荐 30~34，温和平顺无冲击)
#ifndef AVOID_HANDOVER_SPEED
#define AVOID_HANDOVER_SPEED          32
#endif

// =========================================================================
//  8. 保护
// =========================================================================
#ifndef AVOID_COOLDOWN_MS
#define AVOID_COOLDOWN_MS      1500 // 避障后冷却(ms)，彻底保证回线后稳定跑在线上，防对已过障碍二次误触发
#endif
#ifndef AVOID_FINISH_SHIELD_MS
#define AVOID_FINISH_SHIELD_MS 5000 // 避障后屏蔽终点线误判   // ----
#endif

// =========================================================================
//  接口
// =========================================================================
// 比赛规则：障碍物只放在黑线的【一侧】，不会出现左右同时有障碍，
// 所以只有这三种状态，不需要"两侧都有"的优先级分支。
enum ObstacleSide {
  OBS_NONE  = 0,   // 无障碍
  OBS_LEFT  = 1,   // 障碍在左 -> 向右绕
  OBS_RIGHT = 2    // 障碍在右 -> 向左绕
};

extern unsigned long g_shield_finish_until;

void     Avoid_init(void);
ObstacleSide Avoid_detect(void);
void     Avoid_perform(ObstacleSide side = OBS_NONE);
void     Avoid_reset(void);

float    Avoid_getLeftCm(void);
float    Avoid_getRightCm(void);
unsigned long Avoid_getLeftUs(void);
unsigned long Avoid_getRightUs(void);
bool     Avoid_leftBlocked(void);
bool     Avoid_rightBlocked(void);

#endif