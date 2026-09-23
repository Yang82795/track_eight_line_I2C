/* =========================================================================
 *  重走古丝路 · 自动循迹避障小车   主程序
 *
 *  硬件: 8路I2C红外循迹 + HC-SR04超声波 + 舵机扫描 + 双直流电机
 *  思路(对应比赛计分点):
 *   赛段分40: 稳稳沿线跑完全程 → 连续质心偏差PD + 弯道减速 + 丢线原地找回
 *   避障分40: 障碍随机放线的任意一侧 → 超声波扫描判断侧别 + 贴边绕行
 *   时间分40: 直道快、弯道慢(弯道减速), 避免为提速而冲出发夹弯
 *   停止分20: 终点横线(黑块突然变宽)用"先刹后判"识别并刹车停住
 *
 *  ⚠ 上车前必须确认: line_pid.cpp 里的 BLACK_IS_ZERO(0=压线)、SENSOR_PITCH_MM(传感器间距)
 *  📋 现场调试顺序/记录表/待定夺事项: 工程根目录 `现场调试步骤与待定夺清单.md`
 *     参数含义速查: 同目录 `调参说明.md`
 * ========================================================================= */

#include <stdio.h>
#include "motor_car.hpp"
#include "line_pid.hpp"
#include "avoid_obstacle.hpp"

#define u8 uint8_t

#define KEY_PIN 2
const int Press_KEY = 0;
const int Release_KEY = 1;

extern uint8_t x1,x2,x3,x4,x5,x6,x7,x8;    // 8路红外状态(从左往右), 定义在 line_pid.cpp

/* ------------------------- 主循环与判定参数 ------------------------- */
#define CTRL_PERIOD_MS   (10)      // 控制周期 10ms = 100Hz
#define DEBUG_SERIAL     (1)       // 调参时1; 正式比赛建议0(并拔掉USB线)
#define DEBUG_PERIOD_MS  (300)     // 串口调试输出间隔

#define SEARCH_LOST_MS    (60)     // 全白持续这么久 → 原地找线
#define SEARCH_TIMEOUT_MS (1300)   // 找线超时 → 停车(防止跑出赛道/逆行)
#define SEARCH_PWM        (62)     // 原地找线速度

#define CROSS_MIN_MS      (12000)  // 终点线时间闸门(起步后至少12s)
#define CROSS_MIN_MM      (4500)   // 终点线里程闸门(至少4.5m, 挡住赛道前中段的发夹弯)
#define CROSS_NORMAL_MS   (400)    // 候选前必须有"正常压线"记录
#define CROSS_NORMAL_RUN  (3)      // 候选前需连续正常压线周期数(区分"突然出现"与"逐渐变宽")
#define CROSS_ENV_MM      (32)     // 候选前偏差包络需小于此值(近似走直); 误停改小, 检不到终点改999
#define CROSS_WIDE_CNT    (5)      // "宽黑"门限: 黑点数≥5 视为横线/发夹候选
#define CROSS_WIDE_CYCLES (2)      // 宽黑需连续这么多周期才进入判定
#define BRAKE_MS          (90)     // 电子刹车时间
#define CREEP_PWM         (68)     // 判定爬行速度(必须>电机起转60)
#define CREEP_MS          (450)    // 判定爬行最长时间(≈90mm行程)
#define CREEP_CLEAR_MS    (250)    // 黑块必须在这段时间内结束, 才算"薄横线"; 超时=发夹黑块
#define CREEP_CLEAR_CNT   (3)      // 爬行中黑点数降到≤此值 → 黑块结束(终点线候选)
#define CREEP_TURN_MM     (18)     // 爬行中质心偏差超过此值 → 黑块偏在一边, 判为发夹

enum { CAR_IDLE = 0, CAR_RUN, CAR_SEARCH, CAR_DONE };
static uint8_t  g_state = CAR_IDLE;
static uint32_t g_startMs = 0, g_lastNormalMs = 0, g_lostSince = 0;
static uint32_t g_searchStart = 0, g_lastDebug = 0, g_loopLast = 0;
static uint8_t  g_wideCnt = 0;          // 当前宽黑已持续周期数
static uint8_t  g_normalRun = 0;        // 当前连续正常压线周期数
static uint8_t  g_runBeforeWide = 0;    // 宽黑出现前的连续正常压线周期数
static int8_t   g_searchDir = 1;

int  getKeyState(uint8_t pin);
static bool cross_gate_ok(void);
static void cross_handle(void);
static void debug_print(void);

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    pinMode(KEY_PIN, INPUT_PULLUP);
    delay(200);

    avoid_init();        // 舵机回中 + 超声波引脚
    Motor_init();        // 电机初始化
    init_x_PID();        // PID/里程初始化

    Car_stop();
    Serial.println(F("=== Silk-Road Line Car Ready. Align at START line, press KEY ==="));
}

void loop()
{
    uint32_t now = millis();

    /* ---------- 100Hz 定周期调度(去掉原代码每圈的 delay(10)+Serial 阻塞) ---------- */
    if ((uint32_t)(now - g_loopLast) < CTRL_PERIOD_MS) return;
    g_loopLast = now;

    switch (g_state)
    {
    /* ---------------- 待机: 等按键 ---------------- */
    case CAR_IDLE:
        if (getKeyState(KEY_PIN) == Press_KEY)
        {
            delay(200);                 // 等手离开
            init_x_PID();
            line_dist_reset();
            g_startMs      = millis();
            g_lastNormalMs = g_startMs;
            g_lostSince    = 0;
            g_wideCnt      = 0;
            g_normalRun    = 0;
            g_runBeforeWide = 0;
            g_searchDir    = 1;
            avoid_blank(500);
            g_state = CAR_RUN;
            Serial.println(F("GO"));
        }
        break;

    /* ---------------- 巡线主状态 ---------------- */
    case CAR_RUN:
    {
        uint8_t m  = line_read();
        uint8_t n  = line_black_count(m);
        uint8_t ev;

        /* 记录"最近一次正常压线"的时间; 并统计"宽黑出现前连续正常了多少周期"
           —— 终点的横线是突然出现的, 发夹是逐渐变宽的, 这个量能区分二者 */
        if (n >= 1 && n <= 4)
        {
            g_lastNormalMs = millis();
            if (g_normalRun < 250) g_normalRun++;
        }
        else if (n >= CROSS_WIDE_CNT)
        {
            if (g_wideCnt == 0) g_runBeforeWide = g_normalRun;   // 宽黑事件刚开始, 锁存历史
            if (g_wideCnt < 200) g_wideCnt++;
            g_normalRun = 0;
        }
        else
        {
            g_normalRun = 0;
            g_wideCnt = 0;
        }

        /* ---- 终点横线候选: 黑块突然变宽 ----
           正穿的横线 → 8路全黑; 斜穿的横线(本项目起点/终点线是斜穿) → 约5~6路全黑;
           发夹弯顶部同样会"变宽变黑", 所以这里只是"候选", 由 cross_handle() 先刹后判 */
        if (g_wideCnt >= CROSS_WIDE_CYCLES && g_runBeforeWide >= CROSS_NORMAL_RUN &&
            cross_gate_ok())
        {
            cross_handle();             // 先刹后判(内部会切换到 CAR_DONE 或回 CAR_RUN)
            break;
        }

        /* ---- 正常巡线一步 ---- */
        ev = line_follow(m);

        /* ---- 丢线: 原地转向找回(发夹弯/急弯的保命逻辑) ---- */
        if (ev & LINE_EV_LOST)
        {
            if (g_lostSince == 0) g_lostSince = millis();
            if ((uint32_t)(millis() - g_lostSince) > SEARCH_LOST_MS)
            {
                g_searchDir = (int8_t)line_last_dir();   // 朝上次偏差方向原地转, 最容易找回线
                g_searchStart = millis();
                Car_pivot(g_searchDir, SEARCH_PWM);
                g_state = CAR_SEARCH;
                break;
            }
        }
        else
        {
            g_lostSince = 0;
        }

        /* ---- 避障(不阻塞主循环结构, 动作自带分步) ---- */
        if (avoid_need())
        {
            avoid_run();
        }
        break;
    }

    /* ---------------- 丢线原地找线 ---------------- */
    case CAR_SEARCH:
        Car_pivot(g_searchDir, SEARCH_PWM);
        if (line_black_count(line_read()) > 0)
        {
            Car_stop();
            g_lostSince = 0;
            g_state = CAR_RUN;
        }
        else if ((uint32_t)(millis() - g_searchStart) > SEARCH_TIMEOUT_MS)
        {
            Car_stop();                 // 长时间找不到线: 停车, 避免跑出赛道/逆行
            g_state = CAR_DONE;
            Serial.println(F("LOST-STOP"));
        }
        break;

    /* ---------------- 已结束: 保持停车 ---------------- */
    case CAR_DONE:
    default:
        Car_stop();
        break;
    }

    debug_print();
}


/* =======================================================================
 *  终点横线判定
 *
 *  难点: 赛道发夹弯顶部两条腿会并成一块黑, 传感器同样"黑点数变多",
 *        和终点横线(500mm长×20mm宽)的读数很接近!
 *  对策(三步):
 *    1) 闸门: 至少跑了 12s 且 4.5m; 宽黑前连续正常压线(1~4路)≥3个周期;
 *             宽黑前偏差包络 < 32mm(说明是直线段, 不是在发夹里搓弯);
 *    2) 先刹: 命中候选立刻电子刹车(反正终点也要停, 这条同时保住"停止分");
 *    3) 后判: 以 68 PWM 慢速前爬最多 450ms(≈90mm 行程):
 *             - 黑块在 250ms 内结束 → 薄横线 → 确认终点, 停车结束;
 *             - 爬了 250ms 还是黑的(黑块沿路径 150mm+) → 发夹顶部两腿合并的黑块 → 误报;
 *             - 黑块一直偏在一侧(质心偏差>18mm) → 车正在转弯 → 误报。
 *     误报时只损失约 0.3~0.6s, 但绝对不会把"终点前"当成终点而提前停车。
 * ======================================================================= */
static bool cross_gate_ok(void)
{
    uint32_t now = millis();

    /* 闸门1: 时间和里程双闸门(发夹弯集中在赛道前中段, 终点在最末段) */
    if ((uint32_t)(now - g_startMs) < CROSS_MIN_MS) return false;
    if (line_dist_mm() < CROSS_MIN_MM)              return false;

    /* 闸门2: 宽黑之前必须一直在正常压线(排除起步压在起始线上) */
    if ((uint32_t)(now - g_lastNormalMs) > CROSS_NORMAL_MS) return false;

    /* 闸门3: 不是在急弯里(偏差包络小说明最近基本走直) */
    if (line_err_envelope_mm() > CROSS_ENV_MM) return false;

    return true;
}

static void cross_handle(void)
{
    uint32_t t0;
    uint8_t  result = 0;                     // 0=未定, 1=终点线确认, 2=发夹/黑块误报
    uint32_t elapsed;

    Car_brake(BRAKE_MS);                     // 先刹: 无论如何都要保住"停止分"(车尾不越线)

    t0 = millis();
    for (;;)
    {
        uint8_t m;
        uint8_t n;
        int     e;

        elapsed = (uint32_t)(millis() - t0);

        Car_drive(CREEP_PWM, CREEP_PWM, 1);  // 慢速前爬, 看黑块会不会很快结束
        delay(10);

        m = line_read();
        n = line_black_count(m);
        e = line_error_mm(m);

        if (n <= CREEP_CLEAR_CNT)
        {
            /* 黑块结束: 只有"结束得足够快"才算终点横线。
               终点横线(20mm厚)无论正穿还是斜穿, 黑块沿路径只有 20~60mm;
               发夹/两腿合并的黑块沿路径 150mm 以上, 早就超过 CREEP_CLEAR_MS 了。 */
            result = (elapsed <= CREEP_CLEAR_MS) ? 1 : 2;
            break;
        }
        if (e > CREEP_TURN_MM || e < -CREEP_TURN_MM)
        {
            result = 2;                      // 黑块长时间偏在一侧 → 车在转弯, 是发夹不是横线
            break;
        }
        if (elapsed >= CREEP_MS)
        {
            result = 2;                      // 爬够了还是黑的 → 发夹黑块
            break;
        }
    }
    Car_stop();

    if (result == 1)
    {
        g_state = CAR_DONE;                  // 确认终点线 → 停车结束
        Serial.println(F("FINISH-STOP"));
    }
    else
    {
        /* 误报(发夹顶部黑块/斜跨黑块) → 清计数继续巡线, 不能在这里停机! */
        g_wideCnt       = 0;
        g_normalRun     = 0;
        g_runBeforeWide = 0;
        g_state = CAR_RUN;
        Serial.println(F("CROSS-FALSE"));
    }
}

/* =======================================================================
 *  调试输出(把线型/偏差/里程打出来, 用于标定参数)
 *  一行格式:
 *    st状态 m=掩码 n=黑点数 k=黑块形态 e=偏差mm v=偏差包络mm t=秒 d=里程cm
 *    D=前方距离mm L=左侧扫描mm R=右侧扫描mm S=避障判定侧别
 *    k: 0正常(1~4路) 1宽黑(5~7路) 2全黑(8路) 3丢线(全白)
 *    S: +1障碍在右(向左绕) -1障碍在左(向右绕)
 *  典型用法: 跑完一圈看最后一行 d 值 → 校准 CROSS_MIN_MM / MM_PER_PWM_SEC;
 *            终点线/发夹时看 n/k/e/v → 校准 CROSS_WIDE_CNT / CROSS_ENV_MM / CREEP_CLEAR_MS
 *            避障时看 L/R/S → 校准 SONAR_TRIGGER_MM / SERVO_* / LINE_OFFSET_MM
 * ======================================================================= */
static void debug_print(void)
{
#if DEBUG_SERIAL
    uint32_t now = millis();
    uint8_t  m;
    uint8_t  n;
    uint8_t  k;

    if ((uint32_t)(now - g_lastDebug) < DEBUG_PERIOD_MS) return;
    g_lastDebug = now;

    m = (uint8_t)((x1 << 7) | (x2 << 6) | (x3 << 5) | (x4 << 4) |
                  (x5 << 3) | (x6 << 2) | (x7 << 1) | (x8 << 0));
    n = line_black_count(m);

    /* k = 黑块形态: 0正常(1~4路) 1宽黑(5~7路) 2全黑(8路) 3丢线(全白)
       S = 本次避障判定: +1 障碍在右(向左绕) / -1 障碍在左(向右绕) */
    if      (n == 0) k = 3;
    else if (n >= 8) k = 2;
    else if (n >= 5) k = 1;
    else             k = 0;

    Serial.print(F("st"));  Serial.print(g_state);
    Serial.print(F(" m="));   if (m < 0x10) Serial.print('0'); Serial.print(m, HEX);
    Serial.print(F(" n="));   Serial.print(n);
    Serial.print(F(" k="));   Serial.print(k);
    Serial.print(F(" e="));   Serial.print(line_last_error_mm());
    Serial.print(F(" v="));   Serial.print(line_err_envelope_mm());
    Serial.print(F(" t="));   Serial.print((now - g_startMs) / 1000);
    Serial.print(F(" d="));   Serial.print(line_dist_mm() / 10);
    Serial.print(F(" D="));   Serial.print(avoid_last_dist_mm());
    Serial.print(F(" L="));   Serial.print(avoid_last_left_mm());
    Serial.print(F(" R="));   Serial.print(avoid_last_right_mm());
    Serial.print(F(" S="));   Serial.println(avoid_last_side());
#else
    (void)0;
#endif
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

