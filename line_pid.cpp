#include "line_pid.hpp"
#include <math.h>

/* =========================================================================
 *  巡线控制核心 (8 路 I2C 红外 + 连续质心偏差 + 位置式PD + 软里程 + 事件上报)
 *
 *  为什么不用"枚举位型"的老写法：
 *   1) 20mm 线 + 约 11mm 间距，正常压线只有 1~4 路，枚举必漏；质心法对任何
 *      位型都连续可算(单位 mm，PD 直接作用在物理量上)；
 *   2) 发夹弯顶部会"8 路全黑"，枚举法只能沿用上次值，无法区分"终点横线"和
 *      "发夹黑块"——这里用事件位上报给主状态机，由"先刹后判"决策。
 *
 *  融合改进(相对 -ds 版)：
 *   - I2C 读取带成功/失败标志：失败时沿用上次值(防抖)，并由主程序对
 *     "连续多次失败"做兜底停车(避免线松脱后带旧数据高速冲出赛道)；
 *   - 积分限幅 + dt 归一，抑制调试暂停/首帧的微分尖峰。
 * ========================================================================= */

/* -------------------- 可调参数(现场主要调这几个) -------------------- */
#define SENSOR_ADDR      (0x12)    // 巡线模块 I2C 地址
#define SENSOR_REG       (0x30)    // 读取寄存器
#define SENSOR_READ_US   (0)       // 读命令与读数据间隔(us)，读数不稳时改 200~500

#define SENSOR_PITCH_MM  (11.0f)   // 【实测】相邻两路红外间距(mm)
#define BLACK_IS_ZERO    (1)       // 1: 0=压黑线(模板默认)；0: 1=压黑线

#define Speed_Line       (110)     // 直道基准速度(PWM)
#define Speed_Min        (88)      // 普通弯道最低速度(PWM)
#define Speed_Crest      (75)      // 发夹/黑块区速度(PWM)
#define Speed_Lost       (75)      // 丢线保持速度(PWM)
#define K_SLOW           (0.35f)   // 每 mm 偏差降速(PWM/mm)
#define KPx              (3.0f)    // 比例 PWM/mm (太大易摆头，太小切弯不足)
#define KDx              (4.0f)    // 微分 PWM/(mm/周期)
#define KIx              (0.0f)    // 积分(一般保持0，防止绕圈饱和)
#define KD_FILT          (0.35f)   // D 项一阶滤波系数
#define I_LIMIT          (30.0f)   // 积分限幅(mm·s)
#define STEER_MAX        (95)      // 转向量限幅(PWM)
#define ERR_LAST_GAIN    (2.0f)    // 丢线时沿上次方向放大倍数
#define ERR_DIR_MEM_MM   (4.0f)    // 偏差超过此值才更新"方向记忆"
#define ERR_ENV_DECAY    (0.97f)   // 偏差包络衰减(每周期)
#define CTRL_DT_S        (0.01f)   // 控制周期(s)，与主循环一致
#define MM_PER_PWM_SEC   (0.0032f) // 【标定】每 1PWM 对应速度(m/s)，用于软里程

#define FULL_TH          (8)       // "8 路全黑"判定门限
#define WIDE_TH          (5)       // "黑线过宽"判定门限

/* ------------------------------ 全局 ------------------------------ */
uint8_t x1, x2, x3, x4, x5, x6, x7, x8;   // 从左往右

static float    g_err_valid = 0;   // 上一次"有效"偏差(mm，只有正常压线才更新)
static float    g_err_prev  = 0;   // PID 用：上一周期偏差(已减偏置)
static float    g_d_filt    = 0;   // D 项滤波值
static float    g_integral  = 0;   // I 项
static float    g_env       = 0;   // 偏差包络(|err| 峰值保持 + 衰减)
static int      g_offset    = 0;   // 目标偏置(mm)
static int8_t   g_dir_mem   = 1;   // 最近一次"明显偏差"的方向(1右/-1左)
static uint32_t g_dist      = 0;   // 软里程(mm)
static bool     g_read_ok   = true;// 上一次 I2C 读是否成功

/* ------------------------------ 内部函数 ------------------------------ */
// 判断第 i 路(i=0 对应 x1)是否压黑线
static inline uint8_t ch_is_black(uint8_t mask, uint8_t i)
{
#if BLACK_IS_ZERO
  return ((mask >> (7 - i)) & 0x01) == 0;
#else
  return ((mask >> (7 - i)) & 0x01) != 0;
#endif
}

void ResetLineController(void)
{
  g_err_prev = 0;
  g_d_filt   = 0;
  g_integral = 0;
}

void init_x_PID(void)
{
  g_err_valid = 0;
  g_env       = 0;
  g_offset    = 0;
  g_dir_mem   = 1;
  g_dist      = 0;
  g_read_ok   = true;
  x1 = x2 = x3 = x4 = x5 = x6 = x7 = x8 = 1;   // 全白
  ResetLineController();
}

/**
 * @brief 读一次 8 路巡线模块(I2C)，返回掩码 bit7..bit0 = x1..x8
 *        I2C 读失败时沿用上次结果(避免瞬时误判"丢线")，并置 g_read_ok=false，
 *        由主程序对"连续失败"做兜底停车。
 */
uint8_t line_read(void)
{
  static uint8_t last_mask = 0xFF;
  uint8_t m = last_mask;

  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(SENSOR_REG);
  if (Wire.endTransmission(false) != 0) {
    g_read_ok = false;
    // 仍然刷新 x1..x8 为上次值，保证下游逻辑可用
    x1 = (m >> 7) & 0x01; x2 = (m >> 6) & 0x01; x3 = (m >> 5) & 0x01; x4 = (m >> 4) & 0x01;
    x5 = (m >> 3) & 0x01; x6 = (m >> 2) & 0x01; x7 = (m >> 1) & 0x01; x8 = m & 0x01;
    return m;
  }

#if SENSOR_READ_US
  delayMicroseconds(SENSOR_READ_US);
#endif

  const uint8_t received = Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
  if (received == 1 && Wire.available()) {
    m = Wire.read();
    last_mask = m;
    g_read_ok = true;
  } else {
    g_read_ok = false;      // 读失败：沿用上次掩码
  }
  while (Wire.available()) Wire.read();

  x1 = (m >> 7) & 0x01; x2 = (m >> 6) & 0x01; x3 = (m >> 5) & 0x01; x4 = (m >> 4) & 0x01;
  x5 = (m >> 3) & 0x01; x6 = (m >> 2) & 0x01; x7 = (m >> 1) & 0x01; x8 = m & 0x01;
  return m;
}

bool line_read_ok(void)
{
  return g_read_ok;
}

uint8_t line_black_count(uint8_t mask)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (ch_is_black(mask, i)) n++;
  }
  return n;
}

/**
 * @brief 纯质心偏差(mm)，正 = 线偏在车的右侧(需要右转)
 *        丢线/全黑时返回"上一次有效偏差"的放大值
 */
int line_error_mm(uint8_t mask)
{
  float sum = 0;
  uint8_t n = 0;

  for (uint8_t i = 0; i < 8; i++) {
    if (ch_is_black(mask, i)) {
      sum += ((float)i - 3.5f) * SENSOR_PITCH_MM;
      n++;
    }
  }
  if (n == 0) return (int)(g_err_valid * ERR_LAST_GAIN);
  return (int)(sum / (float)n);
}

/**
 * @brief 计算本周期偏差并给出事件位
 */
static float line_calc_err(uint8_t mask, uint8_t *ev)
{
  uint8_t n = line_black_count(mask);
  float e;

  *ev = LINE_EV_NONE;

  if (n == 0) {
    /* 8 路全白：丢线(发夹/急弯冲出去)，沿上次方向继续加大转向 */
    *ev = LINE_EV_LOST;
    e = g_err_valid * ERR_LAST_GAIN;
  } else if (n >= FULL_TH) {
    /* 8 路全黑：终点横线 或 发夹顶部黑块。先按"发夹"处理(保持并放大上次转向、
       减速)；是否终点由主状态机"先刹后判"决定。 */
    *ev = LINE_EV_FULL | LINE_EV_WIDE;
    e = g_err_valid * 1.3f;
  } else if (n >= WIDE_TH) {
    /* 黑线过宽：发夹两腿同压，或斜跨黑块 */
    *ev = LINE_EV_WIDE;
    e = g_err_valid * 1.15f;
  } else {
    /* 正常压线(1~4 路)：质心法算连续偏差 */
    float sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if (ch_is_black(mask, i)) sum += ((float)i - 3.5f) * SENSOR_PITCH_MM;
    }
    e = sum / (float)n;
    g_err_valid = e;               // 只在这里更新"上一次有效偏差"
    if (e >= ERR_DIR_MEM_MM)       g_dir_mem = 1;
    else if (e <= -ERR_DIR_MEM_MM) g_dir_mem = -1;
  }

  if (e >  40.0f) e =  40.0f;
  if (e < -40.0f) e = -40.0f;
  return e;
}

/**
 * @brief 位置式PD(偏置已扣除)，返回转向量PWM(正=右转)
 */
static int line_calc_steer(float err)
{
  float used = err - (float)g_offset;      // 避障时用目标偏置实现"贴边行驶"
  float d    = used - g_err_prev;
  float s;

  g_d_filt += KD_FILT * (d - g_d_filt);
  g_err_prev = used;

  g_integral += used * CTRL_DT_S;
  if (g_integral >  I_LIMIT) g_integral =  I_LIMIT;
  if (g_integral < -I_LIMIT) g_integral = -I_LIMIT;

  s = KPx * used + KDx * g_d_filt + KIx * g_integral;
  if (s >  STEER_MAX) s =  STEER_MAX;
  if (s < -STEER_MAX) s = -STEER_MAX;
  return (int)s;
}

/**
 * @brief 输出PWM + 累计软里程 + 维护偏差包络
 */
static void line_commit(float err, int base)
{
  int steer = line_calc_steer(err);
  float ae  = fabs(err - (float)g_offset);

  if (ae > g_env) g_env = ae;
  else            g_env = g_env * ERR_ENV_DECAY;

  if (base < 0)   base = 0;
  if (base > 255) base = 255;

  Set_speed(base, steer);

  /* 软里程：无编码器，用"指令速度×时间"估算，供终点线里程闸门用 */
  g_dist += (uint32_t)((float)base * MM_PER_PWM_SEC * CTRL_DT_S * 1000.0f);
}

/**
 * @brief 正常巡线一步(含弯道减速/丢线保持/里程累计)
 * @retval 事件位(LINE_EV_xxx)
 */
uint8_t line_follow(uint8_t mask)
{
  uint8_t ev;
  float e = line_calc_err(mask, &ev);
  int base;

  if (ev & LINE_EV_LOST) {
    base = Speed_Lost;
  } else if (ev & LINE_EV_WIDE) {
    base = Speed_Crest;                     // 发夹/黑块：减速慢过
  } else {
    base = Speed_Line - (int)(K_SLOW * fabs(e - (float)g_offset));
    if (base < Speed_Min) base = Speed_Min;
    if (base > Speed_Line) base = Speed_Line;
  }

  line_commit(e, base);
  return ev;
}

/**
 * @brief 指定基准速度与目标偏置走一步(避障绕行时用)
 * @param target_offset_mm: >0 让循迹线保持在车的右侧(即车偏到线的左边)
 */
void line_follow_speed(uint8_t mask, int base_speed, int target_offset_mm)
{
  uint8_t ev;
  float e;

  g_offset = target_offset_mm;
  e = line_calc_err(mask, &ev);
  line_commit(e, base_speed);
}

void line_set_offset(int mm)      { g_offset = mm; }
int  line_last_error_mm(void)     { return (int)g_err_valid; }
int  line_last_dir(void)          { return (int)g_dir_mem; }
int  line_err_envelope_mm(void)   { return (int)g_env; }
uint32_t line_dist_mm(void)       { return g_dist; }
void line_dist_reset(void)        { g_dist = 0; }
