#ifndef LINE_PID_HPP
#define LINE_PID_HPP

#include <Arduino.h>
#include "motor_car.hpp"

/* =========================================================================
 *  8 路循迹控制核心 (I2C 红外 + 连续质心偏差 + 位置式PD + 弯道减速 +
 *                     软里程 + 事件上报)
 *
 *  传感器约定：x1 在车体最左、x8 在最右；车头朝前。
 *  本项目所用模块检测到黑线时输出 0，检测到白底时输出 1
 *  (可用 line_pid.cpp 顶部的 BLACK_IS_ZERO 反转)。
 * ========================================================================= */

extern uint8_t x1, x2, x3, x4, x5, x6, x7, x8;   // 8 路状态(从左往右)

/* ---------------- 巡线事件位(由 line_follow 返回) ---------------- */
#define LINE_EV_NONE  0x00
#define LINE_EV_LOST  0x01   // 本周期 8 路全白(丢线)
#define LINE_EV_WIDE  0x02   // 本周期黑线过宽(疑似发夹/黑块/横线)
#define LINE_EV_FULL  0x04   // 本周期 8 路全黑

void init_x_PID(void);                       // PID/里程/状态初始化
void ResetLineController(void);              // 仅复位 PID 累积项(状态切换用)

/* ---------------- 传感器读取 ---------------- */
uint8_t line_read(void);                     // 读一次 I2C，返回掩码(bit7..bit0=x1..x8)，同时刷新 x1..x8
bool    line_read_ok(void);                  // 上一次 line_read 是否成功(I2C 未失败)
uint8_t line_black_count(uint8_t mask);      // 掩码中压黑线的通道数(0~8)

/* ---------------- 巡线一步(核心) ---------------- */
int     line_error_mm(uint8_t mask);         // 纯质心偏差(mm，正=线在车右侧)
uint8_t line_follow(uint8_t mask);           // 正常巡线一步(含弯道减速/丢线保持/里程)，返回事件
void    line_follow_speed(uint8_t mask, int base_speed, int target_offset_mm); // 指定速度与偏置(避障贴边用)
void    line_set_offset(int mm);             // 目标偏置(mm，>0 让线保持在车右侧)

/* ---------------- 查询接口(供主状态机使用) ---------------- */
int      line_last_error_mm(void);           // 上一次有效偏差(mm，原始，未减偏置)
int      line_last_dir(void);                // 最近"明显偏差"的方向(1右/-1左)，丢线找回用
int      line_err_envelope_mm(void);         // 偏差包络(判断是否正在转弯)
uint32_t line_dist_mm(void);                 // 软里程估计(mm)
void     line_dist_reset(void);

#endif
