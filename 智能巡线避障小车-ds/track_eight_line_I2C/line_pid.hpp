#ifndef __LINE_PID_HPP_
#define __LINE_PID_HPP_


#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include "motor_car.hpp"
//#include "IReight_model.hpp"

#ifdef __cplusplus
extern "C" {
#endif


/* 旧接口 LineWalking()/PID_count_x()/Car_line_track() 已由 line_follow() 取代 */
void init_x_PID(void);
/* 旧接口 PID_count_x(); 已由 line_error_mm(); 取代 */
/* ---------------- 8路循迹传感器 ---------------- */
// 读一次I2C巡线模块, 返回8路掩码(bit7..bit0 = x1..x8), 同时刷新 x1..x8
uint8_t line_read(void);
// 掩码中"压到黑线"的通道数(0~8)
uint8_t line_black_count(uint8_t mask);

/* ---------------- 巡线一步(核心) ---------------- */
// 事件位, 由 line_follow() 返回
#define LINE_EV_NONE  0x00
#define LINE_EV_LOST  0x01   // 本周期8路全白(丢线)
#define LINE_EV_WIDE  0x02   // 本周期黑线过宽(疑似发夹/黑块)
#define LINE_EV_FULL  0x04   // 本周期8路全黑(横线/发夹顶)

void init_x_PID(void);                       // PID与里程初始化
int  line_error_mm(uint8_t mask);            // 计算偏差(mm, 正=线在车右侧)
uint8_t line_follow(uint8_t mask);           // 正常巡线一步(含弯道减速/丢线保持/里程), 返回事件
void line_follow_speed(uint8_t mask, int base_speed, int target_offset_mm); // 指定速度与偏置(避障用)
void line_set_offset(int mm);                // 目标偏置(mm, >0 让线保持在车右侧)
int  line_last_error_mm(void);               // 上一次有效偏差(原始, 未减偏置)
int  line_last_dir(void);                    // 最近"明显偏差"的方向(1右/-1左), 丢线找回用
int  line_err_envelope_mm(void);             // 偏差包络(判断是否正在转弯)
uint32_t line_dist_mm(void);                 // 软里程估计(mm)
void line_dist_reset(void);




#ifdef __cplusplus
}
#endif

#endif



