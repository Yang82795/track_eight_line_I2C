#ifndef __LINE_PID_HPP_
#define __LINE_PID_HPP_

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <Wire.h>
#include "motor_car.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// 8路红外探头数据 (x1最左，x8最右，0=检测到黑线，1=白底)
extern uint8_t x1, x2, x3, x4, x5, x6, x7, x8;

// ==========================================
// 循迹速度与 PID 参数设置 (现场最主要调这几个)
// ==========================================
#define Speed_Line       (100)  // 直道标准巡线速度 (推荐 90~110)
#define Speed_Curve      (85)   // 普通弯道速度
#define Speed_Sharp      (70)   // 急弯/发夹弯安全降速速度 (保证急弯不冲出)

#define KPx              (18.5f) // 比例系数：决定转向响应灵敏度
#define KIx              (0.0f)  // 积分系数：循迹通常为0，防止饱和
#define KDx              (8.0f)  // 微分系数：抑制弯道摆头震荡 (注意原版公式为标准微分)

// ==========================================
// 接口函数
// ==========================================
void I2Cdata(void);             // 读取 8 路红外 I2C 数据
void init_x_PID(void);          // 初始化 PID 控制器与误差
int  PID_count_x(void);         // 计算转向差速输出
void Car_line_track(void);      // 执行一步自适应巡线

bool isCrossLine(void);         // 是否检测到起终点横线 (>=6路同时黑)
int  getBlackCount(void);       // 获取当前黑线探头数量 (0~8)
float getLineError(void);       // 获取当前偏差量 (-7.0 ~ +7.0)

#ifdef __cplusplus
}
#endif

#endif
