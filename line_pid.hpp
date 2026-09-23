#ifndef LINE_PID_HPP
#define LINE_PID_HPP

#include <Arduino.h>
#include "motor_car.hpp"

/*
 * 8 路循迹模块的约定：x1 在车体左侧，x8 在车体右侧；
 * 本项目所用模块检测到黑线时输出 0，检测到白色底板时输出 1。
 */
extern uint8_t x1, x2, x3, x4, x5, x6, x7, x8;

void init_x_PID(void);
void ResetLineController(void);

/* 根据最近一次 I2C 采样更新黑线位置。 */
void LineWalking(void);
int PID_count_x(void);
void Car_line_track(void);

/* 供主程序的避障、终点状态机使用。 */
bool IsLineDetected(void);
bool IsLineCrossing(void);
uint8_t GetLineBlackCount(void);
int GetLastLineError(void);

#endif
