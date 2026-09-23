#ifndef __AVOID_OBSTACLE_HPP_
#define __AVOID_OBSTACLE_HPP_

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include "motor_car.hpp"
#include "line_pid.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/* 超声波 + 舵机 + 避障流程
 * 说明: 障碍物随机放在循迹线任意一侧, 所以这里全部用"现场测量"决定绕行方向,
 *       不写死左绕或右绕。避障时允许短时偏离循迹线(比赛规则允许)。
 */

void     avoid_init(void);          // 舵机/超声波引脚初始化
uint16_t sonar_read_mm(void);       // 单次测距(mm), 超时(无障碍)返回0
bool     avoid_need(void);          // 是否需要避障(内部限频+连续2次确认+屏蔽期)
void     avoid_blank(uint16_t ms);  // 屏蔽超声波一段时间(避免重复触发)
void     avoid_run(void);           // 完整避障动作(扫描→偏出→贴边通过→收尾)
int      avoid_last_side(void);     // 上次判定: +1 障碍在右(向左绕), -1 障碍在左
uint16_t avoid_last_dist_mm(void);  // 上次前方距离(调试用)
uint16_t avoid_last_left_mm(void);
uint16_t avoid_last_right_mm(void);

#ifdef __cplusplus
}
#endif

#endif
