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


void LineWalking(void);
void init_x_PID(void);
int PID_count_x(void);
void Car_line_track(void);




#ifdef __cplusplus
}
#endif

#endif



