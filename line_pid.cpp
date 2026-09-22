#include "line_pid.hpp"


#define Speed_Line (90) //巡线的速度

#define KPx  (22.6) //
#define KIx  (0) //
#define KDx  (1.) //


float KP_x,KI_x,KD_x; //x轴方向PID


int error_x,error_last_x,integral_x;

int err;//红外检测线位置上的错误

void init_x_PID(void)
{ 
    KP_x = KPx;
    KI_x = KIx;
    KD_x = KDx;

    error_x = 0;
    error_last_x = 0;
    integral_x = 0;
    
}




int PID_count_x(void)
{
    int pwmoutx = 0;
    //这里求算错误
    LineWalking();

    error_x = err - 0; //0:代表在线中间
//    Serial.println(error_x);
    
    //位置式
    pwmoutx = error_x * KP_x + integral_x *KI_x + error_last_x *KD_x ;

    integral_x += error_x; 
    error_last_x = error_x;

    if(integral_x > 30)
        integral_x = 30;
    else if(integral_x < -30)
        integral_x = -30; 

    return pwmoutx;
}

//x1-x8 从左往右数
uint8_t x1,x2,x3,x4,x5,x6,x7,x8;
void LineWalking(void)
{
  if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1110 1111
	{
		err = -1;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1100 1111
	{
		err = -2;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1101 1111
	{
		err = -2;
	}
	
	else if(x1 == 1 && x2 == 0  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1011 1111
	{
		err = -3;
	}
	else if(x1 == 1 && x2 == 0  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1001 1111
	{
		err = -3;
	}
		else if(x1 == 0 && x2 == 0  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 0011 1111
	{
		err = -4;  
	}
	else if(x1 == 0 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 0111 1111
	{
		err = -4; 
	}

	
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 1  && x7 == 1 && x8 == 1) // 1111 0111
	{
		err = 1;
	} 
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 0011
	{
		err = 2;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 1011
	{
		err = 2;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 0 && x8 == 1) // 1111 1001
	{
		err = 3;
	}
	
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 0 && x8 == 1) // 1111 1101
	{
		err = 3;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 0 && x8 == 0) // 1111 1100
	{
		err = 4; 
	}
		else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 0) // 1111 1110
	{
		err = 4; 
	}
	

	
 
	else if(x1 == 1 &&x2 == 1 &&x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1&& x8 == 1) //直走
	{
		err = 0;
	}
	
	//剩下的就保持上一个状态	
}



void Car_line_track(void)
{
    int PWMx;
    PWMx = PID_count_x();

    Set_speed(Speed_Line,PWMx); //传入值即可
}

