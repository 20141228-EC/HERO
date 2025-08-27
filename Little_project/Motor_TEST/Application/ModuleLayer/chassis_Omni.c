/* Includes ------------------------------------------------------------------*/
#include "chassis_Omni.h"

/*------------------------- Private variables start --------------------------------*/
rc_info_t rc_info;
v_Omni_t v_Omni;
Mecanum_mode_t Mecanum_mode;


/*------------------------- Private functions start --------------------------------*/

float rotated_X;
float rotated_Y ;
void Omni_calculate(float k_speed)
{
	int16_t X,Y;
	float A1,B1,B2,A2;
	int16_t spin_speed=rc_sensor_info.ch0;
	float yaw_err=(rm_motor[4].info->angle-3200)*2*3.1415926/8192;
	rc_info.rc_right =rc_sensor_info.ch2;
	rc_info.rc_front =rc_sensor_info.ch3;
	X=rc_info.rc_right;
	Y=rc_info.rc_front;
	rotated_X = X * cos(yaw_err) - Y * sin(yaw_err);
    rotated_Y = X * sin(yaw_err) + Y * cos(yaw_err);

	
	A1=  -rotated_X - rotated_Y - spin_speed;
	A2= +rotated_X + rotated_Y  - spin_speed;
	B1=  +rotated_X - rotated_Y - spin_speed;
	B2= -rotated_X + rotated_Y  - spin_speed;           
							    
	v_Omni.v_CHAS_LF=A1*k_speed ;
	v_Omni.v_CHAS_RF=B1*k_speed ;
	v_Omni.v_CHAS_LB=B2*k_speed ;
	v_Omni.v_CHAS_RB=A2*k_speed ;

}           

void Omni_Start()
{
	
	rc_offline_cnt++;
	if(rc_offline_cnt>=100)
	{rc_sensor.work_state=DEV_OFFLINE;}
	
	if(rc_sensor.work_state==DEV_OFFLINE)
		{
			CAN_SendAllZero();
		}
		else if(rc_sensor.work_state==DEV_ONLINE)
		{
			Omni_calculate(4);
			CAN_cmd_chassis(v_Omni.v_CHAS_LF,v_Omni.v_CHAS_RF,v_Omni.v_CHAS_LB,v_Omni.v_CHAS_RB);
		}
}


/*********************AOmni_calc**************************/
//int16_t X,Y;
//	float A1,B1,B2,A2;
//	float yaw_err=(rm_motor[4].info->angle-3200)*2*3.1415926/8192;
//	rc_info.rc_right =rc_sensor_info.ch2;
//	rc_info.rc_front =rc_sensor_info.ch3;
//	int16_t spin_speed=rc_sensor_info.ch0*k_speed;
//	X=rc_info.rc_right;
//	Y=rc_info.rc_front;
//	
//	A1=0.7*(X+Y);
//	A2=A1;
//	B1=0.7*(X-Y);
//	B2=B1;                        
//	v_Omni.v_CHAS_LF=  -A1/4*k_speed - spin_speed/4;
//	v_Omni.v_CHAS_RF=  B1/4*k_speed - spin_speed/4;
//	v_Omni.v_CHAS_LB=  -B2/4*k_speed - spin_speed/4;
//	v_Omni.v_CHAS_RB=  A2/4*k_speed - spin_speed/4;

/*------------------------- Private functions end --------------------------------*/
