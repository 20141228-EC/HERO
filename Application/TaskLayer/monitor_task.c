/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
  ******************************************************************************
  */
#include "monitor_task.h"
#include "device.h"
#include "Ui_Task.h"

uint16_t open_ui = 0;
uint32_t test_time;
uint32_t test_flag;

//float tar,mea,out;
void StartMonitorTask(void const * argument)
{
	
	for(;;)
	{
		rc_sensor.heart_beat(&rc_sensor);
		imu_sensor.heart_beat(&imu_sensor.work_state);
//		Sd_Group.group_heartbeat(&Sd_Group);	  //在Chassis.heartbeat里
//		Wheel_Group.group_heartbeat(&Wheel_Group);//在Chassis.heartbeat里
		Dail_Motor.single_heart_beat(&Dail_Motor);
		Yaw_Motor.single_heart_beat(&Yaw_Motor);
		Chassis.heartbeat(&Chassis);
		Cmd_Heartbeat();
		HAL_IWDG_Refresh(&hiwdg1);
		D_Board_HeartBeat();
		
//		tar = chassis_PID.length_cal[R_Leg]->target;
//		mea = chassis_PID.length_cal[R_Leg]->measure;
//		out = chassis_PID.length_speed_cal[R_Leg]->out;
			if(open_ui == 0)
			{
				My_Ui_Init();
				open_ui = 1;
			}
			else
			{
			  Ui_Info_Update();
        Ui_Send();
				test_time++;
				
				if(test_time>=1498)
				{
					test_flag++;
				}
			}
		osDelay(1);
	}
}

