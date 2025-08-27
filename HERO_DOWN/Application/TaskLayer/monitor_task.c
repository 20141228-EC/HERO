/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   监控任务
  *          1. 各模块心跳失联检测
  *          2. 监控遥控器状态，软件复位
  ******************************************************************************
  */
#include "monitor_task.h"

 

void StartMonitorTask(void const * argument)
{
	
	for(;;)
	{
 
		rm_motor_list_heart_beat(); //rm电机心跳，在CAN中断的rm_motor_update更新
		imu_sensor.heart_beat(&imu_sensor.work_state);//imu心跳
		cap.heart_beat(&cap);//超电心跳
		Communicate_Heartbeat();//板件通信心跳，在CAN中断的各项信息接收里更新
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(1);
		
	}
}


