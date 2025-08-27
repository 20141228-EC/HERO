/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"

void StartMonitorTask(void const *argument)
{

	for (;;)
	{

		rm_motor_list_heart_beat();					   // rm电机心跳，在CAN中断的rm_motor_update更新
		imu_sensor.heart_beat(&imu_sensor.work_state); // imu心跳
		rc_sensor.heart_beat(&rc_sensor);			   // 遥控器心跳,串口3中断的rc_sensor_update
		kt_motor[0].heartbeat(&kt_motor[0]);		   // 领控电机心跳
		Vision_HearBeat();							   // 视觉通信心跳，串口6更新
		Cmd_Heartbeat();							   // 命令执行超时退出，更新命令cmd值
		Communicate_Heartbeat();					   // 板件通信心跳，在CAN中断的各项信息接收里更新
		Soft_Reset();//XC软件复位

			HAL_IWDG_Refresh(&hiwdg);
		osDelay(1);
	}
}

/**
 * @brief  软件复位
 */
void Soft_Reset(void)
{
	if ((rc_sensor.info->X.value == 1 && rc_sensor.info->C.value == 1) || (rc_sensor.info->thumbwheel.value < -450 && IF_RC_SW2_UP && IF_RC_SW1_DOWN))
	{
		CAN_SendAllZero();
		HAL_Delay(500);
		__set_FAULTMASK(1); // 屏蔽中断
		NVIC_SystemReset();
	}
}
