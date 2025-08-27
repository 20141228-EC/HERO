/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
device_t device [] =
{	
	[DEVICE_01] = {
	 .kt_motor = &kt_motor[0],	
	},
	[DEVICE_02] = {
	 .rm_motor = &rm_motor[GIMB_P],	
	},
	[DEVICE_03] = {
	 .rm_motor = &rm_motor[DAIL],	
	},
	[DEVICE_04] = {
	 .rm_motor = &rm_motor[LIMIT],	
	},
	[DEVICE_05]= {
	 .rm_motor = &rm_motor[FRIC_LF],	
	},
	[DEVICE_06] = {
	 .rm_motor = &rm_motor[FRIC_RF],	
	},
	[DEVICE_07] = {
	 .rm_motor = &rm_motor[FRIC_LB],	
	},
	[DEVICE_08] = {
	 .rm_motor = &rm_motor[FRIC_RB],	
	},
	[DEVICE_09] = {
	 .rm_motor = &rm_motor[IMAGE],
	},
	[DEVICE_10] = {
	 .imu_sensor = &imu_sensor,	
	},
	[DEVICE_11] = {
	 .rc_sensor = &rc_sensor,	
	},
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	device[DEVICE_10].imu_sensor->init(device[DEVICE_10].imu_sensor);
	device[DEVICE_11].rc_sensor->init(device[DEVICE_11].rc_sensor);
	rm_motor_list_init();
    kt_motor_list_init();
	ht_motor.init(&ht_motor);
	Read_kt_FISE(); //防止无法更新kt电机原始数据
	kt_motor[0].tx_W_cmd(&kt_motor[0],MOTOR_RUN_ID);
}
/**
  * @Name    Device_Work
  * @brief   设备总控，任务中调用
**/
void Device_Work(void)
{
	
	chassis.work(&chassis);//必须底盘在云台总控前面，因为吊射命令s_run在底盘
	gimbal.work(&gimbal);
	shooting.work(&shooting);
}

