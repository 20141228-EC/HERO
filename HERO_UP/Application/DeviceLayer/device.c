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


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	imu_sensor.init(&imu_sensor);
	rc_sensor.init(&rc_sensor);
	rm_motor_list_init();
    kt_motor_list_init();
	Read_kt_FISE(); //获取电机数据
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

