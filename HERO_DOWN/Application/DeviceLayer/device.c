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
dev_list_t dev_list = {
	.imu_sen    = &imu_sensor,
	
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	imu_sensor.init(&imu_sensor);
	rm_motor_list_init();

}
/**
  * @Name    Device_Work
  * @brief   设备总控，任务中调用
**/
void Device_Work(void)
{
	Chassis_Work(&chassis);
}

