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
	#ifdef PITCH_4310
	dm_motor_list_init();
	rm_motor_list_init();
	#else
	rm_motor_list_init();
	#endif
//    kt_motor_list_init();
//	kt_enable(&kt_motor[0]);
//	ht_motor_list_init();
//	dm_motor_list_init();
	

}

