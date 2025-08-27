#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "motor.h"
#include "RM_motor.h"
#include "KT_motor.h"
#include "KT_module.h"
#include "gimbal.h"
#include "chassis.h"
#include "shooting.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum{
	DEVICE_01,
	DEVICE_02,
	DEVICE_03,
	DEVICE_04,
	DEVICE_05,
	DEVICE_06,
	DEVICE_07,
	DEVICE_08,
	DEVICE_09,
	DEVICE_10,
	DEVICE_11,
	DEVICE_CNT
}device_cnt_e;


typedef struct{
	rc_sensor_t			*rc_sensor;
	imu_sensor_t		*imu_sensor;
	rm_motor_t			*rm_motor;
	KT_motor_t			*kt_motor;
}device_t;


extern device_t device[DEVICE_CNT];

/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void);
void Device_Work(void);
/* Servo functions */


#endif
