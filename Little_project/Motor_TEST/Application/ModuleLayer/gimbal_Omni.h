#ifndef __GIMBAL_OMNI_H
#define __GIMBAL_OMNI_H

/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "rp_config.h"
#include "drv_uart.h"
#include "math.h"
#include "rp_math.h"
#include "arm_math.h"
/*陀螺仪*/
#include "imu_sensor.h"
#include "bmi.h"

/* Private macro -------------------------------------------------------------*/
#define CHASSIS_SPEED_MAX				8700.f	// 每个电机的最大转速
#define CHASSIS_TOP_SPEED 			8000.f	// 原地小陀螺到每个电机转速
#define CHASSIS_TUNNEL_LIMIT		3000.f	// 过隧道模式速度限制

/* Private typedef -----------------------------------------------------------*/



/*------------private function----------------*/
//void Pitch_to_target(int16_t angle);
//void Yaw_to_target(int16_t angle);
/* Exported variables --------------------------------------------------------*/

#endif
