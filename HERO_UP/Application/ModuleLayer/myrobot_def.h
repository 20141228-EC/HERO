#ifndef __MYROBOT_DEF_H
#define __MYROBOT_DEF_H

#include "stm32f4xx_hal.h"
#include "rc_sensor.h"
//  #include "myrobot_def.h"
/* 遥控拨杆与机器人模式定义-------------------------------------------------------------------*/

#define GYRO_MODE (rc_sensor.info->s2 == 3)
#define GYRO_CYCLE_MODE (rc_sensor.info->s2 == 1)
#define MEC_MODE (rc_sensor.info->s2 == 2)

/* 参数定义------------------------------------------------------------------*/

#define CAR_INIT_TIME (1000)
#define CYCLE_SPEED (5000.f) // 小陀螺速度
#define CHASSIS_MAX_SPEED (8000.f)
#define RC_MAX_CNT (660)
#endif
