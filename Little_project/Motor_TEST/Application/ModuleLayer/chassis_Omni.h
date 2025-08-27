#ifndef __CHASSIS_OMNI_H
#define __CHASSIS_OMNI_H

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
/*
	@brief：将遥控器的数据存在结构体里
*/
typedef struct rc_info_struct{
	int16_t rc_front;
	int16_t rc_right;
}rc_info_t;

/*
	@brief：全向轮解算结构体
*/

typedef struct v_Omni_struct{
	int16_t v_ture;
	int16_t v_CHAS_LF;
	int16_t v_CHAS_LB;
	int16_t v_CHAS_RF;
	int16_t v_CHAS_RB;
}v_Omni_t;

typedef struct Mecanum_mode_struct{
	void (*f_Mecanum_Translation)(void);
	void (*f_Mecanum_Spin)(void);
}Mecanum_mode_t;


/*------------private function----------------*/
void Omni_calculate(float k_speed);
void Omni_Start(void);

/* Exported variables --------------------------------------------------------*/
extern rc_info_t rc_info;
extern v_Omni_t v_Omni;
extern Mecanum_mode_t Mecanum_mode;
#endif
