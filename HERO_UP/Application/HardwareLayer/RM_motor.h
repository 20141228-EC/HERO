/**
  ******************************************************************************
  * @file    RM_motor.h
  * @brief   RM电机驱动
  ******************************************************************************
  * @attention
  * 
  * 
  ******************************************************************************
  */
#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "pid.h"
#include "motor_def.h"
#include "drv_can.h"

/* Exported typedef ----------------------------------------------------------*/
/**
 *	@brief	RM标准电机信息与结构体
 *	@class	device
 */
typedef struct rm_motor_info_struct {
	
	volatile uint16_t	angle;
	volatile int16_t    speed;
	volatile int16_t	current;
    volatile uint8_t    temperature;
	volatile uint16_t	angle_prev;
	volatile int32_t	angle_sum;
	volatile uint8_t	init_flag;
	volatile uint8_t	offline_cnt;
	
	         uint8_t	offline_max_cnt;	
} rm_motor_info_t;

typedef struct
{
	int16_t		motor_out;
	
}rm_motor_base_info_t;


typedef struct rm_motor_struct {
	rm_motor_info_t 	        *info;
	drv_can_t				   	*driver;
	rm_motor_base_info_t 		base_info;
	motor_pid_all_t           	motor_all_pid;
	int16_t                     cnt;  //判断堵转
	bool                        flag;
	void					    (*init)(struct rm_motor_struct *self);
	void					    (*update)(struct rm_motor_struct *self, uint8_t *rxBuf);
	void					    (*check)(struct rm_motor_struct *self);	
	void					    (*heart_beat)(struct rm_motor_struct *self);
	motor_type_e     motor_type;    //电机类型
	volatile dev_work_state_t   work_state;
	volatile dev_errno_t	    errno;
} rm_motor_t;

/* Exported functions --------------------------------------------------------*/
void Motor_SendData(rm_motor_t *motor, int16_t data);
void Motor_ToSpeed(rm_motor_t *motor, int16_t speed);
float rm_motor_speed_pid_calc(rm_motor_t *motor, int16_t speed);
void Motor_ToAngle(rm_motor_t *motor, int16_t angle);
void Motor_ToAxleAngle(rm_motor_t *motor, int32_t angle);
float rm_motor_anglesum_pid_calc(rm_motor_t *motor, int32_t angle);
int8_t Motor_DetectStuck(rm_motor_t *motor);
int8_t Motor_DetectStuck_userdef(rm_motor_t *motor,uint16_t speed,uint16_t current,uint16_t time_ms,int8_t force_direction);
void Motor_HandleStuck(rm_motor_t *motor);
uint8_t Motor_InPosition(rm_motor_t *motor);
void rm_motor_init(rm_motor_t *motor);
void rm_motor_heart_beat(rm_motor_t *motor);
 

#endif

