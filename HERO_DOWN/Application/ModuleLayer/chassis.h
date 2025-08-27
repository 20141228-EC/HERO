#ifndef _CHASSIS_H
#define _CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "communicate_protocol.h"
#include "judge_protocol.h"
#include "motor.h"
#include "myrobot_def.h"
/* define-----------------------------------------------------------------------*/
#define CHASSIS_POWER_LIMIT         (judge.judge_work_state == DEV_ONLINE)//功率限制 开：1 关：0
#define CHASSIS_MAX_SPEED           (communicate.chassis_data_rx_info->max_speed)

#define RC_ONLINE (communicate.status->chassis_data_state == DEV_ONLINE)
/* Exported macro ------------------------------------------------------------*/
 /**
 * @brief 底盘PID模式
 * 
 */
typedef enum
{
	CHASSIS_SPEED_PID = 0,//闭速度环
	CHASSIS_POSITION_PID = 1,//闭位置环
}chassis_pid_mode_e;

 /**
 * @brief 打滑优化结构体
 */
typedef __packed struct 
{
	float k_power;//1~2之间、越大后轮功率预测越大
	float max_speed_difference;
	float k_dynamic;
	int16_t slip_low_current;
	uint8_t slip_flag;
	
}chassis_slip_t;

/** 
  * @brief  底盘基本信息定义
  */ 
typedef __packed struct  
{
	
	int16_t target_front_speed;//目标前进速度
	int16_t target_right_speed;//目标右移速度
	int16_t target_cycle_speed;//目标旋转速度

	int16_t output_chassisLF;
	int16_t output_chassisLB;
	int16_t output_chassisRF;
	int16_t output_chassisRB;

	int16_t target_chassisLF_speed;
	int16_t target_chassisLB_speed;
	int16_t target_chassisRF_speed;
	int16_t target_chassisRB_speed;

	int32_t target_chassisLF_position;
	int32_t target_chassisLB_position;
	int32_t target_chassisRF_position;
	int32_t target_chassisRB_position;   
}chassis_base_info_t;


/** 
  * @brief  底盘类定义
  */ 
typedef struct chassis_class_t
{	
	rm_motor_t        		 *chassisLF;
	rm_motor_t        		 *chassisLB;
	rm_motor_t        		 *chassisRF;
	rm_motor_t        		 *chassisRB;

	chassis_base_info_t base_info;
	chassis_slip_t slip;
	chassis_pid_mode_e  pid_mode;
	uint16_t delay_run_cnt;
	uint8_t delay_run_flag;
	void                (*work)(struct chassis_class_t *chassis);
}chassis_t;

/* Exported typedef --------------------------------------------------------*/
extern chassis_t chassis;

/* Exported functions --------------------------------------------------------*/

//总控
void Chassis_Work(chassis_t *chassis);


#endif
