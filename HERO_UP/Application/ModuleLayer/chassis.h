#ifndef _CHASSIS_H
#define _CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "communicate_protocol.h"
#include "vision_protocol.h"
#include "KT_module.h"
#include "motor.h"
#include "Command.h"
#include "myrobot_def.h"
/* define-----------------------------------------------------------------------*/
 
/* Exported macro ------------------------------------------------------------*/

typedef enum
{
	CHASSIS_SPEED_PID = 0,//闭速度环
	CHASSIS_POSITION_PID = 1,//闭位置环
}chassis_pid_mode_e;

/** 
  * @brief  底盘基本信息定义
  */ 
typedef __packed struct  
{
	int16_t target_front_speed;//目标前进速度
	int16_t target_right_speed;//目标右移速度
	int16_t target_cycle_speed;//目标旋转速度

}chassis_base_info_t;

/** 
  * @brief  底盘类定义
  */
typedef struct chassis_class_t
{	
	command_t *cmd_180;//转头信息
	command_t *cmd_auto_lob;//一键吊射
	command_t *cmd_normal_lob;//一键普通吊射
	command_t *cmd_oblique_lob;//一键普通吊射
	command_t *cmd_timer_mec_outpost;
	chassis_pid_mode_e pid_mode;//pid模式，位置环目标值在下主控更新

	chassis_base_info_t  base_info;
		
	void (*work)(struct chassis_class_t *chassis);

}chassis_t;

extern chassis_t chassis;
extern int16_t yaw_angle_err;
/* Exported functions --------------------------------------------------------*/
void Chassis_Work(chassis_t *chassis);  //总控


#endif
