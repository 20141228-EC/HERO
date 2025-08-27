#ifndef __GIMBAL_H
#define __GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "communicate_protocol.h"
#include "myrobot_def.h"
#include "Command.h"
#include "motor.h"
#include "bmi.h"

/* Exported macro ------------------------------------------------------------*/

#define YAW_MOTOR_ANGLE_MIDDLE 		(16481.f)  		  //YAW电机中值
#define PITCH_MOTOR_ENCODER_MIDDLE  (7048.f)    //pitch电机编码器中值

#define GIMBAL_LOB_MED_ANGEL	 (GIMBAL_MAX_MEC_ANGEL - 205.f)      //吊射机械角度  
#define GIMBAL_MAX_MEC_ANGEL   		(910.f)				//pitch机械角度电控限位最大值 
#define GIMBAL_MIN_MEC_ANGEL  		 (-500.f)			//pitch机械角度电控限位最小值 
//pitch陀螺仪角度电控限位最大值
#define GIMBAL_MAX_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle + (GIMBAL_MAX_MEC_ANGEL - gimbal->base_info.pitch_motor_angle) / 8192.f * 180.f)


//pitch陀螺仪角度电控限位最小值       
#define GIMBAL_MIN_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle - (gimbal->base_info.pitch_motor_angle - GIMBAL_MIN_MEC_ANGEL) / 8192.f * 180.f)

#define Yaw_Encoder_max              65535	//编码器最大值
#define LOB_YAW_OFFSET_STEP			 100  //吊射微调步幅
#define LOB_PITCH_OFFSET_STEP			 10  //吊射微调步幅

#define GIMBAL_MOTOR_ONLINE (gimbal.gimbal_p->work_state==DEV_ONLINE&&gimbal.gimbal_y->KT_motor_info.state_info.work_state)

/* Exported type/param -------------------------------------------------------*/
/**
 * @brief pid模式枚举
 * 
 */
typedef enum
{
	GYRO_PID,
	MEC_PID,
	SPEED_PID,
}gimbal_pid_mode_e;

/** 
  * @brief  云台基本信息定义
  */ 
typedef __packed struct  
{
	float yaw_imu_angle;						//云台陀螺仪yaw轴角度
	float yaw_imu_speed;						//云台陀螺仪yaw轴速度 rad/s
	float pitch_imu_angle;					//云台陀螺仪pitch轴角度 初始化后初值是0
	float pitch_imu_speed;          //云台陀螺仪pitch轴速度 rad/s
	
	float  yaw_imu_angle_target;    //陀螺仪模式目标yaw   世界坐标系 (-180°~180°) (顺时针为正)
	float  pitch_imu_angle_target;  //陀螺仪模式目标pitch  世界坐标系 (-90°~90°)   (向上为正)
	
	float  yaw_motor_angle;         //yaw轴 相对底盘  角度(-32768~32768)      (顺时针为正)
	float  yaw_motor_speed;         //yaw轴 相对底盘  速度(dps)               (顺时针为正)
	
	float  pitch_motor_angle;       //pitch轴 相对底盘   角度(0~16383)    (向上为正)
	float  pitch_motor_speed;       //pitch轴 相对底盘   速度(dps)           (向上为正)
	
	float  yaw_mec_angle_target;	  //机械模式目标yaw		底盘坐标系(-180°~180°)
	float  pitch_mec_angle_target;  //机械模式目标pitch	底盘坐标系	(0~16383)  (向上为正)
	
	int16_t  output_gimbal_y;				//yaw轴电机输出
	int16_t  output_gimbal_p;				//pitch轴电机输出

}gimbal_base_info_t;


/**
 * @brief视觉偏置
 * 
 */
typedef __packed struct 
{
	float vision_yaw_offset;
	float vision_pitch_offset;
	float lob_yaw_offset;
}gimbal_offset_info_t;

/**
 * @brief  吊射基本信息
 * 
 */
typedef __packed struct 
{
	float lob_init_angle;
	uint8_t lob_init_angle_flag;
 
}gimbal_lob_info_t;

/** 
  * @brief  云台类定义
  */ 
typedef struct gimbal_class_t
{
	rm_motor_t 		   *gimbal_p;
	KT_motor_t         *gimbal_y;
	command_t          *cmd_r90;
	command_t          *cmd_l90;
	command_t          *cmd_180;
	command_t          *cmd_auto_lob;
	command_t		   *cmd_up;
	command_t		   *cmd_dowm;
	command_t		   *cmd_right;
	command_t		   *cmd_left;
	
	
	gimbal_base_info_t   	base_info;
	gimbal_lob_info_t		lob_info;
	gimbal_offset_info_t  	*offset_info;//偏置信息
 
	
	float (*all_pid_calc)(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
	gimbal_pid_mode_e			ptich_pid_mode;//pitch轴pid模式
	gimbal_pid_mode_e			yaw_pid_mode;//yaw轴pid模式
	
	void              	  		 (*work)(struct gimbal_class_t *gimbal);
	Dev_Reset_State_e			 gimbal_reset_state; //云台初始化状态
}gimbal_t;

/* Exported type --------------------------------------------------------*/
extern gimbal_t gimbal;
/* Exported functions --------------------------------------------------------*/
void Gimbal_Work(gimbal_t *gimbal);
#endif
