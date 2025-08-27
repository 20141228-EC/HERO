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
#if HERO_TYPE ==3
#define YAW_MOTOR_ANGLE_MIDDLE 		(17463.f)  		  //YAW电机中值
#define PITCH_MOTOR_ENCODER_MIDDLE  (2950.f)    //pitch电机编码器中值
#define GIMBAL_LOB_MEC_ANGEL	 (628.f)      //吊射机械角度 15.6弹速606
#define GIMBAL_LOB_LOW_MEC_ANGEL	 (536.f)      //吊射底部机械角度  
#define GIMBAL_MAX_MEC_ANGEL   		(990.f)				//pitch机械角度电控限位最大值 
#define GIMBAL_MIN_MEC_ANGEL  		 (-180.f)			//pitch机械角度电控限位最小值 
//三模
#elif HERO_TYPE ==2
#define YAW_MOTOR_ANGLE_MIDDLE 		(25019.f)  		  //YAW电机中值
#define PITCH_MOTOR_ENCODER_MIDDLE  (7969.f)    //pitch电机编码器中值
#define GIMBAL_LOB_MEC_ANGEL	   (613.f)     //吊射顶部机械角度
#define GIMBAL_LOB_LOW_MEC_ANGEL	  (550.f)      //吊射底部机械角度  

#define GIMBAL_TOP_LOB_MEC_ANGEL	   (820.f)   //梯高吊射机械角度 
#define GIMBAL_TOP_LOB_LOW_MEC_ANGEL	(780.f)      //梯高吊射底部机械角度  
#define GIMBAL_MAX_MEC_ANGEL   		(880.f)				//pitch机械角度电控限位最大值 
#define GIMBAL_MIN_MEC_ANGEL  		 (-10.f)			//pitch机械角度电控限位最小值 

#elif HERO_TYPE ==1
#define YAW_MOTOR_ANGLE_MIDDLE 		(30747.f)  		  //YAW电机中值
#define PITCH_MOTOR_ENCODER_MIDDLE  (4342.f)    //pitch电机编码器中值
#define GIMBAL_LOB_MEC_ANGEL	 (732.f)      //吊射机械角度 
#define GIMBAL_LOB_LOW_MEC_ANGEL	 (500.f)      //吊射底部机械角度  
#define GIMBAL_MAX_MEC_ANGEL   		(940.f)				//pitch机械角度电控限位最大值 
#define GIMBAL_MIN_MEC_ANGEL  		 (-450.f)			//pitch机械角度电控限位最小值 
#endif

#define GIMBAL_MAX_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle + (GIMBAL_MAX_MEC_ANGEL - gimbal->base_info.pitch_motor_angle) / 8192.f * 360.f)
//pitch陀螺仪角度电控限位最小值       
#define GIMBAL_MIN_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle - (gimbal->base_info.pitch_motor_angle - GIMBAL_MIN_MEC_ANGEL) / 8192.f * 360.f)

#define PITCH_ONE_DEGREE_VALUE		 22.75
#define YAW_ONE_DEGREE_VALUE		 182.04
#define Yaw_Encoder_max              65535	//编码器最大值
#define LOB_YAW_OFFSET_STEP			 25  //吊射微调步幅
#define LOB_PITCH_OFFSET_STEP		 3  //吊射微调步幅

#define GIMBAL_MOTOR_ONLINE (gimbal.gimbal_p->work_state==DEV_ONLINE&&gimbal.gimbal_y->KT_motor_info.state_info.work_state==M_ONLINE)

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
	float  yaw_imu_angle_target;    //陀螺仪模式目标yaw   世界坐标系 (-180°~180°) (顺时针为正)
	float  yaw_motor_angle;         //yaw轴 相对底盘  角度(-32768~32768)      (顺时针为正)
	float  yaw_motor_speed;         //yaw轴 相对底盘  速度(dps)               (顺时针为正)
	float  yaw_mec_angle_target;	  //机械模式目标yaw		底盘坐标系(-180°~180°)
	
	float pitch_imu_angle;					//云台陀螺仪pitch轴角度 初始化后初值是0
	float pitch_imu_speed;          //云台陀螺仪pitch轴速度 rad/s
	float  pitch_imu_angle_target;  //陀螺仪模式目标pitch  世界坐标系 (-90°~90°)   (向上为正)
	float  pitch_motor_angle;       //pitch轴 相对底盘   角度(0~16383)    (向上为正)
	float  pitch_motor_speed;       //pitch轴 相对底盘   速度(dps)           (向上为正)
	float  pitch_mec_angle_target;  //机械模式目标pitch	底盘坐标系	(0~16383)  (向上为正)
	
	float yaw_mec_360_angle;
	float pitch_mec_360_angle;
	
	int16_t  output_gimbal_y;				//yaw轴电机输出
	int16_t  output_gimbal_p;				//pitch轴电机输出

}gimbal_base_info_t;

/** 
  * @brief  软件pitch重力补偿结构体
  * @note  mgL*sin(torque_angle)
  */ 
typedef __packed struct  
{
	float const_k;//mgL
	float center_of_gravity_angle;//水平 与 转轴到整头重心连线的夹角
	float torque_angle;//转矩角 
	float gravity_offset_output;

}gimbal_gravity_offset_info_t;

/**
 * @brief视觉偏置
 * 
 */
typedef __packed struct 
{
	float vision_yaw_offset;
	float vision_pitch_offset;
	float lob_yaw_mec_offset;
	float lob_yaw_gyro_offset;
	float lob_pitch_gyro_offset;
}gimbal_offset_info_t;

/**
 * @brief  吊射/预瞄基本信息
 * 
 */
typedef __packed struct 
{
	uint8_t lob_init_angle_flag;//初始化吊射角度标志位,为了只初始化一次
	float lob_init_mec_yaw_angle;//机械角度 

	float pre_aim_yaw_angle;//吊射预瞄yaw陀螺角
	float pre_aim_pitch_angle;//吊射预瞄pitch陀螺角
 
	float gyro_init_lob_yaw_angle; //取吊射命令的那一刻的角度
	float gyro_init_lob_pitch_angle;
	
	uint8_t last_into_oblique_lob_command_flag;//判断下降沿跳变
	uint8_t out_oblique_head_homing_flag;//退斜着吊射头归位，下降沿跳变后变1，直到头归位变0
	uint16_t out_oblique_head_homing_timeout;
	
	uint8_t lob_pitch_type;//0在香蕉道，1在梯高
	
	uint8_t into_auto_lob_command_flag; //只有先进命令才能进lob更新，为了先进命令再进吊射更新,持续为1直到退出
	uint8_t into_normal_lob_command_flag;
	uint8_t into_outpost_lob_command_flag;
	uint8_t into_oblique_lob_command_flag;
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
	command_t          *cmd_timer_mec_outpost;
	command_t          *cmd_auto_lob;
	command_t          *cmd_normal_lob;
	command_t          *cmd_oblique_lob;
	command_t		   *cmd_up;
	command_t		   *cmd_dowm;
	command_t		   *cmd_right;
	command_t		   *cmd_left;
	command_t		   *cmd_change_lob_pitch;
	gimbal_gravity_offset_info_t gravity_offset_info;
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
