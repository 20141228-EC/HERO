#ifndef __GIMBAL_H
#define __GIMBAL_H


/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "communicate.h"
//#include "myrobot_def.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "bmi.h"
#include "rc_sensor.h"
#include "Balance.h"
#include "DM_Motor.h"
#include "rp_device_config.h"
#include "rp_math.h"
#include "communicate.h"
#define YAW_MOTOR_ANGLE_MIDDLE 		(-3.13737416f)       //(1.57075f-0.f)  		  //YAW电机中值
#define GIMBAL_LOB_MEC_ANGEL	 (0.f)      //
#define GIMBAL_LOB_LOW_MEC_ANGEL	 (0.f)      // 

#ifdef PITCH_4310
	#define GIMBAL_MAX_MEC_ANGEL   		(0.60f)				//pitch机械角度电控限位最大值 990  0.60
	#define GIMBAL_MIN_MEC_ANGEL  		 (-0.27f)			//pitch机械角度电控限位最小值 -180  0.33

	#define GIMBAL_MAX_GYRO_ANGEL		(Board_Rx_Info.pitch_imu + (GIMBAL_MAX_MEC_ANGEL - Board_Rx_Info.pitch_mec) / (2*PI) * 360.f)
	//pitch陀螺仪角度电控限位最小值       
	#define GIMBAL_MIN_GYRO_ANGEL		(Board_Rx_Info.pitch_imu - (Board_Rx_Info.pitch_mec - GIMBAL_MIN_MEC_ANGEL) / (2*PI) * 360.f)
#else
	#define GIMBAL_MAX_MEC_ANGEL   		(905.f)				//pitch机械角度电控限位最大值 990
	#define GIMBAL_MIN_MEC_ANGEL  		 (-323.f)			//pitch机械角度电控限位最小值 -180

	#define GIMBAL_MAX_GYRO_ANGEL		(Board_Rx_Info.pitch_imu + (GIMBAL_MAX_MEC_ANGEL - Board_Rx_Info.pitch_mec) / 8192.f * 360.f)
	//pitch陀螺仪角度电控限位最小值       
	#define GIMBAL_MIN_GYRO_ANGEL		(Board_Rx_Info.pitch_imu - (Board_Rx_Info.pitch_mec - GIMBAL_MIN_MEC_ANGEL) / 8192.f * 360.f)
#endif

/*云台pid计算类型*/
typedef enum
{
	GYRO_PID,
	MEC_PID,
	SPEED_PID,
}gimbal_pid_mode_e;

typedef struct __attribute__((packed)) 
{
	uint8_t lob_init_angle_flag;//初始化吊射角度标志位,为了只初始化一次
	float lob_init_mec_yaw_angle;//机械角度 

	float pre_aim_yaw_angle;//吊射预瞄yaw陀螺角
 
	float gyro_init_lob_yaw_angle; //取吊射命令的那一刻的角度
	
	uint8_t last_into_oblique_lob_command_flag;//判断下降沿跳变
	uint16_t out_oblique_head_homing_timeout;
	
	uint8_t into_auto_lob_command_flag; //只有先进命令才能进lob更新，为了先进命令再进吊射更新,持续为1直到退出
	uint8_t into_normal_lob_command_flag;
}gimbal_lob_info_t;

/*视觉偏置*/
typedef struct __attribute__((packed)) 
{
	float vision_yaw_offset;
	float lob_yaw_mec_offset;
	float lob_yaw_gyro_offset;
	float lob_pitch_gyro_offset;
	float lob_pitch_mec_offset;
}gimbal_offset_info_t;

/*pitch控制类型*/
typedef struct __attribute__((packed))  
{
	int8_t gimbal_mode;   //吊射还是陀螺仪还是机械
}gimbal_mode_t;

typedef enum
{
	Gimbal_Turn_IDLE,
	Gimbal_Turn_Going,
	Gimbal_Turn_Num,
}Gimbal_Turn_e;


typedef struct
{
	float yaw_imu_angle;						//云台陀螺仪yaw轴角度
	float yaw_imu_speed;						//云台陀螺仪yaw轴速度 rad/s
	float  yaw_imu_angle_target;    //陀螺仪模式目标yaw   世界坐标系 (-180°~180°) (顺时针为正)
	float  yaw_motor_angle;         //yaw轴 相对底盘  角度(-32768~32768)      (顺时针为正)
	float  yaw_motor_speed;         //yaw轴 相对底盘  速度(dps)               (顺时针为正)
	float  yaw_mec_angle_target;	  //机械模式目标yaw		底盘坐标系(-180°~180°)
  float yaw_mec_360_angle;
	
	float  pitch_motor_angle;       //pitch轴 相对底盘   角度(0~16383)    (向上为正)
	float  pitch_motor_speed;       //pitch轴 相对底盘   速度(dps)           (向上为正)
	float  pitch_mec_angle_target;  //机械模式目标pitch	底盘坐标系	(0~16383)  (向上为正)
	float  pitch_imu_angle_target;  //陀螺仪模式目标pitch  世界坐标系 (-90°~90°)   (向上为正)

	int16_t  output_gimbal_y;				//yaw轴电机输出

	uint16_t init_time;//初始化时间
	uint16_t init_time_max;//初始化超时
	uint16_t init_time_max_count;//初始化超时计数
	
	Gimbal_Turn_e step;
	uint16_t turn_time;//换头时间
	uint16_t turn_time_max;//换头超时
	bool Gimbal_Turn_Finish;

}gimbal_base_info_t;

typedef struct gimbal_all
{
	Motor_DM_t  *gimbal_y;
	
	gimbal_base_info_t base_info;
  gimbal_pid_mode_e yaw_pid_mode;
	gimbal_lob_info_t		lob_info;
	gimbal_offset_info_t  	*offset_info;//偏置信息

	float            (*all_pid_calc)(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
	void        		 (*work)(struct gimbal_all *gimbal);

	Dev_Reset_State_e			 gimbal_reset_state; //云台初始化状态
	gimbal_mode_t          gimbal_ctrl_mode;

}gimbal_t;

extern gimbal_t gimbal;

void Gimbal_Work(gimbal_t *gimbal);


#endif
