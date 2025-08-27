#ifndef _SHOOTING_H
#define _SHOOTING_H

/* Includes ------------------------------------------------------------------*/
#include "RM_motor.h"
#include "motor.h"
#include "motor_def.h"
#include "rp_config.h"
#include "rp_device_config.h"
#include "Command.h"
#include "car.h"
#include "myrobot_def.h"
/* Private def ------------------------------------------------------------------*/

#if HERO_TYPE ==3
#define SHOOTING_MOTOR_ONLINE (rm_motor[FRIC_B_L].work_state==DEV_ONLINE && rm_motor[FRIC_B_R].work_state == DEV_ONLINE && \
								rm_motor[FRIC_B_UP].work_state == DEV_ONLINE \
								&& rm_motor[FRIC_F_UP].work_state == DEV_ONLINE \
								&& rm_motor[FRIC_F_L].work_state == DEV_ONLINE && rm_motor[FRIC_F_R].work_state == DEV_ONLINE \
								&& rm_motor[DAIL].work_state == DEV_ONLINE) //发射电机是否在线
#else 
#define SHOOTING_MOTOR_ONLINE (rm_motor[FRIC_B_L].work_state==DEV_ONLINE && rm_motor[FRIC_B_R].work_state == DEV_ONLINE && \
								rm_motor[FRIC_B_UP].work_state == DEV_ONLINE \
								&& rm_motor[DAIL].work_state == DEV_ONLINE) //发射电机是否在线
#endif

//发射参数
#define FIRING_PERIOD (833)  //连发周期，单位ms
//拨盘参数
//如果偏大会往拨的方向偏移
#define DAIL_ONESHOT_ANGLE    (31481) //拨盘角度环单发一发要走的角度（正）
#define DAIL_REVERT_PISITION (-31481)  //拨盘反转要走的角度（反转为负)

#if HERO_TYPE ==2
#define DAIL_INIT_ANGLE (-2000)  //拨盘初始化补偿角度
#else
#define DAIL_INIT_ANGLE (17000)  //拨盘初始化补偿角度
#endif

#define DAIL_Init_SPEED (-1000.f)  		//拨盘初始化速度(反转为负)
#define DAIL_RELOAD_SPEED (3000.f)  	//拨盘供弹速度
#define DAIL_STUCK_SPEED (-2000.f)    	//拨盘反转处理卡弹的速度(反转为负)
#define DAIL_SLOW_SHOOT_TIMES 600  		//热量剩余少的时候拨慢一点
#define DAIL_FAST_SHOOT_TIMES 300   	    //热量剩余多的时候拨快一点
#define DAIL_MAX_F_STUCK_CNT 2    		//拨盘正向堵转最大次数（判断是否供弹完成）
//摩擦轮参数
#define FRIC_HANDLE_STUCK_ONETIME (600)  //摩擦轮处理一次堵转的时间
#define FRIC_HANDLE_STUCK_SPEED (-1000)  //摩擦轮处理堵转的速度(反转为负)
//视觉允许发弹时间容差，0为唯一时刻
#define VISION_SHOOT_TIMING_TOLERANCE 0 //视觉允许打蛋中断中算出可以打蛋的时间的容差

//#define Z_CHANGE_FRIC_SPEED
/* Exported variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/**
 * @brief 装填状态枚举
 * 
 */
typedef enum 
{
	load_NO = 0,     		//需要装弹
	load_OK,		//装填完毕	 

}shooting_load_state_e;

/**
 * @brief 拨盘状态枚举
 * 
 */
typedef enum
{
	DAIL_SLEEP = 0, //拨盘休眠等待指令
	DAIL_RELOAD,    //拨盘正转补弹丸
	DAIL_REVERT,    //拨盘反转

}dail_state_e;

/**
 * @brief 发射状态
 * 
 */
typedef enum
{
	SHOOTING_RESET_NO, //发射需要初始化
	SHOOTING_INITING,	 //发射正在初始化
	SHOOTING_RESET_OK, //发射初始化完成
}shooting_state_e;

/**
 * @brief 发射最大抖动角度结构体
 * 
 */
typedef __packed struct 
{
	float yaw_shake_angle ;		//yaw抖动角度
	float pitch_shake_angle;	//pitch抖动角度
	float const_offset_current; //前馈常数补偿电流
	float shoot_pitch_offset_current;//实际输出补偿电流
	float pitch_a;				//pitch电机角加速度，用来计算补偿电流
	float kd;					//发射时pitch电机pid的kd系数
	uint16_t feedforward_delay_time;//ms
	uint16_t feedforward_continue_time;//ms
	uint8_t feedforward_current_flag;
}shooting_shake_angle_t;
 

/**
 * @brief 拨盘控制模式枚举
 */
typedef enum
{
	DAIL_SPEED = 0,  		  //拨盘速度模式
	DAIL_ANGLE ,  //拨盘位置模式
	
}dail_ctrl_mode_e;

/**
 * @brief 拨盘类信息
 * 
 */
typedef __packed struct 
{
	float        	  target_speed;     //拨盘目标速度
	int32_t      	  target_angle_sum;  //拨盘目标位置
	uint16_t     	  work_times;		//拨盘工作时间
	bool			  stuck_flag;//堵转标志
	dail_ctrl_mode_e  dail_mode;   //拨盘控制方式，角度环还是速度环
	dail_state_e      work_state;       //拨盘工作状态
	Dev_Reset_State_e dail_reset_state; //拨盘初始化状态
}dail_info_t;


typedef __packed struct 
{
	bool    fri_speed_state;   //摩擦轮速度是否达到目标速度
	uint16_t fri_handle_stuck_time;//堵转处理计时
	uint8_t fri_handle_stuck_cnt;//堵转处理次数
	uint8_t fri_handle_stuck_flag;//堵转标志，处理中为1，处理完成为0
	//目标速度
	int16_t target_fri_F_UP_speed;   //目标第二级上摩擦轮速度
	int16_t target_fri_F_L_speed;    //目标第二级左摩擦轮速度
	int16_t target_fri_F_R_speed;    //目标第二级右摩擦轮速度
	
	int16_t target_fri_B_UP_speed;   //目标第一级上摩擦轮速度
	int16_t target_fri_B_L_speed;    //目标第一级左摩擦轮速度
	int16_t target_fri_B_R_speed;    //目标第一级右摩擦轮速度
    //摩擦轮真实速度
	int16_t fri_F_L_real_speed;     
	int16_t fri_F_R_real_speed;     
	int16_t fri_F_UP_real_speed;
	int16_t fri_B_L_real_speed;     
	int16_t fri_B_R_real_speed;     
	int16_t fri_B_UP_real_speed;
	//发射最小速度
	int16_t fri_F_L_minimum_speed;     
	int16_t fri_F_R_minimum_speed;     
	int16_t fri_F_UP_minimum_speed;

	int16_t fri_B_L_minimum_speed;
	int16_t fri_B_R_minimum_speed;
	int16_t fri_B_UP_minimum_speed;

	
}friction_info_t;

typedef __packed struct 
{
	float target_F_friction_speed;      //第二级摩擦轮目标速度
	float target_B_friction_speed;      //第一级摩擦轮目标速度
	float target_bullet_speed; 	//目标弹速
}shooting_config_t;


/** 
  * @brief  发射基本信息定义
  */ 
typedef __packed struct  
{
	int16_t    	    output_dail;      //拨盘输出
	int16_t         output_fri_F_UP;  //第二级上摩擦输出
	int16_t         output_fri_F_L;   //第二级左摩擦轮输出
	int16_t         output_fri_F_R;   //第二级右摩擦轮输出
	int16_t         output_fri_B_UP;  //第一级上摩擦输出
	int16_t         output_fri_B_L;   //第一级左摩擦轮输出
	int16_t         output_fri_B_R;   //第一级右摩擦轮输出

	dail_info_t     dail_info;      //拨盘信息
	friction_info_t fri_info;       //摩擦轮信息
}shooting_base_info_t;


/**
 * @brief 发射类信息
 * 
 */
typedef struct shooting_struct
{	
	/*注册电机*/
	rm_motor_t        				 *friction_F_L;
	rm_motor_t        				 *friction_F_R;
	rm_motor_t						 *friction_F_UP;
	rm_motor_t        				 *friction_B_L;
	rm_motor_t        				 *friction_B_R;
	rm_motor_t						 *friction_B_UP;
	rm_motor_t        				 *dail;
	command_t        		 *cmd_speed_adapt;
	command_t          		 *cmd_fire;
	command_t          		 *cmd_firing;
	command_t				 *cmd_kill_myself;
	command_t      		    *cmd_timer_mec_outpost;
	shooting_base_info_t   base_info;	
	
	shooting_config_t 		 *config;       //发射配置信息
	shooting_shake_angle_t   shooting_shake_angle;
	void               	    (*work)(struct shooting_struct *shooting);  
	
	shooting_state_e			shooting_state; //发射状态
	uint16_t					shooting_init_time;//初始化时间
    shooting_load_state_e  load_state;  //弹丸装填状态  装好=load_OK
	
}shooting_t;


extern shooting_t shooting;
extern shooting_config_t config;


/* Exported functions --------------------------------------------------------*/
void Shooting_Fri_Speed_Adapt(shooting_t *shooting);  //弹速自适应
void Shoot_pitch_shake_restrict(shooting_t *shooting);//发射pitch抖动补偿
/*总控*/
void Shooting_Work(shooting_t *shooting);

#endif
