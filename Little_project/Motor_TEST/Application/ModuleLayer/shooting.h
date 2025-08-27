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
#define SHOOTING_MOTOR_ONLINE (rm_motor[FRIC_LF].work_state==DEV_ONLINE && rm_motor[FRIC_LB].work_state == DEV_ONLINE && \
								rm_motor[FRIC_RF].work_state == DEV_ONLINE && rm_motor[FRIC_RB].work_state == DEV_ONLINE \
								&& rm_motor[LIMIT].work_state == DEV_ONLINE && rm_motor[DAIL].work_state == DEV_ONLINE) //发射电机是否在线

#define SHOOTING_MOTOR_OFFLINE (rm_motor[FRIC_LF].work_state==DEV_OFFLINE || rm_motor[FRIC_LB].work_state == DEV_OFFLINE || \
								rm_motor[FRIC_RF].work_state == DEV_OFFLINE || rm_motor[FRIC_RB].work_state == DEV_OFFLINE \
								|| rm_motor[LIMIT].work_state == DEV_OFFLINE || rm_motor[DAIL].work_state == DEV_OFFLINE) 

//限位参数

#define LIMIT_POSIT_VALUE (90000.f) 		//限位单发拨弹位置增量（正）
#define LIMIT_SPEED       (5500.f)       	//正转速度  （正） 
#define LIMIT_Init_SPEED       (-5000.f)       	//限位初始化反转转速  （正）   
#define LIMIT_SHOOT_TIME		170            //限位一发转的时间
//拨盘参数
#define DAIL_REVERT_PISITION (500.f)  //拨盘反转要走的角度（反转为负）
#define DAIL_Init_SPEED (-1500.f)  		//拨盘初始化速度
#define DAIL_FEED_SPEED (3000.f)  		//拨盘供弹速度
#define DAIL_STUCK_SPEED (-2000.f)    	//拨盘反转处理卡弹的速度(反转为负)
#define DAIL_MAX_INIT_TIMES 300  		//拨盘最大初始化时间（判断弹仓是否为空）
#define DAIL_MAX_WORK_TIMES 300   	    //非初始化阶段拨盘最大工作时间（判断弹仓是否为空）77
#define DAIL_MAX_F_STUCK_CNT 2    		//拨盘正向堵转最大次数（判断是否供弹完成）
//单发供弹延迟，限位转完在供弹
//  #define CMD_FIRE_LOAD_DELAY_ENABLE

//视觉允许发弹时间容差，0为唯一时刻
#define VISION_SHOOT_TIMING_TOLERANCE 0 //视觉允许打蛋中断中算出可以打蛋的时间的容差
//弹速调整使能
#define FriSpeedAdaptEnabled
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
 * @brief 拨盘类信息
 * 
 */
typedef __packed struct 
{
	float        target_speed;     //拨盘目标速度
	int32_t      target_position;  //拨盘反转目标位置
	uint8_t      f_stucking_cnt;   //向前堵转次数
	uint8_t      b_stucking_cnt;   //向后堵转次数
	uint16_t     work_times;		//拨盘工作时间
	//uint8_t      first_shoot_flag;  //第一次波蛋时间延长，防止第一发因为被顶着到不了限位
	dail_state_e work_state;       //拨盘工作状态
	Dev_Reset_State_e dail_reset_state; //拨盘初始化状态
}dail_info_t;

/**
 * @brief 限位控制模式枚举
 * 
 */
typedef enum
{
	LIMIT_POSITION_MODE = 0,  //限位位置模式
	LIMIT_SPEED_MODE,  		  //限位速度模式
}limit_ctrl_mode_e;

/**
 * @brief 限位类信息
 * 
 */
typedef  __packed struct 
{
	float             target_speed;     //限位目标速度
	int32_t           target_position;  //限位目标位置
	limit_ctrl_mode_e ctrl_mode;		    //限位控制模式
}limit_info_t;


typedef __packed struct 
{
	float target_f_speed;     //前摩擦轮目标速度
	float target_b_speed;     //后摩擦轮目标速度

	int16_t friLF_speed;    //摩擦轮左前速度
	int16_t friRF_speed;    //摩擦轮右前速度
	int16_t friLB_speed;    //摩擦轮左后速度
	int16_t friRB_speed;    //摩擦轮右后速度

	int16_t friLF_minimum_speed;
	int16_t friRF_minimum_speed;
	int16_t friLB_minimum_speed;
	int16_t friRB_minimum_speed;

	int16_t fri_F_diff_speed;//前摩擦轮速度差，左减右
	int16_t fri_B_diff_speed;//后摩擦轮速度差，左减右

	bool    fri_speed_state;   //摩擦轮速度是否达到目标速度
}friction_info_t;

typedef __packed struct 
{
	float target_f_speed;      //前摩擦轮目标速度
	float target_b_speed;      //后摩擦轮目标速度
	float target_bullet_speed; //目标弹速
}shooting_config_t;


/** 
  * @brief  发射基本信息定义
  */ 
typedef __packed struct  
{
	int16_t    	    output_dail;    //拨盘输出
	int16_t			output_limit;   //限位输出
	int16_t         output_friLF;   //摩擦轮左前输出
	int16_t         output_friRF;   //摩擦轮右前输出
	int16_t         output_friLB;   //摩擦轮左后输出
	int16_t         output_friRB;   //摩擦轮右后输出

	dail_info_t     dail_info;      //拨盘信息
	limit_info_t    limit_info;     //限位信息
	friction_info_t fri_info;       //摩擦轮信息
}shooting_base_info_t;


/**
 * @brief 发射类信息
 * 
 */
typedef struct shooting_struct
{	
	/*注册电机*/
	rm_motor_t        				 *frictionLF;
	rm_motor_t        				 *frictionRF;
	rm_motor_t						 *frictionLB;
	rm_motor_t						 *frictionRB;
	rm_motor_t        				 *dail;
	rm_motor_t        				 *limit;
	
	command_t          		 *cmd_fire;
	command_t          		 *cmd_firing;
	command_t				 *cmd_kill_myself;

	shooting_base_info_t   base_info;	
	
	shooting_config_t *config;       //发射配置信息

	void               	   (*work)(struct shooting_struct *shooting); //声明这是shooting_struct结构体类型的指针
	
	shooting_state_e			shooting_state; //发射状态
	uint8_t					stop_shooting_flag;
	uint16_t							 shooting_init_time;//初始化时间
  shooting_load_state_e  load_state;  //弹丸装填状态  装好=load_OK
}shooting_t;


extern shooting_t shooting;
extern shooting_config_t config;


/* Exported functions --------------------------------------------------------*/
void Shooting_Fri_Speed_Adapt(shooting_t *shooting);  //弹速自适应
/*总控*/
void Shooting_Work(shooting_t *shooting);

#endif
