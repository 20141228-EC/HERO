#ifndef __BALANCE_H
#define __BALANCE_H

#include "rc_sensor.h"
#include "chassis.h"

#include "Command_Instance.h"
#define BALANCE_INIT_CNT_MAX 16000  //自己加上的

typedef enum
{
	RC_CTRL = 0,
	KEY_CTRL,
}Balance_Ctrl_e;
typedef enum
{
	Balance_reset_NO,
	Balance_reset_OK,
	
}Balance_reset_state_e;

typedef struct Balance_reset_state_struct_t
{
	Balance_reset_state_e reset_state;
	uint16_t reset_cnt;
}Balance_reset_state_t;

typedef enum
{
	Init_Mode = 0,
	Sleep_Mode,
	Imu_Mode,
	Mec_Mode,
	Lob_Mode,
	Cycle_Mode,
	Vary_Cycle_Mode,
	LEG_TEST_Mode,
	Handle_Mode,
	Rescue_Mode,
}Balance_Mode_e;

typedef struct Balance_Flag_struct_t
{
	bool Clear_Flag;
	bool Chassis_Online_Flag;
	bool Chassis_Sleep_Flag;
	bool Knee_Strike_1_Flag;
	bool Knee_Strike_2_Flag;
	bool Op_Fly_Flag;
	bool Cycle_Flag;
	bool Turn_Flag;
	bool Fly_Flag;
	bool Rescue_Flag;
	bool Rescue_Trigger;
	bool Unable_Rescue_Flag;//无法自救
	bool Leg_length_ctrl_Flag;
	bool Jumping_Flag;//跳跃过程中，用于给chassis状态信号量
	bool Shoot_Flag;
  bool Remedy_Flag;
	
	uint8_t Rescue_step;
	
}Balance_Flag_t;

typedef struct Balance_Remote_Ctrl_struct_t
{
	rc_sensor_t* sensor;
	uint8_t* last_thumbwheel_step;
}Balance_Remote_Ctrl;

typedef struct Launch_Command_struct_t
{
	bool Shoot_Online_Flag;
	
	bool Fric_On_Flag;//开摩擦轮
	
	bool Single_Shoot_Flag;//单发
	
	bool Keep_Shoot_Flag;//连发
	
	bool Auto_Shoot_Flag;//自瞄，自动打
	
	bool no_heat_limit_flag;//解除热量限制
}Shoot_Flag_t;

typedef struct Vision_Command_struct_t
{
	bool Vision_Online_Flag;
	
	bool Auto_Catch_Flag;//自瞄
	
	bool Auto_Catch_Engi_Flag;//自动锁定工程
	
	bool Auto_Base_Flag;//
}Vision_Flag_t;

typedef struct Chassis_Command_struct_t
{
	bool Chassis_Online_Flag;
	
	bool Top_Flag;//小陀螺模式
	
	bool Vary_Speed_Flag;//变速陀螺模式
	
	uint16_t top_num;
	
	bool Jump_Flag;
	
	bool Fly_Flag;//飞坡模式
	
	bool Mid_Length_Flag;
	
	bool Knee_Strike_Flag;
	
	bool Power_Limit_Flag;//功率限制标志位
	
	bool Over_Power_Flag;//极端情况功率超载标志位
	
	bool Chassis_Mid_Flag;//底盘归中
	
	bool SideWay_Right_Flag;//右侧身模式
	
	bool SideWay_Left_Flag;//左侧身模式
	
	bool Save_Success_Flag;//是否翻车
	
	bool Save_Finish_Flag_1;
	
	bool Save_Finish_Flag_2;
	
	uint8_t save_type;//确认自救类型
}Chassis_Command_t;


typedef struct Balance_struct_t
{
	Balance_Ctrl_e ctrl;
	
	Balance_Mode_e mode;
	
	Balance_Mode_e last_mode;
	
	Balance_Remote_Ctrl* rc;
	
	Balance_reset_state_t reset_struct;
	
	Balance_Flag_t* Flag;
	
	Chassis_Command_t *Chassis_Com;
	
	command_t* command;
	
	Shoot_Flag_t Shoot;
	
	Vision_Flag_t Vision;
	
	void(*init)(struct Balance_struct_t* balance);
	
	void(*update)(struct Balance_struct_t* balance);
}Balance_t;

extern Balance_t Balance;


#endif
