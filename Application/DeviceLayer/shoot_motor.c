#include "shoot_motor.h"
/*²¦ÅÌµç»ú*/
#ifdef UART_COMMUNICATE
Motor_DM_Born_Info_t Dail_Born_Info =
{
	.stdId = 0x08,
	
	.hcan = &hfdcan3,

};
#else
Motor_DM_Born_Info_t Dail_Born_Info =
{
	.stdId = 0x08,
	
	.hcan = &hfdcan2,

};
#endif

Motor_DM_Rx_Info_t Dail_Rx_Info_t;

Motor_DM_Tx_Info_t Dail_Tx_Info_t;

Motor_DM_State_t Dail_State_t;

pid_ctrl_t Dail_Speed_Ctrl = 
{
	.kp = 0.8f,//
	.ki = 0.1f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 12.5f,
};

pid_ctrl_t Dail_Pos_Ctrl_out = 
{
	.kp = 70.f,//
	.ki = 0.1f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 30.f,
};

pid_ctrl_t Dail_Pos_Ctrl_inn = 
{
	.kp = 0.6f,//2.5f,//
	.ki = 0.f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 12.5f,
};


Motor_DM_Ctrl_Info_t Dail_Ctrl_t = 
{
	.speed_ctrl = &Dail_Speed_Ctrl,
	.position_inn = &Dail_Pos_Ctrl_inn,
	.position_out = &Dail_Pos_Ctrl_out,
};

Motor_DM_t Dail_Motor = 
{
	.born_info = &Dail_Born_Info,
	
	.rx_info = &Dail_Rx_Info_t,
	
	.tx_info = &Dail_Tx_Info_t,
	
	.state = &Dail_State_t,
	
	.ctrl = &Dail_Ctrl_t,
	
	.single_init = &DM_Single_Motor_Init,
	
	.type = dail_4310,

};