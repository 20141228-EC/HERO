#include "gimbal_motor.h"
#include "DM_Motor.h"
/*yawµç»ú*/
#ifdef UART_COMMUNICATE
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x01,
	
	.hcan = &hfdcan3,

};
#else
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x03,
	
	.hcan = &hfdcan2,

};
#endif

#ifdef YAW_4310
Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

pid_ctrl_t Yaw_Gyro_Ctrl_out = 
{
	.kp = 14.f,////15.f,//18.f,    //30.f,//
	.ki = 2.f,//0.1f,   //0.3f,//0.2f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 0.f,   //400.f,
};

pid_ctrl_t Yaw_Gyro_Ctrl_inn = 
{
	.kp = 0.06f,//0.08f,   //0.05f,   //0.15f,//0.35f,//0.75f,//
	.ki = 0.f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 0.f,
};

pid_ctrl_t Yaw_Lob_Ctrl_out = 
{
	.kp = 23.f,//20.f,  //30.f,//
	.ki = 2.f,//0.1f,   //0.3f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 0.f,   //400.f,
};

pid_ctrl_t Yaw_Lob_Ctrl_inn = 
{
	.kp = 0.07f,//0.05f,   //0.08f,//
	.ki = 0.f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 0.f,
};


Motor_DM_Ctrl_Info_t Yaw_Ctrl_t = 
{
	.angle_ctrl_inner = &Yaw_Gyro_Ctrl_inn,
	.angle_ctrl_outer = &Yaw_Gyro_Ctrl_out,
	.position_inn = &Yaw_Lob_Ctrl_inn,
	.position_out = &Yaw_Lob_Ctrl_out,
};

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.ctrl = &Yaw_Ctrl_t,
	
	.single_init = &DM_Single_Motor_Init,
	
	.type = dail_4310,

};

#else

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

pid_ctrl_t Yaw_Gyro_Ctrl_out = 
{
	.kp = 14.f,////15.f,//18.f,    //30.f,//
	.ki = 2.f,//0.1f,   //0.3f,//0.2f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 600.f,   //400.f,
};

pid_ctrl_t Yaw_Gyro_Ctrl_inn = 
{
	.kp = 0.06f,//0.08f,   //0.05f,   //0.15f,//0.35f,//0.75f,//
	.ki = 0.f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 12.f,
};

pid_ctrl_t Yaw_Lob_Ctrl_out = 
{
	.kp = 23.f,//20.f,  //30.f,//
	.ki = 2.f,//0.1f,   //0.3f,
	.kd = 0.f,
	.integral_max = 5.f,
	.out_max = 200.f,   //400.f,
};

pid_ctrl_t Yaw_Lob_Ctrl_inn = 
{
	.kp = 0.07f,//0.05f,   //0.08f,//
	.ki = 0.f,
	.kd = 0.f,
	.integral_max = 0.f,
	.out_max = 12.f,
};


Motor_DM_Ctrl_Info_t Yaw_Ctrl_t = 
{
	.angle_ctrl_inner = &Yaw_Gyro_Ctrl_inn,
	.angle_ctrl_outer = &Yaw_Gyro_Ctrl_out,
	.position_inn = &Yaw_Lob_Ctrl_inn,
	.position_out = &Yaw_Lob_Ctrl_out,
};

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.ctrl = &Yaw_Ctrl_t,
	
	.single_init = &DM_Single_Motor_Init,
	
	.type = yaw_6006,

};
#endif
