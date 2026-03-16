#include "PID_Instance.h"

/* 腿长控制外环 */
pid_ctrl_t My_Link_Length_Pid[Leg_Num] = 
{
	[R_Leg] = ////////天上16 0.1 120
	{
	.kp = 18.f,//10.f,//     22.f,//   35.f,//30.f,//
    .ki = 0.f,//0.05f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 30.f,//
	},
	[L_Leg] = 
	{
	.kp = 18.f,//10.f,//    20.f,//   35.f,//30.f,//
    .ki = 0.f,//0.05f,
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 2.f,
    .out_max = 30.f,//
	},
};

//pid_ctrl_t My_Link_Length_Pid[Leg_Num] = 
//{
//	[R_Leg] = 
//	{
//	.kp = 7.f,//30.f,//
//    .ki = 0.07f,//0.05f,
//    .kd = 0.f,
//		.a = 1.f,
//    .integral_max = 2.f,
//    .out_max = 30.f,//
//	},
//	[L_Leg] = 
//	{
//	.kp = 7.f,//30.f,//
//    .ki = 0.07f,//0.05f,
//    .kd = 0.f,
//	.a = 1.f,
//    .integral_max = 2.f,
//    .out_max = 30.f,//
//	},
//};


/* 腿长控制内环 */

pid_ctrl_t My_Link_Length_Speed_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
	.kp =60.f,//130.f,  //85,//44.f,//120.f,//   80.f,//30.f,
    .ki = 0.f,//2.f,//0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 5.f,
    .out_max = 200,
	},
	[L_Leg] = 
	{
	.kp =60.f,//130.f, //90,//50,// 120.f,//   80.f,//30.f,
    .ki = 0.f,//2.f,//0.1f,
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 5.f,
    .out_max = 200,
	},
};

//pid_ctrl_t My_Link_Length_Speed_Pid[Leg_Num] = 
//{
//	[R_Leg] = 
//	{
//	.kp = 18.f,//10.f,//40
//    .ki = 0.f,//0.1f,
//    .kd = 0.f,
//		.a = 1.f,
//    .integral_max = 5.f,
//    .out_max = 25.f,//200
//	},
//	[L_Leg] = 
//	{
//	.kp = 18.f,//22.f,//40
//    .ki = 0.f,//o.1
//    .kd = 0.f,
//	.a = 1.f,
//    .integral_max = 5.f,
//    .out_max = 35.f,//200
//	},
//};


/* 单环Roll轴控制 */
pid_ctrl_t My_Link_Roll_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 0.5f,//100.f,//
    .ki = 0.f,//0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 1.f,
    .out_max = 1.f,//
	},
	[L_Leg] = 
	{
		.kp = 0.5f,//100.f,//5
    .ki = 0.f,//0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 1.f,
    .out_max = 1.f,//
	},
};


/* 外环yaw控制 */
pid_ctrl_t My_yaw_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 8.f,//6.f,//
    .ki = 0.f,//0.05f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 10.f,//
	},
	[L_Leg] = 
	{
		.kp = 8.f,//6.f,//5
    .ki = 0.f,//0.05f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 10.f,//
	},
};


/* 内环yaw控制 */
pid_ctrl_t My_yaw_speed_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 6.f,//4.f,//3.f,//1.f,//
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 30.f,//
	},
	[L_Leg] = 
	{
		.kp = 6.f,//4.f,//3.f,//1.f,//5
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 30.f,//
	},
};

/* 单环双腿协调控制 */

pid_ctrl_t My_Link_sync_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
	.kp = 30.f,//15.f,//7.f,//
    .ki = 0.1f,//0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 10.f,//10.f,//
	},
	[L_Leg] = 
	{
	.kp = 30.f,//15.f,//7.f,//
    .ki = 0.1f,//0.1f,
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 2.f,
    .out_max = 10.f,//10.f,//5
	},
};

/* vir_phi0控制内环 */
pid_ctrl_t My_Link_vir_phi0_speed_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
	.kp = 18.f,//3.f,//
    .ki = 0.f,//0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 0.f,//15.f,//5
	},
	[L_Leg] = 
	{
	.kp = 18.f,//3.f,//
    .ki = 0.f,//0.1f,
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 2.f,
    .out_max = 0.f,//15.f,//5
	},
};

/* vir_phi0控制外环 */
pid_ctrl_t My_Link_vir_phi0_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
	.kp = 9.f,//20.f,//7
    .ki = 0.15f,//0.1f,//0.1
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 5.f,//
	},
	[L_Leg] = 
	{
	.kp = 9.f,//20.f,//
    .ki = 0.15f,//0.1f,//0.1
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 2.f,
    .out_max = 5.f,//
	},
};

/* 单环vir_phi0d1控制 */
pid_ctrl_t My_Link_vir_phi0_d1_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 0.f,//1.f,//
    .ki = 0.1f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 2.f,//0
	},
	[L_Leg] = 
	{
		.kp = 0.f,//1.f,//5
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 2.f,//0
	},
};
	
/* 单环phi0d1控制 */
pid_ctrl_t My_Link_phi0_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 1.2f,//
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 0.f,
    .out_max = 15.f,//0
	},
	[L_Leg] = 
	{
		.kp = 1.2f,//5
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 0.f,
    .out_max = 15.f,//0
	},
};


Chassis_Pid_t chassis_PID={
	.roll_cal[R_Leg]=&My_Link_Roll_Pid[R_Leg],
	.roll_cal[L_Leg]=&My_Link_Roll_Pid[L_Leg],
	
	.sync_cal[R_Leg]=&My_Link_sync_Pid[R_Leg],
	.sync_cal[L_Leg]=&My_Link_sync_Pid[L_Leg],
	
	.length_cal[R_Leg]=&My_Link_Length_Pid[R_Leg],
	.length_cal[L_Leg]=&My_Link_Length_Pid[L_Leg],
	
	.length_speed_cal[R_Leg]=&My_Link_Length_Speed_Pid[R_Leg],
	.length_speed_cal[L_Leg]=&My_Link_Length_Speed_Pid[L_Leg],

	.yaw_cal[R_Leg]=&My_yaw_Pid[R_Leg],
	.yaw_cal[L_Leg]=&My_yaw_Pid[L_Leg],
	
	.yaw_speed_cal[R_Leg]=&My_yaw_speed_Pid[R_Leg],
	.yaw_speed_cal[L_Leg]=&My_yaw_speed_Pid[L_Leg],
	
	.vir_phi0_cal[R_Leg]=&My_Link_vir_phi0_Pid[R_Leg],
	.vir_phi0_cal[L_Leg]=&My_Link_vir_phi0_Pid[L_Leg],
	
	.vir_phi0_speed_cal[R_Leg]=&My_Link_vir_phi0_speed_Pid[R_Leg],
	.vir_phi0_speed_cal[L_Leg]=&My_Link_vir_phi0_speed_Pid[L_Leg],
	

	.vir_phi0d1_cal[R_Leg]=&My_Link_vir_phi0_d1_Pid[R_Leg],
	.vir_phi0d1_cal[L_Leg]=&My_Link_vir_phi0_d1_Pid[L_Leg],

  .phi0_cal[R_Leg] = &My_Link_phi0_Pid[R_Leg],
	.phi0_cal[L_Leg] = &My_Link_phi0_Pid[L_Leg],
};

