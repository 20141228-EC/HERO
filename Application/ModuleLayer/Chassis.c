/**
  ******************************************************************************
  * File Name          : Chassis.c
  * Description        : Code for Chassis applications
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 SZU RobotPilots.
  * @version
  * V1.0 October 2025 
  * @author
  * Liang 741427745@qq.com
  ******************************************************************************
  * 
  ==============================================================================
                       ##### How To Use #####
  ==============================================================================
  (#) 调用软件层初始化函数 Chassis_Init

  (#) 调用底盘电机心跳函数 Chassis_HeartBeat
  
  (#) 调用 Chassis_Status_React ，根据车模式来响应底盘状态，依赖Balance文件
		
  (#) 调用 Chassis_Data_Update ，包含整车状态更新和获取遥控器数据的更新
  
  (#) 调用 Chassis_Ctrl ，根据底盘状态来进行相应控制
  
  ==============================================================================
                      ##### How To Input Data #####
  ============================================================================== 
  (#)   车体方向定义：从车的前进方向的右边来观测连杆；无论是左腿和右腿，观测方向都不变。
		例如有一个人站在你面前，你走到他的右手边来观测他的手的运动。
  (#)   以下为正方向：
  (#)	基于HGC模型:
		①https://zhuanlan.zhihu.com/p/563048952
		②https://zhuanlan.zhihu.com/p/613007726
		
  (#) Chassis_Posture 里输出的数据方向应该：
		以你的头为本体，来示意方向
		pitch	方向：抬头为正
		roll	方向：头往右边歪为正
		yaw 	方向：头往左扭为正
		
  (#) Chassis_Data_Update 里需要保证以下数据输入正确：
		phi1 						 单位：rad  ，方向：逆时针,零点：水平向左
		phi4 						 单位：rad  ，方向：逆时针，零点：水平向左
		phi1_d1 					 单位：rad/s  ，方向：逆时针
		phi4_d1 					 单位：rad/s  ，方向：逆时针
		torque_phi1_mea				 单位：N*m ，方向逆时针
		torque_phi4_mea				 单位：N*m ，方向逆时针
		
		
  (#) Chassis_State_Var_Update 里需要保证以下数据输入正确：
		vir_phi0 					 单位：rad  ，方向：逆时针 ，零点：水平向下
		pitch 						 单位：rad  ，方向：逆时针 ，零点：水平向右
		vir_phi0_d1 				 单位：rad/s  ，方向：逆时针
		pitch_d1 					 单位：rad/s  ，方向：逆时针
		L_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
		R_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
		L_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
		R_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
		l0							 单位：m  ，方向：伸腿增大
		α							 -vir_phi0
		
		计算出来的变量方向：
		thetal				 		 单位：rad ，方向:顺时针，零点：水平向下
		thetal_d1					 单位：rad/s ，方向:顺时针
		stator_bias			    	 伸腿时相当于轮子后退（逆时针），具体请看Stator_Correction_Cal
		L_Wheel_speed_Transformed      单位：rad 方向：车前进时为正（顺时针）                                                                                                                                     
		R_Wheel_speed_Transformed	   单位：rad 方向：车前进时为正（顺时针）
		L_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）                                                                                                                                                    
		R_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）
		
	(#) Chassis_Target_Update 这里的方向自己定
	
	==============================================================================
								##### Quick Start #####
	============================================================================== 
  (#)	更改car_info的参数，使其符合车体建模
  
  (#)	连上关节电机，检查五连杆解算，l0，vir_phi0，及其导数是否正确
	
  (#)   连上轮毂电机，检查speed，anglesum是否正确
	
  (#)   连上板子，检查姿态输入是否正确
  
  (#)   检查s，sd1状态变量是否正确，检查状态变量误差是否正确，检查LQR能否正常输出Tp,Tw
  
  (#)   在Balance文件定义的 LEG_TEST_Mode 下检查 l0，phi0控制
 
  (#)	姿态差自动停止控制
  
  (#)	检查重力前馈
  
  (#)   定腿长调k矩阵，实现平衡，仅保留LQR，重力前馈。初始目标腿长在car_info里，k矩阵K_MATRIX_COEFFICIENT
	
  (#)	离地检测，仅保持thetal
	
  (#)	前进后退，旋转；需要测试双腿协调sync
  
  (#)	roll轴控制
  
  (#)	检查是否能防打滑
  
  (#)	根据腿长拟合k矩阵，实现平衡
  
  (#)	磕膝上台阶                                                               
  
  (#)	跳跃  


	==============================================================================
								##### Complete not yet #####
	============================================================================== 
	
	(#)	功率限制未写
  
	(#) 氮气弹簧前馈未写
	
	(#)	串腿自救未试过，未完善
	
	(#)	跳跃未充分验证，仅供参考
	
	(#) 撞膝上台阶未充分验证，仅供参考
	
	
  */

#include "Chassis.h"
#include "Flag_Handle.h"
#include "Filter.h"
#include "tim.h"

void Chassis_Init(Chassis_t* My_Chassis);
static void Chassis_HeartBeat(Chassis_t* My_Chassis);
/*任务调用函数 begin*/
static void Chassis_Data_Update(Chassis_t* My_Chassis);
static void Chassis_Status_React(Chassis_t *My_Chassis);
static void Chassis_Ctrl(Chassis_t *My_Chassis);
/*任务调用函数 end*/
static void Chassis_Work(Chassis_t* My_Chassis);

static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor);
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor);
static void Chassis_State_Var_Update(Chassis_t* My_Chassis);
static void Stator_Correction_Cal(Chassis_t *My_Chassis);
static void Chassis_Offline_Process(Chassis_t* My_Chassis);

/* 目标设置函数begin */
static void Chassis_Rc_Input_Update(Chassis_t* My_Chassis);
static void Chassis_Key_Input_Update(Chassis_t* My_Chassis);
static void Chassis_sd1_Target_Update(Chassis_t* My_Chassis);
static void Chassis_Yaw_Target_Process_All(Chassis_t* My_Chassis);
static void Chassis_Leg_Length_Target_Process(Chassis_t* My_Chassis);
static void Chassis_Speed_Limit(Chassis_t* My_Chassis);
static void Chassis_Target_Update(Chassis_t* My_Chassis);
/* 目标设置函数end */

/*腿长相关力 begin*/
static void Chassis_Leg_Length_Strength_Cal(Chassis_t* My_Chassis);
static void Chassis_Roll_Control(Chassis_t* My_Chassis);
static void Chassis_Link_Feedforward_Cal(Chassis_t* My_Chassis);
static void Chassis_Leg_Fbl_Cal(Chassis_t* My_Chassis);

/*腿长相关力 end*/
//双腿协调
static void Chassis_Leg_Sync_Cal(Chassis_t* My_Chassis);
//用于自救
static void Chassis_Leg_vir_phi0_Cal(Chassis_t* My_Chassis);
static void Chassis_Leg_phi0d1_Cal(Chassis_t* My_Chassis);
static void Clean_Process(Chassis_t* My_Chassis);
static void Rescue_Target_Process(Chassis_t* My_Chassis);


static void Chassis_Wheel_Turn_Cal(Chassis_t* My_Chassis);

/*计算chassis所有电机力矩 end*/
static void Chassis_Torque_Cal(Chassis_t *My_Chassis);
/*计算chassis所有电机力矩 end*/

//由控制层到电机层，将计算出来的扭矩赋给相应电机
static void Chassis_Set_Torque(Chassis_t* My_Chassis);
//电机层目标扭矩设0
static void Chassis_Motor_Set_Sleep(Chassis_t* My_Chassis);
//关控阻尼卸力，保护机械限位
static void Chassis_Stop_Damping(Chassis_t* My_Chassis);
//离地检测
static void Chassis_Takeoff_Detect(Chassis_t* My_Chassis);
//跳跃过程处理
static void Jump_Target_Process(Chassis_t* My_Chassis);
//撞膝上台阶
static void Knee_Strike_1_Target_Process(Chassis_t* My_Chassis);
static void Knee_Strike_2_Target_Process(Chassis_t* My_Chassis);
static void Jump_Down_2_Target_Process(Chassis_t* My_Chassis);
static void Walk_Down_2_Target_Process(Chassis_t* My_Chassis);
static void Fly_Target_Process(Chassis_t* My_Chassis);
static void Op_Fly_Target_Process(Chassis_t* My_Chassis);
static void Turn_Target_Process(Chassis_t* My_Chassis);
static void Cycle_Target_Process(Chassis_t* My_Chassis);

//阻尼卸力
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis);
//保护关节限位
static void My_Sd_Position_Nonlinear_Fix(Chassis_t* My_Chassis);
//功率限制
static float My_Chassis_Power_Limit(void);
static void Chassis_Power_Limit(void);

/* 测试用功能 begin*/
static void Test_phi0_l0_Ctrl(Chassis_t *My_Chassis);
static void Test_Straight_Ctrl(Chassis_t *My_Chassis);
/* 测试用功能 end*/

static void Chassis_Init_Ctrl(Chassis_t* My_Chassis);
/*自救*/
static void My_Chassis_Save_Process(Chassis_t *My_Chassis);
static void My_Chassis_Handle_Save_Process(Chassis_t *My_Chassis);

//static void Chassis_Motor_Target_Angle(Chassis_t* My_Chassis);
//static void My_Chassis_KEY_Input(void);
static void Chassis_Motor_Group_Offline_Check(Chassis_t* My_Chassis);
//static float My_Chassis_Power_Limit(void);

//将Instance导入
Leg_force_t Leg_force[Leg_Num];
Leg_Unit_t Leg_Unit[Leg_Num]=
{
	[R_Leg].Straight=&Straight_Leg[R_Leg],
	[R_Leg].Link=&Link[R_Leg],
	
	[L_Leg].Straight=&Straight_Leg[L_Leg],
	[L_Leg].Link=&Link[L_Leg],
	
	[R_Leg].force=&Leg_force[R_Leg],
	[L_Leg].force=&Leg_force[L_Leg],
};

Chassis_Rc_Input_t Chassis_Rc_Input;
Chassis_Key_Input_t Chassis_Key_Input;
Chassis_state_t Chassis_state;
Chassis_reset_state_t  Chassis_reset_state;
Chassis_Jump_t Chassis_Jump;
Chassis_Turn_t Chassis_Turn;
Chassis_Fly_t Chassis_Fly;
Chassis_Knee_Strike_t Chassis_Knee_Strike;
Chassis_Power_Limit_t Power_Limit;

Chassis_pid_init_parament_t Chassis_pid_init_parament;
Chassis_Target_t Chassis_Target = 
{
	.s = 0.7f,
	.sd1 = 0.f,
	.yaw_v = 0.f,
	.yaw = 0.f,
	.roll = 0.f,
	.thetal_r=0.f,
	.thetal_l=0.f,
	.leg_length_l = TAR_LEG_LENGTH_INITIAL,
	.leg_length_r = TAR_LEG_LENGTH_INITIAL,
	.thetab = 0.0f,
	.thetabd1 = 0.f,
	.limit_v = 0.f,
};

Chassis_t Chassis = {
	.Init = Chassis_Init,
	
};

/**
  * @brief  底盘软件层初始化
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
void Chassis_Init(Chassis_t* My_Chassis)
{
	/*结构体初始化*/
	My_Chassis->Posture = &Chassis_Posture;
	My_Chassis->Wheel=&Wheel_Group;
	My_Chassis->Sd= &Sd_Group;
	My_Chassis->state = &Chassis_state;
	My_Chassis->reset_struct = &Chassis_reset_state;
	My_Chassis->rc_input = &Chassis_Rc_Input;
	My_Chassis->key_input = &Chassis_Key_Input;
	My_Chassis->damping_delay_cnt = DAMPING_DELAY_MAX_CNT;
	My_Chassis->reset_struct->reset_cnt = 0;
	My_Chassis->target = &Chassis_Target;
	My_Chassis->chassis_PID=&chassis_PID;
	My_Chassis->jump_info=&Chassis_Jump;
	My_Chassis->knee_strike_info=&Chassis_Knee_Strike;
	My_Chassis->fly_info = &Chassis_Fly;
	My_Chassis->turn_info = &Chassis_Turn;
	My_Chassis->Power_Limit_info = &Power_Limit;
	My_Chassis->pid_init_parament=&Chassis_pid_init_parament;
	My_Chassis->Leg_Unit[R_Leg]=&Leg_Unit[R_Leg];
	My_Chassis->Leg_Unit[L_Leg]=&Leg_Unit[L_Leg];
	/*参数初始化*/
	My_Chassis->key_input->w_now = 0;
	My_Chassis->key_input->s_now = 0;
	My_Chassis->key_input->w_last = 0;
	My_Chassis->key_input->s_last = 0;
	My_Chassis->key_input->all_ws_now = 0;
	My_Chassis->key_input->all_ws_last = 0;
	My_Chassis->key_input->all_ad_now = 0;
	My_Chassis->key_input->all_ad_last = 0;
	//跳跃
	Chassis_Jump.Minimum_l0_range=0.01f; //NO_PRE_LANDING
	Chassis_Jump.Max_l0_range=0.01f;		//NO_PRE_LANDING
	Chassis_Jump.Landing_l0_range=0.03f;
	
	Chassis_Jump.Max_COMPRESS_tick=800.f; //NO_PRE_LANDING
	Chassis_Jump.Max_EXTEND_tick=1500.f;	//NO_PRE_LANDING
	Chassis_Jump.Max_RETRACT_tick=100.f;	//NO_PRE_LANDING 700
	Chassis_Jump.Max_PRE_LANDING_tick=1000.f;//600
	Chassis_Jump.Max_LANDING_tick=2000.f;//400
	
	Chassis_Jump.COMPRESS_length_kp=30.f; //40//NO_PRE_LANDING400
	Chassis_Jump.EXTEND_length_kp=4000.f;  //NO_PRE_LANDING4000
	Chassis_Jump.RETRACT_length_kp=60.f; //NO_PRE_LANDING4000
	Chassis_Jump.PRE_LANDING_length_kp=100.f;//500
	Chassis_Jump.LANDING_length_kp=100.f;//50//200
	Chassis_Jump.LANDING_length_speed_kp=100.f;//200//10000
	//撞膝上台阶
	Chassis_Knee_Strike.Idle_tick = 0.f;
	Chassis_Knee_Strike.Minimum_l0_range=0.01f;
	Chassis_Knee_Strike.Max_l0_range=0.01f;
	Chassis_Knee_Strike.Max_Stand_High_tick=10000;
	Chassis_Knee_Strike.STAND_length_kp=18.f;
	Chassis_Knee_Strike.RETRACT_length_kp=300.f;
	Chassis_Knee_Strike.thetal_threshold=25.f;//20//23//
	Chassis_Knee_Strike.down_thetal_threshold = 22.f;
	Chassis_Knee_Strike.down_thetab_threshold = 0.5f;
	Chassis_Knee_Strike.Max_RETRACT_tick=200.f;
	//换头
	Chassis_Turn.Going_tick = 0 ;
	Chassis_Turn.Max_GOING_tick = 500; 
	//飞坡 
	Chassis_Fly.Minimum_l0_range = 0.f;
	Chassis_Fly.Max_l0_range = 0.f;
	Chassis_Fly.Landing_l0_range = 0.f;
	Chassis_Fly.Middle_l0_range = 0.f;
	
	Chassis_Fly.HANG_length_kp = 0.f;
	Chassis_Fly.LANDING_length_kp = 0.f;
	Chassis_Fly.LANDING_length_speed_kp = 0.f;
	Chassis_Fly.STAND_length_kp = 0.f;
	Chassis_Fly.Max_Hang_tick = 600.f;
	Chassis_Fly.Max_LANDING_tick = 400.f;
	Chassis_Fly.Max_Stand_tick = 20000.f;
	
	Power_Limit.a = 0.f;
	Power_Limit.kk = 0.f;
	Power_Limit.Nf = 0.f;
	Power_Limit.power_limit = 60.f;
	Power_Limit.Tp_Big = 0.f;
	Power_Limit.Tw_Enable = 0.f;
	
	//pid初始参数    自己加上的
	My_Chassis->pid_init_parament->l0_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
	My_Chassis->pid_init_parament->l0_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
	My_Chassis->pid_init_parament->l0_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
	My_Chassis->pid_init_parament->l0_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;

	/*电机初始化*/
	My_Chassis->Wheel->group_init(My_Chassis->Wheel);
	My_Chassis->Sd->group_init(My_Chassis->Sd);
	
	/*底盘函数初始化*/
	My_Chassis->heartbeat = Chassis_HeartBeat;
	My_Chassis->data_update = Chassis_Data_Update;
	My_Chassis->status_react = Chassis_Status_React;
	My_Chassis->work = Chassis_Work;
	My_Chassis->ctrl = Chassis_Ctrl;
	
	/*五连杆初始化*/
	
	My_Chassis->Leg_Unit[R_Leg]->Link->init(My_Chassis->Leg_Unit[R_Leg]->Link);
	My_Chassis->Leg_Unit[L_Leg]->Link->init(My_Chassis->Leg_Unit[L_Leg]->Link);
	My_Chassis->Leg_Unit[R_Leg]->Straight->init(My_Chassis->Leg_Unit[R_Leg]->Straight);
	My_Chassis->Leg_Unit[L_Leg]->Straight->init(My_Chassis->Leg_Unit[L_Leg]->Straight);
	
	/*卡尔曼滤波器初始化*/
	xvEstimateKF_Init(&vaEstimateKF);
	XEstimateKF_Init(&XEstimateKF);
}

/**
  * @brief  底盘心跳包
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_HeartBeat(Chassis_t* My_Chassis)
{
	My_Chassis->Wheel->group_heartbeat(My_Chassis->Wheel);
	My_Chassis->Sd->group_heartbeat(My_Chassis->Sd);

	Chassis_Motor_Group_Offline_Check(My_Chassis);
	
	if(My_Chassis->state->sd_state == DEV_ONLINE && My_Chassis->state->wheel_state == DEV_ONLINE)
	{
		Balance.Flag->Chassis_Online_Flag = true;
	}
	else
	{
		Balance.Flag->Chassis_Online_Flag = false;
	}

}

/**
  * @brief  底盘运行
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Work(Chassis_t* My_Chassis)
{
	My_Chassis->status_react(My_Chassis);
	
	My_Chassis->data_update(My_Chassis);
	
	My_Chassis->ctrl(My_Chassis);
}

/**
  * @brief  底盘数据更新
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Data_Update(Chassis_t* My_Chassis)
{
	/*更新原始数据*/
	My_Chassis->Posture->data_update(My_Chassis->Posture);
	/*更新五连杆测量数据*/
	float phi1 = My_Phi1_Transform(R_Leg, My_Chassis->Sd->motor[R_F_Sd_M]);
	float phi4 = My_Phi4_Transform(R_Leg, My_Chassis->Sd->motor[R_B_Sd_M]);
	float phi1_d1 = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->speed;
	float phi4_d1 = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->speed;
	float torque_phi1_mea=My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->torque;
	float torque_phi4_mea=My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->torque;
	My_Chassis->Leg_Unit[R_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[R_Leg]->Link,phi1,phi1_d1,phi4,phi4_d1,torque_phi1_mea,torque_phi4_mea);
	
	phi1 = My_Phi1_Transform(L_Leg, My_Chassis->Sd->motor[L_F_Sd_M]);
	phi4 = My_Phi4_Transform(L_Leg, My_Chassis->Sd->motor[L_B_Sd_M]);
	phi1_d1 = -My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->speed;
	phi4_d1 = -My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->speed;
	torque_phi1_mea= - My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->torque;
	torque_phi4_mea= - My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->torque;
	My_Chassis->Leg_Unit[L_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[L_Leg]->Link,phi1,phi1_d1,phi4,phi4_d1,torque_phi1_mea,torque_phi4_mea);

	/*更新五连杆数据*/
	My_Chassis->Leg_Unit[R_Leg]->Link->link_update(My_Chassis->Leg_Unit[R_Leg]->Link);
	My_Chassis->Leg_Unit[L_Leg]->Link->link_update(My_Chassis->Leg_Unit[L_Leg]->Link);
	
	/*更新氮气弹簧数据*/
	My_Spring_Former_Input_Cal(My_Chassis);
	
	/*VMC逆解算,并与串*/
//	My_Chassis->Leg_Unit[R_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[R_Leg]->Link);
//	My_Chassis->Leg_Unit[L_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[L_Leg]->Link);
 
  My_Link_reverse_cal(My_Chassis->Leg_Unit[L_Leg]->Link,-(Sd_Group.motor[L_F_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front)
	                      ,-(Sd_Group.motor[L_B_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back));
  My_Link_reverse_cal(My_Chassis->Leg_Unit[R_Leg]->Link,Sd_Group.motor[R_F_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front
	                      ,Sd_Group.motor[R_B_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back);
	
	/*输入到直腿模型*/
	My_Chassis->Leg_Unit[R_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[R_Leg]->Straight,My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0);
	My_Chassis->Leg_Unit[L_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[L_Leg]->Straight,My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0);
	
	/*K矩阵更新*/
	#ifndef NO_K_Fitting
	My_Chassis->Leg_Unit[R_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[R_Leg]->Straight);
	My_Chassis->Leg_Unit[L_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[L_Leg]->Straight);
	#endif
	
	/*状态量更新，直接对Straight结构体操作*/
	Chassis_State_Var_Update(My_Chassis);
	
	/*目标值更新*/
	Chassis_Target_Update(My_Chassis);//放在Chassis_State_Var_Update后面，才好处理离地情况

}
float Cycle_target_theta_l;
float Cycle_target_theta_r;
static void Chassis_theta_Target_Update(Chassis_t* My_Chassis)
{
	static float init_theta_target;
	init_theta_target=0.f;
	if(My_Chassis->mode==C_Cycle)
	{
		My_Chassis->target->thetal_l=Cycle_target_theta_l;
		My_Chassis->target->thetal_r=Cycle_target_theta_r;
	}
	else
	{
		My_Chassis->target->thetal_l=init_theta_target;
		My_Chassis->target->thetal_r=init_theta_target;
	}
}
/**
  * @brief  底盘目标值更新
  * @param  None
  * @retval None
  */
static void Chassis_Target_Update(Chassis_t* My_Chassis)
{
	if(Balance.ctrl != KEY_CTRL)
	{
	  Chassis_Rc_Input_Update(My_Chassis);//遥控器输入值步进限幅滤波
	}
	else
	{
	  Chassis_Key_Input_Update(My_Chassis);
	}
	
	Chassis_sd1_Target_Update(My_Chassis);//控sd1
	Chassis_Yaw_Target_Process_All(My_Chassis);//输出My_Chassis->target->yaw
	Chassis_Leg_Length_Target_Process(My_Chassis);//腿长目标值控制
	Chassis_Speed_Limit(My_Chassis);//sd1目标值限制
//	Chassis_theta_Target_Update(My_Chassis);
	
	/*离地时位移和pitch的目标值=状态量，使这两项输出为0*/
	if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->Leg_Unit[R_Leg]->Straight->info->s,My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1,
												My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetab,My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetabd1) ;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->target->s,My_Chassis->target->sd1,
												My_Chassis->target->thetab,My_Chassis->target->thetabd1) ;
	}
	if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->Leg_Unit[L_Leg]->Straight->info->s,My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1,
												My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetab,My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetabd1) ;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
												My_Chassis->target->thetal_r,My_Chassis->target->thetald1,
												My_Chassis->target->s,My_Chassis->target->sd1,
												My_Chassis->target->thetab,My_Chassis->target->thetabd1) ;
	}
	
}

/**
  * @brief  状态变量更新
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_State_Var_Update(Chassis_t* My_Chassis)//角度均用弧度制
{
	/*结构体输入*/
	State_info_t* R_Leg_State_Var = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
	State_info_t* L_Leg_State_Var = My_Chassis->Leg_Unit[L_Leg]->Straight->info;
	Link_t * R_Leg_Link =My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t * L_Leg_Link =My_Chassis->Leg_Unit[L_Leg]->Link;
	Chassis_Posture_info_t* My_Posture = My_Chassis->Posture->info;
	Motor_RM_t L_Wheel = *My_Chassis->Wheel->motor[L_WHEEL_M];
	Motor_RM_t R_Wheel = *My_Chassis->Wheel->motor[R_WHEEL_M];
	
	/*杆倾斜角度、机体角度 begin*/
	//与上交模型不同
	R_Leg_State_Var->thetal_last=R_Leg_State_Var->thetal;
	R_Leg_State_Var->thetal = (R_VIR_PHI0_ORDER_CORRECT*R_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
	R_Leg_State_Var->thetald1 = R_VIR_PHI0_ORDER_CORRECT*R_Leg_Link->info->angle->vir_phi0_d1-My_Posture->pitch_v;
	R_Leg_State_Var->thetald1_l_now=R_Leg_State_Var->thetald1;
	R_Leg_State_Var->thetald2=R_Leg_State_Var->thetald1_l_now-R_Leg_State_Var->thetald1_l_last;
	R_Leg_State_Var->thetald1_l_last=R_Leg_State_Var->thetald1_l_now;
	R_Leg_State_Var->thetab= My_Posture->pitch;
	R_Leg_State_Var->thetabd1= My_Posture->pitch_v;
	
	L_Leg_State_Var->thetal_last=L_Leg_State_Var->thetal;
	L_Leg_State_Var->thetal = (L_VIR_PHI0_ORDER_CORRECT*L_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
	L_Leg_State_Var->thetald1 = L_VIR_PHI0_ORDER_CORRECT*L_Leg_Link->info->angle->vir_phi0_d1-My_Posture->pitch_v;
	L_Leg_State_Var->thetald1_l_now=L_Leg_State_Var->thetald1;
	L_Leg_State_Var->thetald2=L_Leg_State_Var->thetald1_l_now-L_Leg_State_Var->thetald1_l_last;
	L_Leg_State_Var->thetald1_l_last=L_Leg_State_Var->thetald1_l_now;
	L_Leg_State_Var->thetab= My_Posture->pitch;
	L_Leg_State_Var->thetabd1= My_Posture->pitch_v;
	/*杆倾斜角度、机体角度 end*/
	
	
	
	
	/*消除杆动-->电机定子动-->编码器变化带来的影响*/
	Stator_Correction_Cal(My_Chassis);
	
	/*俯仰角与俯仰角速度 begin*/
  R_Leg_State_Var->thetab = My_Posture->pitch;
  L_Leg_State_Var->thetab = My_Posture->pitch;
  R_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
  L_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
	/*俯仰角与俯仰角速度 end*/
	
	/*卡尔曼滤波更新*/
	/*路程与速度 begin*/
	float My_Wheel_Sb;
	float My_Imu_Sb;
	float My_filter_Sb;

	float My_filter_S;
	float My_filter_S_d1;
	
	float L_Wheel_speed_Transformed= L_W_SPEED_ORDER_CORRECT* L_Wheel.rx_info->speed;                                                                                                                                                             
	float R_Wheel_speed_Transformed= R_W_SPEED_ORDER_CORRECT*R_Wheel.rx_info->speed;
	float L_Wheel_anglesum_Transformed= L_Wheel.rx_info->motor_angle_sum;//已在chassis_motor矫正方向                                                                                                                                                             
	float R_Wheel_anglesum_Transformed= R_Wheel.rx_info->motor_angle_sum;
	//轮速+腿速
	My_Wheel_Sb = (WHEEL_RADIUS*(L_Wheel_speed_Transformed+R_Wheel_speed_Transformed) \
	               + (R_Leg_Link->info->length->l0*arm_cos_f32(R_Leg_State_Var->thetal)*R_Leg_State_Var->thetald1) \
	               + (L_Leg_Link->info->length->l0*arm_cos_f32(L_Leg_State_Var->thetal)*L_Leg_State_Var->thetald1))/2.f;
	My_Imu_Sb = My_Chassis->Posture->info->x_world;//机体加速度
	xvEstimateKF_Update(&vaEstimateKF, My_Imu_Sb, My_Wheel_Sb);
	My_filter_Sb  = vaEstimateKF.FilteredValue[0];//轮速+腿速

	
	//轮子位移（已考虑定子转动）
	float s = 0.5f * WHEEL_RADIUS * ((float)L_Wheel_anglesum_Transformed -L_Leg_Link->info->stator_correction->stator_bias \
	                                         + (float)R_Wheel_anglesum_Transformed - R_Leg_Link->info->stator_correction->stator_bias);
		
	//轮速
	float sd1 =My_filter_Sb -
	                   (((L_Leg_Link->info->length->l0*arm_cos_f32(L_Leg_State_Var->thetal)*L_Leg_State_Var->thetald1) \
	                    +(R_Leg_Link->info->length->l0*arm_cos_f32(R_Leg_State_Var->thetal)*R_Leg_State_Var->thetald1))/2.f);
	
	XEstimateKF_Update(&XEstimateKF, sd1, s);        
	My_filter_S = XEstimateKF.FilteredValue[0];//位移
	My_filter_S_d1 =  XEstimateKF.FilteredValue[1];//速度
	
	R_Leg_State_Var->s = My_filter_S;
	L_Leg_State_Var->s = My_filter_S;
	R_Leg_State_Var->sd1 = My_filter_S_d1;
	L_Leg_State_Var->sd1 = My_filter_S_d1;
	/*路程与速度 end*/
}


static void Test_Straight_Ctrl(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
	Straight_Leg_t* L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_target begin--------*/
	
	Chassis_Leg_Sync_Cal(My_Chassis);
	
	R_Straight->LQR_cal(R_Straight);//在目标值处做离地处理
	L_Straight->LQR_cal(L_Straight);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR=R_Straight->get_Tp(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR=L_Straight->get_Tp(L_Straight);

//	 if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true||
//		   My_Chassis->Leg_Unit[L_Leg]->off_ground == true) //离地处理
//      {
//       My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
//		   My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
//      }
//	else{
		   My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR
													+My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		   My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR
													+My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
//	    }
	
	
	
	/*-----------求Tp_target end-----------*/
	
	/*-----------求Fb1_target begin--------*/
	/*roll控制力计算*/
	Chassis_Roll_Control(My_Chassis);
			
	/*腿长控制力计算*/
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*前馈计算*/
	Chassis_Link_Feedforward_Cal(My_Chassis);
	
	/*汇总得到Fbl_target*/
			static float t = 0;
	if((my_abs(Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0) <= 0.2f && my_abs(Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0) <= 0.2f) || t == 1)
	{
		t = 1;
		My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F
															+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
	//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
//															+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F
															+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
	//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
//															+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
  }
  else{
	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F;
//															+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
	//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
	//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F;
//															+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
	//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
	//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;

}
	
	/*-----------求Fb1_target end--------*/
	
	/*-----------求Tw_target begin--------*/
	/*驱动轮转向环Tw_turn*/
	Chassis_Wheel_Turn_Cal(My_Chassis);
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR=R_Straight->get_Tw(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR=L_Straight->get_Tw(L_Straight);
	/* 驱动轮电机最终输出 */
	 if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR 
														+ My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
    }
    if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR 
														+ My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
    }
		
	/*-----------求Tw_target end--------*/
	
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	/*限位力矩补偿*/
//	My_Sd_Position_Nonlinear_Fix(My_Chassis);
	/* 关节电机最终输出 */
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix;
}

/**
  * @brief  控phi0、l0,把车架起来测
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Test_phi0_l0_Ctrl(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_target begin--------*/
	
	My_Chassis->target->vir_phi0_r +=My_Chassis->rc_input->ch2_now/660.f*TIME_STEP;
	My_Chassis->target->vir_phi0_l +=My_Chassis->rc_input->ch2_now/660.f*TIME_STEP;
	My_Chassis->target->vir_phi0_r=constrain(My_Chassis->target->vir_phi0_r,angle2rad(-60),angle2rad(60));
	My_Chassis->target->vir_phi0_l=constrain(My_Chassis->target->vir_phi0_l,angle2rad(-60),angle2rad(60));
	
	Chassis_Leg_vir_phi0_Cal(My_Chassis);//内部赋值给chassis
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0;
	/*-----------求Tp_target end--------*/
	
	/*-----------求Fb1_target begin--------*/

	/*腿长控制力计算*/
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*腿部前馈*/
	//车架起来的时候前馈应该收腿
	My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = -( My_R_Link->info->centroid->centriod_coefficient*m_l) * g * cos(My_R_Link->info->angle->vir_phi0);
	My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = -( My_L_Link->info->centroid->centriod_coefficient*m_l) * g * cos(My_L_Link->info->angle->vir_phi0);
	
	My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F;
//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
//	My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F;							
	My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F;
//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;

	/*-----------求Fb1_target end--------*/
	
	/*建模的Tp方向是顺时针，VMC是逆时针，所以从建模--->VMC要加个负号*/
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
	/*赋值到chassis目标结构体*/
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque;          //+My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque;          // -My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque;          // +My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque;          //  -My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back;
	
//	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=0;
//	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=0;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=0;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=0;
}
/**
  * @brief  底盘工作模式更新
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Chassis_Status_React(Chassis_t *My_Chassis)
{
	switch(Balance.mode)
	{
		case Init_Mode:
			My_Chassis->mode = C_Init;
			break;
		case Sleep_Mode:
			My_Chassis->mode = C_Sleep;
			break;
		case Imu_Mode:
		  My_Chassis->mode = C_Follow;
//      if(Balance.Flag->Fly_Flag == 1)
//      {				
//				My_Chassis->mode = C_Fly;
//			}
			if(Balance.Flag->Jumping_Flag == 1)
			{
				My_Chassis->mode = C_Jump;
			}
			if(Balance.Flag->Knee_Strike_1_Flag == 1)
			{
				My_Chassis->mode = C_Knee_Strike_1;
			}
			if(Balance.Flag->Turn_Flag == 1)
			{
				My_Chassis->mode = C_Turn;
			}
			if(Balance.Flag->Down_Two_Step_Flag == 1)
			{
				My_Chassis->mode = C_Down_Two_Step;
			}
//			if(Balance.Flag->Op_Fly_Flag == 1)
//      {				
//				My_Chassis->mode = C_Op_Fly;
//			}
//			if(Balance.command->slave->Slave_Online_Flag == false)
//				My_Chassis->mode = C_Boss;
			break;
		case Mec_Mode:
		case Lob_Mode:
			My_Chassis->mode = C_Boss;
//      if(Balance.Flag->Fly_Flag == 1)
//      {				
//				My_Chassis->mode = C_Fly;
//			}
//			if(Balance.Flag->Jumping_Flag == 1)
//			{
//				My_Chassis->mode = C_Jump;
//			}
//			if(Balance.Flag->Knee_Strike_1_Flag == 1)
//			{
//				My_Chassis->mode = C_Knee_Strike_1;
//			}
//			if(Balance.Flag->Op_Fly_Flag == 1)
//      {				
//				My_Chassis->mode = C_Op_Fly;
//			}
			break;		
		case Cycle_Mode:
			My_Chassis->mode = C_Cycle;
		  break;
		case Vary_Cycle_Mode:
			My_Chassis->mode = C_Vary_Cycle;
		  break;
		
		case LEG_TEST_Mode:
			My_Chassis->mode = C_Test;
			break;	
    case Rescue_Mode:
      My_Chassis->mode = C_Rescue;
      break;	
    case Handle_Mode:
      My_Chassis->mode = C_Handle_Rescue;
		  break;
		default:
			My_Chassis->mode = C_Sleep;
			break;
	}
		
		//自救检测
//		#ifndef NO_RESCUE
//		if(Balance.Flag->Rescue_Flag == true &&Balance.Flag->Unable_Rescue_Flag)
//		{
//			My_Chassis->mode = C_Rescue;
//		}
//		else
//		{
//			My_Chassis->mode = C_Sleep;
//		}
//		
//		#endif
	
	//离线保护
	if(My_Chassis->state->sd_state == DEV_OFFLINE || 
		My_Chassis->state->wheel_state == DEV_OFFLINE||
		Balance.Flag->Rescue_Flag == true ||
			Balance.mode==Sleep_Mode
		)
	{
//		My_Chassis->mode = C_Sleep;
	}
	
	if(My_Chassis->mode != C_Init)
		My_Chassis->reset_struct->reset_cnt = 0;
	
	if(Balance.Flag->Chassis_Sleep_Flag == true)
	{
		My_Chassis->mode = C_Sleep;
	}
}

/**
  * @brief  底盘阻尼卸力
  * @param  Chassis_t* My_Chassis
  * @retval None         
  */
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis)
{
		if(Balance.Flag->Chassis_Online_Flag == false)
		{
			My_Chassis->Sd->group_sleep(My_Chassis->Sd);
			My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
		}
		else
		{
			if(My_Chassis->damping_delay_cnt < DAMPING_DELAY_MAX_CNT)
			{
				Chassis_Stop_Damping(My_Chassis);
				
				My_Chassis->damping_delay_cnt++; 
			}
			else
			{
				My_Chassis->Sd->group_sleep(My_Chassis->Sd);//含发送
				My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
			}
		}
}

/**
  * @brief  底盘总控制
  * @param  None
  * @retval None
  */
static void Chassis_Ctrl(Chassis_t *My_Chassis)
{
	Clean_Process(My_Chassis);
	switch(My_Chassis->mode)
	{
		case C_Sleep:
		  Chassis_Takeoff_Detect(My_Chassis);
//		  Chassis_Torque_Cal(My_Chassis);
		  My_Chassis_Save_Process(My_Chassis);
//		  My_Chassis_Handle_Save_Process(My_Chassis);
		  Chassis_Damping_Sleep(My_Chassis);
		  /*离线处理*/
		  Chassis_Offline_Process(My_Chassis);
		  break;
		
		case C_Follow:
		  Chassis_Takeoff_Detect(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;	
		
		case C_Boss:
		  Chassis_Takeoff_Detect(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		case C_Init://不进行离地判断,不接受遥控和标志位响应
//		case C_Test:

		  My_Chassis_Save_Process(My_Chassis);
		
//		  Balance.Chassis_Com->save_type = 1;
//      My_Chassis_Handle_Save_Process(My_Chassis);
		
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		
//		  if(my_abs(Chassis.Leg_Unit[L_Leg]->Link->info->angle->phi0_ - 15.f) <= 0.5f && my_abs(Chassis.Leg_Unit[R_Leg]->Link->info->angle->phi0_ - 15.f) <= 0.5f)
//			{
//			  Chassis.reset_struct->reset_state = Chassis_reset_OK;
//				Chassis.chassis_PID->length_cal[L_Leg]->target = TAR_LEG_LENGTH_INITIAL;
//				Chassis.chassis_PID->length_cal[R_Leg]->target = TAR_LEG_LENGTH_INITIAL;
////		    Balance.Chassis_Com->save_type = 0;
//			}
		  break;
		
//		case C_Init:
		case C_Test:
		  Chassis_Takeoff_Detect(My_Chassis);
		  Balance.Flag->Leg_length_ctrl_Flag = true;
//		  if(rc_sensor.info->s1 == 3 && rc_sensor.info->s1_last == 1
//				&& Chassis.target->leg_length_l != 0.35f)
//			{
//				Chassis.target->leg_length_l = 0.35f;
//				Chassis.target->leg_length_r = 0.35f;
//			}
//			else if(rc_sensor.info->s1 == 1 && rc_sensor.info->s1_last == 3
//				     && Chassis.target->leg_length_l != TAR_LEG_LENGTH_INITIAL)
//			{
//				Chassis.target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//				Chassis.target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//			}
		
//		    Chassis_Torque_Cal(My_Chassis);
		  Test_Straight_Ctrl(My_Chassis);
//		  Test_phi0_l0_Ctrl(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
//		Chassis_Motor_Set_Sleep(My_Chassis);
		  break;
		
		case C_Rescue:
		case C_Handle_Rescue:
      My_Chassis_Handle_Save_Process(My_Chassis); 
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		case C_Jump:
//		  Chassis_Takeoff_Detect(My_Chassis);
		  Jump_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		case C_Knee_Strike_1:
//	  	Chassis_Takeoff_Detect(My_Chassis);
		  Knee_Strike_1_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
	  	break;
		
		case C_Knee_Strike_2:
//	  	Chassis_Takeoff_Detect(My_Chassis);
		  Knee_Strike_2_Target_Process(My_Chassis);
		  Test_Straight_Ctrl(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
	  	break;
		
		case C_Down_Two_Step:
//	  	Chassis_Takeoff_Detect(My_Chassis);
		#ifdef WALK_DOWN_2
		  Walk_Down_2_Target_Process(My_Chassis);
		#else
      Jump_Down_2_Target_Process(My_Chassis);
		#endif
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
	  	break;
		
		case C_Fly:
	  	Chassis_Takeoff_Detect(My_Chassis);
		  Fly_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);		
		  break;
		
		case C_Turn:
	  	Chassis_Takeoff_Detect(My_Chassis);
		  Turn_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);		
		  break;
		
		case C_Op_Fly:
			
		  break;
		
		case C_Cycle:
		case C_Vary_Cycle:
			Cycle_Target_Process(My_Chassis);
		  Chassis_Torque_Cal(My_Chassis);
		  Chassis_Set_Torque(My_Chassis);
		  break;
		
		default:
			My_Chassis->Sd->group_sleep(My_Chassis->Sd);//含发送
			My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
			Chassis_Offline_Process(My_Chassis);
			break;
	}
}
/**
  * @brief  清标志位等处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Clean_Process(Chassis_t* My_Chassis)
{
	if(My_Chassis->mode!=C_Rescue)
	{
		My_Chassis->target->vir_phi0d1_r=0;
		My_Chassis->target->vir_phi0d1_l=0;
	}
	
	if(My_Chassis->mode!=C_Sleep)
	{
		My_Chassis->damping_delay_cnt = 0;
	}
	
}

/**

  * @brief  飞坡过程处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Fly_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Fly_t* fly_info = My_Chassis->fly_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	fly_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
	fly_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;

	switch(fly_info->step)
	{
		case Fly_IDLE:
			fly_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
			fly_info->IDLE_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
			fly_info->IDLE_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
			fly_info->IDLE_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
		
		  fly_info->Hang_tick = 0;
		  fly_info->LANDING_tick = 0;
		  fly_info->Stand_tick = 0;
		
		
		  fly_info->step = Fly_Stand_Middle;
		  break;
		case Fly_Stand_Middle:
			fly_info->Stand_tick++;
			My_Chassis->target->leg_length_l = fly_info->Middle_l0_range;
			My_Chassis->target->leg_length_r = fly_info->Middle_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=fly_info->STAND_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=fly_info->STAND_length_kp;
		
		  if((fly_info->r_offground == true && fly_info->l_offground == true) || 
				fly_info->Stand_tick >= fly_info->Max_Stand_tick)
			{
				fly_info->step = Fly_Hang;
			}
			break;
		case Fly_Hang:
			fly_info->Hang_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-fly_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-fly_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=fly_info->HANG_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=fly_info->HANG_length_kp;
		
		  if((fly_info->r_offground == false && fly_info->l_offground == false) || 
				fly_info->Hang_tick >= fly_info->Max_Hang_tick)
			{
				fly_info->step = Fly_Land;
			}
			
		  break;
		case Fly_Land:
			fly_info->LANDING_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-5.0f*fly_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-5.0f*fly_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=fly_info->LANDING_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=fly_info->LANDING_length_kp;
		
		  if((fly_info->r_offground == false && fly_info->l_offground == false) || 
				fly_info->Hang_tick >= fly_info->Max_Hang_tick)
			{
				fly_info->step = Fly_IDLE;
				Balance.Flag->Jumping_Flag = false;
//				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=fly_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=fly_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=fly_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=fly_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=fly_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=fly_info->IDLE_length_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=fly_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=fly_info->IDLE_length_speed_outmax;

			}
		  break;
		default:
			break;
	}
}

/**
  * @brief  反飞坡过程处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Op_Fly_Target_Process(Chassis_t* My_Chassis)
{
	
}

/**
  * @brief  换头过程处理   不用
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Turn_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Turn_t* turn_info = My_Chassis->turn_info;
	
	switch(turn_info->step)
	{
		case Turn_IDLE:
			turn_info->step = Turn_Going;
			My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;
		  break;
		case Turn_Going:
		  turn_info->Going_tick++;
		  if(gimbal.base_info.Gimbal_Turn_Finish == 1/* || turn_info->Going_tick >= turn_info->Max_GOING_tick*/)
			{
				turn_info->step = Turn_IDLE;
				Balance.Flag->Turn_Flag = 0;
				gimbal.base_info.Gimbal_Turn_Finish = 0;
			}
		  break;
		default:
			break;
	}
}


/**
  * @brief  跳跃过程处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Jump_Target_Process(Chassis_t* My_Chassis)
{
	//#define TIME_ONLY
	Chassis_Jump_t* jump_info=My_Chassis->jump_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	jump_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
	jump_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;

	jump_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (jump_info->jump_step)
	{
		case J_IDLE:
			//动作
			jump_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
			jump_info->IDLE_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
			jump_info->IDLE_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
			jump_info->IDLE_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
			
			My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=1000.f;
			
			jump_info->COMPRESS_tick=0;
			jump_info->EXTEND_tick=0;
			jump_info->RETRACT_tick=0;
			jump_info->PRE_LANDING_tick=0;
			jump_info->LANDING_tick=0;
			//事件
			jump_info->jump_step=J_COMPRESS;
			break;
		
		case J_COMPRESS://压缩腿  
			//动作
			jump_info->COMPRESS_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->COMPRESS_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->COMPRESS_length_kp;
			//事件
			#ifdef TIME_ONLY
				if(jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			jump_info->jump_step=J_EXTEND;
			#else
				if((jump_info->l0_average<=MIN_LEG_LENGTH+jump_info->Minimum_l0_range)
				|| jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
				{
			    jump_info->jump_step=J_EXTEND;
				}

			#endif
			
			break;
			
		case J_EXTEND://跳
			//动作
			jump_info->EXTEND_tick++;
      jump_info->Jump_F = 260.f;
		//事件
			#ifdef TIME_ONLY
			if(jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#else
				if((jump_info->l0_average>=0.32f)//MAX_LEG_LENGTH -jump_info->Max_l0_range)
				||jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#endif
			{
			jump_info->jump_step=J_RETRACT;
				jump_info->Jump_F = 0.f;
			}
			break;
		
		case J_RETRACT://收腿滞空
			//动作
			jump_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->RETRACT_length_kp;
			//事件
//			if(jump_info->r_offground==false&&jump_info->l_offground==false)
//			{
//				Balance.Flag->Jumping_Flag = false;
//				jump_info->jump_step=J_IDLE;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//			}	
			
			
				#ifdef NO_PRE_LANDING
				#ifdef TIME_ONLY
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#else
				if(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)<=0.f
					||jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#endif
				
				{
					Balance.Flag->Jumping_Flag = false;
					jump_info->jump_step=J_IDLE;
					My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
					My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				}

				#else
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
//					|| my_abs(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)) <= 0.03)
				{
					jump_info->jump_step=J_PRE_LANDING;
				}
				#endif
			break;
		
		 case J_PRE_LANDING://伸腿准备缓冲
			//动作
			jump_info->PRE_LANDING_tick++;
			My_Chassis->target->leg_length_l += 0.2f * TIME_STEP;
			My_Chassis->target->leg_length_r += 0.2f * TIME_STEP;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->PRE_LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->PRE_LANDING_length_kp;
			//事件
			if(jump_info->r_offground==false||jump_info->l_offground==false)
			{
				jump_info->jump_step=J_LANDING;
			}	
			if(jump_info->PRE_LANDING_tick>=jump_info->Max_PRE_LANDING_tick)
			{
				jump_info->jump_step=J_LANDING;
			}
			break;
			
		  case J_LANDING://缓冲
			//动作
			jump_info->LANDING_tick++;
			My_Chassis->target->leg_length_l -= 0.03f * TIME_STEP;
			My_Chassis->target->leg_length_r -= 0.03f * TIME_STEP;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->LANDING_length_kp;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->LANDING_length_speed_kp;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->LANDING_length_speed_kp;
			//事件
			if(jump_info->LANDING_tick>=jump_info->Max_LANDING_tick||
				(my_abs(jump_info->l0_average-MIN_LEG_LENGTH)<=0.03f))
			{
				Balance.Flag->Jumping_Flag = false;
				jump_info->jump_step=J_IDLE;
				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
			}
			break;
			
			
		  default:
            break;
	}
	
}
//static void Jump_Target_Process(Chassis_t* My_Chassis)
//{
//	//#define TIME_ONLY
//	Chassis_Jump_t* jump_info=My_Chassis->jump_info;
//	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
//	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
//	jump_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
//	jump_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;

//	jump_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
//	
//	switch (jump_info->jump_step)
//	{
//		case J_IDLE:
//			//动作
//			jump_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
//			jump_info->IDLE_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
//			jump_info->IDLE_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
//			jump_info->IDLE_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
//			
//			My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=1000.f;
//			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=400.f;
//			My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=1000.f;
//			My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=400.f;
//			
//			jump_info->COMPRESS_tick=0;
//			jump_info->EXTEND_tick=0;
//			jump_info->RETRACT_tick=0;
//			jump_info->PRE_LANDING_tick=0;
//			jump_info->LANDING_tick=0;
//			//事件
//			jump_info->jump_step=J_COMPRESS;
//			break;
//		
//		case J_COMPRESS://压缩腿  
//			//动作
//			jump_info->COMPRESS_tick++;
//			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
//			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->COMPRESS_length_kp;
//		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->COMPRESS_length_kp;
//			//事件
//			#ifdef TIME_ONLY
//				if(jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
//			jump_info->jump_step=J_EXTEND;
//			#else
//				if((jump_info->l0_average<=MIN_LEG_LENGTH+jump_info->Minimum_l0_range)
//				|| jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
//			jump_info->jump_step=J_EXTEND;
//			#endif
//			
//			break;
//			
//		case J_EXTEND://跳
//			//动作
//			jump_info->EXTEND_tick++;
//			My_Chassis->target->leg_length_l = 0.32f;//MAX_LEG_LENGTH-jump_info->Max_l0_range;
//			My_Chassis->target->leg_length_r = 0.32f;//MAX_LEG_LENGTH-jump_info->Max_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->EXTEND_length_kp;
//		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->EXTEND_length_kp;
//			//事件
//			#ifdef TIME_ONLY
//			if(jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
//			#else
//				if((jump_info->l0_average>=0.32f)//MAX_LEG_LENGTH -jump_info->Max_l0_range)
//				||jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
//			#endif
//			
//			jump_info->jump_step=J_RETRACT;
//			break;
//		
//		case J_RETRACT://收腿滞空
//			//动作
//			jump_info->RETRACT_tick++;
//			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
//			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->RETRACT_length_kp;
//		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->RETRACT_length_kp;
//			//事件
////			if(jump_info->r_offground==false&&jump_info->l_offground==false)
////			{
////				Balance.Flag->Jumping_Flag = false;
////				jump_info->jump_step=J_IDLE;
////				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
////				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
////				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
////				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
////			}	
//			
//			
//				#ifdef NO_PRE_LANDING
//				#ifdef TIME_ONLY
//				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
//				#else
//				if(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)<=0.f
//					||jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
//				#endif
//				
//				{
//					Balance.Flag->Jumping_Flag = false;
//					jump_info->jump_step=J_IDLE;
//					My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//					My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//					My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
//					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//					My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
//					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//				}

//				#else
//				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
//				{
//					jump_info->jump_step=J_PRE_LANDING;
//				}
//				#endif
//			break;
//		
//		 case J_PRE_LANDING://伸腿准备缓冲
//			//动作
//			jump_info->PRE_LANDING_tick++;
//			My_Chassis->target->leg_length_l = 0.32f;//MAX_LEG_LENGTH-jump_info->Max_l0_range;
//			My_Chassis->target->leg_length_r = 0.32f;//MAX_LEG_LENGTH-jump_info->Max_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->PRE_LANDING_length_kp;
//		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->PRE_LANDING_length_kp;
//			//事件
//			if(jump_info->r_offground==false||jump_info->l_offground==false)
//			{
//				jump_info->jump_step=J_LANDING;
//			}	
//			if(jump_info->PRE_LANDING_tick>=jump_info->Max_PRE_LANDING_tick)
//			{
//				jump_info->jump_step=J_LANDING;
//			}
//			break;
//			
//		  case J_LANDING://缓冲
//			//动作
//			jump_info->LANDING_tick++;
//			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-5.0f*jump_info->Max_l0_range;
//			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-5.0f*jump_info->Max_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->LANDING_length_kp;
//		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->LANDING_length_kp;
//			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->LANDING_length_speed_kp;
//		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->LANDING_length_speed_kp;
//			//事件
//			if(jump_info->LANDING_tick>=jump_info->Max_LANDING_tick||
//				(my_abs(jump_info->l0_average-TAR_LEG_LENGTH_INITIAL)<=jump_info->Landing_l0_range))
//			{
//				Balance.Flag->Jumping_Flag = false;
//				jump_info->jump_step=J_IDLE;
////				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
////				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//			}
//			break;
//			
//			
//		  default:
//            break;
//	}
//	
//}


/**
  * @brief  撞膝上台阶
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Knee_Strike_1_Target_Process(Chassis_t* My_Chassis)
{
	
	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	knee_strike_info->thetal_average=Rad2Angle*fabsf(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
	
	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (knee_strike_info->step1)
	{
		case Knee_IDLE:
			if(my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) >= PI / 2/* || my_abs(Chassis.Posture->info->yaw - Chassis.target->yaw) >= 0.3f*/)
			{
				knee_strike_info->Idle_tick ++;
				Balance.Flag->Return_Flag = true;
				knee_strike_info->step1=Knee_IDLE;
			}
			else if(Balance.Flag->Return_Flag == false && my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) <= PI / 2 && my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->err) <= 0.2f)
			{
				//动作
				knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
				knee_strike_info->Stand_High_tick=0;
				//事件
				knee_strike_info->step1=Knee_Stand_High;
			}
			else if(knee_strike_info->Idle_tick >= 1000.f)
			{
				knee_strike_info->Idle_tick = 0;
				knee_strike_info->step1=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				
				Balance.Flag->Knee_Strike_1_Flag=false;
			}
			break;
		
		case Knee_Stand_High://立着
			//动作
			knee_strike_info->Stand_High_tick++;
		  if(My_Chassis->target->leg_length_l <= 0.35f)//0.35
			{
				My_Chassis->target->leg_length_l += 0.35f * TIME_STEP;
				My_Chassis->target->leg_length_r += 0.35f * TIME_STEP;
			}
			if(My_Chassis->target->leg_length_l >= 0.35f)
			{
				My_Chassis->target->leg_length_l = 0.35f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
				My_Chassis->target->leg_length_r = 0.35f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			}
			My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->STAND_length_kp;
		  My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->STAND_length_kp;
			//事件
			if(knee_strike_info->thetal_average>=knee_strike_info->thetal_threshold)
			{
				knee_strike_info->step1=Knee_RETRACT;
				knee_strike_info->RETRACT_tick = 0;
			}
			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
			{
				knee_strike_info->step1=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
				Balance.Flag->Knee_Strike_1_Flag=false;
			}
			break;
			
		case Knee_RETRACT://收腿
			//动作
			knee_strike_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		  My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 600.f;
		  My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 600.f;
		My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
			//事件
			if(knee_strike_info->RETRACT_tick>=knee_strike_info->Max_RETRACT_tick
				|| my_abs(knee_strike_info->l0_average-MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range) <= 0.01)
			{
				knee_strike_info->Idle_tick = 0;
				knee_strike_info->step1=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
		    My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 200.f;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 200.f;
				
				Balance.Flag->Knee_Strike_1_Flag=false;
			}
			break;
		
		  default:
            break;
	}
}

/**
  * @brief  跳跃撞膝上台阶
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Knee_Strike_2_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
	knee_strike_info->thetal_average=Rad2Angle*fabsf(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
	Chassis_Jump_t* jump_info=My_Chassis->jump_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	jump_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
	jump_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;

	jump_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);	
	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (knee_strike_info->step2)
	{
		case JK_IDLE:
			//动作
//			knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
//			knee_strike_info->Stand_High_tick=0;
			//事件
//			knee_strike_info->step2=JK_Stand_High;
//			break;
//		case J_IDLE:
			//动作
			jump_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
			jump_info->IDLE_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
			jump_info->IDLE_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
			jump_info->IDLE_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
			
			My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=1000.f;
			
			jump_info->COMPRESS_tick=0;
			jump_info->EXTEND_tick=0;
			jump_info->RETRACT_tick=0;
			jump_info->PRE_LANDING_tick=0;
			jump_info->LANDING_tick=0;
			knee_strike_info->Stand_High_tick=0;
			//事件
			knee_strike_info->step2=JK_COMPRESS;
			break;
		
		case JK_COMPRESS://压缩腿  
			//动作
			jump_info->COMPRESS_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->COMPRESS_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->COMPRESS_length_kp;
			//事件
			#ifdef TIME_ONLY
				if(jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			jump_info->jump_step=J_EXTEND;
			#else
				if((jump_info->l0_average<=MIN_LEG_LENGTH+jump_info->Minimum_l0_range)
				|| jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			knee_strike_info->step2=JK_EXTEND;
			#endif
			
			break;
			
		case JK_EXTEND://跳
			//动作
			jump_info->EXTEND_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->EXTEND_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->EXTEND_length_kp;
			//事件
			#ifdef TIME_ONLY
			if(jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#else
				if((jump_info->l0_average>=MAX_LEG_LENGTH -jump_info->Max_l0_range)
				||jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#endif
			
			knee_strike_info->step2=JK_RETRACT_1;
			break;
		
		case JK_RETRACT_1://收腿滞空
			//动作
			jump_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->RETRACT_length_kp;
			//事件
//			if(jump_info->r_offground==false&&jump_info->l_offground==false)
//			{
//				Balance.Flag->Jumping_Flag = false;
//				jump_info->jump_step=J_IDLE;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//			}	
			
			
				#ifdef NO_PRE_LANDING
				#ifdef TIME_ONLY
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#else
				if(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)<=0.f
					||jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#endif
				
				{
					Balance.Flag->Jumping_Flag = false;
					knee_strike_info->step2=JK_IDLE;
					My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
					My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				}

				#else
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				{
					knee_strike_info->step2=JK_PRE_LANDING;
				}
				#endif
			break;
		
		 case JK_PRE_LANDING://伸腿准备缓冲
			//动作
			jump_info->PRE_LANDING_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->PRE_LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->PRE_LANDING_length_kp;
			//事件
			if(jump_info->r_offground==false||jump_info->l_offground==false)
			{
				knee_strike_info->step2=JK_LANDING;
			}	
			if(jump_info->PRE_LANDING_tick>=jump_info->Max_PRE_LANDING_tick)
			{
				knee_strike_info->step2=JK_LANDING;
			}
			break;
			
		  case JK_LANDING://缓冲
			//动作
			jump_info->LANDING_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-5.0f*jump_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-5.0f*jump_info->Max_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->LANDING_length_kp;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->LANDING_length_speed_kp;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->LANDING_length_speed_kp;
			//事件
			if(jump_info->LANDING_tick>=jump_info->Max_LANDING_tick||
				(my_abs(jump_info->l0_average-TAR_LEG_LENGTH_INITIAL)<=jump_info->Landing_l0_range))
			{
				knee_strike_info->step2=JK_Stand_High;
////				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
////				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
			}
			break;
		
		case JK_Stand_High://立着
			//动作
			knee_strike_info->Stand_High_tick++;
			My_Chassis->target->leg_length_l = MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			My_Chassis->target->leg_length_r = MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			
			//事件
			if(knee_strike_info->thetal_average>=knee_strike_info->thetal_threshold)
			{
				knee_strike_info->step2=JK_RETRACT_2;
			}
			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
			{
				knee_strike_info->step2=JK_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				Balance.Flag->Knee_Strike_2_Flag=false;
			}
			break;
			
		case JK_RETRACT_2://收腿
			//动作
			knee_strike_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=knee_strike_info->RETRACT_length_kp;
			//事件
			if(knee_strike_info->RETRACT_tick>=knee_strike_info->Max_RETRACT_tick
				||knee_strike_info->l0_average<MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range)
			{
				knee_strike_info->step2=JK_IDLE;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				Balance.Flag->Knee_Strike_2_Flag=false;
			}
			
			break;
		
		  default:
            break;
	}
}

/**
  * @brief  走路下二级台阶
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Walk_Down_2_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	knee_strike_info->thetal_average=Rad2Angle*fabsf(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
	
	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (knee_strike_info->step1)
	{
		case Knee_IDLE:
//			if(my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) >= 0.7f || my_abs(Chassis.Posture->info->yaw - Chassis.target->yaw) >= 0.3f)
//			{
//				knee_strike_info->Idle_tick ++;
//				Balance.Flag->Return_Flag = true;
//				knee_strike_info->step1=Knee_IDLE;
//			}
//			else if(Balance.Flag->Return_Flag == false && my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) <= 0.7f && my_abs(Chassis.Posture->info->yaw - Chassis.target->yaw) <= 0.2f)
//			{
				//动作
				knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
				knee_strike_info->Stand_High_tick=0;
				//事件
				knee_strike_info->step1=Knee_Stand_High;
//			}
//			else if(knee_strike_info->Idle_tick >= 1000.f)
//			{
//				knee_strike_info->Idle_tick = 0;
//				knee_strike_info->step1=Knee_IDLE;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
//				
//				Balance.Flag->Knee_Strike_1_Flag=false;
//			}
			break;
		
		case Knee_Stand_High://立着
			//动作
			knee_strike_info->Stand_High_tick++;
		  if(My_Chassis->target->leg_length_l <= 0.23f)//0.35
			{
				My_Chassis->target->leg_length_l += 0.5f * TIME_STEP;
				My_Chassis->target->leg_length_r += 0.5f * TIME_STEP;
			}
			if(My_Chassis->target->leg_length_l >= 0.23f)
			{
				My_Chassis->target->leg_length_l = 0.23f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
				My_Chassis->target->leg_length_r = 0.23f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
			}
			My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->STAND_length_kp;
		  My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->STAND_length_kp;
			//事件
			if(knee_strike_info->thetal_average>=knee_strike_info->down_thetal_threshold)
			{
				knee_strike_info->step1=Knee_RETRACT;
				knee_strike_info->RETRACT_tick = 0;
			}
			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
			{
				knee_strike_info->step1=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
				Balance.Flag->Down_Two_Step_Flag = false;
			}
			break;
			
		case Knee_RETRACT://收腿
			//动作
			knee_strike_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=knee_strike_info->RETRACT_length_kp;
		  My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 600.f;
		  My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 600.f;
		My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
			//事件
			if(knee_strike_info->RETRACT_tick>=knee_strike_info->Max_RETRACT_tick
				|| my_abs(knee_strike_info->l0_average-MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range) <= 0.01)
			{
				knee_strike_info->Idle_tick = 0;
				knee_strike_info->step1=Knee_IDLE;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
		    My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 200.f;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 200.f;
				
				Balance.Flag->Down_Two_Step_Flag = false;
			}
			break;
		
		  default:
            break;
	}
}
//static void Walk_Down_2_Target_Process(Chassis_t* My_Chassis)
//{
//	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
//	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
//	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
//	knee_strike_info->thetab=Rad2Angle*fabsf(My_Chassis->Posture->info->pitch);
//	knee_strike_info->thetal_average=Rad2Angle*fabsf(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
//										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
//	
//	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
//	
//	switch (knee_strike_info->step1)
//	{
//		case Knee_IDLE:
////			if(my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) >= 0.7f || my_abs(Chassis.Posture->info->yaw - Chassis.target->yaw) >= 0.3f)
////			{
////				knee_strike_info->Idle_tick ++;
////				Balance.Flag->Return_Flag = true;
////				knee_strike_info->step1=Knee_IDLE;
////			}
////			else if(Balance.Flag->Return_Flag == false && my_abs(Chassis.chassis_PID->yaw_cal[L_Leg]->target) <= 0.7f && my_abs(Chassis.Posture->info->yaw - Chassis.target->yaw) <= 0.2f)
////			{
//				//动作
//				knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
//				knee_strike_info->Stand_High_tick=0;
//				//事件
//				knee_strike_info->step1=Knee_Stand_High;
////			}
////			else if(knee_strike_info->Idle_tick >= 1000.f)
////			{
////				knee_strike_info->Idle_tick = 0;
////				knee_strike_info->step1=Knee_IDLE;
////				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
////				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
////				
////				Balance.Flag->Knee_Strike_1_Flag=false;
////			}
//			break;
//			
//		case Knee_THETAB:
//			knee_strike_info->Stand_High_tick++;
//		  My_Chassis->target->thetab = 0.1f;//5.7度
//		
//			if(my_abs(knee_strike_info->thetab - 5.5f) <= 0.2f)
//			{
//				knee_strike_info->step1=Knee_Stand_High;
//				knee_strike_info->Stand_High_tick = 0;
//			}
//			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
//			{
//				knee_strike_info->step1=Knee_IDLE;
//		  My_Chassis->target->thetab = 0.f;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				Balance.Flag->Down_Two_Step_Flag = false;
//			}
//			break;
//		
//		case Knee_Stand_High://立着
//			//动作
//			knee_strike_info->Stand_High_tick++;
//		  if(My_Chassis->target->leg_length_l <= 0.23f)//0.35
//			{
//				My_Chassis->target->leg_length_l += 0.5f * TIME_STEP;
//				My_Chassis->target->leg_length_r += 0.5f * TIME_STEP;
//			}
//			if(My_Chassis->target->leg_length_l >= 0.23f)
//			{
//				My_Chassis->target->leg_length_l = 0.23f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
//				My_Chassis->target->leg_length_r = 0.23f;//MAX_LEG_LENGTH-knee_strike_info->Max_l0_range;
//			}
//			My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->STAND_length_kp;
//		  My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->STAND_length_kp;
//			//事件
//			if(knee_strike_info->thetal_average>=knee_strike_info->down_thetal_threshold)
//			{
//				Balance.mode = Handle_Mode;
//				knee_strike_info->step1=Knee_IDLE;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				Balance.Flag->Down_Two_Step_Flag = false;
//			}
//			if(knee_strike_info->Stand_High_tick>=knee_strike_info->Max_Stand_High_tick)
//			{
//				knee_strike_info->step1=Knee_IDLE;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				Balance.Flag->Down_Two_Step_Flag = false;
//			}
//			break;
//			
//		case Knee_RETRACT://收腿
//			//动作
//			knee_strike_info->RETRACT_tick++;
//		  My_Chassis->target->thetal_l = -0.7f;
//		  My_Chassis->target->thetal_r = -0.7f;
//			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
//			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range;
//			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=knee_strike_info->RETRACT_length_kp;
//		  My_Chassis->chassis_PID->length_cal[L_Leg]->kp=knee_strike_info->RETRACT_length_kp;
//		  My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 600.f;
//		  My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 600.f;
//		My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
//			//事件
//			if(knee_strike_info->RETRACT_tick>=knee_strike_info->Max_RETRACT_tick
//				|| my_abs(knee_strike_info->l0_average-MIN_LEG_LENGTH+knee_strike_info->Minimum_l0_range) <= 0.01)
//			{
//				knee_strike_info->Idle_tick = 0;
//				knee_strike_info->step1=Knee_IDLE;
//				My_Chassis->target->thetab = 0.f;
//		    My_Chassis->target->thetal_l = 0.f;
//		    My_Chassis->target->thetal_r = 0.f;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp= knee_strike_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp= knee_strike_info->IDLE_length_kp;
//		    My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 200.f;
//		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 200.f;
//				
//				Balance.Flag->Down_Two_Step_Flag = false;
//			}
//			break;
//		
//		  default:
//            break;
//	}
//}

	
/**
  * @brief  跳跃下二级台阶
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Jump_Down_2_Target_Process(Chassis_t* My_Chassis)
{
	Chassis_Knee_Strike_t* knee_strike_info=My_Chassis->knee_strike_info;
	knee_strike_info->thetal_average=Rad2Angle*fabsf(0.5f*(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal
										+My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
	Chassis_Jump_t* jump_info=My_Chassis->jump_info;
	Link_t* R_Link=My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link=My_Chassis->Leg_Unit[L_Leg]->Link;
	jump_info->r_offground=My_Chassis->Leg_Unit[R_Leg]->off_ground;
	jump_info->l_offground=My_Chassis->Leg_Unit[L_Leg]->off_ground;

	jump_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);	
	knee_strike_info->l0_average = 0.5f*(R_Link->info->length->l0+L_Link->info->length->l0);
	
	switch (knee_strike_info->step2)
	{
		case JK_IDLE:
			//动作
//			knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
//			knee_strike_info->Stand_High_tick=0;
			//事件
//			knee_strike_info->step2=JK_Stand_High;
//			break;
//		case J_IDLE:
			//动作
			jump_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
			jump_info->IDLE_length_speed_kp=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
			jump_info->IDLE_length_outmax=My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
			jump_info->IDLE_length_speed_outmax=My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;
			
			My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=1000.f;
			My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=1000.f;
			
			jump_info->COMPRESS_tick=0;
			jump_info->EXTEND_tick=0;
			jump_info->RETRACT_tick=0;
			jump_info->PRE_LANDING_tick=0;
			jump_info->LANDING_tick=0;
			knee_strike_info->Stand_High_tick=0;
			//事件
			knee_strike_info->step2=JK_EXTEND;
			break;
		
		case JK_COMPRESS://压缩腿  
			//动作
			jump_info->COMPRESS_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->COMPRESS_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->COMPRESS_length_kp;
			//事件
			#ifdef TIME_ONLY
				if(jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			jump_info->jump_step=J_EXTEND;
			#else
				if((jump_info->l0_average<=MIN_LEG_LENGTH+jump_info->Minimum_l0_range)
				|| jump_info->COMPRESS_tick>=jump_info->Max_COMPRESS_tick)
			knee_strike_info->step2=JK_EXTEND;
			#endif
			
			break;
			
		case JK_EXTEND://跳
			//动作
			jump_info->EXTEND_tick++;
      jump_info->Jump_F = 260.f;
		//事件
			#ifdef TIME_ONLY
			if(jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#else
				if((jump_info->l0_average>=0.22f)//MAX_LEG_LENGTH -jump_info->Max_l0_range)
				||jump_info->EXTEND_tick>=jump_info->Max_EXTEND_tick)
			#endif
			{
			knee_strike_info->step2=JK_RETRACT_1;
				jump_info->Jump_F = 0.f;
			}
			break;
		
		case JK_RETRACT_1://收腿滞空
			//动作
			jump_info->RETRACT_tick++;
			My_Chassis->target->leg_length_l = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->target->leg_length_r = MIN_LEG_LENGTH+jump_info->Minimum_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->RETRACT_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->RETRACT_length_kp;
			//事件
//			if(jump_info->r_offground==false&&jump_info->l_offground==false)
//			{
//				Balance.Flag->Jumping_Flag = false;
//				jump_info->jump_step=J_IDLE;
//				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
//				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
//				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
//			}	
			
			
				#ifdef NO_PRE_LANDING
				#ifdef TIME_ONLY
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#else
				if(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)<=0.f
					||jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
				#endif
				
				{
					Balance.Flag->Jumping_Flag = false;
					jump_info->jump_step=J_IDLE;
					My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
					My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
					My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
					My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				}

				#else
				if(jump_info->RETRACT_tick>=jump_info->Max_RETRACT_tick)
//					|| my_abs(jump_info->l0_average-(MIN_LEG_LENGTH+jump_info->Minimum_l0_range)) <= 0.03)
				{
					knee_strike_info->step2=JK_PRE_LANDING;
				}
				#endif
			break;
		
		 case JK_PRE_LANDING://伸腿准备缓冲
			//动作
			jump_info->PRE_LANDING_tick++;
			My_Chassis->target->leg_length_l = 0.40f;
			My_Chassis->target->leg_length_r = 0.40f;   // MAX_LEG_LENGTH-jump_info->Max_l0_range;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->PRE_LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->PRE_LANDING_length_kp;
			//事件
			if(jump_info->r_offground==false||jump_info->l_offground==false)
			{
				knee_strike_info->step2=JK_LANDING;
			}	
			if(jump_info->PRE_LANDING_tick>=jump_info->Max_PRE_LANDING_tick)
			{
				knee_strike_info->step2=JK_LANDING;
			}
			break;
			
		  case JK_LANDING://缓冲
			//动作
			jump_info->LANDING_tick++;
			My_Chassis->target->leg_length_l -= 0.06f * TIME_STEP;
			My_Chassis->target->leg_length_r -= 0.06f * TIME_STEP;
			My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->LANDING_length_kp;
		    My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->LANDING_length_kp;
			My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->LANDING_length_speed_kp;
		    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->LANDING_length_speed_kp;
			//事件
			if(jump_info->LANDING_tick>=jump_info->Max_LANDING_tick||
				(my_abs(jump_info->l0_average-MIN_LEG_LENGTH)<=0.03f))
			{
				knee_strike_info->step2=JK_IDLE;
//				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
//				Balance.Flag->Middle_Flag = true;
				Balance.Flag->Down_Two_Step_Flag = false;
			}		
			break;
		
		  default:
        break;
	}
}

/**
  * @brief  自救目标值处理
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Rescue_Target_Process(Chassis_t* My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* My_L_Leg =  My_Chassis->Leg_Unit[L_Leg]->Straight;
	Straight_Leg_t* My_R_Leg =  My_Chassis->Leg_Unit[R_Leg]->Straight;
	
	/*腿长控制*/
	My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
	My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;
	Chassis_Leg_Length_Strength_Cal(My_Chassis);
	
	/*phi0d1 PID计算*/
	if(Balance.Flag->Rescue_Trigger==true)
	{
			if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal <= 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal <= 0.f)
		{
			My_Chassis->init_state = Front;
			My_Chassis->target->vir_phi0d1_r=angle2rad(90.f);
			My_Chassis->target->vir_phi0d1_l=angle2rad(90.f);
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = Behind;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal < 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = L_Front;
			My_Chassis->target->vir_phi0d1_l=angle2rad(90.f);
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal < 0.f)
		{
			My_Chassis->init_state = R_Front;
			My_Chassis->target->vir_phi0d1_r=angle2rad(90.f);
		}
	}
	
	Chassis_Leg_phi0d1_Cal(My_Chassis);
	/*输入目标Fbl、Tp*/
	My_Chassis->Leg_Unit[L_Leg]->Link->tar_data_update(My_Chassis->Leg_Unit[L_Leg]->Link,My_Chassis->chassis_PID->length_cal[L_Leg]->out ,\
														My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->out);
	My_Chassis->Leg_Unit[R_Leg]->Link->tar_data_update(My_Chassis->Leg_Unit[R_Leg]->Link,My_Chassis->chassis_PID->length_cal[R_Leg]->out ,\
														My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->out);
	/*计算关节实际输出力矩*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
	/*debug用 begin*/
	My_L_Leg->LQR_cal(My_L_Leg);
	My_R_Leg->LQR_cal(My_R_Leg);
	Chassis_Roll_Control(My_Chassis);
	Chassis_Link_Feedforward_Cal(My_Chassis);
	/*debug用 end*/
	
}

/**
  * @brief  消去因小腿连接定子转动导致车体位移测量值不准确的误差(比如在腿进行竖直伸缩的时候，会出现位移的偏差）
  * @param  Link_Var_t* Link_Var, float thetal
  * @retval None
  */
static void Stator_Correction_Cal(Chassis_t *My_Chassis)
{
	//右腿
	float thetal=My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal;
	float phi0=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0;
	float phi3=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi3;
	Link_t* Link_Var=My_Chassis->Leg_Unit[R_Leg]->Link;
	
	//初中知识，看五连杆的图就可以算出来，需要注意方向
	//stator_angle_now为l3杆与竖直方向的夹角
	Link_Var->info->stator_correction->stator_angle_now = phi3 - phi0 - thetal;
	
	if(Link_Var->info->stator_correction->stator_angle_last == 0)
	{
		Link_Var->info->stator_correction->stator_angular_speed = 0;
	}
	else
	{
		Link_Var->info->stator_correction->stator_angular_speed = \
	(Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
	}
	
	Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;
	//伸腿时相当于轮子后退（逆时针），此处stator_angle变小，输出负的stator_bias，则s需要加上一个正的值，故Chassis_State_Var_Update里是减去
	Link_Var->info->stator_correction->stator_bias = R_STATOR_ORDER_CORRECT*Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP
														+Link_Var->info->stator_correction->stator_bias;
	
	//左腿
	thetal=My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal;
	phi0=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0;
	phi3=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi3;
	Link_Var=My_Chassis->Leg_Unit[L_Leg]->Link;
	
	Link_Var->info->stator_correction->stator_angle_now =phi3 - phi0 - thetal;
	
	if(Link_Var->info->stator_correction->stator_angle_last == 0)
	{
		Link_Var->info->stator_correction->stator_angular_speed = 0;
	}
	else
	{
		Link_Var->info->stator_correction->stator_angular_speed = \
	(Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
	}
	
	Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;

	Link_Var->info->stator_correction->stator_bias = L_STATOR_ORDER_CORRECT*Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP
														+Link_Var->info->stator_correction->stator_bias;
	
}
/**
  * @brief  底盘卸力前加阻尼保护机械结构,记得限制最大输出，或者速度大于一定值不处理
  * @param  None
  * @retval None
  */
static void Chassis_Stop_Damping(Chassis_t* My_Chassis)
{
	float torque_test = 0.f;
	
	torque_test = -My_Chassis->Wheel->motor[0]->rx_info->speed * Wheel_Damping_Coefficient;
	
	My_Chassis->Wheel->motor[0]->tx_info->torque = torque_test;
	
	torque_test = -My_Chassis->Wheel->motor[1]->rx_info->speed * Wheel_Damping_Coefficient;
	
	My_Chassis->Wheel->motor[1]->tx_info->torque = torque_test;

	torque_test = -My_Chassis->Sd->motor[0]->rx_info->speed * Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[0]->tx_info->torque = torque_test ;//+ 3;
	
	torque_test = -My_Chassis->Sd->motor[1]->rx_info->speed * Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[1]->tx_info->torque = torque_test ;//+ 3;
	
	torque_test = -My_Chassis->Sd->motor[2]->rx_info->speed * Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[2]->tx_info->torque = torque_test ;//- 3;
	
	torque_test = -My_Chassis->Sd->motor[3]->rx_info->speed * Sd_Damping_Coefficient;
	
	My_Chassis->Sd->motor[3]->tx_info->torque = torque_test ;//- 3;
}

/**
  * @brief  底盘离线数据处理
  * @param  Chassis_t* chassis, 底盘
  * @retval None
  */
static void Chassis_Offline_Process(Chassis_t* My_Chassis)
{
	Chassis.reset_struct->reset_state = Chassis_reset_NO;
	/*驱动轮测量值，目标值归零*/
	My_Chassis->Wheel->motor[0]->rx_info->motor_angle_last = 0;
	My_Chassis->Wheel->motor[0]->rx_info->motor_angle_sum = 0;
	
	My_Chassis->Wheel->motor[1]->rx_info->motor_angle_last = 0;
	My_Chassis->Wheel->motor[1]->rx_info->motor_angle_sum = 0;
	
	My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;//腿长目标值改为初始值
	My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;//腿长目标值改为初始值
	My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;//偏航角目标值等于测量值
	My_Chassis->target->vir_phi0_r = 0.f;
	My_Chassis->target->vir_phi0_l = 0.f;
	My_Chassis->chassis_PID->length_cal[R_Leg]->kp=My_Chassis->pid_init_parament->l0_length_kp;//防止命令执行过程中进入sleep模式导致pid参数不恢复
	My_Chassis->chassis_PID->length_cal[L_Leg]->kp=My_Chassis->pid_init_parament->l0_length_kp;
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=My_Chassis->pid_init_parament->l0_length_speed_kp;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=My_Chassis->pid_init_parament->l0_length_speed_kp;
	My_Chassis->target->roll = 0;
	
	My_Chassis->target->s = 0;
	
	My_Chassis->target->sd1 = 0;
	
	My_Chassis->target->yaw_v = 0;
	
	XEstimateKF_Clear(&XEstimateKF);
	/*连杆信息清零*/
	My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_bias = 0;
	My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_angle_last = 0;
	My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_bias = 0;
	My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_angle_last = 0;

	/*pid控制器清零*/
	pid_clear(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->length_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);

	pid_clear(My_Chassis->chassis_PID->roll_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->length_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
	pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
	My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
	
	
	/*命令状态机清零*/
	Balance.Flag->Jumping_Flag = false;
	Balance.Flag->Fly_Flag = false;
	Balance.Flag->Turn_Flag = false;
  Balance.Flag->Knee_Strike_1_Flag = false;
	Balance.Flag->Knee_Strike_2_Flag = false;
	Balance.Flag->Op_Fly_Flag = false;
	Balance.Flag->Cycle_Flag = false;
	Balance.Flag->Shoot_Flag = false;
	Balance.Flag->Rescue_Flag = false;
	Balance.Flag->Remedy_Flag = false;
	Balance.Flag->Return_Flag = false;
	Balance.Flag->Middle_Flag = false;	
	Balance.Flag->Middle_To_Short_Flag = false;
	Balance.Flag->Down_Two_Step_Flag = false;
	Balance.Vision.Auto_Catch_Flag = false;
	Balance.Vision.Auto_Outpost_Flag = false;
	Balance.Vision.Auto_Base_Flag = false;
	My_Chassis->jump_info->jump_step=J_IDLE;
	My_Chassis->knee_strike_info->step1=Knee_IDLE;
	My_Chassis->knee_strike_info->step2=JK_IDLE;
	My_Chassis->turn_info->step = Turn_IDLE;
	My_Chassis->fly_info->step = Fly_IDLE;
	My_Chassis->turn_info->step = Turn_IDLE;
	Balance.Chassis_Com->Save_Finish_Flag_1 = false;
	Balance.Chassis_Com->Save_Finish_Flag_2 = false;
  Balance.Chassis_Com->Save_Success_Flag = false;
	Balance.Chassis_Com->COMMON_OUTPOST_SHOOT = false;
	Balance.Chassis_Com->COMMON_BASE_SHOOT = false;
	Balance.Flag->Board_Off_Flag = false;
  Balance.ctrl = RC_CTRL;
}



/**
  * @brief  腿部角度协调计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_Sync_Cal(Chassis_t* My_Chassis)
{
	My_Chassis->chassis_PID->sync_cal[R_Leg]->target = 0;
	My_Chassis->chassis_PID->sync_cal[R_Leg]->measure = R_SYNC_ORDER_CORRECT*(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0
														-My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);
	
	My_Chassis->chassis_PID->sync_cal[L_Leg]->target = 0;
	My_Chassis->chassis_PID->sync_cal[L_Leg]->measure = L_SYNC_ORDER_CORRECT*(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0
														-My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);
	
	pid_err_cal(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[L_Leg]);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[L_Leg]->out;
}   

/**
  * @brief  双腿vir_phi0PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_vir_phi0_Cal(Chassis_t* My_Chassis)
{	
	if(Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_EXTEND || My_Chassis->jump_info->jump_step == J_RETRACT || My_Chassis->jump_info->jump_step == J_PRE_LANDING || My_Chassis->jump_info->jump_step == J_LANDING))
	{
		My_Chassis->target->vir_phi0_r = 0.f;
		My_Chassis->target->vir_phi0_l = 0.f;
	}
	if(Balance.Flag->Down_Two_Step_Flag==true && (My_Chassis->knee_strike_info->step1 == Knee_RETRACT))
	{
		My_Chassis->target->vir_phi0_r = 0.3f;
		My_Chassis->target->vir_phi0_l = 0.3f;
	}
//	else if(Balance.Flag->Jumping_Flag==true && My_Chassis->jump_info->jump_step == J_RETRACT)
//	{
//		My_Chassis->target->vir_phi0_r = 0.1f;
//		My_Chassis->target->vir_phi0_l = 0.1f;
//	}
	
	/*外环*/
	My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0;
	My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->target = My_Chassis->target->vir_phi0_r;
	
	My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0;
	My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->target = My_Chassis->target->vir_phi0_l;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
	
	/*内环*/
	
	My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->target=My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->out;
	My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->target=My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->out;
	
	My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->measure=My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1;
	My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->measure=My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0 = My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0 = My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->out;
	
}  
/**
  * @brief  双腿phi0d1PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Leg_phi0d1_Cal(Chassis_t* My_Chassis)
{
	My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1;
	My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->target = My_Chassis->target->vir_phi0d1_r;
	
	My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1;
	My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->target = My_Chassis->target->vir_phi0d1_l;
	
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
}

/**
  * @brief  驱动轮转向力矩PID计算
  * @param  Chassis_t* My_Chassis
  * @retval 力Tp
  */
static void Chassis_Wheel_Turn_Cal(Chassis_t* My_Chassis)
{
	static float yaw_tar;
	static float flag1,flag2 = 0;
	static float limit = 0;
	static uint16_t time = 0;
	if(My_Chassis->mode == C_Turn || My_Chassis->mode == C_Init || Balance.Flag->Return_Flag == 1)// || My_Chassis->mode == C_Cycle)
	{
		flag1 = 0;
	}
		
	if(flag1 == 0)
	{
		if(my_abs(gimbal.base_info.yaw_motor_angle) >= PI/2)  //此时将yaw目标角变为+-180
		{
			yaw_tar = sgn(gimbal.base_info.yaw_motor_angle) * PI;//180.f;
			flag1 = 1;
		}
		else
		{
			yaw_tar = 0.f;
			flag1 = 1;
		}
	}
	
//	if(Balance.Flag->Cycle_To_Imu_Flag == true)
//	{
//		time++;
//		if(time >= 1000)
//		{
//			Balance.Flag->Cycle_To_Imu_Flag = false;
//			time = 0;
//		}
//	}
//	
//	if(Balance.Flag->Cycle_To_Imu_Flag == true)
//	{
//		My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out_max = 5.f;
//		My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out_max = 5.f;
//	}
//	else
//	{
//		My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out_max = 20.f;
//		My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out_max = 20.f;
//	}
//	if((Balance.last_mode == Cycle_Mode && Balance.mode != Cycle_Mode && Balance.mode != Vary_Cycle_Mode)
//		|| (Balance.last_mode == Vary_Cycle_Mode && Balance.mode != Cycle_Mode && Balance.mode != Vary_Cycle_Mode))
//	{
//		flag2 = 0;
//	}
//	if(flag2 == 0)
//	{
//		yaw_tar = 0;
//		flag2 = 1;
//	}
	
	switch (My_Chassis->mode)
	{
		case C_Init:
		case C_Test:
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw;
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = My_Chassis->target->yaw;
			
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw;
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = My_Chassis->target->yaw;
			
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
			
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,2*PI);
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,2*PI);
			
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = gimbal.base_info.yaw_motor_angle / PI * 180.f;	
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = yaw_tar;
//			
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = gimbal.base_info.yaw_motor_angle / PI * 180.f;
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = yaw_tar;
//			
//			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
//			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
//			
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,360);
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,360);
//			
//			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
//			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);

			break;
		case C_Boss:
		case C_Turn:	
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw;
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = My_Chassis->target->yaw;
			
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw;
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = My_Chassis->target->yaw;
			
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
			
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,2*PI);
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,2*PI);
			
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
			break;
		case C_Knee_Strike_1:
		case C_Down_Two_Step:
		case C_Follow:
			if(Balance.Flag->Return_Flag == 1)
			{
				My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = - gimbal.base_info.yaw_motor_angle;// / PI * 180.f;	
				My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = My_Chassis->target->yaw;
//				My_Chassis->chassis_PID->yaw_cal[R_Leg]->kp = 4.f;
				My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = - gimbal.base_info.yaw_motor_angle;// / PI * 180.f;
				My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = My_Chassis->target->yaw;
//				My_Chassis->chassis_PID->yaw_cal[L_Leg]->kp = 4.f;
			}
			else
			{
				My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = - gimbal.base_info.yaw_motor_angle;// / PI * 180.f;	
				My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = yaw_tar;
//				My_Chassis->chassis_PID->yaw_cal[R_Leg]->kp = 8.f;
				My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = - gimbal.base_info.yaw_motor_angle;// / PI * 180.f;
				My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = yaw_tar;
//				My_Chassis->chassis_PID->yaw_cal[L_Leg]->kp = 8.f;
			}
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
			
			My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,2*PI);//360);
			My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,2*PI);//360);
			
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
		
		  break;
		case C_Cycle:
		case C_Vary_Cycle:
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw;	
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->target = My_Chassis->target->yaw;
//			
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw;
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->target = My_Chassis->target->yaw;
//			
//			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
//			pid_err_cal(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
//			
//			My_Chassis->chassis_PID->yaw_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[R_Leg]->err,2*PI);
//			My_Chassis->chassis_PID->yaw_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_cal[L_Leg]->err,2*PI);
//			
//			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[R_Leg]);
//			single_pid_ctrl(My_Chassis->chassis_PID->yaw_cal[L_Leg]);
		  break;
		default:
			break;
  }
	
	switch(My_Chassis->mode)
	{
		case C_Cycle:
		case C_Vary_Cycle:
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->kp = 2.5f;//Board_Rx_Info.kp;//3
		  My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->kp = 2.5f;//Board_Rx_Info.kp;
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->kd = 900.f;//Board_Rx_Info.kd;//900
		  My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->kd = 900.f;//Board_Rx_Info.kd;
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->target=My_Chassis->target->yaw;
			My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->target=My_Chassis->target->yaw;
			
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->measure=My_Chassis->Posture->info->yaw;
			My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->measure=My_Chassis->Posture->info->yaw;
		
			pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	    pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->err,2*PI);//360);
			My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->err=half_cycle(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->err,2*PI);//360);

		break;
		default:
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->kp = 8.f;
		  My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->kp = 8.f;
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->kd = 0.f;
		  My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->kd = 0.f;
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->target=My_Chassis->chassis_PID->yaw_cal[R_Leg]->out;
			My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->target=My_Chassis->chassis_PID->yaw_cal[L_Leg]->out;
	
			My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->measure=My_Chassis->Posture->info->yaw_v;//-gimbal.gimbal_y->rx_info->speed;
			My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->measure=My_Chassis->Posture->info->yaw_v;//-gimbal.gimbal_y->rx_info->speed;
		
			pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
    	pid_err_cal(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);

		break;
	}
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]);

	My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn= R_TURN_ORDER_CORRECT* My_Chassis->chassis_PID->yaw_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn= L_TURN_ORDER_CORRECT* My_Chassis->chassis_PID->yaw_speed_cal[L_Leg]->out;

	if(My_Judge.info->chassis_power_buffer <= 30.f)
	{
		limit = 0.6f * My_Judge.info->chassis_power_buffer/60.f;
		
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn *= limit;
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn *= limit;
	}
	
}   
/**
  * @brief  离地检测，在Chassis_Target_Update处理离地
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Takeoff_Detect(Chassis_t* My_Chassis)
{
	Link_info_t* R_Link_info = My_Chassis->Leg_Unit[R_Leg]->Link->info;
	Link_info_t* L_Link_info = My_Chassis->Leg_Unit[L_Leg]->Link->info;
	State_info_t* R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
	State_info_t* L_Straight_info = My_Chassis->Leg_Unit[L_Leg]->Straight->info;
	
	
	/*计算左右驱动轮的支持力*/
	//VMC力+腿重加速度力
	My_Chassis->Leg_Unit[L_Leg]->force->F_support = L_Link_info->force->F_bl_mea * cos(L_Straight_info->thetal) + m_l*(My_Chassis->Posture->info->z_world \
		- (1.f - L_Link_info->centroid->centriod_coefficient) * L_Link_info->length->l0_dot2 * cos(L_Straight_info->thetal));
	My_Chassis->Leg_Unit[R_Leg]->force->F_support = R_Link_info->force->F_bl_mea * cos(R_Straight_info->thetal) + m_l*(My_Chassis->Posture->info->z_world \
		- (1.f - R_Link_info->centroid->centriod_coefficient) * R_Link_info->length->l0_dot2 * cos(R_Straight_info->thetal));
//	L_Link_info->force->F_support = L_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
//                                  L_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/L_Link_info->length->l0 + \
//	                                mw * (My_Chassis->Posture->info->z_world - L_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
//	                                2*L_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                L_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                L_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));
//	
//	R_Link_info->force->F_support = R_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
//                                  R_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/R_Link_info->length->l0 + \
//	                                mw * (My_Chassis->Posture->info->z_world - R_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
//	                                2*R_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                R_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
//	                                R_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));
	
	if(Balance.Flag->Middle_Flag != true)
	{
		//右腿
		if(My_Chassis->Leg_Unit[R_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt ++;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
		}
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt >= 5)
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground = true;

			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 5;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
		}
		
		//左腿
		if(My_Chassis->Leg_Unit[L_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt ++;
		}
		else
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
		}
		if(My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt >= 5)
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground = true;

			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 5;
		}
		else
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground = false;

		}
	}
	else
	{
		//右腿
		if(My_Chassis->Leg_Unit[R_Leg]->force->F_support <= FLY_OFF_GROUND_SUPPORT)
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt ++;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
		}
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt >= 5)
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground = true;
			
			My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 5;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
		}
		
		//左腿
		if(My_Chassis->Leg_Unit[L_Leg]->force->F_support <= FLY_OFF_GROUND_SUPPORT)
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt ++;
		}
		else
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
		}
		if(My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt >= 5)
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground = true;
			
			My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 5;
		}
		else
		{
			My_Chassis->Leg_Unit[L_Leg]->off_ground = false;

		}
	}
	
	if((My_Chassis->Leg_Unit[L_Leg]->off_ground || My_Chassis->Leg_Unit[R_Leg]->off_ground) == true)
	{
   	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 300);
	}
	else
	{
   	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}
}

/**
  * @brief  腿长控制， 计算不同腿长下所需沿杆方向力F
  * @param  Chassis_t* My_Chassis
  */
static void Chassis_Leg_Length_Strength_Cal(Chassis_t* My_Chassis)
{
	static int16_t count = 0;
	Link_t* R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;
	
	static float cycle_length = 0;
//////	if(Balance.mode == Cycle_Mode )
//////	{
//////		cycle_length = 0.01f;
//////	}
//////	else
//////	{
//////		cycle_length = 0.;
//////	}
//////	if((Balance.mode == Cycle_Mode && Balance.last_mode != Cycle_Mode) || (Balance.mode == Vary_Cycle_Mode && Balance.last_mode != Vary_Cycle_Mode))
//////	{
//////		My_Chassis->chassis_PID->length_cal[R_Leg]->kp = 30.f;
//////		My_Chassis->chassis_PID->length_cal[L_Leg]->kp = 30.f;
//////	}
//////	if((Balance.last_mode == Cycle_Mode && Balance.mode != Cycle_Mode) || (Balance.mode == Vary_Cycle_Mode && Balance.last_mode != Vary_Cycle_Mode))
//////	{
//////		My_Chassis->chassis_PID->length_cal[R_Leg]->kp = 25.f;
//////		My_Chassis->chassis_PID->length_cal[L_Leg]->kp = 25.f;
//////	}
	
	/*位置环*/
	My_Chassis->chassis_PID->length_cal[R_Leg]->measure = R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_cal[L_Leg]->measure = L_Link_Var->info->length->l0;
	//****************************************************************	//****************************************************************

	if(Balance.Flag->Middle_Flag == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == true && My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
	{
		Balance.Flag->Middle_To_Short_Flag = true;
		count = 0;
	}
	if(Balance.Flag->Middle_Flag == true && Balance.Flag->Middle_To_Short_Flag == true)
	{
		count++;
		if(count >= 800)
		{
			count = 0;
			Balance.Flag->Middle_To_Short_Flag = false;
			Balance.Flag->Middle_Flag = false;
		}
	}
	/*赋予处理后的目标值 begin*/
	if(Balance.Flag->Down_Two_Step_Flag == false && Balance.Flag->Knee_Strike_1_Flag == false && Balance.Flag->Jumping_Flag == false)
	{
		if(Balance.Flag->Middle_Flag == true) //飞坡0.26
		{
			if(My_Chassis->target->leg_length_r <= 0.25f)
			{
				My_Chassis->target->leg_length_r += 0.15f * TIME_STEP;
				My_Chassis->target->leg_length_l += 0.15f * TIME_STEP;
			}
			if(My_Chassis->target->leg_length_r >= 0.25f)
			{
				My_Chassis->target->leg_length_r = 0.25f;
				My_Chassis->target->leg_length_l = 0.25f;
			}
		}
		else
		{
			if(My_Chassis->target->leg_length_r >= TAR_LEG_LENGTH_INITIAL)
			{
				My_Chassis->target->leg_length_r -= 0.15f * TIME_STEP;
				My_Chassis->target->leg_length_l -= 0.15f * TIME_STEP;
			}
			if(My_Chassis->target->leg_length_r <= TAR_LEG_LENGTH_INITIAL)
			{
				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
			}
		}
	}
	
		My_Chassis->chassis_PID->length_cal[R_Leg]->target = My_Chassis->target->leg_length_r + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
		My_Chassis->chassis_PID->length_cal[L_Leg]->target = My_Chassis->target->leg_length_l - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;

//	if(Balance.Flag->Middle_Flag == true) //飞坡0.26
//	{
//		if(My_Chassis->chassis_PID->length_cal[R_Leg]->target <= 0.30f)
//		{
//			My_Chassis->chassis_PID->length_cal[R_Leg]->target += 0.15f * TIME_STEP;
//			My_Chassis->chassis_PID->length_cal[L_Leg]->target += 0.15f * TIME_STEP;
//		}
//		if(My_Chassis->chassis_PID->length_cal[R_Leg]->target >= 0.30f)
//		{
//			My_Chassis->chassis_PID->length_cal[R_Leg]->target = 0.30f + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;//0.30f + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//			My_Chassis->chassis_PID->length_cal[L_Leg]->target = 0.30f - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;//0.30f - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//		}
//	}
//	else
//	{
//		My_Chassis->chassis_PID->length_cal[R_Leg]->target = My_Chassis->target->leg_length_r + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//		My_Chassis->chassis_PID->length_cal[L_Leg]->target = My_Chassis->target->leg_length_l - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//	}
	//****************************************************************	//****************************************************************
//  if(Balance.Flag->Down_Two_Step_Flag == false && Balance.Flag->Jumping_Flag == false && Balance.Flag->Knee_Strike_1_Flag == false && Balance.Chassis_Com->Save_Finish_Flag_2 == true)
//	{
//		if(Balance.Flag->Middle_Flag == true)//飞坡处理
//		{
//			if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true &&	My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
//			{
//				Balance.Flag->Middle_Flag = false;
//				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;

//			}
//			else
//			{
//				My_Chassis->target->leg_length_r = 0.25f;//0.30f + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//				My_Chassis->target->leg_length_l = 0.25f;//0.30f - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//			}
//		}
//		else   //普通处理
//		{
//			if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true &&	My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
//			{
//				My_Chassis->target->leg_length_r = 0.33f;
//				My_Chassis->target->leg_length_l = 0.33f;
//			}
//			else if((My_Chassis->target->leg_length_r >= TAR_LEG_LENGTH_INITIAL || My_Chassis->target->leg_length_l >= TAR_LEG_LENGTH_INITIAL)
//				      &&(My_Chassis->target->leg_length_r != TAR_LEG_LENGTH_INITIAL || My_Chassis->target->leg_length_l != TAR_LEG_LENGTH_INITIAL)
//							&& My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == false)
//			{
//				My_Chassis->target->leg_length_r -= 0.00015f;
//				My_Chassis->target->leg_length_l -= 0.00015f;
//			}
//			else
//			{
//				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL + cycle_length;
//				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL - cycle_length;
//			}
//		}
//	}
//	
//			My_Chassis->chassis_PID->length_cal[R_Leg]->target = My_Chassis->target->leg_length_r + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
//			My_Chassis->chassis_PID->length_cal[L_Leg]->target = My_Chassis->target->leg_length_l - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;

	//****************************************************************	//****************************************************************

	if(My_Chassis->chassis_PID->length_cal[L_Leg]->target > MAX_LEG_LENGTH)
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->target = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->chassis_PID->length_cal[L_Leg]->target < MIN_LEG_LENGTH)
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->target = MIN_LEG_LENGTH;
	}
	
	if(My_Chassis->chassis_PID->length_cal[R_Leg]->target > MAX_LEG_LENGTH)
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->target = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->chassis_PID->length_cal[R_Leg]->target < MIN_LEG_LENGTH)
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->target= MIN_LEG_LENGTH;
	}

	
	pid_err_cal(My_Chassis->chassis_PID->length_cal[R_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->length_cal[L_Leg]);
	/*积分分离*/
	if(fabsf(My_Chassis->chassis_PID->length_cal[R_Leg]->err) > 0.05f)
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 0;
	}
	else
	{
		My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 5;
	}
	single_pid_ctrl(My_Chassis->chassis_PID->length_cal[R_Leg]);
	
	/*积分分离*/
	if(fabsf(My_Chassis->chassis_PID->length_cal[L_Leg]->err) > 0.05f)
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 0;
	}
	else
	{
		My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 5;
	}
	single_pid_ctrl(My_Chassis->chassis_PID->length_cal[L_Leg]);
	
	
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->measure = R_Link_Var->info->length->l0_dot;//R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->measure = L_Link_Var->info->length->l0_dot;//L_Link_Var->info->length->l0;
	
	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->length_cal[R_Leg]->out;//R_Link_Var->info->length->l0;
	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->length_cal[L_Leg]->out;//L_Link_Var->info->length->l0;

	pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
	pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);
	
	single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);
	
	My_Chassis->Leg_Unit[R_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out;
	My_Chassis->Leg_Unit[L_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out;
	
}


///**
//  * @brief  赋予左右腿腿长目标值
//  * @param  Chassis_t* My_Chassis
//  * @retval None
//  */
//static void Chassis_Leg_Length_Target_Process(Chassis_t* My_Chassis)
//{
//  static bool last_fly = false;
//	static bool offland = false;
//	static uint16_t fly_tick = 0;

//	
//	if(Balance.Flag->Jumping_Flag == false && Balance.Flag->Knee_Strike_Flag == false && Balance.Flag->Rescue_Flag ==false)
//	{
//		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
//		{
//			if(Balance.Flag->Fly_Flag == true)
//			{
//				My_Chassis->target->leg_length_r += 0.001f;
//		    My_Chassis->target->leg_length_l += 0.001f;
//				
//				if(My_Chassis->target->leg_length_r >= 0.25f)
//				{
//					My_Chassis->target->leg_length_r = 0.25f;
//				}
//				if(My_Chassis->target->leg_length_l >= 0.25f)
//				{
//					My_Chassis->target->leg_length_l = 0.25f;
//				}
//			}
//			else{
//				
//				My_Chassis->target->leg_length_r = 0.29f;
//				My_Chassis->target->leg_length_l = 0.29f;
//			}
//		  
//			My_Chassis->target->thetal_r = 0.f;
//			My_Chassis->target->thetal_l = 0.f;
//			offland = true;
//		}
//		else if(My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == false && offland == true)
//		{
//			My_Chassis->target->leg_length_r -= 0.0004f;
//      My_Chassis->target->leg_length_l -= 0.0004f;
//			
//			if(Balance.Flag->Fly_Flag == true)
//			{
//				if(My_Chassis->target->leg_length_r <= MID_LEG_LENGTH)
//				{
//					My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
//				}
//				if(My_Chassis->target->leg_length_l <= MID_LEG_LENGTH)
//				{
//					My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
//				}
//				
//				if(fabs((My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 + My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0)/2 - MID_LEG_LENGTH) <= 0.01f)
//				{
//					My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
//					My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
//					
//					My_Chassis->target->thetal_r = THETAL_OFFSET;
//					My_Chassis->target->thetal_l = THETAL_OFFSET;
//					offland = false;
//				}
//			}
//			else{
//			  if(My_Chassis->target->leg_length_r <= TAR_LEG_LENGTH_INITIAL)
//				{
//					My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//				}
//				if(My_Chassis->target->leg_length_l <= TAR_LEG_LENGTH_INITIAL)
//				{
//					My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//				}
//				
//				if(fabs((My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 + My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0)/2 - TAR_LEG_LENGTH_INITIAL) <= 0.01f)
//				{
//					My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//					My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//					
//					My_Chassis->target->thetal_r = THETAL_OFFSET;
//					My_Chassis->target->thetal_l = THETAL_OFFSET;
//					
//					offland = false;
//				}
//			
//			}
//		}
//	}
//	
//	if(fly_tick>= 30000)
//	{
//		Balance.Flag->Fly_Flag = false;
//		
//		fly_tick = 0;
//	}
//  if(Balance.Flag->Fly_Flag == true && last_fly == false)
//	{
//		My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
//		My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
//		
//		fly_tick ++;
//	}
//	else if(Balance.Flag->Fly_Flag == true && last_fly == true)
//	{
//		
//		fly_tick ++;
//	}
//	else if(Balance.Flag->Fly_Flag == false && last_fly == true)
//	{
//		My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//		My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//		
//		fly_tick = 0;
//	}
//	
//	  if(Balance.Flag->Leg_length_ctrl_Flag==true)
//	  {
//			if(Balance.ctrl == RC_CTRL)
//			{
//				My_Chassis->target->leg_length_l += ((float)My_Chassis->rc_input->ch2_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
//		    My_Chassis->target->leg_length_r += ((float)My_Chassis->rc_input->ch2_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
//			}
//		  
//	  }


//	
//	/*限制腿长极限值*/
//	if(My_Chassis->target->leg_length_l > MAX_LEG_LENGTH)
//	{
//		My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
//	}
//	else if(My_Chassis->target->leg_length_l < MIN_LEG_LENGTH)
//	{

//		My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
//	}
//	
//	if(My_Chassis->target->leg_length_r > MAX_LEG_LENGTH)
//	{
//		My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
//	}
//	else if(My_Chassis->target->leg_length_r < MIN_LEG_LENGTH)
//	{

//		My_Chassis->target->leg_length_r= MIN_LEG_LENGTH;
//	}
//	
//	last_fly = Balance.Flag->Fly_Flag;

//}

//float lll,rrr,llqr,rlqr= 0;
/**
  * @brief  在所有数据更新后，计算关节、驱动轮所需力矩
  * @param  None
  * @retval None
  */
static void Chassis_Torque_Cal(Chassis_t *My_Chassis)
{
	Link_t* My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
	Link_t* My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
	Straight_Leg_t* R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
	Straight_Leg_t* L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
	/*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/

	/*-----------求Tp_t arget begin--------*/
	
	Chassis_Leg_Sync_Cal(My_Chassis);
	
	My_Theta_Save_Cal(&Chassis);//单环腿角
	
	Chassis_Leg_vir_phi0_Cal(My_Chassis);//双环腿角
	
	R_Straight->LQR_cal(R_Straight);
	L_Straight->LQR_cal(L_Straight);
	My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR=R_Straight->get_Tp(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR=L_Straight->get_Tp(L_Straight);
														
//	if(Balance.Chassis_Com->save_type != 0)                 //自救模式只控角度和控腿长
	if(Balance.Chassis_Com->Save_Finish_Flag_2 != 1 || Balance.mode == Handle_Mode)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->F_Save;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->F_Save;
	}
	#ifdef WALK_DOWN_2
	else if(Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_RETRACT || My_Chassis->jump_info->jump_step == J_PRE_LANDING))
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;		
	}
	else if(Balance.Flag->Down_Two_Step_Flag==true && (My_Chassis->knee_strike_info->step1 == Knee_RETRACT))
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR 
		                                             + My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR
		                                             + My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
	else if(Balance.Flag->Jumping_Flag==false
		     || (Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_COMPRESS || My_Chassis->jump_info->jump_step == J_EXTEND || My_Chassis->jump_info->jump_step == J_LANDING)))               
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR 
													+My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR 
	                      	+My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
	#else
	else if((Balance.Flag->Jumping_Flag==false && Balance.Flag->Down_Two_Step_Flag==false) || (Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_COMPRESS || My_Chassis->jump_info->jump_step == J_EXTEND || My_Chassis->jump_info->jump_step == J_LANDING))
		     || (Balance.Flag->Down_Two_Step_Flag==true && (My_Chassis->knee_strike_info->step2 == JK_COMPRESS || My_Chassis->knee_strike_info->step2 == JK_EXTEND || My_Chassis->knee_strike_info->step2 == JK_LANDING)))               
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target= R_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR 
													+My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target= L_TP_LQR_ORDER_CORRECT* My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR 
	                      	+My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
	else if((Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_RETRACT || My_Chassis->jump_info->jump_step == J_PRE_LANDING))
		     || (Balance.Flag->Down_Two_Step_Flag==true && (My_Chassis->knee_strike_info->step2 == JK_RETRACT_1 || My_Chassis->knee_strike_info->step2 == JK_PRE_LANDING )))               
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0
		                                             + My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
	#endif
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tp_target=My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
		My_Chassis->Leg_Unit[L_Leg]->force->Tp_target=My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
	}
	
	/*-----------求Tp_target end-----------*/
	
	/*-----------求Fb1_target begin--------*/	
	/*roll控制力计算*/
	Chassis_Roll_Control(My_Chassis);
	
		/*腿长控制力计算*/
	Chassis_Leg_Length_Strength_Cal(My_Chassis);

	/*前馈计算*/
	Chassis_Link_Feedforward_Cal(My_Chassis);
	
	/*汇总得到Fbl_target*/
	Chassis_Leg_Fbl_Cal(My_Chassis);//含跳跃和离地处理
	
	/*-----------求Fb1_target end--------*/
	
	
	
	/*-----------求Tw_target begin--------*/
	/*驱动轮转向环Tw_turn*/
	Chassis_Wheel_Turn_Cal(My_Chassis);
	My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR=R_Straight->get_Tw(R_Straight);
	My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR=L_Straight->get_Tw(L_Straight);
	/* 驱动轮电机最终输出 */
//	if(Balance.Chassis_Com->save_type != 0)//自救
	if(Balance.Chassis_Com->Save_Finish_Flag_2 != 1 || Balance.mode == Handle_Mode || My_Chassis->knee_strike_info->step1 == Knee_RETRACT)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
	}
	#ifdef WALK_DOWN_2
	else if(Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_RETRACT || My_Chassis->jump_info->jump_step == J_PRE_LANDING))
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
	}
	#else
	else if((Balance.Flag->Jumping_Flag==true && (My_Chassis->jump_info->jump_step == J_RETRACT || My_Chassis->jump_info->jump_step == J_PRE_LANDING))
		      || (Balance.Flag->Down_Two_Step_Flag==true && (My_Chassis->knee_strike_info->step2 == JK_RETRACT_1 || My_Chassis->knee_strike_info->step2 == JK_PRE_LANDING ))) 	
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
	}
	#endif
	else if(Balance.Flag->Knee_Strike_1_Flag == true && My_Chassis->knee_strike_info->step1 == Knee_RETRACT)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
		My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
	}
	else
	{
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == false)
		{
			My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
			My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR;
		}
		if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true && My_Chassis->Leg_Unit[R_Leg]->off_ground == false)
		{
			My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
			My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR;
		}
		
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
		{
			My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
			My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
		}
		
		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == false)
		{
//			if(My_Chassis->mode == C_Cycle)
//			{
//				My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = 1.5f;
//				My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = -1.5f;
//				My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR
//																											+My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
//				My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR
//																											+My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
//			}
//			else
//			{
				My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR
																											+My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
				My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR
																											+My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
//			}
		}
//		lll = My_Chassis->Leg_Unit[L_Leg]->force->Tw_target;
//		rrr = My_Chassis->Leg_Unit[R_Leg]->force->Tw_target;
		
		
//		if(My_Chassis->Leg_Unit[R_Leg]->off_ground == true )//离地处理
//		{
//			My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=0;
//		}
//		else
//		{
//			My_Chassis->Leg_Unit[R_Leg]->force->Tw_target=My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR
//			                                          +My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
//		}
//		if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true )//离地处理
//		{
//			My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=0;
//		}
//		else
//		{
//			My_Chassis->Leg_Unit[L_Leg]->force->Tw_target=My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR
//			                                          +My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
//		}
	}
	/*-----------求Tw_target end--------*/
	
	
	/*建模的Tp方向是顺时针，VMC是逆时针，所以从建模--->VMC要加个负号*/
	My_R_Link->tar_data_update(My_R_Link,My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
	My_L_Link->tar_data_update(My_L_Link,My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target,My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
	
	/*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
	My_L_Link->torque_cal(My_L_Link);
	My_R_Link->torque_cal(My_R_Link);
	
	/* 保护关节 */
//	My_Sd_Position_Nonlinear_Fix(My_Chassis);
	
	
	/* 关节电机最终输出 */
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque=My_R_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque=My_R_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque=My_L_Link->info->F_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix;
	My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque=My_L_Link->info->B_Sd_Output_Torque+My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix;
	
}

/**
  * @brief  沿腿方向力前馈补偿计算
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
float k_inertial=2.0f;
static void Chassis_Link_Feedforward_Cal(Chassis_t* My_Chassis)
{
	Link_t* R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
	Link_t* L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;
	State_info_t* R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;

	
	/*重力前馈*/
	//杠杆原理，质心越靠近轮子，则支持力提供的力臂越小，所以前馈要更大
	My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = (0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient*m_l) * g * cos(R_Link_Var->info->angle->vir_phi0);
	My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = (0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient*m_l) * g * cos(L_Link_Var->info->angle->vir_phi0);
	
	
	/*侧向力前馈*/
    My_Chassis->Leg_Unit[R_Leg]->force->F_inertial = R_F_INERTIAL_ORDER_CORRECT*((0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient*m_l)*(R_Link_Var->info->length->l0 \
	/ (2.f*Rl))*  My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1)*k_inertial;
	My_Chassis->Leg_Unit[L_Leg]->force->F_inertial = L_F_INERTIAL_ORDER_CORRECT*((0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient*m_l)*(L_Link_Var->info->length->l0 \
	/ (2.f*Rl))*My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1)*k_inertial;
}

/**
  * @brief  计算腿部竖直方向力（期望输出）
  * @param  None
  * @retval None
  */
static void Chassis_Leg_Fbl_Cal(Chassis_t* My_Chassis)
{
	static float t = 0;
		/*自救bu控腿长*/
	if(Balance.Chassis_Com->save_type != 0)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = 0;//My_Chassis->Leg_Unit[R_Leg]->force->F ;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = 0;//My_Chassis->Leg_Unit[L_Leg]->force->F ;
	}
	else if(Balance.Chassis_Com->save_type == 0 && Balance.Chassis_Com->Save_Finish_Flag_2 != 1)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F ;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F ;
		t = 0;
	}
	/*最开始腿角到位前不加重力前馈*/
  else if(t == 0 && (my_abs(Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0) >= 0.2f && my_abs(Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0) >= 0.2f))
	{
	  My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	 My_Chassis->Leg_Unit[R_Leg]->force->F;
	  My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	 My_Chassis->Leg_Unit[L_Leg]->force->F;
		
	}
	/* 正常运动 */
	else if(Balance.Flag->Jumping_Flag==false && Balance.Flag->Down_Two_Step_Flag==false &&
		(My_Chassis->Leg_Unit[R_Leg]->off_ground == false||My_Chassis->Leg_Unit[L_Leg]->off_ground ==false))
	{
		t = 1;
	  My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[R_Leg]->force->F
//														+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
														+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity
														+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
	  My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target =	  My_Chassis->Leg_Unit[L_Leg]->force->F
//														+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
														+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity
														+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
	}
  /*离地时希望伸长腿，所以额外加上前馈*/
	else if (Balance.Flag->Jumping_Flag==false && Balance.Flag->Down_Two_Step_Flag==false && My_Chassis->Leg_Unit[R_Leg]->off_ground == true &&My_Chassis->Leg_Unit[L_Leg]->off_ground ==true)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F 
																										+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F 
																										+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
	}
	#ifdef WALK_DOWN_2
		/* 走下2级台阶时 */
	else if(Balance.Flag->Down_Two_Step_Flag == true)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F 
																                    + My_Chassis->Leg_Unit[R_Leg]->force->F_inertial
																										+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
		My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F 
																                    + My_Chassis->Leg_Unit[L_Leg]->force->F_inertial
																										+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
	}
	/* 跳跃时 */
	else if(Balance.Flag->Jumping_Flag==true)
	{
		if(My_Chassis->jump_info->jump_step == J_EXTEND)
		{
		  My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->jump_info->Jump_F
		                                                    + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
		  My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->jump_info->Jump_F
		                                                    + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
		}
		else if(My_Chassis->jump_info->jump_step == J_COMPRESS || My_Chassis->jump_info->jump_step == J_LANDING)
		{
			My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F 
																												+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
			My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F 
																												+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F ;
			My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F ;
		}
	}

	#else
	/* 跳跃时 */
	else if(Balance.Flag->Jumping_Flag==true || Balance.Flag->Down_Two_Step_Flag==true )
	{
		if(My_Chassis->jump_info->jump_step == J_EXTEND || My_Chassis->knee_strike_info->step2 == JK_EXTEND)
		{
		  My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->jump_info->Jump_F
		                                                    + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
		  My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->jump_info->Jump_F
		                                                    + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
		}
		else if((My_Chassis->jump_info->jump_step == J_COMPRESS || My_Chassis->jump_info->jump_step == J_LANDING)
			      || (My_Chassis->knee_strike_info->step2 == JK_COMPRESS || My_Chassis->knee_strike_info->step2 == JK_PRE_LANDING || My_Chassis->knee_strike_info->step2 == JK_LANDING))
		{
			My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F 
																												+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
			My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F 
																												+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
		}
		else
		{
			My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F ;
			My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F ;
		}
	}
	#endif
	

}

/**
  * @brief  roll控制
  * @param  Link_Var_t* Link_Var
  * @retval 力F
  * @note  右腿减左腿加
  */
static void Chassis_Roll_Control(Chassis_t* My_Chassis)
{
	
	My_Chassis->chassis_PID->roll_cal[R_Leg]->measure = My_Chassis->Posture->info->roll;
	
	My_Chassis->chassis_PID->roll_cal[L_Leg]->measure = My_Chassis->Posture->info->roll;
	
	My_Chassis->chassis_PID->roll_cal[R_Leg]->target = My_Chassis->target->roll;
  
    My_Chassis->chassis_PID->roll_cal[L_Leg]->target = My_Chassis->target->roll;	

	pid_err_cal(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[R_Leg]);
	
	pid_err_cal(My_Chassis->chassis_PID->roll_cal[L_Leg]);
	single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[L_Leg]);
		My_Chassis->Leg_Unit[R_Leg]->force->F_roll = R_TP_Roll_ORDER_CORRECT*My_Chassis->chassis_PID->roll_cal[R_Leg]->out;
	
		My_Chassis->Leg_Unit[L_Leg]->force->F_roll = L_TP_Roll_ORDER_CORRECT*My_Chassis->chassis_PID->roll_cal[L_Leg]->out;

	if(My_Chassis->Leg_Unit[L_Leg]->off_ground == true && My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->F_roll = 0;
		My_Chassis->Leg_Unit[L_Leg]->force->F_roll = 0;
	}
		
	}

/**
  * @brief  Phi1由前关节电机编码器值转换得到
  * @param  Link_e My_Link_e 杆的左右标志位
  * @param  Motor_Ktech_t* my_motor 前关节电机
  * @retval 转换后的角度phi1
  */
static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor)
{
	float phi1 = 0.f;
	
	
	if(My_Leg_e == R_Leg)
	{
		phi1 = R_F_TIME*my_motor->rx_info->motor_angle + R_F_HORIZON_ANGLE_ORDER_CORRECT* R_F_HORIZON_ANGLE;
		
		
		
	}
	else if(My_Leg_e == L_Leg)
	{
		phi1 = L_F_TIME*my_motor->rx_info->motor_angle + L_F_HORIZON_ANGLE_ORDER_CORRECT* L_F_HORIZON_ANGLE;
		
	
	}
	
	
	
	if(phi1 > PI)
	{
		phi1 -= 2* PI;
	}
	if(phi1 < -PI)
	{
		phi1 += 2*PI;
	}
	
	if(phi1 >= 0)
	{
		phi1 = (float)PI - phi1;
	}
	if(phi1 < 0)
	{
		phi1 = -(float)PI - phi1;
	}
	
	return phi1;
}


/**
  * @brief  Phi4由后关节电机编码器值转换得到
  * @param  Link_e My_Link_e 杆的左右标志位
  * @param  Motor_Ktech_t* my_motor 前关节电机
  * @retval 转换后的角度phi4
  */
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t* my_motor)
{
	float phi4 = 0.f;
	
	if(My_Leg_e == R_Leg)
	{
		phi4 = R_B_TIME*my_motor->rx_info->motor_angle +R_B_HORIZON_ANGLE_ORDER_CORRECT* R_B_HORIZON_ANGLE;
	
		
	}
	else if(My_Leg_e == L_Leg)
	{
		phi4 = L_B_TIME*my_motor->rx_info->motor_angle +L_B_HORIZON_ANGLE_ORDER_CORRECT* L_B_HORIZON_ANGLE;
		
	}
	
	
	
	if(phi4 > PI)
	{
		phi4 -= 2*PI;
	}
	if(phi4 < -PI)
	{
		phi4 += 2*PI;
	}
	
	return phi4;
}



/**
  * @brief  底盘电机组离线检测
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Motor_Group_Offline_Check(Chassis_t* My_Chassis)
{
	if(My_Chassis->Sd->motor[0]->state->status == DEV_OFFLINE || My_Chassis->Sd->motor[2]->state->status == DEV_OFFLINE \
		 || My_Chassis->Sd->motor[3]->state->status == DEV_OFFLINE || My_Chassis->Sd->motor[1]->state->status == DEV_OFFLINE)
	{
		My_Chassis->state->sd_state = DEV_OFFLINE;
	}
	else
	{
		My_Chassis->state->sd_state = DEV_ONLINE;
	}
	if(My_Chassis->Wheel->motor[0]->state->status == DEV_OFFLINE || My_Chassis->Wheel->motor[1]->state->status == DEV_OFFLINE)
	{
		My_Chassis->state->wheel_state = DEV_OFFLINE;
	}
	else
	{
		My_Chassis->state->wheel_state = DEV_ONLINE;
	}
}

/**
  * @brief  底盘转向目标值控制，角速度积分成角度
  * @param  Chassis_t* My_Chassis
  * @retval None  如果换头，则锁底盘方向，如果不换，则正常判断模式
  */
static void Chassis_Yaw_Target_Process_All(Chassis_t* My_Chassis)
{
	if(Balance.Flag->Turn_Flag != 1)
	{
		switch(My_Chassis->mode)
		{
			case C_Sleep:
			My_Chassis->target->yaw_v = 0;
			break;
			
			case C_Follow:
				if(Balance.Flag->Return_Flag == 1)
				{
					static uint16_t t;
					My_Chassis->target->yaw = 0;
					t++;
					if(my_abs(My_Chassis->Posture->info->yaw - My_Chassis->target->yaw) <= 0.1 || t >= 1000)
					{
						Balance.Flag->Return_Flag = 0;
						t = 0;
					}
				}
				
			break;
			
			case C_Boss:
				if(Balance.ctrl != KEY_CTRL)
				{
			    My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch2_now / 660.f) * MAX_SPIN_SPEED;	
				}
				else 
				{
					My_Chassis->target->yaw_v = -((float)My_Chassis->key_input->all_ad_now / 660.f) * MAX_SPIN_SPEED;	
				}
        		
			break;
			
			case C_Test:
			  My_Chassis->target->yaw_v = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
			break;
					
			default:
			break;
		}
	}
	else
	{
		My_Chassis->target->yaw_v = 0;
	}
	
		/* 转向功率限制 begin */
	#if Power_limit == 1
	static float limit;
	if(My_Judge.info->chassis_power_buffer <= 30.f)
	{
		limit = My_Judge.info->chassis_power_buffer/60.f;
		
		My_Chassis->target->yaw_v *= limit;
	}
	#endif

	//My_Chassis->target->yaw目标值输出
	if(fabsf(My_Chassis->target->yaw_v) >= MAX_SPIN_SPEED/660.f/10.f)
	{
		My_Chassis->target->yaw += My_Chassis->target->yaw_v*TIME_STEP;
		My_Chassis->target->yaw=half_cycle(My_Chassis->target->yaw,2*PI);

	}
	
}
/**
  * @brief  赋予左右腿腿长目标值
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Leg_Length_Target_Process(Chassis_t* My_Chassis)
{

	if(Balance.Flag->Leg_length_ctrl_Flag==true )
	{
		My_Chassis->target->leg_length_l += ((float)My_Chassis->rc_input->ch1_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
		My_Chassis->target->leg_length_r += ((float)My_Chassis->rc_input->ch1_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
	}
	
	/*限制腿长极限值*/
	if(My_Chassis->target->leg_length_l > MAX_LEG_LENGTH)
	{
		My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->target->leg_length_l < MIN_LEG_LENGTH)
	{

		My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
	}
	
	if(My_Chassis->target->leg_length_r > MAX_LEG_LENGTH)
	{
		My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
	}
	else if(My_Chassis->target->leg_length_r < MIN_LEG_LENGTH)
	{

		My_Chassis->target->leg_length_r= MIN_LEG_LENGTH;
	}

}	

/**
  * @brief  由控制层到电机层，将计算出来的扭矩赋给相应电机
  * @param  None
  * @retval None
  */
static void Chassis_Set_Torque(Chassis_t* My_Chassis)
{
	#ifdef TEST
		My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque * R_F_ORDER_CORRECT  +  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front;  
		My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque * R_B_ORDER_CORRECT  -  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back;
		My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque * L_F_ORDER_CORRECT  -  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front;
		My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque * L_B_ORDER_CORRECT  +  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back;
	#else
	if(Balance.mode == Handle_Mode || Balance.Chassis_Com->save_type != 0)
	{
		My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque * R_F_ORDER_CORRECT ;// +  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front;  
		My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque * R_B_ORDER_CORRECT ;// -  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back;
		My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque * L_F_ORDER_CORRECT ;// -  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front;
		My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque * L_B_ORDER_CORRECT ;// +  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back;
	}
	else if(Balance.Chassis_Com->COMMON_OUTPOST_SHOOT == true || Balance.mode == Lob_Mode )
	{
		My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = 0.f;  
		My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = 0.f; 
		My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = 0.f; 
		My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = 0.f; 
	}
	else
	{
		My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque * R_F_ORDER_CORRECT  +  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front;  
		My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque * R_B_ORDER_CORRECT  -  My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back;
		My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque * L_F_ORDER_CORRECT  -  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front;
		My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque * L_B_ORDER_CORRECT  +  My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back;
	}
	#endif
	
	if(Balance.Chassis_Com->COMMON_OUTPOST_SHOOT == true || Balance.mode == Lob_Mode)
	{
		My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = 0.f;
		My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = 0.f;
	}
	else
	{
		My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Tw_target*R_W_ORDER_CORRECT;
		My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Tw_target*L_W_ORDER_CORRECT;
	}
}

/**
  * @brief  电机层目标力矩赋0，不发送
  */
static void Chassis_Motor_Set_Sleep(Chassis_t* My_Chassis)
{
	My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = 0;
	My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = 0;
	
	My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;
	My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;
}

/**
  * @brief  底盘输入值步进限幅滤波
  * @param  Chassis_t* My_Chassis
  * @retval None
  */
static void Chassis_Rc_Input_Update(Chassis_t* My_Chassis)
{
	Chassis_Rc_Input_t* rc_input = My_Chassis->rc_input;
	
	/*偏航*/
	rc_input->ch0_now = rc_sensor.info->ch0;
	rc_input->ch0_now=step_limit_filter(rc_input->ch0_now,rc_input->ch0_last,1);
	rc_input->ch0_last = rc_input->ch0_now;
	
	/*keep null*/
	rc_input->ch1_now = rc_sensor.info->ch1;
//	rc_input->ch1_now=step_limit_filter(rc_input->ch1_now,rc_input->ch1_last,10);
//	rc_input->ch1_last = rc_input->ch1_now;
	
	/*腿长*/
	rc_input->ch2_now = rc_sensor.info->ch2;
//	rc_input->ch2_now=step_limit_filter(rc_input->ch0_now,rc_input->ch2_last,10);
//	rc_input->ch2_last = rc_input->ch2_now;

	
	/*前后*/
	rc_input->ch3_now = rc_sensor.info->ch3;
	rc_input->ch3_now=step_limit_filter(rc_input->ch3_now,rc_input->ch3_last,2);
	rc_input->ch3_last = rc_input->ch3_now;
	
}


static void Chassis_Key_Input_Update(Chassis_t* My_Chassis)
{
	Chassis_Key_Input_t* key_input = My_Chassis->key_input;
	float k_ratio;//与最大速度有关的系数
	
//	key_input->w_now = rc_sensor.info->W.cnt;
//	key_input->s_now = rc_sensor.info->S.cnt;
	key_input->all_ws_now = (rc_sensor.info->W.cnt - rc_sensor.info->S.cnt);
	if(abs(key_input->all_ws_now - key_input->all_ws_last) > 1)
	{
		key_input->all_ws_now = 2.0*sgn(key_input->all_ws_now - key_input->all_ws_last) + key_input->all_ws_last;//应该跟控制频率有关
		key_input->all_ws_last = key_input->all_ws_now;
	}
	else
	{
		key_input->all_ws_last = key_input->all_ws_now;
	}
	
	key_input->all_ad_now = (rc_sensor.info->A.cnt - rc_sensor.info->D.cnt);//往左是正
//	if(abs(key_input->all_ad_now - key_input->all_ad_last) > 1)
//	{
//		key_input->all_ad_now = 2.0*sgn(key_input->all_ad_now - key_input->all_ad_last) + key_input->all_ad_last;
//		key_input->all_ad_last = key_input->all_ad_now;
//	}
//	else
//	{
//		key_input->all_ad_last = key_input->all_ad_now;
//	}
}

/**
  * @brief  控速度
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_sd1_Target_Update(Chassis_t* My_Chassis)
{
  if(My_Chassis->mode == Cycle_Mode || My_Chassis->mode == Vary_Cycle_Mode)
	{
		 float target_x,target_y;
		 float move_speed_x,move_speed_y,move_angle,angle_err;
		
		angle_err = gimbal.base_info.yaw_motor_angle;
		if(fabs(angle_err) >= PI)
		{
			angle_err -= sgn(angle_err)*2*PI;
		}
		move_angle = angle_err;
		
		if(Balance.ctrl == RC_CTRL)
		{
		target_x = -(My_Chassis->rc_input->ch3_now / 660.f) * 1.8f;
		target_y = -(My_Chassis->rc_input->ch2_now / 660.f) * 1.8f;
		}
		else if(Balance.ctrl == KEY_CTRL)
		{
			target_x = -(My_Chassis->key_input->all_ws_now / KEY_W_CNT_MAX)*1.8f;
//			target_y = -(My_Chassis->key_input->all_ad_now / KEY_A_CNT_MAX)*1.8f;
		}
		move_speed_x = target_x * arm_sin_f32(move_angle);
//		move_speed_y = target_y * arm_cos_f32(move_angle);
		
		My_Chassis->target->sd1 = move_speed_x;// + move_speed_y;
	}
	else
	{
		if(Balance.ctrl != KEY_CTRL)
		{
			My_Chassis->target->sd1 = RC_INPUT_SD1_ORDER_CORRECT* ((float)My_Chassis->rc_input->ch3_now / 660.f) * MAX_STRAIGHT_SPEED;
		}
		else
		{
			My_Chassis->target->sd1 = RC_INPUT_SD1_ORDER_CORRECT* ((float)My_Chassis->key_input->all_ws_now / 330.f/*测测不同数值？*/) * MAX_STRAIGHT_SPEED;
		}
			
		if (fabsf(gimbal.base_info.yaw_motor_angle) >= PI/2) 
		{
			My_Chassis->target->sd1 *= -1;
		}
	}
		/* 平移功率限制 begin */
	#if Power_limit == 1
//	float my_k = 1.f;
//	
//	my_k = My_Chassis_Power_Limit();
//	
	static uint8_t first_limit_flag = 0;
	static uint8_t end_limit_flag = 1;
	 if(My_Judge.org_info->power_heat_data.buffer_energy <= 25.f && My_Judge.org_info->power_heat_data.buffer_energy > 0.f)//范围
		{
			if(first_limit_flag == 0)//第一次进入
			{
				first_limit_flag = 1;
        My_Chassis->target->limit_v = MAX_STRAIGHT_SPEED;
//				My_Chassis.target->velocity_limit = abs(My_Chassis.target->velocity);
			}
			end_limit_flag = 0;
//	    Balance.Flag->chassis->Power_Limit_Flag = true;
			
		}
		else if(My_Judge.org_info->power_heat_data.buffer_energy <= 0.f)
		{
//			My_Balance.command->chassis->Over_Power_Flag = true;
			My_Chassis->target->limit_v = 0.4f;
			
		}
		else
		{
//			My_Balance.command->chassis->Power_Limit_Flag = false;
//			My_Balance.command->chassis->Over_Power_Flag = false;
			end_limit_flag = 1;
			first_limit_flag = 0;
		}

	Chassis_Power_Limit();
		
	if(end_limit_flag == 0)
	{
	  My_Chassis->target->sd1 = My_Chassis->target->sd1 * My_Chassis->target->limit_v / MAX_STRAIGHT_SPEED;
	}
	
		
//	if(/*my_k == 0 ||*/ My_Chassis->mode == Cycle_Mode || My_Chassis->mode == Vary_Cycle_Mode
//		|| (float)fabsf(My_Chassis->rc_input->ch3_now / 660.f)>=0.2f || (float)fabsf(My_Chassis->key_input->all_ws_now / 660.f)>=0.1f)
//	{
//		My_Chassis->target->s = My_Chassis->Leg_Unit[L_Leg]->Straight->info->s;
//	}
	#endif
	/* 平移功率限制 end */

	if((float)fabsf(My_Chassis->rc_input->ch3_now / 660.f)>=0.2f || (float)fabsf(My_Chassis->key_input->all_ws_now / 660.f)>=0.2f)
	{
		My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s + 0.6f;
	}
	if(/*my_k == 0 ||*/ My_Chassis->mode == Cycle_Mode || My_Chassis->mode == Vary_Cycle_Mode)
	{
		My_Chassis->target->s = (My_Chassis->Leg_Unit[L_Leg]->Straight->info->s + My_Chassis->Leg_Unit[R_Leg]->Straight->info->s) / 2.f;
	}
	
	if((My_Chassis->Leg_Unit[L_Leg]->off_ground == true && My_Chassis->Leg_Unit[R_Leg]->off_ground == true) )//&& (My_Chassis.Knee_Strike_Flag_1 != true ))
	{
		if(gimbal.base_info.yaw_mec_angle_target != 0)
		{
		  My_Chassis->rc_input->ch3_last = -(int16_t)(XEstimateKF.FilteredValue[1] * 660.f / MAX_STRAIGHT_SPEED);//速度衔接
		}
		else if(gimbal.base_info.yaw_mec_angle_target == 0)//原来有方向的
		{
			My_Chassis->rc_input->ch3_last = (int16_t)(XEstimateKF.FilteredValue[1] * 660.f / MAX_STRAIGHT_SPEED);
		}

		if(gimbal.base_info.yaw_mec_angle_target != 0)
		 {
			  My_Chassis->key_input->all_ws_last = -(int16_t)(XEstimateKF.FilteredValue[1] * (rc_sensor.info->W.cnt_max) / MAX_STRAIGHT_SPEED);
		 }
		else if(gimbal.base_info.yaw_mec_angle_target == 0)//原来有方向的
		 {
			  My_Chassis->key_input->all_ws_last = (int16_t)(XEstimateKF.FilteredValue[1] * (rc_sensor.info->W.cnt_max) / MAX_STRAIGHT_SPEED);
		 }
  }
}


/**
  * @brief  底盘初始化控制处理                      看起来要改
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Init_Ctrl(Chassis_t* My_Chassis)
{
	My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
	My_Chassis->Leg_Unit[L_Leg]->off_ground = false;
	
//	if(My_Chassis->mode == C_Init)
//	{
//		if(My_Chassis->init_state == Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == Behind)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == L_Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//		else if(My_Chassis->init_state == R_Front)
//		{
//			My_Chassis->target->leg_length = MIN_LEG_LENGTH;
//		}
//	}
	My_Chassis->target->yaw_v = 0;
	My_Chassis->target->sd1 = 0;
	My_Chassis->target->roll = 0;
	My_Chassis->target->yaw = My_Chassis->Posture->info->yaw;//偏航角目标值等于测量值
	

	float thetal_err = My_Chassis->Leg_Unit[L_Leg]->Straight->X_info->X_err_mat_storage[X_thetal];
	float thetar_err = My_Chassis->Leg_Unit[R_Leg]->Straight->X_info->X_err_mat_storage[X_thetal];
	static uint8_t cnt = 0;
	

	if(My_Chassis->reset_struct->reset_cnt == 0)
	{
		if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal <= 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal <= 0.f)
		{
			My_Chassis->init_state = Front;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = Behind;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal < 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal > 0.f)
		{
			My_Chassis->init_state = L_Front;
		}
		else if(My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal > 0.f && My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal < 0.f)
		{
			My_Chassis->init_state = R_Front;
		}
	}
	My_Chassis->reset_struct->reset_cnt ++;
	if((fabsf(thetal_err) < 0.75f && fabsf(thetar_err) < 0.75f) ||My_Chassis->reset_struct->reset_cnt > 800)
	{
		cnt ++;
		if(cnt > 20 || My_Chassis->reset_struct->reset_cnt > 800)
		{
			My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
			My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
			My_Chassis->reset_struct->reset_state = true;
		}
	}
	else
	{
		cnt = 0;
	}
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean);
/**
  * @brief  目标速度限制以限制向心力
  * @param  Chassis_t* My_Chassis, 底盘
  * @retval None
  */
static void Chassis_Speed_Limit(Chassis_t* My_Chassis)
{
	 
	float F_acc, Fw, Ff_cal, Ff_max;
	static float limit_coe_arr[10] = {0.f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
	float limit_coe = 1.f;
	
	if(My_Chassis->target->yaw_v*My_Chassis->target->sd1 > 0)
	{
		if(My_Chassis->target->sd1 > 0)
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
		else
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
		F_acc = My_Chassis->Posture->info->x_world * mw;
		Fw = mw*My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1*My_Chassis->Posture->info->yaw_v;
		arm_sqrt_f32((F_acc*F_acc + Fw*Fw), &Ff_cal);
		if(Ff_cal > Ff_max)
		{
			limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
//			My_Chassis->target->yaw_v *= limit_coe;
			My_Chassis->target->sd1 = My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1*limit_coe;
		}
	}
	else if(My_Chassis->target->yaw_v*My_Chassis->target->sd1 < 0)
	{
		if(My_Chassis->target->sd1 > 0)
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
		else
		Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
		F_acc = My_Chassis->Posture->info->x_world * mw;
		Fw = mw*My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1*My_Chassis->Posture->info->yaw_v;
		arm_sqrt_f32((F_acc*F_acc + Fw*Fw), &Ff_cal);
		if(Ff_cal > Ff_max)
		{
			limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
//			My_Chassis->target->yaw_v *= limit_coe;
			My_Chassis->target->sd1 = My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1*limit_coe;
		}
	}
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean)
{
	int low = 0;
	int high = size - 1;
	int result = 0;  // 用于记录满足条件的最后一个索引

	while (low <= high)
	{
		int mid = low + (high - low) / 2;  // 防止溢出

		// 如果当前元素小于 target，则记录下索引，并向右边继续查找更大的数
		if ((arr[mid]*mean) < target)
		{
			result = mid;
			low = mid + 1;
		}
		// 如果 arr[mid] 大于等于 target，则需要向左侧寻找
		else
		{
			high = mid - 1;
		}
	}
return result;
}



static void My_Sd_Position_Nonlinear_Fix(Chassis_t* My_Chassis)
{
	/*
	phi1为前关节，水平角度为180度，角度往上限位方向递增
	
	phi4为后关节，水平角度为0度，角度往上限位方向递减
	*/
	
	float phi1_gate = 0.f, phi4_gate = 0.f;
	

	phi1_gate = L_PHI1_UP_ANGLE - LIMIT_RANGE;
	phi1_gate=half_cycle(phi1_gate,360.f);
	phi4_gate = L_PHI4_UP_ANGLE + LIMIT_RANGE;
	phi4_gate=half_cycle(phi4_gate,360.f);
	
	//右腿
	float phi1_transformed;
	if(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1_<0)
	{
		phi1_transformed=360.f+My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1_;
	}
	if( phi1_transformed> phi1_gate)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix = ((phi1_transformed - phi1_gate) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix = 0.f;
	}
	
	if(My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4_ < phi4_gate)
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix = ((My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4_ - (phi4_gate)) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix = 0.f;
	}
	
	//左腿
	if(My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1_<0)
	{
		phi1_transformed=360.f+My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1_;
	}
	if(phi1_transformed > phi1_gate)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix = ((phi1_transformed - phi1_gate) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix = 0.f;
	}
	
	if(My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4_ < phi4_gate)
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix = ((My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4_ - (phi4_gate)) / LIMIT_RANGE) * SD_POS_FIX_TOR_K;
	}
	else
	{
		My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix = 0.f;
	}
	
}

/*小陀螺变速小陀螺*/
float Cycle_Speed = 0.f;
static void Cycle_Target_Process(Chassis_t* My_Chassis)
{
//	My_Chassis.target->yaw_angle -= (My_Chassis.Rc_Input->ch0_now/660.f) * MAX_SPIN_SPEED *TIME_STEP;
  static int8_t vary_flag = 1;
	if(My_Chassis->mode == C_Cycle)
	{    
 		Cycle_Speed = Cycle_Speed + 0.2;
		
	  if(fabsf(My_Chassis->target->velocity_y) >= 0.1f || fabsf(My_Chassis->target->sd1) >= 0.1f)
		{
	   	Cycle_Speed = 7.f;
	  }
	  if(Cycle_Speed >= 7.f)
	  {
	 	  Cycle_Speed = 5.f;
	  }	
	}		
  else if(My_Chassis->mode == C_Vary_Cycle)
	{
		if(Cycle_Speed >= 7.f && vary_flag == 1)
		{
			vary_flag = -1;
		}
		else if(Cycle_Speed <= 4.f && vary_flag == -1)
		{
			vary_flag = 1;
		}
		if(vary_flag == 1)
		{
			Cycle_Speed += 0.02;
		}
		else if(vary_flag == -1)
		{
			Cycle_Speed -= 0.02;
		}
	}		
	My_Chassis->target->yaw -= Cycle_Speed * TIME_STEP;

	//限制角度差
	if(fabsf(My_Chassis->target->yaw - My_Chassis->Posture->info->yaw) >= 0.7f)
	{
	  My_Chassis->target->yaw = My_Chassis->Posture->info->yaw - 0.7f;
	}
//	else
//	{
//		Cycle_Speed = 6.f;
//	}

	if(fabsf(My_Chassis->target->yaw) > PI)//防止疯车
	{
		My_Chassis->target->yaw -= 2*PI * sgn(My_Chassis->target->yaw);
	}
}


/*氮气弹簧动态前馈*/
void My_Spring_Former_Input_Cal(Chassis_t* My_Chassis)
{
	static float Alpha_R,Belta_R;//Alpha是腿交点,Belta是腿延长线交点
	static float Alpha_L,Belta_L;
	static float Spring_Force = 400.f;  // * g;//400N
	
	static float feed_f_l,feed_f_r,feed_b_l,feed_b_r;
	
	feed_f_l = 0.f;
	feed_f_r = 0.f;
	feed_b_l = 0.f;
	feed_b_r = 0.f;
	if((Balance.Flag->Jumping_Flag==true && My_Chassis->jump_info->jump_step == J_RETRACT) || My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 <= 0.16f)
	{
		feed_f_l = 7.f;
	}
	else
	{
		feed_f_l = 7.f;
	}
	if((Balance.Flag->Jumping_Flag==true && My_Chassis->jump_info->jump_step == J_RETRACT) || My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.16f)
	{
		feed_f_r = 6.f;
	}
	else
	{
		feed_f_r = 3.f;
	}

	if(My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 >= 0.2f && My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 <= 0.36f)
	{
		feed_b_l = 2.5f;
	}
	else
	{
		feed_b_l = 0.f;
	}
	if(My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 >= 0.2f && My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.36f)
	{
		feed_b_r = 2.5f;
	}
	else
	{
		feed_b_r = 0.f;
	}
	Alpha_R = PI - My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1 + My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi2;
	Belta_R = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi2 - My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4;
	
	Alpha_L = PI - My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1 + My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi2;
	Belta_L = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi2 - My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4;
	
	My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Front = (Spring_Force * 0.06f * 0.095f / 0.115f) * arm_sin_f32(Alpha_R + 0.260403f) *arm_cos_f32(Belta_R)  + feed_f_r;
	My_Chassis->Leg_Unit[R_Leg]->Link->info->force->T_Feed_Back = Spring_Force * arm_sin_f32(1.22173047f) * 0.04715f + feed_b_r;
	
	My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Front = (Spring_Force * 0.06f * 0.095f / 0.115f) * arm_sin_f32(Alpha_L + 0.260403f) *arm_cos_f32(Belta_L)  + feed_f_l;
	My_Chassis->Leg_Unit[L_Leg]->Link->info->force->T_Feed_Back = Spring_Force * arm_sin_f32(1.22173047f) * 0.04715f + feed_b_l;
	
}


static void Chassis_Motor_Target_Angle(Chassis_t* My_Chassis)
{
//	Four_Bar_Link_t* My_L_Link = &My_Chassis->Link[L_Link];
//	Four_Bar_Link_t* My_R_Link = &My_Chassis->Link[R_Link];
//	Straight_Leg_t* My_Leg = My_Chassis->Leg;
//	
//	My_L_Link->angle_cal(My_L_Link);
//	My_R_Link->angle_cal(My_R_Link);
//	
//	My_Chassis->Sd->motor[2]->tx_info->target_angle = R_F_HORIZON_ANGLE+R_F_TIME*My_R_Link->info->angle->target_phi1;
//	My_Chassis->Sd->motor[3]->tx_info->target_angle = R_B_HORIZON_ANGLE+R_B_TIME*My_R_Link->info->angle->target_phi4;
//	My_Chassis->Sd->motor[1]->tx_info->target_angle = L_F_HORIZON_ANGLE+L_F_TIME*My_L_Link->info->angle->target_phi1;
//	My_Chassis->Sd->motor[0]->tx_info->target_angle = L_B_HORIZON_ANGLE+L_B_TIME*My_L_Link->info->angle->target_phi4;
//	
//	if(Balance.command->chassis->LQR_Model == false)
//	{
//		My_Chassis->Sd->motor[2]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[2]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[1]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[1]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[0]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[0]->tx_info->Kd = 1;
//		My_Chassis->Sd->motor[3]->tx_info->Kp = 20;
//		My_Chassis->Sd->motor[3]->tx_info->Kd = 1;
//		My_Leg->T_info->T_mat_storage[Sd_L] = 0;
//		My_Leg->T_info->T_mat_storage[Sd_R] = 0;
//	}
//	else
//	{
//		My_Chassis->Sd->motor[2]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[2]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[1]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[1]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[0]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[0]->tx_info->Kd = 0;
//		My_Chassis->Sd->motor[3]->tx_info->Kp = 0;
//		My_Chassis->Sd->motor[3]->tx_info->Kd = 0;
//	}
}
//float run = 0.1f;
//float run_add = 0.1f;
//float stop = 0.1f;
//float stop_add = 0.1f;
//static void My_Chassis_KEY_Input(void)
//{
//	Chassis_Rc_Input_t* rc_input = Chassis.rc_input;
//	
//	//平移
//	if(rc_sensor.info->W.status == short_press || rc_sensor.info->W.status == long_press || rc_sensor.info->W.status == release_to_press)
//	{
//		rc_input->w_now = (float)rc_sensor.info->W.cnt;
//	}
//	else if(rc_sensor.info->W.status == press_to_release)
//	{
//		rc_input->w_now = Chassis.Leg->info->sd1*KEY_W_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
//	}
//	else if(rc_sensor.info->W.status == release)
//	{
//		if(rc_input->w_now > 0)
//		{
//			rc_input->w_now -= 10;
//			if(rc_input->w_now < 0)
//				rc_input->w_now = 0;
//		}
//		else
//		{
//			rc_input->w_now = 0;
//		}
//	}
//	
//	if(rc_sensor.info->S.status == short_press || rc_sensor.info->S.status == long_press || rc_sensor.info->S.status == release_to_press)
//	{
//		rc_input->s_now = (float)rc_sensor.info->S.cnt;
//	}
//	else if(rc_sensor.info->S.status == press_to_release)
//	{
//		rc_input->s_now = -Chassis.Leg->info->sd1*KEY_S_CNT_MAX/(float)MAX_STRAIGHT_SPEED;;
//	}
//	else if(rc_sensor.info->S.status == release)
//	{
//		if(rc_input->s_now > 0)
//		{
//			rc_input->s_now -= 10;
//			if(rc_input->s_now < 0)
//				rc_input->s_now = 0;
//		}
//		else
//		{
//			rc_input->s_now = 0;
//		}
//	}
//	//平移
//	
//	rc_input->a_now = Lowpass(rc_input->a_last,(float)rc_sensor.info->A.cnt, 0.4f);
//	rc_input->a_last = rc_input->a_now;
//	
//	//腿长
//	//平移
//	rc_input->d_now = Lowpass(rc_input->d_last,(float)rc_sensor.info->D.cnt, 0.4f);
//	rc_input->d_last = rc_input->d_now;
//}
#define CHASSIS_MAX_POWER_BUFFER  		(40.f) //功率限制阈值，当buffer小于40时开始做限制
#define CHASSIS_MID_POWER_BUFFER  		(20.f) 
#define CHASSIS_LOW_POWER_BUFFER  		(10.f) //各个值经简单调整，大量测试后没问题既可
#define CHASSIS_DANGER_POWER_BUFFER		(5.f)  //buffer低于5的时候轮子卸力，关节收腿 
static float My_Chassis_Power_Limit(void)
{
	float buff = 0.f;
	
	float limit_k = 1.f;
	
	if(My_Judge.status->status == DEV_ONLINE)
	{
		buff = My_Judge.info->chassis_power_buffer;
	}
	
	/*功率限制*/
	if(buff > CHASSIS_MAX_POWER_BUFFER)//飞坡完成后,buffer会自动变高并停留几秒
	{
		buff = CHASSIS_MAX_POWER_BUFFER;//无限制 大于40
		Balance.Chassis_Com->Power_Limit_Flag = false;
		limit_k = 1.f;
	}
	else if(buff >= CHASSIS_MID_POWER_BUFFER)//一级限制 20~40
	{
		limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
		limit_k *= 0.8f;
	}
	else if(buff >= CHASSIS_LOW_POWER_BUFFER)//二级限制 10~20
	{
		limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
		limit_k = 0.5f;
	}
	else if(buff >= CHASSIS_DANGER_POWER_BUFFER)
	{
		limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
		limit_k = 0.3f;
	}
	else//三级限制 buffer小于10
	{
//		if(Balance.Flag->Fly_Flag == true)
//	  {
//			limit_k = buff / CHASSIS_MAX_POWER_BUFFER;
//			limit_k = 0.3f;		
//		}
//		else
//		{
		Balance.Chassis_Com->Power_Limit_Flag = true;
		limit_k = 0.1f;
//		}
		
	}
	
//	My_Chassis_Power_Limit_Sd();
	if(Balance.Flag->Fly_Flag == true)
	{
		limit_k = 0.5;
	}	
	return limit_k;

}

/*功率限制*/
//static void Chassis_Power_Limit(void)
//{

//	Link_t *Link;
//	if(fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target) >= fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target))
//	{
//		Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target);
//		Link = Chassis.Leg_Unit[L_Leg]->Link;
//	}
//	else
//	{
//		Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target);
//		Link = Chassis.Leg_Unit[R_Leg]->Link;
//	}
//	
//	Power_Limit.Nf = (My_Judge.info->chassis_power_buffer/24.f* _3508_SELFMADE_TORQUE_CONSTANT/ WHEEL_RADIUS) - ((2*Power_Limit.Tp_Big / Chassis.target->leg_length_l) * fabsf(arm_cos_f32(Link->info->angle->vir_phi0))) \
//		                                    - (float)(mb*g*(Chassis.target->leg_length_l) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0))) \
//																				- (float)(mb*g*fabsf(arm_cos_f32(Link->info->angle->vir_phi0)) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0)));
//	
//	Power_Limit.kk = arm_sin_f32(Link->info->angle->vir_phi0);
//	
//	Power_Limit.a = (float)Power_Limit.Nf/mb;//质量得换整车的
//	
//	Chassis.target->limit_v = Chassis.target->limit_v + Power_Limit.a*TIME_STEP;
//	
//	if(Chassis.target->limit_v >= MAX_STRAIGHT_SPEED)
//	{
//		Chassis.target->limit_v = MAX_STRAIGHT_SPEED;
//	}
//	
//	if(Chassis.target->limit_v <= 0.4)//阈值得调
//	{
//		Chassis.target->limit_v = 0.4;
//	}
//}


static void Chassis_Power_Limit(void)
{
	static float l0_average;
	l0_average = 0.5f*(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0+Chassis.Leg_Unit[L_Leg]->Link->info->length->l0);

	Link_t *Link;
	if(fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target) >= fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target))
	{
		Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target);
		Link = Chassis.Leg_Unit[L_Leg]->Link;
	}
	else
	{
		Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target);
		Link = Chassis.Leg_Unit[R_Leg]->Link;
//		Link = &Link[R_Leg];
	}
	//
	
	Power_Limit.Nf = (My_Judge.info->chassis_power_buffer/24.f* _3508_SELFMADE_TORQUE_CONSTANT/ WHEEL_RADIUS) - ((2*Power_Limit.Tp_Big / l0_average) * fabsf(arm_cos_f32(Link->info->angle->vir_phi0))) \
		                                    - (float)(mb*g*(l0_average) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0))) \
																				- (float)(mb*g*fabsf(arm_cos_f32(Link->info->angle->vir_phi0)) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0)));
	
	Power_Limit.kk = arm_sin_f32(Link->info->angle->vir_phi0);
	
	Power_Limit.a = (float)Power_Limit.Nf/mb;//质量得换整车的
	
	Chassis.target->limit_v = Chassis.target->limit_v + Power_Limit.a*TIME_STEP;
	
	if(Chassis.target->limit_v >= MAX_STRAIGHT_SPEED)
	{
		Chassis.target->limit_v = MAX_STRAIGHT_SPEED;
	}
	
	if(Chassis.target->limit_v <= 0.4)//阈值得调
	{
		Chassis.target->limit_v = 0.4;
	}
}



// 3503-3831老自救
// 3904-4284新自救

//腿在前面伸展角度为178,往上180，-180°,0,往后伸展为18°
//而前翻的时候最大摆角大概是277
//后翻的时候最大摆角大概是284(265),给目标角340应该就能稳救回来
static float R_Leg_Theta = 0.f,L_Leg_Theta = 0.f;
static void My_Chassis_Save_Process(Chassis_t *My_Chassis)
{
	static uint16_t tumble_cnt = 0;//时间过长回到上一阶段
	static uint16_t tumble_cnt_2 = 0;
	static uint16_t wait_cnt_1 = 0;
	static uint16_t wait_cnt_2 = 0;
	static uint16_t wait_cnt_3 = 0;//等车体稳定
	static uint8_t save_must = 0;//type1,强制收腿,给个计时
	static uint8_t save_type_last = 0;
	static uint8_t first_enter = 0;
  static uint8_t finish_save = 0;
	static uint8_t type2_flag = 0;
	static uint8_t type3_flag = 0;
	
	static uint8_t flag,last_flag = 0;
	
	last_flag = flag;
	flag = Balance.Flag->Rescue_Flag;
	if(Balance.Flag->Rescue_Flag == true)
	{
		flag = 1;
	}
 	
	if(rc_sensor.work_state == DEV_OFFLINE || check_hero_revive(&My_Judge) == 1 || Balance.Flag->Chassis_Online_Flag == 0 || Balance.mode == Sleep_Mode)///////// || (flag == 1 && last_flag == 0))//遥控离线
	{
		
		first_enter = 0;
		tumble_cnt = 0;
		tumble_cnt_2 = 0;
		wait_cnt_1 = 0;
		wait_cnt_2 = 0;
		wait_cnt_3 = 0;
		save_type_last = 0;
		type2_flag = 0;
		type3_flag = 0;
		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
	}
	
	R_Leg_Theta = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0_;
	if(R_Leg_Theta <= 0)
	{
		R_Leg_Theta = 360.f + R_Leg_Theta;//转为0~360一圈
		
	}
	
	L_Leg_Theta = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0_;
	if(L_Leg_Theta <= 0)
	{
		L_Leg_Theta = 360.f + L_Leg_Theta;
	}
	
	if(Chassis.Posture->info->pitch <= -0.7f && (Chassis.Posture->info->roll <= 0.7f && Chassis.Posture->info->roll >= -0.7f))
	{
		Balance.Chassis_Com->save_type = 2;//前翻
		Balance.Chassis_Com->Save_Success_Flag = false;
		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 40.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 40.f;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 2.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 2.f;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kd = 10.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kd = 10.f;
		finish_save = 0;
	}
//	else if(Chassis.Posture->info->pitch >= 0.5f || (Chassis.Posture->info->roll >= 1.5f && Chassis.Posture->info->roll <= 2.9f)
//		     || (Chassis.Posture->info->roll <= -1.5f && Chassis.Posture->info->roll >= -2.9f))
  else if(Chassis.Posture->info->pitch >= 0.7f
	      	|| ((Chassis.Posture->info->roll >= -1.57f && Chassis.Posture->info->roll <= -0.7f) && (Chassis.Posture->info->pitch >= 0.7f || Chassis.Posture->info->pitch <= -0.7f))
		      || ((Chassis.Posture->info->roll <= 1.57f && Chassis.Posture->info->roll >= 0.7) && (Chassis.Posture->info->pitch >= 0.7f || Chassis.Posture->info->pitch <= -0.7f)))
	{
		Balance.Chassis_Com->save_type = 3;//后翻
		Balance.Chassis_Com->Save_Success_Flag = false;
		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 40.f;//60;
		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 40.f;//60;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 2.f;//3;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 2.f;//3;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kd = 10.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kd = 10.f;

		finish_save = 0;
	}
//	else if(Chassis.Posture->info->roll >= 1.5f && Chassis.Posture->info->roll <= 2.9f)
//	{
//		Balance.Chassis_Com->save_type = 4;//右翻
//		Balance.Chassis_Com->Save_Success_Flag = false;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
//		finish_save = 0;
//	}
//	else if(Chassis.Posture->info->roll <= -1.5f && Chassis.Posture->info->roll >= -2.9f)
//	{
//		Balance.Chassis_Com->save_type = 4;//左翻
//		Balance.Chassis_Com->Save_Success_Flag = false;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
//		finish_save = 0;
//	}
	else if((Chassis.Posture->info->pitch <= 0.7f && Chassis.Posture->info->pitch >= -0.7f) && ((R_Leg_Theta >= 150.f || R_Leg_Theta <= 1.f) || (L_Leg_Theta >= 150.f || L_Leg_Theta <= 1.f)))//14
	{
		Balance.Chassis_Com->save_type = 1;//腿脱离范围了
		Balance.Chassis_Com->Save_Success_Flag = false;
		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 15;
		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 15;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 1.2f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 1.2f;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kd = 10.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kd = 10.f;
		finish_save = 0;
	}
	else 
	{
		Balance.Chassis_Com->Save_Success_Flag = true;
		Balance.Chassis_Com->save_type = 0;
		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 15;
		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 15;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 1.3f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 1.3f;
		Chassis.chassis_PID->phi0_cal[L_Leg]->kd = 0.f;
		Chassis.chassis_PID->phi0_cal[R_Leg]->kd = 0.f;
//	  Chassis.reset_struct->reset_state = Chassis_reset_OK;
//		Chassis.chassis_PID->length_cal[L_Leg]->target = TAR_LEG_LENGTH_INITIAL;
//		Chassis.chassis_PID->length_cal[R_Leg]->target = TAR_LEG_LENGTH_INITIAL;
	}
	
	if(tumble_cnt >= 4000)//强制收腿
	{
		Balance.Chassis_Com->Save_Success_Flag = true;
		Balance.Chassis_Com->save_type = 0;
	}
	if(tumble_cnt_2 >= 4500)
	{
		type2_flag = 2;
		type3_flag = 2;
	}
	
	if(wait_cnt_1 >= 2000)
	{
		Balance.Chassis_Com->Save_Finish_Flag_1 = true;
		wait_cnt_1 = 0;
		finish_save = 0;
	}
	if(wait_cnt_2 >= 3000)
	{
		Balance.Chassis_Com->Save_Finish_Flag_2 = true;
		wait_cnt_2 = 0;
		finish_save = 0;
	}
	
	if(Balance.Chassis_Com->save_type == 2 || Balance.Chassis_Com->save_type == 3)//翻车控头
	{
//		My_Gimbal.flag->Gimbal_Init_Flag = false;//翻车还是要控好头的吧
		gimbal.gimbal_reset_state = DEV_RESET_NO;
	}
	
	if(save_type_last != Balance.Chassis_Com->save_type)
	{
		first_enter = 0;
		type2_flag = 1;
		type3_flag = 1;
//		tumble_cnt = 0;
//		tumble_cnt_2 = 0;
//		wait_cnt_1 = 0;
//		wait_cnt_2 = 0;
	}		
	switch(Balance.Chassis_Com->save_type)//最后进行过零点处理及pid力计算
	{
		case 0://可以直接起立了//这是dhj的逻辑
		{	
			if(fabsf(My_Chassis->Posture->info->pitch_v) <= 0.03f || wait_cnt_3 >=1600)//等车体稳定
			{
				wait_cnt_3 = 0;
				if((My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.16f) && (My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 <= 0.16f))//2025.7.10改
				{
					Balance.Chassis_Com->Save_Finish_Flag_1 = true;
				}
				
				if(Balance.Chassis_Com->Save_Finish_Flag_1 != true)
				{
					wait_cnt_1 = wait_cnt_1 + 1;
					Chassis.target->leg_length_l = 0.15f;
					Chassis.target->leg_length_r = 0.15f;
				}
				
				if((R_Leg_Theta >= 75.f && R_Leg_Theta <=140.f) && (L_Leg_Theta >= 75.f && L_Leg_Theta <= 140.f) && Balance.Chassis_Com->Save_Finish_Flag_1 == true)//完成自救
				{
					Balance.Chassis_Com->Save_Finish_Flag_2 = true;
					tumble_cnt = 0;
				}
				if(Balance.Chassis_Com->Save_Finish_Flag_2 != true)
				{
					wait_cnt_2 = wait_cnt_2 + 1;
					Chassis.target->save_phi0_l_tar = 90.f;
					Chassis.target->save_phi0_r_tar = 90.f;
				}
	//			else if(Balance.Chassis_Com->Save_Finish_Flag_2 == true && finish_save == 0)
	//			{
	//				Chassis.target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
	//				Chassis.target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
	////				finish_save = 1;
	//			}
				
				if(Balance.Chassis_Com->Save_Finish_Flag_1 == true && Balance.Chassis_Com->Save_Finish_Flag_2 == true)
				{
					finish_save = 1;
					Chassis.reset_struct->reset_state = Chassis_reset_OK;
					Chassis.target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
					Chassis.target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
					first_enter = 0;
					tumble_cnt = 0;
					tumble_cnt_2 = 0;
					wait_cnt_1 = 0;
					wait_cnt_2 = 0;
					wait_cnt_3 = 0;
				}
			}
				else
				{wait_cnt_3++;}
			break;}
		case 1://机体正常，转腿到后面角度
		{ tumble_cnt = tumble_cnt+ 1;			
			if(first_enter == 0)//确定初始值
			{
				Chassis.target->save_phi0_r_tar = R_Leg_Theta;
				Chassis.target->save_phi0_l_tar = L_Leg_Theta;
				first_enter = 1;
			}
//			Chassis.target->leg_length_l = 0.46f;//腿伸到最长
//			Chassis.target->leg_length_r = 0.46f;
			if(R_Leg_Theta >= 100.f || R_Leg_Theta <= 14.f)
			{
				Chassis.target->save_phi0_r_tar += 0.14f;
			}
			if(L_Leg_Theta >= 100.f || L_Leg_Theta <= 14.f)
			{
				Chassis.target->save_phi0_l_tar += 0.14f;
			}
////			if(R_Leg_Theta >= 180.f && my_abs(Chassis.target->save_phi0_r_tar) <= 18.f)自己加上的
////			{
////				Chassis.target->save_phi0_r_tar = 360.f;
////			}
////			if(L_Leg_Theta >= 180.f && my_abs(Chassis.target->save_phi0_l_tar) <= 18.f)
////			{
////				Chassis.target->save_phi0_l_tar = 360.f;
////			}
			
			if(Chassis.target->save_phi0_r_tar >=14.f && Chassis.target->save_phi0_r_tar <= 80.f)
			{
				Chassis.target->save_phi0_r_tar = 14.f;
			}
			if(Chassis.target->save_phi0_l_tar >=14.f && Chassis.target->save_phi0_l_tar <= 80.f)
			{
				Chassis.target->save_phi0_l_tar = 14.f;
			}
		break;}
		case 2://前翻
		{	tumble_cnt_2 = tumble_cnt_2 +1;
			 if(first_enter == 0)//确定初始值
			{
				Chassis.target->save_phi0_r_tar = R_Leg_Theta;
				Chassis.target->save_phi0_l_tar = L_Leg_Theta;
				type2_flag = 1;
				first_enter = 1;
			}
//////			Chassis.target->leg_length_l = 0.46f;
//////			Chassis.target->leg_length_r = 0.46f;
			Chassis.target->save_phi0_r_tar -= 0.12f;
			Chassis.target->save_phi0_l_tar -= 0.12f;
			
			if(Chassis.target->save_phi0_l_tar <= 0.f)
			{
				Chassis.target->save_phi0_l_tar += 360.f;
			}
			if(Chassis.target->save_phi0_r_tar <= 0.f)
			{
				Chassis.target->save_phi0_r_tar += 360.f;
			}
	
			if((fabsf(R_Leg_Theta - L_Leg_Theta) <= 15) && (R_Leg_Theta <= 280.f && R_Leg_Theta >= 250.f) && (L_Leg_Theta <= 280.f && L_Leg_Theta >= 250.f))
			{
			  type2_flag = 2;	
			}
			
			if(type2_flag == 1)
			{				
			  if(Chassis.target->save_phi0_r_tar >= 240.f && Chassis.target->save_phi0_r_tar <= 280.f)
			  {
				  Chassis.target->save_phi0_r_tar = 275.f;
			  }
			  if(Chassis.target->save_phi0_l_tar >= 240.f && Chassis.target->save_phi0_l_tar <= 280.f)
			  {
				  Chassis.target->save_phi0_l_tar = 275.f;
		  	}
			}
			else if(type2_flag == 2)
			{
				if(Chassis.target->save_phi0_r_tar <= 215.f && Chassis.target->save_phi0_r_tar >= 180.f)
			  {
				  Chassis.target->save_phi0_r_tar = 200.f;
			  }
			  if(Chassis.target->save_phi0_l_tar <= 215.f && Chassis.target->save_phi0_l_tar >= 180.f)
			  {
				  Chassis.target->save_phi0_l_tar = 200.f;
		  	}
			}
			break;}
		case 3:
		{	tumble_cnt_2 = tumble_cnt_2 + 1;
			if(first_enter == 0)//确定初始值
			{
				Chassis.target->save_phi0_r_tar = R_Leg_Theta;
				Chassis.target->save_phi0_l_tar = L_Leg_Theta;
				type3_flag = 1;
				first_enter = 1;
			}
//////			Chassis.target->leg_length_l = 0.46f;
//////			Chassis.target->leg_length_r = 0.46f;
			Chassis.target->save_phi0_r_tar += 0.12f;
			Chassis.target->save_phi0_l_tar += 0.12f;
			
			if((fabsf(R_Leg_Theta - L_Leg_Theta) <= 15) && (R_Leg_Theta >= 240.f && R_Leg_Theta <= 290.f) && (L_Leg_Theta >= 240.f && L_Leg_Theta <= 290.f))
			{
			  type3_flag = 2;	
			}
			
			if(type3_flag == 1)
			{				
			  if(Chassis.target->save_phi0_r_tar >= 250.f && Chassis.target->save_phi0_r_tar <= 280.f)
			  {
				  Chassis.target->save_phi0_r_tar = 260.f;
			  }
			  if(Chassis.target->save_phi0_l_tar >= 250.f && Chassis.target->save_phi0_l_tar <= 280.f)
			  {
				  Chassis.target->save_phi0_l_tar = 260.f;
		  	}
			}
			else if(type3_flag == 2)
			{
				if(Chassis.target->save_phi0_r_tar >= 315.f && Chassis.target->save_phi0_r_tar <= 350.f)
			  {
				  Chassis.target->save_phi0_r_tar = 330.f;
			  }
			  if(Chassis.target->save_phi0_l_tar >= 315.f && Chassis.target->save_phi0_l_tar <= 350.f)
			  {
				  Chassis.target->save_phi0_l_tar = 330.f;
		  	}
			}}
		default:
			break;
	}
	
	if(fabsf(Chassis.target->save_phi0_r_tar) >= 360.f)
	{
		Chassis.target->save_phi0_r_tar -= 360.f*sgn(Chassis.target->save_phi0_r_tar);
	}
	if(fabsf(Chassis.target->save_phi0_l_tar) >= 360.f)
	{
		Chassis.target->save_phi0_l_tar -= 360.f*sgn(Chassis.target->save_phi0_l_tar);
	}
	
//	My_Theta_Save_Cal(&Chassis);
	
	save_type_last = Balance.Chassis_Com->save_type;
}
/*自救腿角度计算*/
void My_Theta_Save_Cal(Chassis_t* My_Chassis)
{
	My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure = R_Leg_Theta;
	
	My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure = L_Leg_Theta;
	
	My_Chassis->chassis_PID->phi0_cal[R_Leg]->target = My_Chassis->target->save_phi0_r_tar;
	
	My_Chassis->chassis_PID->phi0_cal[L_Leg]->target = My_Chassis->target->save_phi0_l_tar;
	
	/*手动赋值误差*/
//	if(My_Chassis->chassis_PID->phi0_cal[R_Leg]->target >= 150.f)
//	{
//	  My_Chassis->chassis_PID->phi0_cal[R_Leg]->err = My_Chassis->chassis_PID->phi0_cal[R_Leg]->target - My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure;
//	}
	if(My_Chassis->chassis_PID->phi0_cal[R_Leg]->target <= 20.f && My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure >= 180.f)
	{
	  My_Chassis->chassis_PID->phi0_cal[R_Leg]->err = My_Chassis->chassis_PID->phi0_cal[R_Leg]->target - My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure + 360.f;
	}
	else
	{
	  My_Chassis->chassis_PID->phi0_cal[R_Leg]->err = My_Chassis->chassis_PID->phi0_cal[R_Leg]->target - My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure;
	}
	
//	if(My_Chassis->chassis_PID->phi0_cal[L_Leg]->target >= 150.f)
//	{
//	  My_Chassis->chassis_PID->phi0_cal[L_Leg]->err = My_Chassis->chassis_PID->phi0_cal[L_Leg]->target - My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure;
//	}
	if(My_Chassis->chassis_PID->phi0_cal[L_Leg]->target <= 20.f && My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure >= 180.f)
	{
	  My_Chassis->chassis_PID->phi0_cal[L_Leg]->err = My_Chassis->chassis_PID->phi0_cal[L_Leg]->target - My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure + 360.f;
	}
	else
	{
	  My_Chassis->chassis_PID->phi0_cal[L_Leg]->err = My_Chassis->chassis_PID->phi0_cal[L_Leg]->target - My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure;
	}

	
//	if(fabsf(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err) >= 90)//跳变处理
//	{
//	   My_Chassis->chassis_PID->phi0_cal[R_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err);
//	}
//	
//	if(fabsf(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err) >= 90)//跳变处理
//	{
//	   My_Chassis->chassis_PID->phi0_cal[L_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err);
//	}

	if(fabsf(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err) >= 180)//跳变处理
	{
	   My_Chassis->chassis_PID->phi0_cal[R_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err);
	}
	
	if(fabsf(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err) >= 180)//跳变处理
	{
	   My_Chassis->chassis_PID->phi0_cal[L_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err);
	}

	
	single_pid_ctrl(My_Chassis->chassis_PID->phi0_cal[R_Leg]);
	
	single_pid_ctrl(My_Chassis->chassis_PID->phi0_cal[L_Leg]);
//	single_pid_cal(R_Link_Var->link_pid->yaw_cal->info);
//	
//	single_pid_cal(L_Link_Var->link_pid->yaw_cal->info);
	
	My_Chassis->Leg_Unit[L_Leg]->force->F_Save = My_Chassis->chassis_PID->phi0_cal[L_Leg]->out;
	
	My_Chassis->Leg_Unit[R_Leg]->force->F_Save = My_Chassis->chassis_PID->phi0_cal[R_Leg]->out;
}



////腿在前面伸展角度为160,往上180，-180°,0,往后伸展为16°
////而前翻的时候最大摆角大概是277
////后翻的时候最大摆角大概是284(265),给目标角340应该就能稳救回来
//static float R_Leg_Theta = 0.f,L_Leg_Theta = 0.f;
//static void My_Chassis_Save_Process(Chassis_t *My_Chassis)
//{
//	static uint16_t tumble_cnt = 0;//时间过长回到上一阶段
//	static uint16_t tumble_cnt_2 = 0;
//	static uint16_t wait_cnt_1 = 0;
//	static uint16_t wait_cnt_2 = 0;
//	static uint8_t save_must = 0;//type1,强制收腿,给个计时
//	static uint8_t save_type_last = 0;
//	static uint8_t first_enter = 0;
//  static uint8_t finish_save = 0;
//	static uint8_t type2_flag = 0;
//	static uint8_t type3_flag = 0;
// 	
//	if(rc_sensor.work_state == DEV_OFFLINE)//遥控离线
//	{
//		first_enter = 0;
//		tumble_cnt = 0;
//		tumble_cnt_2 = 0;
//		wait_cnt_1 = 0;
//		wait_cnt_2 = 0;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
//	}
//	
//	R_Leg_Theta = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0_;
//	if(R_Leg_Theta <= 0)
//	{
//		R_Leg_Theta = 360.f + R_Leg_Theta;//转为0~360一圈
//		
//	}
//	
//	L_Leg_Theta = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0_;
//	if(L_Leg_Theta <= 0)
//	{
//		L_Leg_Theta = 360.f + L_Leg_Theta;
//	}
//	
//	if(Chassis.Posture->info->pitch <= -0.5f)
//	{
//		Balance.Chassis_Com->save_type = 2;//前翻
//		Balance.Chassis_Com->Save_Success_Flag = false;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
////		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 50;
////		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 50;
////		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 2;
////		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 2;
//		finish_save = 0;
//	}
//	else if(Chassis.Posture->info->pitch >= 0.5f || (Chassis.Posture->info->roll >= 1.5f && Chassis.Posture->info->roll <= 2.9f)
//		     || (Chassis.Posture->info->roll <= -1.5f && Chassis.Posture->info->roll >= -2.9f))
//	{
//		Balance.Chassis_Com->save_type = 3;//后翻
//		Balance.Chassis_Com->Save_Success_Flag = false;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;

//		finish_save = 0;
//	}
//	else if((Chassis.Posture->info->pitch <= 0.5f && Chassis.Posture->info->pitch >= -0.5f) && ((R_Leg_Theta >= 150.f || R_Leg_Theta <= 12.f) || (L_Leg_Theta >= 150.f || L_Leg_Theta <= 12.f)))
//	{
//		Balance.Chassis_Com->save_type = 1;//腿脱离范围了
//		Balance.Chassis_Com->Save_Success_Flag = false;
//		Balance.Chassis_Com->Save_Finish_Flag_1 = false;
//		Balance.Chassis_Com->Save_Finish_Flag_2 = false;
//		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 20;
//		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 20;
//		Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 2.f;
//		Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 2.f;

//		finish_save = 0;
//	}
//	else 
//	{
//		Balance.Chassis_Com->Save_Success_Flag = true;
//		Balance.Chassis_Com->save_type = 0;
////	  Chassis.reset_struct->reset_state = Chassis_reset_OK;
////		Chassis.chassis_PID->length_cal[L_Leg]->target = TAR_LEG_LENGTH_INITIAL;
////		Chassis.chassis_PID->length_cal[R_Leg]->target = TAR_LEG_LENGTH_INITIAL;
//	}
//	
//	if(tumble_cnt >= 5000)//强制收腿
//	{
//		Balance.Chassis_Com->Save_Success_Flag = true;
//		Balance.Chassis_Com->save_type = 0;
//	}
//	if(tumble_cnt_2 >= 6000)
//	{
//		type2_flag = 2;
//		type3_flag = 2;
//	}
//	
//	if(wait_cnt_1 >= 4000)
//	{
//		Balance.Chassis_Com->Save_Finish_Flag_1 = true;
//		wait_cnt_1 = 0;
//		finish_save = 0;
//	}
//	if(wait_cnt_2 >= 6000)
//	{
//		Balance.Chassis_Com->Save_Finish_Flag_2 = true;
//		wait_cnt_2 = 0;
//		finish_save = 0;
//	}
//	
//	if(Balance.Chassis_Com->save_type == 2 || Balance.Chassis_Com->save_type == 3)//翻车控头
//	{
////		My_Gimbal.flag->Gimbal_Init_Flag = false;//翻车还是要控好头的吧
//		gimbal.gimbal_reset_state = DEV_RESET_NO;
//	}
//	
//	if(save_type_last != Balance.Chassis_Com->save_type)
//	{
//		first_enter = 0;
//		type2_flag = 1;
//		type3_flag = 1;
////		tumble_cnt = 0;
////		tumble_cnt_2 = 0;
////		wait_cnt_1 = 0;
////		wait_cnt_2 = 0;
//	}		
//	switch(Balance.Chassis_Com->save_type)//最后进行过零点处理及pid力计算
//	{
//		case 0://可以直接起立了//这是dhj的逻辑
//		{	
//		  if((My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.16f) || (My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0 <= 0.16f))//2025.7.10改
//			{
//				Balance.Chassis_Com->Save_Finish_Flag_1 = true;
//			}
//			
//			if(Balance.Chassis_Com->Save_Finish_Flag_1 != true)
//			{
//				wait_cnt_1 = wait_cnt_1 + 1;
//				Chassis.target->leg_length_l = 0.15f;
//				Chassis.target->leg_length_r = 0.15f;
//			}
//			
//			if((R_Leg_Theta >= 75.f && R_Leg_Theta <=140.f) && (L_Leg_Theta >= 75.f && L_Leg_Theta <= 140.f) && Balance.Chassis_Com->Save_Finish_Flag_1 == true)//完成自救
//			{
//				Balance.Chassis_Com->Save_Finish_Flag_2 = true;
//				Chassis.target->save_phi0_r_tar = 0;
//				Chassis.target->save_phi0_l_tar = 0;

//				tumble_cnt = 0;
//			}
//			if(Balance.Chassis_Com->Save_Finish_Flag_2 != true)
//			{
//				wait_cnt_2 = wait_cnt_2 + 1;
//				Chassis.target->save_phi0_l_tar = 20.f;
//				Chassis.target->save_phi0_r_tar = 20.f;
//			}
////			else if(Balance.Chassis_Com->Save_Finish_Flag_2 == true && finish_save == 0)
////			{
////				Chassis.target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
////				Chassis.target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//////				finish_save = 1;
////			}
//			
//			if(Balance.Chassis_Com->Save_Finish_Flag_1 == true && Balance.Chassis_Com->Save_Finish_Flag_2 == true)
//			{
//				finish_save = 1;
//				Chassis.reset_struct->reset_state = Chassis_reset_OK;
//				Chassis.target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
//				Chassis.target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
//				first_enter = 0;
//		    tumble_cnt = 0;
//		    tumble_cnt_2 = 0;
//		    wait_cnt_1 = 0;
//		    wait_cnt_2 = 0;

//			}
//			break;}
//		case 1://机体正常，转腿到后面角度
//		{ tumble_cnt = tumble_cnt+ 1;			
//			if(first_enter == 0)//确定初始值
//			{
//				Chassis.target->save_phi0_r_tar = 0;
//				Chassis.target->save_phi0_l_tar = 0;
//				first_enter = 1;
//			}
//			Chassis.target->leg_length_l = 0.46f;//腿伸到最长
//			Chassis.target->leg_length_r = 0.46f;
//			if(R_Leg_Theta >= 80.f || R_Leg_Theta <= 14.f)
//			{
//				Chassis.target->save_phi0_r_tar = 10.f;
//			}
//			if(L_Leg_Theta >= 80.f || L_Leg_Theta <= 14.f)
//			{
//				Chassis.target->save_phi0_l_tar = 10.f;
//			}
//			
//			if(R_Leg_Theta >= 14.f && R_Leg_Theta <= 80.f)
//			{
//				Chassis.target->save_phi0_r_tar = -10.f;
//			}
//			if(L_Leg_Theta >= 14.f && L_Leg_Theta <= 80.f)
//			{
//				Chassis.target->save_phi0_l_tar = -10.f;
//			}
//			
//			if(R_Leg_Theta >= 12.f && R_Leg_Theta <= 16.f)
//			{
//				Chassis.target->save_phi0_r_tar = 0.f;
//			}
//			if(L_Leg_Theta >= 12.f && L_Leg_Theta <= 16.f)
//			{
//				Chassis.target->save_phi0_l_tar = 0.f;
//			}
//			
//		break;}
//		case 2://前翻
//		{	tumble_cnt_2 = tumble_cnt_2 +1;
//			 if(first_enter == 0)//确定初始值
//			{
//				Chassis.target->save_phi0_r_tar = 0;
//				Chassis.target->save_phi0_l_tar = 0;
//				type2_flag = 1;
//				first_enter = 1;
//			}
//			Chassis.target->leg_length_l = 0.46f;
//			Chassis.target->leg_length_r = 0.46f;
//			
//			if(type2_flag == 1)
//			{
//			  Chassis.target->save_phi0_r_tar = -10.f;
//			  Chassis.target->save_phi0_l_tar = -10.f;
//			}
//			else if(type2_flag == 2)
//			{
//			  Chassis.target->save_phi0_r_tar = -10.f;
//			  Chassis.target->save_phi0_l_tar = -10.f;
//			}
//			
//			if((fabsf(R_Leg_Theta - L_Leg_Theta) <= 12) && (R_Leg_Theta <= 310.f && R_Leg_Theta >= 270.f) && (L_Leg_Theta <= 310.f && L_Leg_Theta >= 270.f))
//			{
//			  type2_flag = 2;	
//			}
//			
//			if(type2_flag == 1)
//			{				
//			  if(Chassis.target->save_phi0_r_tar >= 260.f && Chassis.target->save_phi0_r_tar <= 295.f)
//			  {
//				  Chassis.target->save_phi0_r_tar = 0.f;
//			  }
//			  if(Chassis.target->save_phi0_l_tar >= 260.f && Chassis.target->save_phi0_l_tar <= 295.f)
//			  {
//				  Chassis.target->save_phi0_l_tar = 0.f;
//		  	}
//			}
//			else if(type2_flag == 2)
//			{
//				if(Chassis.target->save_phi0_r_tar <= 200.f && Chassis.target->save_phi0_r_tar >= 180.f)
//			  {
//				  Chassis.target->save_phi0_r_tar = 0.f;
//			  }
//			  if(Chassis.target->save_phi0_l_tar <= 200.f && Chassis.target->save_phi0_l_tar >= 180.f)
//			  {
//				  Chassis.target->save_phi0_l_tar = 0.f;
//		  	}
//			}
//			break;}
//		case 3:
//		{	tumble_cnt_2 = tumble_cnt_2 + 1;
//			if(first_enter == 0)//确定初始值
//			{
//				Chassis.target->save_phi0_r_tar = 0;
//				Chassis.target->save_phi0_l_tar = 0;
//				type3_flag = 1;
//				first_enter = 1;
//			}
//			Chassis.target->leg_length_l = 0.46f;
//			Chassis.target->leg_length_r = 0.46f;
//			
//			if(type3_flag == 1)
//			{	
//			  Chassis.target->save_phi0_r_tar = 10.f;
//			  Chassis.target->save_phi0_l_tar = 10.f;
//		    Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 80;
//		    Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 80;
////		    Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 1.5f;
////		    Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 1.5f;

//			}
//			else if(type3_flag == 2)
//			{
//			  Chassis.target->save_phi0_r_tar = 20.f;
//			  Chassis.target->save_phi0_l_tar = 20.f;
//		    Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 80;
//		    Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 80;
////		    Chassis.chassis_PID->phi0_cal[L_Leg]->kp = 2.5f;
////		    Chassis.chassis_PID->phi0_cal[R_Leg]->kp = 2.5f;
//			}
//			
//			if((fabsf(R_Leg_Theta - L_Leg_Theta) <= 15) && (R_Leg_Theta >= 280.f && R_Leg_Theta <= 310.f) && (L_Leg_Theta >= 280.f && L_Leg_Theta <= 310.f))
//			{
//			  type3_flag = 2;	
//			}
//			
//			if(type3_flag == 1)
//			{				
//			  if(R_Leg_Theta >= 280.f && R_Leg_Theta <= 300.f)
//			  {
//				  Chassis.target->save_phi0_r_tar = -3.f;
//			  }
//			  if(L_Leg_Theta >= 280.f && L_Leg_Theta <= 300.f)
//			  {
//				  Chassis.target->save_phi0_l_tar = -3.f;
//		  	}
//			}
//			else if(type3_flag == 2)
//			{
//				if(Chassis.target->save_phi0_r_tar >= 330.f && Chassis.target->save_phi0_r_tar <= 360.f)
//			  {
//				  Chassis.target->save_phi0_r_tar = 0.f;
//			  }
//			  if(Chassis.target->save_phi0_l_tar >= 330.f && Chassis.target->save_phi0_l_tar <= 360.f)
//			  {
//				  Chassis.target->save_phi0_l_tar = 0.f;
//		  	}
//			}
//			
//			if(Chassis.chassis_PID->phi0_cal[L_Leg]->out >= 15.f && Chassis.chassis_PID->phi0_cal[L_Leg]->measure <= 2.f)
//			{
//				Chassis.chassis_PID->phi0_cal[L_Leg]->kp += 0.002f;
//			}
//			if(Chassis.chassis_PID->phi0_cal[R_Leg]->out >= 15.f && Chassis.chassis_PID->phi0_cal[R_Leg]->measure <= 2.f)
//			{
//				Chassis.chassis_PID->phi0_cal[R_Leg]->kp += 0.002f;
//			}
//			
//		}
//		default:
//			break;
//	}
//		
//	My_Theta_Save_Cal(&Chassis);
//	
//	save_type_last = Balance.Chassis_Com->save_type;
//}
///*自救腿角度计算*/
//void My_Theta_Save_Cal(Chassis_t* My_Chassis)
//{
//	My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1;
//	
//	My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1;
//	
//	My_Chassis->chassis_PID->phi0_cal[R_Leg]->target = My_Chassis->target->save_phi0_r_tar;
//	
//	My_Chassis->chassis_PID->phi0_cal[L_Leg]->target = My_Chassis->target->save_phi0_l_tar;
//	
//	My_Chassis->chassis_PID->phi0_cal[L_Leg]->err = My_Chassis->chassis_PID->phi0_cal[L_Leg]->target - My_Chassis->chassis_PID->phi0_cal[L_Leg]->measure;
//	
//	My_Chassis->chassis_PID->phi0_cal[R_Leg]->err = My_Chassis->chassis_PID->phi0_cal[R_Leg]->target - My_Chassis->chassis_PID->phi0_cal[R_Leg]->measure;

//	if(fabsf(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err) >= 180)//跳变处理
//	{
//	  My_Chassis->chassis_PID->phi0_cal[R_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[R_Leg]->err);
//	}
//	
//	if(fabsf(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err) >= 180)//跳变处理
//	{
//	  My_Chassis->chassis_PID->phi0_cal[L_Leg]->err -= 360*sgn(My_Chassis->chassis_PID->phi0_cal[L_Leg]->err);
//	}

//	single_pid_ctrl(My_Chassis->chassis_PID->phi0_cal[R_Leg]);
//	
//	single_pid_ctrl(My_Chassis->chassis_PID->phi0_cal[L_Leg]);
////	single_pid_cal(R_Link_Var->link_pid->yaw_cal->info);
////	
////	single_pid_cal(L_Link_Var->link_pid->yaw_cal->info);
//	
//	My_Chassis->Leg_Unit[L_Leg]->force->F_Save = My_Chassis->chassis_PID->phi0_cal[L_Leg]->out;
//	
//	My_Chassis->Leg_Unit[R_Leg]->force->F_Save = My_Chassis->chassis_PID->phi0_cal[R_Leg]->out;
//}



/*手动自救*/
static void My_Chassis_Handle_Save_Process(Chassis_t *My_Chassis)
{
	R_Leg_Theta = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0_;
	if(R_Leg_Theta <= 0)
	{
		R_Leg_Theta = 360.f + R_Leg_Theta;//转为0~360一圈	
	}
	L_Leg_Theta = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0_;
	if(L_Leg_Theta <= 0)
	{
		L_Leg_Theta = 360.f + L_Leg_Theta;
	}

	static float first_in = 0;
//	if(rc_sensor.work_state == DEV_OFFLINE)
//	{
//		first_in = 0;
//	}
	if(Balance.mode != Balance.last_mode && Balance.mode == Handle_Mode)
	{
		first_in = 0;
	}

	if(first_in == 0)
	{
		Chassis.target->save_phi0_r_tar = R_Leg_Theta;
		Chassis.target->save_phi0_l_tar = L_Leg_Theta;
		first_in = 1;
	}
	if(Balance.ctrl == RC_CTRL)
	{
		if(rc_sensor.info->ch3 >= 100)
		{
			Chassis.target->save_phi0_l_tar += 0.08f;
		}
		if(rc_sensor.info->ch3 <= -100)
		{
			Chassis.target->save_phi0_l_tar -= 0.08f;
		}
		if(rc_sensor.info->ch1 >= 100)
		{
			Chassis.target->save_phi0_r_tar += 0.08f;
		}
		if(rc_sensor.info->ch1 <= -100)
		{
			Chassis.target->save_phi0_r_tar -= 0.08f;
		}
	}
  else if(Balance.ctrl == KEY_CTRL)
	{
		if(rc_sensor.info->Q.cnt >= 300)
		{
			Chassis.target->save_phi0_l_tar += 0.10f;
		}
		if(rc_sensor.info->Z.cnt >= 300)
		{
			Chassis.target->save_phi0_l_tar -= 0.1f;
		}
		if(rc_sensor.info->E.cnt >= 300)
		{
			Chassis.target->save_phi0_r_tar += 0.1f;
		}
		if(rc_sensor.info->C.cnt >= 300)
		{
			Chassis.target->save_phi0_r_tar -= 0.1f;
		}
	}
	if(fabsf(Chassis.target->save_phi0_r_tar) >= 360.f)
	{
		Chassis.target->save_phi0_r_tar -= 360.f*sgn(Chassis.target->save_phi0_r_tar);
	}
	if(fabsf(Chassis.target->save_phi0_l_tar) >= 360.f)
	{
		Chassis.target->save_phi0_l_tar -= 360.f*sgn(Chassis.target->save_phi0_l_tar);
	}

//	My_Theta_Save_Cal(&Chassis);

}

