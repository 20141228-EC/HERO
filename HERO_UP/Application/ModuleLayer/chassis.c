/*
 * chassis
 *
 *	2023.9.11
 * 底盘
 */

/* Includes ------------------------------*/
#include "chassis.h"
#include "device.h"
#include "rp_math.h"
#include "car.h"
#include "communicate_protocol.h"

/* Exported variables --------------------*/
chassis_t chassis =
	{
		.cmd_auto_lob = &command[AUTO_LOB],
		.cmd_normal_lob =&command[NORMAL_LOB],
		.cmd_oblique_lob =&command[OBLIQUE_LOB],
		.cmd_180 = &command[GIM_180],
		.cmd_timer_mec_outpost = &command[TIMER_MEC_OUTPOST],
		.work = Chassis_Work,

};
int16_t yaw_angle_err;

/* Private function prototypes -----------------------------------------------*/
void Chassis_Mec_Update(chassis_t *chassis, uint8_t ctrl_mode);						// 机械模式底盘数据更新
void Chassis_Gyro_Update(chassis_t *chassis, uint8_t ctrl_mode, uint8_t move_mode); // 陀螺仪模式底盘数据更新
void Chassis_Cmd_Excute(chassis_t *chassis);										// 吊射命令初始化时底盘跟云台

/**
 * @brief 底盘结构体数据更新
 * @param ctrl_move 0:遥控器模式 1：键盘模式
 */
void Chassis_Mec_Update(chassis_t *chassis, uint8_t ctrl_mode)
{
	 
		if (ctrl_mode == 0) // 遥控器模式   //线性拟合遥控器数据得到正确的目标底盘X、Y、w速度，
	{
		
		chassis->base_info.target_front_speed = (float)rc_sensor.info->ch3 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_right_speed = (float)rc_sensor.info->ch2 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_cycle_speed = (float)rc_sensor.info->ch0 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
	}
	else if (ctrl_mode == 1) // 键盘模式
	{
		chassis->base_info.target_front_speed = 0;
		chassis->base_info.target_right_speed = 0;
		chassis->base_info.target_front_speed += (float)rc_sensor.info->W.cnt / (float)KEY_W_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_front_speed -= (float)rc_sensor.info->S.cnt / (float)KEY_S_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_right_speed += (float)rc_sensor.info->D.cnt / (float)KEY_D_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_right_speed -= (float)rc_sensor.info->A.cnt / (float)KEY_A_CNT_MAX * CHASSIS_MAX_SPEED;
		chassis->base_info.target_cycle_speed = rc_sensor.info->mouse_vx * 18.f;
		
		if(rc_sensor.info->Ctrl.value==1)
		{
			chassis->base_info.target_front_speed = 0;
			chassis->base_info.target_right_speed = 0;
		}
	}

	
	#if HERO_TYPE!=2
	if (abs(gimbal.base_info.yaw_motor_angle) > 16384) // 如果云台yaw轴电机超过16384就对XY目标速度取反
	{
		chassis->base_info.target_front_speed *= -1;
		chassis->base_info.target_right_speed *= -1;
	}
	#endif
	
}

/**
 * @brief 底盘陀螺仪模式更新
 */

// 计算陀螺仪坐标和底盘坐标角度差
int16_t yaw_angle_err;
float k1_cycle=30;
float k2_cycle=1;

void Chassis_Gyro_Update(chassis_t *chassis, uint8_t ctrl_mode, uint8_t move_mode)
{
	Chassis_Mec_Update(chassis, ctrl_mode);
	float front = chassis->base_info.target_front_speed;
	float right = chassis->base_info.target_right_speed;
	#if HERO_TYPE!=2
	if (abs(gimbal.base_info.yaw_motor_angle) > 16384) 
	{
		front *= -1;
		right *= -1;
	}
	#endif
	
	
	yaw_angle_err = gimbal.base_info.yaw_motor_angle / 32768.f * 4096.f; // yaw轴   相对底盘   角度(-4096~4096)(顺时针为正)
	float yaw_angle_err_rad = (double)yaw_angle_err / 4096.f * 3.14159;	 // yaw轴角度转弧度制（-π~π）

	// front和right值计算
	chassis->base_info.target_front_speed = front * cos(yaw_angle_err_rad) - right * sin(yaw_angle_err_rad);
	chassis->base_info.target_right_speed = right * cos(yaw_angle_err_rad) + front * sin(yaw_angle_err_rad);
	
	if (move_mode == 0)
	{
		
		#if HERO_TYPE!=2 //正常麦轮
		if (abs(yaw_angle_err) > 2048) // 头可以朝后
		{
			yaw_angle_err -= 4096 * sgn(yaw_angle_err);
		}
		#else //全麦
			#ifdef HEAD_BACK //头朝全向轮方向
			if (abs(yaw_angle_err) > 2048) // 头可以朝后
			{
				yaw_angle_err -= 4096 * sgn(yaw_angle_err);
			}
			#else //正常朝向
			yaw_angle_err=gimbal.base_info.yaw_motor_angle;
			#endif
		
		#endif
		
		#if 1 //决定底盘旋转速度计算方式
			#if HERO_TYPE!=2 //正常脉轮
			chassis->base_info.target_cycle_speed = yaw_angle_err * yaw_angle_err*sgn(yaw_angle_err) /4096.f*30;
			
			#else //全麦
				#ifdef HEAD_BACK //头朝全向轮方向
				chassis->base_info.target_cycle_speed = yaw_angle_err * yaw_angle_err*sgn(yaw_angle_err) /4096.f*30;
				#else
				chassis->base_info.target_cycle_speed=yaw_angle_err;
				if(abs(chassis->base_info.target_cycle_speed)>=CHASSIS_MAX_SPEED)
				{
					chassis->base_info.target_cycle_speed=sgn(yaw_angle_err)*CHASSIS_MAX_SPEED;
				}
				#endif
			
			#endif
		

		#else
		
		if (abs(yaw_angle_err) >= 100) // 大于4.4°斜率变大
		{
			chassis->base_info.target_cycle_speed = yaw_angle_err * k1_cycle - k2_cycle*100 * sgn(yaw_angle_err);
		}
		else
		{
			chassis->base_info.target_cycle_speed = yaw_angle_err * k2_cycle;
		}
		#endif
	}
	/*小陀螺部分*/
		else if (move_mode == 1)
		{

		if (car.car_move_mode == vision_cycle_CAR)
		{
			chassis->base_info.target_cycle_speed = -CYCLE_SPEED;
		}
		else
		{
			#if CHASSIS_CYCLE_MODE==0 //普通
			chassis->base_info.target_cycle_speed = CYCLE_SPEED; // 小陀螺速度
			#endif
			#if CHASSIS_CYCLE_MODE==1 //变速
			static uint32_t cycle_tick;//用于变速小陀螺
			cycle_tick++;
			if(abs(yaw_angle_err) <628||abs(yaw_angle_err) >3468||abs(sin(cycle_tick*0.001)<0.3))//30°
			{
				cycle_tick+=3;
			}
			chassis->base_info.target_cycle_speed = CYCLE_SPEED*sin(cycle_tick*0.001); // 小陀螺速度
			#endif
			#if CHASSIS_CYCLE_MODE==2 //斜视
			int16_t oblique_yaw_angle_err;//修正后的斜的yaw误差
			if (abs(yaw_angle_err) > 2048) // 头可以朝后
			{
				yaw_angle_err -= 4096 * sgn(yaw_angle_err);
			}
			oblique_yaw_angle_err=yaw_angle_err-800;
			oblique_yaw_angle_err=motor_half_cycle(oblique_yaw_angle_err,4096);
			if (abs(oblique_yaw_angle_err) >= 100) // 大于4.4°斜率变大
			{
				chassis->base_info.target_cycle_speed = oblique_yaw_angle_err * 7.f - 300.f * sgn(oblique_yaw_angle_err);
			}
			else
			{
				chassis->base_info.target_cycle_speed = oblique_yaw_angle_err * 1.f;
			}
			#endif
			
		}
	}
}

/**
 * @Name    Chassis_Cmd_Excute
 * @brief   吊射命令初始化时底盘跟云台
 */
float chassis_error;
void Chassis_Cmd_Excute(chassis_t *chassis)
{
	static command_t *lob_class;//执行动作是一样的，所以用这个来代替
	//这里应该用elseif
	if (chassis->cmd_auto_lob->cmd_value == 1&&lob_class->cmd_status != RUNING_C)//防止运行的时候指针乱指
	{
		lob_class=chassis->cmd_auto_lob;
		 
	}
	if (chassis->cmd_normal_lob->cmd_value == 1&&lob_class->cmd_status != RUNING_C)//防止运行的时候指针乱指
	{
		lob_class=chassis->cmd_normal_lob;
	 
	}
	if (chassis->cmd_oblique_lob->cmd_value == 1&&lob_class->cmd_status != RUNING_C)//防止运行的时候指针乱指
	{
		lob_class=chassis->cmd_oblique_lob;
	 
	}
	//底盘跟随
	if (lob_class->cmd_status == RUNING_C)
	{
#ifdef ACROSS_LOB
		  chassis_error = gimbal.base_info.yaw_motor_angle - sgn(gimbal.base_info.yaw_motor_angle) * 16384;
#else
		 
		if(abs(gimbal.base_info.yaw_motor_angle)<16384)
		{
			chassis_error = gimbal.base_info.yaw_motor_angle;
		}
		else
		{
			chassis_error = gimbal.base_info.yaw_motor_angle - sgn(gimbal.base_info.yaw_motor_angle)*32768;
		}
#endif

		chassis->base_info.target_cycle_speed = chassis_error;
		//不同吊射类不同完成条件
		#ifdef GYRO_LOB_INIT
		

		if(lob_class==chassis->cmd_auto_lob)
		{
			/*云台模块*/
			gimbal.ptich_pid_mode = GYRO_PID;
			gimbal.yaw_pid_mode = GYRO_PID;
			#ifdef ONLY_ONE_DATA_LOB_INIT
			gimbal.base_info.pitch_imu_angle_target = gimbal.lob_info.gyro_init_lob_pitch_angle ;
			gimbal.base_info.yaw_imu_angle_target = gimbal.lob_info.gyro_init_lob_yaw_angle ;
			#else
			gimbal.base_info.pitch_imu_angle_target = gimbal.lob_info.pre_aim_pitch_angle ;
			gimbal.base_info.yaw_imu_angle_target = gimbal.lob_info.pre_aim_yaw_angle ;
			#endif
			/*END*/
			if (
				abs(gimbal.base_info.pitch_imu_speed)<2&&abs(gimbal.base_info.yaw_imu_speed)<2 \
				&&abs(gimbal.base_info.yaw_imu_angle_target-gimbal.base_info.yaw_imu_angle)<0.1 \
				&&abs(gimbal.base_info.pitch_imu_angle_target-gimbal.base_info.pitch_imu_angle)<0.1 \
				)
			{
				lob_class->s_finish(lob_class);
				
			}
		}
		else if(lob_class==chassis->cmd_normal_lob)
		{
			/*云台模块*/
			gimbal.ptich_pid_mode = MEC_PID;
			gimbal.yaw_pid_mode = GYRO_PID;
			/*END*/
			if (abs(chassis_error) < 10 && abs(gimbal.base_info.yaw_motor_speed) < 30)
			{
				lob_class->s_finish(lob_class);
			}
		}
		else if(lob_class==chassis->cmd_oblique_lob)
		{
			/*云台模块*/
			gimbal.ptich_pid_mode = MEC_PID;
			gimbal.yaw_pid_mode = GYRO_PID;
			/*END*/
			if (abs(chassis_error) < 10 && abs(gimbal.base_info.yaw_motor_speed) < 30)
			{
				lob_class->s_finish(lob_class);
			}
		}
		#else
		if (abs(chassis_error) < 10 && abs(gimbal.base_info.yaw_motor_speed) < 30)
		{
			lob_class->s_finish(auto_lob_or_normal_lob);
		}
		#endif
		
	}
	
	
}


/**
 * @Name    Chassis_Work
 * @brief   总控
 * @param   chassis
 **/

void Chassis_Work(chassis_t *chassis)
{
	Chassis_Cmd_Excute(chassis);
	/*更新目标值**********************************/
	switch (car.car_move_mode)
	{
	case offline_CAR:
	case init_CAR:
		chassis->base_info.target_cycle_speed = 0;
		chassis->base_info.target_front_speed = 0;
		chassis->base_info.target_right_speed = 0;
		break;

	case mec_CAR:
		if (chassis->cmd_auto_lob->cmd_status != RUNING_C)
		{
			Chassis_Mec_Update(chassis, car.car_ctrl_mode); // 底盘机械模式更新
		}
		break;

	case vision_gyro_CAR:
	case gyro_CAR:
		
		if (chassis->cmd_180->cmd_status == RUNING_C||gimbal.lob_info.out_oblique_head_homing_flag==1) // 掉头时底盘不旋转
		{
			Chassis_Mec_Update(chassis, car.car_ctrl_mode); // 底盘机械模式更新
		}
		else
		{
			Chassis_Gyro_Update(chassis, car.car_ctrl_mode, 0); // 陀螺仪模式
		}
	
		
		break;

	case vision_cycle_CAR:
	case cycle_CAR:
		Chassis_Gyro_Update(chassis, car.car_ctrl_mode, 1); // 小陀螺模式
		break;

	case lob_CAR:
		if (chassis->cmd_auto_lob->cmd_status == FINISH_C||chassis->cmd_normal_lob->cmd_status == FINISH_C) // 吊射模式初始化完成，底盘固定
		{
			#ifdef LOB_TEST
			chassis->base_info.target_front_speed = (float)rc_sensor.info->ch3 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
			chassis->base_info.target_right_speed = (float)rc_sensor.info->ch2 / RC_MAX_CNT * CHASSIS_MAX_SPEED ;
			#if HERO_TYPE!=2
			if (abs(gimbal.base_info.yaw_motor_angle) > 16384) // 如果云台yaw轴电机超过16384就对XY目标速度取反
			{
				chassis->base_info.target_front_speed *= -1;
				chassis->base_info.target_right_speed *= -1;
			}
			#endif
			
			#else
			chassis->base_info.target_cycle_speed = 0;
			chassis->base_info.target_front_speed = 0;
			chassis->base_info.target_right_speed = 0;
			#endif
			
		}
		break;

	default:
		break;
	}

	/*pid模式更新**********************************/
	
	#ifdef LOB_TEST //吊射测试下底盘可运动
	static uint16_t lob_move_cnt=0;
	static uint8_t lob_move_flag=0;
	//满足拨杆计时
	if(abs(rc_sensor.info->ch3)>=50||abs(rc_sensor.info->ch2)>=50)
	{
		lob_move_cnt++;
	}
	else
	{
		lob_move_cnt=0;
	}
	
	//满足时间条件底盘给动
	if(lob_move_cnt>=500)
	{
		lob_move_flag=1;
	}
	else
	{
		lob_move_flag=0;
	}
	
	if (car.car_move_mode == lob_CAR && (chassis->cmd_auto_lob->cmd_status != RUNING_C&&chassis->cmd_normal_lob->cmd_status != RUNING_C)&&lob_move_flag==0)
	{
		chassis->pid_mode = CHASSIS_POSITION_PID;
	}
	else
	{
		chassis->pid_mode = CHASSIS_SPEED_PID;
	}
	#else
	if (car.car_move_mode == lob_CAR && chassis->cmd_auto_lob->cmd_status != RUNING_C)
	{
		chassis->pid_mode = CHASSIS_POSITION_PID;
	}
	else
	{
		chassis->pid_mode = CHASSIS_SPEED_PID;
	}
	#endif
	 
	/*底盘包更新******************************/
	communicate.chassis_data_tx_info->target_front_speed = chassis->base_info.target_front_speed;
	communicate.chassis_data_tx_info->target_right_speed = chassis->base_info.target_right_speed;
	communicate.chassis_data_tx_info->target_cycle_speed = chassis->base_info.target_cycle_speed;
	communicate.chassis_data_tx_info->pid_mode = chassis->pid_mode;
	communicate.chassis_data_tx_info->max_speed = CHASSIS_MAX_SPEED;

}
