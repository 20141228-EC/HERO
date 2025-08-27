 /*
* chassis
*	
*	2023.9.11
* 底盘
*/

/* Includes ------------------------------*/
#include "chassis.h"
#include "gimbal.h"
#include "device.h"
#include "rp_math.h"
#include "car.h"
#include "communicate_protocol.h"
/* Private function prototypes -----------------------------------------------*/
void Gimbal_init_action(gimbal_t *gimbal);				    //初始化要执行的内容
void Gimbal_External_Date_Update(gimbal_t *gimbal);			//外部数据更新
void Gimbal_Gyro_Update(gimbal_t *gimbal,uint8_t ctrl_mode);//陀螺仪模式遥控数据更新，有累加限制
void Gimbal_Mec_Update(gimbal_t *gimbal,uint8_t ctrl_mode); //机械模式判断是否换头
void Gimbal_Vision_Update(gimbal_t *gimbal,uint8_t ctrl_mode); //视觉模式更新
/*  typedef----------------------------------------------------------------**/



gimbal_offset_info_t offset_info =
{
	.vision_yaw_offset = 0, //视觉偏置 
	.vision_pitch_offset = 0,
	.lob_yaw_mec_offset = 0,    //吊射偏置
	
};

gimbal_t gimbal = 
{
	.gimbal_p=&rm_motor[GIMB_P],
	.gimbal_y=&kt_motor[0],
	.cmd_r90 = &command[CAR_R90],
	.cmd_l90 = &command[CAR_L90],
	.cmd_180 = &command[GIM_180],
	.cmd_timer_mec_outpost = &command[TIMER_MEC_OUTPOST],
	.cmd_auto_lob = &command[AUTO_LOB],
	.cmd_normal_lob =&command[NORMAL_LOB],
	.cmd_oblique_lob=&command[OBLIQUE_LOB],
	.cmd_right = &command[GIM_RIGHT],
	.cmd_left = &command[GIM_LEFT],
	.cmd_up = &command[GIM_UP],
	.cmd_dowm = &command[GIM_DOWM],
	.cmd_change_lob_pitch = &command[CHANGE_LOB_PITCH_ANGLE],
	.lob_info.pre_aim_yaw_angle = 0,
	.lob_info.pre_aim_pitch_angle = 35,
	.offset_info=&offset_info,
	.gravity_offset_info.const_k=0,
	.gravity_offset_info.center_of_gravity_angle=43,
	.all_pid_calc=all_pid_calc,
	.work = Gimbal_Work,
	.gimbal_reset_state = DEV_RESET_NO,
};

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  云台pitch轴陀螺仪角度限位
  */
void Gimbal_Pitch_Gyro_Angle_Limit(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.pitch_imu_angle_target;
	if(angle > GIMBAL_MAX_GYRO_ANGEL)
	{
		angle = GIMBAL_MAX_GYRO_ANGEL;
	}
	if(angle < GIMBAL_MIN_GYRO_ANGEL)
	{
		angle = GIMBAL_MIN_GYRO_ANGEL;
	}
	gimbal->base_info.pitch_imu_angle_target = angle;
}

/**
  * @brief  云台pitch轴机械角度限位
  */
void Gimbal_Pitch_Mec_Angle_Limit(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.pitch_mec_angle_target;
	if(angle > GIMBAL_MAX_MEC_ANGEL)
	{
		angle = GIMBAL_MAX_MEC_ANGEL;
	}
	if(angle < GIMBAL_MIN_MEC_ANGEL)
	{
		angle = GIMBAL_MIN_MEC_ANGEL;
	}
	gimbal->base_info.pitch_mec_angle_target = angle;
}

/**
  * @brief  云台陀螺仪yaw目标角度检查 陀螺仪目标角度是累加的需要限制
  * @param  
  * @retval 
  */
void Gimbal_Yaw_Angle_Check(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.yaw_imu_angle_target;//-180°~180°
	if(angle>=10000)//防卡死
	{
		angle =0;
	}
	while (abs(angle) > 180)//有可能卡死
	{
		angle -= 360 * sgn(angle);
	}
	gimbal->base_info.yaw_imu_angle_target = angle;
}

/**
  * @brief  更新陀螺仪和电机角度，速度数据，并进行换算
  * @param  
  * @retval 
  */

#if HERO_TYPE ==3
float k=0.1;
#elif HERO_TYPE ==2
float k=0;
#elif HERO_TYPE ==1
float k=0;
#endif

float add ;//零漂补偿
void Gimbal_External_Date_Update(gimbal_t *gimbal)
{
	/*陀螺仪角度更新***********************************/
	//记得调整正负
	add += k*0.001;//零漂补偿
	gimbal->base_info.yaw_imu_angle   = -imu_sensor.info->base_info.yaw+add;

	float angle=gimbal->base_info.yaw_imu_angle;float max=360;
	while (abs(angle) > (max / 2))//可能卡死
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	gimbal->base_info.yaw_imu_angle = angle;


	gimbal->base_info.yaw_imu_angle = motor_half_cycle(gimbal->base_info.yaw_imu_angle, 360.f);

	gimbal->base_info.yaw_imu_speed   = imu_sensor.info->base_info.rate_yaw;
	gimbal->base_info.pitch_imu_angle = -imu_sensor.info->base_info.roll+sgn(imu_sensor.info->base_info.roll)*180;
	gimbal->base_info.pitch_imu_speed = -imu_sensor.info->base_info.ave_rate_roll;
	
	/*电机数据更新*************************************/
	//yaw轴电机角度更新
	gimbal->base_info.yaw_motor_angle = YAW_MOTOR_ANGLE_MIDDLE - (float)gimbal->gimbal_y->KT_motor_info.rx_info.encoder;
	gimbal->base_info.yaw_motor_angle = motor_half_cycle(gimbal->base_info.yaw_motor_angle, 65536.f);
	gimbal->base_info.yaw_motor_speed = -(float)gimbal->gimbal_y->KT_motor_info.rx_info.speed;  
	//pitch轴电机角度更新
	gimbal->base_info.pitch_motor_angle =  (float)gimbal->gimbal_p->info->angle - PITCH_MOTOR_ENCODER_MIDDLE;
	gimbal->base_info.pitch_motor_angle = motor_half_cycle(gimbal->base_info.pitch_motor_angle, 8192.f);
	gimbal->base_info.pitch_motor_speed = (float)gimbal->gimbal_p->info->speed;
	//360度标准化角度
	gimbal->base_info.pitch_mec_360_angle=gimbal->base_info.pitch_motor_angle/8192.f*360.f;
	gimbal->base_info.yaw_mec_360_angle=gimbal->base_info.yaw_motor_angle/65536.f*360.f;
	
}

/**
 *	@brief 陀螺仪模式云台可控
 */

 
void Gimbal_Gyro_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//对吊射模式的YAW偏置清零
	gimbal->offset_info->lob_yaw_mec_offset = 0;
	gimbal->lob_info.lob_init_angle_flag=0;
	gimbal->offset_info->vision_pitch_offset =0;
	gimbal->offset_info->vision_yaw_offset =0;
	/*控制值接收*/
	if(gimbal->cmd_180->cmd_status != RUNING_C&&gimbal->cmd_l90->cmd_status != RUNING_C&&\
		gimbal->cmd_r90->cmd_status != RUNING_C)//换头或者左右转时遥控器不能改变目标值
	{
		if(car.car_ctrl_mode==RC_CTRL_MODE)
		{
			gimbal->base_info.yaw_imu_angle_target+=rc_sensor.info->ch0*0.001f*0.3;
			gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->ch1*0.001f*0.1;
		}
		else
		{
			gimbal->base_info.yaw_imu_angle_target+=rc_sensor.info->mouse_x * 0.001f;
			gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->mouse_y*0.0005f;
		}
	}
	
	//机械pitch实时更新
	gimbal->base_info.pitch_mec_angle_target = gimbal->base_info.pitch_motor_angle;
	
	//处理特殊命令
	
	/*检测狗洞命令*/
	static	uint8_t tunnel_mode;
	if(command[TUNNEL_MODE].cmd_value==1&&tunnel_mode==0)
	{
		tunnel_mode=1;
	}
	else if((command[TUNNEL_MODE].cmd_value==1&&tunnel_mode==1)||rc_sensor.info->mouse_btn_r.value==1)
	{
		tunnel_mode=0;
		
	}
	/*检测退斜着吊射命令在命令执行里*/
	
		
	
	/*控制方式选择*/
	//pitch
	if(tunnel_mode==1)
	{
		gimbal->ptich_pid_mode = MEC_PID;
		gimbal->base_info.pitch_mec_angle_target =-100;
		//保证转陀螺仪模式正常
		gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
	}
	else
	{
		gimbal->ptich_pid_mode = GYRO_PID;
	}
	//yaw
	if(gimbal->lob_info.out_oblique_head_homing_flag==1)
	{
		
		//云台就近归位
		if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
		{
			gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * 32766.f;
		}
		else
		{
			gimbal->base_info.yaw_mec_angle_target = 0;
			
		}
		gimbal->yaw_pid_mode = MEC_PID;
		//实时更新yaw_mec_angle_target
		gimbal->base_info.yaw_imu_angle_target=gimbal->base_info.yaw_imu_angle;
		//到位退出
		if(abs(gimbal->base_info.yaw_mec_angle_target - gimbal->base_info.yaw_motor_angle) < 5)
		{
			gimbal->lob_info.out_oblique_head_homing_flag=0;
			gimbal->lob_info.out_oblique_head_homing_timeout=0;
			
		}
	}
	else
	{
		gimbal->yaw_pid_mode = GYRO_PID;
	}
	/*心跳*/
	
	if(gimbal->lob_info.out_oblique_head_homing_flag==1)
	{
		gimbal->lob_info.out_oblique_head_homing_timeout++;
	}
	//超时退出
	if(gimbal->lob_info.out_oblique_head_homing_timeout>=1500)
	{
		gimbal->lob_info.out_oblique_head_homing_flag=0;
		gimbal->lob_info.out_oblique_head_homing_timeout=0;
	
	}
	
}

/**
 *	@brief 机械模式云台跟随底盘
 */
void Gimbal_Mec_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//标志位清零
	gimbal->lob_info.lob_init_angle_flag=0;
	//云台就近归位
//	#if HERO_TYPE!=2
	if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
	{
		gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * 32766;//动态调整目标正负，免去pid计算时的半圈处理
	}
	else
	{
		gimbal->base_info.yaw_mec_angle_target = 0;
	}
//	#else
//		gimbal->base_info.yaw_mec_angle_target = 0;
//	#endif
	
	
	//开手打前哨命令保证不动
	if(gimbal->cmd_timer_mec_outpost->cmd_value!=true)
	{
		if (ctrl_mode == 0)
		{
			gimbal->base_info.pitch_mec_angle_target  +=  (float)rc_sensor.info->ch1 / 750.f; 
		}
		else if(ctrl_mode == 1)
		{
			gimbal->base_info.pitch_mec_angle_target  += rc_sensor.info->mouse_vy / 150.f; 
		}
	}
	
	//保证机械模式切陀螺仪模式云台不动
	gimbal->base_info.yaw_imu_angle_target=gimbal->base_info.yaw_imu_angle;
	gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
}

/**
 * @brief 软件重力补偿
 * @note  要放在pid计算之前
 * @author LYQ
 */
void Gimbal_Pitch_Gravity_Offset(gimbal_t *gimbal)
{
	float pitch_angle=gimbal->base_info.pitch_imu_angle;
	float center_of_gravity_angle=gimbal->gravity_offset_info.center_of_gravity_angle;
	gimbal->gravity_offset_info.torque_angle=90.f-center_of_gravity_angle-pitch_angle;
	float output=gimbal->gravity_offset_info.const_k*sin(gimbal->gravity_offset_info.torque_angle);
	gimbal->gravity_offset_info.gravity_offset_output=output;
}
#if HERO_TYPE==2
/**
 * @brief 根据基地是否开花来确定顶部或者底部装甲板pitch、再根据中值切换香蕉道和梯高角度
 */
void Change_Lob_Pitch_Angle(gimbal_t *gimbal)
{
	
		//基地开花
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
				//实际角度偏高
				if(gimbal->lob_info.lob_pitch_type==0)
				{
					//换矮的角度
					gimbal->base_info.pitch_mec_angle_target=GIMBAL_LOB_LOW_MEC_ANGEL;
					gimbal->lob_info.lob_pitch_type=1;
				}
				else//实际角度偏低
				{
					//换高的角度
					gimbal->base_info.pitch_mec_angle_target=GIMBAL_TOP_LOB_LOW_MEC_ANGEL;
					gimbal->lob_info.lob_pitch_type=0;
				}
				
		}
		else//没开花
		{
				//实际角度偏高
				if(gimbal->lob_info.lob_pitch_type==0)
				{
					//换矮的角度
					gimbal->base_info.pitch_mec_angle_target=GIMBAL_LOB_MEC_ANGEL;
					gimbal->lob_info.lob_pitch_type=1;
				}
				else//实际角度偏低
				{
					//换高的角度
					gimbal->base_info.pitch_mec_angle_target=GIMBAL_TOP_LOB_MEC_ANGEL;
					gimbal->lob_info.lob_pitch_type=0;
				}
		}
	
}
#endif

/**
 * @brief 吊射模式初始化完后执行的程序
 * @param gimbal 
 * @param ctrl_mode 0:遥控器模式 1：键盘模式
 */
void Gimbal_Lob_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	
	#ifdef USE_GYRO_LOB
	//不同吊射命令进不一样的初始角度
	if(gimbal->lob_info.into_normal_lob_command_flag==1||gimbal->lob_info.into_outpost_lob_command_flag==1)
	#endif
	{
		if(gimbal->lob_info.lob_init_angle_flag==0&&gimbal->cmd_auto_lob->cmd_status == FINISH_C&&\
		gimbal->cmd_normal_lob->cmd_status == FINISH_C &&gimbal->cmd_oblique_lob->cmd_status == FINISH_C)
		{
		 
		#ifdef ACROSS_LOB
		gimbal->lob_info.lob_init_angle=sgn(gimbal->base_info.yaw_motor_angle) * 16384;
		#else
			
		if(gimbal->lob_info.into_oblique_lob_command_flag==1)
		{
			gimbal->lob_info.lob_init_mec_yaw_angle=gimbal->base_info.yaw_motor_angle+YAW_ONE_DEGREE_VALUE*75.f;
		}
		else
		{
			gimbal->lob_info.lob_init_mec_yaw_angle=gimbal->base_info.yaw_motor_angle;
		}
		
		#endif
		
		gimbal->lob_info.lob_init_angle_flag=1;
		}
	}
	#ifdef USE_GYRO_LOB
	else if(gimbal->lob_info.into_auto_lob_command_flag==1)//持续更新陀螺仪吊射角度
	{
		gimbal->lob_info.gyro_init_lob_pitch_angle=gimbal->lob_info.pre_aim_pitch_angle;
		gimbal->lob_info.gyro_init_lob_yaw_angle=gimbal->lob_info.pre_aim_yaw_angle;
	}
	#endif
	#ifdef LOB_TEST
	//大幅度调整
	if(ctrl_mode==0)
	{
		if(abs(rc_sensor.info->ch1||abs(rc_sensor.info->ch0)>=500))
		{
			#ifdef USE_GYRO_LOB
			//更新不同传感器偏置
			if(gimbal->lob_info.into_auto_lob_command_flag!=1)
			{
				gimbal->base_info.pitch_mec_angle_target+=rc_sensor.info->ch1/660.f/1000.f*100.f;//每秒加100
				gimbal->offset_info->lob_yaw_mec_offset+=rc_sensor.info->ch0/660.f/1000.f*1000;// 
			}
			else
			{
				gimbal->offset_info->lob_pitch_gyro_offset+=rc_sensor.info->ch1/660.f/1000.f*5.f;
				gimbal->offset_info->lob_yaw_gyro_offset+=rc_sensor.info->ch0/660.f/1000.f*5;// 
			}
			#else
				gimbal->base_info.pitch_mec_angle_target+=rc_sensor.info->ch1/660.f/1000.f*100.f;//每秒加100
				gimbal->offset_info->lob_yaw_mec_offset+=rc_sensor.info->ch0/660.f/1000.f*1000;// 
			#endif
			
			
		}
	}
	else
	{
		if(rc_sensor.info->W.status==release_to_press&&rc_sensor.info->Ctrl.value==1)
		{
			gimbal->base_info.pitch_mec_angle_target += PITCH_ONE_DEGREE_VALUE;
		}
		else if(rc_sensor.info->S.status==release_to_press&&rc_sensor.info->Ctrl.value==1 )
		{
			gimbal->base_info.pitch_mec_angle_target -= PITCH_ONE_DEGREE_VALUE;
		}
		else if(rc_sensor.info->D.status==release_to_press&&rc_sensor.info->Ctrl.value==1)
		{
			gimbal->offset_info->lob_yaw_mec_offset+=YAW_ONE_DEGREE_VALUE*2;
		}
		else if(rc_sensor.info->A.status==release_to_press&&rc_sensor.info->Ctrl.value==1)
		{
			gimbal->offset_info->lob_yaw_mec_offset-=YAW_ONE_DEGREE_VALUE*2;
		}
 
	}
	
	#endif
	//加偏置
	#ifdef USE_GYRO_LOB
	//目标角度赋值
	if(gimbal->lob_info.into_normal_lob_command_flag==1||gimbal->lob_info.into_outpost_lob_command_flag==1)
	{
		gimbal->base_info.yaw_mec_angle_target = gimbal->lob_info.lob_init_mec_yaw_angle+ \
											gimbal->offset_info->lob_yaw_mec_offset;
		//实时更新角度
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
		gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
		gimbal->ptich_pid_mode = MEC_PID;
		gimbal->yaw_pid_mode   = MEC_PID;
	}
	else if(gimbal->lob_info.into_auto_lob_command_flag==1)
	{
		//目标角度一直等于视觉发过来的角度+自己的偏置
		gimbal->base_info.pitch_imu_angle_target=gimbal->lob_info.pre_aim_pitch_angle+gimbal->offset_info->lob_pitch_gyro_offset;
		gimbal->base_info.yaw_imu_angle_target=gimbal->lob_info.pre_aim_yaw_angle+gimbal->offset_info->lob_yaw_gyro_offset;
		gimbal->ptich_pid_mode = GYRO_PID;
		gimbal->yaw_pid_mode   = GYRO_PID;
		//实时更新机械目标角度
		gimbal->base_info.yaw_mec_angle_target=gimbal->base_info.yaw_motor_angle;
		gimbal->base_info.pitch_mec_angle_target=gimbal->base_info.pitch_motor_angle;
	}
		
	#else
	gimbal->base_info.yaw_mec_angle_target = gimbal->lob_info.lob_init_mec_yaw_angle+ \
											gimbal->offset_info->lob_yaw_mec_offset;
											
											
	#endif
	//吊射底部装甲板 start
	static	uint8_t last_is_base_open;
	uint8_t is_base_open =communicate.game_robot_status_rx_info->game_process.bit.is_base_open;
	 //上升沿触发
	if(is_base_open==1&&last_is_base_open==0&&car.car_move_mode==lob_CAR&&
		gimbal->lob_info.into_outpost_lob_command_flag==0)
		
	{
		gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_LOW_MEC_ANGEL;
	}
	
		last_is_base_open=is_base_open;
	//吊射底部装甲板 end
	#ifndef USE_GYRO_LOB
	if(gimbal->cmd_auto_lob->cmd_status == FINISH_C&&\
		gimbal->cmd_normal_lob->cmd_status == FINISH_C&&\
		gimbal->cmd_oblique_lob->cmd_status == FINISH_C)//保险
	{
		//陀螺仪目标角度更新
		 
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
		gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
		gimbal->ptich_pid_mode = MEC_PID;
		gimbal->yaw_pid_mode   = MEC_PID;
	}
	#endif
}

/**
 * @brief 视觉模式遥控器更新
 * @param gimbal 
 * @param ctrl_mode 0:遥控器模式 1：键盘模式
 */
void Gimbal_Vision_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//对吊射模式的YAW偏置清零
	gimbal->offset_info->lob_yaw_mec_offset = 0;
	gimbal->lob_info.lob_init_angle_flag=0;
	if(vision.status->rx_state == DEV_ONLINE && vision.rx_info->is_find_target == 1)//发现敌人
	{
		if(vision.rx_info->yaw <= 8192 && vision.rx_info->yaw >= 0)
		{
			gimbal->base_info.yaw_imu_angle_target = ((4096.f - vision.rx_info->yaw) / 4096.f) * 180.f ;
		}
		if(vision.rx_info->pitch <= 8192 && vision.rx_info->pitch >= 0)
		{
			gimbal->base_info.pitch_imu_angle_target = ((4096.f - vision.rx_info->pitch) / 4096.f) * 180.f ;
		}
	}
	else//没有发现敌人，正常控制
	{
		Gimbal_Gyro_Update(gimbal,ctrl_mode);
	}

	//机械pitch实时更新
	gimbal->base_info.pitch_mec_angle_target = gimbal->base_info.pitch_motor_angle;
	
}

/**
 *	@brief 初始化时执行的程序
 */
void Gimbal_init_action(gimbal_t *gimbal)
{
		gimbal->offset_info->lob_yaw_mec_offset=0;
		gimbal->lob_info.lob_init_angle_flag=0;
		//设置初始化目标值
		gimbal->base_info.pitch_mec_angle_target = 0;
		gimbal->base_info.pitch_imu_angle_target = 0;
		//云台就近归位
		
		if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
		{
			gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * 32766.f;
		}
		else
		{
			gimbal->base_info.yaw_mec_angle_target = 0;
		}
		
		
		//设置pid为机械模式
		gimbal->ptich_pid_mode = MEC_PID;
		gimbal->yaw_pid_mode = MEC_PID;
		//判断初始化是否完成
		if(abs(gimbal->base_info.yaw_motor_speed) <= 20 && abs(abs(gimbal->base_info.yaw_motor_angle) - abs(gimbal->base_info.yaw_mec_angle_target)) <= 20   \
		&& (abs(gimbal->base_info.pitch_motor_angle)-abs(gimbal->base_info.pitch_mec_angle_target)) <= 20 && abs(gimbal->base_info.pitch_motor_speed) <= 50 )
		{
			gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;//car初始化里有，可以试试能不能删去
			gimbal->gimbal_reset_state = DEV_RESET_OK;
		}
}

/**
 * @brief 云台pitch轴PID计算
 * 
 * @param gimbal 
 */
void Gimbal_Pitch_Pid_Calculating(gimbal_t *gimbal)
{
	float gyro_meas_in,gyro_meas_out,gyro_target,mec_meas_in,mec_meas_out,mec_target,shoot_offset_current;

	switch (gimbal->ptich_pid_mode)
	{
	case GYRO_PID:
		gyro_meas_out = gimbal->base_info.pitch_imu_angle;				//外环
		gyro_meas_in = gimbal->base_info.pitch_imu_speed;			  //内环
		gyro_target = gimbal->base_info.pitch_imu_angle_target;  //目标值
		
		shoot_offset_current=shooting.shooting_shake_angle.shoot_pitch_offset_current;//发射抖动补偿电流
		gimbal->base_info.output_gimbal_p = feedforward_pid_calc(&gimbal->gimbal_p->motor_all_pid.gyro_pid.angle,&gimbal->gimbal_p->motor_all_pid.gyro_pid.speed,gyro_target,gyro_meas_out,gyro_meas_in,-1,0);
		break;

	case MEC_PID:
		mec_meas_out = gimbal->base_info.pitch_motor_angle;		        //外环
		mec_meas_in = gimbal->base_info.pitch_imu_speed;			      //内环  
		mec_target = gimbal->base_info.pitch_mec_angle_target;				//目标值 
		shoot_offset_current=shooting.shooting_shake_angle.shoot_pitch_offset_current;//发射抖动补偿电流
	
		
		gimbal->gimbal_p->motor_all_pid.mec_pid.angle.feedforward.const_val=shoot_offset_current;
		gimbal->base_info.output_gimbal_p = feedforward_pid_calc(&gimbal->gimbal_p->motor_all_pid.mec_pid.angle,&gimbal->gimbal_p->motor_all_pid.mec_pid.speed,mec_target,mec_meas_out,mec_meas_in,-1,0);
		break;

	case SPEED_PID:

		gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc( NULL,&gimbal->gimbal_p->motor_all_pid.speed_pid.speed,0,NULL,gimbal->base_info.pitch_imu_speed,-1,0);
		break;

	default:
		break;
	}
}

/**
 * @brief 云台yaw轴PID计算
 * @param gimbal 
 */
void Gimbal_Yaw_Pid_Calculating(gimbal_t *gimbal)
{
	float gyro_meas_in,gyro_meas_out,gyro_target,mec_meas_in,mec_meas_out,mec_target;

	switch (gimbal->yaw_pid_mode)
	{
	case GYRO_PID:
		gyro_meas_out = gimbal->base_info.yaw_imu_angle;				//外环
		gyro_meas_in = gimbal->base_info.yaw_imu_speed	;			  //内环
		gyro_target = gimbal->base_info.yaw_imu_angle_target;  //目标值
		
		gimbal->base_info.output_gimbal_y = -gimbal->all_pid_calc( &gimbal->gimbal_y->motor_all_pid.gyro_pid.angle,&gimbal->gimbal_y->motor_all_pid.gyro_pid.speed,gyro_target,gyro_meas_out,gyro_meas_in,-1,3);
		break;

	case MEC_PID:
		mec_meas_out = (float)gimbal->base_info.yaw_motor_angle / 32768.f * 180.f;   //外环 转为角度
		mec_meas_in = gimbal->base_info.yaw_imu_speed;			            	   //内环 
		mec_target = gimbal->base_info.yaw_mec_angle_target / 32768.f * 180.f;
		
		gimbal->base_info.output_gimbal_y = -gimbal->all_pid_calc( &gimbal->gimbal_y->motor_all_pid.mec_pid.angle,&gimbal->gimbal_y->motor_all_pid.mec_pid.speed,mec_target,mec_meas_out,mec_meas_in,-1,3);
		break;
	
	case SPEED_PID:
		break;
	default:
		break;
	}
}
/**
*@brief 预瞄角度更新
*/
void Gimbal_pre_aim_angle_update(gimbal_t *gimbal)
{
	
	if(vision.status->rx_state==DEV_ONLINE)
	{
		/*预瞄角度更新************************************/
		//当前角度+偏置
		if(vision.rx_info->building_pitch <= 8192 && vision.rx_info->building_yaw >= 0)
		{
			gimbal->lob_info.pre_aim_yaw_angle = ((4096.f - vision.rx_info->building_yaw) / 4096.f) * 180.f ;
		}
		if(vision.rx_info->pitch <= 8192 && vision.rx_info->pitch >= 0)
		{
			gimbal->lob_info.pre_aim_pitch_angle = ((4096.f - vision.rx_info->building_pitch) / 4096.f) * 180.f ;
		}
		return;
	}//如果视觉不在线就定成当前角度
	else
	{
		gimbal->lob_info.pre_aim_yaw_angle=gimbal->base_info.yaw_imu_angle;
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
			gimbal->lob_info.pre_aim_pitch_angle=GIMBAL_LOB_MEC_ANGEL*0.0439453125;//  /8192.f*360.f
		}
		else
		{
			gimbal->lob_info.pre_aim_pitch_angle=GIMBAL_LOB_MEC_ANGEL*0.0439453125;//  /8192.f*360.f
			
		}
			
		
	
	}
	return;

	//磁力计，uwb，不能用
//	float distance=communicate.game_robot_pos_rx_info->target_distance;
//	float discriminant = 1.1090 * 1.1090 - 4 * (-0.0121) * (-5.3547 - distance);
//	if (discriminant < 0 || distance < 1) 
//	{
//		return; // 如果判别式小于0或 distance 不满足条件，直接返回
//	}

//	float delta = sqrt(discriminant); // 计算平方根
//	float predict_pitch = (-1.1090 + delta) / (2.f * -0.0121); // 正确的二次方程解公式

//	gimbal->lob_info.pre_aim_pitch_angle = predict_pitch; // 将结果赋值
//	//半圈处理
//	float temp_pre_aim_yaw_angle =gimbal->base_info.yaw_imu_angle+communicate.game_robot_pos_rx_info->angle_err;
//	gimbal->lob_info.pre_aim_yaw_angle = motor_half_cycle(temp_pre_aim_yaw_angle,360);
	
	
}
/**
 * @brief 云台吊射pitch角度选择
 */
#if HERO_TYPE==2
void Gimbal_Lob_Pitch_check(gimbal_t *gimbal)
{
	//基地开花
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
			if(communicate.power_heat_data_rx_info->rfid==1)
			{
				gimbal->lob_info.lob_pitch_type=1;
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_TOP_LOB_LOW_MEC_ANGEL;//梯高吊射底部
			}
			else
			{	
				gimbal->lob_info.lob_pitch_type=0;
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_LOW_MEC_ANGEL;//香蕉道吊射底部
			}
			
		}
		else//没开花
		{
			if(communicate.power_heat_data_rx_info->rfid==1)
			{
				gimbal->lob_info.lob_pitch_type=1;
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_TOP_LOB_MEC_ANGEL;//梯高吊射顶部
			}
			else
			{
				gimbal->lob_info.lob_pitch_type=0;
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_MEC_ANGEL;//香蕉道吊射顶部
			}
		}
}
#endif


/**
 * @brief 云台命令执行
 * @param gimbal  
 */

void Gimbal_Cmd_Excute(gimbal_t *gimbal)
{
	//云台右转90
	if(gimbal->cmd_r90->cmd_value == true) 
	{
		gimbal->base_info.yaw_imu_angle_target += 90;
		gimbal->cmd_r90->s_run(gimbal->cmd_r90);
	}
	if(gimbal->cmd_r90->cmd_status == RUNING_C&&
		abs(gimbal->base_info.yaw_imu_angle - gimbal->base_info.yaw_imu_angle_target) < 3)
 	{
		gimbal->cmd_r90->s_finish(gimbal->cmd_r90);
 	}
	//云台左转90
	if(gimbal->cmd_l90->cmd_value == true) 
	{
		gimbal->base_info.yaw_imu_angle_target -= 90;
		gimbal->cmd_l90->s_run(gimbal->cmd_l90);
	}
	if(gimbal->cmd_l90->cmd_status == RUNING_C&&
		abs(gimbal->base_info.yaw_imu_angle - gimbal->base_info.yaw_imu_angle_target) < 3)
 	{
		gimbal->cmd_l90->s_finish(gimbal->cmd_l90);
 	}
	//云台转180
	if(gimbal->cmd_180->cmd_value == true)
	{
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle - sgn(gimbal->base_info.yaw_imu_angle)*179.f;
		gimbal->cmd_180->s_run(gimbal->cmd_180);
	}
	//云台转180完成判断
  if(gimbal->cmd_180->cmd_status == RUNING_C&&
		abs(gimbal->base_info.yaw_imu_angle - gimbal->base_info.yaw_imu_angle_target) < 3)
 	{
		gimbal->cmd_180->s_finish(gimbal->cmd_180);
 	}
	/*吊射偏置*/
	//MARK:退出机械模式之后偏置会在gimbal_move和Gimbal_Vision_Update中清零
	if ( car.car_move_mode == lob_CAR)
	{

			//云台向上微调
			if(gimbal->cmd_up->cmd_value == true)
			{
				gimbal->base_info.pitch_mec_angle_target += LOB_PITCH_OFFSET_STEP;
			}
			//云台向下微调
			if(gimbal->cmd_dowm->cmd_value == true)
			{
				gimbal->base_info.pitch_mec_angle_target -= LOB_PITCH_OFFSET_STEP;
			};
			//云台向左微调
			if(gimbal->cmd_left->cmd_value == true)
			{
				gimbal->offset_info->lob_yaw_mec_offset -= LOB_YAW_OFFSET_STEP;
			}
			//云台向右微调
			if(gimbal->cmd_right->cmd_value == true)
			{
				gimbal->offset_info->lob_yaw_mec_offset += LOB_YAW_OFFSET_STEP;
			}
		
	}
		/*视觉偏置,视觉方主动处理偏置*/
	    if (car.car_move_mode == vision_cycle_CAR || car.car_move_mode == vision_gyro_CAR)
	    {
			//云台向上微调
			if(gimbal->cmd_up->cmd_value == true)
			{
				gimbal->offset_info->vision_pitch_offset -= 3;
			}
			//云台向下微调
			if(gimbal->cmd_dowm->cmd_value == true)
			{
				gimbal->offset_info->vision_pitch_offset += 3;
			};
			//云台向左微调   
			if(gimbal->cmd_left->cmd_value == true)
			{
				gimbal->offset_info->vision_yaw_offset += 3;
			}
			//云台向右微调
			if(gimbal->cmd_right->cmd_value == true)
			{
				gimbal->offset_info->vision_yaw_offset -= 3;
			}
		}
	//手打前哨命令
	static uint8_t init_angle_flag;//用吊射模式打，初始化云台角度
	if(gimbal->cmd_timer_mec_outpost->cmd_value==true)
	{
		if(init_angle_flag==0)
		{
			gimbal->base_info.pitch_mec_angle_target=gimbal->base_info.pitch_motor_angle;
			gimbal->lob_info.lob_init_mec_yaw_angle=gimbal->base_info.yaw_motor_angle;
			gimbal->offset_info->lob_yaw_mec_offset=0;//lob_init_mec_yaw_angle重新复位，偏置清零
			init_angle_flag=1;
			gimbal->lob_info.lob_init_angle_flag=1;//防止进吊射模式后重新初始化角度
		}
	}
	if(gimbal->cmd_timer_mec_outpost->cmd_value!=true)
	{
		init_angle_flag=0;//清标志位
	}
	
	//一键吊射
	if(gimbal->cmd_auto_lob->cmd_value == true)
	{
		#ifdef GYRO_LOB_INIT
		//初次赋值，在Chassis_Cmd_Excute里持续赋值，直到误差小于0.5
		
		gimbal->lob_info.gyro_init_lob_yaw_angle=gimbal->lob_info.pre_aim_yaw_angle;
		gimbal->lob_info.gyro_init_lob_pitch_angle=gimbal->lob_info.pre_aim_pitch_angle;
		
		gimbal->cmd_auto_lob->s_run(gimbal->cmd_auto_lob);
		
		#else
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
			gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_LOW_MEC_ANGEL;
		}
		else
		{
			gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_MEC_ANGEL;
			
		}
		
		#endif
	}
	//普通吊射
	if(gimbal->cmd_normal_lob->cmd_value == true)
	{
		#if HERO_TYPE==2
		Gimbal_Lob_Pitch_check(gimbal);
		#else
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_LOW_MEC_ANGEL;//香蕉道吊射底部
			
		}
		else//没开花
		{ 
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_MEC_ANGEL;//香蕉道吊射顶部
		}
		#endif
		
		gimbal->cmd_normal_lob->s_run(gimbal->cmd_normal_lob);
	}
	//斜着吊射
	if(gimbal->cmd_oblique_lob->cmd_value == true)
	{
		#if HERO_TYPE==2
		Gimbal_Lob_Pitch_check(gimbal);
		#else
		if(communicate.game_robot_status_rx_info->game_process.bit.is_base_open==1)
		{
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_LOW_MEC_ANGEL;//香蕉道吊射底部
			
		}
		else//没开花
		{ 
				gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_MEC_ANGEL;//香蕉道吊射顶部
		}
		#endif
		gimbal->cmd_oblique_lob->s_run(gimbal->cmd_oblique_lob);
	}
	//切换pitch吊射角度
	if(gimbal->cmd_change_lob_pitch->cmd_value == true)
	{
		#if HERO_TYPE==2
		Change_Lob_Pitch_Angle(gimbal);
		#endif
		
	}
	
	/*检测退出斜着吊射命令标志位*/
	if(gimbal->lob_info.into_oblique_lob_command_flag==0&& \
		gimbal->lob_info.last_into_oblique_lob_command_flag==1)
	{
		gimbal->lob_info.out_oblique_head_homing_flag=1;
	
	}
	gimbal->lob_info.last_into_oblique_lob_command_flag=gimbal->lob_info.into_oblique_lob_command_flag;
	
}

/**
 *  @name  Gimbal_Work
 *	@brief 云台总控 
 */

void Gimbal_Work(gimbal_t *gimbal)
{
	Gimbal_pre_aim_angle_update(gimbal);
	Gimbal_External_Date_Update(gimbal);
	Gimbal_Cmd_Excute(gimbal);
	switch(car.car_move_mode)
	{
		case offline_CAR://离线
			gimbal->gimbal_reset_state = DEV_RESET_NO;

			break;
		
		case init_CAR://初始化
			Gimbal_init_action(gimbal);
			break;
		
		case gyro_CAR://陀螺仪模式
		case cycle_CAR://小陀螺模式
			//目标值更新
			if(gimbal->cmd_180->cmd_status != RUNING_C)//换头时遥控器不能改变目标值
			{
				Gimbal_Gyro_Update(gimbal,car.car_ctrl_mode);
			}
			
			break;
		
		case mec_CAR://机械模式
			 
				Gimbal_Mec_Update(gimbal,car.car_ctrl_mode);
				//设置PID模式
				gimbal->ptich_pid_mode = MEC_PID;
				gimbal->yaw_pid_mode = MEC_PID;
			
			break;
			
		case lob_CAR://吊射模式
			//手打前哨模式更新标志位
			if(gimbal->cmd_timer_mec_outpost->cmd_value==true)
			{
				gimbal->lob_info.into_outpost_lob_command_flag=1;
			}
			if (gimbal->cmd_auto_lob->cmd_status == RUNING_C)
			{
			    #ifdef GYRO_LOB_INIT
				gimbal->ptich_pid_mode = GYRO_PID;
				//仅预瞄吊射时需要实时更新pitch，普通吊射不用
				gimbal->base_info.pitch_mec_angle_target=gimbal->base_info.pitch_motor_angle;
				gimbal->lob_info.into_auto_lob_command_flag=1;

				#else
				gimbal->ptich_pid_mode = MEC_PID;
				#endif
				
				gimbal->yaw_pid_mode = GYRO_PID;
			}
			else if(gimbal->cmd_normal_lob->cmd_status == RUNING_C)
			{
				gimbal->ptich_pid_mode = MEC_PID;
				gimbal->yaw_pid_mode = GYRO_PID;
				gimbal->lob_info.into_normal_lob_command_flag=1;
			}
			else if(gimbal->cmd_oblique_lob->cmd_status == RUNING_C)
			{
				gimbal->ptich_pid_mode = MEC_PID;
				gimbal->yaw_pid_mode = GYRO_PID;
				gimbal->lob_info.into_oblique_lob_command_flag=1;
			}
			
			
			//规范时序
			else if(gimbal->lob_info.into_auto_lob_command_flag==1 \
				||gimbal->lob_info.into_outpost_lob_command_flag==1 \
				||gimbal->lob_info.into_normal_lob_command_flag==1 \
				||gimbal->lob_info.into_oblique_lob_command_flag==1)
			{
				//在Gimbal_Lob_Update里更新了PID模式
				Gimbal_Lob_Update(gimbal,car.car_ctrl_mode);
				
			}
			
			break;
			
		case vision_gyro_CAR://视觉模式
		case vision_cycle_CAR:
			//目标值更新
			Gimbal_Vision_Update(gimbal,car.car_ctrl_mode);
			//设置PID模式
			gimbal->ptich_pid_mode = GYRO_PID;
			gimbal->yaw_pid_mode = GYRO_PID;
			break;
		
		default:
			break;
	}
	//yaw的imu角度限制已Gimbal_Gyro_Update更新
	Gimbal_Yaw_Angle_Check(gimbal);//Yaw角度检查
	Gimbal_Pitch_Mec_Angle_Limit(gimbal);//pitch机械角度限位
	Gimbal_Pitch_Gyro_Angle_Limit(gimbal);//pitch陀螺仪角度限位
	/**发给电机*/
		if(car.car_move_mode != offline_CAR)//开控
		{
			//pid计算
			#ifdef SHOOT_PITCH_SHAKE_OFFSET
			Shoot_pitch_shake_restrict(&shooting);
			#endif
			Gimbal_Yaw_Pid_Calculating(gimbal);
			Gimbal_Pitch_Pid_Calculating(gimbal);
			//输出赋值
			gimbal->gimbal_y->base_info.motor_out=gimbal->base_info.output_gimbal_y; 
			gimbal->gimbal_p->base_info.motor_out=gimbal->base_info.output_gimbal_p;
			
		}
	else//关控
	{
		gimbal->gimbal_y->base_info.motor_out=0; 
		gimbal->gimbal_p->base_info.motor_out=0;
	}
	
}



