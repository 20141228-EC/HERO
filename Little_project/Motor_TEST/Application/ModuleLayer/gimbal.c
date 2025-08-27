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
	
	.lob_yaw_offset = 0,    //吊射偏置
	
};

gimbal_t gimbal = 
{
	.gimbal_p=&rm_motor[GIMB_P],
	.gimbal_y=&kt_motor[0],
	.cmd_r90 = &command[CAR_R90],
	.cmd_l90 = &command[CAR_L90],
	.cmd_180 = &command[GIM_180],
	.cmd_auto_lob = &command[AUTO_LOB],
	.cmd_right = &command[GIM_RIGHT],
	.cmd_left = &command[GIM_LEFT],
	.cmd_up = &command[GIM_UP],
	.cmd_dowm = &command[GIM_DOWM],
	.offset_info=&offset_info,
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
  * @brief  更新陀螺仪和电机角度，速度数据，并进行换算
  * @param  
  * @retval 
  */
void Gimbal_External_Date_Update(gimbal_t *gimbal)
{
	/*陀螺仪角度更新***********************************/
	//记得调整正负
	gimbal->base_info.yaw_imu_angle   = -imu_sensor.info->base_info.yaw;
	gimbal->base_info.yaw_imu_speed   = imu_sensor.info->base_info.rate_yaw;
	gimbal->base_info.pitch_imu_angle = -imu_sensor.info->base_info.roll+sgn(imu_sensor.info->base_info.roll)*180;
	gimbal->base_info.pitch_imu_speed = -imu_sensor.info->base_info.ave_rate_roll;
	
	/*电机数据更新*************************************/
	//过零点处理
	gimbal->base_info.yaw_motor_angle = YAW_MOTOR_ANGLE_MIDDLE - (float)gimbal->gimbal_y->KT_motor_info.rx_info.encoder;
	gimbal->base_info.yaw_motor_angle = motor_half_cycle(gimbal->base_info.yaw_motor_angle, 65536.f);
	//yaw轴电机角度更新
	gimbal->base_info.yaw_motor_speed = -(float)gimbal->gimbal_y->KT_motor_info.rx_info.speed;  
	//pitch轴电机角度更新
	gimbal->base_info.pitch_motor_angle =  (float)gimbal->gimbal_p->info->angle - PITCH_MOTOR_ENCODER_MIDDLE;
	gimbal->base_info.pitch_motor_angle = motor_half_cycle(gimbal->base_info.pitch_motor_angle, 8192.f);
	gimbal->base_info.pitch_motor_speed = (float)gimbal->gimbal_p->info->speed;  
}

/*
 *	@brief 陀螺仪模式云台可控
 */
void Gimbal_Gyro_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//对吊射模式的YAW偏置清零
	gimbal->offset_info->lob_yaw_offset = 0;
	gimbal->lob_info.lob_init_angle_flag=0;
	
	if(gimbal->cmd_180->cmd_status != RUNING_C)//换头时遥控器不能改变目标值
	{
		if(car.car_ctrl_mode==RC_CTRL_MODE)
		{
			gimbal->base_info.yaw_imu_angle_target+=rc_sensor.info->ch0*0.0002f;
			
			gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->ch1*0.0001f;
			
		}
		else
		{
			gimbal->base_info.yaw_imu_angle_target+=rc_sensor.info->mouse_x * 0.0005f;
			gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->mouse_y*0.0005f;
		}
	}
	//累加限制
	while (abs(gimbal->base_info.yaw_imu_angle_target) > 180)
	{
		gimbal->base_info.yaw_imu_angle_target -= 360 * sgn(gimbal->base_info.yaw_imu_angle_target);
	}
	//云台主动模式下不用更新yaw机械目标值为实时，因为切换机械模式是控底盘而不是控云台
	
	//机械pitch实时更新
	gimbal->base_info.pitch_mec_angle_target = gimbal->base_info.pitch_motor_angle;
	
}

/*
 *	@brief 机械模式云台跟随底盘
 */
void Gimbal_Mec_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//云台就近归位
	gimbal->lob_info.lob_init_angle_flag=0;
	if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
	{
		gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * 32766;//动态调整目标正负，免去pid计算时的半圈处理
	}
	else
	{
		gimbal->base_info.yaw_mec_angle_target = 0;
	}
	
	if (ctrl_mode == 0)
	{
	
		gimbal->base_info.pitch_mec_angle_target  +=  (float)rc_sensor.info->ch1 / 750.f; 
		
	}
	else if(ctrl_mode == 1)
	{
		gimbal->base_info.pitch_mec_angle_target  -= rc_sensor.info->mouse_vy / 3000.f; 
	}
	
	//保证机械模式切陀螺仪模式云台不动
	gimbal->base_info.yaw_imu_angle_target=gimbal->base_info.yaw_imu_angle;
	gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
 
}

/**
 * @brief 吊射模式初始化完后执行的程序
 * @note  只能用键盘微调
 * @param gimbal 
 * @param ctrl_mode 0:遥控器模式 1：键盘模式
 */
void Gimbal_Lob_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//初始化吊射角度
	if(gimbal->lob_info.lob_init_angle_flag==0&&gimbal->cmd_auto_lob->cmd_status == FINISH_C)
	{
		 
		#ifdef ACROSS_LOB
		gimbal->lob_info.lob_init_angle=sgn(gimbal->base_info.yaw_motor_angle) * 16384;
		#else
		if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
		{
			gimbal->lob_info.lob_init_angle = sgn(gimbal->base_info.yaw_motor_angle) * 32766;
		}
		else
		{
		gimbal->lob_info.lob_init_angle = 0;
		}
		#endif
		gimbal->lob_info.lob_init_angle_flag=1;
	}	
	//加偏置
	gimbal->base_info.yaw_mec_angle_target = gimbal->lob_info.lob_init_angle+gimbal->offset_info->lob_yaw_offset;
	
	if(gimbal->cmd_auto_lob->cmd_status != RUNING_C)//双重判断可能更稳定一点吧hhh
	{
		//陀螺仪目标角度更新
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
		gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
		gimbal->ptich_pid_mode = MEC_PID;
		gimbal->yaw_pid_mode   = MEC_PID;
	}
	
}

/**
 * @brief 视觉模式遥控器更新
 * @param gimbal 
 * @param ctrl_mode 0:遥控器模式 1：键盘模式
 */
void Gimbal_Vision_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	//对吊射模式的YAW偏置清零
	gimbal->offset_info->lob_yaw_offset = 0;
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

/*
 *	@brief 初始化时执行的程序
 */
void Gimbal_init_action(gimbal_t *gimbal)
{
		gimbal->offset_info->lob_yaw_offset=0;
		gimbal->lob_info.lob_init_angle_flag=0;
		//设置初始化目标值
			gimbal->base_info.pitch_mec_angle_target = 0;
			gimbal->base_info.pitch_imu_angle_target = 0;
			//云台就近归为
			if (abs(gimbal->base_info.yaw_motor_angle) > 16384)
			{
				gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * 32766;
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
	float gyro_meas_in,gyro_meas_out,gyro_target,mec_meas_in,mec_meas_out,mec_target;

	switch (gimbal->ptich_pid_mode)
	{
	case GYRO_PID:
		gyro_meas_out = gimbal->base_info.pitch_imu_angle;				//外环
		gyro_meas_in = gimbal->base_info.pitch_imu_speed;			  //内环
		gyro_target = gimbal->base_info.pitch_imu_angle_target;  //目标值
		
		gimbal->base_info.output_gimbal_p = feedforward_pid_calc(&gimbal->gimbal_p->motor_all_pid.gyro_pid.angle,&gimbal->gimbal_p->motor_all_pid.gyro_pid.speed,gyro_target,gyro_meas_out,gyro_meas_in,-1,0);
		break;
	
	case MEC_PID:
		mec_meas_out = gimbal->base_info.pitch_motor_angle;		        //外环
		mec_meas_in = gimbal->base_info.pitch_imu_speed;			      //内环  
		mec_target = gimbal->base_info.pitch_mec_angle_target;				//目标值 

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
 * 
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
		
		gimbal->base_info.output_gimbal_y = -gimbal->all_pid_calc( &gimbal->gimbal_y->motor_all_pid.mec_pid.angle,&gimbal->gimbal_y->motor_all_pid.mec_pid.speed,mec_target,mec_meas_out,mec_meas_in,-1,4);
		break;
	
	case SPEED_PID:
		break;
	default:
		break;
	}
}

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
	}
	//云台左转90
	if(gimbal->cmd_l90->cmd_value == true) 
	{
		gimbal->base_info.yaw_imu_angle_target -= 90;
	}
	//云台转180
	if(gimbal->cmd_180->cmd_value == true)
	{
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle - sgn(gimbal->base_info.yaw_imu_angle)*180.f;
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
	if (car.car_move_mode == mec_CAR || car.car_move_mode == lob_CAR)
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
			gimbal->offset_info->lob_yaw_offset -= LOB_YAW_OFFSET_STEP;
		}
		//云台向右微调
		if(gimbal->cmd_right->cmd_value == true)
		{
			gimbal->offset_info->lob_yaw_offset += LOB_YAW_OFFSET_STEP;
		}
	}
		/*视觉偏置,视觉方主动处理偏置*/
	    if (car.car_move_mode == vision_cycle_CAR || car.car_move_mode == vision_gyro_CAR)
	    {
			//云台向上微调
			if(gimbal->cmd_up->cmd_value == true)
			{
				gimbal->offset_info->vision_pitch_offset -= 5;
			}
			//云台向下微调
			if(gimbal->cmd_dowm->cmd_value == true)
			{
				gimbal->offset_info->vision_pitch_offset += 5;
			};
			//云台向左微调   
			if(gimbal->cmd_left->cmd_value == true)
			{
				gimbal->offset_info->vision_yaw_offset += 1;
			}
			//云台向右微调
			if(gimbal->cmd_right->cmd_value == true)
			{
				gimbal->offset_info->vision_yaw_offset -= 1;
			}
		}
		//一键吊射
	if(gimbal->cmd_auto_lob->cmd_value == true)
	{
		float base_lob_pitch=30;float base_lob_yaw=0;
		#ifdef GYRO_LOB_INIT
		gimbal->base_info.pitch_imu_angle = base_lob_pitch ;
		gimbal->base_info.yaw_imu_angle_target = base_lob_yaw ;
		#else
		gimbal->base_info.pitch_mec_angle_target = GIMBAL_LOB_MED_ANGEL;
		#endif
	}

}

/**
 *  @name  Gimbal_Work
 *	@brief 新云台Yaw总控 
 */

void Gimbal_Work(gimbal_t *gimbal)
{
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
			//设置PID模式
			gimbal->ptich_pid_mode = GYRO_PID;
			gimbal->yaw_pid_mode = GYRO_PID;
			break;
		
		case mec_CAR://机械模式
			//如果正在进行吊射初始化则Yaw轴imu定向
			//目标值更新
			if (gimbal->cmd_auto_lob->cmd_status == RUNING_C)
			{
				gimbal->yaw_pid_mode = GYRO_PID;
			}
			else
			{
				Gimbal_Mec_Update(gimbal,car.car_ctrl_mode);
				//设置PID模式
				gimbal->ptich_pid_mode = MEC_PID;
				gimbal->yaw_pid_mode = MEC_PID;
			}
			break;
			
		case lob_CAR://吊射模式
			//目标值更新
			if (gimbal->cmd_auto_lob->cmd_status == RUNING_C)
			{
			    #ifdef GYRO_LOB_INIT
				gimbal->ptich_pid_mode = GYRO_PID;
				#else
				gimbal->ptich_pid_mode = MEC_PID;
				#endif
				
				gimbal->yaw_pid_mode = GYRO_PID;
			}
			else
			{
				Gimbal_Lob_Update(gimbal,car.car_ctrl_mode);
				//在Gimbal_Lob_Update里设置了PID模式
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
	Gimbal_Pitch_Mec_Angle_Limit(gimbal);//pitch机械角度限位
	Gimbal_Pitch_Gyro_Angle_Limit(gimbal);//pitch陀螺仪角度限位
	/**发给电机*/
		if(car.car_move_mode != offline_CAR)//开控
		{
			//pid计算
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



