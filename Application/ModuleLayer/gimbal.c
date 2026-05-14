#include "gimbal.h"


gimbal_offset_info_t offset_info =
{
	.vision_yaw_offset = 0, //视觉偏置 
	.lob_yaw_mec_offset = 0,    //吊射偏置
	.lob_pitch_gyro_offset = 0,
	.lob_pitch_mec_offset = 0,
};

gimbal_t gimbal=
{
	.gimbal_y = &Yaw_Motor,
	.offset_info = &offset_info,
	.all_pid_calc = &all_pid_calc,
	.work = Gimbal_Work,
	.yaw_pid_mode = 0,
	.gimbal_reset_state = DEV_RESET_NO,
	.gimbal_ctrl_mode = 1,//陀螺仪
	.lob_info.pre_aim_yaw_angle = 45,
	.base_info.init_time=0,
	.base_info.init_time_max=3000,	
	.base_info.init_time_max_count=0,
  .base_info.pitch_imu_angle_target = 0,
	.base_info.pitch_mec_angle_target = 0,
	.base_info.turn_time = 0,
	.base_info.turn_time_max = 2000,
	.base_info.Gimbal_Turn_Finish = 0,
	.lob_info.gyro_lob_pitch_angle_dynamic_tar = 0,
	.lob_info.gyro_lob_yaw_angle_dynamic_tar = 0,
};

/*云台状态更新*/
void Gimbal_Status_Update(gimbal_t *gimbal)
{
	switch(Balance.mode)
	{
		case Init_Mode:			
		case Mec_Mode:
		case Rescue_Mode:
	  case Handle_Mode:
			gimbal->gimbal_ctrl_mode.gimbal_mode = 3;
		  break;
		case LEG_TEST_Mode:
		case Lob_Mode:		
			gimbal->gimbal_ctrl_mode.gimbal_mode = 2;
		  break;
		case Imu_Mode:
		case Cycle_Mode:
		case Vary_Cycle_Mode:
			gimbal->gimbal_ctrl_mode.gimbal_mode = 1;
		  break;			
		default:
			break;
	}
}

/*云台pitch轴陀螺仪角度限位*/
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

/*云台pitch轴机械角度限位*/
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

	
/*云台信息更新*/
static float k=0.f;
static float add ;//零漂补偿
void Gimbal_Extern_Updata(gimbal_t *gimbal)
{
	static float mec = 0;
	static float yaw = 0;//1目前，0等待
	add += k*0.001;//零漂补偿
	if(Board_HeartBeat.status == DEV_ONLINE)
	{
		gimbal->base_info.yaw_imu_angle = Board_Rx_Info.yaw_imu + add;
		gimbal->base_info.yaw_imu_speed = Board_Rx_Info.yaw_v;
		gimbal->base_info.pitch_motor_angle = Board_Rx_Info.pitch_mec;
		gimbal->base_info.pitch_motor_speed = Board_Rx_Info.pitch_v;
		if(gimbal->gimbal_ctrl_mode.gimbal_mode == 2 && mec == 0)
		{
			gimbal->base_info.pitch_mec_angle_target = Board_Rx_Info.pitch_mec;     //切模式更新就好了  
			mec = 1;
			yaw = 0;
		}
		if(gimbal->gimbal_ctrl_mode.gimbal_mode == 1 && yaw == 0)
		{	
			gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu;
			mec = 0;
			yaw = 1;
		}
	}
	else
	{
		mec = 0;
		yaw = 1;
	}
	float angle=gimbal->base_info.yaw_imu_angle;float max=360;
	while (my_abs(angle) > (max / 2))//可能卡死
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	gimbal->base_info.yaw_imu_angle = angle;
  gimbal->base_info.yaw_imu_angle = half_cycle(gimbal->base_info.yaw_imu_angle, 360.f);
	
	/*yaw轴电机角度更新*/
	gimbal->base_info.yaw_motor_angle = -(YAW_MOTOR_ANGLE_MIDDLE - (float)gimbal->gimbal_y->rx_info->motor_angle);
	gimbal->base_info.yaw_motor_angle = half_cycle(gimbal->base_info.yaw_motor_angle, 2*PI);
	gimbal->base_info.yaw_motor_speed = (float)gimbal->gimbal_y->rx_info->speed;
	
//	gimbal->gimbal_reset_state = Board_Rx_Info.gimbal_state;
//	gimbal->gimbal_ctrl_mode.gimbal_mode = Board_Rx_Info.gimbal_mode;

	/*360度标准化角度*/
	gimbal->base_info.yaw_mec_360_angle=gimbal->base_info.yaw_motor_angle/(2*PI)*360.f;
}

/*云台yaw轴角度检查*/
void Gimbal_Yaw_Angle_Check(gimbal_t *gimbal)
{
	float angle = gimbal->base_info.yaw_imu_angle_target;//-180°~180°
	if(angle>=10000)//防卡死
	{
		angle =0;
	}
	while (my_abs(angle) > 180)//有可能卡死
	{
		angle -= 360 * sgn(angle);
	}
	gimbal->base_info.yaw_imu_angle_target = angle;
}

/*云台yaw轴PID计算*/
void Gimbal_Yaw_Pid_Cal(gimbal_t *gimbal)
{
	float gyro_meas_in,gyro_meas_out,gyro_target,mec_meas_in,mec_meas_out,mec_target;

	switch (gimbal->yaw_pid_mode)
	{
	case GYRO_PID:
		gyro_meas_out = gimbal->base_info.yaw_imu_angle;				//外环
		gyro_meas_in = gimbal->base_info.yaw_imu_speed	;			  //内环
		gyro_target = gimbal->base_info.yaw_imu_angle_target;  //目标值
		
		gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc( gimbal->gimbal_y->ctrl->angle_ctrl_outer,gimbal->gimbal_y->ctrl->angle_ctrl_inner,gyro_target,gyro_meas_out,gyro_meas_in,-1,3);
		break;

	case MEC_PID:
		mec_meas_out = (float)gimbal->base_info.yaw_motor_angle / PI * 180.f;   //外环 转为角度
		mec_meas_in = gimbal->base_info.yaw_imu_speed;			            	   //内环 
		mec_target = gimbal->base_info.yaw_mec_angle_target / PI * 180.f;
		
		gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc( gimbal->gimbal_y->ctrl->position_out,gimbal->gimbal_y->ctrl->position_inn,mec_target,mec_meas_out,mec_meas_in,-1,3);
		break;
	
	case SPEED_PID:
		break;
	default:
		break;
	}
}

/*云台初始化*/
void Gimbal_init(gimbal_t *gimbal)
{
	 gimbal->base_info.init_time_max_count++;
	gimbal->offset_info->lob_yaw_mec_offset=0;
	gimbal->lob_info.lob_init_angle_flag=0;
	//云台就近归位		
	if (my_abs(gimbal->base_info.yaw_motor_angle) > PI/2.f)
	{
		gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI;
	}
	else
	{
		gimbal->base_info.yaw_mec_angle_target = 0;
	}
		
	gimbal->yaw_pid_mode = MEC_PID;
	if(my_abs(gimbal->base_info.yaw_motor_speed) <= 20 && my_abs(my_abs(gimbal->base_info.yaw_motor_angle) 
		- my_abs(gimbal->base_info.yaw_mec_angle_target)) <= 0.01f  && (my_abs(gimbal->base_info.pitch_motor_angle)
  	- my_abs(gimbal->base_info.pitch_mec_angle_target)) <= 0.01f && my_abs(gimbal->base_info.pitch_motor_speed) <= 50 )
	{
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;//car初始化里有，可以试试能不能删去
		gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu;
		gimbal->gimbal_reset_state = DEV_RESET_OK;
//		car.car_move_mode=gyro_CAR;
		gimbal->base_info.init_time=0;
     gimbal->base_info.init_time_max_count=0;
	}
	
	if(gimbal->base_info.init_time_max_count>=gimbal->base_info.init_time_max)
	{
		gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;//car初始化里有，可以试试能不能删去
		gimbal->gimbal_reset_state = DEV_RESET_OK;
//		car.car_move_mode=gyro_CAR;
		gimbal->base_info.init_time=0;
		gimbal->base_info.init_time_max_count=0;
	}
}

/*云台自救模式*/
void Gimbal_Save_Update(gimbal_t *gimbal)
{
	//云台就近归位		
	if (my_abs(gimbal->base_info.yaw_motor_angle) > PI/2.f)
	{
		gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI;
	}
	else
	{
		gimbal->base_info.yaw_mec_angle_target = 0;
	}
	  gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
	  gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu;
}

/*云台陀螺仪模式*/
void Gimbal_Gyro_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
  if((Balance.Vision.Auto_Catch_Flag == true || Balance.Vision.Auto_Outpost_Flag == true || Balance.Vision.Auto_Base_Flag == true)
		&& Board_Rx_Info.vision_state == 1 && Board_Rx_Info.is_find_Target == 1)
	{
    gimbal->base_info.yaw_imu_angle_target = Board_Rx_Info.vision_yaw_tar;
		gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.vision_pitch_tar;
	}
	else
	{
		if(ctrl_mode != KEY_CTRL)
		{
			gimbal->base_info.yaw_imu_angle_target -= rc_sensor.info->ch0*0.001f*0.2;
			gimbal->base_info.pitch_imu_angle_target += rc_sensor.info->ch1*0.001f*0.1;
		}
		else
		{
			gimbal->base_info.yaw_imu_angle_target -= rc_sensor.info->mouse_x * 0.001f;
			gimbal->base_info.pitch_imu_angle_target += rc_sensor.info->mouse_y*0.001f;
		}
	}
	
	gimbal->base_info.yaw_imu_angle_target = half_cycle(gimbal->base_info.yaw_imu_angle_target,360.f);//////////////////////////////
//	gimbal->base_info.pitch_mec_angle_target = gimbal->base_info.pitch_motor_angle;
		
//	if (my_abs(gimbal->base_info.yaw_motor_angle) > PI/2.f)//给lob模式更新初始yaw************************************
//	{
//		gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI;//动态调整目标正负，免去pid计算时的半圈处理
//	}
//	else
//	{
//		gimbal->base_info.yaw_mec_angle_target = 0 ;
//	}
	gimbal->base_info.yaw_mec_angle_target = gimbal->base_info.yaw_motor_angle;
	gimbal->base_info.pitch_mec_angle_target = Board_Rx_Info.pitch_mec;

	
	if(Balance.Flag->Turn_Flag == 1)//换头
	{
		switch(gimbal->base_info.step)
		{
			case Gimbal_Turn_IDLE:
		    gimbal->base_info.yaw_imu_angle_target += 180.f;
		    gimbal->base_info.yaw_imu_angle_target = half_cycle(gimbal->base_info.yaw_imu_angle_target,360.f);
			  gimbal->base_info.step = Gimbal_Turn_Going;
			  break;
			case Gimbal_Turn_Going:
				gimbal->base_info.turn_time++;
			  if(my_abs(gimbal->base_info.yaw_imu_angle_target - gimbal->base_info.yaw_imu_angle) <= 5.f || gimbal->base_info.turn_time >=gimbal->base_info.turn_time_max)
				{
//					Balance.Flag->Turn_Flag = 0;
					gimbal->base_info.Gimbal_Turn_Finish = 1;
					gimbal->base_info.step = Gimbal_Turn_IDLE;
					gimbal->base_info.turn_time = 0;
				}
			  break;
			default:
				break;
		}
//		if(my_abs(gimbal->base_info.yaw_motor_angle) > PI/2.f)
//		{
//			gimbal->base_info.yaw_mec_angle_target = 0 ;
//			if(my_abs(gimbal->base_info.yaw_motor_angle) <= 0.02f || gimbal->base_info.turn_time >=gimbal->base_info.turn_time_max)
//			{
//				Balance.Flag->Turn_Flag = 0;
//				gimbal->base_info.turn_time = 0;
//			}
//		}
//		else
//		{
//			gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI ;
//      if((PI - my_abs(gimbal->base_info.yaw_motor_angle)) <= 0.02f || gimbal->base_info.turn_time >=gimbal->base_info.turn_time_max)
//			{
//				Balance.Flag->Turn_Flag = 0;
//				gimbal->base_info.turn_time = 0;
//			}		
//		}
	}
	gimbal->base_info.yaw_mec_angle_target = gimbal->base_info.yaw_motor_angle;
	gimbal->lob_info.gyro_lob_yaw_angle_dynamic_tar = gimbal->base_info.yaw_motor_angle;
	gimbal->lob_info.gyro_lob_pitch_angle_dynamic_tar = 0.52f;//Board_Rx_Info.pitch_mec;
}

/*云台吊射模式*/
void Gimbal_Lob_Update(gimbal_t *gimbal,uint8_t ctrl_mode)
{
	
//  if(Balance.Vision.Auto_Base_Flag == 1 && Board_Rx_Info.vision_state == 1)
//	{
//    gimbal->base_info.yaw_mec_angle_target = Board_Rx_Info.vision_yaw_tar / 180.f * PI + gimbal->offset_info->lob_yaw_mec_offset ;
//	}
//	else
//	{
		if(ctrl_mode != KEY_CTRL)
		{
			if(my_abs((float)rc_sensor.info->ch0)>=10)
			{
				gimbal->base_info.yaw_mec_angle_target -= rc_sensor.info->ch0*0.001f*0.0003f;
			}
			if(my_abs((float)rc_sensor.info->ch1)>=10)
			{
				gimbal->base_info.pitch_mec_angle_target += rc_sensor.info->ch1*0.001f*0.2f*0.005f;
			}
		}
		else
		{
			
//			if(rc_sensor.info->Z.status == long_press || rc_sensor.info->Z.status == release_to_press || rc_sensor.info->Z.status == short_press)
//			{
////				gimbal->base_info.yaw_mec_angle_target -= gimbal->offset_info->lob_yaw_mec_offset;
////				gimbal->base_info.pitch_mec_angle_target += gimbal->offset_info->lob_pitch_mec_offset;
//			}
//			else
//			{
//				gimbal->base_info.yaw_mec_angle_target -= rc_sensor.info->mouse_x * 0.000006f;
//				gimbal->base_info.pitch_mec_angle_target += rc_sensor.info->mouse_y*0.000006f;			
////				gimbal->base_info.yaw_mec_angle_target -= gimbal->offset_info->lob_yaw_mec_offset;
////				gimbal->base_info.pitch_mec_angle_target += gimbal->offset_info->lob_pitch_mec_offset;
//			}
			
			if(rc_sensor.info->Z.status == long_press || rc_sensor.info->Z.status == release_to_press || rc_sensor.info->Z.status == short_press)
			{
				
				gimbal->base_info.yaw_mec_angle_target = gimbal->offset_info->lob_yaw_mec_offset + gimbal->lob_info.gyro_lob_yaw_angle_dynamic_tar;
				gimbal->base_info.pitch_mec_angle_target = gimbal->offset_info->lob_pitch_mec_offset + gimbal->lob_info.gyro_lob_pitch_angle_dynamic_tar;
			}
			else
			{
				gimbal->lob_info.gyro_lob_yaw_angle_dynamic_tar -= rc_sensor.info->mouse_x * 0.000006f;
				gimbal->lob_info.gyro_lob_pitch_angle_dynamic_tar += rc_sensor.info->mouse_y*0.000006f;			
				gimbal->base_info.yaw_mec_angle_target = gimbal->offset_info->lob_yaw_mec_offset + gimbal->lob_info.gyro_lob_yaw_angle_dynamic_tar;
				gimbal->base_info.pitch_mec_angle_target = gimbal->offset_info->lob_pitch_mec_offset + gimbal->lob_info.gyro_lob_pitch_angle_dynamic_tar;
			}
			
			
		}

	  gimbal->base_info.yaw_mec_angle_target = half_cycle(gimbal->base_info.yaw_mec_angle_target, 2*PI);
		
	  gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
	  gimbal->base_info.yaw_imu_angle_target = half_cycle(gimbal->base_info.yaw_imu_angle_target,360.f);
	  gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.pitch_imu;
}

//void Gimbal_Board_Update(gimbal_t *gimbal)
//{
//	Board_Tx_Info.pitch_imu_tar = gimbal->base_info.pitch_imu_angle_target;
//	Board_Tx_Info.pitch_mec_tar = gimbal->base_info.pitch_mec_angle_target;
//	Board_Tx_Info.yaw_imu_tar = gimbal->base_info.yaw_imu_angle_target;
//	
//}

void Gimbal_Work(gimbal_t *gimbal)
{
	Gimbal_Status_Update(gimbal);
	Gimbal_Extern_Updata(gimbal);
	#ifndef TEST
  switch(gimbal->gimbal_reset_state)
	{
		case DEV_RESET_NO:
			Gimbal_init(gimbal);
		  gimbal->yaw_pid_mode = MEC_PID;
		  break;
		case DEV_RESET_OK:
			switch(gimbal->gimbal_ctrl_mode.gimbal_mode)
			{	
				case 1:
			    Gimbal_Gyro_Update(gimbal,Balance.ctrl);
		      gimbal->yaw_pid_mode = GYRO_PID;
				break;
				case 2:
					Gimbal_Lob_Update(gimbal,Balance.ctrl);
				  gimbal->yaw_pid_mode = MEC_PID;
				break;
				case 3:
					Gimbal_Save_Update(gimbal);
				  gimbal->yaw_pid_mode = MEC_PID;
				  break;
				default:
		      break;
			}
		  break;		
		default:
		  break;
	}
		Gimbal_Yaw_Angle_Check(gimbal);//Yaw角度检查
	  Gimbal_Pitch_Gyro_Angle_Limit(gimbal);
	  Gimbal_Pitch_Mec_Angle_Limit(gimbal);

//	  Gimbal_Board_Update(gimbal);
		if(RC_ONLINE)//开控
		{
			Gimbal_Yaw_Pid_Cal(gimbal);
			gimbal->gimbal_y->tx_info->torque=gimbal->base_info.output_gimbal_y; 			
		}
		else//关控
		{
			gimbal->base_info.step = Gimbal_Turn_IDLE;
			gimbal->gimbal_reset_state = DEV_RESET_NO;
			gimbal->gimbal_y->tx_info->torque=0;
      gimbal->base_info.pitch_mec_angle_target = 0;
		}
		#else
		gimbal->base_info.yaw_imu_angle_target+=rc_sensor.info->ch0*0.001f*0.3;
		gimbal->base_info.pitch_imu_angle_target+=rc_sensor.info->ch1*0.001f*0.1;
//		gimbal->base_info.pitch_mec_angle_target+=rc_sensor.info->ch1*0.001f*0.5;
		gimbal->base_info.pitch_mec_angle_target = 0.f;
		gimbal->yaw_pid_mode = MEC_PID;
		Gimbal_Yaw_Pid_Cal(gimbal);
		if(RC_ONLINE)
		{
		gimbal->gimbal_y->tx_info->torque=gimbal->base_info.output_gimbal_y; 	
		}
		else
		{
		gimbal->gimbal_y->tx_info->torque=0;
		}
			#endif
}