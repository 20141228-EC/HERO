#include "shoot.h"

static void Shoot_Ext_Work(shoot_out_t* shoot_out);

shoot_out_t shoot_out=
{
	.dail = &Dail_Motor,
	.work = Shoot_Ext_Work,
};

void Shoot_Ext_Init(shoot_out_t* shoot_out)
{
	shoot.cmd.vision_tx_cmd.is_ready_flag = 1;
	shoot.info.rt_rx_info.flag_Info.fire_mode_flag = 0;
}

/*外部拨盘信息更新*/
void Dail_Info_Ext_Update(shoot_out_t* shoot_out)
{
	shoot.info.rt_rx_info.dial_info.angle = shoot_out->dail->rx_info->motor_angle;
	shoot.info.rt_rx_info.dial_info.speed = shoot_out->dail->rx_info->speed;
	shoot.info.rt_rx_info.dial_info.current = shoot_out->dail->rx_info->torque;
}

/*离线检测*/
void Shoot_offline_cheak(shoot_out_t* shoot_out)
{
	if(shoot_out->dail->state->status == DEV_OFFLINE )
	{
		shoot.info.rt_rx_info.flag_Info.is_mtr_offline_flag = 1;
	}
	else
	{
		shoot.info.rt_rx_info.flag_Info.is_mtr_offline_flag = 0;
	}
}

static uint16_t judge_heat,last_judge_heat;
static uint16_t local_heat;
/*热量限制*/
void Shoot_Heat_Limit(shoot_out_t* shoot_out)
{		
//	last_judge_heat = judge_heat;
//	judge_heat = My_Judge.info->shooter_cooling_heat;
//	
//	if(last_judge_heat != judge_heat)
//	{
//		local_heat = judge_heat;
//	}
//	else
//	{
//		local_heat = local_heat;
//	}
	
	
	#ifdef JUDGE_ENABLE
	if(My_Judge.org_info->game_status.game_progress == 4)
	{
		if(My_Judge.info->shooter_cooling_limit - My_Judge.info->shooter_cooling_heat <=105//local_heat <=105         //
			|| My_Judge.org_info->projectile_allowance.projectile_allowance_42mm <= 0)
		{
			shoot_out->base_info.is_heat_allow = 0;
		}
		else
		{
			shoot_out->base_info.is_heat_allow = 1;
		}
		
		if(shoot_out->base_info.is_heat_allow == 1 && Board_Rx_Info.is_fric_speed == 1 && Board_Rx_Info.is_fric_work == 1)
		{
			shoot.info.rt_rx_info.flag_Info.run_limit_flag = 0;
		
		}
		else
		{
			shoot.info.rt_rx_info.flag_Info.run_limit_flag = 1;		
		}
	}
	else
	{
		if(My_Judge.info->shooter_cooling_limit - My_Judge.info->shooter_cooling_heat <=110)
		{
			shoot_out->base_info.is_heat_allow = 0;
		}
		else
		{
			shoot_out->base_info.is_heat_allow = 1;
		}
		
		if(shoot_out->base_info.is_heat_allow == 1 && Board_Rx_Info.is_fric_speed == 1 && Board_Rx_Info.is_fric_work == 1)
		{
			shoot.info.rt_rx_info.flag_Info.run_limit_flag = 0;
		
		}
		else
		{
			shoot.info.rt_rx_info.flag_Info.run_limit_flag = 1;		
		}
	}
	
	#else
//	    shoot_out->base_info.is_heat_allow = 1;
			shoot.info.rt_rx_info.flag_Info.run_limit_flag = 0;
		
	#endif

}

/*拨盘pid计算*/
void Dail_Pid_Cal(shoot_out_t* shoot_out)
{
	if(shoot.cmd.dial_tx_cmd.mode == DIAL_ANGLE)
	{
		shoot_out->dail->ctrl->position_out->target = shoot.cmd.dial_tx_cmd.angle_sum_target;
		shoot_out->dail->ctrl->position_out->measure = shoot_out->dail->rx_info->motor_angle_sum;
		Motor_Set_Angle_Position_DM(&Dail_Motor);
	}
	else if(shoot.cmd.dial_tx_cmd.mode == DIAL_SPEED)
	{
		shoot_out->dail->ctrl->speed_ctrl->target = shoot.cmd.dial_tx_cmd.speed_target;
		shoot_out->dail->ctrl->speed_ctrl->measure = shoot_out->dail->rx_info->speed;
		Motor_Set_Speed_Position_DM(&Dail_Motor);
	}
}

/*发射总控*/
//将sleep=0关联到整车复位完成，sleep=1关联到整车未复位
//将init=0关联到我外部拨盘复位的动作
//firemode=0单发//
//掉线关联offline，程序内死了//
//将热量限制(外部限制条件)关联到limit
//elev外部不跳变，在后状态就是1，前状态就是0//
void Shoot_Ext_Work(shoot_out_t* shoot_out)
{	
	Dail_Info_Ext_Update(shoot_out);
	if(Balance.Flag->Shoot_Flag == 1)                      //sleep开发射置0，光发射1
	{		
		shoot.info.rt_rx_info.flag_Info.is_sleep_flag = 0;
	}
	else
	{
		shoot.info.rt_rx_info.flag_Info.is_sleep_flag = 1;
	}
	
	shoot.info.rt_rx_info.flag_Info.fire_mode_flag = 0;     //单发0
	
//	if(shoot.info.rt_rx_info.flag_Info.is_sleep_flag == 0 && rc_sensor.info->s1 == 1 && rc_sensor.info->s2 == 2 && rc_sensor.info->s2_last == 3)
//	{
//	  shoot.info.rt_rx_info.flag_Info.init_flag = 0;		//拨盘复位，未完善
//	}
	
	Shoot_offline_cheak(shoot_out);                         //离线
	
	Shoot_Heat_Limit(shoot_out);                            //热量限制
	
  if(Balance.Shoot.Single_Shoot_Flag == 1/* && My_Judge.info->shooter_cooling_limit - My_Judge.info->shooter_cooling_heat >=180*/)           //开发射时判断elec电平高低
	{
		shoot.info.rt_rx_info.flag_Info.elec_level_flag = 1;	
	}
	else
	{
		shoot.info.rt_rx_info.flag_Info.elec_level_flag = 0;	
	} 
	
//	static uint8_t last_flag,flag;
//	last_flag = flag;
//	flag = Balance.Shoot.Single_Shoot_Flag;
//	if(last_flag == 0 && flag == 1)
//	{
//		local_heat += 100;
//	}
	
	Shoot_Base_Work(&shoot);
	
	if(shoot.cmd.vision_tx_cmd.is_ready_flag == 1) //给视觉发1
	{
		shoot_out->base_info.is_enable_shoot = 1;
	}
	else
	{
		shoot_out->base_info.is_enable_shoot = 0;
	}

	switch(shoot.cmd.fric_tx_cmd.work_state)                //开摩擦轮指令
	{
		case RUN:
		  shoot_out->fric.is_fric_on = 1;
		  break;
		case STOP:
		  shoot_out->fric.is_fric_on = 0;
		  break;
		default:
			break;
	}
	
	#ifdef SHOOT_HEAT 
	  #ifdef VISION
			if(Balance.Vision.Auto_Catch_Flag == 1 || Balance.Vision.Auto_Catch_Engi_Flag == 1 || Balance.Vision.Auto_Base_Flag == 1)
			{
				if(shoot_out->base_info.is_enable_shoot == 1 && Board_Rx_Info.hit_enable == 1 &&My_Judge.org_info->projectile_allowance.projectile_allowance_42mm >= 1)
				{
					Dail_Pid_Cal(shoot_out);
				}
			}
			//无视觉
		#else
			{
				if(RC_ONLINE && shoot.cmd.dial_tx_cmd.work_state != SLEEP )
				{
					Dail_Pid_Cal(shoot_out);
				}
				else
			  {
				  Dail_Motor.tx_info->torque = 0;
			  }
			}
		#endif
	#else
	  #ifdef VISION
			if(Balance.Vision.Auto_Catch_Flag == 1 || Balance.Vision.Auto_Catch_Engi_Flag == 1 || Balance.Vision.Auto_Base_Flag == 1)
			{
				if(Board_Rx_Info.hit_enable == 1 && shoot.cmd.dial_tx_cmd.work_state != SLEEP)
				{
					Dail_Pid_Cal(shoot_out);
				}
				else
				{
				  Dail_Motor.tx_info->torque = 0;
			  }
			}
			else
			{
			  Dail_Motor.tx_info->torque = 0;
		  }
			//无视觉
		#else
			if(RC_ONLINE && shoot.cmd.dial_tx_cmd.work_state != SLEEP)
			{
				Dail_Pid_Cal(shoot_out);				
			}
			else
			{
				Dail_Motor.tx_info->torque = 0;
			}
		#endif
  #endif
}