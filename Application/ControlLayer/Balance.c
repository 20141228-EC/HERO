#include "Balance.h"
void Balance_Init(Balance_t* balance);
static void Balance_Init_Judge (Balance_t* balance);
static void Balance_Status_Update(Balance_t* balance);
static void RC_Move_Mode_Update(Balance_t* balance);
static void KEY_Move_Mode_Update(Balance_t* balance);
void Rescue_Check(void);
Chassis_Command_t Chassis_Command;
uint8_t last_step[4];
Balance_Remote_Ctrl Balance_Rc = 
{
	.sensor = &rc_sensor,
	
	.last_thumbwheel_step = last_step,
};

Balance_t Balance =
{
	.ctrl = RC_CTRL,

	.mode = Sleep_Mode,
	
	.init = Balance_Init,
	
	.rc = &Balance_Rc,
	
  .Chassis_Com = &Chassis_Command,
};

void Balance_Init(Balance_t* balance)
{
	balance->update = Balance_Status_Update;
	balance->command = command;
}

/**
  * @brief  整车状态更新
  * @param  Balance_t* balance
  * @retval None
  */
//static void Balance_Status_Update(Balance_t* balance)
//{
//	Balance_Init_Judge(balance);//初始化完成判断
//	
//	if(rc_sensor.work_state == DEV_OFFLINE)
//	{
//		balance->mode=Sleep_Mode;
//		balance->reset_struct.reset_cnt=0;
//		//RC_Offline_Flag_Clean
//	}
//	else if(balance->mode==Sleep_Mode)//开控但是sleep就初始化
//	{
//		balance->mode=Init_Mode;
//	}
//	else if(balance->mode==Init_Mode&&balance->reset_struct.reset_state==Balance_reset_OK)
//	{
//		Rescue_Check();
//		balance->reset_struct.reset_cnt=0;
//		balance->mode=Imu_Mode;
//	}
//	else
//	{
//		Rescue_Check();
//		if(balance->ctrl != KEY_CTRL)
//		{
//		  RC_Move_Mode_Update(balance);
//		}
//		else
//		{
//			KEY_Move_Mode_Update(balance);
//		}
//	}	
//}
static void Balance_Status_Update(Balance_t* balance)
{
	Balance.last_mode = Balance.mode;
	Balance_Init_Judge(balance);//初始化完成判断
	
	if(rc_sensor.work_state == DEV_OFFLINE)
	{
		balance->mode=Sleep_Mode;
		balance->reset_struct.reset_cnt=0;
		balance->reset_struct.reset_state=Balance_reset_NO;

		//RC_Offline_Flag_Clean
	}
	else if(check_hero_revive(&My_Judge) == 1 
		     || ((Chassis.Posture->info->pitch <= -0.5f || Chassis.Posture->info->pitch >= 0.5f 
		     || (Chassis.Posture->info->roll >= 1.5f && Chassis.Posture->info->roll <= 2.9f)
		     || (Chassis.Posture->info->roll <= -1.5f && Chassis.Posture->info->roll >= -2.9f)//60
	       || ((Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_ <= -60.f || Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_ <= -60.f) && (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0 >= 0.3f || Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 >= 0.3f) && balance->Flag->Knee_Strike_1_Flag == false)
				 || ((Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_ >= 60.f || Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_ >= 60.f) && (Chassis.Leg_Unit[L_Leg]->Link->info->length->l0 >= 0.3f || Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 >= 0.3f) && balance->Flag->Knee_Strike_1_Flag == false))
	       && (balance->mode != Init_Mode && balance->mode != Sleep_Mode && balance->mode != Handle_Mode)))
	{
		balance->mode = Sleep_Mode;
		balance->Flag->Rescue_Flag = true;
		balance->reset_struct.reset_cnt=0;
		balance->reset_struct.reset_state=Balance_reset_NO;
	}
	else if(balance->mode==Sleep_Mode)//开控但是sleep就初始化
	{
		balance->mode = Init_Mode;
	}
	else if(balance->mode==Init_Mode && balance->reset_struct.reset_state == Balance_reset_OK)
	{
		balance->reset_struct.reset_cnt=0;
		balance->mode = Imu_Mode;
		balance->Flag->Rescue_Flag = false;
	    if(fabsf(gimbal.base_info.yaw_motor_angle) >= 0.7f)
	    {
		    balance->Flag->Turn_Flag = true;//起立头回正
	    }
	}
	else
	{
		if(rc_sensor.info->s1 == 2 && rc_sensor.info->s2 == 2)
		{
			balance->ctrl = KEY_CTRL;
		}
		else
		{
			balance->ctrl = RC_CTRL;
		}
		
		if(balance->ctrl != KEY_CTRL)
		{
		  RC_Move_Mode_Update(balance);
		}
		else
		{
			KEY_Move_Mode_Update(balance);
		}
		
		if((fabsf(Chassis.chassis_PID->yaw_cal[L_Leg]->err) >= 0.10f || fabsf(Chassis.chassis_PID->yaw_cal[R_Leg]->err) >= 0.10f)
			&& (fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_) >= 8.f || fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_) >= 8.f)
		  && (fabsf(Chassis.Posture->info->pitch) >= 0.04)
		  && (fabsf(Chassis.Leg_Unit[L_Leg]->force->Tp_target) >= 2.3f || fabsf(Chassis.Leg_Unit[R_Leg]->force->Tp_target) >= 2.3f)
		  && (fabsf(Wheel_Group.motor[L_WHEEL_M]->rx_info->speed) <= 0.3f || fabsf(Wheel_Group.motor[R_WHEEL_M]->rx_info->speed) <= 0.3f)
		  && balance->Flag->Jumping_Flag == false && balance->Flag->Knee_Strike_1_Flag == false)
		{
			Balance.Flag->Remedy_Flag = 1;
//			Balance.mode = Sleep_Mode;
			
		}
		else
		{
			Balance.Flag->Remedy_Flag = 0;
		}
	}	
}

/**
 * @brief 初始化完成判断
 */
static void Balance_Init_Judge(Balance_t* balance)
{
	if(Chassis.reset_struct->reset_state==Chassis_reset_OK && gimbal.gimbal_reset_state == DEV_RESET_OK)
	{
		balance->reset_struct.reset_state=Balance_reset_OK;
		balance->reset_struct.reset_cnt = 0;
	}
	if(balance->mode==Init_Mode)
	{
		balance->reset_struct.reset_cnt++;
	}
	if(balance->reset_struct.reset_cnt>=BALANCE_INIT_CNT_MAX)
	{
		balance->reset_struct.reset_cnt = 0;
		balance->reset_struct.reset_state=Balance_reset_OK;//手动自救？
//		balance->mode = Handle_Mode;
	}
}


/**
  * @brief  底盘自救判断
  * @param  None
  * @retval None
  */
uint8_t t1_rescue_cnt;
void Rescue_Check(void)
{
	static bool last_Rescue_Flag;
	float R_phi0 = Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_ ;
	float L_phi0 = Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_ ;
	float thetab	 = Chassis.Posture->info->pitch;
	float roll	 = Chassis.Posture->info->roll;
	/*自救条件判断*/
	if(fabsf(thetab)>= angle2rad(90.f)||fabsf(roll)>=angle2rad(25.f))//机体太斜
	{
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=true;
		t1_rescue_cnt++;
	}
	else if(fabsf(R_phi0)>=80||fabsf(L_phi0)>=80)//机体角度还行但是腿的姿态很离谱，可以自救
	{
		
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=false;
		t1_rescue_cnt++;
	}
	else//机体正常控，不自救
	{
		Balance.Flag->Rescue_Flag=false;
		Balance.Flag->Unable_Rescue_Flag=false;
	}
	/*自救标志位上升沿*/
	if(last_Rescue_Flag==false&&Balance.Flag->Rescue_Flag==true)
	{
		Balance.Flag->Rescue_Trigger=true;
	}
	else
	{
		Balance.Flag->Rescue_Trigger=false;
	}
	last_Rescue_Flag=Balance.Flag->Rescue_Flag;
	
}

/**
 * @brief 遥控模式整车移动模式更新  183~344  346~422
 */
static void RC_Move_Mode_Update(Balance_t* balance)
{
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
		
	if(balance->mode == Imu_Mode || balance->mode == Mec_Mode)//动作命令识别
	{
		if(balance->command[JUMP].cmd_value == true && balance->Flag->Jumping_Flag != true)
		{
			balance->Flag->Jumping_Flag = true;
		}
//		else if(balance->command[JUMP].cmd_value == true && balance->Flag->Jumping_Flag == true)
//		{
//			Chassis.jump_info->jump_step = J_LANDING;
//			Chassis.jump_info->LANDING_tick = Chassis.jump_info->Max_LANDING_tick;
//		}
		
		if(balance->command[KNEE_STRIKE_1].cmd_value == true && balance->Flag->Knee_Strike_1_Flag != true)
		{
			balance->Flag->Knee_Strike_1_Flag = true;
		}
		else if(balance->command[KNEE_STRIKE_1].cmd_value == true && balance->Flag->Knee_Strike_1_Flag == true)
		{
			Chassis.knee_strike_info->step1 = Knee_RETRACT;
			Chassis.knee_strike_info->RETRACT_tick = Chassis.knee_strike_info->Max_RETRACT_tick;
		}
		
		if(balance->command[KNEE_STRIKE_2].cmd_value==true)
		{
			balance->Flag->Knee_Strike_2_Flag = true;
		}
		if(balance->command[FLY].cmd_value==true)
		{
			balance->Flag->Fly_Flag = true;
		}
		if(balance->command[TURN].cmd_value==true)
		{
			balance->Flag->Turn_Flag = true;
		}
		
	}
	else
	{
		balance->command[JUMP].cmd_value = false;
		balance->command[KNEE_STRIKE_1].cmd_value = false;
		balance->command[KNEE_STRIKE_2].cmd_value = false;
		balance->command[FLY].cmd_value = false;
		balance->command[TURN].cmd_value = false;
	}

{//	if(rc_info->s1 == 1 &&rc_info->s2 == 3)//小陀螺变速小陀螺
//	{
////		if(rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1)
////		{
////			if(balance->mode != Cycle_Mode)
////			{
////				balance->mode = Cycle_Mode;
////			}
////			else
////			{
////				balance->mode = Imu_Mode;
////			}
////		}
////		if(rc_info->thumbwheel.step_change[RC_MD_TO_DO] == 1)
////		{
////			if(balance->mode != Vary_Cycle_Mode)
////			{
////				balance->mode = Vary_Cycle_Mode;
////			}
////			else
////			{
////				balance->mode = Imu_Mode;
////			}
////		}
//	}
//	
//  if(rc_info->s1 == 1 &&rc_info->s2 == 2)//（机械）测试模式可以在这里，键盘模式
//	{
//		if(rc_info->thumbwheel.step[RC_TB_UP] != rc_info->thumbwheel.last_step[RC_TB_UP])
//		{
//			if(balance->mode != LEG_TEST_Mode)
//			{
//				balance->mode = LEG_TEST_Mode;
//			}
//			else
//			{
//				balance->mode = Imu_Mode;
//			}
//		}
//		if(rc_info->thumbwheel.step_change[RC_MD_TO_DO] == 1 || rc_info->thumbwheel.step[RC_TB_DN] != rc_info->thumbwheel.last_step[RC_TB_DN])
//		{
//			if(balance->ctrl != KEY_CTRL)
//			{
//				balance->ctrl = KEY_CTRL;
//			}
//			else
//			{
//				balance->ctrl = RC_CTRL;
//			}
//		}
//	}
//	
//  if((balance->mode == Imu_Mode || balance->mode == Mec_Mode ////////////////////////////开发射
//	 || balance->mode == Cycle_Mode || balance->mode == Vary_Cycle_Mode || balance->mode == LEG_TEST_Mode) && rc_info->s1 == 3)
//  {
//	  if(rc_info->thumbwheel.step[RC_TB_UP] != rc_info->thumbwheel.last_step[RC_TB_UP])
//	 	{
//	 	  if(balance->Flag->Shoot_Flag == 0)
//		  {
//			  balance->Flag->Shoot_Flag = 1;
//		  }
//		  else
//		  {
//		 	  balance->Flag->Shoot_Flag = 0;
//		  }
//	  }  
//  }
//{//
////	if(balance->mode == Imu_Mode || balance->mode == Mec_Mode)//动作,完成标志位0
////	{
////	  if(rc_info->s1 == 2 &&rc_info->s2 == 1)
////	  {
////	  	if(rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1)
////	  	{
////	  		balance->Flag->Knee_Strike_1_Flag = 1;
////	  	}
////	  	if(rc_info->thumbwheel.step_change[RC_MD_TO_DO] == 1)
////	  	{
////	  		balance->Flag->Jumping_Flag = 1;
////	  	}
////	  }
////		
////		if(rc_info->s1 == 2 &&rc_info->s2 == 2)
////	  {
////	  	if(rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1)
////	  	{
////	  		balance->Flag->Fly_Flag = 1;
////	  	}
////	  	if(rc_info->thumbwheel.step_change[RC_MD_TO_DO] == 1)
////	  	{
////	  		balance->Flag->Op_Fly_Flag = 1;
////	  	}
////	  }

////		if(rc_info->s1 == 2 &&rc_info->s2 == 3)
////	  {
////	  	if(rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1)
////	  	{
////	  		balance->Flag->Turn_Flag = 1;
////	  	}
////	  }
////	}
////	
//}
//	if(balance->Flag->Shoot_Flag == 1)//单，连，自瞄
//	{
//		if(rc_info->s1 == 1 && rc_info->s2_last == 3 && rc_info->s2 == 1)
//		{
//			balance->Shoot.Single_Shoot_Flag = 1;
//		}
//		else
//		{
//			balance->Shoot.Single_Shoot_Flag = 0;
//		}
//		
////		if(rc_info->s1 == 3 && rc_info->s2 == 1)
////		{
////			balance->Shoot.Keep_Shoot_Flag = 1;
////		}
////		else
////		{
////			balance->Shoot.Keep_Shoot_Flag = 0;
////		}
//		
//		if(rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)
//		{
//			balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
//		}
//	}
	}
	static uint8_t wheel_up,last_wheel_up,wheel_dn,last_wheel_dn = 0;
	last_wheel_up = wheel_up;
	wheel_up = rc_info->thumbwheel.step[RC_TB_UP];
	last_wheel_dn = wheel_dn;
	wheel_dn = rc_info->thumbwheel.step[RC_TB_DN];
	
	static uint32_t cnt;
	
	switch (balance->mode)
	{
		case Imu_Mode:
			if(rc_info->s1 == 1 && rc_info->s2 == 2 && last_wheel_up != wheel_up)
			{
				balance->mode = Lob_Mode;				
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 2 && last_wheel_dn != wheel_dn)
			{
				balance->Flag->Middle_Flag =! balance->Flag->Middle_Flag;			
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_up != wheel_up)
			{
				balance->mode = Cycle_Mode;
				balance->Flag->Cycle_Flag = true;
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_dn != wheel_dn)
			{
				balance->mode = Vary_Cycle_Mode;
			}
			if(rc_info->s1 == 3 && last_wheel_up != wheel_up)
			{
				balance->Flag->Shoot_Flag =! balance->Flag->Shoot_Flag;
			}	
			
			if(/*balance->Flag->Shoot_Flag == 1 && */rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)//开关自瞄
			{
				balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
			}
//			if(balance->Flag->Shoot_Flag == 0)
//			{
//				balance->Vision.Auto_Catch_Flag = 0;
//			}
			if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 0)//无自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt=0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 != 2 )//自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}	
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 == 2)//给视觉打
			{
				if(Board_Rx_Info.hit_enable == 1 && cnt >= 600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
			
		break;
		case Lob_Mode:	
		case Mec_Mode:
		case LEG_TEST_Mode:
			if(rc_info->s1 == 1 && rc_info->s2 == 2 && last_wheel_up != wheel_up)
			{
				balance->mode = Imu_Mode;				
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 2 && last_wheel_dn != wheel_dn)
			{
				balance->Flag->Middle_Flag =! balance->Flag->Middle_Flag;			
			}
			if(rc_info->s1 == 3 && last_wheel_up != wheel_up)
			{
				balance->Flag->Shoot_Flag =! balance->Flag->Shoot_Flag;
			}	
//			if(balance->Flag->Shoot_Flag == 1)//单，连，自瞄
//			{
//				if(rc_info->s1 == 1 && rc_info->s2 == 1)
//				{
//					balance->Shoot.Single_Shoot_Flag = 1;
//				}
//				else
//				{
//					balance->Shoot.Single_Shoot_Flag = 0;
//				}
//				if(rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)
//				{
//					balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
//				}
//			}
			if(balance->Flag->Shoot_Flag == 1 && rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)//开关自瞄
			{
				balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
			}
			if(balance->Flag->Shoot_Flag == 0)
			{
				balance->Vision.Auto_Catch_Flag = 0;
			}
			
			if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 0)//无自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt >= 600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 != 2)//自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt >= 600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}	
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 == 2)//给视觉打
			{
				if(Board_Rx_Info.hit_enable == 1 && cnt >= 300)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 300)
					{
						cnt = 300;
					}
				}
			}

		break;
			
		case Cycle_Mode:
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_up != wheel_up)
			{
				balance->mode = Imu_Mode;
				balance->Flag->Cycle_Flag = false;
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_dn != wheel_dn)
			{
				balance->mode = Vary_Cycle_Mode;
			}
			if(rc_info->s1 == 3 && last_wheel_up != wheel_up)
			{
				balance->Flag->Shoot_Flag =! balance->Flag->Shoot_Flag;
			}		
			if(balance->Flag->Shoot_Flag == 1 && rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)//开关自瞄
			{
				balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
			}
			if(balance->Flag->Shoot_Flag == 0)
			{
				balance->Vision.Auto_Catch_Flag = 0;
			}
			if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 0)//无自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt=0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 != 2 )//自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}	
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 == 2)//给视觉打
			{
				if(Board_Rx_Info.hit_enable == 1 && cnt >= 600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
		break;
			
		case Vary_Cycle_Mode:
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_dn != wheel_dn)
			{
				balance->mode = Imu_Mode;
			}
			if(rc_info->s1 == 1 && rc_info->s2 == 3 && last_wheel_up != wheel_up)
			{
				balance->mode = Cycle_Mode;
			}
			if(rc_info->s1 == 3 && last_wheel_up != wheel_up)
			{
				balance->Flag->Shoot_Flag =! balance->Flag->Shoot_Flag;
			}	
			if(balance->Flag->Shoot_Flag == 1 && rc_info->s1 == 3 && rc_info->s2_last == 3 && rc_info->s2 == 2)//开关自瞄
			{
				balance->Vision.Auto_Catch_Flag = !balance->Vision.Auto_Catch_Flag;			
			}
			if(balance->Flag->Shoot_Flag == 0)
			{
				balance->Vision.Auto_Catch_Flag = 0;
			}
			if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 0)//无自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt=0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 != 2 )//自瞄手打
			{
				if(rc_info->s1 == 1 && rc_info->s2 == 1 && cnt>=600)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 600)
					{
						cnt = 600;
					}
				}
			}	
			else if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1 && rc_info->s1 == 2)//给视觉打
			{
				if(Board_Rx_Info.hit_enable == 1 && cnt >= 500)
				{
					balance->Shoot.Single_Shoot_Flag = 1;
					cnt = 0;
				}
				else
				{
					balance->Shoot.Single_Shoot_Flag = 0;
					cnt++;
					if(cnt >= 500)
					{
						cnt = 500;
					}
				}
			}
		break;
			
		default:
			break;
	}
		
}
/**
 * @brief 键鼠模式整车移动模式更新
 */
static void KEY_Move_Mode_Update(Balance_t* balance)
{
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
	static uint16_t cnt;
	
	if(balance->mode == Imu_Mode || balance->mode == Mec_Mode)//动作命令识别
	{
//		if(balance->command[JUMP].cmd_value==true)//v
//		{
//			balance->Flag->Jumping_Flag = true;
//		}
		if(balance->command[KNEE_STRIKE_1].cmd_value==true &&  balance->Flag->Knee_Strike_1_Flag != true && my_abs(gimbal.base_info.yaw_motor_angle) <= 0.7f)//x
		{
			balance->Flag->Knee_Strike_1_Flag = true;
		}
		else if(balance->command[KNEE_STRIKE_1].cmd_value==true &&  balance->Flag->Knee_Strike_1_Flag != true && my_abs(gimbal.base_info.yaw_motor_angle) >= 0.7f)
		{
			balance->Flag->Return_Flag = true;
		}
		else if(balance->command[KNEE_STRIKE_1].cmd_value == true && balance->Flag->Knee_Strike_1_Flag == true)
		{
			Chassis.knee_strike_info->step1 = Knee_RETRACT;
			Chassis.knee_strike_info->RETRACT_tick = Chassis.knee_strike_info->Max_RETRACT_tick;
		}
//		if(balance->command[KNEE_STRIKE_2].cmd_value==true)
//		{
//			balance->Flag->Knee_Strike_2_Flag = true;
//		}
//		if(balance->command[FLY].cmd_value==true)
//		{
//			balance->Flag->Fly_Flag = true;
//		}
		if(balance->command[TURN].cmd_value==true)//r
		{
			balance->Flag->Turn_Flag = true;
		}
	}
	
		
	if(rc_info->Shift.status == release_to_press)//小陀螺
	{
			if(balance->mode != Cycle_Mode)
			{
				balance->mode = Cycle_Mode;
				balance->Flag->Cycle_Flag = true;
			}
			else
			{
				balance->mode = Imu_Mode;
				balance->Flag->Cycle_Flag = false;
			}
	}
//  if(rc_info->C.status == release_to_press && rc_info->Shift.status == release_to_press)//变速
//	{
//			if(balance->mode != Vary_Cycle_Mode)
//			{
//				balance->mode = Vary_Cycle_Mode;
//				balance->Flag->Cycle_Flag = true;
//			}
//			else
//			{
//				balance->mode = Imu_Mode;
//				balance->Flag->Cycle_Flag = false;
//			}
//	}	
	
//  if(rc_info->Z.status == release_to_press)//机械
//	{
//		if(balance->mode != Lob_Mode)
//		{
//			balance->mode = Lob_Mode;
//		}
//		else
//		{
//			balance->mode = Imu_Mode;
//		}
//	}
  check_z_key_5times();

//		balance->mode = Handle_Mode;
//		Chassis.chassis_PID->phi0_cal[L_Leg]->out_max = 20.f;
//		Chassis.chassis_PID->phi0_cal[R_Leg]->out_max = 20.f;

  if(rc_info->Ctrl.status == release_to_press && Balance.mode == Handle_Mode)
	{
		Balance.mode = Sleep_Mode;
		balance->Flag->Rescue_Flag = true;
		balance->reset_struct.reset_cnt=0;
		balance->reset_struct.reset_state=Balance_reset_NO;
	}
	else if(rc_info->Ctrl.status == release_to_press && balance->Flag->Knee_Strike_1_Flag == true)
	{
		Chassis.knee_strike_info->step1 = Knee_RETRACT;
		Chassis.knee_strike_info->RETRACT_tick = Chassis.knee_strike_info->Max_RETRACT_tick;
	}
	else if(rc_info->Ctrl.status == release_to_press)
	{
		Balance.Flag->Cycle_Flag = false;
		Balance.Flag->Rescue_Flag = false;
		Balance.Flag->Return_Flag = true;
		balance->Flag->Middle_Flag = false;	
	}
	

	if((balance->mode == Imu_Mode || balance->mode == Mec_Mode || balance->mode == Lob_Mode////////////////////////////开发射
	  || balance->mode == Cycle_Mode || balance->mode == Vary_Cycle_Mode) && balance->Flag->Shoot_Flag == 0
	  && rc_info->mouse_btn_l.status == release_to_press)
	{
		balance->Flag->Shoot_Flag = 1;
	}
	
  if((balance->mode == Imu_Mode || balance->mode == Mec_Mode || balance->mode == Lob_Mode////////////////////////////开发射
	 || balance->mode == Cycle_Mode || balance->mode == Vary_Cycle_Mode) && rc_info->B.status == release_to_press
	 && balance->Flag->Shoot_Flag == true)
  {  	
		balance->Flag->Shoot_Flag = false;
  }
 
//	if(balance->Flag->Shoot_Flag == 0)
//	{
//		balance->Vision.Auto_Catch_Flag = 0;
//	}
//  if(rc_info->Z.status == release_to_press && rc_info->Shift.status == long_press)//吊射
//	{
//		if(balance->mode != Mec_Mode)
//		{
//		  balance->mode = Mec_Mode;
//		}
//		else
//		{
//			balance->mode = Imu_Mode;
//		}
//	}
	
		if(rc_info->mouse_btn_r.status == long_press)//开关自瞄
		{
			balance->Vision.Auto_Catch_Flag = 1;			
		}
		else
		{
			balance->Vision.Auto_Catch_Flag = 0;
		}
	
	if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 0)//无自瞄手打
	{
		if(rc_info->mouse_btn_l.status == release_to_press && cnt >= 600)
		{
			balance->Shoot.Single_Shoot_Flag = 1;
			cnt = 0;
		}
		else
		{
			balance->Shoot.Single_Shoot_Flag = 0;
			cnt++;
			if(cnt >= 600)
			{
				cnt = 600;
			}
		}
	}
		
	if(balance->Flag->Shoot_Flag == 1 && balance->Vision.Auto_Catch_Flag == 1)
	{
		if(rc_info->mouse_btn_l.status == release_to_press || rc_info->mouse_btn_l.status == short_press || rc_info->mouse_btn_l.status == long_press)
		{
		  if(Board_Rx_Info.hit_enable == 1 && cnt >= 600)
			{
				balance->Shoot.Single_Shoot_Flag = 1;
				cnt = 0;
			}
			else
			{
				balance->Shoot.Single_Shoot_Flag = 0;
				cnt++;
			  if(cnt >= 600)
			  {
			  	cnt = 600;
			  }
			}
		}
		else
		{
			balance->Shoot.Single_Shoot_Flag = 0;
			cnt++;
			if(cnt >= 600)
			{
				cnt = 600;
			}
		}
	}
	
	if((balance->mode == Imu_Mode || balance->mode == Lob_Mode) 
		 && rc_info->F.status == release_to_press)
	{
		balance->Flag->Middle_Flag =! balance->Flag->Middle_Flag;	
	}
			
  if(rc_info->Z.status == press_to_release && rc_info->X.status == press_to_release && rc_info->C.status == press_to_release)//软复位
	{
		balance->mode = Sleep_Mode;
		Chassis.reset_struct->reset_state = Chassis_reset_NO;
		Chassis.reset_struct->reset_cnt = 0;
		gimbal.gimbal_reset_state = DEV_RESET_NO;
		gimbal.base_info.init_time = 0;
		balance->reset_struct.reset_state=Balance_reset_NO;
		balance->reset_struct.reset_cnt = 0;
		HAL_Delay(500);
		__set_FAULTMASK(1); // 屏蔽中断
		HAL_NVIC_SystemReset();
	}
}

static uint8_t key_count = 0;          // 连续按Z键的计数
static uint32_t last_z_press_time = 0; // 上一次按Z键的时间（ms）
/**
 * @brief 检测Z键是否按下
 * @return true：Z键按下，false：未按下
 */
bool is_z_key_pressed(void)
{
	if(rc_sensor.info->Z.status == release_to_press)
	{
		return true;
	}
	else
	{
    return false;
	}
}

/**
 * @brief Z键连续按5次检测逻辑（需周期性调用，如10ms/次）
 */
void check_z_key_5times(void)
{
    uint32_t current_time = HAL_GetTick();
    static bool last_z_state = false;  // 上一次Z键状态（用于检测上升沿）

    // 1. 检测Z键上升沿（只在按下瞬间处理，避免长按重复计数）
    bool current_z_state = is_z_key_pressed();
    if (current_z_state && !last_z_state)
    {
        // 2. 判断与上一次按Z键的间隔是否≤1000ms
        if ((current_time - last_z_press_time) <= 1000 || key_count == 0)
        {
            key_count++;  // 间隔符合，计数+1
            last_z_press_time = current_time;  // 更新上次按键时间

            // 3. 计数达到5次，触发flag1
            if (key_count >= 5)
            {
                Balance.mode = Handle_Mode;          // 触发标志位
                key_count = 0;         // 重置计数，避免重复触发
                last_z_press_time = 0; // 重置时间
            }
        }
        else
        {
            // 间隔超过1s，重置计数
            key_count = 1;            // 本次按下算第1次
            last_z_press_time = current_time;
        }
    }
    // 4. 超过1s未按Z键，自动重置计数（避免计数残留）
    else if (!current_z_state && (current_time - last_z_press_time) > 1000)
    {
        key_count = 0;
        last_z_press_time = 0;
    }
		
    // 更新上一次按键状态（用于下一次上升沿检测）
    last_z_state = current_z_state;

}