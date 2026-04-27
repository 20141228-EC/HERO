#include "control_task.h"
#include "chassis_motor.h"
#include "shoot.h"
#include "LaserRanging_kalman.h"
#include "RP_Log.h"
#include "tim.h"
extern osThreadId_t CtrlTaskHandle;

extern osSemaphoreId_t semTaskObserveToCtrl;
extern osSemaphoreId_t semTaskCtrlToObserve;
//static uint32_t last_shoot_time;
//static uint32_t stuck_count;
uint8_t flaag,lastflaag = 0;
uint8_t r,l;

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		UBaseType_t free_stack = uxTaskGetStackHighWaterMark(CtrlTaskHandle);
		osSemaphoreAcquire(semTaskObserveToCtrl, osWaitForever);

		Command_Update();
		Chassis.status_react(&Chassis);
#ifdef TEST_DAIL			  
		Balance.Flag->Shoot_Flag = 1;
		shoot.info.rt_rx_info.flag_Info.is_sleep_flag = 0;
		
		if(flaag == 1 && lastflaag == 0)
		{
			Balance.Shoot.Single_Shoot_Flag = true;
		}
		else
		{
			Balance.Shoot.Single_Shoot_Flag = false;
		}
		lastflaag = flaag;
		if(flaag == 1)
		{
		RP_LOG_INFO("System started");
			flaag = 0;
		}
#else
#endif
    gimbal.work(&gimbal);
		Chassis.ctrl(&Chassis);
		shoot_out.work(&shoot_out);
		
#ifdef TEST_DAIL			  
		Dail_Pid_Cal(&shoot_out);
#else
#endif
		
		My_Judge_Realtime_Task(&My_Judge);
//Send_Read_Command_L();
//Send_Read_Command_R();
//LaserRange_KF_Update(&LR_Speed_KF,&LaserRanging[LaserRange_L]);
//LaserRange_KF_Update(&LR_Speed_KF,&LaserRanging[LaserRange_R]);
  if(RC_ONLINE || Chassis.damping_delay_cnt < DAMPING_DELAY_MAX_CNT)
	{		
//		Yaw_Motor.tx_info->torque = 0;
		
		Yaw_Motor.single_set_torque(&Yaw_Motor);
#ifdef NO_CHASSIS		
				Chassis.Sd->motor[R_F_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[R_B_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[L_F_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Sd->motor[L_B_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;//正往前
				Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;//正往后
#else
//				Chassis.Sd->motor[R_F_Sd_M]->tx_info->torque = 0;//往前
//				Chassis.Sd->motor[R_B_Sd_M]->tx_info->torque = 0;//往前
//				Chassis.Sd->motor[L_F_Sd_M]->tx_info->torque = 0;//负往前
//				Chassis.Sd->motor[L_B_Sd_M]->tx_info->torque = 0;//负往前
//				Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;//正往前
//				Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;//正往后
#endif		 
		  if (Balance.Flag->Chassis_Online_Flag != true)
		  {
				Chassis.Sd->motor[R_F_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[R_B_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[L_F_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Sd->motor[L_B_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;//正往前
				Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;//正往后
//				Balance.mode = Sleep_Mode;
//		    Balance.Flag->Rescue_Flag = true;
//		    Balance.reset_struct.reset_cnt=0;
//		    Balance.reset_struct.reset_state=Balance_reset_NO;
				
		  }
			
	  Sd_Group.group_set_torque(&Sd_Group); 
		Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
		Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);

//    Dail_Motor.tx_info->torque = 0;
    Dail_Motor.single_set_torque(&Dail_Motor);
		
		CAN3_SEND();
			
		rc_sensor_s_last_update(&rc_sensor);
	}	
	else
	{
		Yaw_Motor.tx_info->torque = 0;		
		Yaw_Motor.single_set_torque(&Yaw_Motor);		
    Sd_Group.group_sleep(&Sd_Group);
		Sd_Group.group_set_torque(&Sd_Group); 
		Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;//正往前
		Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;//正往后		
		Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
		Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
#ifdef TEST_DAIL
//    Dail_Motor.tx_info->torque = 0;
#else
    Dail_Motor.tx_info->torque = 0;
#endif

    Dail_Motor.single_set_torque(&Dail_Motor);
		Board_Tx_Info.is_rc_online = 1;
		CAN3_SEND();
		
	}
	
//	if(Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque >= 4.6f)
//	{
//		r = 1;
//	}
//	if(Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque >= 4.6f)
//	{
//		l = 1;
//	}
	
	if(Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque >= 4.6f || Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque >= 4.6f)
	{
   	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 300);
	}
	else
	{
   	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}
	
	
		osSemaphoreRelease(semTaskCtrlToObserve);
		osDelay(1);
  }
}



//		Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque=-0.1;//往逆时针   
//		Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque=0.1;
//		Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
//		Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
