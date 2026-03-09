#include "control_task.h"
#include "chassis_motor.h"
extern osSemaphoreId_t semTaskObserveToCtrl;
extern osSemaphoreId_t semTaskCtrlToObserve;
static uint32_t last_shoot_time;
static uint32_t stuck_count;

//float l_tp,r_tp,l_tw,r_tw;
void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		osSemaphoreAcquire(semTaskObserveToCtrl, osWaitForever);
		
		Command_Update();
		Chassis.status_react(&Chassis);
			  
    gimbal.work(&gimbal);
		Chassis.ctrl(&Chassis);
		shoot_out.work(&shoot_out);
		
		My_Judge_Realtime_Task(&My_Judge);
		
  if(RC_ONLINE || Chassis.damping_delay_cnt < DAMPING_DELAY_MAX_CNT)
	{		
		Yaw_Motor.tx_info->torque = 0;
		
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		
				Chassis.Sd->motor[R_F_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[R_B_Sd_M]->tx_info->torque = 0;//往前
				Chassis.Sd->motor[L_F_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Sd->motor[L_B_Sd_M]->tx_info->torque = 0;//负往前
				Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;//正往前
				Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;//正往后
			
	  Sd_Group.group_set_torque(&Sd_Group); 
		Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
		Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);

//		l_tp = Chassis.Leg_Unit[L_Leg]->force->Tp_target;
//		r_tp = Chassis.Leg_Unit[R_Leg]->force->Tp_target;
//		l_tw = Chassis.Leg_Unit[L_Leg]->force->Tw_target;
//		r_tw = Chassis.Leg_Unit[R_Leg]->force->Tw_target;

{

//if(RC_ONLINE)
//{
//    static float tt=0;
//		tt--;
//		if(tt<=0)
//		{tt=0;}
//		
//    if (HAL_GetTick() - last_shoot_time >= 4000) {
//       Dail_Motor.ctrl->position_out->target+=-2.7f;
//        last_shoot_time = HAL_GetTick();}	
//				
//				if(fabs(Dail_Motor.ctrl->position_inn->measure)<=0.4 && fabs(Dail_Motor.ctrl->position_inn->out)>=20)
//				{
//					stuck_count++;
//						if(stuck_count>=200 && tt==0)
//						{
//							Dail_Motor.ctrl->position_out->target+=2.7f;
//							tt=4000;
//						}
//				}
//				else
//				{
//					stuck_count=0;
//					
//				}
//				Motor_Set_Angle_Position_DM(&Dail_Motor);


//static uint8_t x=0;
//static float speed = 0;
////if(x==0)
////	speed = -4;
////else
////speed=0;
////				if(fabs(Dail_Motor.ctrl->speed_ctrl->measure)<=0.4 && fabs(Dail_Motor.ctrl->speed_ctrl->out)>=20)
////				{
////					stuck_count++;
////						if(stuck_count>=200)
////						{
////							x=1;
////							
////						}
////				}
////		Motor_Set_Speed_Position_DM(&Dail_Motor,speed);

////    Dail_Motor.single_set_torque(&Dail_Motor);

////if(x==0)
////	speed = 6;
////else
////speed=0;
////				if(fabs(Dail_Motor.ctrl->speed_ctrl->measure)<=0.3 && fabs(Dail_Motor.ctrl->speed_ctrl->out)>=18)
////				{
////					stuck_count++;
////						if(stuck_count>=200)
////						{
////							x=1;
////							
////						}
////				}
////		Motor_Set_Speed_Position_DM(&Dail_Motor,speed);

////    Dail_Motor.single_set_torque(&Dail_Motor);
////记得积分清零

//}		
//else
//	{Dail_Motor.tx_info->torque = 0;}
}
//    Dail_Motor.tx_info->torque = 0;
    Dail_Motor.single_set_torque(&Dail_Motor);
		
		CAN3_SEND();
		Cap_Tx_Data_Update(&Cap_Tx_Info);
		
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
    Dail_Motor.tx_info->torque = 0;
    Dail_Motor.single_set_torque(&Dail_Motor);
		Board_Tx_Info.is_rc_online = 1;
		CAN3_SEND();

	}
		osSemaphoreRelease(semTaskCtrlToObserve);
		osDelay(1);
  }
}



//		Chassis.Wheel->motor[R_WHEEL_M]->tx_info->torque=-0.1;//往逆时针   
//		Chassis.Wheel->motor[L_WHEEL_M]->tx_info->torque=0.1;
//		Chassis.Wheel->motor[R_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[R_WHEEL_M]);
//		Chassis.Wheel->motor[L_WHEEL_M]->single_set_torque(Chassis.Wheel->motor[L_WHEEL_M]);
