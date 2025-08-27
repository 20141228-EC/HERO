/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   ��������,����������̨�ͷ������
  ******************************************************************************
  */
#include "control_task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "rp_user_config.h"
#include "rp_config.h"		
#include "chassis.h"


//#define HT_MOTOR
#define LK_MOTOR
#define RM_MOTOR
float target_p=0; float target_v=5; float p_kp=0; float v_kd=0.5; float f_t=0;
float mec_target;
uint32_t Init_tick;
uint16_t id=0x142;
float t=200;
void StartControlTask(void const * argument)
{
	Car_Init();
	#ifdef HT_MOTOR
	ht_motor.mode_cmd(&ht_motor,CMD_MOTOR_MODE);
	#endif
	
	Init_tick=HAL_GetTick();
	for(;;)
	{
		#ifdef HT_MOTOR
		ht_motor.control_cmd(&ht_motor,target_p,target_v,p_kp,v_kd,f_t);
		#endif
		
		#ifdef RM_MOTOR
		//1.大疆6020接收ID是电机亮灯次数（拨码数）+4，3508直接是电机亮灯次数（拨码数）
		//2.改电机种类
		//3.改标识符CAN1_CMD_1FF();还是CAN1_CMD_200();
		Motor_ToSpeed(&rm_motor[DAIL],200);
		CAN1_CMD_200();
		CAN1_CMD_1FF();
		#endif
		
		#ifdef LK_MOTOR
		//①改can接收id
		float mec_meas_out=gimbal.gimbal_y->KT_motor_info.rx_info.encoder/65535.f*360.f;
		float mec_meas_in=gimbal.gimbal_y->KT_motor_info.rx_info.speed;
		gimbal.gimbal_y->base_info.motor_out = all_pid_calc(&gimbal.gimbal_y->motor_all_pid.mec_pid.angle,&gimbal.gimbal_y->motor_all_pid.mec_pid.speed,mec_target,mec_meas_out,mec_meas_in,-1,3 );
		kt_motor[0].KT_motor_info.id.tx_id=id;
		kt_motor[0].W_iqControl(&kt_motor[0],gimbal.gimbal_y->base_info.motor_out);
		kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID); //YAW
		#endif
//		if(HAL_GetTick()-Init_tick>=1000)
//		{
//			osDelay(1);
//		}
                                                                                    
		osDelay(1);
		
	}
} 

/* 1 测试命令锁
* 2. 测试不横着吊射宏定义，
* 3.  
* 4.可以加领空电机的初始化
* 5. 
* 6. 
* 7. 
 */



//		Car_Ctrl(&car);
//		Device_Work();
//		CAN_Send();
