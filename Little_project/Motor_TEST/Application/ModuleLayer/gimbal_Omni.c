/* Includes ------------------------------------------------------------------*/
#include "gimbal_Omni.h"

/*------------------------- Private variables start --------------------------------*/

int16_t Pitch_current;

/*------------------------- Private functions start --------------------------------*/
void Pitch_to_target(int16_t angle)
{		
		rm_motor[5].motor_pid.angle.measure = rm_motor[5].info->angle;
		rm_motor[5].motor_pid.angle.err = angle - rm_motor[5].motor_pid.angle.measure;

		single_pid_ctrl(&rm_motor[5].motor_pid.angle);  //计算外环

		rm_motor[5].motor_pid.speed.err=rm_motor[5].motor_pid.angle.out-imu_sensor.info->raw_info.gyro_x; //速度环的期望输入是位置环的输出
		single_pid_ctrl(&rm_motor[5].motor_pid.speed);		//计算内环

		Pitch_current=rm_motor[5].motor_pid.speed.out;
		
		if(rm_motor[5].work_state == DEV_ONLINE)
		{
			int16_to_uint8(&CAN2_1FF_DATA[2],&Pitch_current);
		}
		
}

void Yaw_to_target(int16_t angle)
{
	Motor_ToAngle(&rm_motor[4],angle);
}


/*------------------------- Private functions end --------------------------------*/
