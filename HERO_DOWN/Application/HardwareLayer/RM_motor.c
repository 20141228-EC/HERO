/**
  ******************************************************************************
  * @file    RM_motor.c
  * @brief   电机控制
  * @version 
  * @date    
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rm_motor.h"
#include "pid.h"
#include "rp_math.h"
#include "motor.h" 
/**
 * @brief  电机将CAN数据存到CANX_XXX_DATA数组（如CAN1_200_DATA）中，
 *		   后由CANX_CMD_XXX函数发送
 * @param  motor: 电机结构体
 * @param  data: 电机数据
 */
void Motor_SendData(rm_motor_t *motor, int16_t data)
{

    *(motor->driver->CANx_XXX_DATA + motor->driver->data_id) = data>>8;
	*(motor->driver->CANx_XXX_DATA + motor->driver->data_id + 1) = data;
}

/**
 * @brief  电机单环pid控制速度
 * @param  motor: 电机结构体
 * @param  speed: 速度目标值
 * @retval None
 */
void Motor_ToSpeed(rm_motor_t *motor, int16_t speed)
{
	motor->motor_all_pid.speed_pid.speed.target=speed;
	motor->motor_all_pid.speed_pid.speed.measure = motor->info->speed;
	motor->motor_all_pid.speed_pid.speed.err = motor->motor_all_pid.speed_pid.speed.target - motor->motor_all_pid.speed_pid.speed.measure;
	single_pid_ctrl(&motor->motor_all_pid.speed_pid.speed);
	if(motor->work_state == DEV_ONLINE)
    {
        Motor_SendData(motor, motor->motor_all_pid.speed_pid.speed.out);
    }
}
float rm_motor_speed_pid_calc(rm_motor_t *motor, int16_t speed)
{
	motor->motor_all_pid.speed_pid.speed.target=speed;
	motor->motor_all_pid.speed_pid.speed.measure = motor->info->speed;
	motor->motor_all_pid.speed_pid.speed.err = motor->motor_all_pid.speed_pid.speed.target - motor->motor_all_pid.speed_pid.speed.measure;
	single_pid_ctrl(& motor->motor_all_pid.speed_pid.speed);
	return motor->motor_all_pid.speed_pid.speed.out;
}
	
	

/**
 * @brief  电机双环pid控制角度
 * @param  motor: 电机结构体
 * @param  angle: 角度目标值
 * @retval None
 */
void Motor_ToAngle(rm_motor_t *motor, int16_t angle)
{
	angle = angle % 8192;
	motor->motor_all_pid.angle_pid.angle.measure = motor->info->angle;
	motor->motor_all_pid.angle_pid.angle.err = angle - motor->motor_all_pid.angle_pid.angle.measure;

	if(motor->motor_all_pid.angle_pid.angle.err < 0)
	{
		motor->motor_all_pid.angle_pid.angle.err += 8192;
	}
	if(motor->motor_all_pid.angle_pid.angle.err > 4096)
	{
		motor->motor_all_pid.angle_pid.angle.err = motor->motor_all_pid.angle_pid.angle.err - 8192;
	}
	//角度环
	single_pid_ctrl(&motor->motor_all_pid.angle_pid.angle);
	motor->motor_all_pid.angle_pid.speed.err = motor->motor_all_pid.angle_pid.angle.out - motor->info->speed;
	//速度环
	single_pid_ctrl(&motor->motor_all_pid.angle_pid.speed);
	//发送数据到数组
    if(motor->work_state == DEV_ONLINE)
    {
        Motor_SendData(motor, motor->motor_all_pid.angle_pid.speed.out);
    }
}

 

/**
 * @brief  电机到达转子累计角度
 * @param  motor: 电机结构体
 * @param  angle: 角度目标值
 * @retval None
 */
void Motor_ToAxleAngle(rm_motor_t *motor, int32_t angle)
{
	motor->motor_all_pid.position_pid.angle.measure = motor->info->angle_sum;
	motor->motor_all_pid.position_pid.angle.err = angle - motor->motor_all_pid.position_pid.angle.measure;

	//角度环
	single_pid_ctrl(&motor->motor_all_pid.position_pid.angle);
	motor->motor_all_pid.position_pid.speed.err =motor->motor_all_pid.position_pid.angle.out - motor->info->speed;
	//速度环
	single_pid_ctrl(&motor->motor_all_pid.position_pid.speed);
	//发送数据到数组
	Motor_SendData(motor, motor->motor_all_pid.position_pid.speed.out);
}

float rm_motor_anglesum_pid_calc(rm_motor_t *motor, int32_t angle)
{
	motor->motor_all_pid.position_pid.angle.measure = motor->info->angle_sum;
	motor->motor_all_pid.position_pid.angle.err = angle - motor->motor_all_pid.position_pid.angle.measure;

	//角度环
	single_pid_ctrl(&motor->motor_all_pid.position_pid.angle);
	motor->motor_all_pid.position_pid.speed.err =motor->motor_all_pid.position_pid.angle.out - motor->info->speed;
	//速度环
	single_pid_ctrl(&motor->motor_all_pid.position_pid.speed);
	
	return motor->motor_all_pid.position_pid.speed.out;

}
/**
 * @brief  立即检测是否堵转。
 * @param  motor: 电机结构体
 * @retval 1: 堵转 0: 未堵转
 */
int8_t Motor_DetectStuck_immediately(rm_motor_t *motor)
{
	if(abs(motor->info->speed)<=50 && abs(motor->info->current) > 3000)//达到堵转条件
	{
		return 1;
	}
	else
		return 0;
}

/**
 * @brief  电机检测是否堵转，速度为0且有电流判为堵转。
 * @param  motor: 电机结构体
 * @retval 1: 堵转 0: 未堵转
 */
int8_t Motor_DetectStuck(rm_motor_t *motor)
{
	if(abs(motor->info->speed)<=50 && abs(motor->info->current) > 1000)//达到堵转条件
	{
		motor->cnt++;
		if(motor->cnt >= 40)//达到堵转条件并且计时够
		{
			motor->cnt = 40;
			return 1;
		}
		return 0;//达到堵转条件但是计时不够
	}
	else
	{
		motor->cnt = 0;//没堵转了就置零
		return 0;
	}
}
/**
 * @brief  堵转检测+处理，速度为0且有电流判为堵转。
 * @param  motor: 电机结构体
 * @retval 无
 */
void Motor_HandleStuck(rm_motor_t *motor)
{
	if((motor->info->speed == 0 && abs(motor->info->current) > 5000)|| motor->cnt > 40)
	{
		motor->cnt++;
		if(motor->cnt > 40)
		{	
			/*卡住40ms后反转40ms*/
			Motor_ToSpeed(motor, -1000);
			if(motor->cnt > 240)
			{	
				motor->cnt = 0;
			}
		}
	}
	else
	{
		motor->cnt = 0;
	}
}

/**
 * @brief  电机初始化
 * @param  motor: 电机结构体
 * @retval 无
 */
void rm_motor_init(rm_motor_t *motor)
{ 
	if(motor == NULL )
	{
		return;
	}
	if(motor->motor_type==RM3508||motor->motor_type==RM2006)
	{
		if(motor->driver->rx_id-0x201U<4)
		{
			motor->driver->data_id=(motor->driver->rx_id-0x200U)*2-0x2;
			if(motor->driver->can_id==DRV_CAN1)
			{
				motor->driver->CANx_XXX_DATA=CAN1_200_DATA;
				motor->driver->tx_id=0x200;
			}
			else
			{
				motor->driver->CANx_XXX_DATA=CAN2_200_DATA;
				motor->driver->tx_id=0x200;
			}
		}
		else
		{
			motor->driver->data_id=(motor->driver->rx_id-0x204U)*2-0x2;
			if(motor->driver->can_id==DRV_CAN1)
			{
				motor->driver->CANx_XXX_DATA=CAN1_1FF_DATA;
				motor->driver->tx_id=0x1FF;
			}
			else
			{
				motor->driver->CANx_XXX_DATA=CAN2_1FF_DATA;
				motor->driver->tx_id=0x1FF;
			}
		}
	}
		
	
	if(motor->motor_type==GM6020)
	{
		if(motor->driver->rx_id-0x205U<4)
		{
			motor->driver->data_id=(motor->driver->rx_id-0x204U)*2-0x2;
			if(motor->driver->can_id==DRV_CAN1)
			{
				motor->driver->CANx_XXX_DATA=CAN1_1FF_DATA;
				motor->driver->tx_id=0x1FF;
			}
			else
			{
				motor->driver->CANx_XXX_DATA=CAN2_1FF_DATA;
				motor->driver->tx_id=0x1FF;
			}
		}
		else
		{
			motor->driver->data_id=(motor->driver->rx_id-0x208U)*2-0x2;
			if(motor->driver->can_id==DRV_CAN1)
			{
				motor->driver->CANx_XXX_DATA=CAN1_2FF_DATA;
				motor->driver->tx_id=0x2FF;
			}
			else
			{
				motor->driver->CANx_XXX_DATA=CAN2_2FF_DATA;
				motor->driver->tx_id=0x2FF;
			}
		}
	}
	
	motor->heart_beat=rm_motor_heart_beat;
	motor->check=rm_motor_check;
	motor->update=rm_motor_update;
	motor->work_state = DEV_OFFLINE;
    motor->info->init_flag = 1;
    motor->info->offline_cnt = 0;
    motor->info->offline_max_cnt = 50;
}

/**
 * @brief  电机PID单个PID结构体初始化
 */
void rm_motor_pid_init(motor_pid_t *motor_pid,motor_pid_t extern_motor_pid)
{ 
	if(motor_pid==NULL)
	{
		return;
	}
	*motor_pid=extern_motor_pid;
}

/**
 * @brief  电机心跳失联检测
 * @param  motor: 电机结构体
 * @retval 无
 */
void rm_motor_heart_beat(rm_motor_t *motor)
{
    rm_motor_info_t *motor_info = motor->info;
    motor_info->offline_cnt++;
    if(motor_info->offline_cnt > motor_info->offline_max_cnt) 
	{
        motor_info->offline_cnt = motor_info->offline_max_cnt;
        motor->work_state = DEV_OFFLINE;
    }
    else 
	{
        if(motor->work_state == DEV_OFFLINE)
            motor->work_state = DEV_ONLINE;
    }
}

uint8_t Motor_InPosition(rm_motor_t *motor)
{
    if(abs(motor->motor_all_pid.angle_pid.angle.err) < 1)
    {
            return 1;
    }
    else
    {
        return 0;
    }
}
