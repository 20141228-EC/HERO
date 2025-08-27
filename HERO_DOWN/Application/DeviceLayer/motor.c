/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机驱动
  ******************************************************************************
  * @attention
  * typedef struct
  * {
  *  	can_id_t    id;				// CAN1或CAN2
  * 	uint32_t	rx_id;  		// 反馈报文标识符
  * 	uint32_t	tx_id;  		// 上传报文标识符
  * 	uint8_t		data_idx;		// 数组索引
  * 	uint8_t		*CANx_XXX_DATA; // 发送的数组
  * } drv_can_t;
  * 3508最大电流16384
  * 6020最大电压25000
  * 2006最大电流10000
  * 最大电流不应超过额定值
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include <RM_motor.h>

/* Private variables ---------------------------------------------------------*/
drv_can_t rm_motor_driver[] = {
	[CHAS_LF] = {
		.rx_id = ID_CHAS_LF,
		.can_id = DRV_CAN2, 
	},
	[CHAS_RF] = {
		.rx_id = ID_CHAS_RF,
		.can_id = DRV_CAN2,
	},
	[CHAS_LB] = {
		.rx_id = ID_CHAS_LB,
		.can_id = DRV_CAN2,
	},
    [CHAS_RB] = {
		.rx_id = ID_CHAS_RB,
		.can_id = DRV_CAN2,
	},

};


rm_motor_info_t rm_motor_info[RM_MOTOR_LIST];

rm_motor_t rm_motor[] = {
	[CHAS_LF] = {
        .info = &rm_motor_info[CHAS_LF],
        .driver = &rm_motor_driver[CHAS_LF],
        .init = rm_motor_init,
		.motor_type = RM3508,
	},
	[CHAS_RF] = {
        .info = &rm_motor_info[CHAS_RF],
        .driver = &rm_motor_driver[CHAS_RF],
        .init = rm_motor_init,
		.motor_type = RM3508,
	},
	[CHAS_RB] = {
        .info = &rm_motor_info[CHAS_RB],
        .driver = &rm_motor_driver[CHAS_RB],
        .init = rm_motor_init,
		.motor_type = RM3508,
	},
	[CHAS_LB] = {
        .info = &rm_motor_info[CHAS_LB],
        .driver = &rm_motor_driver[CHAS_LB],
        .init = rm_motor_init,
		.motor_type = RM3508,
	}
};

/*底盘*/
motor_pid_t CHAS_speed_pid = {
		.speed.kp = 8,
		.speed.ki = 0.33,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 15000,
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};
motor_pid_t CHAS_LB_speed_pid = {
		.speed.kp = 8,
		.speed.ki = 0.33,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		#if HERO_TYPE==2
		.speed.out_max = 15000,
		#else
		.speed.out_max = 10000,
		#endif 
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};
motor_pid_t CHAS_RB_speed_pid = {
		.speed.kp = 8,
		.speed.ki = 0.33,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		#if HERO_TYPE==2
		.speed.out_max = 15000,
		#else
		.speed.out_max = 10000,
		#endif 
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};
motor_pid_t CHAS_position_pid = {
		.speed.kp = 8,
		.speed.ki = 0.1,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 1,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 10000,
	};
 
/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
	{
		rm_motor_init(&rm_motor[i] );
	}

	/*pid初始化*/
	/*底盘*/
	rm_motor_pid_init(&rm_motor[CHAS_LF].motor_all_pid.speed_pid,CHAS_speed_pid);
	rm_motor_pid_init(&rm_motor[CHAS_RF].motor_all_pid.speed_pid,CHAS_speed_pid);
	rm_motor_pid_init(&rm_motor[CHAS_RB].motor_all_pid.speed_pid,CHAS_RB_speed_pid);
	rm_motor_pid_init(&rm_motor[CHAS_LB].motor_all_pid.speed_pid,CHAS_LB_speed_pid);

	rm_motor_pid_init(&rm_motor[CHAS_LF].motor_all_pid.position_pid,CHAS_position_pid);
	rm_motor_pid_init(&rm_motor[CHAS_RF].motor_all_pid.position_pid,CHAS_position_pid);
	rm_motor_pid_init(&rm_motor[CHAS_RB].motor_all_pid.position_pid,CHAS_position_pid);
	rm_motor_pid_init(&rm_motor[CHAS_LB].motor_all_pid.position_pid,CHAS_position_pid);

}
 

void rm_motor_list_heart_beat()
{
	rm_motor_heart_beat(&rm_motor[CHAS_LF]);
	rm_motor_heart_beat(&rm_motor[CHAS_RF]);
	rm_motor_heart_beat(&rm_motor[CHAS_RB]);
	rm_motor_heart_beat(&rm_motor[CHAS_LB]);
}

/**
 * @brief 电机工作状态
 * @param  无
 * @retval 全部在线返回1，否则返回0
 */
uint8_t rm_motor_list_workstate()
{
	for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
	{
		if(rm_motor[i].work_state != DEV_ONLINE)
			return 0;
	}
        return 1;
}
 
