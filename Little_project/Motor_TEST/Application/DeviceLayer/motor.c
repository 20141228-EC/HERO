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
//添加电机时需要初始化的参数：3508还是6020，CAN1还是CAN2，以及电机的接收ID；
//CAN发送数组包含了用CAN1还是CAN2，发送数据下标在电机Motor_SendData判断。
//

drv_can_t rm_motor_driver[] = {
	[DAIL] = {
		.rx_id = ID_DAIL,
		.can_id = DRV_CAN1,
	},
	[FRIC_RF] = {
		.rx_id = ID_FRIC_RF,
		.can_id = DRV_CAN2,
	},
	[FRIC_LB] = {
		.rx_id = ID_FRIC_LB,
		.can_id = DRV_CAN2,
	},
	[FRIC_RB] = {
		.rx_id=ID_FRIC_RB,
		.can_id = DRV_CAN2,
	},
	[FRIC_LF] = {
		.rx_id=ID_FRIC_LF,
		.can_id = DRV_CAN2,
	},
	[GIMB_P] = {
		.rx_id = ID_GIMB_P,
		.can_id = DRV_CAN2,
	},
	[LIMIT] = {
		.rx_id = ID_LIMIT,
		.can_id = DRV_CAN2,
	},
	[IMAGE] = {
		.rx_id = ID_IMAGE,
		.can_id = DRV_CAN2,
	},
};
/*PID结构体定义------------------------------------------------*/
//注意定义了之后需要在rm_motor_list_init用rm_motor_pid_init初始化
//索引：摩擦轮：FRIC_LF_speed  FRIC_RF_speed  FRIC_LB_speed  FRIC_RB_speed  
//pitch：GIMB_P_mec  GIMB_P_gyro    
//拨盘：DAIL_speed
//限位：LIMIT_speed  LIMIT_position
//图传：IMAGE_position
/*摩擦轮*/
motor_pid_t FRIC_LF_speed = {
		.speed.kp = 27,
		.speed.ki = 0.5,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};

motor_pid_t FRIC_RF_speed = {
		.speed.kp = 27,
		.speed.ki = 0.5,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};

motor_pid_t FRIC_LB_speed = {
		.speed.kp = 27,
		.speed.ki = 0.5,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};
motor_pid_t FRIC_RB_speed = {
		.speed.kp = 27,
		.speed.ki = 0.5,
		.speed.kd = 0,
		.speed.integral_max = 6000,
		.speed.out_max = 10000,
		.angle.kp = 0,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000,
	};
/*Pitch*/
motor_pid_t GIMB_P_mec = {
		.speed.kp = 200,
		.speed.ki = 5,
		.speed.kd = 0,
		.speed.integral_max = 3000,
		.speed.out_max = 28000,
		.angle.kp = 0.7,//0.45
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 500,
		.angle.feedforward.ka=0,
		.angle.feedforward.kb=0,
		.angle.feedforward.feedforward_outmax=5000,
		.angle.dynamic_integration.full_speed_integral_threshold=0,
		.angle.dynamic_integration.lower_speed_integral_threshold=0,
		.angle.dynamic_integration.use_dynamic_integration=0,
	};

motor_pid_t GIMB_P_gyro={
		.speed.kp = 300,          //300 3 0 7 0 0  
		.speed.ki = 5,			 
		.speed.kd = 0,			 
		.speed.integral_max = 3000, 
		.speed.out_max = 28000,	 
		.angle.kp = 13,	
		.angle.ki = 0,			 	 
		.angle.kd = 0,				 
		.angle.integral_max = 0,	 
		.angle.out_max = 500,
		.angle.feedforward.ka=0,
		.angle.feedforward.kb=0,
		.angle.feedforward.feedforward_outmax=5000,
		.angle.dynamic_integration.full_speed_integral_threshold=0,
		.angle.dynamic_integration.lower_speed_integral_threshold=0,
		.angle.dynamic_integration.use_dynamic_integration=0,
};


/*拨盘*/
motor_pid_t DAIL_speed = {
		.speed.kp = 15,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 8000,
		.speed.out_max = 28000,
		.angle.kp = 0.25,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 1000,
	};
/*主动限位*/
motor_pid_t LIMIT_speed = {
		.speed.kp = 1,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 8000,
		.speed.out_max = 10000,
		.angle.kp = 1,
		.angle.ki = 0,
		.angle.kd = 6,
		.angle.integral_max = 3000,
		.angle.out_max = 5000,
	};

motor_pid_t LIMIT_position = {
		.speed.kp = 3,
		.speed.ki = 0,
		.speed.kd = 10,
		.speed.integral_max = 8000,
		.speed.out_max = 10000,
		.angle.kp = 2,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 3000,
		.angle.out_max = 5000,
	};
/*图传电机*/
motor_pid_t IMAGE_position = {
		.speed.kp = 5,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 8000,
		.speed.out_max = 5000,
		.angle.kp = 1,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 1000,
		.angle.out_max = 1000,
	};
/*yaw*/
motor_pid_t GIMB_Y_mec = {
		.speed.kp = 0.1,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 1000,
		.speed.out_max = 20000,
		.angle.kp = 100,//0.45
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 1680,
	};
motor_pid_t GIMB_Y_gyro = {
		.speed.kp = 13,
		.speed.ki = 0.02,
		.speed.kd = 0,
		.speed.integral_max = 7000,
		.speed.out_max = 20000,
		.angle.kp = 10,//0.45
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 1680,
	}; 


drv_can_t ht_motor_drive={
		.rx_id = 0x01,
		.tx_id =0x01,
		.can_id = DRV_CAN1,
};

HT_motor_t ht_motor={
	
	.driver =&ht_motor_drive,
	.init=ht_motor_class_init,
};

rm_motor_info_t rm_motor_info[RM_MOTOR_LIST];
rm_motor_t rm_motor[] = {
	[GIMB_P] = {
        .info = &rm_motor_info[GIMB_P],
        .driver = &rm_motor_driver[GIMB_P],
		.init=rm_motor_init,
		.motor_type = GM6020,
	},
	[FRIC_LB] = {
        .info = &rm_motor_info[FRIC_LB],
        .driver = &rm_motor_driver[FRIC_LB],
		.init=rm_motor_init,
		.motor_type = RM3508,
	},
	[FRIC_RB] = {
        .info = &rm_motor_info[FRIC_RB],
        .driver = &rm_motor_driver[FRIC_RB],
		.init=rm_motor_init,
		.motor_type = RM3508,
	},
	[FRIC_LF] = {
        .info = &rm_motor_info[FRIC_LF],
        .driver = &rm_motor_driver[FRIC_LF],
		.init=rm_motor_init,
		.motor_type = RM3508,
	},
	[FRIC_RF] = {
        .info = &rm_motor_info[FRIC_RF],
        .driver = &rm_motor_driver[FRIC_RF],
		.init=rm_motor_init,
		.motor_type = RM3508,
	},
	[DAIL] = {
        .info = &rm_motor_info[DAIL],
        .driver = &rm_motor_driver[DAIL],
		.init=rm_motor_init,
		.motor_type = RM3508,

	},
	[LIMIT] = {
        .info = &rm_motor_info[LIMIT],
        .driver = &rm_motor_driver[LIMIT],
		.init=rm_motor_init,
		.motor_type = RM2006,
	},
	[IMAGE] = {
        .info = &rm_motor_info[IMAGE],
        .driver = &rm_motor_driver[IMAGE],
		.init=rm_motor_init,
		.motor_type = RM2006,
	},
};


KT_motor_t kt_motor[] = {
    [0] = {
        .KT_motor_info = {
            .tx_info = {
                .angle_single_Control = 0,
                .angle_single_Control_maxSpeed = 0,
                .angle_single_Control_spinDirection = 0,
                .angle_add_Control = 0,
                .angle_add_Control_maxSpeed = 0,
                .angle_sum_Control = 0,
                .angle_sum_Control_maxSpeed = 0,
                .iqControl = 0,
                .speedControl = 0,
            },
            .id = {
                .tx_id = ID_GIMB_YAW,
                .rx_id = 0x88,
                .drive_type = M_CAN1,
                .motor_type = KT9015,
            },
        },
        .init = KT_motor_class_init,
    },
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
	{
		rm_motor_init(&rm_motor[i]);
	}
	/*pid初始化*/
	/*pitch*/
	motor_pid_init(&rm_motor[GIMB_P].motor_all_pid.gyro_pid,GIMB_P_gyro);
	motor_pid_init(&rm_motor[GIMB_P].motor_all_pid.mec_pid,GIMB_P_mec);

	/*摩擦轮*/
	motor_pid_init(&rm_motor[FRIC_LF].motor_all_pid.speed_pid,FRIC_LF_speed);
	motor_pid_init(&rm_motor[FRIC_RF].motor_all_pid.speed_pid,FRIC_RF_speed);
	motor_pid_init(&rm_motor[FRIC_LB].motor_all_pid.speed_pid,FRIC_LB_speed);
	motor_pid_init(&rm_motor[FRIC_RB].motor_all_pid.speed_pid,FRIC_RB_speed);
	/*主动限位*/
	motor_pid_init(&rm_motor[LIMIT].motor_all_pid.speed_pid,LIMIT_speed);
	motor_pid_init(&rm_motor[LIMIT].motor_all_pid.position_pid,LIMIT_position);
	/*拨盘*/
	motor_pid_init(&rm_motor[DAIL].motor_all_pid.speed_pid,DAIL_speed);
}

 void kt_motor_list_init()
{
    kt_motor[0].init(&kt_motor[0]);
	motor_pid_init(&kt_motor[0].motor_all_pid.mec_pid,GIMB_Y_mec);
	motor_pid_init(&kt_motor[0].motor_all_pid.gyro_pid,GIMB_Y_gyro);
}

void rm_motor_list_heart_beat()
{
	for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
	{
		rm_motor[i].heart_beat(&rm_motor[i] );
	}
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


