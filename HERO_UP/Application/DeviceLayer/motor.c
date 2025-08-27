
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include <RM_motor.h>

/* Private variables ---------------------------------------------------------*/
// 添加电机时需要初始化的参数：3508还是6020，CAN1还是CAN2，以及电机的接收ID；
// CAN发送数组包含了用CAN1还是CAN2，发送数据下标在电机Motor_SendData判断。
//

drv_can_t rm_motor_driver[] = {
	[DAIL] = {
		.rx_id = ID_DAIL,
		.can_id = DRV_CAN1,
	},
	[GIMB_P] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_GIMB_P,  //0x204+电机id：2
	},
	[FRIC_F_UP] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_FRIC_F_UP,

	},
	[FRIC_F_L] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_FRIC_F_L,

	},
	[FRIC_F_R] = {
		.can_id=DRV_CAN2,
		.rx_id=ID_FRIC_F_R,
	},
	[FRIC_B_UP] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_FRIC_B_UP,

	},
	[FRIC_B_L] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_FRIC_B_L,

	},
	[FRIC_B_R] = {
		.can_id=DRV_CAN2,
		.rx_id=ID_FRIC_B_R,
	},
	[IMAGE] = {
		.rx_id = ID_IMAGE,
		.can_id = DRV_CAN1,
	},
	[TELESCOPE] = {
		.rx_id = ID_TELESCOPE,
		.can_id = DRV_CAN1,
	},
};
/*PID结构体定义------------------------------------------------*/
// 注意定义了之后需要在rm_motor_list_init用rm_motor_pid_init初始化
/*第二级摩擦轮*/
motor_pid_t FRIC_F_L_speed = {
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

motor_pid_t FRIC_F_R_speed = {
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

motor_pid_t FRIC_F_UP_speed = {
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
/*第一级摩擦轮*/
motor_pid_t FRIC_B_L_speed = {
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
motor_pid_t FRIC_B_R_speed = {
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
motor_pid_t FRIC_B_UP_speed = {
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
#if HERO_TYPE ==3
motor_pid_t GIMB_P_mec = {
	.speed.kp = 800,
	.speed.ki = 5,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 1, // 0.45
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,    
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};
motor_pid_t GIMB_P_gyro = {
	.speed.kp = 800, // 300 3 0 7 0 0  
	.speed.ki = 8,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 28000,
	.angle.kp = 13,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};

#elif HERO_TYPE ==2
motor_pid_t GIMB_P_mec = {
	.speed.kp = 200,
	.speed.ki = 6,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 1.7, // 0.45
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};
motor_pid_t GIMB_P_gyro = {
	.speed.kp = 200, // 300 3 0 7 0 0  
	.speed.ki = 10,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 28000,
	.angle.kp = 8,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};
#elif HERO_TYPE ==1
motor_pid_t GIMB_P_mec = {
	.speed.kp = 250,
	.speed.ki = 5,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 0.7, // 0.45
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};
motor_pid_t GIMB_P_gyro = {
	.speed.kp = 800, // 300 3 0 7 0 0  
	.speed.ki = 10,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 28000,
	.angle.kp = 16,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
	.angle.feedforward.ka = 0,
	.angle.feedforward.kb = 0,
	.angle.feedforward.feedforward_outmax = 5000,
	.angle.dynamic_integration.full_speed_integral_threshold = 0,
	.angle.dynamic_integration.lower_speed_integral_threshold = 0,
	.angle.dynamic_integration.use_dynamic_integration = 0,
};
#endif




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
#if HERO_TYPE ==3
motor_pid_t DAIL_position={
		.speed.kp = 15,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 0,
		.speed.out_max = 20000,
		.angle.kp = 0.15,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000
};
#elif HERO_TYPE ==2
motor_pid_t DAIL_position={
		.speed.kp = 15,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 0,
		.speed.out_max = 20000,
		.angle.kp = 0.15,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000
};
#elif HERO_TYPE ==1
motor_pid_t DAIL_position={
		.speed.kp = 15,
		.speed.ki = 0,
		.speed.kd = 0,
		.speed.integral_max = 0,
		.speed.out_max = 20000,
		.angle.kp = 0.25,
		.angle.ki = 0,
		.angle.kd = 0,
		.angle.integral_max = 0,
		.angle.out_max = 8000
};
#endif
/*IMAGE*/
motor_pid_t IMAGE_position = {
	.speed.kp = 20,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 2500,
	.speed.out_max = 3000,
	.angle.kp = 0.4,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 1000,
	.angle.out_max = 1000,
};

motor_pid_t IMAGE_speed = {
	.speed.kp = 5,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 8000,
	.speed.out_max = 3000,
	.angle.kp = 1,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 1000,
	.angle.out_max = 1000,
};
/*yaw*/
#if  HERO_TYPE ==3
motor_pid_t GIMB_Y_mec = {
	.speed.kp = 20,
	.speed.ki = 0.5,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 7, 
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
}; 
motor_pid_t GIMB_Y_gyro = {
	.speed.kp = 20,
	.speed.ki = 0.15,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 7,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
};
#elif  HERO_TYPE ==2
motor_pid_t GIMB_Y_mec = {
	.speed.kp = 20,
	.speed.ki = 0.2,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 17, 
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
}; 
motor_pid_t GIMB_Y_gyro = {
	.speed.kp = 20,
	.speed.ki = 0.2,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 12, 
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
};
#elif  HERO_TYPE ==1
motor_pid_t GIMB_Y_mec = {
	.speed.kp = 20,
	.speed.ki = 0.2,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 8, 
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
}; 
motor_pid_t GIMB_Y_gyro = {
	.speed.kp = 20,
	.speed.ki = 0.2,
	.speed.kd = 0,
	.speed.integral_max = 1000,
	.speed.out_max = 20000,
	.angle.kp = 8, 
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 1680,
};
#endif
/*TELESCOPE*/
motor_pid_t TELESCOPE_position = {
	.speed.kp = 20,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 2500,
	.speed.out_max = 2500,
	.angle.kp = 0.4,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 1000,
	.angle.out_max = 1000,
};

motor_pid_t TELESCOPE_speed = {
	.speed.kp = 5,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 8000,
	.speed.out_max = 2500,
	.angle.kp = 1,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 1000,
	.angle.out_max = 1000,
};

rm_motor_info_t rm_motor_info[RM_MOTOR_LIST];

rm_motor_t rm_motor[] = {
	[GIMB_P] = {
		.info = &rm_motor_info[GIMB_P],
		.driver = &rm_motor_driver[GIMB_P],
		.init = rm_motor_init,
		.motor_type = GM6020,
	},
	[FRIC_F_L] = {
        .info = &rm_motor_info[FRIC_F_L],
        .driver = &rm_motor_driver[FRIC_F_L],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[FRIC_F_UP] = {
        .info = &rm_motor_info[FRIC_F_UP],
        .driver = &rm_motor_driver[FRIC_F_UP],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[FRIC_F_R] = {
        .info = &rm_motor_info[FRIC_F_R],
        .driver = &rm_motor_driver[FRIC_F_R],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[FRIC_B_L] = {
        .info = &rm_motor_info[FRIC_B_L],
        .driver = &rm_motor_driver[FRIC_B_L],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[FRIC_B_UP] = {
        .info = &rm_motor_info[FRIC_B_UP],
        .driver = &rm_motor_driver[FRIC_B_UP],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[FRIC_B_R] = {
        .info = &rm_motor_info[FRIC_B_R],
        .driver = &rm_motor_driver[FRIC_B_R],
		.init = rm_motor_init,
        .motor_type = RM3508,
	},
	[DAIL] = {
		.info = &rm_motor_info[DAIL],
		.driver = &rm_motor_driver[DAIL],
		.init = rm_motor_init,
		.motor_type = RM3508,

	},
	[IMAGE] = {
		.info = &rm_motor_info[IMAGE],
		.driver = &rm_motor_driver[IMAGE],
		.init = rm_motor_init,
		.motor_type = RM2006,
	},
	[TELESCOPE] = {
		.info = &rm_motor_info[TELESCOPE],
		.driver = &rm_motor_driver[TELESCOPE],
		.init = rm_motor_init,
		.motor_type = RM2006,
	}
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
	/*电机信息初始化*/
	for (uint8_t i = 0; i < RM_MOTOR_LIST; i++)
	{
		rm_motor_init(&rm_motor[i]);
	}
	/*pid参数初始化*/
	/*pitch*/
	motor_pid_init(&rm_motor[GIMB_P].motor_all_pid.gyro_pid, GIMB_P_gyro);
	motor_pid_init(&rm_motor[GIMB_P].motor_all_pid.mec_pid, GIMB_P_mec);

	/*摩擦轮*/
	motor_pid_init(&rm_motor[FRIC_F_UP].motor_all_pid.speed_pid,FRIC_F_UP_speed);
	motor_pid_init(&rm_motor[FRIC_F_R].motor_all_pid.speed_pid,FRIC_F_R_speed);
	motor_pid_init(&rm_motor[FRIC_F_L].motor_all_pid.speed_pid,FRIC_F_L_speed);
	
	motor_pid_init(&rm_motor[FRIC_B_UP].motor_all_pid.speed_pid,FRIC_B_UP_speed);
	motor_pid_init(&rm_motor[FRIC_B_R].motor_all_pid.speed_pid,FRIC_B_R_speed);
	motor_pid_init(&rm_motor[FRIC_B_L].motor_all_pid.speed_pid,FRIC_B_L_speed);

	/*拨盘*/
	motor_pid_init(&rm_motor[DAIL].motor_all_pid.position_pid,DAIL_position);
	motor_pid_init(&rm_motor[DAIL].motor_all_pid.speed_pid,DAIL_speed);
	
	/*图传*/
	motor_pid_init(&rm_motor[IMAGE].motor_all_pid.position_pid,IMAGE_position);
	motor_pid_init(&rm_motor[IMAGE].motor_all_pid.speed_pid,IMAGE_speed);
	
	/*望远镜*/
	motor_pid_init(&rm_motor[TELESCOPE].motor_all_pid.position_pid,TELESCOPE_position);
	motor_pid_init(&rm_motor[TELESCOPE].motor_all_pid.speed_pid,TELESCOPE_speed);
}

void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);
	motor_pid_init(&kt_motor[0].motor_all_pid.mec_pid, GIMB_Y_mec);
	motor_pid_init(&kt_motor[0].motor_all_pid.gyro_pid, GIMB_Y_gyro);
}

void rm_motor_list_heart_beat()
{
	for (uint8_t i = 0; i < RM_MOTOR_LIST; i++)
	{
		rm_motor[i].heart_beat(&rm_motor[i]);
	}
}

/**
 * @brief 电机工作状态
 * @param  无
 * @retval 全部在线返回1，否则返回0
 */
uint8_t rm_motor_list_workstate()
{
	for (uint8_t i = 0; i < RM_MOTOR_LIST; i++)
	{
		if (rm_motor[i].work_state != DEV_ONLINE)
			return 0;
	}
	return 1;
}
