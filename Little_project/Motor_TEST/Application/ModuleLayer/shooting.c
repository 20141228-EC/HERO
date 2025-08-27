
/* Includes ------------------------------*/
#include "shooting.h"

/* Private function prototypes -----------------------------------------------*/
uint8_t Shooting_Work_Reloading(shooting_t *shooting);//补弹逻辑
void Shooting_Fri_Speed_Adapt(shooting_t *shooting);  //弹速自适应
void Shooting_Pid_Calculating(shooting_t *shooting);   //pid计算
void Fri_Speed_Check(shooting_t *shooting);				//摩擦轮速度检查
void Shooting_Command_Execute(shooting_t *shooting);	//发射命令执行
void Shooting_Extreme_Low_Value_Calculation(shooting_t *shooting); //摩擦轮转速极低值计算
bool Check_Cooling_Heat(void);								//检查热量是否允许发射 
//摩擦轮速度配置
shooting_config_t config =
{
	.target_b_speed = 5150.f,  //5000
	.target_f_speed = 5150.f,
	.target_bullet_speed = 15.8,
};

shooting_t shooting=
{
	.frictionLF = &rm_motor[FRIC_LF],
	.frictionRF = &rm_motor[FRIC_RF],
	.frictionLB = &rm_motor[FRIC_LB],
	.frictionRB = &rm_motor[FRIC_RB],
	.dail       = &rm_motor[DAIL],
	.limit      = &rm_motor[LIMIT],
	
	.cmd_fire   = &command[SHOOTING_FIRE],
	.cmd_firing = &command[SHOOTING_FIRING],
	.cmd_kill_myself = &command[KILL_MYSELF],
	.config     = &config,
	.base_info.dail_info.dail_reset_state = DEV_RESET_NO,
	.work       =Shooting_Work,
	.stop_shooting_flag=0,
	.shooting_state=SHOOTING_RESET_NO,
	.load_state = load_NO,
};


/**
 * @brief 补弹
 * 
 * @param shooting 
 * @return uint8_t 补弹完成：1 未完成：0
 */
uint8_t Shooting_Work_Reloading(shooting_t *shooting)
{
	int32_t position = shooting->dail->info->angle_sum;//拨盘现在位置
	int16_t speed    = shooting->dail->info->speed;    //拨盘现在速度 
	dail_info_t *info = &shooting->base_info.dail_info;
	
	uint8_t res = 0;
	float dail_max_work_time;//拨盘最大工作时间
/*动态改变最大工作时间************************************************/
	if (info->dail_reset_state == DEV_RESET_NO)//波盘未初始化
	{
		dail_max_work_time = DAIL_MAX_INIT_TIMES;
	}
	else//波盘已初始化
	{
		dail_max_work_time = DAIL_MAX_WORK_TIMES;
	}
/*补弹逻辑**********************************************************/
	switch (info->work_state)
	{
	case DAIL_SLEEP://唤醒拨盘，开始补弹
		info->work_times = 0;
		info->target_speed = 0;
		info->f_stucking_cnt = 0;
		if (shooting->load_state == load_NO 
				&& shooting->base_info.fri_info.fri_speed_state == true)//需要装弹且摩擦轮速度正常
		{
			info->work_state = DAIL_RELOAD;
		}
		break;
	case DAIL_RELOAD://正转补弹
		if (Motor_DetectStuck(shooting->dail) != 1)//正转没有堵转 
		{
			if (info->work_times >= dail_max_work_time)//工作超时,没有弹->供弹完成，退出
			{
				/* break */
				info->work_times = 0;
				info->target_speed = 0;
				info->f_stucking_cnt = 0;
				shooting->load_state = load_OK;
				info->work_state = DAIL_SLEEP;
				res = 1;
			}
			else//没有堵转且工作没有超时->有弹
			{
				info->target_speed = DAIL_FEED_SPEED ;
				info->work_times++;
			}
		}
		else//正常情况下应该不会进入
		{
			info->f_stucking_cnt++;
			if (info->f_stucking_cnt < DAIL_MAX_F_STUCK_CNT)//堵转次数未满足->反转
			{
				/*正转下的堵转才会改变反转目标位置*/ 
				info->target_position = position + DAIL_REVERT_PISITION;//计算反转目标位置
				info->work_times = 0; //工作时间清零
				info->work_state = DAIL_REVERT;//进入反转模式
			}
			else//正向堵转满足次数->供弹成功，退出，只堵转一次不能保证供弹成功，所以多堵一次
			{
				/* break */
				info->work_times = 0;
				info->target_speed = 0;
				info->f_stucking_cnt = 0;
				shooting->load_state = load_OK;
				info->work_state = DAIL_SLEEP;
				 
				res = 1;
			}
		}
		break;
	case DAIL_REVERT:
		if (Motor_DetectStuck(shooting->dail) == 1)//反转堵转了->正转补弹
		{
			info->work_state = DAIL_RELOAD;
		}
		else if(position <= info->target_position)//(反转为负)没堵转但到位了->正转补弹
		{
			info->work_state = DAIL_RELOAD;
		}
		else if (info->work_times >= dail_max_work_time)//没堵转但工作超时->供弹完成，退出
		{
			/* break */
			info->work_times = 0;
			info->target_speed = 0;
			info->f_stucking_cnt = 0;
			shooting->load_state = load_OK;
			info->work_state = DAIL_SLEEP;
			res = 1;
		}
		else//没堵转也没到位
		{
			info->work_times++;
			info->target_speed = DAIL_STUCK_SPEED;//保持反转速度
		}
		break;
	default:
		break;
	}
	return res;
}

/**
 * @brief 发射PID计算
 * 
 * @param shooting 
 */
void Shooting_Pid_Calculating(shooting_t *shooting)
{
	//拨盘PID计算
	shooting->base_info.output_dail=rm_motor_speed_pid_calc(&rm_motor[DAIL],shooting->base_info.dail_info.target_speed);
	//限位PID计算
	if (shooting->base_info.limit_info.ctrl_mode == LIMIT_SPEED_MODE)
	{
		shooting->base_info.output_limit=rm_motor_speed_pid_calc(&rm_motor[LIMIT],shooting->base_info.limit_info.target_speed);
	}
	else
	{
		shooting->base_info.output_limit=rm_motor_anglesum_pid_calc(&rm_motor[LIMIT],shooting->base_info.limit_info.target_position);
	}
	//摩擦轮PID计算

		shooting->base_info.output_friLF=rm_motor_speed_pid_calc(&rm_motor[FRIC_LF],shooting->base_info.fri_info.target_f_speed);
		shooting->base_info.output_friRF=rm_motor_speed_pid_calc(&rm_motor[FRIC_RF],-shooting->base_info.fri_info.target_f_speed);
		shooting->base_info.output_friLB=rm_motor_speed_pid_calc(&rm_motor[FRIC_LB],shooting->base_info.fri_info.target_b_speed);
		shooting->base_info.output_friRB=rm_motor_speed_pid_calc(&rm_motor[FRIC_RB],-shooting->base_info.fri_info.target_b_speed);

	

}

/**
 * @brief 摩擦轮速度检查
 * 
 * @param shooting
 */
void Fri_Speed_Check(shooting_t *shooting)
{
	int16_t LF_speed = abs(shooting->frictionLF->info->speed);
	int16_t RF_speed = abs(shooting->frictionRF->info->speed);
	int16_t LB_speed = abs(shooting->frictionLB->info->speed);
	int16_t RB_speed = abs(shooting->frictionRB->info->speed);

	shooting->base_info.fri_info.friLF_speed = LF_speed;
	shooting->base_info.fri_info.friLB_speed = LB_speed;
	shooting->base_info.fri_info.friRF_speed = RF_speed;
	shooting->base_info.fri_info.friRB_speed = RB_speed;

	int16_t LF_error = abs(LF_speed - shooting->config->target_f_speed);
	int16_t RF_error = abs(RF_speed - shooting->config->target_f_speed);
	int16_t LB_error = abs(LB_speed - shooting->config->target_b_speed);
	int16_t RB_error = abs(RB_speed - shooting->config->target_b_speed);

//	if(SHOOTING_MOTOR_OFFLINE)//电机不在线
//	{
//		shooting->base_info.fri_info.fri_speed_state = false;
//		return;
//	}

	if (LF_speed < 1000 || RF_speed < 1000 || LB_speed < 1000 || RB_speed < 1000)//防止摩擦轮转速太低
	{
		shooting->base_info.fri_info.fri_speed_state = false;
		return;
	}

	if (LF_error <= 100 && RF_error <= 100 && LB_error <= 100 && RB_error <= 100)//防止误差太大
	{
		shooting->base_info.fri_info.fri_speed_state = true;
	}
//	else
//	{
//		shooting->base_info.fri_info.fri_speed_state = false;
//	}	

}
/**
 * @brief 左右摩擦轮极低值计算
 * 
 * @param shooting 
 */

void Shooting_Extreme_Low_Value_Calculation(shooting_t *shooting)
{
	static uint8_t last_shooting_cmd_value = 0;
	static uint8_t start_flag = 0;
	//收到打弹命令，赋初始值
	if (last_shooting_cmd_value == 0 && shooting->cmd_fire->cmd_value == 1)
	{
		shooting->base_info.fri_info.friLB_minimum_speed = shooting->frictionLB->info->speed;
		shooting->base_info.fri_info.friRB_minimum_speed = shooting->frictionRB->info->speed;
		shooting->base_info.fri_info.friLF_minimum_speed = shooting->frictionLF->info->speed;
		shooting->base_info.fri_info.friRF_minimum_speed = shooting->frictionRF->info->speed;
		start_flag = 1;
	}
	last_shooting_cmd_value = shooting->cmd_fire->cmd_value;
	//开始计算
	if (start_flag == 1)
	{
		//更新最小值
		if (abs(shooting->frictionLB->info->speed) < abs(shooting->base_info.fri_info.friLB_minimum_speed))
		{
			shooting->base_info.fri_info.friLB_minimum_speed = abs(shooting->frictionLB->info->speed);
		}
		if (abs(shooting->frictionRB->info->speed) < abs(shooting->base_info.fri_info.friRB_minimum_speed))
		{
			shooting->base_info.fri_info.friRB_minimum_speed = abs(shooting->frictionRB->info->speed);
		}
		if (abs(shooting->frictionLF->info->speed) < abs(shooting->base_info.fri_info.friLF_minimum_speed))
		{
			shooting->base_info.fri_info.friLF_minimum_speed = abs(shooting->frictionLF->info->speed);
		}
		if (abs(shooting->frictionRF->info->speed) < abs(shooting->base_info.fri_info.friRF_minimum_speed))
		{
			shooting->base_info.fri_info.friRF_minimum_speed = abs(shooting->frictionRF->info->speed);
		}	
	}
	//结束计算
	static float last_bullet_speed = 0;
	if (last_bullet_speed != communicate.shoot_data_rx_info->shooting_speed)
	{
		shooting->base_info.fri_info.fri_F_diff_speed = shooting->base_info.fri_info.friLF_minimum_speed - shooting->base_info.fri_info.friRF_minimum_speed;
		shooting->base_info.fri_info.fri_B_diff_speed = shooting->base_info.fri_info.friLB_minimum_speed - shooting->base_info.fri_info.friRB_minimum_speed;
		start_flag = 0;
	}
	last_bullet_speed = communicate.shoot_data_rx_info->shooting_speed;
}
/**
 * @brief 热量限制检查
 * 
 * @return true 可以打
 * @return false 打你?
 */
bool Check_Cooling_Heat(void)
{
	uint16_t cooling_limit = communicate.game_robot_status_rx_info->shooter_cooling_limit;
	uint16_t cooling_heat  = communicate.power_heat_data_rx_info->shooter_cooling_heat;
	//自爆直接返回true
	if(shooting.cmd_kill_myself->cmd_value == true)
	{
		return true;
	}
	//判断裁判系统是否在线
	if(communicate.status->work_state == DEV_ONLINE)
	{
		if (cooling_limit - cooling_heat >= 110)//剩余热量大于110
		{
			return true;//可以打
		}
		else
		{
			return false;//会超热量
		}
	}
	else
	{
		return true;//裁判系统离线或下主控没发
	}
}

/**
 * @brief 弹速自适应
 * @note  接受裁判系统中断中调用
 * 
 * @param shooting 
 */
void Shooting_Fri_Speed_Adapt(shooting_t *shooting)
{
	
	/*用户定义参数**********************************************************/
	#define SPEED_SAVE_NUM 2          //速度保存个数
	const float add_kp   = 4.f;       //增加增益
	const float minus_kp = 6.f;       //减少增益
	const float over_blind_err = 0.1; //超过多少内不调整
	const float less_blind_err = 0.1; //低于多少内不调整
	const float max_adapt_range = 30; //最大调整量

	/*函数变量**************************************************************/
	static float last_speed[SPEED_SAVE_NUM]={0}; //保存上一发速度数组
	float now_speed = communicate.shoot_data_rx_info->shooting_speed; //当前速度
	uint8_t over_cnt = 0, less_cnt = 0; // 大于目标速度计数，小于目标速度计数
		
	/*执行弹速调整的条件*****************************************************/
	#ifndef FriSpeedAdaptEnabled
		return;
	#endif
	if(shooting->shooting_state != SHOOTING_RESET_OK)//发射未初始化
		if(shooting->base_info.fri_info.target_b_speed == 0)//摩擦轮目标速度为0
			if(shooting->base_info.fri_info.target_f_speed == 0)
				if(abs(communicate.shoot_data_rx_info->shooting_speed - shooting->config->target_bullet_speed) > 3)//收到数据过于离谱
	{
		return;
	}
	/*计算目前存储数组里弹速的情况******************************************/
	for(uint8_t i = 0; i < SPEED_SAVE_NUM; i++) 
	{
		if(last_speed[i] == 0) 
		{
			// 如果找到一个元素为零，跳出循环
			continue;
		}
		else if(last_speed[i] > shooting->config->target_bullet_speed) 
		{
			over_cnt++;
		}
		else if(shooting->config->target_bullet_speed > last_speed[i]) 
		{
			less_cnt++;
		}
	}

	/*根据情况调整摩擦轮速度***********************************************/
	if(now_speed - shooting->config->target_bullet_speed > over_blind_err)//速度大于目标速度
	{
		if(over_cnt * minus_kp > max_adapt_range) return;
		shooting->config->target_f_speed -= over_cnt * minus_kp;
		shooting->config->target_b_speed -= over_cnt * minus_kp;
	}
	else if(shooting->config->target_bullet_speed - now_speed > less_blind_err)//速度小于目标速度
	{
		if(less_cnt * add_kp > max_adapt_range) return;
		shooting->config->target_f_speed += less_cnt * add_kp;
		shooting->config->target_b_speed += less_cnt * add_kp;
	}
	/*保存当前速度到数组**************************************************/
	for(uint8_t i = 1; i < SPEED_SAVE_NUM; i++) 
	{
		last_speed[i] = last_speed[i - 1];
	}
	last_speed[0] = now_speed;
}

/**
 * @brief 发射命令执行
 * @param shooting 
 */

void Shooting_Command_Execute(shooting_t *shooting)
{
//	static uint32_t limit_get_shoot_command_time;
//	uint32_t limit_shoot_real_time=HAL_GetTick();
//	//限位停转时间判断
//	if(limit_shoot_real_time>limit_get_shoot_command_time+LIMIT_SHOOT_TIME)
//	{
//		shooting->base_info.limit_info.target_speed = 0;
//	}
	if (shooting->shooting_state == SHOOTING_RESET_OK && 
			shooting->base_info.fri_info.fri_speed_state == true)  //且摩擦轮速度正常  
	{
		/*单发命令*********************************************************************/
		//正常模式下为边沿型，视觉模式下会转换cmd_fire的条件为电平型而不是边沿型
		if(shooting->cmd_fire->cmd_value == true &&	  
				shooting->load_state == load_OK        )   //收到打弹命令并且装弹完成
		{
//			limit_get_shoot_command_time=HAL_GetTick();
			shooting->base_info.limit_info.target_position =shooting->limit->info->angle_sum + LIMIT_POSIT_VALUE;
			shooting->base_info.limit_info.ctrl_mode = LIMIT_POSITION_MODE	;
			Shooting_Cmd_Excute_Tick_Calculating(0);//命令开始执行,开始计时,用于发给视觉发弹时间
			shooting->load_state = load_NO;//单发命令后需要装弹
		}
		/*连发命令*********************************************************************/
		if(shooting->cmd_firing->cmd_value == true )
		{
			shooting->base_info.limit_info.ctrl_mode = LIMIT_SPEED_MODE;//限位切换到速度环
			shooting->base_info.limit_info.target_speed = LIMIT_SPEED;//设置限位速度
			shooting->load_state = load_NO;//一直补弹
			shooting->cmd_firing->s_run(shooting->cmd_firing);//切换命令状态
		}
		else if(shooting->cmd_firing->cmd_status == RUNING_C)
		{
			shooting->base_info.limit_info.target_speed = 0;//限位速度为0
			shooting->base_info.limit_info.target_position = shooting->limit->info->angle_sum;//设置限位目标位置
			shooting->base_info.limit_info.ctrl_mode = LIMIT_POSITION_MODE;//限位切换到位置环
			shooting->cmd_firing->s_finish(shooting->cmd_firing);//命令执行完成
		}
	}
	else if(shooting->cmd_fire->cmd_value == true || shooting->cmd_firing->cmd_value == true)//发射初始化未完成或摩擦轮速度不正常或热量不允许打弹
	{
		shooting->cmd_fire->s_finish(shooting->cmd_fire);//单发命令退出
		shooting->cmd_firing->s_finish(shooting->cmd_firing);//连发命令退出
		shooting->base_info.limit_info.target_speed = 0;//限位速度为0
		shooting->base_info.limit_info.target_position = shooting->limit->info->angle_sum;//设置限位目标位置
		shooting->base_info.limit_info.ctrl_mode = LIMIT_POSITION_MODE;//限位切换到位置环
	}
	/*按下G开关摩擦轮********************/

	if(rc_sensor.info->G.status==release_to_press&&shooting->stop_shooting_flag==1) //第二次按下开转
	{
		shooting->stop_shooting_flag=0;
	}
	else if(rc_sensor.info->G.status==release_to_press)  //第一次按下设停转标志位
	{
		shooting->stop_shooting_flag=1;
	}
}

/**
 * @brief 发射总控
 * 
 * @param shooting 
 */
void Shooting_Work(shooting_t *shooting)
{
	//摩擦轮速度检查
	Fri_Speed_Check(shooting);
	//极低值计算，用于debug
	Shooting_Extreme_Low_Value_Calculation(shooting);
	/*视觉模式下发弹命令一直开启，通过视觉方发时间队列用if来启用发弹命令****************/
	switch (car.car_move_mode)
	{
	case offline_CAR:
	case init_CAR:
		shooting->cmd_fire->s_finish(shooting->cmd_fire);//单发命令退出
		shooting->cmd_firing->s_finish(shooting->cmd_firing);//连发命令退出
		break;

	case vision_cycle_CAR:
	case vision_gyro_CAR:
		//不断检查三个数组看看有没有符合打弹的时间
		for(uint8_t i = 0; i<sizeof(vision.timestamp_info->vision_shoot_timing)/sizeof(vision.timestamp_info->vision_shoot_timing[0]); i++)
		{
			int32_t time_err = HAL_GetTick() - vision.timestamp_info->vision_shoot_timing[i]; 
			if(time_err <= VISION_SHOOT_TIMING_TOLERANCE && time_err >= 0)//时间在容差范围内
			{
				Shooting_Command_Execute(shooting);
				break;
			}
		}		
		break;

	default:
		/*非视觉模式下正常执行打弹命令*/
		Shooting_Command_Execute(shooting);
		break;
	}


	/*拨盘，摩擦轮控制***********************************************************************/

	// 关控、发射机构有断电的                   或者收到关闭发射命令          
	if(RC_OFFLINE|| SHOOTING_MOTOR_OFFLINE)
	{
		//全部停止
		shooting->base_info.fri_info.target_f_speed = 0;
		shooting->base_info.fri_info.target_b_speed = 0;
		shooting->base_info.limit_info.target_speed = 0;
		shooting->base_info.dail_info.target_speed  = 0;
		//需要初始化
		shooting->shooting_state = SHOOTING_RESET_NO;
		shooting->load_state = load_NO;
	}
	/*发射初始化流程*******************************************************************************/
	switch (shooting->shooting_state)
	{
	case SHOOTING_RESET_NO:
		shooting->shooting_init_time = 0;
		shooting->base_info.fri_info.target_f_speed = 0;
		shooting->base_info.fri_info.target_b_speed = 0;
		#ifdef OPEN_RC_LOCK
		if (shooting->cmd_fire->cmd_value == true &&car.car_move_mode!=init_CAR&&car.car_move_mode!=offline_CAR)//收到打弹命令
		{
			shooting->shooting_state = SHOOTING_INITING;
		}
		break;
		#else
		if (shooting->cmd_fire->cmd_value == true  )//收到打弹命令
		{
			shooting->shooting_state = SHOOTING_INITING;
		}
		break;
		#endif
		
	case SHOOTING_INITING:
		shooting->base_info.fri_info.target_f_speed = shooting->config->target_f_speed;
		shooting->base_info.fri_info.target_b_speed = shooting->config->target_b_speed;
		shooting->shooting_init_time++;
		//拨盘初始化
		shooting->base_info.dail_info.target_speed = DAIL_Init_SPEED;
		//限位反转
		shooting->base_info.limit_info.target_speed = LIMIT_Init_SPEED;
		shooting->base_info.limit_info.ctrl_mode = LIMIT_SPEED_MODE;
		if (Motor_DetectStuck(shooting->dail) == 1)
		{
			shooting->base_info.dail_info.target_speed = 0;
			shooting->base_info.limit_info.target_speed = 0;
			shooting->base_info.limit_info.target_position=shooting->limit->info->angle_sum;
			shooting->base_info.limit_info.ctrl_mode = LIMIT_POSITION_MODE;
			shooting->shooting_state = SHOOTING_RESET_OK;
			shooting->base_info.dail_info.dail_reset_state = DEV_RESET_OK;
		}
		//退出
		if(shooting->shooting_init_time > 1000 )
		{
			shooting->base_info.dail_info.target_speed = 0;
			shooting->base_info.limit_info.target_speed = 0;
			shooting->base_info.limit_info.target_position=shooting->limit->info->angle_sum;
			shooting->base_info.limit_info.ctrl_mode = LIMIT_POSITION_MODE;
			shooting->shooting_state = SHOOTING_RESET_OK;
			shooting->base_info.dail_info.dail_reset_state = DEV_RESET_OK;
		}
		break;
	case SHOOTING_RESET_OK:
		shooting->base_info.fri_info.target_f_speed = shooting->config->target_f_speed;  // 实时更新弹速
		shooting->base_info.fri_info.target_b_speed = shooting->config->target_b_speed;  
		shooting->shooting_init_time = 0;
		Shooting_Work_Reloading(shooting);   //持续补弹
		break;
	default:
		break;
	}

	/*关控保护**********************************************************************/
	
	if(RC_OFFLINE||rc_sensor.info->s1.value==2||shooting->stop_shooting_flag==1)
	{
		//发射模块输出置零
		shooting->base_info.output_dail  = 0 ;
		shooting->base_info.output_limit = 0 ;
		shooting->base_info.output_friLF = 0 ;
		shooting->base_info.output_friRF = 0 ;
		shooting->base_info.output_friLB = 0 ;
		shooting->base_info.output_friRB = 0 ;
		shooting->base_info.limit_info.ctrl_mode=LIMIT_SPEED_MODE;
		shooting->limit->base_info.motor_out=0;
		shooting->dail->base_info.motor_out=0;
		shooting->frictionLF->base_info.motor_out=rm_motor_speed_pid_calc(&rm_motor[FRIC_LF],0);
		shooting->frictionRF->base_info.motor_out=rm_motor_speed_pid_calc(&rm_motor[FRIC_RF],0);
		shooting->frictionLB->base_info.motor_out=rm_motor_speed_pid_calc(&rm_motor[FRIC_LB],0);
		shooting->frictionRB->base_info.motor_out=rm_motor_speed_pid_calc(&rm_motor[FRIC_RB],0);
		shooting->shooting_state = SHOOTING_RESET_NO;
	}
	else if(RC_ONLINE)//开控并且不关摩擦轮,s1下关摩擦轮
	{
		Shooting_Pid_Calculating(shooting);
		shooting->dail->base_info.motor_out = shooting->base_info.output_dail;
		shooting->limit->base_info.motor_out = shooting->base_info.output_limit;
		shooting->frictionLF->base_info.motor_out = shooting->base_info.output_friLF;
		shooting->frictionRF->base_info.motor_out = shooting->base_info.output_friRF;
		shooting->frictionLB->base_info.motor_out = shooting->base_info.output_friLB;
		shooting->frictionRB->base_info.motor_out = shooting->base_info.output_friRB;
	}

}       

