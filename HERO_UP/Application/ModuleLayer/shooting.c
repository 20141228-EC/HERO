
/* Includes ------------------------------*/
#include "shooting.h"

/* Private function prototypes -----------------------------------------------*/
uint8_t Shooting_Work_Reloading(shooting_t *shooting);			   // 补弹逻辑
void Shooting_Fri_Speed_Adapt(shooting_t *shooting);			   // 弹速自适应
void Shooting_Pid_Calculating(shooting_t *shooting);			   // pid计算
void Fri_Speed_Check(shooting_t *shooting);						   // 摩擦轮速度检查
void Shooting_Command_Execute(shooting_t *shooting);			   // 发射命令执行
void Shooting_Extreme_Low_Value_Calculation(shooting_t *shooting); // 摩擦轮转速极低值计算
bool Check_Cooling_Heat(void);									   // 检查热量是否允许发射
// 摩擦轮速度配置0
shooting_config_t config =
	{	
		/*摩擦轮正常工作转速*/
		#if HERO_TYPE ==3
		.target_F_friction_speed = 4150.f,//4150  5000 15.6弹速
		.target_B_friction_speed = 5000.f,  
		#elif HERO_TYPE ==2
		.target_B_friction_speed = 4000.f, // 4000
		#elif HERO_TYPE ==1
		.target_B_friction_speed = 4150.f, // 4000
		#endif
		/*弹速自适应期望弹速*/
		#if HERO_TYPE ==2
		.target_bullet_speed = 15.6,//更偏向于减速
		#else
		.target_bullet_speed = 15.5,//吊射顶部
		#endif
};
	
shooting_t shooting =
	{	
		//注册电机
		.friction_F_L = &rm_motor[FRIC_F_L],
		.friction_F_R = &rm_motor[FRIC_F_R],
		.friction_F_UP= &rm_motor[FRIC_F_UP],
		.friction_B_L = &rm_motor[FRIC_B_L],
		.friction_B_R = &rm_motor[FRIC_B_R],
		.friction_B_UP= &rm_motor[FRIC_B_UP],
		.dail = &rm_motor[DAIL],
		//命令
		.cmd_fire = &command[SHOOTING_FIRE],
		.cmd_firing = &command[SHOOTING_FIRING],
		.cmd_kill_myself = &command[KILL_MYSELF],
		.cmd_timer_mec_outpost = &command[TIMER_MEC_OUTPOST],
		.cmd_speed_adapt = &command[OPEN_SPEED_ADAPT],
		//发射弹丸速度配置信息
		.config = &config,
		
		//发射消抖
		.shooting_shake_angle.feedforward_delay_time=50,
		.shooting_shake_angle.feedforward_continue_time=200,
		.shooting_shake_angle.const_offset_current=5000,
		.shooting_shake_angle.kd=1,
		//初始状态
		.base_info.dail_info.dail_reset_state = DEV_RESET_NO,
		.shooting_state = SHOOTING_RESET_NO,
		.load_state = load_NO,
		//总控
		.work = Shooting_Work,
};

/**
 * @brief 补弹
 * @param shooting
 * @return uint8_t 补弹完成：1 未完成：0
 */
float angle=33000;
uint8_t Shooting_Work_Reloading(shooting_t *shooting)
{
	int32_t position = shooting->dail->info->angle_sum; // 拨盘现在位置
	int16_t speed = shooting->dail->info->speed;		// 拨盘现在速度
	dail_info_t *info = &shooting->base_info.dail_info;

	uint8_t res = 0;
	float dail_max_work_time; // 拨盘最大工作时间
	/*动态改变拨弹时间************************************************/
	//裁判系统有延迟，即使限热量110才能打，但是实际上也会超，所以控射频
	uint16_t shooter_cooling_limit=communicate.game_robot_status_rx_info->shooter_cooling_limit;
	uint16_t shooter_cooling_heat=communicate.power_heat_data_rx_info->shooter_cooling_heat;
	if (shooter_cooling_limit-shooter_cooling_heat>=350) // 波盘可以拨快一点
	{
		dail_max_work_time = DAIL_FAST_SHOOT_TIMES;
	}
	else // 波盘拨慢一点
	{
		dail_max_work_time = DAIL_SLOW_SHOOT_TIMES;
	}
	/*补弹逻辑**********************************************************/
	switch (info->work_state)
	{
	case DAIL_SLEEP: // 唤醒拨盘，开始补弹
		info->work_times = 0; 
		info->target_speed = 0;
		if (shooting->load_state == load_NO && shooting->base_info.fri_info.fri_speed_state == true) // 需要装弹且摩擦轮速度正常
		{
			/* next */
			info->target_angle_sum += DAIL_ONESHOT_ANGLE; //收到命令的标志位，加一个弹丸的位置
			info->work_state = DAIL_RELOAD;
			info->dail_mode=DAIL_ANGLE;
		}
		break;
	case DAIL_RELOAD:								// 正转补弹
		if (Motor_DetectStuck_userdef(shooting->dail,50,12000,250,1) != 1) // 正转并且堵转
		{
			shooting->base_info.dail_info.stuck_flag=0;
//			if(abs(shooting->dail->info->angle_sum-info->target_angle_sum)<=300)//正转没有堵转&&检测到位
//			{
//				/* break */
//				info->work_times = 0;
//				shooting->load_state = load_OK; //补弹完成
//				.info->work_state =	 DAIL_SLEEP; //返回上一个等待补弹命令状态
//				res = 1;
//			}
			if (info->work_times >= dail_max_work_time) // 正转没有堵转&&检测超时
			{
				/* break */
				info->work_times = 0;
				info->target_speed = 0;
				shooting->load_state = load_OK;
				info->work_state = DAIL_SLEEP;
				res = 1;
			}
			else // 没超时没到位继续工作
			{
				info->work_times++;
			}
		}
		else // 堵转了
		{
			/*注释掉这段堵转情况下不继续拨弹-----------*/
			/* next */
			#ifdef HANDLE_STUCK
			info->target_angle_sum = position - DAIL_ONESHOT_ANGLE; // 卡弹就不拨了，回去
			info->work_times = 0;				
			// 工作时间清零
			info->work_state = DAIL_REVERT;							   // 进入反转模式
			#endif
			/*-----------------------------------*/
			info->work_times++;
			shooting->base_info.dail_info.stuck_flag=1;
		}


		break;
	case DAIL_REVERT:
 
		if (position <= info->target_angle_sum) //没堵转但到位了（没弹）
		{
			/* break */
			info->work_times = 0;
			info->target_speed = 0;
			info->work_state = DAIL_SLEEP;			
			shooting->load_state = load_OK;
			shooting->base_info.dail_info.stuck_flag= 0;
		}
		else if (info->work_times >= dail_max_work_time) // 没堵转但工作超时->供弹完成，退出
		{
			/* break */
			info->work_times = 0;
			info->target_speed = 0;
			shooting->load_state = load_OK;
			info->work_state = DAIL_SLEEP;
			shooting->base_info.dail_info.stuck_flag=0;
			res = 1;
		}
		else // 没堵转也没到位
		{
			info->work_times++;
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
	// 拨盘PID计算
	if(shooting->base_info.dail_info.dail_mode==DAIL_SPEED)
	{
		shooting->base_info.output_dail = rm_motor_speed_pid_calc(&rm_motor[DAIL], shooting->base_info.dail_info.target_speed);
	}
	else
	{
		shooting->base_info.output_dail = rm_motor_anglesum_pid_calc(&rm_motor[DAIL], shooting->base_info.dail_info.target_angle_sum);
	}

	//第二级摩擦轮
	
	#if HERO_TYPE ==3
	shooting->base_info.output_fri_F_UP=rm_motor_speed_pid_calc(shooting->friction_F_UP,-shooting->base_info.fri_info.target_fri_F_UP_speed);
	shooting->base_info.output_fri_F_L=rm_motor_speed_pid_calc(shooting->friction_F_L,shooting->base_info.fri_info.target_fri_F_L_speed);
	shooting->base_info.output_fri_F_R=rm_motor_speed_pid_calc(shooting->friction_F_R,-shooting->base_info.fri_info.target_fri_F_R_speed);
	#endif
	//第一级摩擦轮
	shooting->base_info.output_fri_B_UP=rm_motor_speed_pid_calc(shooting->friction_B_UP,-shooting->base_info.fri_info.target_fri_B_UP_speed);
	shooting->base_info.output_fri_B_L=rm_motor_speed_pid_calc(shooting->friction_B_L,shooting->base_info.fri_info.target_fri_B_L_speed);
	shooting->base_info.output_fri_B_R=rm_motor_speed_pid_calc(shooting->friction_B_R,-shooting->base_info.fri_info.target_fri_B_R_speed);
}

/**
 * @brief 摩擦轮速度检查
 *
 * @param shooting
 */
void Fri_Speed_Check(shooting_t *shooting)
{
	shooting->base_info.fri_info.fri_B_L_real_speed =  abs(shooting->friction_B_L->info->speed);
	shooting->base_info.fri_info.fri_B_R_real_speed =  abs(shooting->friction_B_R->info->speed);
	shooting->base_info.fri_info.fri_B_UP_real_speed = abs(shooting->friction_B_UP->info->speed);
	shooting->base_info.fri_info.fri_F_L_real_speed =  abs(shooting->friction_F_L->info->speed);
	shooting->base_info.fri_info.fri_F_R_real_speed =  abs(shooting->friction_F_R->info->speed);
	shooting->base_info.fri_info.fri_F_UP_real_speed = abs(shooting->friction_F_UP->info->speed);
	
	int16_t B_L_error = abs(shooting->base_info.fri_info.fri_B_L_real_speed - shooting->config->target_B_friction_speed);
	int16_t B_R_error = abs(shooting->base_info.fri_info.fri_B_R_real_speed - shooting->config->target_B_friction_speed);
	int16_t B_UP_error = abs(shooting->base_info.fri_info.fri_B_UP_real_speed - shooting->config->target_B_friction_speed);
	int16_t F_L_error = abs(shooting->base_info.fri_info.fri_F_L_real_speed - shooting->config->target_F_friction_speed);
	int16_t F_R_error = abs(shooting->base_info.fri_info.fri_F_R_real_speed - shooting->config->target_F_friction_speed);
	int16_t F_UP_error =abs(shooting->base_info.fri_info.fri_F_UP_real_speed- shooting->config->target_F_friction_speed);
	if(!SHOOTING_MOTOR_ONLINE)//电机不在线
	{
		shooting->base_info.fri_info.fri_speed_state = false;
		return;
	}
	#if HERO_TYPE ==3
	if (shooting->base_info.fri_info.fri_B_L_real_speed < 1000 || shooting->base_info.fri_info.fri_B_R_real_speed < 1000 || shooting->base_info.fri_info.fri_B_UP_real_speed < 1000 \
		||shooting->base_info.fri_info.fri_F_L_real_speed < 1000 || shooting->base_info.fri_info.fri_F_R_real_speed < 1000 || shooting->base_info.fri_info.fri_F_UP_real_speed < 1000)//防止摩擦轮转速太低
	{
		shooting->base_info.fri_info.fri_speed_state = false;
		return;
	}
	#else 
	if (shooting->base_info.fri_info.fri_B_L_real_speed < 1000 || shooting->base_info.fri_info.fri_B_R_real_speed < 1000 || shooting->base_info.fri_info.fri_B_UP_real_speed < 1000 )//防止摩擦轮转速太低
	{
		shooting->base_info.fri_info.fri_speed_state = false;
		return;
	}
	#endif
	if (B_L_error <= 100 && B_R_error <= 100 && B_UP_error <= 100)//防止误差太大
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
	// 收到打弹命令，赋初始值
	if (last_shooting_cmd_value == 0 && shooting->cmd_fire->cmd_value == 1)
	{
		shooting->base_info.fri_info.fri_B_L_minimum_speed =  shooting->friction_B_L->info->speed;
		shooting->base_info.fri_info.fri_B_R_minimum_speed =  shooting->friction_B_R->info->speed;
		shooting->base_info.fri_info.fri_B_UP_minimum_speed = shooting->friction_B_UP->info->speed;
		shooting->base_info.fri_info.fri_F_L_minimum_speed =  shooting->friction_F_L->info->speed;
		shooting->base_info.fri_info.fri_F_R_minimum_speed =  shooting->friction_F_R->info->speed;
		shooting->base_info.fri_info.fri_F_UP_minimum_speed = shooting->friction_F_UP->info->speed;
		start_flag = 1;
	}
	last_shooting_cmd_value = shooting->cmd_fire->cmd_value;
	// 开始计算
	if (start_flag == 1)
	{
		// 更新最小值
		if (abs(shooting->friction_B_L->info->speed) < abs(shooting->base_info.fri_info.fri_B_L_minimum_speed))
		{
			shooting->base_info.fri_info.fri_B_L_minimum_speed = abs(shooting->friction_B_L->info->speed);
		}
		if (abs(shooting->friction_B_R->info->speed) < abs(shooting->base_info.fri_info.fri_B_R_minimum_speed))
		{
			shooting->base_info.fri_info.fri_B_R_minimum_speed = abs(shooting->friction_B_R->info->speed);
		}
		if (abs(shooting->friction_B_UP->info->speed) < abs(shooting->base_info.fri_info.fri_B_UP_minimum_speed))
		{
			shooting->base_info.fri_info.fri_B_UP_minimum_speed = abs(shooting->friction_B_UP->info->speed);
		}
		if (abs(shooting->friction_F_L->info->speed) < abs(shooting->base_info.fri_info.fri_F_L_minimum_speed))
		{
			shooting->base_info.fri_info.fri_F_L_minimum_speed = abs(shooting->friction_F_L->info->speed);
		}
		if (abs(shooting->friction_F_R->info->speed) < abs(shooting->base_info.fri_info.fri_F_R_minimum_speed))
		{
			shooting->base_info.fri_info.fri_F_R_minimum_speed = abs(shooting->friction_F_R->info->speed);
		}
		if (abs(shooting->friction_F_UP->info->speed) < abs(shooting->base_info.fri_info.fri_F_UP_minimum_speed))
		{
			shooting->base_info.fri_info.fri_F_UP_minimum_speed = abs(shooting->friction_F_UP->info->speed);
		}
		
	}
	// 结束计算
	static float last_bullet_speed = 0;
	if (last_bullet_speed != communicate.shoot_data_rx_info->shooting_speed)
	{
		start_flag = 0;
	}
	last_bullet_speed = communicate.shoot_data_rx_info->shooting_speed;
}
/**
 * @brief 热量、击打限制检查
 *
 * @return true 可以打
 */
bool Check_Cooling_Heat(void)
{
	
	uint16_t cooling_limit = communicate.game_robot_status_rx_info->shooter_cooling_limit;
	uint16_t cooling_heat = communicate.power_heat_data_rx_info->shooter_cooling_heat;
	//按下G可以射出最后一发
	if(shooting.cmd_kill_myself->cmd_value==true&&cooling_limit - cooling_heat >= 120)
	{
		return true; // 可以打
	}
	
		/*联盟赛特殊情况******牢*牢*牢*牢*牢*牢*牢*牢*牢*牢*/
		if(cooling_limit==100&&cooling_heat==0)//一级时只能等热量清空才能射下一发
		{
			return true; // 可以打
		}
		//①热量够②在游戏中（不在游戏中默认为0）③并且不是最后一发单（最后一发单打出摩擦轮会停转）
		else if (cooling_limit - cooling_heat >= 120&&communicate.game_robot_status_rx_info->game_process.bit.is_the_last_bullet!=1) // 剩余热量大于110
		{
			return true; // 可以打
		}
		else
		{
			return false; // 会超热量，不打
		}
	
}

/**
 * @brief 弹速自适应
 * @note  接受裁判系统中断中调用
 *
 * @param shooting
 */
uint8_t flag;
void Shooting_Fri_Speed_Adapt(shooting_t *shooting)
{
	
/*用户定义参数**********************************************************/

#define SPEED_SAVE_NUM 2			  // 速度保存个数
	const float add_kp = 8.f;		  // 增加增益
	const float minus_kp = 8.f;		  // 减少增益
	#if HERO_TYPE==2
	const float over_blind_err = 0.2; // 超过多少内不调整
	#else
	const float over_blind_err = 0.3; // 超过多少内不调整
	#endif
	
	const float less_blind_err = 0.2; // 低于多少内不调整
	const float max_adapt_range = 100; // 最大单次调整量

	/*函数变量**************************************************************/
	static uint8_t normal_speed_flag;	

	static float last_speed[SPEED_SAVE_NUM] = {0};					  // 保存上一发速度数组
	float now_speed = communicate.shoot_data_rx_info->shooting_speed; // 当前速度

	uint8_t over_cnt = 0, less_cnt = 0;								  // 大于目标速度计数，小于目标速度计数
	
    /*执行弹速调整的条件*****************************************************/
	#if HERO_TYPE==3
		#ifdef Z_CHANGE_FRIC_SPEED
			return ;
		#endif
	#endif
	
	if(communicate.car_data0_tx_info->car_state.bit.is_open_adapt==0)
	{
		
		return ;
	}
#ifndef FriSpeedAdaptEnabled
	return;
#endif
	if (shooting->shooting_state != SHOOTING_RESET_OK)		  // 发射未初始化
		if (shooting->base_info.fri_info.target_fri_B_L_speed == 0) // 摩擦轮目标速度为0
				if (abs(communicate.shoot_data_rx_info->shooting_speed - shooting->config->target_bullet_speed) > 3) // 收到数据过于离谱
				{
					return;
				}
	//超弹速！！！大量下降
	if(now_speed>16.f)
	{
		shooting->config->target_B_friction_speed -= 40;
		shooting->config->target_F_friction_speed -= 40;
		return;
	}
	
	/*计算目前存储数组里弹速的情况******************************************/
	for (uint8_t i = 0; i < SPEED_SAVE_NUM; i++)
	{
		if (last_speed[i] == 0)
		{
			// 如果找到一个元素为零，跳出循环
			continue;
		}
		else if (last_speed[i] > shooting->config->target_bullet_speed)
		{
			over_cnt++;
		}
		else if (shooting->config->target_bullet_speed > last_speed[i])
		{
			less_cnt++;
		}
	}

	/*根据情况调整摩擦轮速度***********************************************/
	//施密特触发器
	if (now_speed - shooting->config->target_bullet_speed > over_blind_err) // 速度大于目标速度
	{
		if (over_cnt * minus_kp > max_adapt_range)//限幅
			return;
		shooting->config->target_B_friction_speed -= over_cnt * minus_kp;
		shooting->config->target_F_friction_speed -= over_cnt * minus_kp;
	}
 
	else if (shooting->config->target_bullet_speed - now_speed > less_blind_err) // 速度小于目标速度
	{
		if (less_cnt * add_kp > max_adapt_range||normal_speed_flag==1)
			return;
		if(less_cnt>=2)//数组里面两个都低于弹速才提高弹速
		{
			shooting->config->target_B_friction_speed += less_cnt * add_kp;
			shooting->config->target_F_friction_speed += less_cnt * add_kp;
		}
		
	}

	/*保存当前速度到数组**************************************************/
	for (uint8_t i = 1; i < SPEED_SAVE_NUM; i++)
	{
		last_speed[i] = last_speed[i - 1];
	}
	last_speed[0] = now_speed;
}
 
/**
 * @brief 发射命令执行
 * @param shooting
 */
uint32_t timer_mec_outpost_tick;//手打前哨时间戳
void Shooting_Command_Execute(shooting_t *shooting)
{
	
	timer_mec_outpost_tick++;
	#ifdef Check_Heat
	if (shooting->shooting_state == SHOOTING_RESET_OK &&
		shooting->base_info.fri_info.fri_speed_state == true 
		&&Check_Cooling_Heat()==true) // ①初始化完成 ②摩擦轮速度正常 ③热量允许发射
	#else
	if (shooting->shooting_state == SHOOTING_RESET_OK &&
		shooting->base_info.fri_info.fri_speed_state == true ) // ①初始化完成 ②摩擦轮速度正常 ③热量允许发射
	#endif
	{
		/*单发命令*********************************************************************/
		// 正常模式下为边沿型，视觉模式下会转换cmd_fire的条件为电平型而不是边沿型
		if (shooting->cmd_fire->cmd_value == true &&
			shooting->load_state == load_OK&&
			shooting->cmd_timer_mec_outpost->cmd_value!=true) // ①收到打弹命令②装弹完成③不在触发手打前哨命令
		{
			Shooting_Cmd_Excute_Tick_Calculating(0); // 命令开始执行,开始计时,用于计算发弹时间
			shooting->load_state = load_NO;			 // 发标志位给补弹程序
			//每次普通发射都重置时间
			timer_mec_outpost_tick=0;	
		}
		/*连发命令*********************************************************************/
		if (shooting->cmd_firing->cmd_value == true &&
			shooting->load_state == load_OK &&
			shooting->cmd_timer_mec_outpost->cmd_value!=true) // 收到连发命令并且装弹完成
		{
			static uint32_t last_shoot_tick;
			if(HAL_GetTick()-last_shoot_tick>FIRING_PERIOD)
			{
				Shooting_Cmd_Excute_Tick_Calculating(0); // 命令开始执行,开始计时,用于计算发弹时间
				shooting->load_state = load_NO;			 // 发标志位给补弹程序
				last_shoot_tick=HAL_GetTick();
				//每次普通发射都重置时间
				timer_mec_outpost_tick=0;
			}
		}
		/*手打前哨命令****************************************************************/
		if(shooting->cmd_timer_mec_outpost->cmd_value==true&&
			shooting->load_state == load_OK)
		{
			if(timer_mec_outpost_tick%833==0)
			{
				Shooting_Cmd_Excute_Tick_Calculating(0); // 命令开始执行,开始计时,用于计算发弹时间
				shooting->load_state = load_NO;			 // 发标志位给补弹程序
			}
		}
	}
	//不符合发射条件取消命令
	else if (shooting->cmd_fire->cmd_value == true || shooting->cmd_firing->cmd_value == true) // 发射初始化未完成或摩擦轮速度不正常或热量不允许打弹
	{
		shooting->cmd_fire->s_finish(shooting->cmd_fire);								   // 单发命令退出
		shooting->cmd_firing->s_finish(shooting->cmd_firing);							   // 连发命令退出
	}
	if(shooting->cmd_speed_adapt->cmd_value==1&&communicate.car_data0_tx_info->car_state.bit.is_open_adapt==0)
	{
		communicate.car_data0_tx_info->car_state.bit.is_open_adapt=1;
	}
	else if(shooting->cmd_speed_adapt->cmd_value==1 &&communicate.car_data0_tx_info->car_state.bit.is_open_adapt==1)
	{
		communicate.car_data0_tx_info->car_state.bit.is_open_adapt=0;
	}
}
/**
*@brief 计算发射最大抖动角度
*
*/
void Shooting_max_shake_angle_calc(shooting_t *shooting)
{
	static uint32_t last_calc_tick=9999999;
	static float shoot_moment_pitch;
	static float shoot_moment_yaw;
	
	if(shooting->cmd_fire->cmd_value==true&&shooting->shooting_state==SHOOTING_RESET_OK \
		&&shooting->base_info.fri_info.fri_speed_state == true)
	{
		shoot_moment_pitch=gimbal.base_info.pitch_mec_360_angle;
		shoot_moment_yaw=gimbal.base_info.yaw_mec_360_angle;
		last_calc_tick=HAL_GetTick();
	}
	if(HAL_GetTick()-last_calc_tick<=300)
	{
		if(abs(gimbal.base_info.pitch_mec_360_angle-shoot_moment_pitch)>shooting->shooting_shake_angle.pitch_shake_angle)
		{
			shooting->shooting_shake_angle.pitch_shake_angle=abs(gimbal.base_info.pitch_mec_360_angle-shoot_moment_pitch);
		}
		if(abs(gimbal.base_info.yaw_mec_360_angle-shoot_moment_yaw)>shooting->shooting_shake_angle.yaw_shake_angle)
		{
			shooting->shooting_shake_angle.yaw_shake_angle=abs(gimbal.base_info.yaw_mec_360_angle-shoot_moment_yaw);
		}
	}
}
/**
*@brief 发射pitch抖动补偿
*@note  需要放在pitch的pid计算之前
*@author LYQ
*/
void Shoot_pitch_shake_restrict(shooting_t *shooting)
{
	#define CONSTANT_FEEDFORWARD_CURRENT
	
	static uint32_t get_fire_tick=999999999;//防止第一次进入就触发一次
	uint16_t delay_time=shooting->shooting_shake_angle.feedforward_delay_time;//接受命令到开始加力的时间
	uint16_t continue_time=shooting->shooting_shake_angle.feedforward_continue_time;//开始加力到结束加力的时间
	 
	
	if( shooting->cmd_fire->cmd_value == true &&
		shooting->load_state == load_OK&&shooting->shooting_state==SHOOTING_RESET_OK 
		&&shooting->base_info.fri_info.fri_speed_state == true)
		
	{
		get_fire_tick=HAL_GetTick();
	}
	if(HAL_GetTick()-get_fire_tick>=delay_time&&HAL_GetTick()-get_fire_tick<delay_time+continue_time)//开始前馈
	{
		#ifdef CONSTANT_FEEDFORWARD_CURRENT//前馈控制，前馈电流为常数
		shooting->shooting_shake_angle.shoot_pitch_offset_current=shooting->shooting_shake_angle.const_offset_current;
		#else //反馈控制，反馈电流为电机速度微分项 
	
		//计算微分项输出
		static int16_t last_pitch_speed;
		shooting->shooting_shake_angle.pitch_a=gimbal.gimbal_p->info->speed-last_pitch_speed;
		shooting->shooting_shake_angle.shoot_pitch_offset_current=shooting->shooting_shake_angle.pitch_a*shooting->shooting_shake_angle.kd;
		last_pitch_speed=gimbal.gimbal_p->info->speed;
		#endif
		shooting->shooting_shake_angle.feedforward_current_flag=1;
	}
	if(HAL_GetTick()-get_fire_tick>=delay_time+continue_time)//结束前馈，电流置零
	{
		shooting->shooting_shake_angle.shoot_pitch_offset_current=0;
		shooting->shooting_shake_angle.feedforward_current_flag=0;
	}
	
}
/**
 * @brief 发射总控
 *
 * @param shooting
 */
uint8_t stop_auto_fric_speed_flag;
uint8_t fric_speed_type;//0低转速 1高转速
void Shooting_Work(shooting_t *shooting)
{
	
	#if HERO_TYPE==3
	#ifdef Z_CHANGE_FRIC_SPEED//基地装甲板开就升高弹速
	if(rc_sensor.info->Z.status==release_to_press)
	{
		stop_auto_fric_speed_flag=1;
			if(fric_speed_type==0)
			{
				fric_speed_type=1;
			}
			else if(fric_speed_type==1)
			{
				fric_speed_type=0;
			}
	}
	
	if(stop_auto_fric_speed_flag==0&&car.car_move_mode==lob_CAR&& \
		communicate.game_robot_status_rx_info->game_process.bit.is_outpost_done==1&&   
		communicate.game_robot_status_rx_info->game_process.bit.is_base_open==0)//前哨爆了&&吊射模式&&基地没开花=吊射顶部
	{
		fric_speed_type=0;
	}
	else if(stop_auto_fric_speed_flag==0)
	{
		fric_speed_type=1;
	}
	if(fric_speed_type==1)
	{
		shooting->config->target_F_friction_speed=4150.f;
		shooting->config->target_B_friction_speed=5000.f;
	}
	else if(fric_speed_type==0)
	{
		shooting->config->target_F_friction_speed=3850.f;
		shooting->config->target_B_friction_speed=4650.f;
	}
	#endif
	
	
	
	#endif
	// 摩擦轮速度检查
	Fri_Speed_Check(shooting);
	// 极低值计算，用于debug
	Shooting_Extreme_Low_Value_Calculation(shooting);
	//计算最大抖动
	Shooting_max_shake_angle_calc(shooting);
	/*视觉模式下发弹命令一直开启，通过视觉方发时间队列用if来启用发弹命令****************/
	switch (car.car_move_mode)
	{
	case offline_CAR:
	case init_CAR:
		shooting->cmd_fire->s_finish(shooting->cmd_fire);	  // 单发命令退出
		shooting->cmd_firing->s_finish(shooting->cmd_firing); // 连发命令退出
		break;

	case vision_cycle_CAR:
	case vision_gyro_CAR:
		// 不断检查三个数组看看有没有符合打弹的时间
		timer_mec_outpost_tick++;//计时
		for (uint8_t i = 0; i < sizeof(vision.timestamp_info->vision_shoot_timing) / sizeof(vision.timestamp_info->vision_shoot_timing[0]); i++)
		{
			int32_t time_err = HAL_GetTick() - vision.timestamp_info->vision_shoot_timing[i];
			if (time_err <= VISION_SHOOT_TIMING_TOLERANCE && time_err >= 0) // 时间在容差范围内
			{
#ifdef VISION_SHOOT_FRE
				static uint32_t last_vision_shoot_tick;
				if (HAL_GetTick() - last_vision_shoot_tick >= VISION_SHOOT_FRE)
				{
					timer_mec_outpost_tick--;//计时
					Shooting_Command_Execute(shooting);
					last_vision_shoot_tick = HAL_GetTick();
				}
#else
				Shooting_Command_Execute(shooting);
#endif
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
	
	// 关控  或者收到关闭发射命令   裁判系统断电不需要重新初始化
	if (RC_OFFLINE||rc_sensor.info->s1.status==down_R)
	{
		// 全部停止
		shooting->dail->base_info.motor_out=0;
		//要重新初始化
		shooting->shooting_state = SHOOTING_RESET_NO;
		shooting->base_info.dail_info.dail_reset_state=DEV_RESET_NO;
		//防止重新开控第一发就拨弹
		shooting->load_state = load_OK;

	}
	/*发射初始化流程*******************************************************************************/
	switch (shooting->shooting_state)
	{
	case SHOOTING_RESET_NO:
		shooting->shooting_init_time = 0;
#ifdef OPEN_RC_LOCK
		if (shooting->cmd_fire->cmd_value == true && Fcar.car_move_mode != init_CAR && car.car_move_mode != offline_CAR) // 收到打弹命令
		{
			shooting->shooting_state = SHOOTING_INITING;
		}
		break;
#else
		if (shooting->cmd_fire->cmd_value == true|| \
			shooting->cmd_firing->cmd_value == true) // 收到打弹命令
		{
			/*next*/
			shooting->shooting_state = SHOOTING_INITING;
		}
		break;
#endif

	case SHOOTING_INITING:
		//摩擦轮开转
		
		shooting->base_info.fri_info.target_fri_F_UP_speed = shooting->config->target_F_friction_speed;
		shooting->base_info.fri_info.target_fri_F_L_speed  = shooting->config->target_F_friction_speed;
		shooting->base_info.fri_info.target_fri_F_R_speed  = shooting->config->target_F_friction_speed;
		shooting->base_info.fri_info.target_fri_B_UP_speed = shooting->config->target_B_friction_speed;
		shooting->base_info.fri_info.target_fri_B_L_speed  = shooting->config->target_B_friction_speed;
		shooting->base_info.fri_info.target_fri_B_R_speed  = shooting->config->target_B_friction_speed;

		shooting->shooting_init_time++;
		// 拨盘初始化
		shooting->base_info.dail_info.dail_mode=DAIL_SPEED;
		shooting->base_info.dail_info.target_speed = DAIL_Init_SPEED;
		// 限位反转
		if (Motor_DetectStuck(shooting->dail) == 1)
		{
			//切换拨盘pid模式
			shooting->base_info.dail_info.target_speed = 0;
			shooting->base_info.dail_info.dail_mode=DAIL_ANGLE;
			shooting->base_info.dail_info.target_angle_sum=shooting->dail->info->angle_sum+DAIL_INIT_ANGLE;

			/*next*/
			shooting->shooting_state = SHOOTING_RESET_OK;
			shooting->base_info.dail_info.dail_reset_state = DEV_RESET_OK;
		}
		//超时
		if (shooting->shooting_init_time > 1000)
		{
			//切换拨盘pid模式
			shooting->base_info.dail_info.target_speed = 0;
			shooting->base_info.dail_info.dail_mode=DAIL_ANGLE;
			shooting->base_info.dail_info.target_angle_sum=shooting->dail->info->angle_sum+DAIL_INIT_ANGLE;

			/*next*/
			shooting->shooting_state = SHOOTING_RESET_OK;
			shooting->base_info.dail_info.dail_reset_state = DEV_RESET_OK;
		}
		break;
	case SHOOTING_RESET_OK:
		//发射复位完摩擦轮也是用这个速度转
		#if HERO_TYPE ==3
		shooting->base_info.fri_info.target_fri_F_UP_speed = shooting->config->target_F_friction_speed+0;
		shooting->base_info.fri_info.target_fri_F_L_speed  = shooting->config->target_F_friction_speed+0;
		shooting->base_info.fri_info.target_fri_F_R_speed  = shooting->config->target_F_friction_speed+0;
		#endif
		
		//复位初始化计时
		shooting->shooting_init_time = 0;
		
		//只在不堵转的时候堵转检测
		if( (Motor_DetectStuck_userdef(shooting->friction_B_L,100,5000,100,0)==1&& \
			Motor_DetectStuck_userdef(shooting->friction_B_R,100,5000,100,0)==1 && \
			Motor_DetectStuck_userdef(shooting->friction_B_UP,100,5000,100,0)==1)&& \
		   shooting->base_info.fri_info.fri_handle_stuck_flag==0)
		{
			//已在pid输出里修正方向
			shooting->base_info.fri_info.target_fri_B_UP_speed = FRIC_HANDLE_STUCK_SPEED;
			shooting->base_info.fri_info.target_fri_B_L_speed  = FRIC_HANDLE_STUCK_SPEED;
			shooting->base_info.fri_info.target_fri_B_R_speed  = FRIC_HANDLE_STUCK_SPEED;
			/*break*/
			shooting->base_info.fri_info.fri_handle_stuck_flag=1;
//			shooting->base_info.dail_info.target_angle_sum -= DAIL_ONESHOT_ANGLE; //反转
			shooting->base_info.fri_info.fri_handle_stuck_cnt++;//方便debug
		}
		//堵转
		if(shooting->base_info.fri_info.fri_handle_stuck_flag==1)
		{
			shooting->base_info.fri_info.fri_handle_stuck_time++;//计时退出
			
			/*break*/
			if(shooting->base_info.fri_info.fri_handle_stuck_time>=FRIC_HANDLE_STUCK_ONETIME)
			{
				//复位堵转处理相关参数
				shooting->base_info.fri_info.fri_handle_stuck_time=0;
				shooting->base_info.fri_info.fri_handle_stuck_flag=0;
				
//				shooting->base_info.dail_info.target_angle_sum += DAIL_ONESHOT_ANGLE; //回到原来位置
			}
		}
		else //正常状态
		{
			shooting->base_info.fri_info.target_fri_B_UP_speed = shooting->config->target_B_friction_speed+0;
			shooting->base_info.fri_info.target_fri_B_L_speed  = shooting->config->target_B_friction_speed+0;
			shooting->base_info.fri_info.target_fri_B_R_speed  = shooting->config->target_B_friction_speed+0;
			
		}
		Shooting_Work_Reloading(shooting); // 持续补弹
		break;
	default:
		break;
	}

	/*关控保护**********************************************************************/

		
	if (RC_OFFLINE ||shooting->shooting_state == SHOOTING_RESET_NO)
	{
		
		shooting->friction_F_UP->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_F_UP,0);
		shooting->friction_F_L ->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_F_L ,0);
		shooting->friction_F_R ->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_F_R ,0);
		
		shooting->friction_B_UP->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_B_UP,0);
		shooting->friction_B_L ->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_B_L ,0);
		shooting->friction_B_R ->base_info.motor_out=rm_motor_speed_pid_calc(shooting->friction_B_R ,0);
		shooting->dail->base_info.motor_out = 0;
		//积分清零
		integral_to_zero(&shooting->friction_B_UP->motor_all_pid.speed_pid.speed);
		integral_to_zero(&shooting->friction_B_R->motor_all_pid.speed_pid.speed);
		integral_to_zero(&shooting->friction_B_L->motor_all_pid.speed_pid.speed);
		integral_to_zero(&shooting->friction_F_UP->motor_all_pid.speed_pid.speed);
		integral_to_zero(&shooting->friction_F_R->motor_all_pid.speed_pid.speed);
		integral_to_zero(&shooting->friction_F_L->motor_all_pid.speed_pid.speed);
		//拨盘总实时更新，可能不加也行，因为需要重新用速度环初始化
		shooting->base_info.dail_info.target_angle_sum=shooting->dail->info->angle_sum; 
	}
	else if (RC_ONLINE) // 开控并且不关摩擦轮,s1下关摩擦轮
	{
		Shooting_Pid_Calculating(shooting);
		
		//模块输出->电机输出
		shooting->friction_B_L->base_info.motor_out = shooting->base_info.output_fri_B_L;
		shooting->friction_B_R->base_info.motor_out = shooting->base_info.output_fri_B_R;
		shooting->friction_B_UP->base_info.motor_out= shooting->base_info.output_fri_B_UP;
		#if HERO_TYPE == 3
		shooting->friction_F_L->base_info.motor_out = shooting->base_info.output_fri_F_L;
		shooting->friction_F_R->base_info.motor_out = shooting->base_info.output_fri_F_R;
		shooting->friction_F_UP->base_info.motor_out= shooting->base_info.output_fri_F_UP;
		#endif
		shooting->dail->base_info.motor_out = shooting->base_info.output_dail;
	}
}
