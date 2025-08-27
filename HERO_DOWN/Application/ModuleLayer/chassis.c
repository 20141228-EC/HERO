/*
* chassis
*	
*	2023.9.11
* 底盘
*/

/* Includes ------------------------------*/
#include "chassis.h"
#include "device.h"
#include "rp_math.h"
#include "communicate_protocol.h"

/* Private function prototypes -----------------------------------------------*/

void Chassis_Speed_Calculating(chassis_t *chassis);//麦轮底盘电机目标速度解算
void Chassis_Pid_Calculating(chassis_t *chassis);//底盘PID计算
void Chassis_Power_Limit(chassis_t * chassis);//底盘功率限制
float Calculate_Predicted_Power(float i, float w);
void New_Chassis_Power_Limit(chassis_t *chassis);
void Caluculate_All_Predicted_Power(chassis_t *chassis,float *each_power,float *power_all,float *power_error);
/* Exported variables --------------------*/
chassis_t chassis = 
{
	.chassisLF = &rm_motor[CHAS_LF],
	.chassisRF = &rm_motor[CHAS_RF],
	.chassisLB = &rm_motor[CHAS_LB],
	.chassisRB = &rm_motor[CHAS_RB],
	.slip.k_power=1.7,
	.slip.max_speed_difference=8000,
	.slip.slip_low_current=0,
	.work = Chassis_Work,
	
};

/* Function  body --------------------------------------------------------*/

/**
 * @brief 从上主控更新底盘目标值
 * 
 * @param chassis 
 */
void Chassis_Master_Info_Update(chassis_t *chassis)
{
	chassis->base_info.target_cycle_speed = communicate.chassis_data_rx_info->target_cycle_speed;
	chassis->base_info.target_front_speed = communicate.chassis_data_rx_info->target_front_speed;
	chassis->base_info.target_right_speed = communicate.chassis_data_rx_info->target_right_speed;
	chassis->pid_mode					  = communicate.chassis_data_rx_info->pid_mode;
}

/**
  * @Name    Chassis_Speed_Calculating
  * @brief   底盘电机目标速度计算
**/
float k_cycle=1;  //旋转效果还好但是前进会扭 k_cycle0.5， k_tran2 
float k_tran =1;
void Chassis_Speed_Calculating(chassis_t *chassis)
{
	int16_t front = chassis->base_info.target_front_speed;
	int16_t right = chassis->base_info.target_right_speed;
	int16_t cycle = chassis->base_info.target_cycle_speed;
	
		
	int16_t speed_sum;
	float K;
	
	speed_sum = abs(front) + abs(right) + abs(cycle);
	
	if(speed_sum > CHASSIS_MAX_SPEED)
	{
		K = (float)CHASSIS_MAX_SPEED / (float)speed_sum;
	}
	else 
	{
		K = 1;
	}

	front *= K;
	right *= K;
	cycle *= K;
	#if HERO_TYPE!=2
	chassis->base_info.target_chassisLF_speed   =   front + right + cycle; 
	chassis->base_info.target_chassisLB_speed   =   front - right + cycle;
	chassis->base_info.target_chassisRF_speed   = - front + right + cycle; 
	chassis->base_info.target_chassisRB_speed   = - front - right + cycle; 
	#else
	float theta=0.87266461;
	//float k_speed=0.707*(sin(theta)+cos(theta))/cos(theta); //左上轮与中心点连线、与前面两轮连线的夹角
	
	chassis->base_info.target_chassisLF_speed   =   front + right + cycle; 
	chassis->base_info.target_chassisLB_speed   =k_tran*(front - right)+ k_cycle*cycle    ;
	chassis->base_info.target_chassisRF_speed   = - front + right + cycle; 
	chassis->base_info.target_chassisRB_speed   =k_tran*(- front - right)+ k_cycle*cycle; 
	#endif
	
}

/**
 * @brief PID计算 
 * @param chassis 
 */
void Chassis_Pid_Calculating(chassis_t *chassis)
{
	if(chassis->pid_mode == CHASSIS_SPEED_PID)//速度环
	{
		chassis->base_info.output_chassisLF = rm_motor_speed_pid_calc(chassis->chassisLF,chassis->base_info.target_chassisLF_speed);
		chassis->base_info.output_chassisRF = rm_motor_speed_pid_calc(chassis->chassisRF,chassis->base_info.target_chassisRF_speed);
		chassis->base_info.output_chassisLB = rm_motor_speed_pid_calc(chassis->chassisLB,chassis->base_info.target_chassisLB_speed);
		chassis->base_info.output_chassisRB = rm_motor_speed_pid_calc(chassis->chassisRB,chassis->base_info.target_chassisRB_speed);
	}
	else//位置环
	{
		chassis->base_info.output_chassisLF = rm_motor_anglesum_pid_calc(chassis->chassisLF,chassis->base_info.target_chassisLF_position);
		chassis->base_info.output_chassisRF = rm_motor_anglesum_pid_calc(chassis->chassisRF,chassis->base_info.target_chassisRF_position);
		chassis->base_info.output_chassisLB = rm_motor_anglesum_pid_calc(chassis->chassisLB,chassis->base_info.target_chassisLB_position);
		chassis->base_info.output_chassisRB = rm_motor_anglesum_pid_calc(chassis->chassisRB,chassis->base_info.target_chassisRB_position);
 
	}
	
}

/**
  * @name    Chassis_Power_Limit
  * @brief   底盘功率限制(经典祖传算法)
  * @param   底盘 
  * @retval
  * @author  HWX
  * @date    2022-11-06
**/
void Chassis_Power_Limit(chassis_t * chassis)
{
	if(CHASSIS_POWER_LIMIT)
	{
		int16_t limit_output_current[4];
	
		float buffer = (float)judge.power_heat_data.chassis_power_buffer;
		float heat_rate;//输出电流缩放比例
		float Limit_k; //轮组速度和缩放比例
		float CHAS_LimitOutput;//缩放后轮组最大速度之和
		float CHAS_TotalOutput;//轮组电流之和
		
		//获取理想的底盘输出
		limit_output_current[0] = chassis->base_info.output_chassisLF;
		limit_output_current[1] = chassis->base_info.output_chassisRF;
		limit_output_current[2] = chassis->base_info.output_chassisLB;
		limit_output_current[3] = chassis->base_info.output_chassisRB;
		
		uint16_t OUT_MAX = 0;
	
		OUT_MAX = CHASSIS_MAX_SPEED * 4;//最大速度之和
		
		if(buffer > 60)buffer = 60;//防止飞坡之后缓冲250J变为正增益系数
		
		Limit_k = buffer / 60.f;  //最大为1，飞坡后底盘一直最大速度运行
		
		if(buffer < 25)
			Limit_k = Limit_k * Limit_k ;//缓冲没多小就更慢一点
		else
			Limit_k = Limit_k;// 缓冲能量还有比较多就限制一点
		
		if(buffer < 60)
			CHAS_LimitOutput = Limit_k * OUT_MAX; //只要缓冲能量没满才限制
		else 
			CHAS_LimitOutput = OUT_MAX;    //缓冲能量满的就全速前进
		
		CHAS_TotalOutput = abs(limit_output_current[0]) + abs(limit_output_current[1]) + abs(limit_output_current[2]) + abs(limit_output_current[3]) ;
		
		heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;//电流缩放比例 = 利用现在剩余缓冲能量算出的速度和限制比例 * 轮组最大速度和 / 解算出的理想轮组速度和
		
	  if(CHAS_TotalOutput >= CHAS_LimitOutput)
	  {
			for(uint8_t i = 0 ; i < 4 ; i++) 
			{	
				limit_output_current[i] = (int16_t)(limit_output_current[i] * heat_rate);	
			}
		}
		/*重新赋值*/
		chassis->base_info.output_chassisLF = limit_output_current[0];
		chassis->base_info.output_chassisRF = limit_output_current[1];
		chassis->base_info.output_chassisLB = limit_output_current[2];
		chassis->base_info.output_chassisRB = limit_output_current[3];	
	}
}
void Chassis_Delay_run(chassis_t *chassis)
{
	if(chassis->chassisLB->work_state==DEV_OFFLINE &&
		chassis->chassisRB->work_state==DEV_OFFLINE &&
		chassis->chassisLF->work_state==DEV_OFFLINE &&
		chassis->chassisRF->work_state==DEV_OFFLINE )//四个电机离线
	{
		chassis->delay_run_flag=1;
	}
	//部署后延迟发电流
	if(chassis->delay_run_flag==1)
	{
		chassis->base_info.output_chassisLF = 0;
		chassis->base_info.output_chassisRF = 0;
		chassis->base_info.output_chassisLB = 0;
		chassis->base_info.output_chassisRB = 0;	
		chassis->delay_run_cnt++;
	}
	//全部连接上或者超时就给力
	if((chassis->delay_run_flag==1&&chassis->delay_run_cnt>=800)||
		(chassis->chassisLB->work_state==DEV_ONLINE &&
		chassis->chassisRB->work_state==DEV_ONLINE &&
		chassis->chassisLF->work_state==DEV_ONLINE &&
		chassis->chassisRF->work_state==DEV_ONLINE))
	{
		chassis->delay_run_flag=0;
	}
	
	
}

/**
  * @Name    Chassis_Work
  * @brief   总控
**/
float each_power[4];
float power_all;
float power_error;
int16_t error_test = 0;
void Chassis_Work(chassis_t *chassis)
{
	/*关控保护**************************************/
	if (communicate.status->chassis_data_state == DEV_OFFLINE)
	{//没收到上主控信息，卸力
		chassis->base_info.output_chassisLF = 0;
		chassis->base_info.output_chassisLB = 0;
		chassis->base_info.output_chassisRF = 0;
		chassis->base_info.output_chassisRB = 0;
	}
	else
	{
		Chassis_Master_Info_Update(chassis);//从上主控更新底盘目标值
	}

	/*目标值赋值**************************************/
	if (chassis->pid_mode == CHASSIS_SPEED_PID)//速度环，位置环在Chassis_Pid_Calculating里
	{
		//实时更新位置数据
		chassis->base_info.target_chassisLB_position = chassis->chassisLB->info->angle_sum;
		chassis->base_info.target_chassisLF_position = chassis->chassisLF->info->angle_sum;
		chassis->base_info.target_chassisRB_position = chassis->chassisRB->info->angle_sum;
		chassis->base_info.target_chassisRF_position = chassis->chassisRF->info->angle_sum;
		Chassis_Speed_Calculating(chassis);//麦轮底盘电机目标速度解算
	}
	

	/*输出电流值计算**********************************/
	Chassis_Pid_Calculating(chassis);//底盘PID计算
   // Chassis_Power_Limit(chassis);//底盘功率限制
	//新功率限制
	New_Chassis_Power_Limit(chassis);
	/*功率限制后给电机输出赋值********************************/
	chassis->chassisLF->base_info.motor_out=chassis->base_info.output_chassisLF ;
	chassis->chassisRF->base_info.motor_out=chassis->base_info.output_chassisRF ;
	chassis->chassisLB->base_info.motor_out=chassis->base_info.output_chassisLB ;
	chassis->chassisRB->base_info.motor_out=chassis->base_info.output_chassisRB ;
	//功率预测
	Caluculate_All_Predicted_Power(chassis,each_power,&power_all,&power_error);//方便debug
	Chassis_Delay_run(chassis);
}
/**
  * @Name    Calculate_Predicted_Power
  * @brief   将功率用电流和转速表达
  * @param   电流  转速
  * @result  功率
**/
float Calculate_Predicted_Power(float i, float w) {
    // 系数
    const float k_1 = 3e-07;
    const float k_2 = 1.23e-07;
	const float c = 4.081;//常数

    // 计算多项式曲面值
    float result = k_1*w*w + k_2*i*i + c + 1.99688994e-6f*i*w;

    return result;  
}
/**
  * @Name    Calculate_Current_Out
  * @brief   解算出电流输出
  * @param   目标功率  转速  原始电流
  * @result  电流
**/
float Calculate_Current_Out(float target_power,float w,int16_t raw_curret)
{
	if (target_power < 0)
	{
		return raw_curret;
	}
	

	// 系数
  const float k_1 = 3e-07;
  const float k_2 = 1.23e-07;
	const float c = 4.081;
	//已知转速解出功率相同时加速时和减速时的电流
	float t = sqrt((1.99688994e-6f * w) * (1.99688994e-6f * w) - 4 * k_2 * (k_1 * w * w + c - target_power));
	float i_1 = (-(1.99688994e-6f * w) + t) / (2 * k_2);
	float i_2 = (-(1.99688994e-6f * w) - t) / (2 * k_2);
	
	/*通过原始电流正负判断用算出来的正电流还是负电流*/
	if (raw_curret > 0)
	{
		//检测解是否正确
		if (abs(Calculate_Predicted_Power(i_1, w) - target_power) > 1)
		{
			error_test++;  
		}
		
		return i_1;
	}
	else if (raw_curret < 0)
	{
		if (abs(Calculate_Predicted_Power(i_2, w) - target_power) > 1)
		{
			error_test++;
		}

		return i_2;
	}
	else
	{
		return 0;
	}
}
/**
  * @Name    New_Chassis_Power_Limit
  * @brief   给底盘电机输出进行功率限制赋值
  * @param   chassis
**/
float limit=60;
uint8_t buf[5];
#if HERO_TYPE==2
float k_cap=20;
#else
float k_cap=13;
#endif

void New_Chassis_Power_Limit(chassis_t *chassis)
{
	if (CHASSIS_POWER_LIMIT)
	{
		if (judge.power_heat_data.chassis_power_buffer < 20)
		{
			Chassis_Power_Limit(chassis);
			return;
		}

		/*计算预测功率*/
		int16_t limit_output_current[4];

		limit_output_current[0] = chassis->base_info.output_chassisLF;
		limit_output_current[1] = chassis->base_info.output_chassisRF;
		limit_output_current[2] = chassis->base_info.output_chassisLB;
		limit_output_current[3] = chassis->base_info.output_chassisRB;

		int16_t motor_speed[4];

		motor_speed[0] = chassis->chassisLF->info->speed;
		motor_speed[1] = chassis->chassisRF->info->speed;
		motor_speed[2] = chassis->chassisLB->info->speed;
		motor_speed[3] = chassis->chassisRB->info->speed;

		float power_fit;
		float temp_power[4];
		for(uint8_t i = 0; i < 4; i++)
		{
			#if HERO_TYPE==2
			float LF_speed_abs=abs(chassis->chassisLF->info->speed);
			float RF_speed_abs=abs(chassis->chassisRF->info->speed);
			float LB_speed_abs=abs(chassis->chassisLB->info->speed);
			float RB_speed_abs=abs(chassis->chassisRB->info->speed);
			float target_front_speed=chassis->base_info.target_front_speed;
			float target_right_speed=chassis->base_info.target_right_speed;
			float target_cycle_speed=chassis->base_info.target_cycle_speed;
			buf[0]=(LF_speed_abs+LF_speed_abs-(LB_speed_abs+RB_speed_abs)>=chassis->slip.max_speed_difference);
				buf[1]=(abs(target_front_speed)>(CHASSIS_MAX_SPEED/4.f));
				buf[2]=(target_right_speed)<(CHASSIS_MAX_SPEED/4.f);
				buf[3]=abs(target_cycle_speed)<(CHASSIS_MAX_SPEED/6.f);
			
			//chassis->slip.k_dynamic=chassis->slip.k_power;
			/*不动态分配功率*/
			chassis->slip.k_dynamic=1;
			//判断前轮打滑,打滑前轮卸力
			if((((LF_speed_abs+LF_speed_abs)-(LB_speed_abs+RB_speed_abs))>=chassis->slip.max_speed_difference)&& \
				(abs(target_front_speed)>(CHASSIS_MAX_SPEED/4.f)&&abs(target_right_speed)<(CHASSIS_MAX_SPEED/4.f)&&abs(target_cycle_speed)<(CHASSIS_MAX_SPEED/6.f))\
				)
			{
				/*不进行打滑处理*/
				//chassis->slip.slip_flag=1;
			}
			if(abs(target_front_speed)<=CHASSIS_MAX_SPEED/6.f)
			{
				chassis->slip.slip_flag=0;
			}
			
			if(chassis->slip.slip_flag==1)
			{
				chassis->base_info.output_chassisLF=chassis->slip.slip_low_current;
				chassis->base_info.output_chassisRF=chassis->slip.slip_low_current;
				chassis->slip.k_dynamic=1;
			}
			//只在前进时给后轮分配更多功率，如果不是只前进或者旋转分量太大就后驱
			if(chassis->slip.k_power>=2||chassis->slip.k_power<1||(abs(target_right_speed)>(CHASSIS_MAX_SPEED/4.f)||(abs(chassis->base_info.target_cycle_speed>CHASSIS_MAX_SPEED/5.f))))
			{
				chassis->slip.k_dynamic=1;
			}
			//分配功率
			if(i==2||i==3)
			{
				temp_power[i] = (2-chassis->slip.k_dynamic)*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			}
			else
			{
				temp_power[i] = chassis->slip.k_dynamic*Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			}
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
			#else
			temp_power[i] = Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
			#endif
			
		}
		/*计算最大输出功率*/
		float max_power;
		if(communicate.car_data0_rx_info->car_state.bit.is_userdef_chas_power_limit==0)
		{
			max_power = judge.game_robot_status.chassis_power_limit*1.15;

		}
		else
		{
			max_power = communicate.car_data0_rx_info->userdef_chassis_power_limit;
		}
		
		if(communicate.car_data0_rx_info->car_state.bit.is_on_cap)//①开超电
		{
			if (cap.state == CAP_ONLINE)//②如果电容在线
			{
					if (cap.cap_U > 13)
				{
					max_power += (cap.cap_U - 13.f) *k_cap + 10;
				}
			}
		}
		
		float power_rate = max_power / power_fit;//折算率
		/*计算输出电流*/
		//预测功率大于最大功率才限制
		if (power_fit > max_power)
		{
			//通过折算后的功率、电机现在的转速、pid算出的电流来得到折算后的电流
			chassis->base_info.output_chassisLF = Calculate_Current_Out(temp_power[0] * power_rate, chassis->chassisLF->info->speed,chassis->base_info.output_chassisLF);
			chassis->base_info.output_chassisRF = Calculate_Current_Out(temp_power[1] * power_rate, chassis->chassisRF->info->speed,chassis->base_info.output_chassisRF);
			chassis->base_info.output_chassisLB = Calculate_Current_Out(temp_power[2] * power_rate, chassis->chassisLB->info->speed,chassis->base_info.output_chassisLB);
			chassis->base_info.output_chassisRB = Calculate_Current_Out(temp_power[3] * power_rate, chassis->chassisRB->info->speed,chassis->base_info.output_chassisRB);
		}
	}
}

/**
  * @Name    Caluculate_All_Predicted_Power
  * @brief   计算出所有的预测功率通过指针传出到全局变量，方便debug
  * @param   底盘结构体  电机的功率  电机总功率  预测总功率和裁判系统收到的功率的差值
**/
void Caluculate_All_Predicted_Power(chassis_t *chassis,float *each_power,float *power_all,float *power_error)
{
	int16_t limit_output_current[4];
	float motor_speed[4];
	float power_fit = 0;
	
	limit_output_current[0] = chassis->base_info.output_chassisLF;
	limit_output_current[1] = chassis->base_info.output_chassisRF;
	limit_output_current[2] = chassis->base_info.output_chassisLB;
	limit_output_current[3] = chassis->base_info.output_chassisRB;

	motor_speed[0] = chassis->chassisLF->info->speed;
	motor_speed[1] = chassis->chassisRF->info->speed;
	motor_speed[2] = chassis->chassisLB->info->speed;
	motor_speed[3] = chassis->chassisRB->info->speed;

	for(uint8_t i = 0; i < 4; i++)
	{
		each_power[i] = Calculate_Predicted_Power(limit_output_current[i], motor_speed[i]);
		if(each_power[i] > 0)
		{
			power_fit += each_power[i];
		}
	}
	*power_all = power_fit;
	*power_error = power_fit - judge.power_heat_data.chassis_power;
}
	



