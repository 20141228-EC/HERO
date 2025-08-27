/*
* chassis
*	
*	2023.9.11
* 底盘
*/

/* Includes ------------------------------*/
#include "chassis.h"
#include "gimbal_Omni.h"
#include "device.h"
#include "rp_math.h"
#include "car.h"
#include "communicate_protocol.h"

/* Exported variables --------------------*/
chassis_t chassis = 
{	
	.cmd_auto_lob = &command[AUTO_LOB],
	.cmd_180   = &command[GIM_180],
	.work = Chassis_Work,
	
};
int16_t yaw_angle_err;

/* Private function prototypes -----------------------------------------------*/
void Chassis_Mec_Update(chassis_t *chassis,uint8_t ctrl_mode);//机械模式底盘数据更新
void Chassis_Gyro_Update(chassis_t *chassis,uint8_t ctrl_mode, uint8_t move_mode);//陀螺仪模式底盘数据更新
void Chassis_Cmd_Excute(chassis_t *chassis);				//吊射命令初始化时底盘跟云台

/**
 * @brief 底盘结构体数据更新
 * @param ctrl_move 0:遥控器模式 1：键盘模式
 */
void Chassis_Mec_Update(chassis_t *chassis,uint8_t ctrl_mode)
{	
	if(ctrl_mode == 0)//遥控器模式   //线性拟合遥控器数据得到正确的目标底盘X、Y、w速度，
	{
		chassis->base_info.target_front_speed = (float) rc_sensor.info->ch3 / RC_MAX_CNT * CHASSIS_MAX_SPEED;
		chassis->base_info.target_right_speed = (float) rc_sensor.info->ch2 / RC_MAX_CNT * CHASSIS_MAX_SPEED;
		chassis->base_info.target_cycle_speed = (float) rc_sensor.info->ch0 / RC_MAX_CNT * CHASSIS_MAX_SPEED;
	}
	else if (ctrl_mode == 1)//键盘模式
	{
		chassis->base_info.target_front_speed = 0;
		chassis->base_info.target_right_speed = 0;
		chassis->base_info.target_front_speed += (float)rc_sensor.info->W.cnt / (float)KEY_W_CNT_MAX * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_front_speed -= (float)rc_sensor.info->S.cnt / (float)KEY_S_CNT_MAX * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_right_speed += (float)rc_sensor.info->D.cnt / (float)KEY_D_CNT_MAX * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_right_speed -= (float)rc_sensor.info->A.cnt / (float)KEY_A_CNT_MAX * CHASSIS_MAX_SPEED ;
		chassis->base_info.target_cycle_speed =  rc_sensor.info->mouse_vx*10.f;
	}
	
	if (abs(gimbal.base_info.yaw_motor_angle) > 16384)  //如果云台yaw轴电机超过16384就对XY目标速度取反
	{
		chassis->base_info.target_front_speed *= -1;
		chassis->base_info.target_right_speed *= -1;
	}
}

/**
 * @brief 底盘陀螺仪模式更新
 * 
 * @param chassis 
 * @param ctrl_move 0:遥控器模式 1：键盘模式
 * @param move_mode 0:陀螺仪模式 1：小陀螺模式
 * @note 通过“底盘机械模式更新”更新目标值，再把目标值映射到底盘
 */
void Chassis_Gyro_Update(chassis_t *chassis,uint8_t ctrl_mode, uint8_t move_mode)
{
	Chassis_Mec_Update(chassis,ctrl_mode);
	float front = chassis->base_info.target_front_speed;
	float right = chassis->base_info.target_right_speed;
	if (abs(gimbal.base_info.yaw_motor_angle) > 16384)  //将机械模式头朝后的变化变回正常，再进行旋转
	{
		front *= -1;
		right *= -1;
	}
	//计算陀螺仪坐标和底盘坐标角度差 
	int16_t yaw_angle_err;
	
	yaw_angle_err  = gimbal.base_info.yaw_motor_angle / 32768.f * 4096.f;			//yaw轴   相对底盘   角度(-4096~4096)(顺时针为正)
	float yaw_angle_err_rad = (double)yaw_angle_err / 4096.f * 3.14159;             //yaw轴角度转弧度制（-π~π）

	//front和right值计算
	chassis->base_info. target_front_speed=  front * cos(yaw_angle_err_rad) - right * sin(yaw_angle_err_rad);
	chassis->base_info. target_right_speed=  right * cos(yaw_angle_err_rad) + front * sin(yaw_angle_err_rad);

	
	if(move_mode == 0)
	{
		//cycle值计算
		if(abs(yaw_angle_err) > 2048)//头可以朝后
		{
			yaw_angle_err -= 4096 * sgn(yaw_angle_err);
		}
		if(abs(yaw_angle_err) >= 100) //大于4.4°斜率变大 
		{
			chassis->base_info.target_cycle_speed = yaw_angle_err * 6.f - 300.f * sgn(yaw_angle_err);
		}
		else 
		{
			chassis->base_info.target_cycle_speed = yaw_angle_err * 1.f;
		}
	}
	else if (move_mode == 1)
	{
		if (car.car_move_mode == vision_cycle_CAR)
		{
			chassis->base_info.target_cycle_speed = -CYCLE_SPEED;
		}
		else
		{
			chassis->base_info.target_cycle_speed = CYCLE_SPEED;//小陀螺速度
		}
	}
	
}

/*
  * @Name    Chassis_Cmd_Excute
  * @brief   吊射命令初始化时底盘跟云台
*/

void Chassis_Cmd_Excute(chassis_t *chassis)
{
	if(chassis->cmd_auto_lob->cmd_value == 1)
	{
		chassis->cmd_auto_lob->s_run(chassis->cmd_auto_lob);
	}

	if(chassis->cmd_auto_lob->cmd_status == RUNING_C)
	{
		#ifdef ACROSS_LOB
		float chassis_error = gimbal.base_info.yaw_motor_angle - sgn(gimbal.base_info.yaw_motor_angle)*16384;
		#else
		float chassis_error = gimbal.base_info.yaw_motor_angle - sgn(gimbal.base_info.yaw_motor_angle)*32768;
		#endif

		chassis->base_info.target_cycle_speed = chassis_error;

		if(abs(chassis_error) < 10&&abs(gimbal.base_info.yaw_motor_speed)<30)
		{
			chassis->cmd_auto_lob->s_finish(chassis->cmd_auto_lob);
		}
	}
}

/**
  * @Name    Chassis_Work
  * @brief   总控
  * @param   chassis
**/

void Chassis_Work(chassis_t *chassis)
{
	Chassis_Cmd_Excute(chassis);
	/*更新目标值**********************************/
  switch(car.car_move_mode)
	{
		case offline_CAR:
		case init_CAR:
      chassis->base_info.target_cycle_speed = 0;
      chassis->base_info.target_front_speed = 0;
      chassis->base_info.target_right_speed = 0;
		break;
		
		case mec_CAR:
				if (chassis->cmd_auto_lob->cmd_status != RUNING_C)
				{
					Chassis_Mec_Update(chassis,car.car_ctrl_mode);//底盘机械模式更新
				}
			break;
		
		case vision_gyro_CAR:
		case gyro_CAR:
			if (chassis->cmd_180->cmd_status == RUNING_C) //掉头时只控底盘
			{
				Chassis_Mec_Update(chassis,car.car_ctrl_mode);//底盘机械模式更新
			}
			else{
					Chassis_Gyro_Update(chassis,car.car_ctrl_mode,0); //陀螺仪模式
				}
			break;

		case vision_cycle_CAR:
		case cycle_CAR:
					Chassis_Gyro_Update(chassis,car.car_ctrl_mode,1); //小陀螺模式
			break;
		
		case lob_CAR:
			if (chassis->cmd_auto_lob->cmd_status == FINISH_C) //吊射模式初始化完成，底盘固定
			{
				chassis->base_info.target_cycle_speed = 0;
				chassis->base_info.target_front_speed = 0;
				chassis->base_info.target_right_speed = 0;
			}
			break;

		default:
			break;
	}
	
	/*pid模式更新**********************************/
	if (car.car_move_mode == lob_CAR && chassis->cmd_auto_lob->cmd_status != RUNING_C)
	{
		chassis->pid_mode = CHASSIS_POSITION_PID;
	}
	else
	{
		chassis->pid_mode = CHASSIS_SPEED_PID;
	}

	/*底盘包更新******************************/
	communicate.chassis_data_tx_info->target_front_speed = chassis->base_info.target_front_speed;
	communicate.chassis_data_tx_info->target_right_speed = chassis->base_info.target_right_speed;
	communicate.chassis_data_tx_info->target_cycle_speed = chassis->base_info.target_cycle_speed;
	communicate.chassis_data_tx_info->pid_mode           = chassis->pid_mode;
	communicate.chassis_data_tx_info->max_speed          = CHASSIS_MAX_SPEED;
	//Chassis_Data_Tx();
	
}







