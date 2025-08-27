/**
 * @file control.c
 * @author Isaac
 * @brief 
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "Dial.h"

/* tyepdef ------------------------------------------------------------------*/
dail_info_t dail_info={
	.target_speed=DIAL_FEED_SPEED,
	.target_position=DIAL_STUCK_SPEED,
	.f_stucking_cnt=DIAL_MAX_F_STUCK_CNT,
	.b_stucking_cnt=2,
	.dail_reset_state=DEV_RESET_NO,
	.work_state=DIAL_SLEEP,
	                              //还有拨盘工作状态
								  //拨盘初始化状态
};
dail_state_e dail_state;
dail_detect_stuck_time_t dail_detect_stuck_time;  //拨盘卡单检测时间结构体
dial_release_e dial_release;  //拨盘复位后是否断电枚举

/* 
 *	@brief 卡弹检测
 */
void Detect_stuck(void)
{
	dail_detect_stuck_time.dail_now_time=HAL_GetTick();
	
	//检测到发射要求，设置发射标志位，并且开始卡弹计时
	if(NEED_SHOOT)
	{
		
		shooting_command=SHOOT_NOW;
		dail_detect_stuck_time.dail_start_time=HAL_GetTick();
	}
	
	//如果发弹成功，重置卡弹计时器
	
	if(rm_motor[DIAL].info->angle_sum>DIAL_FEED_ONE_ANGEL)
	{
		dail_detect_stuck_time.dail_start_time=HAL_GetTick();
		shooting_command=WAIT_SHOOT;
	}
	
	//如果堵转设标志位，并且重置卡弹计时器
	if(dail_detect_stuck_time.dail_now_time-dail_detect_stuck_time.dail_start_time>DIAL_MAX_FEED_TIMES)
	{
		dail_info.work_state=DIAL_STUCK;
		shooting_command=WAIT_SHOOT;
		dail_detect_stuck_time.dail_start_time=HAL_GetTick();
	}
	
}
/* Private function prototypes -----------------------------------------------*/

/* 
 *	@brief 拨盘拨一次函数，设置重新装弹标志位，角度闭环
 */
void Dail_one_shot (void)
{
	dail_info.work_state=DIAL_RELOAD;
	Motor_ToAxleAngle(&rm_motor[DIAL],DIAL_FEED_ONE_ANGEL);
}








