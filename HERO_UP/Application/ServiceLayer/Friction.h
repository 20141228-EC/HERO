#ifndef __FRICTION_H
#define __FRICTION_H


#include "rp_config.h"
#include "main.h"

/* typedef----------------------------------------------------------------------------*/
typedef  __packed struct 
{
	float target_f_speed;     //前摩擦轮目标速度
	float target_b_speed;     //后摩擦轮目标速度

	int16_t friLF_speed;    //摩擦轮左前速度
	int16_t friRF_speed;    //摩擦轮右前速度
	int16_t friLB_speed;    //摩擦轮左后速度
	int16_t friRB_speed;    //摩擦轮右后速度

	int16_t friLF_minimum_speed;
	int16_t friRF_minimum_speed;
	int16_t friLB_minimum_speed;
	int16_t friRB_minimum_speed;

	int16_t fri_F_diff_speed;//前摩擦轮速度差，左减右
	int16_t fri_B_diff_speed;//后摩擦轮速度差，左减右

	bool    fri_speed_state;   //摩擦轮速度是否达到目标速度
}friction_info_t;

typedef __packed struct 
{
	float target_f_speed;      //前摩擦轮目标速度
	float target_b_speed;      //后摩擦轮目标速度
	float target_bullet_speed; //目标弹速
}shooting_config_t;

/* typedef end----------------------------------------------------------------------------*/
/*----------Function------------*/


/*----------Function_end------------*/
/**
 * @brief 命令列表
 * 
 */

#endif
