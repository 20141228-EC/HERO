#ifndef __LIMIT_H
#define __LIMIT_H


#include "rp_config.h"
#include "rm_motor.h"
#include "motor.h"
#include "main.h"

/* Private define----------------------------------------------------------------------------*/
//拨盘参数
#define DIAL_REVERT_PISITION (-5000.f)  //拨盘反转要走的角度（反转为负）
#define DIAL_FEED_SPEED (3000.f)  		//拨盘供弹速度
#define DIAL_FEED_ONE_ANGEL (32000.f)  	//拨盘供弹一次的角度
#define DIAL_STUCK_SPEED (-3000.f)    	//拨盘反转处理卡弹的速度(反转为负)
#define DIAL_MAX_INIT_TIMES 1500 		//拨盘最大初始化时间（判断弹仓是否为空）
#define DIAL_MAX_FEED_TIMES 3000 		//拨盘最大供弹时间（判断弹仓是否堵转）
#define DIAL_MAX_WORK_TIMES 300   		//非初始化阶段拨盘最大工作时间（判断弹仓是否为空）
#define DIAL_MAX_F_STUCK_CNT 2    		//拨盘正向堵转最大次数（判断是否供弹完成）

/* typedef----------------------------------------------------------------------------------*/
/**
 * @brief 限位控制模式枚举
 * 
 */
typedef enum
{
	LIMIT_POSITION_MODE = 0,  //限位位置模式
	LIMIT_SPEED_MODE,     //限位速度模式
}limit_ctrl_mode_e;

/**
 * @brief 限位类信息
 * 
 */
typedef __packed struct 
{
	float             target_speed;     //限位目标速度
	int32_t           target_position;  //限位目标位置
	limit_ctrl_mode_e ctrl_mode;		    //限位控制模式
}limit_info_t;



/* typedef end----------------------------------------------------------------------------------*/
/* Function----------------------------------------------------------------------*/



/* Function_end-------------------------------------------------------------------*/

#endif
