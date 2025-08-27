#ifndef __DIAL_H
#define __DIAL_H


#include "rp_config.h"
#include "RM_motor.h"
#include "motor.h"
#include "drv_can.h"
#include "main.h"

/* Private define----------------------------------------------------------------------------------*/


/* typedef----------------------------------------------------------------------------------*/

/**
 * @brief 拨盘是否断电状态枚举
 */
typedef enum
{
	DIAL_RELEASE , //拨盘断电
	DIAL_NO_RELEASE, //拨盘上电
	DIAL_RELEASE_FLAG, //拨盘断电标志位
}dial_release_e;

/**
 * @brief 卡弹检测时间结构体(一定时间内没拨到对应角度就标志卡弹)
 */
typedef struct dail_detect_stuck_time_struct
{
	uint32_t dail_start_time ;	  //开始波蛋的时间
	uint32_t dail_now_time;  	  //实时时间
}dail_detect_stuck_time_t;



extern dail_detect_stuck_time_t dail_detect_stuck_time;  //拨盘卡单检测时间结构体
extern dial_release_e dial_release;  //拨盘复位后是否断电枚举
extern dail_info_t dail_info;
extern dail_state_e dail_state;
/* typedef end----------------------------------------------------------------------------------*/

/*----------Function-----------------------------------------------------------------------------*/



/*----------Function_end------------------------------------------------------------------------*/
/**
 * @brief 命令列表
 * 
 */

#endif
