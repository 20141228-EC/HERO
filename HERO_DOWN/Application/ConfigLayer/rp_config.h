/**
 * @file        rp_config.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Configuration.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(7-November-2021)
 *                  1.优化设备类信息与结构体的变量定义，增加volatile/const关键字
 *                  //2.将rp_config.h分成driver_config.h, device_config.h, user_config.h三个头文件    
 */
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/
/*有无超电*/
#define CAP_ENABLE		1
/*关闭板间通信*/
//#define COMMUNICATE_ENABLE		
/*有无裁判系统*/
#define JUDGE_ENABLE 	1
	
/*1普通三摩 2狗洞三摩 3六魔*/
#define HERO_TYPE 2

/*是否关闭测试功能*/
#define TEST_TASK
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif
