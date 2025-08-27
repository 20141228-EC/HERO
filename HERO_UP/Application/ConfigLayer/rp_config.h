
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
//大部分测试功能都集成在这里，通过注释来选择是否使用功能

/*1普通三摩 2狗洞三摩 3六魔*/
#define HERO_TYPE 3
 
/*×自定义底盘功率限制×*/
//#define USERDEF_CHASSIS_POWER_LIMIT 65
/*0选择小陀螺底盘方式0*/// 0普通小陀螺 1变速小陀螺 2斜视
#define CHASSIS_CYCLE_MODE 0
/*×头可以朝后×*/
#define HEAD_BACK //头朝全向轮方向
/*×是否关闭测试功能×*/
#define TEST_TASK
/*有无热量限制*/
#define Check_Heat
 
//#ifndef IS_SIX_FRIC
/*遥控器预瞄吊射还是机械吊射*/
#define GYRO_LOB_INIT
/*是否用陀螺仪数据吊射*/
//#define USE_GYRO_LOB
//#endif
/*只取一次雷达数据来初始化吊射*/
#define ONLY_ONE_DATA_LOB_INIT

/*有无视觉打弹频率限制*/
#define VISION_SHOOT_FRE 0
/*弹速自适应*/
#define FriSpeedAdaptEnabled
/*×是否遥控器连发×*/
//#define RC_FIRING
/*是否加堵转处理*/
#define HANDLE_STUCK
/*遥控器模式下是否开超电*/ 
#define CAP_ENABLE
/*×有无开控锁×*/
// #define OPEN_RC_LOCK
/*有无遥控吊射*///注释后有遥控器机械模式
#define LOB_TEST
/*×有无横着吊射×*/
//#define ACROSS_LOB
/*×有无吊射底盘断电×*/
//#define LOB_CHASSIS_NO_POWER
/*×是否视觉可控底盘×*/
//#define VISION_CONTROL_CHASSIS

/*是否发射时补偿抖动*/
//#define SHOOT_PITCH_SHAKE_OFFSET

/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY  1
/*选择IMU解算算法为EKF*/
  #define IMU_USE_EKF 	0


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif

