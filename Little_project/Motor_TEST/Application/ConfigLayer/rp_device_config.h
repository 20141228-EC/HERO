/**
 * @file        rp_device_config.h
 * @author      RobotPilots
 * @Version     v1.0
 * @brief       RobotPilots Robots' Device Configuration.
 * @update
 *              v1.0(7-November-2021)  
 */
#ifndef __RP_DEVICE_CONFIG_H
#define __RP_DEVICE_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "rp_driver_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* 设备层 --------------------------------------------------------------------*/
/**
 *	@brief	设备id列表
 *	@class	device
 */
typedef enum {
	DEV_ID_IMU = 0,
  DEV_ID_IMU_EX,
	DEV_ID_RC,
	DEV_ID_VISION,
	DEV_ID_CNT,
} dev_id_t;

/**
 *	@brief	设备工作状态(通用)
 *	@class	device
 */
typedef enum {
	DEV_ONLINE,
	DEV_OFFLINE,
} dev_work_state_t;


/**
 * @brief 未初始化：DEV_RESET_NO 初始化完成:DEV_RESET_OK
 * 
 */
typedef enum DEV_RESET_STATE
{
	DEV_RESET_NO,
	DEV_RESET_OK,
}Dev_Reset_State_e;

/**
 *	@brief	错误代码(通用)
 *  @note   可自定义设备错误代码类型并替换变量errno的变量类型，如
 *          typedef enum {
 *              IMU_NONE_ERR,
 *              IMU_ID_ERR,
 *              IMU_COM_FAILED,
 *              IMU_DEV_NOT_FOUND,
 *              ...
 *          } imu_errno_t;
 *          
 *          typedef struct imu_sensor_struct {
 *              ...
 *	            imu_errno_t errno;
 *              ...	
 *          } imu_sensor_t;
 *	@class	device
 */
typedef enum {
	NONE_ERR,		// 正常(无错误)
	DEV_ID_ERR,		// 设备ID错误
	DEV_INIT_ERR,	// 设备初始化错误
	DEV_DATA_ERR,	// 设备数据错误
} dev_errno_t;

typedef enum {
	GIMB_P,		//云台pitch轴		0		CAN2	 0x205
	FRIC_LF, 	//左前摩擦轮		1		CAN2	 0x204
	FRIC_RF,	//右前摩擦轮		2 		CAN2	 0x201
	FRIC_LB,	    //左后摩擦轮		3		CAN2	 0x202
	FRIC_RB,	//右后摩擦轮  		4		CAN2	 0x203
	LIMIT,          //主动限位			5		CAN2	 0x206
	DAIL,		    //拨盘				6		CAN1     0x207	
	IMAGE,        //图传				7		CAN2	 0x207
	RM_MOTOR_LIST,
	} dev_rm_motor_list_e;			  //  Yaw轴kt电机CAN1  0x142
									  //CAN1 底盘一个包，UI四个包，整车信息1个包，收3个包



#endif
