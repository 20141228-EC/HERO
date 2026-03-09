#ifndef __shoot_H_
#define __shoot_H_

#include "main.h"
#include "DM_Motor.h"
#include "shoot_motor.h"
#include "communicate.h"
#include "motor_def.h"
#include "judge.h"
#include "shoot_base.h"
#include "rc_sensor.h"
///**
// * @brief  拨盘实时数据接收结构体
// * @note  数据在外部文件更新
// */
//typedef struct{
//	int16_t               angle;           //角度
//	int16_t               speed;           //速度
//  int16_t              current;         //电流

//}Dial_Info_t;

///**
//* @brief  发射机构实时标志位接收结构体
// * @note  标志位在外部文件更新，内部处理
// */
//typedef struct{
//	uint8_t is_sleep_flag;                  //发射机构睡眠/卸力标志位，开控置0，关控置1，若置1后需重新复位，机器人阵亡时无需复位不应置1  
//	uint8_t is_mtr_offline_flag; 						//发射机构是否有相关电机掉线，1为掉线，0为在线，掉线了不响应开火操作 && is_ready_flag=0，但不一定进入sleep状态
//	uint8_t fire_mode_flag;                 //开火模式标志位，单发为 0，连发为 1 
//	uint8_t elec_level_flag;                //电平标志位，高电平为 1，低电平为 0

//}Flag_Info_t;

/*摩擦轮速度*/
typedef struct __attribute__((packed)){
  bool is_fric_on;
}friction_info_t;

/*发射基础信息包*/
typedef struct __attribute__((packed)){
		int16_t    	    output_dail;      
	
	bool is_heat_allow;//热量允许打弹
	bool is_enable_shoot;//热量允许打弹
	uint16_t launch_timer;//延时发弹
	
	friction_info_t fric_info;
}shoot_base_info_t;

typedef struct __attribute__((packed))shoot_out_t{

	Motor_DM_t *dail;
	shoot_base_info_t base_info;
	uint8_t stuck_count;
	friction_info_t fric;
	void     	    (*work)(struct shoot_out_t *shoot_out);  

}shoot_out_t;


extern shoot_out_t shoot_out;



#endif
