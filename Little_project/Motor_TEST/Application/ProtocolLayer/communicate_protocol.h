/**
 * @file communicate_protocol.h
 * @author Isaac
 * @brief 板间通信协议（上主控）
 * @version 0.1
 * @date 2023-11-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */



#ifndef __COMMUNICATE_PROTOCOL_H
#define __COMMUNICATE_PROTOCOL_H

#define POWER_HEAT_DATA_RX_ID        (0x100)//power_heat_data发送ID
#define GAME_ROBOT_STATUS_RX_ID      (0x101)//game_robot_status发送ID
#define SHOOT_DATA_RX_ID             (0x102)//shoot_data发送ID_
#define CHASSIS_DATA_TX_ID           (0X250)
#define CAR_DATA_TX_ID               (0X104)
#define COMMUNICATE_OFFLINE_CNT_MAX  (40)//离线最大计数(ms)

#include "driver.h"
#include "device.h"
#include "vision_protocol.h"
#include "shooting.h"
#include "rp_config.h"
/**
 * @brief 机器人状态接收结构体
 * 
 */
typedef __packed struct 
{
	uint16_t shooter_cooling_limit;			//机器人 42mm 枪口热量上限
	uint8_t car_color;                  //蓝色:1  红色:0
	uint16_t chassis_power_limit;       //底盘功率限制
	uint8_t is_laser_on;								//通过前哨站血量判断是否开激光											
	uint16_t base_distance;             //似乎是和基地的距离
}game_robot_status_rx_info_t;


/**
 * @brief 底盘功率和枪口热量接收结构体
 * 
 */
typedef  __packed struct 
{
  int16_t chassis_power_buffer;             //底盘缓存功率
	uint16_t shooter_cooling_heat; 			//机器人 42mm 枪口热量
	uint8_t rfid;                           //高地RFID状态 1：刷上 0：没刷上
	uint16_t remain_HP;                     //剩余血量
	uint8_t game_status;                    //比赛状态
}power_heat_data_rx_info_t;

/**
 * @brief 射速接收结构体
 * 
 */
typedef __packed struct 
{
	float shooting_speed;                   //弹丸初速度
	uint32_t keep_null;						//保留位
}shoot_data_rx_info_t;

/**
 * @brief 底盘包
 * 
 */
typedef __packed struct 
{
	int16_t target_front_speed;             //底盘目标前进速度
	int16_t target_right_speed;							//底盘目标平移速度
	int16_t target_cycle_speed;							//底盘目标旋转速度

	uint16_t max_speed : 15;								//最大速度设置
	uint16_t pid_mode : 1;									//0:speed  1:position 
	
}chassis_data_tx_info_t;


/**
 * @brief 整车信息包
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;//包ID

	uint16_t car_move_mode : 3;//移动模式
	uint16_t target_f_speed : 13 ;//摩擦轮目标速度
	int16_t pitch_angel;//俯仰角 *100
	__packed union   //这个总共占用一个字节，整车状态的内容就是下面这个结构体的内容，目的是节省内存
	{
		uint8_t state;  //整车状态
		__packed struct
		{
			uint8_t is_find_target : 1;//视觉是否找到目标
			uint8_t is_vision_online : 1;//视觉是否在线
			uint8_t is_on_cap : 1;//是否开超电
			uint8_t is_fire : 1;//是否单发
			uint8_t is_firing : 1;//是否连发
			uint8_t fri_speed_state : 1;//摩擦轮转速是否正常
			uint8_t is_image_up : 1;//图传是否朝上
			uint8_t is_key_ctrl : 1;//头是否朝前
		}bit;
	}car_state;

	uint8_t lf_temp;
	uint8_t rf_temp;
}car_data0_tx_info_t;

/**
 * @brief 视觉偏置包
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;//ID 1

	uint8_t chassis_angel;//底盘相对角度

	int8_t vision_yaw_offset;
	int8_t vision_pitch_offset;

  uint8_t F_fri_speed_diff;
	uint8_t B_fri_speed_diff;

	int16_t pitch_motor_angle;
}car_data1_tx_info_t;

/**
 * @brief 2包
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;

	uint16_t ui_x;
	uint16_t ui_y;
	uint8_t  detect_num;

	uint8_t lb_temp;
	uint8_t rb_temp;
}car_data2_tx_info_t;

/**
 * @brief 3包
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;

	uint8_t uix_lt;
	uint8_t uiy_lt;

	uint8_t uix_lb;
	uint8_t uiy_lb;

	uint8_t uix_rb;
	uint8_t uiy_rb;

	uint8_t uix_rt;

}car_data3_tx_info_t;

/**
 * @brief 4包
 * 
 */
typedef  __packed struct 
{
	uint8_t pack_id;

	uint8_t uiy_rt;

	uint8_t uix_left;
	uint8_t uiy_left;

	uint8_t uix_right;
	uint8_t uiy_right;

	uint16_t keep_null;
}car_data4_tx_info_t;


/**
 * @brief 板间通信状态结构体
 * 
 */
typedef __packed struct 
{
	dev_work_state_t work_state;						//工作状态
	uint8_t offline_cnt;										//离线计数
	uint8_t offline_cnt_max;								//离线最大计数
	float game_robot_status_fre;		//数据接收频率，越小频率越高
	float power_heat_data_fre;
	float shoot_data_fre;              
}communicate_status_t;

/**
 * @brief 板间通信总结构体
 * 
 */
typedef __packed struct 
{

	game_robot_status_rx_info_t *game_robot_status_rx_info;
	power_heat_data_rx_info_t *power_heat_data_rx_info;
	shoot_data_rx_info_t *shoot_data_rx_info;
	chassis_data_tx_info_t *chassis_data_tx_info;
	car_data0_tx_info_t *car_data0_tx_info;
	car_data1_tx_info_t *car_data1_tx_info;
	car_data2_tx_info_t *car_data2_tx_info;
	car_data3_tx_info_t *car_data3_tx_info;
	car_data4_tx_info_t *car_data4_tx_info;
	communicate_status_t *status;
	
}communicate_t;


extern communicate_t communicate;
extern chassis_data_tx_info_t chassis_data_tx_info;

void Chassis_Data_Tx(void);
void Game_Robot_Status_Rx(uint8_t *rxBuf);
void Power_Heat_Data_Rx(uint8_t *rxBuf);
void Shoot_Data_Rx(uint8_t *rxBuf);


void Communicate_Heartbeat(void);
void Car_Data_Tx(void);
#endif
