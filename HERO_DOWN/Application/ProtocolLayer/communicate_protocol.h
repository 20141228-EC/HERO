#ifndef __COMMUNICATE_PROTOCOL_H
#define __COMMUNICATE_PROTOCOL_H

#include "driver.h"
#include "device.h"
#include "rp_config.h"
// 在这里添加你的宏定义
#define POWER_HEAT_DATA_TX_ID    (0x100)//power_heat_data发送ID
#define GAME_ROBOT_STATUS_TX_ID  (0x101)//game_robot_status发送ID
#define SHOOT_DATA_TX_ID         (0x102)//shoot_data发送ID
#define GAME_ROBOT_POS_TX_ID     (0x103)//game_robot_pos发送ID
#define CHASSIS_DATA_RX_ID       (0x250)//上主控底盘包的ID
#define CAR_DATA_RX_ID			 (0X104)//上主控整车信息包ID

#define NEW_COMMUNICATE

 
/**
 * @brief 机器人状态发送结构体 10Hz
 */
typedef __packed struct 
{
	uint16_t shooter_cooling_limit;			//机器人 42mm 枪口热量上限
	__packed union   
	{
		uint8_t process;  
		__packed struct
		{
			uint8_t car_color:1;         //蓝色:1  红色:0
			uint8_t is_outpost_done : 1; //前哨爆了没
			uint8_t is_the_last_bullet : 1; //是不是最后一个允许发单量
			uint8_t is_base_open : 1; //基地下两千血没
			uint8_t null:4;
		}bit;
	}game_process;
	uint8_t chassis_power_buffer;				//剩余缓冲能量									
	float cap_U;					//电容电压
}game_robot_status_tx_info_t;

/**
 * @brief 底盘功率和枪口热量发送结构体 50Hz
 */
typedef __packed struct 
{
	float chassis_power;   					// 底盘瞬时功率，单位：W
	uint16_t shooter_cooling_heat; 			//机器人 42mm 枪口热量
	uint8_t rfid;                           //高地RFID状态 1：刷上 0：没刷上
	uint8_t game_process;                    //比赛状态
}power_heat_data_tx_info_t;

/**
* @brief 前哨基地距离和yaw磁力角度结构体，10Hz
*
*/
typedef __packed struct 
{ 
	float target_distance;             //和基地或前哨的距离
	float angle_err; //机器人枪口朝向与目标的角度差值，可以通过叠加陀螺仪到目标角度来利用
}game_robot_pos_tx_info_t; 


 

/**
 * @brief 射速发送结构体，测速检测到发送
 * 
 */
typedef __packed struct 
{
	float shooting_speed;               		    //射速
	uint32_t keep_null;							    //保留位
}shoot_data_tx_info_t;


/**
 * @brief 底盘包接受结构体
 * 
 */
typedef __packed struct 
{
	int16_t target_front_speed;              //底盘目标前进速度
	int16_t target_right_speed;				 //底盘目标平移速度
	int16_t target_cycle_speed;				 //底盘目标旋转速度

	uint16_t max_speed : 15;								//最大速度设置
	uint16_t pid_mode : 1;									//0:speed  1:position 
}chassis_data_rx_info_t;

/**
 * @brief 1包
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;//ID 1

	uint8_t chassis_angel;//底盘相对角度

	uint16_t fric_b_speed;

    uint16_t fric_f_speed;

	int16_t pitch_motor_angle;
}car_data1_rx_info_t;

/**
 * @brief 整车信息包接受结构体
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;//包ID

	uint16_t car_move_mode : 3;//移动模式
	uint8_t  pre_charge_flag : 1 ;//预充电模式标志位
	uint16_t keep_null : 12 ;
	int16_t pitch_angel;//俯仰角 *100
	__packed union 
	{
		uint8_t state;  //整车状态
		__packed struct
		{
			uint8_t is_find_target : 1;//视觉是否找到目标
			uint8_t is_vision_online : 1;//视觉是否在线
			uint8_t is_on_cap : 1;//是否开超电
			uint8_t is_userdef_chas_power_limit : 1;//是否用上主控发的功率限制
			uint8_t is_open_adapt : 1;//是否是半全半麦底盘
			uint8_t fri_speed_state : 1;//摩擦轮转速是否正常
			uint8_t is_shooting_motor_online : 1;//摩擦轮电机是否在线
			uint8_t is_key_ctrl : 1;//头是否朝前
		}bit;
	}car_state;

	uint16_t userdef_chassis_power_limit;
}car_data0_rx_info_t;
/**
 * @brief 2包 视觉装甲板
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;

	uint16_t ui_x;
	uint16_t ui_y;
	uint8_t  detect_num;

	uint8_t uix_right;
	uint8_t uiy_right;
}car_data2_rx_info_t;

/**
 * @brief 3包 ROI
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

}car_data3_rx_info_t;

/**
 * @brief 4包  ROI
 * 
 */
typedef __packed struct 
{
	uint8_t pack_id;

	uint8_t uiy_rt;

	uint8_t uix_left;
	uint8_t uiy_left;

	float vision_robot_distance;
}car_data4_rx_info_t;

/**
 * @brief 板间通信状态结构体
 * 
 */
typedef __packed struct 
{
	dev_work_state_t car_data_state;			//工作状态
	uint8_t car_data_offline_cnt;					//离线计数
	
	dev_work_state_t chassis_data_state;	//底盘包状态（开控上主控才会发，用于判断有无开控）
	uint8_t chassis_data_offline_cnt;     //离线计数

	uint8_t offline_cnt_max;						  //离线最大计数
}communicate_status_t;

/**
 * @brief 板间通信总结构体
 * 
 */
typedef __packed struct 
{
	game_robot_status_tx_info_t *game_robot_status_tx_info;
	power_heat_data_tx_info_t   *power_heat_data_tx_info;
	game_robot_pos_tx_info_t			*game_robot_pos_tx_info;
	shoot_data_tx_info_t        *shoot_data_tx_info;
	chassis_data_rx_info_t      *chassis_data_rx_info;
	car_data0_rx_info_t         *car_data0_rx_info;
	car_data1_rx_info_t     	*car_data1_rx_info;
	car_data2_rx_info_t         *car_data2_rx_info;
	car_data3_rx_info_t         *car_data3_rx_info;
	car_data4_rx_info_t         *car_data4_rx_info;
	communicate_status_t        *status;
}communicate_t;

extern communicate_t communicate;

// 在这里添加你的函数声明
void Game_Robot_Status_Tx(void);
void Power_Heat_Data_Tx(void);
void Shoot_Data_Tx(void);
void Game_robot_pos_Tx(void);

void Chassis_Data_Rx(uint8_t *rxBuf);
void Car_Data_Rx(uint8_t *rxBuf);
void Communicate_Heartbeat(void);
#endif
