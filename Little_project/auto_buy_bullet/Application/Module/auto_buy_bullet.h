#ifndef _AUTO_BUY_BULLET_H
#define _AUTO_BUY_BULLET_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "string.h"
#include "crc.h"
/* define-----------------------------------------------------------------------*/
#define CLIENT_INTERACTIVE_DATA_LENGTH 8
#define CRC8_INDEX 5 		//CRC8在发送数组中的下标
#define KEY_MOUSE_DATA_CMD_ID 0x306
#define SOF 0xA5
/* Exported macro ------------------------------------------------------------*/
/**
* @brief 包头
* @note  
*/
typedef __packed struct 
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;//LEN_FRAME_HEAD

/**
* @brief 通信数据结构体
* @note 采用通用键值（上网搜）;I 73,
							 O 79 
							 H 72
							 Y 89
*/
typedef  __packed struct
{
   	uint8_t  key_value1;
    uint8_t  key_value2;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
} custom_client_data_t;
/**
* @brief 通信信息结构体
* @note  
*/
typedef __packed struct
{
	std_frame_header_t  std_frame_header_;	
	uint16_t cmd_id;
	custom_client_data_t custom_client_data;
	uint16_t CRC16;
	
} client_interactive_frame_t;
/**
* @brief 一键买弹命令状态
* @note  
*/
typedef enum 
{
	 auto_buy_bullet_SLEEP=0,
	 KEY_MOVING,
	 MOUSE_MOVING_SELECT,
  	 MOUSE_MOVING_YES1,
 	 MOUSE_MOVING_YES2,
	 KEY_CLOSE_BUY,
}auto_buy_bullet_state_e;

/**
* @brief 按键1234的任务
* @note  
*/
typedef enum 
{
	 COMMAND_SLEEP=0,
	 COMMAND_KEY1,
	 COMMAND_KEY2,
	 COMMAND_KEY3,
	 COMMAND_KEY4,
}command_running_state_e;

/**
* @brief 选手端鼠标宏总结构体
* @note  
*/
typedef __packed struct
{
	uint8_t key_running_state;//传入函数里面的
	uint8_t mouse_running_state;
	auto_buy_bullet_state_e auto_buy_bullet_state;
	command_running_state_e command_running_state; 
	client_interactive_frame_t* client_interactive_frame;
	
} Interactive_Class_t;

void test ();
void Mouse_movement_work (client_interactive_frame_t *client_interactive_frame, uint16_t x_position,uint16_t y_position, \
					 uint8_t left_state, uint8_t right_state,uint8_t* command_start_flag);
void Keyboard_movement_work (client_interactive_frame_t *client_interactive_frame,uint8_t key1_value,uint8_t key2_value,uint8_t* command_start_flag);
void Buy_bullet(uint8_t bullet_num_level);
void Deploy(void);
extern Interactive_Class_t Interactive_Class;

#define A_KEY_VALUE 65
#define B_KEY_VALUE 66
#define C_KEY_VALUE 67
#define D_KEY_VALUE 68
#define E_KEY_VALUE 69
#define F_KEY_VALUE 70
#define G_KEY_VALUE 71
#define H_KEY_VALUE 72
#define I_KEY_VALUE 73
#define J_KEY_VALUE 74
#define K_KEY_VALUE 75
#define L_KEY_VALUE 76
#define M_KEY_VALUE 77
#define N_KEY_VALUE 78
#define O_KEY_VALUE 79
#define P_KEY_VALUE 80
#define Q_KEY_VALUE 81
#define R_KEY_VALUE 82
#define S_KEY_VALUE 83
#define T_KEY_VALUE 84
#define U_KEY_VALUE 85
#define V_KEY_VALUE 86
#define W_KEY_VALUE 87
#define X_KEY_VALUE 88
#define Y_KEY_VALUE 89
#define Z_KEY_VALUE 90


#endif
