/**
 * @file vision_protocol.h
 * @author Isaac
 * @brief 视觉通信协议
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

#include "rp_device_config.h"
#include "led.h"
#define  VISION_OFFLINE_CNT_MAX  (80)//离线最大计数(ms)


/* 与lzs的视觉协定
        角度取0~8191
	
	          前方
	          4096
					   
 左方 6144	车体  2048 右方
	
	         8191/0
						后方
						
	pitch轴角度 角度取0~8191
	
            头顶
						2048

 枪管 4096  车体   0/8191 后脑勺
 
            6144
						脖子
	
*/

/**
 * @brief 视觉通信 接受信息结构体
 * 
 */
typedef __packed  struct 
{
	uint8_t  SOF;
	uint8_t  mode;//视觉状态：1 自瞄   2 识别小陀螺
	uint8_t  CRC8;
  
	float    yaw;			 //云台yaw目标角度
	float    pitch;		 //云台pitch目标角度
	uint8_t  is_find_target;//是否捕获到目标，用于交出云台控制权
	uint8_t  is_shoot_enable;//是否开火，用于更新时间戳，时间戳是视觉控制发射的权力来源
	uint16_t timing;//发射延时

	uint16_t UI_x;
	uint16_t UI_y;
 
	uint16_t uix_lt;
	uint16_t uiy_lt;

	uint16_t uix_lb;
	uint16_t uiy_lb;

	uint16_t uix_rb;
	uint16_t uiy_rb;

	uint16_t uix_rt;
	uint16_t uiy_rt;

	uint16_t uix_left;
	uint16_t uiy_left;

	uint16_t uix_right;
	uint16_t uiy_right;

	uint8_t  detect_num; 
 
	uint16_t CRC16;
}Vison_Rx_Info_t;

/**
 * @brief 视觉通信 发送信息结构体
 */
typedef __packed  struct 
{
    uint8_t  SOF;
    uint8_t  mode; // 模式
    uint8_t  CRC8;

    uint8_t  is_ready;     // 是否准备打弹 0：没准备好 1：准备好了
	uint16_t bullet_id;    // 每打出一发加1

    float    yaw;          // 云台yaw 从上往下看顺时针为负
    float    pitch;        // 云台pitch 向上为负
    float    roll;         // 云台roll
	float    v_yaw;    // 云台yaw speed 从上往下看顺时针为负
	float    v_pitch;  // 云台pitch speed 向上为负

	int8_t   yaw_offset;		//操作手手动发给视觉偏置
	int8_t	 pitch_offset;		//操作手手动发给视觉偏置

	float    bullet_speed; // 子弹速度
    uint8_t  my_color;     // 己方颜色,
	uint8_t  dune;		   // 5s倒计时和比赛开始时为1，其余时间为0

    uint16_t CRC16;
} Vision_Tx_Info_t;

/**
 * @brief 视觉通信 状态结构体
 */
typedef __packed struct 
{
	dev_work_state_t tx_state;						//发送状态
	dev_work_state_t rx_state;						//接受状态
	uint32_t send_time;                   //发送间隔
	uint32_t rx_tick;						//接受到信息时的时间
	uint8_t offline_cnt;									//接受离线计数
	uint8_t offline_cnt_max;							//接受离线最大计数
}Vision_Status_t;

/**
 * @brief 时间戳信息
 * 
 */

typedef  __packed struct 
{
	uint32_t vision_shoot_timing[3];
	uint32_t shooting_begin_tick; //开始打弹时用上一帧接受视觉的tick
}Vision_Timestamp_Info_t;


/**
 * @brief 视觉通信 总结构体
 * 
 */
typedef __packed struct 
{
	/* data */
	Vision_Tx_Info_t *tx_info;
	Vison_Rx_Info_t *rx_info;
	Vision_Timestamp_Info_t *timestamp_info;
	Vision_Status_t  *status;
	uint32_t shooting_cmd_excute_tick;
}Vision_t;
	
extern Vision_t vision;

void Vison_Interrupt_Update(void);
void Vision_led_work(void);
void Vision_DataTx(UART_HandleTypeDef *huart);
void Vision_DataRx(uint8_t *rxBuf);
void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag);
void Rearrange_Vision_Timing_Buff(uint32_t* vision_timing_buff, uint8_t size);
void Vision_HearBeat(void);
#endif
