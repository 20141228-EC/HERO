/**
 * @file communicate_protocol.c
 * @author Isaac 
 * @brief 板间通信协议（上主控）
 * @version 0.1
 * @date   
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "communicate_protocol.h"
#include "judge_protocol.h"
#include "drv_can.h"
#include "stdbool.h"
#include "rp_math.h"
#include "math.h"
#include "string.h"
game_robot_status_tx_info_t game_robot_status_tx_info;
power_heat_data_tx_info_t power_heat_data_tx_info;
shoot_data_tx_info_t shoot_data_tx_info;
game_robot_pos_tx_info_t	game_robot_pos_tx_info;
chassis_data_rx_info_t chassis_data_rx_info;
car_data0_rx_info_t car_data0_rx_info;
car_data1_rx_info_t car_data1_rx_info;
car_data2_rx_info_t car_data2_rx_info;
car_data3_rx_info_t car_data3_rx_info;
car_data4_rx_info_t car_data4_rx_info;

communicate_status_t communicate_status =
{
  .chassis_data_offline_cnt = 0,
  .chassis_data_state = DEV_OFFLINE,
  
  .car_data_offline_cnt = 0,
  .car_data_state = DEV_OFFLINE,

  .offline_cnt_max = 50,
};

communicate_t communicate =
{
  .game_robot_status_tx_info = &game_robot_status_tx_info,
  .power_heat_data_tx_info = &power_heat_data_tx_info,
  .game_robot_pos_tx_info=&game_robot_pos_tx_info,
  .shoot_data_tx_info = &shoot_data_tx_info,
  .chassis_data_rx_info = &chassis_data_rx_info,
  .car_data0_rx_info = &car_data0_rx_info,
  .car_data1_rx_info = &car_data1_rx_info,
  .car_data2_rx_info = &car_data2_rx_info,
  .car_data3_rx_info = &car_data3_rx_info,
  .car_data4_rx_info = &car_data4_rx_info,
  .status = &communicate_status,
};

extern CAN_HandleTypeDef hcan1;

/**
  * @name    Power_Heat_Data_Tx
  * @brief   机器人功率热量数据发送给上主控
  * @param   
  * @retval  None
  * @note 在接收到power_heat_data数据后，调用此函数
  */
void Power_Heat_Data_Tx(void)
{
  power_heat_data_tx_info_t *tx_info = &power_heat_data_tx_info;
  uint8_t tx_buff[8];
  uint32_t txMailBox;
  //底盘瞬时功率
  tx_info->chassis_power = judge.power_heat_data.chassis_power;
  //枪口热量
  tx_info->shooter_cooling_heat = judge.power_heat_data.shooter_id1_42mm_cooling_heat;
  //rfid
  if (judge.rfid_status.rfid_status == 0x02)
  {
    tx_info->rfid = 1;
  }
  else
  {
    tx_info->rfid = 0;
  }
  //比赛状态
  tx_info->game_process = judge.game_status.game_progress;

  memcpy(tx_buff, tx_info, sizeof(power_heat_data_tx_info_t));
  HAL_CAN_TxHeadeInit(POWER_HEAT_DATA_TX_ID);//配置ID
  HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeadeType,tx_buff,&txMailBox);
}

/**
  * @name    Game_Robot_Status_Tx
  * @brief   机器人状态发送给上主控
  * @param   
  * @retval  None
  * @note 在接收到game_robot_status数据后，调用此函数
  */
void Game_Robot_Status_Tx(void)
{
  game_robot_status_tx_info_t *tx_info = &game_robot_status_tx_info;
  uint8_t tx_buff[8];
  uint32_t txMailBox;
  
  //机器人 42mm 枪口热量上限
  tx_info->shooter_cooling_limit = \
    judge.game_robot_status.shooter_barrel_heat_limit;
  //判断自己是什么颜色
  if (judge.game_robot_status.robot_id <= 10)
  {
    tx_info->game_process.bit.car_color = 0;//红方
  }
  else
  {
    tx_info->game_process.bit.car_color = 1;//蓝方
  }
  //缓冲能量
  tx_info->chassis_power_buffer=judge.power_heat_data.chassis_power_buffer;
  //超电电压
  tx_info->cap_U=cap.cap_U;
  //发前哨有没有爆
  if (tx_info->game_process.bit.car_color == 0)//红方
  {
    if (judge.game_robot_HP.blue_outpost_HP == 0)
    {
      tx_info->game_process.bit.is_outpost_done = 1;//爆了
 
    }
    else
    {
      tx_info->game_process.bit.is_outpost_done = 0;//没爆  
	 		
    }
  }
  else//蓝方
  {
    if (judge.game_robot_HP.red_outpost_HP == 0)
    {
      tx_info->game_process.bit.is_outpost_done = 1;      
	}
    else
    {
      tx_info->game_process.bit.is_outpost_done = 0;
    }

  }
  //判断是否剩下最后一颗弹丸，如果是就发1，否则发0
  if(judge.bullet_remaining.bullet_remaining_num_42mm==1)
  {
	tx_info->game_process.bit.is_the_last_bullet=1;
  }
  else
  {
	tx_info->game_process.bit.is_the_last_bullet=0;
  }
  //基地打开没
  if (tx_info->game_process.bit.car_color == 0)//红方
  {
    if (judge.game_robot_HP.blue_base_HP <=2000)
    {
      tx_info->game_process.bit.is_base_open = 1;//爆了
 
    }
    else
    {
      tx_info->game_process.bit.is_base_open = 0;//没爆  
	 		
    }
  }
  else//蓝方
  {
    if (judge.game_robot_HP.red_base_HP <=2000)
    {
      tx_info->game_process.bit.is_base_open = 1;      
	}
    else
    {
      tx_info->game_process.bit.is_base_open = 0;
    }

  }
  
  //发送数据
  memcpy(tx_buff, tx_info, sizeof(game_robot_status_tx_info_t));
  HAL_CAN_TxHeadeInit(GAME_ROBOT_STATUS_TX_ID);//配置ID
	HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeadeType,tx_buff,&txMailBox);
}
/**
  * @name    Shoot_Data_Tx
  * @brief   射击数据发送给上主控
  * @retval  None
  * @note 在接收到shoot_data数据后，调用此函数
  */
void Shoot_Data_Tx(void)
{
  shoot_data_tx_info_t *tx_info = &shoot_data_tx_info;
  uint8_t tx_buff[8];
  uint32_t txMailBox;
  
  tx_info->shooting_speed = judge.shoot_data.bullet_speed;
  
  memcpy(tx_buff, tx_info, sizeof(shoot_data_tx_info_t));
  HAL_CAN_TxHeadeInit(SHOOT_DATA_TX_ID);//配置ID
  HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeadeType,tx_buff,&txMailBox);
}

/**
  * @name    Game_robot_pos_Tx
  * @brief   机器人枪口朝向以及位置发送给上主控
  * @retval  None
  * @note 在接收到Game_robot_pos数据后，调用此函数
  */
#define UWB_TEST
float x=8; 
float y=2; 
uint8_t red_outpost_HP;
uint8_t   car_color ;
float offset_angle=165;//将机器人在红方基地直接瞄准蓝方基地时候的磁力计角度
void Game_robot_pos_Tx(void)
{
  game_robot_pos_tx_info_t *tx_info = &game_robot_pos_tx_info;
  uint8_t tx_buff[8];
  uint32_t txMailBox;
    #ifdef UWB_TEST
	float uwb_x = x;
    float uwb_y =y; 
	#else 
   float uwb_x = judge.game_robot_pos.x;
   float uwb_y = judge.game_robot_pos.y;
	#endif
   
	//angle为补偿到基地对基地为正北方向+半圈处理，磁力计逆时针增大
   float angle = judge.game_robot_pos.angle-offset_angle;
	angle=motor_half_cycle(angle,360);
	#ifdef UWB_TEST
	if ( car_color == 0)//红方
	#else 
	if (game_robot_status_tx_info.car_color == 0)//红方
	#endif
  
  {
		#ifdef UWB_TEST
	if ( red_outpost_HP == 0)//前哨爆了打基地
	#else 
	if (judge.game_robot_HP.red_outpost_HP == 0)//前哨爆了打基地
	#endif
	  
	  {
          tx_info->target_distance = sqrt((uwb_x - 26.24f)*(uwb_x - 26.24f)+(uwb_y - 7.5f)*(uwb_y - 7.5f)) ;
		  float x_err=26.24f-uwb_x;
		  float y_err=7.5f-uwb_y;
		  tx_info->angle_err=angle-atan2(y_err,x_err)*57.295;//atan2(x,y)
	  }
    
    else//前哨没爆打前哨
    {
	  tx_info->target_distance = sqrt((uwb_x - 17.01f)*(uwb_x - 17.01f)+(uwb_y - 11.36f)*(uwb_y - 11.36f)) ;
		float x_err=17.01f-uwb_x;
		float y_err=11.36f-uwb_y;
		tx_info->angle_err=angle-atan2(y_err,x_err)*57.295;//atan2(x,y)
    }
  }
  else//蓝方
  {
    if (judge.game_robot_HP.red_outpost_HP == 0) //前哨爆了打基地
    {
	  tx_info->target_distance = sqrt((uwb_x - 1.75f)*(uwb_x - 1.75f)+(uwb_y - 7.5f)*(uwb_y - 7.5f)) ;
		float x_err=1.75f-uwb_x;
		float y_err=7.5f-uwb_y;
		tx_info->angle_err=angle-atan2(y_err,x_err)*57.295f;//atan2(x,y)
	}
    else//前哨没爆打前哨
    {
	  tx_info->target_distance = sqrt((uwb_x - 10.99f)*(uwb_x - 10.99f)+(uwb_y - 3.64f)*(uwb_y - 3.64f)) ;
		float x_err=10.99f-uwb_x;
		float y_err=3.64f-uwb_y;
		tx_info->angle_err=angle-atan2(y_err,x_err)*57.295f;//atan2(x,y)
    }

  }
  memcpy(tx_buff, tx_info, sizeof(game_robot_pos_tx_info_t));
  HAL_CAN_TxHeadeInit(GAME_ROBOT_POS_TX_ID);//配置ID
  HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeadeType,tx_buff,&txMailBox);
}

/**
 * @brief 底盘信息接受
 * 
 * @param rxBuf 
 */
void Chassis_Data_Rx(uint8_t *rxBuf)
{
  memcpy(&chassis_data_rx_info, rxBuf, sizeof(chassis_data_rx_info_t));
  communicate.status->chassis_data_offline_cnt = 0;
}

/**
 * @brief 整车信息接受
 * @param rxBuf 
 */
void Car_Data_Rx(uint8_t *rxBuf)
{
  uint8_t pack_id = rxBuf[0];
  if (pack_id == 0)
  {
    memcpy(&car_data0_rx_info,rxBuf,sizeof(car_data0_rx_info_t));
  }
  else if (pack_id == 1)
  {
    memcpy(&car_data1_rx_info,rxBuf,sizeof(car_data1_rx_info_t));
  }
  else if (pack_id == 2)
  {
    memcpy(&car_data2_rx_info,rxBuf,sizeof(car_data2_rx_info_t));
  }
//  else if (pack_id == 3)
//  {
//    memcpy(&car_data3_rx_info,rxBuf,sizeof(car_data3_rx_info_t));
//  }
//  else if (pack_id == 4)
//  {
//    memcpy(&car_data4_rx_info,rxBuf,sizeof(car_data4_rx_info_t));
//  }

  communicate.status->car_data_offline_cnt = 0;
}


/**
 * @brief 通信心跳 监控任务中调用
 * 
 */
void Communicate_Heartbeat(void)
{
	uint8_t offline_cnt_max = communicate.status->offline_cnt_max;
  //整车信息包接受心跳
  communicate.status->car_data_offline_cnt ++;
	if(communicate.status->car_data_offline_cnt > offline_cnt_max)
  {
    communicate.status->car_data_offline_cnt = offline_cnt_max;
    communicate.status->car_data_state = DEV_OFFLINE;
  }
  else if(communicate.status->car_data_state == DEV_OFFLINE)
  {
    communicate.status->car_data_state = DEV_ONLINE;
  }
  //底盘包接受心跳 用于判断是否开控
  communicate.status->chassis_data_offline_cnt ++;
	if(communicate.status->chassis_data_offline_cnt > offline_cnt_max)
  {
    communicate.status->chassis_data_offline_cnt = offline_cnt_max;
    communicate.status->chassis_data_state = DEV_OFFLINE;
  }
  else if(communicate.status->chassis_data_state == DEV_OFFLINE)
  {
    communicate.status->chassis_data_state = DEV_ONLINE;
  }
}
  
