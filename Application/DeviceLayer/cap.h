/**
  ******************************************************************************
  * @file           : cap.c\h
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __CAP_H
#define __CAP_H

#include "stm32h7xx_hal.h"
#define CAP_OUTPUT_POWER_LIMIT 300
#define MASTER_CAN_RX_ID 0x211
#define CAPBOARD_CAN_TX_ID 0x222
#define WIRELESS_CHARGE_CAN_RX_ID 0x212

typedef struct __attribute__((packed)) rx_info_struct {
    
    uint8_t  chassis_power_buffer;          // 底盘能量缓冲
    uint16_t chassis_power_limit ;          // 机器人底盘功率限制上限
    int16_t  cap_power_out_limit ;          // 电容放电功率限制，定义为负值
    uint16_t cap_power_in_limit  ;          // 电容充电功率限制，定义为正值
    
    struct __attribute__((packed)) bit_control_struct
    {
        uint8_t cap_switch : 1;             // 电容开关，1为开，0为关
        uint8_t turbo_mode : 1;             // 是否使用缓冲能量来充电，0为不用，1为用
        uint8_t unuse      : 6;             // 暂时未使用
    }bit_control;
    
} capboard_tx_info_t;

typedef struct __attribute__((packed)) tx_info_struct {
    
    int16_t now_chassis_power;              // 当前底盘消耗功率
    int16_t now_cap_V;                      // 当前电容组电压
    int16_t now_cap_I;                      // 当前电容组电流
    
    struct __attribute__((packed)) bit_state_struct
    {
        uint8_t ability             : 1;    // 电容是否有放电能力，0为无，1为有
        uint8_t unuse               : 7;    // 暂时未使用
    }bit_state;
    
} capboard_rx_info_t;

typedef struct 
{
	int16_t chassis_power;
	float cap_v;
	float cap_i;
	
  uint8_t   cap_offline_cnt;
	uint8_t   wireless_offline_cnt;
	uint8_t   offline_max_cnt;

}cap_t;

extern capboard_tx_info_t Cap_Tx_Info;
extern cap_t My_Cap;

extern capboard_rx_info_t Cap_Rx_Info;
void Cap_Tx_Data_Update(capboard_tx_info_t *cap);
void Cap_Rx_Data_Update(capboard_rx_info_t *cap_rx_info,uint8_t *rxbuf);

#endif
