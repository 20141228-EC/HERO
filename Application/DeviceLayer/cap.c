/**
  ******************************************************************************
  * @file           : cap.c\h
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "cap.h"
#include "drv_can.h"
#include "fdcan.h"
#include "judge.h"
#include "string.h"
#include "stdio.h"

cap_t My_Cap =
{
	.cap_offline_cnt = 0,
	.wireless_offline_cnt = 0,
	.offline_max_cnt = 150,
};

static float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    int32_t a_32 = a, a_max_32 = a_max, a_min_32 = a_min;
    int32_t diff_a = a_max_32 - a_min_32;
    
    if (diff_a == 0) return (b_max + b_min) / 2.0f; // 处理除零
    
    float ratio = (float)(a_32 - a_min_32) / (float)diff_a;
    return ratio * (b_max - b_min) + b_min;
}

static int16_t float_to_int16(float b, float b_max, float b_min, int16_t a_max, int16_t a_min)
{
    // 处理除零和无效输入
    if (b_max == b_min) return (int16_t)((a_max + a_min) / 2);
    
    // 计算比例并映射到整数范围
    float ratio = (b - b_min) / (b_max - b_min);
    
    // 提升计算范围避免溢出
    int32_t a = (int32_t)(ratio * (a_max - a_min) + a_min + 0.5f); // 四舍五入
    
    // 钳位到目标范围
    a = (a < a_min) ? a_min : (a > a_max) ? a_max : a;
    
    return (int16_t)a;
}

capboard_tx_info_t Cap_Tx_Info;

capboard_rx_info_t Cap_Rx_Info;

uint8_t Cap_Tx_Buf[8];
void Cap_Tx_Data_Update(capboard_tx_info_t *cap)
{
	
	cap->chassis_power_limit = My_Judge.org_info->game_robot_status.chassis_power_limit;
	cap->chassis_power_buffer = My_Judge.org_info->power_heat_data.buffer_energy;
	cap->cap_power_out_limit = -300;
	cap->cap_power_in_limit = 300;
	cap->bit_control.cap_switch = 1;
	cap->bit_control.turbo_mode = 0;
	
	memcpy(Cap_Tx_Buf,cap,sizeof(capboard_tx_info_t));
	
	CAN_SendData(&hfdcan3,0x222,Cap_Tx_Buf);
}

float cap_V,cap_I;
void Cap_Rx_Data_Update(capboard_rx_info_t *cap_rx_info,uint8_t *rxbuf)
{
   
    /* 拷贝内存 */
    memcpy(cap_rx_info, rxbuf, sizeof(capboard_rx_info_t));
	  My_Cap.cap_offline_cnt = 0;
	  My_Cap.chassis_power = cap_rx_info->now_chassis_power;
	  My_Cap.cap_v = int16_to_float(cap_rx_info->now_cap_V, 32000, -32000, 25, 0);
    My_Cap.cap_i = int16_to_float(cap_rx_info->now_cap_I, 32000, -32000, 16, -16);
	
}

