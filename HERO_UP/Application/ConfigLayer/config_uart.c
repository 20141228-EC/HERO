/**
  ******************************************************************************
  * @file           : config_uart.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#include "config_uart.h"
#include "vision_protocol.h"
#include "rc_sensor.h"
#include "communicate_protocol.h"

/**
  * @Name    USART1_rxDataHandler
  * @brief   视觉数据更新
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	Vision_DataRx(rxBuf);
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   遥控器更新
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	//		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
//		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
	rc_sensor.update(&rc_sensor, rxBuf);//解析协议
	
}

/**
  * @Name    USART6_rxDataHandler
  * @brief    
**/

void USART6_rxDataHandler(uint8_t *rxBuf)
{
	
}
