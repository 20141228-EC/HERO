/**
  ******************************************************************************
  * @file           : config_uart.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#include "config_uart.h"
#include "judge_protocol.h"


/**
 * @brief 裁判系统更新
 * @param rxBuf 
 */
void USART1_rxDataHandler(uint8_t *rxBuf)
{
	judge_update(&judge,rxBuf);
}

void USART3_rxDataHandler(uint8_t *rxBuf)
{

}

void USART6_rxDataHandler(uint8_t *rxBuf)
{

}
