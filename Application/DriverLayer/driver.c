/**
 ******************************************************************************
 * @file        driver.c
 * @author      RobotPilots@2020
 * @brief       Drivers' Manager.
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 *  
 * @Version     V1.0
 * @date        9-September-2020
 ****************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"
#include "usart.h"
#include "WL_debug_uart.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DRIVER_Init(void)
{
	USART1_Init();
	USART5_Init();
	USART7_Init();
	USART10_Init();
	USART8_Init();
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	CAN3_Filter_Init();
}

int RP_Log_Transmit(const uint8_t *data, uint16_t length)
{
    if (HAL_UART_Transmit_DMA(&huart7, data, length) == HAL_OK) {
        return 0;
    }
    return -1;
}

