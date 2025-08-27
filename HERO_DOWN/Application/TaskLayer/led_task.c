#include "led_task.h"
#include "usart.h"
#include "driver.h"
#include "drv_uart.h"
#include "main.h"

uint8_t data[1];
uint8_t key_num=0;

void StartLedTask(void const * argument)
{
	
  for(;;)
  {		

		
		//data[0]=usart1_dma_rxbuf[0];
// 		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
// 		osDelay(100);
// 		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
//		osDelay(100);

		osDelay(1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
		if(GPIO_PIN==GPIO_PIN_0)
		{
			//HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
			key_num=1;
		}
}

