#include "led_task.h"
#include "usart.h"
#include "driver.h"
#include "drv_uart.h"
#include "main.h"

uint8_t data[1];


void StartLedTask(void const * argument)
{
	
  for(;;)
  {		
		Vision_led_work();
		led_work(&led);
		
		osDelay(1);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
		if(GPIO_PIN==GPIO_PIN_0)
		{
			//HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
			
		}
}

