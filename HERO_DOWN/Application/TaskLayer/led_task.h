#ifndef __LED_TASK
#define __LED_TASK

#include "cmsis_os.h"
#include "main.h"

#define  LED_PORT   	 GPIOH
#define  LED_BLUE_PIN    GPIO_PIN_10
#define  LED_GREEN_PIN   GPIO_PIN_11
#define  LED_RED_PIN     GPIO_PIN_12



extern int16_t round_num;
extern uint8_t key_num;
void   StartLedTask(void const * argument);
extern IWDG_HandleTypeDef hiwdg;

#endif
