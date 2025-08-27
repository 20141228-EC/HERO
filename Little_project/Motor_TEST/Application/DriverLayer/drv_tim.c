/**
 ******************************************************************************
 * @file    drv_tim.c
 * @brief   定时器驱动
 ******************************************************************************
 * @attention
 * 
 * Copyright 2024 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t val_up = 2500;
uint16_t val_down = 2170;
uint16_t val_slep = 0;
/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void TIM1_Init(void)
{
	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void TIM4_Init(void)
{
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
}
/* Servo functions */
// 500<CCR<2500，对应0~180°
void Telescope_Up(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_up);
}

void Telescope_Down(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_down);
}

void Telescope_Sleep(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_slep);
}


