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
#include "main.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//#ifdef IS_SIX_FRIC
//uint16_t val_up = 2500;//倍镜抬起CCR
//uint16_t val_down = 1350;//倍镜放下CCR
//#else
//uint16_t val_up = 1200;//倍镜抬起CCR
//uint16_t val_down = 760;//倍镜放下CCR
//#endif
//uint16_t val_slep = 0;//倍镜卸力
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

//    // 必须加上这段！配置TIM4_CH3为PWM模式
//    TIM_OC_InitTypeDef sConfigOC = {0};
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 0;  // 初始占空比为0，不响
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//    {
//        Error_Handler();
//    }	
//	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
/* Servo functions */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
// 500<CCR<2500，对应0~180°
//void Telescope_Up(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_up);
//}

//void Telescope_Down(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_down);
//}

//void Telescope_Sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, val_slep);
//}


