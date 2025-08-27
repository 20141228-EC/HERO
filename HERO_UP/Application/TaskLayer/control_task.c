/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "rp_user_config.h"
#include "rp_config.h"	
#include "chassis.h"

void StartControlTask(void const * argument)
{
	Car_Init();
	 
	for(;;) 

	{
		
		Car_Ctrl(&car);
		Car_Work();
		
		CAN_Send();
		#ifdef TEST_TASK

	
		#endif
	
	 
		osDelay(1);
	}
}



