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
#include "can_protocol.h"	
#include "chassis.h"
#include "ui.h"
#include "priority_ui.h"
 
void StartControlTask(void const * argument)
{
 
	for(;;)
	{
			
		Device_Work();
		CAN_Send();
		Ui_Info_Update();
		Ui_Send();
		#ifdef TEST_TASK
		
		#endif
		osDelay(1);
	}
} 




