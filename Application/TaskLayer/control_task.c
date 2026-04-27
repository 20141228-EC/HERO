/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
//float t;
void StartControlTask(void const * argument)
{
//RC_ResetData(&rc_sensor);

	for(;;) 
	{
//		rc_sensor.check(&rc_sensor);
		Car_Ctrl(&car) ;
		
    Car_Work();
		
		CAN_BOARD_send();
		
//		/////////////test////////////自己加上的
//		
//		Board_Tx_Send_Data();
//				
		osDelay(1);
	}
}



