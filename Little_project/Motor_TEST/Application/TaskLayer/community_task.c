/**
  ******************************************************************************
  * File Name          : community_task.c
  * Description        : 其他设备控制，如键盘，视觉等
  ******************************************************************************
  */
#include "community_task.h"

uint8_t slave_data_tx_buf[8]={0,1,2,3};
uint8_t data_1[4]={0,1,2,3};
void slave_data_send(uint8_t can_num);
int16_t remain_bullet = 650;

void StartCommunityTask(void const * argument)
{

	static uint8_t vision_delay_cnt = 1;
	for(;;)
    {
		Car_Communicate_Info_Update(&car);
		if(vision_delay_cnt >= 1)
		{
			Vision_DataTx(&huart1);
			vision_delay_cnt = 1;
		}
		else
		{
			vision_delay_cnt++;
		}
		keyboard_update(rc_sensor.info);

		osDelay(1);
	}
}
