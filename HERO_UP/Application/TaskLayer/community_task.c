#include "community_task.h"

void StartCommunityTask(void const *argument)
{

	static uint8_t vision_delay_cnt = 1;
	for (;;)
	{
		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
		Car_Communicate_Info_Update(&car);
		keyboard_update(rc_sensor.info); // 键鼠状态检测

		if (vision_delay_cnt >= 1)
		{
			Vision_DataTx(&huart1);
			vision_delay_cnt = 1;
		}
		else
		{
			vision_delay_cnt++;
		}

		
		osDelay(1);
	}
}
