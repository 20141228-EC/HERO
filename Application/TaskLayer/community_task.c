#include "community_task.h"
#include "imu_sensor.h"
#include "vision_protocol.h"
#include "usart.h"

void StartCommunityTask(void const *argument)
{


	for (;;)
	{
//		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
//		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
//		keyboard_update(rc_sensor.info); // 键鼠状态检测
		if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
				(imu_sensor.work_state.err_code == IMU_DATA_CALI))
		{
			imu_sensor.update(&imu_sensor);
			
		}
		
		Vision_Board_Update();

		Vision_DataTx(&huart1);
		osDelay(1);
	}
}
