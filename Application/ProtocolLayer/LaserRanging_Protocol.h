#ifndef __LASERRANGING_PROTOCOL_H
#define __LASERRANGING_PROTOCOL_H

#include "stm32H7xx_hal.h"
#include "rp_device_config.h"

typedef enum 
{
	LaserRange_R,//
	LaserRange_L,//
	LaserRange_List

} LaserRange_list_e;

typedef struct LaserRanging_command
{
	uint16_t addr;
	uint16_t code;
	uint16_t startaddr_high;
	uint16_t startaddr_low;
	uint16_t NUM_high;
	uint16_t NUM_low;
	uint16_t CRC_high;
	uint16_t CRC_low;
}LaserRanging_command_t;

typedef struct LaserRanging_send
{
	uint16_t addr;
	uint16_t code;
	uint16_t startaddr_high;
	uint16_t startaddr_low;
	uint16_t data_high;
	uint16_t data_low;
	uint16_t CRC_high;
	uint16_t CRC_low;
}LaserRanging_send_t;

typedef struct LaserRanging_receive
{
	uint16_t Addr;
	uint16_t Code;
	float Distance;//单位：m
	uint16_t Current_Confidence_Level;//当前可信度
	uint16_t Set_Confidence_Level;//设置可信度
	uint16_t Threshold;//阈值
	uint16_t Valid_Flag;//有效标志	
	uint16_t Output_Setting;//输出设置	
	uint16_t LED_Mode;//LED 模式	
	uint16_t POF;//开关机
	uint16_t FPS;//帧率0 = 4800, 1 = 9600 （默认）, 2 = 115200

	uint16_t Startup_Calibration;//启动校准
	uint16_t Calibration_Position1;//校准位置 1
	uint16_t Calibration_Position2;//校准位置 2
	uint16_t Calibration_Flag;//校准标志

	uint16_t CRC_high;
	uint16_t CRC_low;
}LaserRanging_receive_t;

typedef struct LaserRanging
{
	LaserRanging_send_t LaserRanging_send;
	LaserRanging_receive_t  LaserRanging_receive;
	
	uint16_t offline_cnt;
	uint16_t offline_max_cnt;
	dev_work_state_t	work_state;
	void					   	(*update)(struct LaserRanging *self, uint8_t *rxBuf);
	void					    (*check)(struct LaserRanging *self);	
	void					    (*heart_beat)(struct LaserRanging *self);
	void					    (*init)(struct LaserRanging *self);
	void 						(*heart_beat_list)(struct LaserRanging *self);
}LaserRanging_t;
//extern Modbus_t Modbus;


extern LaserRanging_t LaserRanging[LaserRange_List];


void Send_Read_Command_L(void);
void Send_Read_Command_R(void);


void LaserRange_heart_beat_list(void);
#endif
