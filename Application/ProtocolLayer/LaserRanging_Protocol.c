#include "LaserRanging_Protocol.h"
#include "drv_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t CommandFrame[8];// 命令格式数组
uint8_t rx_index=0;

extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart7;

extern void LaserRange_init(LaserRanging_t *LaserRanging);
extern void LaserRange_update(LaserRanging_t *LaserRanging,uint8_t *rxBuf);
extern void LaserRange_check(LaserRanging_t *LaserRanging);
extern void LaserRange_heart_beat(LaserRanging_t *LaserRanging);
extern void LaserRange_heart_beat_list();

LaserRanging_t LaserRanging[]=
{
	[LaserRange_R]=
	{
		.work_state = DEV_OFFLINE,
		.offline_cnt =  0,
		.offline_max_cnt = 60,
		.init = LaserRange_init,
		.update=LaserRange_update,
		.check = LaserRange_check,
		.heart_beat=LaserRange_heart_beat,
	},
	[LaserRange_L]=
	{
		.work_state = DEV_OFFLINE,
		.offline_cnt = 0,
		.offline_max_cnt = 60,
		.init = LaserRange_init,
		.update = LaserRange_update,
		.check = LaserRange_check,
		.heart_beat = LaserRange_heart_beat,
	},
};



uint16_t Modbus_CRC(uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 8; i!= 0; i--)
    {
      if ((crc & 0x0001)!= 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}


void LaserRange_init(LaserRanging_t *LaserRanging)
{
	LaserRanging->work_state=DEV_OFFLINE;
	LaserRanging->offline_cnt=LaserRanging->offline_max_cnt+1;
}

void LaserRange_update(LaserRanging_t *LaserRanging,uint8_t *rxBuf)
{
	LaserRanging->offline_cnt=0;
	uint16_t Length = 9;// sizeof(rxBuf);
	uint16_t crc;
	if(rxBuf[0] != 0x01)
	{
		return;
	}
    // 提取 LaserRanging 从机地址和功能码
    LaserRanging->LaserRanging_receive.Addr = (uint16_t)rxBuf[0];
    LaserRanging->LaserRanging_receive.Code = (uint16_t)rxBuf[1];
	
	LaserRanging->LaserRanging_receive.CRC_low = (uint16_t)rxBuf[7];
	LaserRanging->LaserRanging_receive.CRC_high = (uint16_t)rxBuf[8];
	crc = (LaserRanging->LaserRanging_receive.CRC_high << 8) | LaserRanging->LaserRanging_receive.CRC_low;
	
    // 校验 CRC
    if(crc != Modbus_CRC(rxBuf, 7))
	{
		LaserRanging->LaserRanging_receive.Distance = 0;
    }

    // 根据功能码解析数据
   if(LaserRanging->LaserRanging_receive.Code == 0x03) {
            // 提取距离
        LaserRanging->LaserRanging_receive.Distance = (float)((rxBuf[3] << 8) | rxBuf[4]);
		LaserRanging->LaserRanging_receive.Current_Confidence_Level = (uint16_t)((rxBuf[5] << 8) | rxBuf[6]);
   }
	if(LaserRanging->LaserRanging_receive.Current_Confidence_Level == 0)//超过4300mm量程时传感器数据会跳变为1300~1800，强行设置此时值为5000
	{
		LaserRanging->LaserRanging_receive.Distance = 5000.f * 0.001f;//后续处理将5000的点去除
	}
	else
	{
		LaserRanging->LaserRanging_receive.Distance *= 0.001f;//转换为单位m
	}
}

void LaserRange_check(LaserRanging_t *LaserRanging)
{
	
}


void Send_Read_Command_L(void)//发送右测距模块读取命令
{
	CommandFrame[0] = 0x01; 
	CommandFrame[1] = 0x03;  //功能码0x03用于读取保持寄存器
	CommandFrame[2] = 0x00;  //寄存器起始地址高字节
	CommandFrame[3] = 0x03;  //寄存器起始地址低字节
	CommandFrame[4] = 0x00;  //寄存器数量高字节
	CommandFrame[5] = 0x02;  //寄存器数量低字节

  // 计算CRC校验码并填充到缓冲区
  uint16_t crc = Modbus_CRC(CommandFrame, 6);
  CommandFrame[6] = (uint8_t)(crc & 0xFF);
  CommandFrame[7] = (uint8_t)(crc >> 8);

  // 通过串口发送数据

  HAL_UART_Transmit(&huart7, CommandFrame, sizeof(CommandFrame), 100);
  
}

void Send_Read_Command_R(void)//发送读取命令
{
	CommandFrame[0] = 0x01; 
	CommandFrame[1] = 0x03;  //功能码0x03用于读取保持寄存器
	CommandFrame[2] = 0x00;  //起始地址高字节
	CommandFrame[3] = 0x03;  //起始地址低字节
	CommandFrame[4] = 0x00;  //寄存器数量高字节
	CommandFrame[5] = 0x02;  //寄存器数量低字节

  // 计算CRC校验码并填充到缓冲区
  uint16_t crc = Modbus_CRC(CommandFrame, 6);
  CommandFrame[6] = (uint8_t)(crc & 0xFF);
  CommandFrame[7] = (uint8_t)(crc >> 8);

  // 通过串口发送数据
  HAL_UART_Transmit(&huart10, CommandFrame, sizeof(CommandFrame),100);

}


void LaserRange_heart_beat(LaserRanging_t *LaserRanging)
{
	LaserRanging->offline_cnt++;
	if(LaserRanging->offline_cnt>LaserRanging->offline_max_cnt)
	{
		LaserRanging->offline_cnt=LaserRanging->offline_max_cnt;
		LaserRanging->work_state=DEV_OFFLINE;
	}
	else
	{
			LaserRanging->work_state=DEV_ONLINE;
	}
}

void LaserRange_heart_beat_list()
{
	LaserRange_heart_beat(&LaserRanging[LaserRange_R]);
	LaserRange_heart_beat(&LaserRanging[LaserRange_L]);
}

void USART7_rxDataHandler(uint8_t *rxBuf)
{
	LaserRanging[LaserRange_L].update(&LaserRanging[LaserRange_L],rxBuf);
}

void USART10_rxDataHandler(uint8_t *rxBuf)
{
	LaserRanging[LaserRange_R].update(&LaserRanging[LaserRange_R],rxBuf);
}
