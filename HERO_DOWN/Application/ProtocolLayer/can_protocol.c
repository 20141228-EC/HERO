#include "can_protocol.h"

uint8_t Cap_Buff0x2F[8], Cap_Buff0x2E[8];
uint8_t cap_data_tx_buf[8];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN_SendAll()
{
	CAN1_CMD_200();
	CAN2_CMD_200();
	//CAN1_CMD_1FF();
	CAN2_CMD_1FF();
}

void CAN_SendAllZero()
{
	memset(CAN1_200_DATA, 0, sizeof(CAN1_200_DATA));
	//memset(CAN1_1FF_DATA, 0, sizeof(CAN1_1FF_DATA));
	memset(CAN2_200_DATA, 0, sizeof(CAN2_200_DATA));
	memset(CAN2_1FF_DATA, 0, sizeof(CAN2_1FF_DATA));
	
	CAN_SendAll();
}

void CAN_Send(void)
{
	
	CAP_txMessage();

	if(RC_ONLINE)
	{
		for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
		{
			Motor_SendData(&rm_motor[i],rm_motor[i].base_info.motor_out);
		}	}
	else
	{
		for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
		{
			Motor_SendData(&rm_motor[i],0);
		}
	}
		CAN2_CMD_200();  //底盘

}

/**
 * @brief 电容CAN发送
 * @param hcan can1 can2
 */
void CAP_txMessage(void)
{
	static uint8_t online_cnt = 0; 
	//CAP_OFFLINE不发
	if(cap.state == CAP_OFFLINE)
	{
		online_cnt = 0;
		return;
	}
	//online超过100ms发
	if(online_cnt <= 100)
	{
		online_cnt ++;
		return;
	}

	static uint8_t cap_send_cnt = 1;
	cap_send_cnt ++;

	//裁判系统包
	if(cap.judge_pack_state == PACK_HAVE_UPDATED)
	{
		capboard_tx_info_t *tx_info = &cap.info.cap_tx_data;
		uint8_t tx_buff[8];
		uint32_t txMailBox;
		memcpy(tx_buff, tx_info, sizeof(capboard_tx_info_t));
		HAL_CAN_TxHeadeInit(cap.info.can_tx_Id);//配置ID
		HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeadeType,tx_buff,&txMailBox);
		cap.judge_pack_state = PACK_HAVE_NOT_UPDATED;
		return;
	}
 

}
/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case CHASSIS_DATA_RX_ID:
			Chassis_Data_Rx(rxBuf);
		break;	
		
		case CAR_DATA_RX_ID:
			Car_Data_Rx(rxBuf);
		break;
		
		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		case ID_CHAS_LF:
		{
			rm_motor_update(&rm_motor[CHAS_LF], rxBuf);
			rm_motor_check(&rm_motor[CHAS_LF]);
			break;
		}
		case ID_CHAS_LB:
		{
			rm_motor_update(&rm_motor[CHAS_LB], rxBuf);
			rm_motor_check(&rm_motor[CHAS_LB]);
			break;
		} 
		case ID_CHAS_RF:
		{
			rm_motor_update(&rm_motor[CHAS_RF], rxBuf);
			rm_motor_check(&rm_motor[CHAS_RF]);
			break;
		}
		case ID_CHAS_RB:
		{
			rm_motor_update(&rm_motor[CHAS_RB], rxBuf);
			rm_motor_check(&rm_motor[CHAS_RB]);
			break;
		}
		default:
			break;
	}
	cap.update(&cap,canId,rxBuf);
}
