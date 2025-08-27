#include "can_protocol.h"

uint8_t Cap_Buff0x2F[8], Cap_Buff0x2E[8];
uint8_t cap_data_tx_buf[8];

void CAN_SendAll()
{
	//CAN1_CMD_200();
	CAN2_CMD_200();
	CAN1_CMD_1FF();
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
	
	
	Car_Data_Tx();
	if(RC_ONLINE)
	{
		for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
		{
			Motor_SendData(&rm_motor[i],rm_motor[i].base_info.motor_out);
		}
		kt_motor[0].W_iqControl(&kt_motor[0],kt_motor[0].base_info.motor_out);
		Chassis_Data_Tx(); //函数内部写了开控才发
	}
	else
	{
		for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
		{
			Motor_SendData(&rm_motor[i],0);
		}
		kt_motor[0].W_iqControl(&kt_motor[0],0);
	}

		kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID); //YAW
		CAN2_CMD_200();  //四个摩擦轮
		CAN1_CMD_1FF();	 // DAIL 
		CAN2_CMD_1FF(); //接收：205 PITCH ，206 LIMIT, 207 IMAGE
		
}


/**
 *  @brief  CAN1 接收数据
 */
uint32_t rx_Id;
uint16_t freq;
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_YAW:
        {
            kt_motor[0].get_info(&kt_motor[0], rxBuf);
			break;
        }
		case 0x00:
		{
			ht_motor.get_info(&ht_motor,rxBuf);
			rx_Id=rxId;
			freq++;
			break;
		}
		case ID_DAIL:
		{
			rm_motor_update(&rm_motor[DAIL], rxBuf);
			rm_motor_check(&rm_motor[DAIL]);
			break;
		}
		case GAME_ROBOT_STATUS_RX_ID:
			Game_Robot_Status_Rx(rxBuf);
			break;
		case POWER_HEAT_DATA_RX_ID:
			Power_Heat_Data_Rx(rxBuf);
			break;
		case SHOOT_DATA_RX_ID:
			Shoot_Data_Rx(rxBuf);
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
		case ID_FRIC_RF:
		{
			rm_motor_update(&rm_motor[FRIC_RF], rxBuf); //摩擦轮右前
			rm_motor_check(&rm_motor[FRIC_RF]);
			break;
		}
		case ID_FRIC_LB:
		{
			rm_motor_update(&rm_motor[FRIC_LB], rxBuf);  //摩擦轮左后
			rm_motor_check(&rm_motor[FRIC_LB]);
			break;
		}
		case ID_FRIC_RB:
		{
			rm_motor_update(&rm_motor[FRIC_RB], rxBuf);  //右后摩擦轮
			rm_motor_check(&rm_motor[FRIC_RB]);
			break;
		}
		case ID_FRIC_LF:
		{
			rm_motor_update(&rm_motor[FRIC_LF], rxBuf);  //左前摩擦轮
			rm_motor_check(&rm_motor[FRIC_LF]);
			break;
		}
		case ID_GIMB_P:
		{
			rm_motor_update(&rm_motor[GIMB_P], rxBuf);  //pitch轴
			rm_motor_check(&rm_motor[GIMB_P]);
			break;
		}
		case ID_LIMIT: 						
		{
			rm_motor_update(&rm_motor[LIMIT], rxBuf);	//主动限位
			rm_motor_check(&rm_motor[LIMIT]);
			break;
		}
		default:
			break;
	}
}
