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
		#ifdef LOB_CHASSIS_NO_POWER
		if(car.car_move_mode!=lob_CAR||command[AUTO_LOB].cmd_status==RUNING_C)
		{
			Chassis_Data_Tx(); //函数内部写了开控才发
		}
		#else
			Chassis_Data_Tx(); //函数内部写了开控才发
		#endif
		
	}
	else
	{
		for(uint8_t i=0;i<RM_MOTOR_LIST;i++)
		{
			if(i==FRIC_B_UP||i==FRIC_B_R||i==FRIC_B_L
				||i==FRIC_F_UP||i==FRIC_F_R||i==FRIC_F_L)
			{
				Motor_SendData(&rm_motor[i],rm_motor[i].base_info.motor_out);
			}
			else
			{
				Motor_SendData(&rm_motor[i],0);
			}
		}
		kt_motor[0].W_iqControl(&kt_motor[0],0);
	}

		kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID); //YAW
		//摩擦轮和pitch
		CAN2_CMD_200();  // 
		CAN2_CMD_1FF(); // 
	    //拨盘和图传电机
		CAN1_CMD_1FF();	 //  
		
		
}


/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_YAW:
        {
            kt_motor[0].get_info(&kt_motor[0], rxBuf);
			break;
        }
		case ID_IMAGE:
        {
            rm_motor_update(&rm_motor[IMAGE], rxBuf);
			rm_motor_check(&rm_motor[IMAGE]);
			break;
        }
		case ID_DAIL:
		{
			rm_motor_update(&rm_motor[DAIL], rxBuf);
			rm_motor_check(&rm_motor[DAIL]);
			break;
		}
		case ID_TELESCOPE:
		{
			rm_motor_update(&rm_motor[TELESCOPE], rxBuf);
			rm_motor_check(&rm_motor[TELESCOPE]);
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
		case GAME_ROBOT_POS_RX_ID:
			Game_robot_pos_Rx(rxBuf);
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
		case ID_FRIC_F_UP:
		{
			rm_motor_update(&rm_motor[FRIC_F_UP], rxBuf);  
			rm_motor_check(&rm_motor[FRIC_F_UP]);
			break;
		}
		case ID_FRIC_F_L:
		{
			rm_motor_update(&rm_motor[FRIC_F_L], rxBuf);   
			rm_motor_check(&rm_motor[FRIC_F_L]);
			break;
		}
		case ID_FRIC_F_R:
		{
			rm_motor_update(&rm_motor[FRIC_F_R], rxBuf);   
			rm_motor_check(&rm_motor[FRIC_F_R]);
			break;
		}
		case ID_FRIC_B_UP:                
		{
			rm_motor_update(&rm_motor[FRIC_B_UP], rxBuf);   
			rm_motor_check(&rm_motor[FRIC_B_UP]);
			break;
		}
		case ID_FRIC_B_L:
		{
			rm_motor_update(&rm_motor[FRIC_B_L], rxBuf);  
			rm_motor_check(&rm_motor[FRIC_B_L]);
			break;
		}
		case ID_FRIC_B_R:
		{
			rm_motor_update(&rm_motor[FRIC_B_R], rxBuf);   
			rm_motor_check(&rm_motor[FRIC_B_R]);
			break;
		}
		case ID_GIMB_P:
		{
			rm_motor_update(&rm_motor[GIMB_P], rxBuf);  //pitch轴
			rm_motor_check(&rm_motor[GIMB_P]);
			break;
		}
		default:
			break;
	}
}
