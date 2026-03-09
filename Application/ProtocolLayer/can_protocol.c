#include "can_protocol.h"
#include "Chassis_Motor.h"
#include "gimbal_Motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"

#ifdef UART_COMMUNICATE

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x044:
		Sd_Group.motor[R_F_Sd_M]->rx(Sd_Group.motor[R_F_Sd_M], rxBuf);
		break;
		case 0x033:
		Sd_Group.motor[R_B_Sd_M]->rx(Sd_Group.motor[R_B_Sd_M], rxBuf);
		break;		
		case 0x201:
		Wheel_Group.motor[R_WHEEL_M]->rx(Wheel_Group.motor[R_WHEEL_M], rxBuf);
		break;
		default:
			break;
	}
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x011:      //0x011:   //
		Sd_Group.motor[L_F_Sd_M]->rx(Sd_Group.motor[L_F_Sd_M], rxBuf);
		break;
		case 0x022:      //0x013:   //
		Sd_Group.motor[L_B_Sd_M]->rx(Sd_Group.motor[L_B_Sd_M], rxBuf);
		break;
		case 0x202:
 		Wheel_Group.motor[L_WHEEL_M]->rx(Wheel_Group.motor[L_WHEEL_M], rxBuf);
		break;
		
		default:
			break;
	}
}


/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x11:
			Yaw_Motor.rx(&Yaw_Motor,rxBuf);
		  break;
		case 0x88:
		  Dail_Motor.rx(&Dail_Motor,rxBuf);
		  break;
		case MASTER_CAN_RX_ID:
			Cap_Rx_Data_Update(&Cap_Rx_Info,rxBuf);
		  break;
		default:
			break;
	}
}

#else

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x044:
		Sd_Group.motor[R_F_Sd_M]->rx(Sd_Group.motor[R_F_Sd_M], rxBuf);
		break;
		case 0x033:
		Sd_Group.motor[R_B_Sd_M]->rx(Sd_Group.motor[R_B_Sd_M], rxBuf);
		break;		
		case 0x201:
		Wheel_Group.motor[R_WHEEL_M]->rx(Wheel_Group.motor[R_WHEEL_M], rxBuf);
		break;
		case 0x011:
		Yaw_Motor.rx(&Yaw_Motor,rxBuf);//确定
		break;
		default:
			break;
	}
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x011:      //0x011:   //
		Sd_Group.motor[L_F_Sd_M]->rx(Sd_Group.motor[L_F_Sd_M], rxBuf);
		break;
		case 0x022:      //0x013:   //
		Sd_Group.motor[L_B_Sd_M]->rx(Sd_Group.motor[L_B_Sd_M], rxBuf);
		break;
		case 0x202:
 		Wheel_Group.motor[L_WHEEL_M]->rx(Wheel_Group.motor[L_WHEEL_M], rxBuf);
		break;
		case 0x88:
		Dail_Motor.rx(&Dail_Motor,rxBuf);
		break;
		default:
			break;
	}
}


/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case MASTER_CAN_RX_ID:
			Cap_Rx_Data_Update(&Cap_Rx_Info,rxBuf);
		  break;
		case 0xC1:
			Board_Rx_D1(rxBuf);
		break;
		case 0xC2:
			Board_Rx_D2(rxBuf);
		break;
		default:
			break;
	}
}

#endif