#include "communicate.h"
#include "string.h"
#include "crc.h"
#include "drv_can.h"
//#include "usbd_cdc_if.h"

#ifdef UART_COMMUNICATE

//下主控D开头，上主控C开头，数字对应
 Board_Tx_Info_t Board_Tx_Info;
 Board_Rx_Info_t Board_Rx_Info;
Board_HeartBeat_t Board_HeartBeat=
{
  .offline_cnt = 0,
	.offline_cnt_max = 50,
};

Board_Tx_Info_t Board_D_Tx_Info = 
{
	.SOF = 0xA5,
};
Board_Rx_Info_t Board_D_Rx_Info;
uint8_t Board_D_TxBuf[80];

bool Board_Tx_Send_Data(void)
{
	memcpy(Board_D_TxBuf, &Board_D_Tx_Info, sizeof(Board_Tx_Info_t));
		
	Append_CRC8_Check_Sum(Board_D_TxBuf, 3);
		
	Append_CRC16_Check_Sum(Board_D_TxBuf, sizeof(Board_Tx_Info_t));
	
	if(HAL_UART_Transmit_DMA(&huart10,Board_D_TxBuf,sizeof(Board_Tx_Info_t)) == HAL_OK)
	{
			return true;
	}
	return false;
}

bool Board_D_Recieve_Data(uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(Board_Rx_Info_t)) == true)
			{
				memcpy(&Board_D_Rx_Info, rxBuf, sizeof(Board_Rx_Info_t));
				Board_HeartBeat.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
	Board_Tx_Info->gimbal_mode = gimbal.gimbal_ctrl_mode.gimbal_mode;
	Board_Tx_Info->gimbal_state = gimbal.gimbal_reset_state;//?
	Board_Tx_Info->is_ready_shoot = shoot_out.base_info.is_enable_shoot;
	Board_Tx_Info->pitch_imu_tar = gimbal.base_info.pitch_imu_angle_target;
	Board_Tx_Info->pitch_mec_tar = gimbal.base_info.pitch_mec_angle_target;
  Board_Tx_Info->yaw_imu_tar = gimbal.base_info.yaw_imu_angle_target;
	Board_Tx_Info->is_fric_on = shoot_out.fric.is_fric_on;
  Board_Tx_Info->blood_1 = My_Judge.org_info->ext_game_robot_HP.ally_1_robot_HP;
	Board_Tx_Info->blood_2 = My_Judge.org_info->ext_game_robot_HP.ally_2_robot_HP;
	Board_Tx_Info->blood_3 = My_Judge.org_info->ext_game_robot_HP.ally_3_robot_HP;
	Board_Tx_Info->blood_4 = My_Judge.org_info->ext_game_robot_HP.ally_4_robot_HP;
	Board_Tx_Info->blood_7 = My_Judge.org_info->ext_game_robot_HP.ally_7_robot_HP;
	float angle_err =gimbal.base_info.yaw_motor_angle - YAW_MOTOR_ANGLE_MIDDLE;
	if(my_abs(angle_err) > PI)
	{
		angle_err -= 2*PI;
	}
//	Board_Tx_Info->v_x = arm_cos_f32(angle_err) * speed_result;
//	Board_Tx_Info->v_y = arm_sin_f32(angle_err) * speed_result;
//	Board_Tx_Info->is_on_lob = 
	//	Board_Tx_Info->my_color = 
//	Board_Tx_Info->is_handle_shoot = 

//	Board_Tx_Info->vision_mode = 
//	Board_Tx_Info->video_open = 

}

void USART8_rxDataHandler(uint8_t *rxBuf)
{
	Board_D_Recieve_Data(rxBuf);
}

#else

 Board_Tx_Info_t Board_Tx_Info;
 Board_Rx_Info_t Board_Rx_Info;
Board_HeartBeat_t Board_HeartBeat=
{
  .offline_cnt_1 = 0,
	.offline_cnt_2 = 0,
	.offline_cnt_max = 50,
};

uint8_t board_tx_buf_1[8];
uint8_t board_tx_buf_2[8];
uint8_t board_tx_buf_3[8];
uint8_t board_tx_buf_4[8];
void Board_Tx_D1(void)
{
	uint16_t pitch_tar_temp,yaw_tar_temp;
	
	pitch_tar_temp = float_to_uint(Board_Tx_Info.pitch_imu_tar,-360.f,360.f,16);
	yaw_tar_temp = float_to_uint(Board_Tx_Info.yaw_mec_imu,-360.f,360.f,16);
	
	board_tx_buf_1[0] = (pitch_tar_temp>>8);
	board_tx_buf_1[1] = pitch_tar_temp;
	board_tx_buf_1[2] = (yaw_tar_temp>>8);
	board_tx_buf_1[3] = yaw_tar_temp;
	board_tx_buf_1[4] = Board_Tx_Info.gimbal_mode;
	board_tx_buf_1[5] = Board_Tx_Info.my_color;
	board_tx_buf_1[6] = Board_Tx_Info.video_open;
	board_tx_buf_1[7] = Board_Tx_Info.vision_mode;
	
	CAN_SendData(&hfdcan3,0xD1,board_tx_buf_1);
}

void Board_Tx_D2(void)
{
	uint16_t v_x_temp,v_y_temp;
	
	v_x_temp = float_to_uint(Board_Tx_Info.v_x,-10.f,10.f,16);
	v_y_temp = float_to_uint(Board_Tx_Info.v_y,-10.f,10.f,16);
	
	uint16_t pitch_mec_temp;
	pitch_mec_temp = float_to_uint(Board_Tx_Info.pitch_mec_tar,-PI,PI,16);
		
	uint8_t compressed = 0;  // 用来存储压缩后的结果
	// 将每个 bool 变量映射到 uint8_t 的不同位上
	compressed |= (Board_Tx_Info.gimbal_state << 0);  //  存放在最低位
	compressed |= (Board_Tx_Info.is_fric_on << 1);  //  存放在第 1 位
	compressed |= (Board_Tx_Info.is_ready_shoot << 2);  //  存放在第 2 位
	compressed |= (Board_Tx_Info.is_on_lob << 3);  //  存放在第 3 位
	compressed |= (Board_Tx_Info.is_handle_shoot << 4);  //  存放在第 4 位
	compressed |= (Board_Tx_Info.is_rc_online << 5);  //  存放在第 5 位
	
	board_tx_buf_2[0] = Board_Tx_Info.shoot_count;
	board_tx_buf_2[1] = compressed;
	board_tx_buf_2[2] = (v_x_temp>>8);
	board_tx_buf_2[3] = v_x_temp;
	board_tx_buf_2[4] = (v_y_temp>>8);
	board_tx_buf_2[5] = v_y_temp;
	board_tx_buf_2[6] = (pitch_mec_temp>>8);
	board_tx_buf_2[7] = pitch_mec_temp;
	
  CAN_SendData(&hfdcan3,0xD2,board_tx_buf_2);
}

void Board_Tx_D3(void)
{
	board_tx_buf_3[0] = Board_Tx_Info.blood_0;
	board_tx_buf_3[1] = Board_Tx_Info.blood_1;
	board_tx_buf_3[2] = Board_Tx_Info.blood_2;
	board_tx_buf_3[3] = Board_Tx_Info.blood_3;
	board_tx_buf_3[4] = Board_Tx_Info.blood_4;
	board_tx_buf_3[5] = Board_Tx_Info.blood_5;
	board_tx_buf_3[6] = Board_Tx_Info.blood_6;
	board_tx_buf_3[7] = Board_Tx_Info.blood_7;
	
	CAN_SendData(&hfdcan3,0xD3,board_tx_buf_3);
}

void Board_Tx_D4(void)
{
	uint16_t bullet;
	
	bullet = float_to_uint(Board_Tx_Info.bullet_speed,-20.f,20.f,16);
	
	board_tx_buf_4[0] = (bullet>>8);
	board_tx_buf_4[1] = bullet;
	
	uint16_t pitch_offset;
//	
	pitch_offset = float_to_uint(Board_Tx_Info.pitch_offset,-10.f,10.f,16);
//	mea = float_to_uint(Board_Tx_Info.mea,-10.f,10.f,16);

	board_tx_buf_4[2] = (pitch_offset>>8);
	board_tx_buf_4[3] = pitch_offset;
//	board_tx_buf_4[4] = (mea>>8);
//	board_tx_buf_4[5] = mea;
	
//	uint16_t pitch_offset;
//	
//	board_tx_buf_4[6] = (pitch_offset>>8);
//	board_tx_buf_4[7] = pitch_offset;

//	
//	pitch_offset = float_to_uint(Board_Tx_Info.pitch_offset,-30.f,30.f,16);
	
	
	CAN_SendData(&hfdcan3,0xD4,board_tx_buf_4);
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
	Board_Tx_Info->gimbal_mode = gimbal.gimbal_ctrl_mode.gimbal_mode;
	Board_Tx_Info->gimbal_state = gimbal.gimbal_reset_state;//?
	Board_Tx_Info->is_ready_shoot = shoot_out.base_info.is_enable_shoot;
	Board_Tx_Info->pitch_imu_tar = gimbal.base_info.pitch_imu_angle_target;
	Board_Tx_Info->pitch_mec_tar = gimbal.base_info.pitch_mec_angle_target;
	Board_Tx_Info->pitch_offset = gimbal.offset_info->lob_pitch_gyro_offset;
	if(gimbal.yaw_pid_mode != MEC_PID)
	{
		Board_Tx_Info->yaw_mec_imu = gimbal.base_info.yaw_imu_angle;//180
	}
	else
	{
    Board_Tx_Info->yaw_mec_imu = gimbal.base_info.yaw_motor_angle;//pi
	}
	Board_Tx_Info->is_fric_on = shoot_out.fric.is_fric_on;
	Board_Tx_Info->is_rc_online = rc_sensor.work_state;
	Board_Tx_Info->bullet_speed = My_Judge.org_info->shoot_data.initial_speed;
  Board_Tx_Info->blood_1 = My_Judge.org_info->ext_game_robot_HP.ally_1_robot_HP;
	Board_Tx_Info->blood_2 = My_Judge.org_info->ext_game_robot_HP.ally_2_robot_HP;
	Board_Tx_Info->blood_3 = My_Judge.org_info->ext_game_robot_HP.ally_3_robot_HP;
	Board_Tx_Info->blood_4 = My_Judge.org_info->ext_game_robot_HP.ally_4_robot_HP;
	Board_Tx_Info->blood_7 = My_Judge.org_info->ext_game_robot_HP.ally_7_robot_HP;
//	float angle_err =gimbal.base_info.yaw_motor_angle - YAW_MOTOR_ANGLE_MIDDLE;
//	if(my_abs(angle_err) > PI)
//	{
//		angle_err -= 2*PI;
//	}
//	Board_Tx_Info->v_x = arm_cos_f32(angle_err) * speed_result;
//	Board_Tx_Info->v_y = arm_sin_f32(angle_err) * speed_result;
	if(Balance.mode == Lob_Mode)
	{
   	Board_Tx_Info->is_on_lob = 1;
	}
	else
	{
		Board_Tx_Info->is_on_lob = 0;
	}
		Board_Tx_Info->my_color = My_Judge.info->car_color;
//	  Board_Tx_Info->is_handle_shoot = 
  
	if(Balance.Vision.Auto_Catch_Flag == true)
	{
	  Board_Tx_Info->vision_mode = 1;
	}
	else if(Balance.Chassis_Com->COMMON_OUTPOST_SHOOT == true)
	{
	  Board_Tx_Info->vision_mode = 2;
	}
	else if(Balance.Chassis_Com->COMMON_BASE_SHOOT == true)
	{
	  Board_Tx_Info->vision_mode = 3;
	}
	else
	{
	  Board_Tx_Info->vision_mode = 0;
	}
	
	if(Balance.Vision.Auto_Catch_Flag == 1 || Balance.Vision.Auto_Base_Flag == 1 ||Balance.Vision.Auto_Outpost_Flag == 1)
	{
	  Board_Tx_Info->video_open = 1;
	}
	else{
		Board_Tx_Info->video_open = 0;
	}
	
	Board_Tx_Info->mea = Chassis.chassis_PID->yaw_speed_cal[L_Leg]->measure;
	Board_Tx_Info->tar = Chassis.chassis_PID->yaw_speed_cal[L_Leg]->target;

}


void Board_Rx_D1(uint8_t *rxbuf)
{
	uint16_t pitch_imu_int,yaw_imu_int,yaw_v_int,pitch_mec_int;
	
	pitch_imu_int = (rxbuf[0]<<8 | rxbuf[1]);
	yaw_imu_int = (rxbuf[2]<<8 | rxbuf[3]);
	yaw_v_int = (rxbuf[4]<<8 | rxbuf[5]);
	pitch_mec_int = (rxbuf[6]<< 8 | rxbuf[7]);
	
	Board_Rx_Info.pitch_imu = uint_to_float(pitch_imu_int,-360.f,360.f,16);
	Board_Rx_Info.yaw_imu = uint_to_float(yaw_imu_int,-360.f,360.f,16);
	Board_Rx_Info.yaw_v = uint_to_float(yaw_v_int,-5000.f,+5000.f,16);//考虑到角速度可能会非常大，故把映射范围调大
	Board_Rx_Info.pitch_mec= uint_to_float(pitch_mec_int,-PI,PI,16);
	Board_HeartBeat.offline_cnt_1 = 0;
}

void Board_Rx_D2(uint8_t *rxbuf)
{
	uint16_t vision_pitch_int,vision_yaw_int,compressed;
	
	vision_pitch_int = (rxbuf[0]<<8 | rxbuf[1]);
	vision_yaw_int = (rxbuf[2]<<8 | rxbuf[3]);
	compressed = rxbuf[5];

	Board_Rx_Info.vision_pitch_tar = uint_to_float(vision_pitch_int,-180.f,180.f,16);
	Board_Rx_Info.vision_yaw_tar = uint_to_float(vision_yaw_int,-180.f,180.f,16);
	Board_Rx_Info.vision_state = rxbuf[4];
	Board_Rx_Info.detect_num = rxbuf[6];
//  Board_Rx_Info.launch_timer = (rxbuf[6]<<8 | rxbuf[7]);
	
//	Board_Rx_Info.hit_enable = (compressed >> 0) & 0x01;
//	Board_Rx_Info.is_find_base = (compressed >> 1) & 0x01;
//	Board_Rx_Info.is_find_outpost = (compressed >> 2) & 0x01;
//	Board_Rx_Info.is_find_Target = (compressed >> 3) & 0x01;
	Board_Rx_Info.hit_enable = (compressed & (1 << 0)) ? true : false;       // 只看bit0
Board_Rx_Info.is_find_base = (compressed & (1 << 1)) ? true : false;     // 只看bit1
Board_Rx_Info.is_find_outpost = (compressed & (1 << 2)) ? true : false;  // 只看bit2
Board_Rx_Info.is_find_Target = (compressed & (1 << 3)) ? true : false;   // 只看bit3
	Board_Rx_Info.is_fric_speed = (compressed & (1 << 4)) ? true : false; 
	Board_Rx_Info.is_fric_work = (compressed & (1 << 5)) ? true : false; 
	
	
	Board_HeartBeat.offline_cnt_2 = 0;
}

void Board_Rx_D3(uint8_t *rxbuf)
{
	uint16_t kp,kd;
	
	kp = (rxbuf[0]<<8 | rxbuf[1]);
	kd = (rxbuf[2]<<8 | rxbuf[3]);
	Board_Rx_Info.kp = uint_to_float(kp,-360.f,360.f,16);
	Board_Rx_Info.kd = uint_to_float(kd,-2000.f,2000.f,16);
//	Board_HeartBeat.offline_cnt_3 = 0;
	Board_Rx_Info.detect_num = rxbuf[4];
}

void CAN3_SEND(void)
{
	Board_Tx_Update(&Board_Tx_Info);
	Board_Tx_D1();
	Board_Tx_D2();
	Board_Tx_D3();
	Board_Tx_D4();
}

#endif

void D_Board_HeartBeat(void)
{
	Board_HeartBeat.offline_cnt_1++;
	Board_HeartBeat.offline_cnt_2++;
	if(Board_HeartBeat.offline_cnt_1 > Board_HeartBeat.offline_cnt_max || Board_HeartBeat.offline_cnt_2 > Board_HeartBeat.offline_cnt_max)
  {
    Board_HeartBeat.offline_cnt_1 = Board_HeartBeat.offline_cnt_max;
    Board_HeartBeat.offline_cnt_2 = Board_HeartBeat.offline_cnt_max;
    Board_HeartBeat.status = DEV_OFFLINE;
	
  }
  else if(Board_HeartBeat.status == DEV_OFFLINE)
  {
    Board_HeartBeat.status = DEV_ONLINE;
  }	
}


