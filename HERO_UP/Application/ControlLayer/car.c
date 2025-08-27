/* Includes ------------------------------------------------------------------*/
#include "car.h"
#include "rp_math.h"
#include "communicate_protocol.h"
#include "stdbool.h"
/* typedef--------------------------------------------------------------------*/
command_t command[COMMAND_LIST] = 
{
  [CAR_L90] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = 1000,  
	.init = Cmd_Class_Init,
	},
	[CAR_R90] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = 1000,
		.init = Cmd_Class_Init,  
	},
	[GIM_180] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = 1000,  
		.init = Cmd_Class_Init, 
		.Trigger_lock.Trigger_lock_on=1,
		.Trigger_lock.lock_time=500,
	},
	[GIM_UP] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init, 
	},
	[GIM_DOWM] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init, 
	},
	[GIM_RIGHT] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init, 
	},
	[GIM_LEFT] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init, 
	},
	//MARK:会在命令更新中判断是否进视觉模式改变命令类型
	[SHOOTING_FIRE] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,  
	},
	[SHOOTING_FIRING] = {
		.cmd_type = HIGH_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[TELESCOPE_SWITCH] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	
	[OPEN_SPEED_ADAPT] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[NORMAL_LOB] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max =500,
		.init = Cmd_Class_Init,
	},
	[AUTO_LOB] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = 2000,
		.init = Cmd_Class_Init,
	},
	[OBLIQUE_LOB] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = 350,
		.init = Cmd_Class_Init,
	},
	
	[IMAGE_SWITCH_MAX] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[IMAGE_SWITCH_MIN] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[CHANGE_LOB_PITCH_ANGLE] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[CAP_ON] = {
		.cmd_type = HIGH_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[KILL_MYSELF] = {
		.cmd_type = HIGH_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[TIMER_MEC_OUTPOST] = {
		.cmd_type = HIGH_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[TUNNEL_MODE] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},
	[PRE_CHARGE] = {
		.cmd_type = RISE_TRIGER_C,
		.run_time_max = OUT_TIME_OFF,
		.init = Cmd_Class_Init,
	},

};


Car_Ctrl_Mode_e Car_Ctrl_Mode;

car_t car = 
{
    .car_reset_state = DEV_RESET_NO,
	.init_cnt_max = CAR_INIT_TIME,
};

/**
 * @brief 控制模式更新
 * 
 * @param car 
 */
void Car_Ctrl_Mode_Update(car_t *car)
{
	//右拨杆上：键盘模式  其余都是遥控器模式
	switch(rc_sensor.info->s1.value)
	{
	case 2:	//左拨杆下
		
     car->car_ctrl_mode = KEY_CTRL_MODE;
		break;
	default://左拨杆不是下
		car->car_ctrl_mode = RC_CTRL_MODE;
		break;
	}
	
	//从遥控器模式转到键盘模式，进入陀螺仪模式
	switch (rc_sensor.info->s1.status)
	{
	case down_R://左拨杆下拨进入陀螺仪模式
		#ifdef OPEN_RC_LOCK
		if(car->unlock_car_flag==1)
		#endif
	
		#ifndef LOB_TEST  
		car->car_move_mode = gyro_CAR;
		#endif
		break;
	default:
		break;
	}

}
/**
 * @brief 初始化判断
 * 
 * @param car 
 */
void Car_Init_Judge(car_t *car)
{
	//判断各个设备是否初始化完成
	if(gimbal.gimbal_reset_state == DEV_RESET_OK&& \
		image.image_reset_state==DEV_RESET_OK)
	{
		car->car_reset_state = DEV_RESET_OK;
	}
	//初始化计时
	if (car->car_move_mode == init_CAR)
	{
		car->init_cnt ++;
	}
	//初始化超时退出
	if (car->init_cnt >= car->init_cnt_max)
	{
		car->car_reset_state = DEV_RESET_OK;
		
		car->init_cnt = car->init_cnt_max;
	}
}
/**
 * @brief 遥控模式整车移动模式更新
 * @param car 
 */

void RC_Move_Mode_Update(car_t *car)
{
	//放这里的原因是避免因为任务优先级出现问题导致错过
	static uint8_t thumbweheel_step,thumbweheel_last_step;
	thumbweheel_last_step = thumbweheel_step;
	thumbweheel_step = rc_sensor.info->thumbwheel.step[RC_TB_UP];
	if(rc_sensor.info->s2.status==up_R)  //右拨杆向上拨机械模式
		{
			#ifdef LOB_TEST //如果测吊射右拨杆拨到上面开吊射
		    car->car_move_mode = lob_CAR ;
		    #else
		    car->car_move_mode = mec_CAR ;
		    #endif
		}
	switch(rc_sensor.info->s2.value)
	{
	case 0x01:  //右拨杆上

		break;
	case 0x02:  //右拨杆下
		//拨轮向上切换视觉陀螺仪和视觉小陀螺模式
		if(thumbweheel_step!=thumbweheel_last_step)
		{

			if(car->car_move_mode == vision_gyro_CAR)
			{
				car->car_move_mode = vision_cycle_CAR;
			}
			else if(car->car_move_mode == vision_cycle_CAR)
			{
				car->car_move_mode = vision_gyro_CAR;
			}
		}
		if (car->car_move_mode != vision_cycle_CAR&&
			car->car_move_mode != vision_gyro_CAR)
		{
			car->car_move_mode = vision_gyro_CAR;
		}
		break;

	case 0x03:	//右拨杆中
		
		//拨轮向上切换陀螺仪和小陀螺
		if(thumbweheel_step!=thumbweheel_last_step)
		{
			if(car->car_move_mode == gyro_CAR)
			{
				car->car_move_mode = cycle_CAR;
			}
			else if(car->car_move_mode == cycle_CAR)
			{
				car->car_move_mode = gyro_CAR;
			}
		}
		
		//拨杆中间时，切换到陀螺仪模式
		if (car->car_move_mode != gyro_CAR &&
			car->car_move_mode != cycle_CAR)
		{
			car->car_move_mode = gyro_CAR;
		}
		break;
	default:
	 	break;	
	}
}

/**
 * @brief 键盘模式整车移动模式更新
 * 
 * @param car 
 */
void Key_Move_Mode_Update(car_t *car)
{
	//ctrl+v/c进吊射
	if ((command[AUTO_LOB].cmd_value == true||command[NORMAL_LOB].cmd_value == true||command[OBLIQUE_LOB].cmd_value == true)&&car->car_move_mode != lob_CAR)
	{
		car->car_move_mode = lob_CAR;
	}
	
	//按下X进手打前哨模式
	if (command[TIMER_MEC_OUTPOST].cmd_value == true)
	{
		car->car_move_mode = lob_CAR;
	}
	switch (car->car_move_mode)
	{
	case mec_CAR:
		//右键按下，进入视觉陀螺仪模式
		if(rc_sensor.info->mouse_btn_r.value == 1)
		{
			car->car_move_mode = vision_gyro_CAR;
		}
		//F键按下，进入小陀螺模式
		if(rc_sensor.info->F.status== release_to_press)
		{
			car->car_move_mode = cycle_CAR;
		}
		//shift按下，进入陀螺仪模式
		if (rc_sensor.info->Shift.status == release_to_press)
		{
			car->car_move_mode = gyro_CAR;
		}
		break;
	case gyro_CAR:
		//右键按下，进入视觉陀螺仪模式
		if(rc_sensor.info->mouse_btn_r.value == 1)
		{
			car->car_move_mode = vision_gyro_CAR;
		}
		//C键按下，进入机械模式
		if(rc_sensor.info->C.status== release_to_press)
		{
			car->car_move_mode = mec_CAR;
		}
		//F键按下，进入小陀螺模式
		if(rc_sensor.info->F.status== release_to_press)
		{
			car->car_move_mode = cycle_CAR;
		}
		
		break;
	case cycle_CAR:
		//C键按下，进入机械模式
		if(rc_sensor.info->C.status== release_to_press)
		{
			car->car_move_mode = mec_CAR;
		}
		//F键按下，进入陀螺仪模式
		if(rc_sensor.info->F.status== release_to_press||rc_sensor.info->Shift.status== release_to_press)
		{
			car->car_move_mode = gyro_CAR;
		}
		//右键按下进入视觉小陀螺模式
		if(rc_sensor.info->mouse_btn_r.status == release_to_press)
		{
			car->car_move_mode = vision_cycle_CAR;
		}
		break;
	
	case lob_CAR:
		//Shift键按下，进入陀螺仪模式
		if(rc_sensor.info->Shift.status == release_to_press&&car->car_move_mode!=lob_CAR) 
		{
			car->car_move_mode = gyro_CAR;
		}
		//鼠标右键按下，进入陀螺仪模式
		if(rc_sensor.info->mouse_btn_r.status == release_to_press) 
		{
			car->car_move_mode = gyro_CAR;
		}
		break;
	case vision_gyro_CAR:
		//右键松开，进入陀螺仪模式
		if(rc_sensor.info->mouse_btn_r.value == 0 )
		{
			car->car_move_mode = gyro_CAR;
		}
		//F键按下，进入视觉小陀螺模式
		if(rc_sensor.info->F.status == release_to_press)
		{
			car->car_move_mode = vision_cycle_CAR;
		}
		break;
	case vision_cycle_CAR:
		//右键松开，进入小陀螺模式
		if(rc_sensor.info->mouse_btn_r.value == 0)
		{
			car->car_move_mode = cycle_CAR;
		}
		//F键按下，进入视觉陀螺仪模式
		if(rc_sensor.info->F.status == release_to_press)
		{
			car->car_move_mode = vision_gyro_CAR;
		}
		break;
	default:
		break;
	}
}

/**
 * @brief 整车移动模式更新
 * 
 * @param car 
 */
 
void Car_Move_Mode_Update(car_t *car)
{
	Car_Init_Judge(car);//如果要取消超时退出进里面改
	 
	 
	//关控或者裁判系统断电进入offline_CAR
	if(RC_OFFLINE||(!IMAGE_MOTOR_ONLINE && !GIMBAL_MOTOR_ONLINE && !SHOOTING_MOTOR_ONLINE))
	{
		car->car_move_mode = offline_CAR;
		#ifdef OPEN_RC_LOCK
		car->unlock_car_flag=0;
		#endif
		car->car_reset_state = DEV_RESET_NO;
		//清空命令
		for(uint8_t i = 0; i < COMMAND_LIST; i++)
		{
			command[i].clean(&command[i]);
		}
		car->init_cnt = 0;//清空计时初始化计时时间，init_cnt用在Car_Init_Judge(car);里
	}
	#ifdef OPEN_RC_LOCK
		else if(car->unlock_car_flag==0)
		{
			if(UNLOCK_CAR_CONDITION)
			{
				HAL_Delay(500);
				car->unlock_car_flag=1;
			}
		}
	#endif
	else if(car->car_reset_state == DEV_RESET_NO)//没初始化完成，进入初始化
	{
		car->car_move_mode = init_CAR;
	}
	else if (car->car_move_mode == init_CAR)//初始化完了
	{
		car->init_cnt = 0;//初始化成功
		gimbal.base_info.yaw_imu_angle_target=gimbal.base_info.yaw_imu_angle;
		car->car_move_mode = gyro_CAR;//进入默认模式（陀螺仪模式）
	}
	else 
	{
		if (car->car_ctrl_mode == RC_CTRL_MODE)//遥控器模式
		{
			RC_Move_Mode_Update(car);//右拨杆控制移动模式
		}
		else//键盘模式 
		{
			Key_Move_Mode_Update(car);
		}
	}
}

/**
 * @brief 遥控器模式命令更新
 * 
 */
void RC_Command_Update(void)
{
 
	//视觉模式下自动打弹
	if(car.car_move_mode == vision_cycle_CAR ||
		 car.car_move_mode == vision_gyro_CAR)
	{
		command[SHOOTING_FIRE].cmd_type = HIGH_TRIGER_C;
	}
	else
	{
		command[SHOOTING_FIRE].cmd_type = RISE_TRIGER_C;
	}
	command[TUNNEL_MODE].update(&command[TUNNEL_MODE],abs(rc_sensor.info->thumbwheel.value)>=100);
	#ifdef RC_FIRING
	command[SHOOTING_FIRING].update(&command[SHOOTING_FIRING],rc_sensor.info->s1.value==1);	

	#else
	command[SHOOTING_FIRE].update(&command[SHOOTING_FIRE],rc_sensor.info->s1.value==1);	
	#endif
	
	command[GIM_180].update(&command[GIM_180],rc_sensor.info->thumbwheel.step_rising_trigger[RC_TB_DN]==true);
	#ifdef LOB_TEST
	
	
		#ifndef GYRO_LOB_INIT
		command[NORMAL_LOB].update(&command[NORMAL_LOB],rc_sensor.info->s2.status==up_R);
		#else
		command[AUTO_LOB].update(&command[AUTO_LOB],rc_sensor.info->s2.status==up_R);
		#endif
	
	
	
	command[GIM_UP].update(&command[GIM_UP],rc_sensor.info->ch1>100);
	command[GIM_DOWM].update(&command[GIM_DOWM],rc_sensor.info->ch1<-100);
	command[GIM_LEFT].update(&command[GIM_LEFT],rc_sensor.info->ch0<-100);
	command[GIM_RIGHT].update(&command[GIM_RIGHT],rc_sensor.info->ch0>100);
	if(car.car_move_mode == lob_CAR)
	{
		command[GIM_UP].cmd_type = RISE_TRIGER_C;
		command[GIM_DOWM].cmd_type = RISE_TRIGER_C;
		command[GIM_LEFT].cmd_type = RISE_TRIGER_C;
		command[GIM_RIGHT].cmd_type = RISE_TRIGER_C;
	}
	else
	{
		command[GIM_UP].cmd_type =   NO_CMD;
		command[GIM_DOWM].cmd_type = NO_CMD;
		command[GIM_LEFT].cmd_type = NO_CMD;
		command[GIM_RIGHT].cmd_type = NO_CMD;
	}
	#endif
}

/**
 * @brief 键盘模式命令更新
 * 
 */
void Key_Command_Update(void)
{
	//视觉模式下将控制权交给视觉，电控只要一直按下左键就发弹
	if(car.car_move_mode == vision_cycle_CAR ||car.car_move_mode == vision_gyro_CAR)
	{
		command[SHOOTING_FIRE].cmd_type = HIGH_TRIGER_C;
		command[SHOOTING_FIRE].update(&command[SHOOTING_FIRE],rc_sensor.info->mouse_btn_l.value==1);
	}
	else
	{
		command[SHOOTING_FIRE].cmd_type = RISE_TRIGER_C;
		command[SHOOTING_FIRE].update(&command[SHOOTING_FIRE],rc_sensor.info->mouse_btn_l.status==short_press);
		//非视觉模式才检测连发
		command[SHOOTING_FIRING].update(&command[SHOOTING_FIRING],rc_sensor.info->mouse_btn_l.status==long_press||rc_sensor.info->mouse_btn_l.status==release_to_press);
	}
	//云台转头命令  QER
	command[CAR_L90].update(&command[CAR_L90],rc_sensor.info->Q.status==release_to_press);
	command[CAR_R90].update(&command[CAR_R90],rc_sensor.info->E.status==release_to_press);
	command[GIM_180].update(&command[GIM_180],rc_sensor.info->R.status==release_to_press);
	//吊射命令  
	//command[AUTO_LOB].update(&command[AUTO_LOB],rc_sensor.info->Ctrl.value==1&&rc_sensor.info->V.status==release_to_press&&car.car_move_mode!=lob_CAR );
	command[NORMAL_LOB].update(&command[NORMAL_LOB],rc_sensor.info->Ctrl.value==1&&rc_sensor.info->C.status==release_to_press&&car.car_move_mode!=lob_CAR );
	command[OBLIQUE_LOB].update(&command[OBLIQUE_LOB],rc_sensor.info->Ctrl.value==1&&rc_sensor.info->V.status==release_to_press&&car.car_move_mode!=lob_CAR );
	//云台微调命令，吊射模式下有效  Ctrl+WASD
	#ifdef LOB_TEST
	command[GIM_UP].cmd_type = RISE_TRIGER_C;
	command[GIM_DOWM].cmd_type = RISE_TRIGER_C;
	command[GIM_LEFT].cmd_type = RISE_TRIGER_C;
	command[GIM_RIGHT].cmd_type = RISE_TRIGER_C;
	command[GIM_UP].update(&command[GIM_UP],rc_sensor.info->Ctrl.value!=1&&rc_sensor.info->W.status==release_to_press);
	command[GIM_DOWM].update(&command[GIM_DOWM],rc_sensor.info->Ctrl.value!=1&&rc_sensor.info->S.status==release_to_press);
	command[GIM_LEFT].update(&command[GIM_LEFT],rc_sensor.info->Ctrl.value!=1&&rc_sensor.info->A.status==release_to_press);
	command[GIM_RIGHT].update(&command[GIM_RIGHT],rc_sensor.info->Ctrl.value!=1&&rc_sensor.info->D.status==release_to_press);
	//图传命令  鼠标滚轮上拨图传抬头，下拨图传低头
	command[IMAGE_SWITCH_MAX].update(&command[IMAGE_SWITCH_MAX],rc_sensor.info->mouse_vz > 8);
    command[IMAGE_SWITCH_MIN].update(&command[IMAGE_SWITCH_MIN],rc_sensor.info->mouse_vz < -8);	
	#if HERO_TYPE ==2 //只有狗洞三摩有这个命令
	command[CHANGE_LOB_PITCH_ANGLE].update(&command[CHANGE_LOB_PITCH_ANGLE],rc_sensor.info->Shift.value==1&& \
											car.car_move_mode==lob_CAR);
	#endif
		
	
	#endif											
	//倍镜命令  松开Ctrl的情况下按V切换倍镜
	command[TELESCOPE_SWITCH].update(&command[TELESCOPE_SWITCH],rc_sensor.info->V.status == release_to_press && rc_sensor.info->Ctrl.value == 0);
	//超电命令  按下shift用超电
	command[CAP_ON].update(&command[CAP_ON],rc_sensor.info->Shift.value == 1);

	//自爆模式  长按G解除视线模糊热量限制，但是不会锁发射机构
	command[KILL_MYSELF].update(&command[KILL_MYSELF],rc_sensor.info->G.value == 1);
	//视觉打前哨
	command[TIMER_MEC_OUTPOST].update(&command[TIMER_MEC_OUTPOST],rc_sensor.info->X.value==1);
	//过隧道模式，pitch为机械目标0
	command[TUNNEL_MODE].update(&command[TUNNEL_MODE],rc_sensor.info->B.value==1&&rc_sensor.info->Ctrl.value!=1);
	//预充电模式 PRE_CHARGE
	command[PRE_CHARGE].update(&command[PRE_CHARGE],rc_sensor.info->B.value==1&&rc_sensor.info->Ctrl.value==1);
	//开启或关闭弹速自适应
	//command[OPEN_SPEED_ADAPT].update(&command[OPEN_SPEED_ADAPT],rc_sensor.info->Z.value==1);
}

/**
 * @brief 命令更新 car_ctrl中调用
 * 
 * @param car 
 */
void Command_Update(car_t *car)
{
  if(car->car_ctrl_mode == RC_CTRL_MODE)
  {
    RC_Command_Update();
  }
  else if(car->car_ctrl_mode == KEY_CTRL_MODE)
  {
    Key_Command_Update();
  }
}




/*************************外设通信********************************/

/**
 * @brief 视觉发送信息更新
 * 
 */

void Vision_TxData_Update(void)
{
    Vision_Tx_Info_t *tx_info = vision.tx_info;
	
	//模式更新
	if (rc_sensor.info->Ctrl.value==1)
	{
		tx_info->mode = 2; //只打前哨
	}
	else
	{
		tx_info->mode = 1;//
	}
	
	//是否准备打弹更新
	if (shooting.load_state == load_OK && shooting.shooting_state == SHOOTING_RESET_OK)
	{
		tx_info->is_ready = 1;
	}
	else
	{
		tx_info->is_ready = 0;
	}

	// yaw轴角度更新
    float yaw = gimbal.base_info.yaw_imu_angle;
	tx_info->yaw =  4096.f - (yaw / 180.f) * 4096.f;
//    tx_info->yaw =  2048.f + (yaw / 180.f) * 4096.f;
//	if(tx_info->yaw<0)
//	{
//		tx_info->yaw+=8192;
//	}
	//逆解
// 	float rx_yaw;
// 	rx_yaw=((tx_info->yaw-2048.f)/4096.f) *180;
// 	if (abs(rx_yaw)>180.f)
//     {
//         rx_yaw -= 360.f*sgn(rx_yaw);
//     }

	
  //pitch轴角度更新
    float pitch = gimbal.base_info.pitch_imu_angle;
    tx_info->pitch =  4096.f - (pitch / 180.f) * 4096.f;
	
	//roll轴角度更新
	float roll = imu_sensor.info->base_info.pitch;
	tx_info->roll =  4096.f + (roll / 180.f) * 4096.f;
	
	//pitch轴角速度更新
	tx_info->v_pitch = -gimbal.base_info.pitch_imu_speed;
	
	//yaw轴角速度更新
	tx_info->v_yaw = -gimbal.base_info.yaw_imu_speed;
	
	//offset
	tx_info->yaw_offset = gimbal.offset_info->vision_yaw_offset;
	tx_info->pitch_offset = gimbal.offset_info->vision_pitch_offset;
	
	//颜色更新
	if(communicate.game_robot_status_rx_info->game_process.bit.car_color == 0)
	{
		  tx_info->my_color = 1;
	}
	else
	{
		  tx_info->my_color = 0;
	}
	//比赛进程更新 //todo : 打训练赛关掉录视频
	if(	communicate.power_heat_data_rx_info->game_process == 3 ||
		 communicate.power_heat_data_rx_info->game_process == 4)
	{
		tx_info->dune = 1;
	}
	else
	{
		tx_info->dune = 0;
	}
}  
uint16_t userdef_chassis_power_limit=65;
void Car_TxData_Update(void)
{
	/*car_data0_tx_info pack update*****************************************************/
	car_data0_tx_info_t *car_data0_tx_info = communicate.car_data0_tx_info;
	//压缩chassis_angel取值范围
	float chassis_angel = (YAW_MOTOR_ANGLE_MIDDLE - gimbal.gimbal_y->KT_motor_info.rx_info.encoder) / 32768.f * 180 / 2;
	if (chassis_angel < 0)
	{
		chassis_angel += 180;
	}
	if(command[PRE_CHARGE].cmd_value==1&&car_data0_tx_info->pre_charge_flag==0)
	{
		car_data0_tx_info->pre_charge_flag=1;
	}
	else if(command[PRE_CHARGE].cmd_value==1&&car_data0_tx_info->pre_charge_flag==1)
	{
		car_data0_tx_info->pre_charge_flag=0;
	}
	
	car_data0_tx_info->pitch_angel = gimbal.base_info.pitch_imu_angle * 100;
	car_data0_tx_info->car_move_mode = car.car_move_mode;
	car_data0_tx_info->car_state.bit.is_find_target = vision.rx_info->is_find_target;
	car_data0_tx_info->car_state.bit.is_vision_online = vision.status->rx_state == DEV_ONLINE;
	#ifdef USERDEF_CHASSIS_POWER_LIMIT
	car_data0_tx_info->car_state.bit.is_userdef_chas_power_limit = 1;
	car_data0_tx_info->userdef_chassis_power_limit = USERDEF_CHASSIS_POWER_LIMIT;
	#endif
	
	
	
	car_data0_tx_info->car_state.bit.fri_speed_state = shooting.base_info.fri_info.fri_speed_state;
	car_data0_tx_info->car_state.bit.is_shooting_motor_online = SHOOTING_MOTOR_ONLINE;// 1是好的，0是掉线的
	car_data0_tx_info->car_state.bit.is_key_ctrl = (car.car_ctrl_mode == KEY_CTRL_MODE && RC_ONLINE);
	if(rc_sensor.info->B.value==1)//刷新ui
	{
		car_data0_tx_info->car_state.bit.is_key_ctrl =0;
	}

	if (car.car_ctrl_mode == KEY_CTRL_MODE)//键盘模式
	{
		if (command[CAP_ON].cmd_value == true || car.car_move_mode==lob_CAR)
		{
			car_data0_tx_info->car_state.bit.is_on_cap = 1;
		}
		else
		{
			car_data0_tx_info->car_state.bit.is_on_cap = 0;
		}
	}
	else//遥控器模式
	{
		#ifdef CAP_ENABLE
		
		if(abs(chassis.base_info.target_front_speed)>=CHASSIS_MAX_SPEED*0.8)
		{
			car_data0_tx_info->car_state.bit.is_on_cap = 1;
		}
		else
		{
			car_data0_tx_info->car_state.bit.is_on_cap = 0;
		}
		#else
		car_data0_tx_info->car_state.bit.is_on_cap = 0;
		#endif 
	}
	/*car_data1_tx_info pack update******************************/
	car_data1_tx_info_t *car_data1_tx_info = communicate.car_data1_tx_info;
	car_data1_tx_info->chassis_angel = chassis_angel;
	car_data1_tx_info->fric_b_speed = shooting.config->target_B_friction_speed;
	car_data1_tx_info->fric_f_speed = shooting.config->target_F_friction_speed;
	car_data1_tx_info->pitch_motor_angle = gimbal.base_info.pitch_mec_360_angle;
	/*car_data2_tx_info pack update******************************/
	car_data2_tx_info_t *car_data2_tx_info = communicate.car_data2_tx_info;
	car_data2_tx_info->detect_num = vision.rx_info->detect_num;
	car_data2_tx_info->uix_right = vision.rx_info->uix_right / 10;
	car_data2_tx_info->uiy_right = vision.rx_info->uiy_right / 5;
	car_data2_tx_info->ui_x = vision.rx_info->UI_x;
	car_data2_tx_info->ui_y = vision.rx_info->UI_y;
	/*car_data3 */
	car_data3_tx_info_t *car_data3_tx_info = communicate.car_data3_tx_info;
	car_data3_tx_info->uix_lb = vision.rx_info->uix_lb / 10;
	car_data3_tx_info->uix_rb = vision.rx_info->uix_rb / 10;
	car_data3_tx_info->uiy_lb = vision.rx_info->uiy_lb / 5;
	car_data3_tx_info->uiy_rb = vision.rx_info->uiy_rb / 5;

	car_data3_tx_info->uix_lt = vision.rx_info->uix_lt / 10;
	car_data3_tx_info->uix_rt = vision.rx_info->uix_rt / 10;
	car_data3_tx_info->uiy_lt = vision.rx_info->uiy_lt / 5;
	/*car_data4*/
	car_data4_tx_info_t *car_data4_tx_info = communicate.car_data4_tx_info;
	car_data4_tx_info->uiy_rt = vision.rx_info->uiy_rt / 5;
	car_data4_tx_info->vision_robot_distance = vision.rx_info->distance;
	

	car_data4_tx_info->uix_left = vision.rx_info->uix_left / 10;
	car_data4_tx_info->uiy_left = vision.rx_info->uiy_left / 5;
}

/**
 * @brief 通信内容更新 实时任务中调用
 * 
 * @param car 
 */
void Car_Communicate_Info_Update(car_t *car)
{
	Vision_TxData_Update();  //更新发给视觉的数据
	Car_TxData_Update();
}

/* External function prototypes -----------------------------------------------*/

/**
 * @brief 命令初始化 
 * 
 */
void Cmd_Init(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].init(&command[i]);
	}
}

/**
 * @brief 命令心跳 监控任务调用
 */
void Cmd_Heartbeat(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].heartbeat(&command[i]);
	}
}

/**
 * @brief 控制任务 实时任务中调用
 * @param car 
 */
void Car_Ctrl(car_t *car) 
{
	/*陀螺仪初始化不控*/
	while(HAL_GetTick()<=400)
	{}
	bmi.Kp=0.125;
	/*控制模式更新*/
	Car_Ctrl_Mode_Update(car);
	
	/* 整车模式更新 */
    Car_Move_Mode_Update(car);
	
	/* 整车命令更新 */
    Command_Update(car);
}

/**
 * @brief 模块层运行 实时任务中调用
 * @param car 
 */
void Car_Work(void)
{
	chassis.work(&chassis);//必须底盘在云台总控前面，因为吊射命令s_run在底盘
	gimbal.work(&gimbal);
	shooting.work(&shooting);
	image.work(&image);
}
/** 
 *	@brief 初始化应用层，任务初始化调用一次
 */
void Car_Init(void)
{
	Cmd_Init(); //全部命令初始化
}



