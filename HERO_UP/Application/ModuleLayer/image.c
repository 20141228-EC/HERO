/**
 * @file image.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* Includes ------------------------------*/
#include "image.h"
#include "car.h"
/* Private function prototypes -----------------------------------------------*/
void Image_Init(image_t *image);
void Image_Cmd_Excute(image_t *image);
/* Exported variables --------------------*/
image_t image = 
{
	.image = &rm_motor[IMAGE],
	.telescope = &rm_motor[TELESCOPE],
	.cmd_tele_switch = &command[TELESCOPE_SWITCH],
	.cmd_auto_lob = &command[AUTO_LOB],
	.cmd_normal_lob =&command[NORMAL_LOB],
	.cmd_oblique_lob=&command[OBLIQUE_LOB],
	.cmd_image_switch_max = &command[IMAGE_SWITCH_MAX],
	.cmd_image_switch_min = &command[IMAGE_SWITCH_MIN],
	.image_reset_state = DEV_RESET_NO,
	.work = Image_Work,
};

/* Function  body --------------------------------------------------------*/
//倍镜相关
void Telescope_Up(image_t *image)
{
	#if HERO_TYPE ==3
	image->telescope->base_info.motor_out=3000;//3000
	#else
	image->telescope->base_info.motor_out=2000;
	#endif
}

void Telescope_Down(image_t *image)
{
	#if HERO_TYPE ==3
	image->telescope->base_info.motor_out=-3000;
	#else
	image->telescope->base_info.motor_out=-2000;
	#endif
}

void Telescope_Sleep(image_t *image)
{
	image->telescope->base_info.motor_out=0;
}
/**
 * @brief 图传PID计算
 * @param image 
 */
void Image_Pid_Calculating(image_t *image)
{
  
  
  switch (image->base_info.ctrl_mode)
  {
  case IMAGE_POSITION_MODE:
    image->base_info.output_image = rm_motor_anglesum_pid_calc(image->image, image->base_info.target_position);
    break;
  case IMAGE_SPEED_MODE:
    image->base_info.output_image = rm_motor_speed_pid_calc(image->image, image->base_info.target_speed);
    break;
  case IMAGE_CURRENT_MODE:
	
    break;
  default:
    break;
  }
}

/**
 * @brief 图传角度转换
 * @param image 
 */
void Image_Pitch_Angel_Transform(image_t *image)
{
  if (image->image_reset_state == DEV_RESET_OK)
  {
    image->base_info.image_to_gimbal_pitch_angel = (float)(image->image->info->angle_sum - image->base_info.init_position)/819.1f;
  }
  else
  {
    image->base_info.image_to_gimbal_pitch_angel = 999.999f;
  }
}

/**
 * @brief 图传目标更新
 * @param image 
 */
float k_image_position=0.3;
void Image_Update(image_t *image)
{
  if (image->image_reset_state == DEV_RESET_OK)
  {
    switch (car.car_ctrl_mode)
    {
    case RC_CTRL_MODE://遥控器控制
//      image->base_info.target_position += (float)(rc_sensor.info->thumbwheel.value) * 0.1f;
      break;
	
    case KEY_CTRL_MODE://键鼠控制
		if(abs(rc_sensor.info->mouse_y)>0.3&&car.car_move_mode==lob_CAR)
		{
			 
			image->base_info.target_position -=k_image_position*rc_sensor.info->mouse_y;

		}

      break;
		
    default:
      break;
    }
  }
  else
  {
    Image_Init(image);
  }
}

/**
 * @brief 图传初始化流程
 * @param image 
 */
static uint16_t outoff_work_cnt;//时间太长退出
void Image_Init(image_t *image)
{
	
   image->base_info.ctrl_mode = IMAGE_SPEED_MODE;//电流控制
   image->base_info.target_speed = IMAGE_INIT_SPEED;//设置目标速度
	Image_Pid_Calculating(image);//计算输出
   image->base_info.telescope_status = TELESCOPE_UP;
	outoff_work_cnt++;
    if (Motor_DetectStuck_userdef(image->image,30,2000,200,0) == 1)//如果堵转
    {
      image->base_info.target_speed = 0.f;//停止
	  image->base_info.target_position = image->image->info->angle_sum;//设置目标位置
      image->base_info.init_position = image->image->info->angle_sum;//记录复位位置
	  /*break*/
      image->image_reset_state = DEV_RESET_OK;//复位完成
	  image->base_info.image_status = IMAGE_KEEP_UP;//复位完优先向上
		outoff_work_cnt=0;
    }
//	if (outoff_work_cnt>=CAR_INIT_TIME-10)//如果超时
//    {
//      image->base_info.target_speed = 0.f;//速度清零
//      image->base_info.target_position = image->image->info->angle_sum;//设置目标位置
//      image->base_info.init_position = image->image->info->angle_sum;//记录复位成功时位置
//	  /*break*/
//	  image->base_info.ctrl_mode = IMAGE_CURRENT_MODE;//电流控制
//	  image->base_info.image_status = IMAGE_KEEP_UP;//复位完优先向上
//      image->image_reset_state = DEV_RESET_OK;//复位完成
//		outoff_work_cnt=0;
//    }
	
}

int32_t image_test = 5000;

/**
 * @brief 检查是否退出吊射模式，并抬起图传
 * @param image 
 */
void Check_OFF_Lob(image_t *image)
{
	static Car_Move_Mode_e last_Car_Move_Mode;
	//如果现在不是吊射但刚才是吊射模式
	if(car.car_move_mode!=lob_CAR&&last_Car_Move_Mode==lob_CAR)
	{
		image->base_info.image_status = IMAGE_KEEP_UP;//抬起图传
		image->base_info.telescope_status = TELESCOPE_UP;//抬起倍镜
		/*耦合性高，是shi，为了让他在瞬时进吊射退吊射能及时进入陀螺仪模式*/
		image->cmd_auto_lob->cmd_status=FINISH_C; //
		gimbal.base_info.pitch_imu_angle_target=0;
		
		/*清进命令标志位*/
		gimbal.lob_info.into_auto_lob_command_flag=0;
		gimbal.lob_info.into_outpost_lob_command_flag=0;
		gimbal.lob_info.into_normal_lob_command_flag=0;
		gimbal.lob_info.into_oblique_lob_command_flag=0;
		/*清除吊射偏置*/
		gimbal.offset_info->lob_yaw_gyro_offset=0;
		gimbal.offset_info->lob_pitch_gyro_offset=0;
		gimbal.offset_info->lob_yaw_mec_offset=0;
		
	}
	last_Car_Move_Mode=car.car_move_mode;
}
/**
 * @brief 吊射一键图传对准角度
 * @param image 
 */
void Image_Lob_Angel_Choose(image_t *image)
{
	
}
/**
 * @brief 倍镜舵机命令执行,状态更新
 * @param image 
 */

void Image_Cmd_Excute(image_t *image)
{
  if(image->image_reset_state != DEV_RESET_OK)//图传未复位  
  {
    return;
  }
  
  /*倍镜切换V***********************************************************/
  if (image->cmd_tele_switch->cmd_value == true)
  {
    if (image->base_info.telescope_status == TELESCOPE_UP)
    {
      image->base_info.telescope_status = TELESCOPE_DOWN;
    }
    else
    {
      image->base_info.telescope_status = TELESCOPE_UP;
    }
  }
  /*一键吊射CTRL+V/C***********************************************************/
  if(image->cmd_oblique_lob->cmd_value == true||image->cmd_normal_lob->cmd_value == true||
	 image->cmd_auto_lob->cmd_value == true)
  {
    image->base_info.image_status = IMAGE_POSITION;
	image->base_info.target_position=image->base_info.init_position+IMAGE_LOB_ANGLE_OFFSET;

  }
  /*图传到最上*********************************************************/
  if(image->cmd_image_switch_max->cmd_value == true)
  {
	      image->base_info.image_status = IMAGE_KEEP_UP;
  }
  /*图传到最下*********************************************************/
  if(image->cmd_image_switch_min->cmd_value == true)
  {
     
	      image->base_info.image_status = IMAGE_KEEP_DOWN;
	 
  }
  /*位置控制*******************************************************************/
  if(abs(rc_sensor.info->mouse_y> 0.1)&&car.car_move_mode==lob_CAR)//吊射时才能调
  {
	image->base_info.image_status = IMAGE_POSITION;
  }
  /*隧道模式**************************************************************/
  if(command[TUNNEL_MODE].cmd_value==1)
	{
		image->base_info.image_status = IMAGE_KEEP_UP;
	}
}

/**
 * @brief 总控
 * 
 * @param image 
 */
void Image_Work(image_t *image)
{
  Image_Pitch_Angel_Transform(image);//图传角度转换DEBUG
  Image_Cmd_Excute(image);//命令执行
  Check_OFF_Lob(image);//退出吊射图传自动抬起
  /*断电重新初始化****************************************/
  if (image->image->work_state == DEV_OFFLINE)//图传电机没电
  {
    image->image_reset_state = DEV_RESET_NO;
    image->base_info.telescope_status = TELESCOPE_UP;
	image->base_info.image_status =IMAGE_SLEEP;  
  }
  /*优先执行初始化,再执行其他****************************************/
  if(image->image_reset_state == DEV_RESET_NO)
  {
	Image_Init(image);
  }
  else
  {
	switch (car.car_move_mode)
  {
	case offline_CAR:
		image->base_info.target_speed = 0.f;
		image->base_info.output_image = 0;
		image->base_info.target_position = image->image->info->angle_sum;//实时更新目标位置
		image->image_reset_state = DEV_RESET_NO;
		image->base_info.image_status =IMAGE_SLEEP;
		break;
	case init_CAR:
		Image_Init(image);
		break;
	case mec_CAR:
	case lob_CAR:
	case gyro_CAR:
		Image_Update(image); 
		break;
	default:
		break;
	}
  }
  
	
  //位置限幅
  image->base_info.target_position = constrain(image->base_info.target_position, 
                                                image->base_info.init_position, //min
                                                image->base_info.init_position + IMAGE_MAX_STROKE);
  /*计算输出值****************************************/
    //图传2006输出
    switch (image->base_info.image_status)
    {
   
    case IMAGE_KEEP_DOWN:
		 image->base_info.ctrl_mode = IMAGE_CURRENT_MODE;
		 image->base_info.output_image = 3000;
		 image->base_info.target_position = image->image->info->angle_sum;//实时更新目标位置
      break;
    case IMAGE_KEEP_UP:
		image->base_info.ctrl_mode = IMAGE_CURRENT_MODE;
		 image->base_info.output_image = -3000;
	    image->base_info.target_position = image->image->info->angle_sum;//实时更新目标位置
		static uint16_t init_position_cnt;
		static uint16_t last_image_angle_sum;
		if(abs(image->image->info->speed)<=100)
		image->base_info.init_position = image->image->info->angle_sum;//记录复位位置
      break;
	case IMAGE_POSITION:
		image->base_info.ctrl_mode = IMAGE_POSITION_MODE;
		 Image_Pid_Calculating(image);
      break;
	
      break;
    default:
      break;
    }
	//舵机直接输出
	if(car.car_move_mode != offline_CAR)
	{
		if (image->base_info.telescope_status == TELESCOPE_DOWN)
		{
			Telescope_Down(image);
		}
		else if (image->base_info.telescope_status == TELESCOPE_UP)
		{
			Telescope_Up(image);
		}
	}
	else
	{
		 Telescope_Sleep(image);
	}
  
  /*开控输出赋值给电机****************************************/
  if (car.car_move_mode != offline_CAR)
  {
	  
    image->image->base_info.motor_out = image->base_info.output_image;
  }
  else
  {
    Telescope_Sleep(image);
	  image->base_info.output_image = 0;
    image->image->base_info.motor_out = 0;
  }
}

