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
  .cmd_tele_switch = &command[TELESCOPE_SWITCH],
  .cmd_auto_lob = &command[AUTO_LOB],
  .cmd_image_switch_max = &command[IMAGE_SWITCH_MAX],
  .cmd_image_switch_min = &command[IMAGE_SWITCH_MIN],
	.image_reset_state = DEV_RESET_NO,
	.work = Image_Work,
};

/* Function  body --------------------------------------------------------*/

/**
 * @brief 图传PID计算
 * 
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
  default:
    break;
  }
}

/**
 * @brief 图传角度转换
 * 
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
 * 
 * @param image 
 */
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
 * @brief 图传初始化
 * 
 * @param image 
 */
void Image_Init(image_t *image)
{
  if (image->image_reset_state == DEV_RESET_NO)//图传未复位
  {
   image->base_info.ctrl_mode = IMAGE_SPEED_MODE;//速度环控制
   image->base_info.target_speed = IMAGE_INIT_SPEED;//设置目标速度
    if (Motor_DetectStuck(image->image) == 1)//如果堵转
    {
      image->base_info.target_speed = 0.f;//速度清零
      image->base_info.ctrl_mode = IMAGE_POSITION_MODE;//位置环控制
      image->base_info.target_position = image->image->info->angle_sum;//设置目标位置
      image->base_info.init_position = image->image->info->angle_sum;//记录复位成功时位置
      image->image_reset_state = DEV_RESET_OK;//复位完成
    }
  }
  image->base_info.telescope_status = TELESCOPE_UP;
  image->base_info.image_status = IMAGE_UP;
}

int32_t image_test = 5000;
/**
 * @brief 倍镜舵机命令执行
 * @param image 
 */
void Image_Cmd_Excute(image_t *image)
{
  if(image->image_reset_state != DEV_RESET_OK)//图传未复位  
  {
    return;
  }
  //遥控器控制模式下，机械模式和抛球模式才执行
  if(car.car_ctrl_mode == RC_CTRL_MODE && car.car_move_mode != mec_CAR && car.car_move_mode != lob_CAR)
  {
    return;
  }
  
  /*倍镜切换***********************************************************/
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
  /*一键吊射***********************************************************/
  if(image->cmd_auto_lob->cmd_value == true)
  {
    image->base_info.image_status = IMAGE_KEEP_DOWN;
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

}

/**
 * @brief 总控
 * 
 * @param image 
 */
void Image_Work(image_t *image)
{
  Image_Pitch_Angel_Transform(image);//图传角度转换
  Image_Cmd_Excute(image);//命令执行
  /*断电处理****************************************/
  if (image->image->work_state == DEV_OFFLINE)//图传电机没电
  {
    image->image_reset_state = DEV_RESET_NO;
    image->base_info.telescope_status = TELESCOPE_UP;
  }

  /*标志位更改****************************************/
	switch (car.car_move_mode)
  {
  case offline_CAR:
    image->base_info.target_speed = 0.f;
    image->base_info.output_image = 0;
    image->image_reset_state = DEV_RESET_NO;
    break;
  case init_CAR:
    Image_Init(image);
    break;
  case mec_CAR:
  case lob_CAR:
    Image_Update(image);//遥控器才会更新
    break;
  default:
    break;
  }
  /*目标值更改****************************************/
    //图传2006
  if (image->image_reset_state == DEV_RESET_OK && car.car_ctrl_mode == KEY_CTRL_MODE)
  {
    switch (image->base_info.image_status)
    {
    case IMAGE_UP:
      image->base_info.target_position = image->base_info.init_position;
      break;
    case IMAGE_DOWN:
      image->base_info.target_position = image->base_info.init_position - IMAGE_MAX_STROKE;
      break;
//写着玩用的功能
//    case IMAGE_FOLLOW_IMU:
//      image->base_info.target_position = image->base_info.init_position - imu_sensor.info->base_info.pitch * 36.f * 8192.f / 360.f;
//      break;
//    case IMAGE_MID:
//      image->base_info.target_position = image->base_info.init_position - IMAGE_MAX_STROKE + 10000;
//      break;
    case IMAGE_KEEP_DOWN:
      image->base_info.output_image = 1300;
      break;
    case IMAGE_KEEP_UP:
      image->base_info.output_image = -1000;
      break;

    default:
      break;
    }
  }
//限幅
  image->base_info.target_position = constrain(image->base_info.target_position, 
                                                image->base_info.init_position, //min
                                                image->base_info.init_position + IMAGE_MAX_STROKE);  //max

   //倍镜舵机
  if (image->base_info.telescope_status == TELESCOPE_DOWN)
  {
    Telescope_Down();
  }
  else if (image->base_info.telescope_status == TELESCOPE_UP)
  {
    Telescope_Up();
  }
  
  
  
  /*开控有输出****************************************/
  if (car.car_move_mode != offline_CAR)
  {
	  //除了保持状态有其它特殊动作才计算pid
	 if (image->base_info.image_status != IMAGE_KEEP_DOWN || image->base_info.image_status != IMAGE_KEEP_UP)
		{
			Image_Pid_Calculating(image);
		}
    
    image->image->base_info.motor_out = image->base_info.output_image;
  }
  else
  {
    Telescope_Sleep();//舵机关控卸力
    image->image->base_info.motor_out = 0;
  }
}

