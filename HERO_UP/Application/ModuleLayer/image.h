#ifndef _IMAGE_H
#define _IMAGE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "device.h"
#include "command.h"
/* Private variables ---------------------------------------------------------*/

#define IMAGE_INIT_SPEED (-1200.f) //图传复位速度

#if HERO_TYPE ==2
#define IMAGE_LOB_ANGLE_OFFSET (52000.f) 
#define IMAGE_MAX_STROKE (61700+810.f) //图传最大行程
#else
#define IMAGE_LOB_ANGLE_OFFSET (20000.f) 
#define IMAGE_MAX_STROKE (61700+810.f) //图传最大行程
#endif


#define IMAGE_MOTOR_ONLINE (image.image->work_state == DEV_ONLINE) //图传电机是否在线
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 图传控制模式
 * 
 */
typedef enum
{
  IMAGE_POSITION_MODE = 0,
  IMAGE_SPEED_MODE,
  IMAGE_CURRENT_MODE,
}image_ctrl_mode_e;

typedef enum
{
  TELESCOPE_DOWN = 0,
  TELESCOPE_UP,
}tele_status;

typedef enum
{
	IMAGE_SLEEP,//图传断电
  IMAGE_POSITION,//图传到最上
	IMAGE_INIT,
  IMAGE_FOLLOW_IMU,//图传保持pitch_imu = 0
  IMAGE_KEEP_DOWN,
  IMAGE_KEEP_UP,
}image_status;

/** 
  * @brief  图传基本信息定义
  */ 
typedef __packed struct  
{
	int16_t            output_image;    //图传输出
  image_ctrl_mode_e  ctrl_mode;       //图传控制模式
  float              target_position; //图传目标位置
  float              target_speed;    //图传目标速度

  int32_t            init_position;   //图传复位成功时位置
  float              image_to_gimbal_pitch_angel; //图传相对云台角度

  tele_status  telescope_status;//倍镜状态
  image_status image_status;//图传状态
}image_base_info_t;

/** 
  * @brief  图传类定义
  */ 
typedef struct image_class_t
{	
	rm_motor_t        		 *image;
	rm_motor_t        		 *telescope;
	command_t          *cmd_tele_switch;//图传切换
	command_t          *cmd_auto_lob;//一键预瞄吊射
	command_t          *cmd_normal_lob;//一键机械吊射
	command_t          *cmd_image_switch_max;//图传到最上
	command_t          *cmd_image_switch_min;//图传到最下
	command_t          *cmd_oblique_lob;

	image_base_info_t  base_info;
	
  Dev_Reset_State_e	 image_reset_state; //图传复位状态

	void               (*work)(struct image_class_t *image);

}image_t;


extern image_t image;



/* Exported functions --------------------------------------------------------*/

//总控
void Image_Work(image_t *image);
#endif

