#ifndef __CAR_H
#define __CAR_H


#include "rp_config.h"
#include "chassis.h"
#include "gimbal.h"
#include "image.h"
#include "Command.h"
#include "main.h"



/*----------Function------------*/

void Car_Init(void);



/*----------Function_end------------*/
/**
 * @brief 命令列表
 * 
 */
enum 
{
  CAR_L90,//左转90度
  CAR_R90,//右转90度
  GIM_180,//云台转180度
  GIM_UP,//云台向上微调
  GIM_DOWM,//云台向下微调
  GIM_RIGHT,//云台向右微调
  GIM_LEFT,//云台向左微调

  SHOOTING_FIRE,//单发
  SHOOTING_FIRING,//连发
  
  TELESCOPE_SWITCH,//切换倍镜状态

  
  NORMAL_LOB,//机械吊射，不动yaw，只动pitch，车模式转吊射,不动倍镜
  AUTO_LOB,//一键吊射，动yaw、pitch、车模式转吊射，动倍镜
  OBLIQUE_LOB,//斜着吊射，吊射命令完成后加一个头偏置角度
  IMAGE_SWITCH_MAX,//图传到最上
  IMAGE_SWITCH_MIN,//图传到最下

  CHANGE_LOB_PITCH_ANGLE,//改变吊射梯高、香蕉道pitch角度
  CAP_ON,//打开超电

  KILL_MYSELF,//自爆

  TIMER_MEC_OUTPOST,//定时器机械击打前哨模式
  
  TUNNEL_MODE,//过隧道模式
  
  PRE_CHARGE,//无线充电预充电模式
  
  OPEN_SPEED_ADAPT,//开启弹速自适应
  
  COMMAND_LIST,
};

/* 车行动模式枚举 */
typedef enum 
{
  offline_CAR,        //离线模式         0
  init_CAR,           //初始化模式       1
  mec_CAR,            //机械模式         2
  gyro_CAR,           //陀螺仪模式       3
  cycle_CAR,		  //小陀螺模式       4
  vision_gyro_CAR,    //视觉陀螺仪模式   5
  vision_cycle_CAR,   //视觉小陀螺模式   6
  lob_CAR,            //抛球模式(底盘位置环)         7

}Car_Move_Mode_e;

/* 车控制模式枚举 */
typedef enum 
{
  RC_CTRL_MODE,        //遥控器模式  0
  KEY_CTRL_MODE,       //键盘模式    1
}Car_Ctrl_Mode_e;

/**
 * @brief car类结构体
 * 
 */
typedef  __packed struct  car_struct
{
  Car_Move_Mode_e car_move_mode;
  Dev_Reset_State_e	 car_reset_state;
  Car_Ctrl_Mode_e  car_ctrl_mode;
  
  uint8_t  unlock_car_flag;
  uint16_t init_cnt;
  uint16_t init_cnt_max;
  

}car_t;

/*Export function---------------------------------------*/
void Cmd_Heartbeat(void);
void Cmd_Init(void);
void Car_Ctrl(car_t *car);
void Car_Work(void);
void Car_Communicate_Info_Update(car_t *car);
/*Export typedef---------------------------------------*/
extern Car_Ctrl_Mode_e Car_Ctrl_Mode;
extern command_t command[COMMAND_LIST];
extern car_t car;
#define UNLOCK_CAR_CONDITION  (rc_sensor.info->ch2>400&&rc_sensor.info->ch3<-400&&rc_sensor.info->ch0<-400&&rc_sensor.info->ch1<-400&&(rc_sensor.info->s1.value==2||rc_sensor.info->s1.value==3))

#endif
