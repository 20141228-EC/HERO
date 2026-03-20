
#include "Command_Instance.h"

command_t command[COMMAND_LIST] = 
{
  [JUMP] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [KNEE_STRIKE_1] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [KNEE_STRIKE_2] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	  .init = Cmd_Class_Init,
	  },
  [FLY] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [Op_FLY] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},

  [TURN] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
};

/**
 * @brief 命令初始化，调用一次
 */
void Cmd_Init(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].init(&command[i]);
	}
}
/**
 * @brief 命令心跳 循环调用
 */
void Cmd_Heartbeat(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].heartbeat(&command[i]);
	}
}
/**
 * @brief 命令更新 循环调用
 */
void Command_Update(void)
{
	static uint32_t RC_ONLINE_TICK;
	
	rc_sensor_info_t*  rc_info=Balance.rc->sensor->info;
	
	static uint8_t wheel_up,last_wheel_up,wheel_dn,last_wheel_dn = 0;
	last_wheel_up = wheel_up;
	wheel_up = rc_info->thumbwheel.step[RC_TB_UP];
	last_wheel_dn = wheel_dn;
	wheel_dn = rc_info->thumbwheel.step[RC_TB_DN];

	if(Balance.rc->sensor->work_state==DEV_ONLINE)
	{
		RC_ONLINE_TICK++;
	}
	else
	{
		RC_ONLINE_TICK=0;
	}
	static uint8_t last_rc_info_s1;
	
	if(RC_ONLINE_TICK>=200)//屏蔽开控命令
	{
		if(Balance.ctrl != KEY_CTRL)
		{
			/*命令更新填这里*/
//			command[JUMP].update(&command[JUMP],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_UP &&
//			wheel_dn != last_wheel_dn);
			
			command[KNEE_STRIKE_1].update(&command[KNEE_STRIKE_1],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_UP &&
			wheel_up != last_wheel_up);
			
//			command[KNEE_STRIKE_2].update(&command[KNEE_STRIKE_2],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_UP &&
//			rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1);

//			command[FLY].update(&command[FLY],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_DOWN &&
//			rc_info->thumbwheel.step_change[RC_MD_TO_UP] == 1);

//			command[Op_FLY].update(&command[Op_FLY],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_DOWN &&
//			rc_info->thumbwheel.step_change[RC_MD_TO_DO] == 1);
			
			command[TURN].update(&command[TURN],rc_info->s1 == RC_SW_DOWN && rc_info->s2==RC_SW_MID &&
			wheel_up != last_wheel_up);
		}
		else
		{
//			command[JUMP].update(&command[JUMP],rc_info->V.status == release_to_press );
			
			command[KNEE_STRIKE_1].update(&command[KNEE_STRIKE_1],rc_info->C.status == release_to_press);
			
//			command[KNEE_STRIKE_2].update(&command[KNEE_STRIKE_2],rc_info->X.status == release_to_press && rc_info->Shift.status == release_to_press);

//			command[FLY].update(&command[FLY],rc_info->F.status == release_to_press);
//			
//			command[Op_FLY].update(&command[FLY],rc_info->F.status == release_to_press && rc_info->Ctrl.status == release_to_press);

			command[TURN].update(&command[TURN],rc_info->R.status == release_to_press);
		}
	}
	
	
	last_rc_info_s1=rc_info->s1;
}



