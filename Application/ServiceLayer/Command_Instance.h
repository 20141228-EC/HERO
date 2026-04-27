#ifndef __COMMAND_Instance_H
#define __COMMAND_Instance_H

#include "command.h"
#include "Balance.h"

enum 
{
  JUMP,
  KNEE_STRIKE_1,
  KNEE_STRIKE_2,
	FLY,
	Op_FLY,
	TURN,
	DOWN_TWO_STEP,
  COMMAND_LIST,
};

void Cmd_Init(void);
void Cmd_Heartbeat(void);
void Command_Update(void);
extern command_t command[COMMAND_LIST];
#endif
