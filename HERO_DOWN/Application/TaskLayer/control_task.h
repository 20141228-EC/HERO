#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "can_protocol.h"
#include "motor_def.h"

void StartControlTask(void const * argument);
extern IWDG_HandleTypeDef hiwdg;

#endif
