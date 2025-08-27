#ifndef __MONITOR_TASK
#define __MONITOR_TASK

#include "main.h"
#include "cmsis_os.h"
#include "device.h"
#include "command.h"
#include "vision_protocol.h"
#include "usart.h"
#include "car.h"
extern IWDG_HandleTypeDef hiwdg;

void StartMonitorTask(void const * argument);
void Soft_Reset(void);

#endif
