#ifndef __MONITOR_TASK
#define __MONITOR_TASK

#include "main.h"
#include "cmsis_os.h"
#include "device.h"
#include "usart.h"
extern IWDG_HandleTypeDef hiwdg;

void StartMonitorTask(void const * argument);
void Soft_Reset(void);

#endif
