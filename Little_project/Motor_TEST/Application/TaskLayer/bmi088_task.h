#ifndef __BMI088_TASK
#define __BMI088_TASK

#include "main.h"
#include "cmsis_os.h"
#include "device.h"

void StartBMI088Task(void const * argument);
extern IWDG_HandleTypeDef hiwdg;
#endif
