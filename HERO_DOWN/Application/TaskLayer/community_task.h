#ifndef __IMU_TASK
#define __IMU_TASK

#include "cmsis_os.h"
#include "main.h"
#include "bmi.h"
#include "usart.h"
#include "judge_protocol.h"
void StartCommunityTask(void const * argument);

#endif
