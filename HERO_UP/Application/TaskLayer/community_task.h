#ifndef __IMU_TASK
#define __IMU_TASK

#include "cmsis_os.h"
#include "main.h"
#include "bmi.h"
#include "rc_sensor.h"
#include "rc_protocol.h"
#include "vision_protocol.h"
#include "usart.h"
#include "car.h"
#include "judge_protocol.h"
void StartCommunityTask(void const * argument);

#endif
