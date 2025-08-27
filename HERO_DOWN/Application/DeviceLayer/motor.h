#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "rm_protocol.h"
#include "motor_def.h"
#include "drv_can.h"

#define ID_CHAS_LF  0x201
#define ID_CHAS_RF  0x202
#define ID_CHAS_LB  0x203
#define ID_CHAS_RB  0x204

extern  rm_motor_t rm_motor[RM_MOTOR_LIST];
extern  motor_pid_t motor_pid[RM_MOTOR_LIST];
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
uint8_t rm_motor_list_workstate(void);
 
#endif

