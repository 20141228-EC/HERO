#ifndef _KT_MODULE_H
#define _KT_MODULE_H

/* Includes ------------------------------------------------------------------*/
#include "KT_motor.h"
/* Exported macro ------------------------------------------------------------*/

/* Exported function ------------------------------------------------------------*/
void kt_motor_run(void);
void kt_motor_stop(void);
void kt_motor_write_pid_param(void); 
void kt_motor_speed_control(int32_t speed);
void kt_motor_close(void);
void kt_motor_iq_control(int16_t iqControl);
void kt_motor_angle_control(uint16_t angle,uint8_t angle_single_Control_spinDirection,uint16_t angle_single_Control_maxSpeed);
void Read_kt_Encoder(void);
void Read_kt_FISE(void);

#endif
