#ifndef __PID_H
#define __PID_H
#include "main.h"
 

typedef struct feedforward {
	float ka;
	float kb;  //二阶差分太大容易急停
	
	float val_now;
	float val_last;
	float val_pre;
	
	float const_val;
	float feedforward_outmax;
	float feed_val;

	//	int16_t		(*cal)(struct Feedforward_Struct_t*	Feedforward);
} feedforward_t;

typedef struct {
	float full_speed_integral_threshold; //阈值须为正数
	float lower_speed_integral_threshold;
	uint8_t  use_dynamic_integration; //启用时需设置为1，否则无法积分
} dynamic_integral_t;



typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float	integral;
	float 	integral_max;
	float 	out_max;
	feedforward_t 	   feedforward;
	dynamic_integral_t dynamic_integration;
} pid_ctrl_t;

void single_pid_ctrl(pid_ctrl_t *pid);
float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
float feedforward_pid_calc(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
#endif
