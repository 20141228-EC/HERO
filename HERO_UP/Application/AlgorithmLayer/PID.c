#include "pid.h"
#include "rp_math.h"
/**
 *  @name   single_pid_ctrl
 *	@brief  新的pid计算，对微分进行低通滤波，积分进行变速
 *  @author HERMIT_PURPLE
 */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    // 保存误差值(需要在外面自行计算误差)
	//pid->err = pid->target-pid->measure;
    // 变速积分，使用时需要在pid结构体里将use_dynamic_integration置1
	if(pid->dynamic_integration.use_dynamic_integration==1)
	{
		if(abs(pid->err)<=pid->dynamic_integration.full_speed_integral_threshold)
		pid->integral += pid->err;
		//线性减少积分大小，降低比例从0~100%
		else if((abs(pid->err)<pid->dynamic_integration.lower_speed_integral_threshold)&&(abs(pid->err)>pid->dynamic_integration.full_speed_integral_threshold))
		pid->integral += (sgn(pid->err)*pid->dynamic_integration.full_speed_integral_threshold-pid->err)/(pid->dynamic_integration.lower_speed_integral_threshold - pid->dynamic_integration.full_speed_integral_threshold);
		else;
		//pid->integral +=0;
	}
	else
	{
		pid->integral += pid->err;
	}
  
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
    // p i d 输出项计算
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
	//对微分项低通滤波
    pid->dout = Lowpass(pid->last_dout,pid->kd * (pid->err - pid->last_err),0.7);
	pid->last_dout=pid->dout;
    // 累加pid输出值
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
    // 记录上次误差值
    pid->last_err = pid->err;
}
/**
 * @brief  pid积分清零
 * 
 */
void integral_to_zero(pid_ctrl_t *pid)
{
	pid->integral=0;
}
/**
 *	@brief	pid总控制 参数：外环 内环  外环或内环目标值 外环观测值 内环观测值  内环观测值kp，一般填负的 err处理方式
 *          err_cal_mode：err处理方式 半圈还是四分之一圈 0，1，2 速度环使用0 yaw轴使用1 
						陀螺仪角度环 3
 *         内环不能为NULL
 *	@note   使用示例：
			pid_ctrl_t *out	  = ;
			pid_ctrl_t *inn	  = ;
			float target   	  = ;
			float mea_out       = ;
			float mea_in        = ;
			float inner_kp      = ;
			uint8_t err_cal_mode= ;
			=all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
 *  @author HERMIT_PURPLE
 *
 *  @return 返回计算结果
 */

float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	if(inn == NULL)return 0;  //没有内环，为0
	
	 else if(out == NULL&&inn!=NULL)  //只有速度环
	{
		inn->target=target;
		inn->measure=mea_in;
		inn->err=inn->target-mea_in;
		single_pid_ctrl(inn);
		return inn->out;
	}
	
	else if(out != NULL&&inn!=NULL)  //双环PID
	{
		
		out->target=target;
		out->measure=mea_out;
		out->err=out->target-out->measure; //计算角度环误差，后面再进行误差处理
		switch(err_cal_mode)
		{
			
			case 0:			
				break;
			
			case 1:
				out->err = motor_half_cycle(out->err, 8191);
				break;		
			
			case 2:
				out->err = motor_half_cycle(out->err, 8191);
				out->err = motor_half_cycle(out->err, 4095);
				break;
			
			case 3:
				out->err = motor_half_cycle(out->err, 360);
				break;
			
			case 4:
				out->err = motor_half_cycle(out->err, 65535);
				break;
			
			default:
				break;
		}
		
		single_pid_ctrl(out);  //计算出处理过误差的角度环的值
		inn->target=out->out; //角度环输出作为速度环目标值
		inn->measure=mea_in*inner_kp;//内环输入kp，可以调整正负和大小
		inn->err=inn->target+inn->measure;  //速度环误差计算
		single_pid_ctrl(inn);
		return inn->out;  //输出内环计算值
	}
	else  //只有角度环
	{
		return 0;
	}
}

/**
 *	@brief   对外环使用前馈pid计算
 *  @note   使用时需要在pid结构体里引出前馈的参数并定义
 *  @author HERMIT_PURPLE
 *  @return 返回计算结果
 */

float feedforward_pid_calc(pid_ctrl_t *out ,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	if(out==NULL)
	{
		return 0 ;
	}
	out->feedforward.val_now=out->target;
	
	//计算差分
	float delta_target=out->feedforward.val_now-out->feedforward.val_last;
	float delta_target_accel=delta_target-(out->feedforward.val_last-out->feedforward.val_pre);//now-last-(last-pre)
	//计算前馈项：差分比例项+常数项
	 out->feedforward.feed_val= out->feedforward.ka * delta_target + out->feedforward.kb * delta_target_accel+ out->feedforward.const_val;
	//限幅
	 out->feedforward.feed_val=constrain(out->feedforward.feed_val, -out->feedforward.feedforward_outmax, out->feedforward.feedforward_outmax);
	//计算前馈pid总输出:前馈项+pid计算输出
	float output=  out->feedforward.feed_val+all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
	out->feedforward.val_pre=out->feedforward.val_last;
	out->feedforward.val_last=out->feedforward.val_now;
	
	return output;
}







