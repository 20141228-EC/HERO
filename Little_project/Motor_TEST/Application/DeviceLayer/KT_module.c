
/* Includes ------------------------------*/
#include "KT_module.h"
#include "motor.h"
#include "motor_def.h"
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 电机开始运行状态
 */

void kt_motor_run(void)
{
	kt_motor[0].tx_W_cmd(&kt_motor[0],MOTOR_RUN_ID);
}

/**
 * @brief 电机停止状态
 */

void kt_motor_stop(void)
{
	kt_motor[0].tx_W_cmd(&kt_motor[0],MOTOR_STOP_ID);
}

/**
 * @brief 电机关闭状态
 */

void kt_motor_close(void)
{
	kt_motor[0].tx_W_cmd(&kt_motor[0],MOTOR_CLOSE_ID);
}

/**
 * @brief 速度闭环（参数写100则每秒转1度）
 */
void kt_motor_speed_control(int32_t speed)
{
	kt_motor[0].W_speedControl(&kt_motor[0],speed);
	kt_motor[0].tx_W_cmd(&kt_motor[0],SPEED_CLOSE_LOOP_ID);
}

/**
 * @brief 角度闭环（36000代表360°）
 */
 

void kt_motor_angle_control(uint16_t angle,uint8_t angle_single_Control_spinDirection,uint16_t angle_single_Control_maxSpeed)
{
	kt_motor[0].W_angle_single_Control(&kt_motor[0],angle,angle_single_Control_spinDirection,angle_single_Control_maxSpeed);
	kt_motor[0].tx_W_cmd(&kt_motor[0],POSI_CLOSE_LOOP_ID3);
}

/**
 * @brief 转矩闭环（16.5A，2048）
 */
void kt_motor_iq_control(int16_t iqControl)
{
	kt_motor[0].W_iqControl(&kt_motor[0],iqControl);
	kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID);
}

/**
 * @brief 读取kt编码器
 */
void Read_kt_Encoder(void)
{
	kt_motor[0].tx_R_cmd(&kt_motor[0],ENCODER_RX_ID);
}

/**
 * @brief 读取kt转矩、电流、速度、编码器
 */
void Read_kt_FISE(void)
{
	kt_motor[0].tx_R_cmd(&kt_motor[0],STATE2_ID);
}










