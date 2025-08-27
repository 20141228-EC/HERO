/**
 * @name 海泰电机包，于HT8108验证可用
 * @note -只有MIT模式，无一拖四
 *  		-每次上电均需要切换电机模式，电机模式时亮绿灯
 *		-1000Hz
 *		-滑到最下面有使用示例和调用的外部函数
 *		-接收到的数据的转换可能还有点问题，批判性使用
 * @author LYQ
 * @date 2024/12/15
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "HT_motor.h"
/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Private macro -------------------------------------------------------------*/
// 命令码
#define CMD_MOTOR_MODE 0x01
#define CMD_RESET_MODE 0x02
#define CMD_ZERO_POSITION 0x03
// 限幅
#define P_MIN -95.5f // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function  ---------------------------------------------------------*/

/**
 *	@brief  电机模式切换
 */

void ht_motor_mode_cmd(HT_motor_t *motor, uint8_t cmd)
{

	motor->tx_buff[0] = 0xFF;
	motor->tx_buff[1] = 0xFF;
	motor->tx_buff[2] = 0xFF;
	motor->tx_buff[3] = 0xFF;
	motor->tx_buff[4] = 0xFF;
	motor->tx_buff[5] = 0xFF;
	motor->tx_buff[6] = 0xFF;
	motor->tx_buff[7] = 0x00;
	switch (cmd)
	{
	case CMD_MOTOR_MODE:
		motor->tx_buff[7] = 0xFC;
		break;

	case CMD_RESET_MODE:
		motor->tx_buff[7] = 0xFD;
		break;

	case CMD_ZERO_POSITION: // 重置零点
		motor->tx_buff[7] = 0xFE;
		break;

	default:
		return; /* 直接退出函数 */
	}
	if (motor->driver->can_id == DRV_CAN1)
		CAN_SendData(&hcan1, motor->driver->tx_id, motor->tx_buff);
	else
		CAN_SendData(&hcan2, motor->driver->tx_id, motor->tx_buff);
}
/**
 *	@brief  电机MIT控制，内含CAN发送，调用电机即转
 *  @param  范围：target_p±95.5 rad ，target_v ±45.0 rad/s，p_kp 0~500，v_kd 0~5，f_t ±18 N*M
 */

void ht_motor_control(HT_motor_t *motor, float target_p, float target_v, float p_kp, float v_kd, float f_t)
{
	uint16_t p, v, kp, kd, t;
	/*限幅*/
	motor->info.tx_info.target_p = constrain(target_p, P_MIN, P_MAX);
	motor->info.tx_info.target_v = constrain(target_v, V_MIN, V_MAX);
	motor->info.tx_info.p_kp = constrain(p_kp, KP_MIN, KP_MAX);
	motor->info.tx_info.v_kd = constrain(v_kd, KD_MIN, KD_MAX);
	motor->info.tx_info.f_t = constrain(f_t, T_MIN, T_MAX);

	/* 根据协议，对float参数进行转换 */
	p = float_to_uint(motor->info.tx_info.target_p, P_MIN, P_MAX, 16);
	v = float_to_uint(motor->info.tx_info.target_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(motor->info.tx_info.p_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor->info.tx_info.v_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor->info.tx_info.f_t, T_MIN, T_MAX, 12);

	/* 根据传输协议，把数据转换为CAN命令数据字段 */
	motor->tx_buff[0] = p >> 8;
	motor->tx_buff[1] = p & 0xFF;
	motor->tx_buff[2] = v >> 4;
	motor->tx_buff[3] = ((v & 0xF) << 4) | (kp >> 8);
	motor->tx_buff[4] = kp & 0xFF;
	motor->tx_buff[5] = kd >> 4;
	motor->tx_buff[6] = ((kd & 0xF) << 4) | (t >> 8);
	motor->tx_buff[7] = t & 0xff;

	if (motor->driver->can_id == DRV_CAN1)
		CAN_SendData(&hcan1, motor->driver->tx_id, motor->tx_buff);
	else
		CAN_SendData(&hcan2, motor->driver->tx_id, motor->tx_buff);
}

/**
 *	@brief   获取电机数据
 *	@note   占位大小 8、16、12、12
 */

void ht_motor_get_info(HT_motor_t *motor, uint8_t *buf)
{

	motor->info.rx_info.rx_id = (uint16_t)(buf[0]);
	if (motor->info.rx_info.rx_id == motor->driver->rx_id)
	{
		motor->state_info.offline_cnt = 0;
		motor->info.rx_info.angle = uint_to_float(((buf[1] << 8) | buf[2]), P_MIN, P_MAX, 16);
		motor->info.rx_info.speed = uint_to_float((buf[3] << 4) | (buf[4] >> 4), V_MIN, V_MAX, 12);
		motor->info.rx_info.current = uint_to_float((buf[4] & 0x0F) << 8 | buf[5], -T_MAX, T_MAX, 12);
	}
}
/**
 *	@brief	电机心跳，如果发生失联，下一次收到数据时，offline_cnt会在电机更新中进行清零
 */
void HT_motor_class_heartbeat(HT_motor_t *motor)
{
	static float current_last;
	if (motor == NULL)
		return;

	ht_motor_state_info_t *state_info = &motor->state_info;

	if (state_info->init_flag == M_DEINIT)
	{
		state_info->work_state = DEV_ONLINE;
		return;
	}

	state_info->offline_cnt++;
	// 发过来的电流一直相同判断为进入保护（不一定正确）
	if (motor->info.rx_info.current == current_last)
	{
		state_info->selfprotect_cnt++;
	}
	else
	{
		state_info->selfprotect_cnt = 0;
	}
	current_last = motor->info.rx_info.current;
	// 失联状态赋值
	if (state_info->offline_cnt > state_info->offline_cnt_max)
	{
		state_info->offline_cnt = state_info->offline_cnt_max;
		state_info->work_state = DEV_OFFLINE;
	}
	else
	{
		if (state_info->work_state == DEV_OFFLINE)
			state_info->work_state = DEV_ONLINE;
	}
	// 保护状态赋值
	if (state_info->selfprotect_cnt > state_info->selfprotect_cnt_max)
	{
		state_info->selfprotect_cnt = state_info->selfprotect_cnt_max;
		state_info->selfprotect_flag = M_PROTECT_OFF;
	}
	else
	{
		if (state_info->selfprotect_flag == M_PROTECT_OFF)
			state_info->selfprotect_flag = M_PROTECT_ON;
	}
}
/**
 *	@brief   初始化
 */

void ht_motor_class_init(HT_motor_t *motor)
{
	if (motor == NULL)
		return;

	memset((uint8_t *)motor->tx_buff, 0, sizeof(motor->tx_buff));
	memset(&motor->info, 0, sizeof(ht_motor_info_t));

	// 函数
	motor->control_cmd = ht_motor_control;
	motor->get_info = ht_motor_get_info;
	motor->mode_cmd = ht_motor_mode_cmd;
	motor->heartbeat = HT_motor_class_heartbeat;
	// 初始化赋值
	motor->state_info.init_flag = M_INIT;
	motor->state_info.offline_cnt_max = 50;
	motor->state_info.selfprotect_cnt_max = 2000;
}

/*外部函数start-------------------------------------------------*/
///**
// *	@brief	将属于[x_min,x_max]的x映射到[0,bit_max],返回两个字节
// */
// uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
//{
//    float span = x_max - x_min;
//    float offset = x_min;
//
//    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
//}
///**
// *	@brief	将属于[x_min,x_max]的x映射到[0,bit_max],返回浮点
// */
// float uint_to_float(int x_int, float x_min, float x_max, int bits)
//{
//    float span = x_max - x_min;
//    float offset = x_min;
//    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//}
// #define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
/*外部函数end-------------------------------------------------*/

/**电机包使用示例start-----------------------------------------------

* @note 通过设置target_p来让电机转（单位为rad，范围±95.5）
*		也可以修改f_t的值来控电牛
*/

/*定义电机*/
// drv_can_t ht_motor_drive={
//		.rx_id = 0x01,
//		.tx_id =0x01,
//		.can_id = DRV_CAN1,
// };
// HT_motor_t ht_motor={
//
//	.driver =&ht_motor_drive,
//	.init=ht_motor_class_init,
// };
/*接收电机数据*/
// void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
//{
//	switch (rxId)
//	{
//		case 0x00:
//		{
//			ht_motor.get_info(&ht_motor,rxBuf);
//			break;
//		}
//		default:
//			break;
//	}
// }
/*驱动任务*/
// float target_p=0; float target_v=5; float p_kp=0; float v_kd=0.5; float f_t=0;
// void StartControlTask(void const * argument)
//{
//	ht_motor.init(&ht_motor);
//	ht_motor.mode_cmd(&ht_motor,CMD_MOTOR_MODE);
//	for(;;)
//	{
//		ht_motor.heartbeat(&ht_motor);
//		ht_motor.control_cmd(&ht_motor,target_p,target_v,p_kp,v_kd,f_t);
//		osDelay(1);
//	}
// }
/**电机包使用示例end------------------------------------------*/
