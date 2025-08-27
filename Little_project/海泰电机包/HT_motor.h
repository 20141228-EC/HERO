#ifndef __HT_motor_H
#define __HT_motor_H

#include "stm32f4xx.h"
#include "driver.h"
#include "motor_def.h"
#include "algo.h"

#define CMD_MOTOR_MODE 0x01
#define CMD_RESET_MODE 0x02
#define CMD_ZERO_POSITION 0x03

typedef struct
{
	float target_p; // 单位为弧度(rad);
	float target_v; // 单位为 rad/s;
	float p_kp;		// 位置增益，单位为 N-m/rad；
	float v_kd;		// 速度增益，单位为 N-m*s/rad；
	float f_t;		// 单位为 N-m；

} ht_motor_tx_info_t;

typedef struct
{
	uint16_t rx_id;
	float angle;   // rad，范围±95.5
	float speed;   // rad/s
	float current; // 单位未知，ft发多少应该就返回多少

} ht_motor_rx_info_t;

typedef struct
{
	ht_motor_rx_info_t rx_info;
	ht_motor_tx_info_t tx_info;

} ht_motor_info_t;

// typedef enum
//{
//
//	M_PROTECT_ON = 0,
//	M_PROTECT_OFF ,
//
// }motor_protect_e;
// typedef enum motor_init_e
//{
//	M_DEINIT = 0,
//	M_INIT,
// }motor_init_e;
// typedef enum {
//	DEV_ONLINE,
//	DEV_OFFLINE,
// } dev_work_state_t;
typedef struct ht_motor_state_info_t
{
	motor_init_e init_flag;
	uint8_t offline_cnt_max;
	uint8_t offline_cnt;
	uint16_t selfprotect_cnt_max;
	uint16_t selfprotect_cnt;
	dev_work_state_t work_state;

	motor_protect_e selfprotect_flag;
} ht_motor_state_info_t;
typedef struct HT_motor_class_t
{
	ht_motor_info_t info;
	ht_motor_state_info_t state_info;
	uint8_t tx_buff[8];
	drv_can_t *driver;
	void (*init)(struct HT_motor_class_t *motor);
	void (*mode_cmd)(struct HT_motor_class_t *motor, uint8_t cmd);
	void (*control_cmd)(struct HT_motor_class_t *motor, float target_p, float target_v, float p_kp, float v_kd, float f_t);
	void (*get_info)(struct HT_motor_class_t *motor, uint8_t *rxBuf);
	void (*heartbeat)(struct HT_motor_class_t *motor);

} HT_motor_t;

/*Export Funtion-------------------------------------------------------*/
void ht_motor_class_init(HT_motor_t *motor);

#endif
