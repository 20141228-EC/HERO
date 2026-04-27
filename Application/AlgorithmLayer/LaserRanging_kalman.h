#ifndef __LASERRANGING_KALMAN_H
#define __LASERRANGING_KALMAN_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "kalman_filter.h"
#include "LaserRanging_Protocol.h"
//#define NOISE_PROCESS  1.0f;//过程噪声方差
//#define NOISE_V_2   1.0f;//速度噪声方差
//#define NOISE_A_2     1.0f;//加速度噪声方差

extern uint8_t Kalman_Init_Flag;
extern KalmanFilter_t LR_Speed_KF;
extern float speed_result;
extern float LR_s_result;
void LaserRange_KF_Init(KalmanFilter_t *kf);
void LaserRange_KF_Update(KalmanFilter_t *kf, LaserRanging_t *LaserRanging);

#endif
