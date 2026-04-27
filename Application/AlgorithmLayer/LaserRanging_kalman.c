/**
 * @file LaserRanging_kalman.c
 * @brief 基于激光测距+机身速度的双观测卡尔曼滤波
 * @作者：LXG
 * @版本：V1.3
 * @日期：2025.11.16
 * @说明：双观测源（激光距离+机身计算速度），输出平滑后的机身距离
 *        speed_result：机身预计算的有效速度（外部传入/全局变量）
 */

#include "LaserRanging_kalman.h"
#include "car_info.h" // 包含 TIME_STEP（单位：秒，如0.01f=10ms）
#include "arm_math.h"
#include "rp_math.h"
#include "Filter.h"

// 全局变量声明
KalmanFilter_t LR_Speed_KF;
float LR_s_result = 0.0f;        // 滤波后的最终距离（核心输出）
extern float speed_result;       // 机身计算的有效速度（外部定义，需确保全局可访问）
float LR_v_result;
// --- 卡尔曼滤波参数（根据实际场景调优）---
#define LR_NOISE_V_2        0.005f   // 速度过程噪声方差（m2/s?）：动态场景增大，静态减小
#define LR_NOISE_D_MEAS     0.000004f// 激光距离观测噪声方差（m2）：激光误差±0.002m，方差=0.0022
#define LR_NOISE_V_MEAS     0.01f    // 机身速度观测噪声方差（m2/s2）：根据speed_result精度调整

/**
 * @brief 卡尔曼滤波器初始化（双观测模型）
 * @param kf：卡尔曼滤波器结构体指针
 */
void LaserRange_KF_Init(KalmanFilter_t *kf)
{
    if (kf == NULL) return;

    // 1. 状态转移矩阵 F（状态向量：[d; v]，d=距离，v=速度）
    // 物理意义：d_k = d_{k-1} + v_{k-1}*TIME_STEP；v_k = v_{k-1}（忽略加速度，匀速假设）
    static float F_Init[4] = 
    {
        1.0f, -TIME_STEP,   // 距离转移：上一距离 - 上一速度×时间
        0.0f, 1.0f  // 速度转移：速度保持不变
    };
    
    // 2. 过程噪声协方差矩阵 Q（与速度随机波动相关）
    static float Q_Init[4] = 
    {
        0.25f * LR_NOISE_V_2 * TIME_STEP*TIME_STEP*TIME_STEP*TIME_STEP,  // d的过程噪声协方差
        0.5f  * LR_NOISE_V_2 * TIME_STEP*TIME_STEP*TIME_STEP,     // d与v的过程噪声协方差
        0.5f  * LR_NOISE_V_2 * TIME_STEP*TIME_STEP*TIME_STEP,     // v与d的过程噪声协方差
        LR_NOISE_V_2 * TIME_STEP*TIME_STEP                 // v的过程噪声协方差
    };
    
    // 3. 观测矩阵 H（双观测：激光测距离，机身测速度）
    // 观测向量 Z = [d_obs; v_obs] = H*X + v，H为单位矩阵（直接观测状态量）
    static float H_Init[4] = 
    {
        1.0f, 0.0f,  // 第一个观测（激光）→ 状态d
        0.0f, 1.0f   // 第二个观测（机身速度）→ 状态v
    };
    
    // 4. 观测噪声协方差矩阵 R（双观测，对角矩阵）
    // 对角元素分别为激光距离噪声、机身速度噪声（非对角元素为0，假设观测独立）
    static float R_Init[4] = 
    {
        LR_NOISE_D_MEAS, 0.0f,        // 激光距离观测噪声
        0.0f,            LR_NOISE_V_MEAS // 机身速度观测噪声
    };
    
    // 5. 初始误差协方差矩阵 P（表示初始状态的不确定度，无需过大）
    static float P_Init[4] = 
    {
        0.1f,  0.0f,  // d的初始不确定度
        0.0f,  0.1f   // v的初始不确定度
    };
    
    // 初始化滤波器：状态维度=2（d,v）、控制输入维度=0、观测维度=2（双观测）
    Kalman_Filter_Init(kf, 2, 0, 2);
    
    // 赋值矩阵（严格匹配维度）
    memcpy(kf->F_data, F_Init, sizeof(F_Init));
    memcpy(kf->Q_data, Q_Init, sizeof(Q_Init));
    memcpy(kf->H_data, H_Init, sizeof(H_Init));
    memcpy(kf->R_data, R_Init, sizeof(R_Init));
    memcpy(kf->P_data, P_Init, sizeof(P_Init));
    
    // 初始状态暂设为0，第一次更新时用真实观测值覆盖（避免初始偏差）
    kf->xhat_data[0] = 0.0f; // 初始距离估计
    kf->xhat_data[1] = 0.0f; // 初始速度估计
}

/**
 * @brief 卡尔曼滤波更新（双观测模型）
 * @param kf：卡尔曼滤波器结构体指针
 * @param LaserRanging：激光测距数据结构体（含原始距离）
 */
void LaserRange_KF_Update(KalmanFilter_t *kf, LaserRanging_t *LaserRanging)
{
    // 空指针容错
    if (kf == NULL || LaserRanging == NULL) return;

    // 1. 获取两个有效观测值
    float d_obs = LaserRanging->LaserRanging_receive.Distance; // 激光距离观测（m）
    float v_obs = XEstimateKF.FilteredValue[1];                                // 机身速度观测（m/s，外部已计算有效）

    // 2. 第一次更新：用真实观测值初始化状态（避免固定初始值导致的偏差）
    static uint8_t first_update_flag = 1;
    if (first_update_flag)
    {
        kf->xhat_data[0] = d_obs; // 初始距离 = 第一次激光观测值
        kf->xhat_data[1] = v_obs; // 初始速度 = 第一次机身速度值
        first_update_flag = 0;
        LR_s_result = d_obs;      // 首次输出直接用激光值
        return;
    }

    // 3. 设置双观测向量（索引0=距离，索引1=速度，严格对应zSize=2）
    // 注意：之前用索引3是错误的（超出双观测维度），现在修正为索引1
    kf->MeasuredVector[0] = d_obs; // 第一个观测：激光距离
    kf->MeasuredVector[1] = v_obs; // 第二个观测：机身速度

    // 4. 执行卡尔曼滤波核心更新（预测+修正）
    Kalman_Filter_Update(kf);

    // 5. 提取滤波结果：状态向量第一个元素是平滑后的距离
    LR_s_result = kf->FilteredValue[0];
	LR_v_result = kf->FilteredValue[1];

	if(LR_s_result<0)
	{
		LR_s_result = 0.01f;
	}
    // 6. 异常值限幅（根据实际应用场景调整，单位：m）
    // 用fabs适配float类型（之前abs会强制转int，导致精度丢失）
    if (fabs(LR_s_result) >= 5.0f)
    {
        LR_s_result = sgn(LR_s_result) * 5.0f;
    }
	
}