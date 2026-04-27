#ifndef __CAR_INFO_H
#define __CAR_INFO_H


/*************************** 机体属性 begin ****************************/
#define WHEEL_RADIUS  0.058f//驱动轮半径，单位：m  
//腿杆长，如果为串联腿请将l5置零，l1=l2,l3=l4
#define l1     0.235f
#define l2     0.282f
#define l3     0.282f
#define l4     0.235f
#define l5     0.f
//各杆质心系数
#define l1_cen    0.4742f            //5
#define l2_cen    0.f            //0
#define l3_cen    0.4009f            //5
#define l4_cen    0.4297f            //5

//各杆质量
#define m_l1     0.1677f
#define m_l2     0.f
#define m_l3     0.3985f 
#define m_l4     0.43f 

//驱动轮质量（算上定子）
#define mw      0.588f            //0.55chuan        ************************************************
            //
//机体质量
#define mb    29.f          //19.25f          //3.2f  //kg bing      *********************************************
#define g  9.81f

//整车旋转半径
#define Rl  0.2885f////////////////            *********************************************
//最长最短腿长
#define MAX_LEG_LENGTH    0.458f
#define MIN_LEG_LENGTH    0.133f

//单腿质量，四杆总和
#define m_l 1.4f            //
//
//
#define R_PHI1_UP_ANGLE    (-170.96)/////////////////代码里无-17096
#define R_PHI4_UP_ANGLE    (-14.4)///////////////////-14.4
							
#define L_PHI1_UP_ANGLE    (-164.0f)///////////////            //
#define L_PHI4_UP_ANGLE    (-13.5f)//////////////////            //
/*************************** 机体属性 end ****************************/


/*************************** 控制配置 begin ****************************/

#define TAR_LEG_LENGTH_INITIAL   (0.16f)//初始目标腿长////////////            //
#define OFF_GROUND_SUPPORT 		 52.f//离地支持力阈值，越小越难触发，单位：N    普通70，飞坡50
#define FLY_OFF_GROUND_SUPPORT 		 48.f//飞坡离地支持力阈值，越小越难触发，单位：N    普通70，飞坡50
#define MAX_LIFT_SPEED           0.2f       //单位：m/s  腿长改变最大速度
#define MAX_SPIN_SPEED           2.0f    //单位：rad/s 车体转向运动最大速度////////

/*软件限位相关，保护机械结构,应该可以不用加*/
#define LIMIT_RANGE      (10.f)
#define SD_POS_FIX_TOR_K			(-0.f)   //关节限位力矩补偿系数 -0.1

/*卸力阻尼时间与阻尼系数*/
#define DAMPING_DELAY_MAX_CNT     3000   //阻尼持续时间1.5s
#define Wheel_Damping_Coefficient 0.0001f //
#define Sd_Damping_Coefficient    3.5f

#define MAX_STRAIGHT_SPEED	2.0f///////////2.3

#define RC_INPUT_SD1_ORDER_CORRECT 1.f/////////
    

/*************************** 控制配置 end ****************************/




/*************************** 零点、方向配置 begin ****************************/

/*关节电机零点  42.37 0.739496 2.4020966  13.03 0.227416 2.9141766*/
/*关节电机零点  46.02 0.803201 2.3383916  15.25 0.266163 2.8754296*/
#define R_F_HORIZON_ANGLE (0.67107892f+2.3383916f)         //(-1.638f)            //=0.779
#define R_B_HORIZON_ANGLE (-0.30771184f+0.266163f)         //(-0.764f)            //=0.227
#define L_F_HORIZON_ANGLE (-0.43254137f-2.3383916f)         //(-2.118f-2.723f+3.1415f+5.42f)    //514bing         //
#define L_B_HORIZON_ANGLE (-1.2977196f-0.266163f)         //(-1.552f+2.687f-0.0274f   /*-0.71f*/)     //2.75bing            //

/*关节电机零点运算方向校正*/
#define R_F_HORIZON_ANGLE_ORDER_CORRECT 1
#define R_B_HORIZON_ANGLE_ORDER_CORRECT -1
#define L_F_HORIZON_ANGLE_ORDER_CORRECT -1
#define L_B_HORIZON_ANGLE_ORDER_CORRECT 1

/*电机编码器值递增方向修正，逆时针为1，顺时针为-1*/
#define R_F_TIME  -1    //从右看nishizhen
#define R_B_TIME  1    
#define L_F_TIME  1   //从右往左看是与上面镜像
#define L_B_TIME  -1

/*建模与VMC的Tp方向矫正*/
#define L_TP_LQR_ORDER_CORRECT    -1///
#define R_TP_LQR_ORDER_CORRECT    -1///

/*建模与VMC的vir_phi0方向矫正*/
#define L_VIR_PHI0_ORDER_CORRECT  -1///
#define R_VIR_PHI0_ORDER_CORRECT  -1///

/*建模与电机扭矩输出方向矫正*/
#define L_F_ORDER_CORRECT    -1  //关节电机--++
#define L_B_ORDER_CORRECT    -1
#define R_F_ORDER_CORRECT    1
#define R_B_ORDER_CORRECT    1

#define L_W_ORDER_CORRECT    -1 //驱动轮///
#define R_W_ORDER_CORRECT    1///

/* 双腿协调Tp_sync方向矫正 */
#define L_SYNC_ORDER_CORRECT   -1
#define R_SYNC_ORDER_CORRECT   1

/* Roll角控制Tp_roll方向矫正 */
#define L_TP_Roll_ORDER_CORRECT    1
#define R_TP_Roll_ORDER_CORRECT    -1

/* 转向控制Tw_turn方向矫正 */
#define R_TURN_ORDER_CORRECT 1//2.21gai
#define L_TURN_ORDER_CORRECT -1

/* 侧向前馈竖直力F_inertial方向矫正 */
#define L_F_INERTIAL_ORDER_CORRECT -1
#define R_F_INERTIAL_ORDER_CORRECT 1

/* 消除电机定子转动对位移影响方向矫正 */
#define R_STATOR_ORDER_CORRECT 1
#define L_STATOR_ORDER_CORRECT -1

/* 用于求s、sd1的轮速、轮总角度方向矫正 */
#define R_W_SPEED_ORDER_CORRECT 1//  负改正
#define L_W_SPEED_ORDER_CORRECT 1

#define R_W_ANGLESUM_ORDER_CORRECT 1
#define L_W_ANGLESUM_ORDER_CORRECT -1

/* 关节电机总角度方向矫正 */
#define R_F_SD_ANGLESUM_ORDER_CORRECT -1
#define R_B_SD_ANGLESUM_ORDER_CORRECT -1
#define L_F_SD_ANGLESUM_ORDER_CORRECT 1
#define L_B_SD_ANGLESUM_ORDER_CORRECT 1

/*************************** 零点、方向配置 end ****************************/


#define TIME_STEP			0.001f//任务运行周期，单位：s

typedef enum
{
	R_Leg,
	
	L_Leg,
	
	Leg_Num,
}Leg_e;

#endif
