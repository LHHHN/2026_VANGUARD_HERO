/**
 * @file vmc.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __VMC_H__
#define __VMC_H__

#include <stdint.h>

#include "normal_filter.h"
#include "INS.h"

/******************************并联腿*****************************/

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5; // AE长度 //单位为m
	float l1, l2, l3, l4; // 单位为m

	float XB, YB; // B点的坐标
	float XD, YD; // D点的坐标
	float XC, YC;   // C点的直角坐标

	float L0, phi0; // C点的极坐标
	float alpha;
	float last_alpha;
	float d_alpha;
	float last_d_alpha;

	float l_BD; // BD两点的距离

	float d_phi0;    // 现在C点角度phi0的变换率
	float last_phi0; // 上一次C点角度，用于计算角度phi0的变换率d_phi0
	float last_d_phi0;

	float A0, B0, C0; // 中间变量
	float phi2, phi3;
	float phi1, phi4;

	float j11, j12, j21, j22; // 笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float front_joint_torque, back_joint_torque;

	float ik_phi1, ik_phi4; // 逆解算得到的phi1和phi4
	float ik_x_c, ik_y_c; // 逆解算输入的C点坐标

	float inv_j11, inv_j12, inv_j21, inv_j22; // 关节空间力到笛卡尔空间的力的雅可比矩阵系数
	float mea_F, mea_Tp, p;                   // 逆解算用到的参数
	float dd_z_wheel, d_z_wheel;
	float dd_z_M, d_z_M;

	float height, d_height;

	float F0;
	float Tp;
	float T1, T2;

	float pitch;
	float d_pitch;

	float theta;
	float last_theta;
	float d_theta; // theta的一阶导数
	float last_d_theta;
	float dd_theta; // theta的二阶导数

	float last_L0;
	float d_L0;  // L0的一阶导数
	float last_d_L0;
	float dd_L0; // L0的二阶导数

	float v;
	float x;

	float v_tar;
	float x_tar;

	float F_N; // 支持力

	float wheel_w;
	float last_wheel_w;
	float d_wheel_w;
	float wheel_m;
	float wheel_torque;
	float slip_k;

	float leg_b_v;

	float F_0_forward;
	float roll_f0_forward;
	
	float theta_target;
	float leg_set;  // 期望腿长，单位是m
	float delta_length_set;

	uint8_t first_init_flag; //初始化标志

	uint8_t off_ground_flag; // 离地标志
	uint8_t off_cnt;
	uint8_t touch_cnt;
	uint8_t leg_flag; // 腿长完成标志
} vmc_leg_t;

extern moving_average_filter_t F_N_L_filter;
extern moving_average_filter_t F_N_R_filter;

extern uint8_t touch_threshold;
extern uint8_t off_threshold;

extern void VMC_Init(vmc_leg_t *vmc); // 给杆长赋值

extern void VMC_Calc_Base_Data(vmc_leg_t *vmc, INS_behaviour_t *ins, float dt);

extern void VMC_Calc_T_Joint(vmc_leg_t *vmc); // 计算期望的关节输出力矩

extern uint8_t VMC_FN_Ground_Detection(vmc_leg_t *vmc, float dead_line); // 腿离地检测

extern float LQR_K_Calc(float *coe, float len);

extern void Leg_Force_Calc(vmc_leg_t *vmc);

extern void Forward_Kinematics(vmc_leg_t *chassis_leg);

extern void Inverse_Kinematics(float x, float y, vmc_leg_t *chassis_leg);

/******************************并联腿*****************************/

#endif /* __VMC_H__ */