/**
******************************************************************************
 * @file    procotol.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

/******************************procotol处为联系各处的纽带，包含各种需要的头文件*****************************/

#include "procotol.h"
#include "INS.h"
#include "shoot.h"
#include "gimbal.h"
#include "chassis.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"

#include "balance_chassis.h"

extern bmi088_data_t imu_data;
extern wfly_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

void VOFA_Display_IMU(void)
{
	/******************************IMU滤波数据测试*****************************/
	// vofa_data_view[0] = imu_data.acc[0];
	// vofa_data_view[1] = imu_data.acc[1];
	// vofa_data_view[2] = imu_data.acc[2];

	// vofa_data_view[3] = test_data_acc[0];
	// vofa_data_view[4] = test_data_acc[1];
	// vofa_data_view[5] = test_data_acc[2];

	// vofa_data_view[6] = imu_data.gyro[0];
	// vofa_data_view[7] = imu_data.gyro[1];
	// vofa_data_view[8] = imu_data.gyro[2];

	// vofa_data_view[9]  = test_data_gyro[0];
	// vofa_data_view[10] = test_data_gyro[1];
	// vofa_data_view[11] = test_data_gyro[2];
	/******************************IMU滤波数据测试*****************************/

	/******************************车体轮毂测试*****************************/
	// vofa_data_view[0] = chassis_right_leg.wheel_torque;
	// vofa_data_view[1] = chassis_left_leg.wheel_torque;
	// vofa_data_view[2] = balance_chassis.wheel_motor[0]->motor_controller.pid_ref;
	// vofa_data_view[3] = balance_chassis.wheel_motor[1]->motor_controller.pid_ref;
	// vofa_data_view[4] = balance_chassis.wheel_motor[0]->receive_data.real_current;
	// vofa_data_view[5] = balance_chassis.wheel_moto r[1]->receive_data.real_current;
	// vofa_data_view[6] = chassis_right_leg.wheel_w * WHEEL_RADIUS;
	// vofa_data_view[7] = chassis_left_leg.wheel_w * WHEEL_RADIUS;
	// vofa_data_view[8] = balance_chassis.turn_T;
	// vofa_data_view[9] = balance_chassis.v_set;
	// vofa_data_view[10] = balance_chassis.v_filter;
	// vofa_data_view[11] = balance_chassis.x_filter;
	// vofa_data_view[12] = chassis_right_leg.leg_b_v;
	// vofa_data_view[13] = chassis_left_leg.leg_b_v;
	// vofa_data_view[14] = balance_chassis.pitch;
	/******************************车体轮毂测试*****************************/

	/******************************车体输入误差测试*****************************/
	// vofa_data_view[0] = chassis_right_leg.wheel_torque;
	// vofa_data_view[1] = chassis_left_leg.wheel_torque;
	// vofa_data_view[2] = INS.MotionAccel_b[1];//balance_chassis.turn_T;
	// vofa_data_view[3] = chassis_err_left_test[0];
	// vofa_data_view[4] = chassis_err_right_test[0];
	// vofa_data_view[5] = chassis_err_left_test[1];
	// vofa_data_view[6] = chassis_err_right_test[1];
	// vofa_data_view[7] = chassis_err_left_test[2];
	// vofa_data_view[8] = chassis_err_right_test[2];
	// vofa_data_view[9] = chassis_err_left_test[3];
	// vofa_data_view[10] = chassis_err_right_test[3];
	// vofa_data_view[11] = chassis_err_left_test[4];
	// vofa_data_view[12] = chassis_err_right_test[4];
	// vofa_data_view[13] = chassis_err_left_test[5];
	// vofa_data_view[14] = chassis_err_right_test[5];
	/******************************车体输入误差测试*****************************/

	/******************************离地检测测试*****************************/
	// vofa_data_view[0] = F_N_forward_test;
	// vofa_data_view[0] = DD_W_forward_test;

	// vofa_data_view[1] = chassis_right_leg.dd_z_M;
	// vofa_data_view[2] = chassis_left_leg.dd_z_M;

	// vofa_data_view[3] = chassis_right_leg.v;
	// vofa_data_view[4] = chassis_left_leg.v;

	// vofa_data_view[5] = chassis_right_leg.F_N;
	// vofa_data_view[6] = chassis_left_leg.F_N;

	// vofa_data_view[7] = chassis_right_leg.p;
	// vofa_data_view[8] = chassis_left_leg.p;

	// vofa_data_view[9]  = chassis_right_leg.dd_z_wheel;
	// vofa_data_view[10] = chassis_left_leg.dd_z_wheel;

	// vofa_data_view[11] = chassis_right_leg.back_joint_torque;
	// vofa_data_view[12] = chassis_left_leg.back_joint_torque;
	// vofa_data_view[13] = chassis_right_leg.front_joint_torque;
	// vofa_data_view[14] = chassis_left_leg.front_joint_torque;
	/******************************离地检测测试*****************************/

	/******************************打滑檢測测试*****************************/
	// // vofa_data_view[0]  = chassis_right_leg.d_phi0;
	// // vofa_data_view[1]  = chassis_left_leg.d_phi0;
	// vofa_data_view[0]  = chassis_right_leg.d_wheel_w;
	// vofa_data_view[1]  = chassis_left_leg.d_wheel_w;
	// vofa_data_view[2]  = -INS.MotionAccel_b[1];
	// vofa_data_view[3]  = balance_chassis.a_filter;
	// vofa_data_view[4]  = chassis_right_leg.wheel_w * WHEEL_RADIUS;
	// vofa_data_view[5]  = chassis_left_leg.wheel_w * WHEEL_RADIUS;
	// vofa_data_view[6]  = chassis_right_leg.leg_b_v;
	// vofa_data_view[7]  = chassis_left_leg.leg_b_v;
	// vofa_data_view[8]  = chassis_right_leg.v;
	// vofa_data_view[9]  = chassis_left_leg.v;
	// vofa_data_view[10] = balance_chassis.delta_v;
	// vofa_data_view[11] = balance_chassis.slip_flag;
	// vofa_data_view[12] = balance_chassis.v_delta_wheel;
	// vofa_data_view[13] = balance_chassis.v_delta_leg;
	// vofa_data_view[14] = balance_chassis.v_gyro;
	/******************************打滑檢測测试*****************************/

	/******************************车体状态量测试*****************************/
	// vofa_data_view[0] = balance_chassis.slip_flag;//balance_chassis.leg_tp; 

	// // vofa_data_view[1] = chassis_right_leg.back_joint_torque;
	// // vofa_data_view[2] = chassis_left_leg.back_joint_torque;
	// // vofa_data_view[3] = chassis_right_leg.front_joint_torque;
	// // vofa_data_view[4] = chassis_left_leg.front_joint_torque;
	// vofa_data_view[1] = balance_chassis.joint_motor[0]->receive_data.torque;
	// vofa_data_view[2] = balance_chassis.joint_motor[1]->receive_data.torque;
	// vofa_data_view[3] = balance_chassis.joint_motor[2]->receive_data.torque;
	// vofa_data_view[4] = balance_chassis.joint_motor[3]->receive_data.torque;

	// vofa_data_view[5] = chassis_right_leg.L0;
	// vofa_data_view[6] = chassis_left_leg.L0;
	// vofa_data_view[7] = balance_chassis.height_set;

	// vofa_data_view[8] = chassis_right_leg.F_N;
	// vofa_data_view[9] = chassis_left_leg.F_N;

	// vofa_data_view[10] = chassis_right_leg.F0;
	// vofa_data_view[11] = chassis_left_leg.F0;

	// // vofa_data_view[12]  = F_N_forward_test[0];
	// // vofa_data_view[13]  = F_N_forward_test[1];
	// // vofa_data_view[12] = chassis_right_leg.v_tar;
	// // vofa_data_view[13] = chassis_left_leg.v_tar;
	// // vofa_data_view[14] = balance_chassis.v_filter;
	// // vofa_data_view[15] = balance_chassis.x_filter;
	// // vofa_data_view[12]  = chassis_right_leg.wheel_w * WHEEL_RADIUS;
	// // vofa_data_view[13]  = chassis_left_leg.wheel_w * WHEEL_RADIUS;
	// // vofa_data_view[14] = (float)balance_chassis.wheel_motor[0]->receive_data.real_current / 1000.0f;
	// // vofa_data_view[15] = (float)balance_chassis.wheel_motor[1]->receive_data.real_current / 1000.0f;

	// vofa_data_view[12] = balance_chassis.v_delta_wheel;
	// vofa_data_view[13] = balance_chassis.v_gyro;
	// vofa_data_view[14] = balance_chassis.d_yaw;
	// vofa_data_view[15] = balance_chassis.roll_f0_forward;

	// vofa_data_view[16] = balance_chassis.roll;
	// vofa_data_view[17] = balance_chassis.roll_f0;

	// vofa_data_view[18] = chassis_right_leg.off_ground_flag;
	// vofa_data_view[19] = chassis_left_leg.off_ground_flag;

	/******************************车体状态量测试*****************************/
	vofa_data_view[0] = chassis_right_leg.d_phi0;
	vofa_data_view[1] = balance_chassis.roll;
	vofa_data_view[2] = balance_chassis.body_height;
	vofa_data_view[3] = balance_chassis.delta_height_set;
	vofa_data_view[4] = balance_chassis.roll_f0;
	vofa_data_view[5] = chassis_right_leg.L0;
	vofa_data_view[6] = chassis_left_leg.L0;
	vofa_data_view[7] = chassis_right_leg.delta_length_set;
	vofa_data_view[8] = chassis_left_leg.delta_length_set;
	vofa_data_view[9] = chassis_right_leg.leg_set;
	vofa_data_view[10] = chassis_left_leg.leg_set;
	vofa_data_view[11] = chassis_right_leg.F_N;
	vofa_data_view[12] = chassis_left_leg.F_N;

	vofa_data_view[13] = chassis_left_leg.d_phi0;//TODO(GUATAI):d_phi0一致性
	vofa_data_view[14] = (balance_chassis.joint_motor[0]->receive_data.velocity + balance_chassis.joint_motor[1]->receive_data.velocity) / 2.0f;//TODO(GUATAI):d_phi0一致性
	vofa_data_view[15] = (balance_chassis.joint_motor[2]->receive_data.velocity + balance_chassis.joint_motor[3]->receive_data.velocity) / 2.0f;

	vofa_data_view[16] = balance_chassis.d_yaw;

	vofa_data_view[17] = balance_chassis.turn_set;
	vofa_data_view[18] = balance_chassis.wz_set;
	vofa_data_view[19] = balance_chassis.total_yaw;

	VOFA_Send_Data(vofa_data_view, 20);
	// VOFA_JustFloat(vofa_data_view, 7);
}

void RC_Receive_Control(void)
{
}
