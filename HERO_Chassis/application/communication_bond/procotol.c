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

#include "robot_frame_config.h"
#include "procotol.h"
#include "INS.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"
#include "remote_control.h"
#include "omni_mecanum_chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "rs485.h"
#include "user_lib.h"

extern bmi088_data_t imu_data;
// extern wfly_t *rc_data;
extern RC_ctrl_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

extern gimbal_cmd_t gimbal_cmd;
extern shoot_cmd_t shoot_cmd;

extern ramp_function_source_t *gimbal_angle_ramp;

static uint8_t Get_Control_Source(void)
{
	if (rc_data->rc.switch_left == 2 && rc_data->rc.switch_right == 2)
	{
		return CONTROL_SRC_KEYMOUSE;
	}
	return CONTROL_SRC_REMOTE;
}

static uint8_t Get_Fire_Command(uint8_t control_src)
{
	if (control_src == CONTROL_SRC_KEYMOUSE)
	{
		return rc_data->mouse.press_l ? 1U : 0U;
	}
	return (rc_data->rc.dial == 660) ? 1U : 0U;
}

void VOFA_Display_IMU(void)
{
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

	//	// vofa_data_view[6] = imu_data.temperature;
	//
	//	vofa_data_view[12] = INS.Pitch;
	//	vofa_data_view[13] = INS.Roll;
	//	vofa_data_view[14] = INS.Yaw;
}

void RC_Transfer_Control(void)
{
	// uart2_tx_message.angle_tar = target_yaw_pro;
	// uart2_tx_message.rocker_r_ = rc_data->rc.rocker_r_;
	// uart2_tx_message.rocker_r1 = rc_data->rc.rocker_r1;
	// uart2_tx_message.rc_switch = 0x01 << (rc_data->rc.switch_right - 1) | 0x08 << (rc_data->rc.switch_left - 1) ;

	uint8_t control_src = Get_Control_Source();

	uart2_tx_message.rocker_r1 = rc_data->rc.rocker_r1;
	uart2_tx_message.mouse_y = rc_data->mouse.y;
	uart2_tx_message.rc_switch = 0x01 << (rc_data->rc.switch_right - 1) | 0x08 << (rc_data->rc.switch_left - 1);
	uart2_tx_message.control_src = control_src;
	uart2_tx_message.gimbal_mode = gimbal_cmd.mode;
	uart2_tx_message.shoot_mode = shoot_cmd.mode;

	// 板间485通信
	uart2_online_check();
}
