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

#include "procotol.h"
#include "INS.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"
#include "remote_control.h"
#include "omni_mecanum_chassis.h"
#include "gimbal.h"
#include "shoot.h"

#include "VPC.h"
#include "Serial.h"

#include "rs485.h"

extern bmi088_data_t imu_data;
// extern wfly_t *rc_data;
extern RC_ctrl_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];
extern shoot_cmd_t shoot_cmd;

float INS_YAW_angle_test;
float INS_YAW_speed_test;
float YAW_tar;

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

void VS_Receive_Control(void)
{
	VPC_UpdatePackets();
	VS_Pack_And_Send_Data_ROS2(&vs_aim_packet_to_nuc); // 视觉数据包发送
}

float vs_yaw_tar;
float vs_pitch_tar;
void RC_Receive_Control(void)
{
	YAW_tar = uart2_rx_message.angle_tar;
	INS_YAW_angle_test = INS.Yaw;
	INS_YAW_speed_test = INS.Gyro[2];
	uart2_tx_message.speed_yaw = INS.Gyro[2];
	uart2_tx_message.angle_yaw = INS.Yaw;
	uart2_tx_message.vs_yaw_tar = vs_aim_packet_from_nuc.yaw;
	uart2_tx_message.vs_mode = vs_aim_packet_from_nuc.mode;
	uart2_tx_message.shoot_launched = shoot_cmd.fire_launched;

	vs_yaw_tar = vs_aim_packet_from_nuc.yaw;
	vs_pitch_tar = vs_aim_packet_from_nuc.pitch;

	//板间485通信
	uart2_online_check();
}
