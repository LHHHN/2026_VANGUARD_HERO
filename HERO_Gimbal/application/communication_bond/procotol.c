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
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "INS.h"

#include "vofa.h"
#include "bmi088.h"
#include "wfly_control.h"
#include "remote_control.h"

#include "VPC.h"
#include "Serial.h"

#include "rs485.h"

extern bmi088_data_t imu_data;
extern RC_ctrl_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

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
	static uint8_t bullet_count_cnt = 0;
	// if(shoot_cmd.fire_launched == 1)
	// {
	// 	vs_aim_packet_to_nuc.bullet_count = 1;
	// 	bullet_count_cnt = 50;
	// }
	// else
	// {
	// 	if(bullet_count_cnt > 0)
	// 	{
	// 		bullet_count_cnt--;
	// 	}
	// 	else
	// 	{
	// 		vs_aim_packet_to_nuc.bullet_count = 0;
	// 	}
	// }
	VPC_UpdatePackets();
	VS_Pack_And_Send_Data_ROS2(&vs_aim_packet_to_nuc); // 视觉数据包发送
}

//float vs_yaw_tar;
//float vs_pitch_tar;
void RC_Receive_Control(void)
{
#if RS485_CHA == 1	
	YAW_tar = uart2_rx_message.angle_tar;
	INS_YAW_angle_test = INS.Yaw;
	INS_YAW_speed_test = INS.Gyro[2];
	uart2_tx_message.speed_yaw = INS.Gyro[2];
	uart2_tx_message.angle_yaw = INS.Yaw;
	uart2_tx_message.vs_yaw_tar = vs_aim_packet_from_nuc.yaw;
	uart2_tx_message.vs_mode = vs_aim_packet_from_nuc.mode;
	uart2_tx_message.shoot_launched = shoot_cmd.fire_launched;
//	vs_yaw_tar = vs_aim_packet_from_nuc.yaw;
//	vs_pitch_tar = vs_aim_packet_from_nuc.pitch;

	//板间485通信
	uart2_online_check();
#else
	RS485_Handle_Rx_Data();
	rs485_tx_message.chassis_mode = chassis_cmd.mode;
	rs485_tx_message.gimbal_mode = gimbal_cmd.mode;
	rs485_tx_message.shoot_mode = shoot_cmd.mode;
	rs485_tx_message.chassis_target_vx = chassis_cmd.target_vx;
	rs485_tx_message.chassis_target_vy = chassis_cmd.target_vy;
	rs485_tx_message.chassis_target_wz = chassis_cmd.target_wz;
	rs485_tx_message.chassis_target_leg_angle = chassis_cmd.target_leg_angle;
	rs485_tx_message.gimbal_target_yaw = gimbal_cmd.yaw_target;
	rs485_tx_message.gimbal_target_yaw_speed = gimbal_cmd.yaw_v;
	rs485_tx_message.gimbal_measure_yaw = INS.Yaw;
	rs485_tx_message.gimbal_measure_yaw_speed = INS.Gyro[2];
	rs485_tx_message.gimbal_measure_pitch = INS.Pitch;
	rs485_tx_message.shoot_fire_en_flag = shoot_cmd.fire_allowed;
	rs485_tx_message.shoot_launched_flag = shoot_cmd.fire_launched;
	rs485_tx_message.auto_aiming_flag = vs_aim_packet_from_nuc.mode;
	rs485_tx_message.control_remote_flag = chassis_cmd.key_state.key_EN_state;
	RS485_Handle_Tx_Data();
#endif	
}

