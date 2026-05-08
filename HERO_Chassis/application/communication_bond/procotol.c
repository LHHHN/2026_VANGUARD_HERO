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
#include "super_cap.h"
#include "referee_task.h"
#include "power_ctrl.h"

extern bmi088_data_t imu_data;
// extern wfly_t *rc_data;
extern RC_ctrl_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

extern gimbal_cmd_t gimbal_cmd;
extern shoot_cmd_t shoot_cmd;

extern ramp_function_source_t *gimbal_angle_ramp;

typedef enum
{
	SUPER_CAP_POLICY_ERROR_FALLBACK = 0,
	SUPER_CAP_POLICY_LOW_MINUS_30,
	SUPER_CAP_POLICY_HOLD,
	SUPER_CAP_POLICY_MINUS_15,
	SUPER_CAP_POLICY_PLUS_15,
	SUPER_CAP_POLICY_PLUS_30,
} super_cap_power_policy_e;

static super_cap_power_policy_e super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
float super_cap_chassis_power_target;

static uint16_t SuperCap_Clamp_U16(uint16_t value, uint16_t min, uint16_t max)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;
}

static float SuperCap_Clamp_Float(float value, float min, float max)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;
}

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
	RS485_Handle_Rx_Data();
	if(rs485_command.online == 0)
	{
		rs485_rx_message.chassis_mode = CHASSIS_DISABLE;
		rs485_rx_message.gimbal_mode = GIMBAL_DISABLE;
		rs485_rx_message.shoot_mode = SHOOT_DISABLE;
	}
}

void SuperCap_PowerControl_Update(void)
{
	uint16_t referee_power_limit = SUPER_CAP_POWER_INIT;
	uint16_t referee_energy_buffer = SUPER_CAP_ENERGY_BUFFER_INIT;
	uint8_t cap_energy;
	uint8_t cap_unavailable;
	float target_power;

	if (Super_Cap_instance == NULL)
	{
		chassis_max_power = SUPER_CAP_CHASSIS_POWER_MIN_W;
		super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
		return;
	}

	if (referee_outer_info != NULL)
	{
		referee_power_limit = referee_outer_info->RobotPerformance.chassis_power_limit;
		referee_energy_buffer = referee_outer_info->PowerHeatData.buffer_energy;
	}

	referee_power_limit = SuperCap_Clamp_U16(referee_power_limit,
											SUPER_CAP_POWER_LIMIT_MIN_W,
											SUPER_CAP_POWER_LIMIT_MAX_W);
	referee_energy_buffer = SuperCap_Clamp_U16(referee_energy_buffer,
											  0U,
											  SUPER_CAP_ENERGY_BUFFER_INIT);

	cap_energy = Super_Cap_instance->receive_data.capEnergy;
	cap_unavailable = (Super_Cap_instance->online == 0U) ||
					  (Super_Cap_instance->receive_data.errorCode != SUPER_CAP_ERROR_NONE);

	if (cap_unavailable)
	{
		target_power = (float)referee_power_limit;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_15_RAW)
	{
		target_power = (float)referee_power_limit * 0.70f;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_LOW_MINUS_30;
	}
	else if (cap_energy <= SUPER_CAP_ENERGY_20_RAW)
	{
		target_power = chassis_max_power; // 15%~20%保持上一档，避免边界反复跳变
		super_cap_power_policy = SUPER_CAP_POLICY_HOLD;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_40_RAW)
	{
		target_power = (float)referee_power_limit * 0.85f;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_MINUS_15;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_70_RAW)
	{
		target_power = (float)referee_power_limit * 1.15f;
		Power_Control_Mode = SUPERCAP_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_PLUS_15;
	}
	else
	{
		target_power = (float)referee_power_limit * 1.30f;
		Power_Control_Mode = SUPERCAP_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_PLUS_30;
	}

	super_cap_chassis_power_target = SuperCap_Clamp_Float(target_power,
														 SUPER_CAP_CHASSIS_POWER_MIN_W,
														 SUPER_CAP_CHASSIS_POWER_MAX_W);
	// chassis_max_power = super_cap_chassis_power_target;
	chassis_max_power = 75.0f ;4

	Super_Cap_Enable(Super_Cap_instance);
	Super_Cap_instance->transmit_data.refereePowerLimit = referee_power_limit;
	Super_Cap_instance->transmit_data.refereeEnergyBuffer = referee_energy_buffer;
	Super_Cap_SendData(Super_Cap_instance);
}
