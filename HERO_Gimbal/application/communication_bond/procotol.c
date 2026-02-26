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
#include "omni_mecanum_chassis.h"
#include "gimbal.h"
#include "shoot.h"

extern bmi088_data_t imu_data;
extern wfly_t *rc_data;
extern INS_behaviour_t INS;
extern float test_data_acc[3];
extern float test_data_gyro[3];

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
	
	// vofa_data_view[0] = tar;
	// vofa_data_view[1] = gimbal_MF9025_motor->receive_data.speed_rps;
	// vofa_data_view[2] = gimbal_MF9025_motor->receive_data.RAD_single_round;

	// vofa_data_view[0] = leg_tar;
	// vofa_data_view[1] = DM_arthrosis_motor[0]->receive_data.position;
	// vofa_data_view[2] = DM_arthrosis_motor[1]->receive_data.position;
	
	// vofa_data_view[0] = shoot_stir_tar;
	// vofa_data_view[1] = shoot_stir_motor->receive_data.speed_aps;

	// VOFA_JustFloat(vofa_data_view, 6);
	// VOFA_JustFloat(vofa_data_view, 15);
}

void RC_Receive_Control(void)
{
	
}
