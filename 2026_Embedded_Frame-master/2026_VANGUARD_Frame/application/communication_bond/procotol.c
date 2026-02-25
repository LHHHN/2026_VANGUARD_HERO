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

	VOFA_Send_Data(vofa_data_view, 20);
	// VOFA_JustFloat(vofa_data_view, 7);
}

void RC_Receive_Control(void)
{
}
