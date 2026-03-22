/**
******************************************************************************
 * @file    chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "chassis.h"

#include "DM_motor.h"
#include "DJI_motor.h"

/******************************DJI电机底盘初始化实例*****************************/

// static DJI_motor_instance_t *chassis_motor[4];

// motor_init_config_t chassis_motor_config = {
//      .controller_param_init_config = {
// 		.angle_PID = NULL,
// 		.speed_PID = NULL,
// 		.current_PID = NULL,
// 		.torque_PID = NULL,

// 		.other_angle_feedback_ptr = NULL,
// 		.other_speed_feedback_ptr = NULL,

// 		.angle_feedforward_ptr = NULL,
// 		.speed_feedforward_ptr = NULL,
// 		.current_feedforward_ptr = NULL,
// 		.torque_feedforward_ptr = NULL,

// 		.pid_ref = 0.0f,
// 	},
// 	.controller_setting_init_config = {
// 		.close_loop_type = TORQUE_LOOP,

// 		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
// 		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

// 		.angle_feedback_source = MOTOR_FEED,
// 		.speed_feedback_source = MOTOR_FEED,

// 		.feedforward_flag = FEEDFORWARD_NONE,
// 	},

// 	.motor_type = M3508,

// 	.can_init_config = {
// 		.can_handle = &hfdcan3,
// 		.tx_id = 0x00,
// 		.rx_id = 0x00,
// 	},

// 	.motor_control_type = TORQUE_LOOP_CONTROL,
// };

//   for (uint8_t i = 0; i < 4; i++)
//   {
//     chassis_motor_config.can_init_config.tx_id = i + 1;
//     chassis_motor[i] = DJI_Motor_Init(&chassis_config);
//   }

/******************************DJI电机底盘初始化实例*****************************/
