/**
******************************************************************************
 * @file    gimbal.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "gimbal.h"
#include "remote_control.h"

#include "pid.h"
#include "rs485.h"

LK_motor_instance_t *gimbal_MF9025_motor;
gimbal_cmd_t gimbal_cmd;

// PID_t gimbal_angle_pid = {
//     // .kp = 5.0f,
//     // .ki = 0.0f,
//     // .kd = 2.50f,
//     .kp = 40.0f,
//     .ki = 0.0f,
//     .kd = 75.0f,
//     .output_limit = 20.0f, 
//     .integral_limit = 10.0f,
//     .dead_band = 0.0f,
// };

// PID_t gimbal_speed_pid = {
//     // .kp = 100.0f,
//     // .ki = 2.50f,
//     // .kd = 0.00f,
//     .kp = 200.0f,
//     .ki = 0.65f,
//     .kd = 0.00f,
//     .output_limit = 10000.0f, 
//     .integral_limit = 2000.0f,
//     .dead_band = 0.0f,
// };

PID_t gimbal_angle_pid = {
    .kp = 4.0f,
    .ki = 0.0f,
    .kd = 3.0f,
    .output_limit = 20.0f, 
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_speed_pid = {
    .kp = 200.0f,
    .ki = 0.085f,
    .kd = 0.0f,
		.kf = 130.0f,
    .output_limit = 2048.0f, 
    .integral_limit = 100000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t gimbal_motor_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_angle_pid,
        .speed_PID = &gimbal_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &(uart2_rx_message.angle_yaw),
        .other_speed_feedback_ptr = &(uart2_rx_message.speed_yaw),

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        // .outer_loop_type = SPEED_LOOP,
        // .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        // .angle_feedback_source = MOTOR_FEED,
        // .speed_feedback_source = MOTOR_FEED,
        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
			
		.control_button = POLYCYCLIC_LOOP_CONTROL,
    },

    .motor_type = MF9025,

    .can_init_config = {
//        .can_mode = FDCAN_CLASSIC_CAN,
        .can_handle = &hfdcan3,
			
        .tx_id = 0x141,
        .rx_id = 0x141,
    },
};

static void Gimbal_Enable(void);
static void Gimbal_Disable(void);
static void Gimbal_Stop(void);

// float target_yaw = 2.166f;
float target_yaw = 0.0f;

void Gimbal_Init(void)
{
    gimbal_MF9025_motor = LK_Motor_Init(&gimbal_motor_init);
    gimbal_MF9025_motor->lk_ctrl_mode = TORQUE_MODE;
    // gimbal_MF9025_motor->motor_feedback = LK_MOTOR_DIFF;
    gimbal_MF9025_motor->motor_feedback = LK_MOTOR_ABSOLUTE;

    Gimbal_Enable();
    LK_Motor_GetData(gimbal_MF9025_motor);
    // target_yaw = gimbal_MF9025_motor ->receive_data.RAD_single_round;
}

extern RC_ctrl_t *rc_data;

void Gimbal_Set_Mode()
{
    if( rc_data -> rc . switch_left == 3)
    // if(uart2_rx_message.switch_right == 3 )
    {
        gimbal_cmd.mode = GIMBAL_ENABLE;
    }
    else if( rc_data -> rc . switch_left == 1 &&  rc_data -> rc . switch_right == 1)
    // else if( uart2_rx_message.switch_left == 1 &&  uart2_rx_message.switch_right == 1)
    {
        // gimbal_cmd.mode = GIMBAL_STOP;
        gimbal_cmd.mode = GIMBAL_DISABLE;
    }
    else
    {
        // gimbal_cmd.mode = GIMBAL_DISABLE;
        gimbal_cmd.mode = GIMBAL_STOP;
    }

    if( gimbal_cmd.mode == GIMBAL_ENABLE && gimbal_MF9025_motor->motor_state_flag != MOTOR_ENABLE)
    // if( gimbal_cmd.mode == GIMBAL_ENABLE )
    {
        Gimbal_Enable();
    }
    else if( gimbal_cmd.mode == GIMBAL_STOP && gimbal_MF9025_motor->motor_state_flag != MOTOR_ENABLE)
    // else if( gimbal_cmd.mode == GIMBAL_STOP )
    {
        Gimbal_Stop();
    }
    else if ( gimbal_cmd.mode == GIMBAL_DISABLE && gimbal_MF9025_motor->motor_state_flag != MOTOR_DISABLE)
    // else if ( gimbal_cmd.mode == GIMBAL_DISABLE )
    {
        Gimbal_Disable();
    }
}

void Gimbal_Reference( )
{
    if(gimbal_cmd.mode == GIMBAL_ENABLE)
    {
        gimbal_cmd.v_yaw = -(float) rc_data -> rc . rocker_r_ * REMOTE_YAW_SEN;
        // gimbal_cmd.v_yaw = -(float) uart2_rx_message.rocker_r_ * REMOTE_YAW_SEN;
    }
    else if(gimbal_cmd.mode == GIMBAL_STOP)
    {
        gimbal_cmd.v_yaw = 0.0f;
    }
}

//云台pid调整临时变量
float gimbal_speed_kp_test;
float gimbal_speed_ki_test;
float gimbal_speed_kd_test;
float gimbal_speed_test;

float gimbal_angle_kp_test;
float gimbal_angle_ki_test;
float gimbal_angle_kd_test;
float gimbal_angle_test;

// uint16_t sin_cnt = 0;
// float frq = 0.0f;
// float val = 0.0f;

void Gimbal_Console( )
{
    target_yaw += gimbal_cmd.v_yaw ;
    while(target_yaw - uart2_rx_message.angle_yaw> PI)
        target_yaw -= 2 * PI ;
    while(target_yaw - uart2_rx_message.angle_yaw < -PI)
        target_yaw += 2 * PI ;


    // if((arm_sin_f32(2.0f * PI * 0.1f * (float)sin_cnt / 1000.0f) * 6.28f) >= 0)
    // {
    //     target_yaw = arm_sin_f32(2.0f * PI * 0.1f * (float)sin_cnt / 1000.0f) * 6.28f ;
    // }
    // else
    // {
    //     target_yaw = - (arm_sin_f32(2.0f * PI * 0.1f * (float)sin_cnt / 1000.0f) * 6.28f);
    // }
    // sin_cnt ++ ;
    // sin_cnt %= 1000 ;
    // target_yaw = val +1 + (arm_sin_f32(2.0f * PI * frq * (float)sin_cnt / 1000.0f) * val) ;

    // gimbal_MF9025_motor->motor_controller.speed_PID->kp = gimbal_speed_kp_test;
    // gimbal_MF9025_motor->motor_controller.speed_PID->ki = gimbal_speed_ki_test;
    // gimbal_MF9025_motor->motor_controller.speed_PID->kd = gimbal_speed_kd_test;
    // gimbal_speed_test = gimbal_MF9025_motor->receive_data.speed_rps;
    gimbal_speed_test = uart2_rx_message.speed_yaw;

    // gimbal_MF9025_motor->motor_controller.angle_PID->kp = gimbal_angle_kp_test;
    // gimbal_MF9025_motor->motor_controller.angle_PID->ki = gimbal_angle_ki_test;
    // gimbal_MF9025_motor->motor_controller.angle_PID->kd = gimbal_angle_kd_test; 
    // gimbal_angle_test = gimbal_MF9025_motor->receive_data.RAD_single_round;
    gimbal_angle_test = uart2_rx_message.angle_yaw;

//    gimbal_MF9025_motor->receive_data.lk_diff = - gimbal_cmd.v_yaw;
}


void Gimbal_Send_Cmd()
{
    LK_Motor_SetTar(gimbal_MF9025_motor, target_yaw);
    LK_Motor_Control(NULL);
}



static void Gimbal_Enable(void)
{
    LK_Motor_Enable(gimbal_MF9025_motor);
    LK_Motor_Start(gimbal_MF9025_motor);
}

static void Gimbal_Disable(void)
{
    LK_Motor_Stop(gimbal_MF9025_motor);
    LK_Motor_Disable(gimbal_MF9025_motor);
}

static void Gimbal_Stop(void)
{
    // LK_Motor_Stop(gimbal_MF9025_motor);
    LK_Motor_Enable(gimbal_MF9025_motor);
    LK_Motor_Start(gimbal_MF9025_motor);
}
