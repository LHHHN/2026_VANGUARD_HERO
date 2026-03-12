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

#include "rs485.h"
#include "pid.h"
#include "serial.h"

gimbal_cmd_t gimbal_cmd;

DM_motor_instance_t *pitch_motor;

// 云台电机实例初始化
motor_init_config_t pitch_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = NULL,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = OPEN_LOOP,
        .close_loop_type = OPEN_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
			
		.control_button = POLYCYCLIC_LOOP_CONTROL,
    },

    .motor_type = DM4310,

    .can_init_config = {
//        .can_mode = FDCAN_CLASSIC_CAN,
        .can_handle = &hfdcan2,
        .tx_id = 0x04,
        .rx_id = 0x14,
    },
};

void Gimbal_Init(void)
{
    pitch_motor = DM_Motor_Init(&pitch_init);
    pitch_motor ->dm_mode = POS_MODE ;

    gimbal_cmd.pitch_target = 0.0f;
}

void Gimbal_Observer( )
{
    // uart2_tx_message.rocker_l_ = rc_data->rc.rocker_l_;
    // uart2_tx_message.rocker_l1 = rc_data->rc.rocker_l1;
    // uart2_tx_message.rocker_r_ = rc_data->rc.rocker_r_;
    // uart2_tx_message.rocker_r1 = rc_data->rc.rocker_r1;
    // uart2_tx_message.dial = rc_data->rc.dial;
    // uart2_tx_message.switch_left = rc_data->rc.switch_left;
    // uart2_tx_message.switch_right = rc_data->rc.switch_right;
}

static void Gimbal_Enable(void);
static void Gimbal_Disable(void);

// 设置云台模式
void Gimbal_Set_Mode( )
{
    // if( rc_data -> rc . switch_right == 3)
    if(((uart2_rx_message.rc_switch & 0x20) == 0x20) || ((uart2_rx_message.rc_switch & 0x11) == 0x11) 
    || ((uart2_rx_message.rc_switch & 0x12) == 0x12)) // 0b00100000 0b00010001 0b00010100
    {
        gimbal_cmd.mode = GIMBAL_ENABLE;
    }
    else if(((uart2_rx_message.rc_switch & 0x0C) == 0x0C) || ((uart2_rx_message.rc_switch & 0x0A) == 0x0A)) // 0b00001100 和0b00001010
    {
        gimbal_cmd.mode = GIMBAL_AUTO_AIMING ;
    }
    // else if( rc_data -> rc . switch_left == 1 &&  rc_data -> rc . switch_right == 1)
    else if((uart2_rx_message.rc_switch & 0x09) == 0x09) //0x00001001
    {
        // gimbal_cmd.mode = GIMBAL_STOP;
        gimbal_cmd.mode = GIMBAL_DISABLE;
    }
    else
    {
        // gimbal_cmd.mode = GIMBAL_DISABLE;
        gimbal_cmd.mode = GIMBAL_STOP;
    }

    if(gimbal_cmd.mode == GIMBAL_ENABLE || gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
    {
        Gimbal_Enable();
        DM_Motor_Start(pitch_motor);
    }
    else if(gimbal_cmd.mode == GIMBAL_STOP)
    {
        Gimbal_Enable();
        DM_Motor_Stop(pitch_motor);
    }
    else
    {
        Gimbal_Disable();
    }
}

// 更新目标量
void Gimbal_Reference( )
{
    if( gimbal_cmd.mode == GIMBAL_ENABLE )
    {
        // gimbal_cmd.pitch_v = (float)rc_data->rc.rocker_r1 * REMOTE_PITCH_SEN;
        gimbal_cmd.pitch_v = (float)uart2_rx_message.rocker_r1 * REMOTE_PITCH_SEN;
        gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
    }
    else if(gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
    {
        if(vs_aim_packet_from_nuc.mode == 1 || vs_aim_packet_from_nuc.mode == 2)
        {
            gimbal_cmd.pitch_v = 0.0f;
            gimbal_cmd.pitch_target = - vs_aim_packet_from_nuc.pitch;
        }
        else if(vs_aim_packet_from_nuc.mode == 0)
        {
            gimbal_cmd.pitch_v = (float)uart2_rx_message.rocker_r1 * REMOTE_PITCH_SEN;
            gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
        }
        else
        {
            gimbal_cmd.pitch_v = 0.0f;
            gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
        }
    }
    else
    {
        gimbal_cmd.pitch_v = 0.0f;
    }
}

// 计算控制量
void Gimbal_Console( )
{
    pitch_motor ->transmit_data.velocity_des = PITCH_VELOCITY_MAX;
    if(gimbal_cmd.pitch_target < PTICH_MIN_ANGLE)
    {
        gimbal_cmd.pitch_target = PTICH_MIN_ANGLE ;
    }
    if(gimbal_cmd.pitch_target > PTICH_MAX_ANGLE)
    {
        gimbal_cmd.pitch_target = PTICH_MAX_ANGLE ;
    }
}
// 发送控制量
void Gimbal_Send_Cmd( )
{
    DM_Motor_SetTar(pitch_motor , gimbal_cmd.pitch_target);
    DM_Motor_Control(pitch_motor);
}

static void Gimbal_Enable(void)
{
    DM_Motor_Enable(pitch_motor);
}

static void Gimbal_Disable(void)
{
    DM_Motor_Disable(pitch_motor);
}