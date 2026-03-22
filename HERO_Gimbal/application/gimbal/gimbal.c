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

#include "INS.h"
#include "user_lib.h"
#include "kalman_one_filter.h"

gimbal_cmd_t gimbal_cmd;

DM_motor_instance_t *pitch_motor;

float pitch_G ;

//imu参数
PID_t pitch_angle_pid = {
    .kp = 12.5f,
    .ki = 0.0f,
    .kd = 1.25f,
    .kf = 0.0f,
    .output_limit = 10.0f, 
    .fout_limit = 10.0f,
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};


PID_t pitch_speed_pid = {
    .kp = 7.5f,
    .ki = 0.0125f,
    .kd = 2.25f,
	.kf = 0.5f,
    .output_limit = 15.0f, 
    .fout_limit = 5.0f,
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

// 云台电机实例初始化
motor_init_config_t pitch_init = {
    .controller_param_init_config = {
        .angle_PID = &pitch_angle_pid,
        .speed_PID = &pitch_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &(INS.Pitch),
        .other_speed_feedback_ptr = &(INS.Gyro[0]),

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = &(pitch_G),

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        // .outer_loop_type = OPEN_LOOP,
        // .close_loop_type = OPEN_LOOP,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

        // .angle_feedback_source = MOTOR_FEED,
        // .speed_feedback_source = MOTOR_FEED,
        .angle_feedback_source = OTHER_FEED,
				 .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = TORQUE_FEEDFORWARD,
			
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

kalman_one_filter_t pitch_mess_kf; //pitch的卡尔曼滤波

void PitchMesskf_Init(void)
{
	Kalman_One_Init(&pitch_mess_kf, 0.01f, 5.0f);
}

void Gimbal_Init(void)
{
    pitch_motor = DM_Motor_Init(&pitch_init);
    pitch_motor -> dm_mode = MIT_MODE ;
    pitch_motor -> contorl_mode_state = SINGLE_TORQUE ;
    pitch_motor ->motor_feedback = DM_MOTOR_ABSOLUTE ;
    PitchMesskf_Init();
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

float pitch_vs_tar ;
float pitch_imu_test ;
float pitch_imu_speed_test ;
float pitch_motor_test ;
float pitch_motor_torque_test ;

float pitch_angle_kp_test ;
float pitch_angle_ki_test ;
float pitch_angle_kd_test ;
float pitch_angle_kf_test ;
float pitch_angle_outpitlimit_test ;
float pitch_speed_kp_test ;
float pitch_speed_ki_test ;
float pitch_speed_kd_test ;
float pitch_speed_kf_test ;
float pitch_speed_outpitlimit_test ;

float pitch_m = 3.25f;
float pitch_l = -0.1f ;
// 更新目标量
void Gimbal_Reference( )
{
    pitch_G = pitch_m * 9.8 * pitch_l * cosf(INS.Pitch);
    pitch_vs_tar = vs_aim_packet_from_nuc.pitch ;
    pitch_imu_test = - INS.Pitch ;
    pitch_imu_speed_test = INS.Gyro[0] ;
    pitch_motor_test = - pitch_motor->receive_data.position ;
    pitch_motor_torque_test = pitch_motor->receive_data.torque ;

    // pitch_motor->motor_controller.angle_PID->kp = pitch_angle_kp_test ;
    // pitch_motor->motor_controller.angle_PID->ki = pitch_angle_ki_test ;
    // pitch_motor->motor_controller.angle_PID->kd = pitch_angle_kd_test ;
    // pitch_motor->motor_controller.angle_PID->kf = pitch_angle_kf_test ;
    // pitch_motor->motor_controller.angle_PID->output_limit = pitch_angle_outpitlimit_test ;
    // pitch_motor->motor_controller.speed_PID->kp = pitch_speed_kp_test ;
    // pitch_motor->motor_controller.speed_PID->ki = pitch_speed_ki_test ;
    // pitch_motor->motor_controller.speed_PID->kd = pitch_speed_kd_test ;
    // pitch_motor->motor_controller.speed_PID->kf = pitch_speed_kf_test ;
    // pitch_motor->motor_controller.speed_PID->output_limit = pitch_speed_outpitlimit_test ;

}

float pitch_target_pro ;
// 计算控制量
void Gimbal_Console( )
{
    if( gimbal_cmd.mode == GIMBAL_ENABLE )
    {
        // gimbal_cmd.pitch_v = (float)rc_data->rc.rocker_r1 * REMOTE_PITCH_SEN;
        gimbal_cmd.pitch_v = (float)uart2_rx_message.rocker_r1 * REMOTE_PITCH_SEN;
        if(pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
        {
            if(gimbal_cmd.pitch_v > 0.0f)
            {
                gimbal_cmd.pitch_target +=gimbal_cmd.pitch_v ;
            }
            else
            {
                gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
            }
        }
        else if(pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
        {
            if(gimbal_cmd.pitch_v < 0.0f)
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
            }
            else
            {
                gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
            }
        }
        else
        {
            gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
        }
    }
    else if(gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
    {
        if(vs_aim_packet_from_nuc.mode == 1 || vs_aim_packet_from_nuc.mode == 2)
        {
            // if(pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            // {
            //     if(vs_aim_packet_from_nuc.pitch > gimbal_cmd.pitch_target)
            //     {
            //         gimbal_cmd.pitch_target = - vs_aim_packet_from_nuc.pitch;
            //     }
            //     else
            //     {
            //         gimbal_cmd.pitch_target = INS.Pitch ;
            //     }
            // }
            // else if(pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            // {
            //     if(vs_aim_packet_from_nuc.pitch < gimbal_cmd.pitch_target)
            //     {
            //         gimbal_cmd.pitch_target = - vs_aim_packet_from_nuc.pitch;
            //     }
            //     else
            //     {
            //         gimbal_cmd.pitch_target = INS.Pitch ;
            //     }
            // }
            // else
            // {
            //     gimbal_cmd.pitch_v = 0.0f;
            //     gimbal_cmd.pitch_target = - vs_aim_packet_from_nuc.pitch;
            // }

            gimbal_cmd.pitch_v = (float)uart2_rx_message.rocker_r1 * REMOTE_PITCH_SEN;
            if(pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            {
                if(gimbal_cmd.pitch_v > 0.0f)
                {
                    gimbal_cmd.pitch_target +=gimbal_cmd.pitch_v ;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
                }
            }
            else if(pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            {
                if(gimbal_cmd.pitch_v < 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
                }
            }
            else
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
            }
        }
        else if(vs_aim_packet_from_nuc.mode == 0)
        {
            gimbal_cmd.pitch_v = (float)uart2_rx_message.rocker_r1 * REMOTE_PITCH_SEN;
            if(pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            {
                if(gimbal_cmd.pitch_v > 0.0f)
                {
                    gimbal_cmd.pitch_target +=gimbal_cmd.pitch_v ;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
                }
            }
            else if(pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            {
                if(gimbal_cmd.pitch_v < 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target ;
                }
            }
            else
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v ;
            }
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
    // pitch_motor ->transmit_data.velocity_des = PITCH_VELOCITY_MAX;

    // while(gimbal_cmd.pitch_target - INS.Pitch > PI)
    //     gimbal_cmd.pitch_target -= 2 * PI ;
    // while(gimbal_cmd.pitch_target - INS.Pitch < - PI)
    //     gimbal_cmd.pitch_target += 2 * PI ;
        
    if(gimbal_cmd.pitch_target <= PTICH_MIN_ANGLE)
    {
        gimbal_cmd.pitch_target = PTICH_MIN_ANGLE;
    }
    else if(gimbal_cmd.pitch_target >= PTICH_MAX_ANGLE)
    {
    gimbal_cmd.pitch_target = PTICH_MAX_ANGLE;
    }
}
// 发送控制量
void Gimbal_Send_Cmd( )
{
    pitch_target_pro = Kalman_One_Filter(&pitch_mess_kf , gimbal_cmd.pitch_target) ;
    DM_Motor_SetTar(pitch_motor , gimbal_cmd.pitch_target);
    // DM_Motor_SetTar(pitch_motor , pitch_target_pro);
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