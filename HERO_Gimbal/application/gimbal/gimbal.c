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

#include "servo.h"
#include "rs485.h"
#include "serial.h"

#include "remote_control.h"
#include "remote_vt03.h"

#include "pid.h"

#include "INS.h"
#include "user_lib.h"
#include "kalman_one_filter.h"

DM_motor_instance_t *pitch_motor;
gimbal_cmd_t gimbal_cmd;

float pitch_G;

// imu参数
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

servo_instance_t *vt_servo;

motor_init_config_t vt_servo_init = {
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

        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = TORQUE_FEEDFORWARD,

        .control_button = POLYCYCLIC_LOOP_CONTROL,
    },

    .motor_type = SERVO,
};

pwm_init_config_t vt_pwm_init = {
    .htim = &htim1,
    .channel = TIM_CHANNEL_1,
    .dutyratio = 0.0f,
    .period = 20000 - 1,
};

// PITCH斜坡函数
ramp_function_source_t *pitch_speed_ramp;

ramp_init_config_t pitch_speed_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.0f,             // 最大输出
    .min_value = 0.0f,             // 最小输出
    .increase_value = 0.005f,      // 加速度
    .decrease_value = 0.005f,      // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};

kalman_one_filter_t pitch_mess_kf; // pitch的卡尔曼滤波

void PitchMesskf_Init(void)
{
    Kalman_One_Init(&pitch_mess_kf, 0.01f, 5.0f);
}

void Gimbal_Init(void)
{
    pitch_motor = DM_Motor_Init(&pitch_init);
    pitch_motor->dm_mode = MIT_MODE;
    pitch_motor->contorl_mode_state = SINGLE_TORQUE;
    pitch_motor->motor_feedback = DM_MOTOR_ABSOLUTE;
    PitchMesskf_Init();
    gimbal_cmd.pitch_target = 0.0f;

    // pitch应该也要加斜坡,感觉???
    pitch_speed_ramp = ramp_init(&pitch_speed_ramp_init);

    vt_servo = Servo_Init(&vt_servo_init, &vt_pwm_init);
}

void Gimbal_Observer(void)
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

/*********************
 *| 左拨杆 switch_left | 右拨杆 switch_right | 云台状态 | 说明 |
 *|---|---:|---|---|
 *| 1 | 1 | GIMBAL_DISABLE | 云台失能 |
 *| 1 | 3 | GIMBAL_AUTO_AIMING | 自瞄模式 |
 *| 1 | 2 | GIMBAL_AUTO_AIMING | 自瞄模式 |
 *| 3 | 1 | GIMBAL_ENABLE | 手动控制 |
 *| 3 | 3 | GIMBAL_ENABLE | 手动控制 |
 *| 3 | 2 | GIMBAL_ENABLE | 手动控制 |
 *| 2 | 1 | GIMBAL_ZERO | 回中模式 |
 *| 2 | 3 | GIMBAL_ENABLE | 手动控制 |
 *| 2 | 2 | 进入键鼠入口 | 不直接定模式，转键鼠逻辑 |
 *
 * 云台一旦进入自瞄模式,底盘需要传输对应的状态位和键鼠数据过来
 *********************/

extern RC_ctrl_t *rc_data;
extern VT03_ctrl_t *vt03_data;

#include "chassis.h"

// 设置云台模式
void Gimbal_Set_Mode()
{
        if(rc_data -> online == 0 && vt03_data -> online == 0)
        {
            gimbal_cmd.mode = GIMBAL_DISABLE;
        }
        else
        {
            //遥控器控制
            gimbal_cmd.key_state.key_EN_state = 0;
            if( rc_data -> rc . switch_left == 1 )
            {
                switch (rc_data -> rc . switch_right)
                {
                case 1:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_DISABLE;
                    break;
                case 3:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_AUTO_AIMING;
                    break;
                case 2:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_AUTO_AIMING;
                    break;
                default:
                    gimbal_cmd.mode = GIMBAL_DISABLE;
                    break;
                }
            }
            else if( rc_data -> rc . switch_left == 3 )
            {
                switch (rc_data -> rc . switch_right)
                {
                case 1:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_ENABLE;
                    break;
                case 3:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_ENABLE;
                    break;
                case 2:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_ENABLE;
                    break;
                default:
                    gimbal_cmd.mode = GIMBAL_DISABLE;
                    break;
                }
            }
            else if( rc_data -> rc . switch_left == 2 )
            {
                switch (rc_data -> rc . switch_right)
                {
                case 1:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_ZERO;
                    break;
                case 3:
                    /* code */
                    gimbal_cmd.mode = GIMBAL_ENABLE;
                    break;
                case 2:
                    /* code */
                    gimbal_cmd.key_state.key_EN_state = 1;
                    break;
                default:
                    gimbal_cmd.mode = GIMBAL_DISABLE;
                    break;
                }
            }
            else
            {
                gimbal_cmd.mode = GIMBAL_DISABLE;
            }
        }        

    if (gimbal_cmd.key_state.key_EN_state == 1)
    {
        if (chassis_cmd.mode == CHASSIS_DISABLE)
        {
            gimbal_cmd.mode = GIMBAL_DISABLE;
            gimbal_cmd.key_state.gimbal_EN_state = 0;
        }
        else if(chassis_cmd.mode != CHASSIS_DISABLE)
        {
            gimbal_cmd.mode = GIMBAL_ENABLE;
            gimbal_cmd.key_state.gimbal_EN_state = 1;
        }
        if(gimbal_cmd.key_state.gimbal_EN_state == 1)
        {
            /* 蹬腿云台跟随键鼠 */
            if (vt03_data->mouse.press_r == 1)
            {
                gimbal_cmd.mode = GIMBAL_AUTO_AIMING;
            }
            else if(vt03_data->mouse.press_r == 0 && gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
            {
                gimbal_cmd.mode = GIMBAL_ENABLE;
            }
            if(vt03_data->key->q == 1)
            {
                gimbal_cmd.mode = GIMBAL_ZERO;
            }
            else if(vt03_data->key->q ==0 && gimbal_cmd.mode == GIMBAL_ZERO)
            {
                gimbal_cmd.mode = GIMBAL_ENABLE;
            }
        }
    }

    if (gimbal_cmd.mode == GIMBAL_ENABLE || gimbal_cmd.mode == GIMBAL_AUTO_AIMING || gimbal_cmd.mode == GIMBAL_ZERO)
    {
        Gimbal_Enable();
        DM_Motor_Start(pitch_motor);
    }
    else
    {
        Gimbal_Disable();
    }
}

static float pitch_vs_tar;
static float pitch_imu_test;
static float pitch_imu_speed_test;
static float pitch_motor_test;
static float pitch_motor_torque_test;

static float pitch_angle_kp_test;
static float pitch_angle_ki_test;
static float pitch_angle_kd_test;
static float pitch_angle_kf_test;
static float pitch_angle_outpitlimit_test;
static float pitch_speed_kp_test;
static float pitch_speed_ki_test;
static float pitch_speed_kd_test;
static float pitch_speed_kf_test;
static float pitch_speed_outpitlimit_test;

static float pitch_m = 3.25f;
static float pitch_l = -0.1f;

// 更新目标量
void Gimbal_Reference()
{
    pitch_G = pitch_m * 9.8f * pitch_l * cosf(INS.Pitch);
    pitch_vs_tar = vs_aim_packet_from_nuc.pitch;
    pitch_imu_test = -INS.Pitch;
    pitch_imu_speed_test = INS.Gyro[0];
    pitch_motor_test = -pitch_motor->receive_data.position;
    pitch_motor_torque_test = pitch_motor->receive_data.torque;

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

static float pitch_target_pro;

// 计算控制量
void Gimbal_Console()
{
    if (gimbal_cmd.mode == GIMBAL_ENABLE)
    {
        if (gimbal_cmd.key_state.key_EN_state == 0)
        {
            gimbal_cmd.yaw_v = -(float)rc_data->rc.rocker_r_ * REMOTE_YAW_SEN;
            gimbal_cmd.pitch_v = (float)rc_data->rc.rocker_r1 * REMOTE_PITCH_SEN;
        }
        else if(gimbal_cmd.key_state.key_EN_state == 1)
        {
            gimbal_cmd.yaw_v = (float)vt03_data->mouse.y * KEY_PITCH_SEN;
            gimbal_cmd.pitch_v = ramp_calc(pitch_speed_ramp, (float)vt03_data->mouse.y * KEY_PITCH_SEN);
            pitch_speed_ramp->real_value = gimbal_cmd.pitch_v;
        }

        if (pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
        {
            if (gimbal_cmd.pitch_v > 0.0f)
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
            }
            else
            {
                gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
            }
        }
        else if (pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
        {
            if (gimbal_cmd.pitch_v < 0.0f)
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
            }
            else
            {
                gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
            }
        }
        else
        {
            gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
        }
    }
    else if (gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
    {
        /* hyw???为什么这里视觉的pitch整段被注释了🤷‍♂️ */
        if (vs_aim_packet_from_nuc.mode == 0)
        {
            if(gimbal_cmd.key_state.key_EN_state == 0)
            {
                gimbal_cmd.yaw_v = -(float) rc_data -> rc . rocker_r_ * REMOTE_YAW_SEN;
                gimbal_cmd.pitch_v = (float)rc_data->rc.rocker_r1 * REMOTE_PITCH_SEN;
            }
            else if(gimbal_cmd.key_state.key_EN_state == 1)
            {
                gimbal_cmd.yaw_v = (float)vt03_data->mouse.y * KEY_PITCH_SEN;
                gimbal_cmd.pitch_v = ramp_calc(pitch_speed_ramp, (float)vt03_data->mouse.y * KEY_PITCH_SEN);
                pitch_speed_ramp->real_value = gimbal_cmd.pitch_v;
            }

            if (pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            {
                if (gimbal_cmd.pitch_v > 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
                }
            }
            else if (pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            {
                if (gimbal_cmd.pitch_v < 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
                }
            }
            else
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
            }

        }
        else if (vs_aim_packet_from_nuc.mode == 1 || vs_aim_packet_from_nuc.mode == 2)
        {
            gimbal_cmd.yaw_target = vs_aim_packet_from_nuc.yaw;

            if (pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            {
                if (vs_aim_packet_from_nuc.pitch > gimbal_cmd.pitch_target)
                {
                    gimbal_cmd.pitch_target = -vs_aim_packet_from_nuc.pitch;
                } 
                else
                {
                    gimbal_cmd.pitch_target = INS.Pitch;
                }
            }
            else if (pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            {
                if (vs_aim_packet_from_nuc.pitch < gimbal_cmd.pitch_target)
                {
                    gimbal_cmd.pitch_target = -vs_aim_packet_from_nuc.pitch;
                }
                else
                {
                    gimbal_cmd.pitch_target = INS.Pitch;
                }
            }
            else
            {
                gimbal_cmd.pitch_v = 0.0f;
                gimbal_cmd.pitch_target = -vs_aim_packet_from_nuc.pitch;
            }

            if (pitch_motor->receive_data.position <= PTICH_MIN_ANGLE)
            {
                if (gimbal_cmd.pitch_v > 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
                }
            }
            else if (pitch_motor->receive_data.position >= PTICH_MAX_ANGLE)
            {
                if (gimbal_cmd.pitch_v < 0.0f)
                {
                    gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
                }
                else
                {
                    gimbal_cmd.pitch_target = gimbal_cmd.pitch_target;
                }
            }
            else
            {
                gimbal_cmd.pitch_target += gimbal_cmd.pitch_v;
            }
        }
        else
        {
            gimbal_cmd.pitch_v = 0.0f;
            gimbal_cmd.yaw_v = 0.0f;
        }
    }
    else
    {
        gimbal_cmd.pitch_v = 0.0f;
        gimbal_cmd.yaw_v = 0.0f;
    }

    if (gimbal_cmd.pitch_target <= PTICH_MIN_ANGLE)
    {
        gimbal_cmd.pitch_target = PTICH_MIN_ANGLE;
    }
    else if (gimbal_cmd.pitch_target >= PTICH_MAX_ANGLE)
    {
        gimbal_cmd.pitch_target = PTICH_MAX_ANGLE;
    }	
}

// 发送控制量
void Gimbal_Send_Cmd()
{
    pitch_target_pro = Kalman_One_Filter(&pitch_mess_kf, gimbal_cmd.pitch_target);
    DM_Motor_SetTar(pitch_motor, gimbal_cmd.pitch_target);

    DM_Motor_Control(pitch_motor);

    // Servo_SetTar(vt_servo, ???, ABS);
    // Servo_Control(vt_servo);
}

static void Gimbal_Enable(void)
{
    DM_Motor_Enable(pitch_motor);
}

static void Gimbal_Disable(void)
{
    DM_Motor_Disable(pitch_motor);
}
