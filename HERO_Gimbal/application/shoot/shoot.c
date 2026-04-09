/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"

#include "remote_control.h"
#include "pid.h"
#include "bsp_can.h"
#include "rs485.h"

DJI_motor_instance_t *shoot_m3508_motor[3];
shoot_cmd_t shoot_cmd;

CAN_instance_t shoot_m3519_motor = {
    .can_handle = &hfdcan1,
    .tx_header.Identifier = 0x010,
    .tx_header.IdType = FDCAN_STANDARD_ID,
    .tx_header.DataLength = FDCAN_DLC_BYTES_8,
    .tx_buff = {0}};

PID_t shoot_3508_speed_pid = {
    .kp = 50.0f,
    .ki = 2.5f,
    .kd = 50.0f,
    .output_limit = 10000.0f,
    .integral_limit = 4000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t shoot_m3508_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &shoot_3508_speed_pid,
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
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,

        .control_button = POLYCYCLIC_LOOP_CONTROL,
    },

    .motor_type = M3508,

    .can_init_config = {
        //        .can_mode = FDCAN_CLASSIC_CAN,
        .can_handle = &hfdcan1,
        .tx_id = 0x04,
        .rx_id = 0x04,
    },
};

float shoot_tar_1 = 3500.0f;
float shoot_tar_2 = 3800.0f;

float shoot_tar_16mps_1 = 4720.0f;
float shoot_tar_16mps_2 = 4905.0f;

// float shoot_tar_1 = 4750.0f;
// float shoot_tar_2 = 4900.0f;

// float shoot_tar_1 = 4600.0f;
// float shoot_tar_2 = 4800.0f;

// float shoot_tar_16mps_1 = 4600.0f;
// float shoot_tar_16mps_2 = 4780.0f;

void Shoot_Init(void)
{
    for (int i = 0; i < 3; i++)
    {
        shoot_m3508_init.can_init_config.tx_id = 0x01 + i;
        shoot_m3508_init.can_init_config.rx_id = 0x01 + i;
        shoot_m3508_motor[i] = DJI_Motor_Init(&shoot_m3508_init);
        shoot_m3508_motor[i]->motor_feedback = ORIGIN;
    }
}

static void Shoot_Enable(void);
static void Shoot_Disable(void);

extern RC_ctrl_t *rc_data;

void Shoot_Set_Mode(void)
{
    // #ifdef RMUC
    //     // 不再通过拨杆组合直接判断发射模式，底盘主控直接下发发射模式
    //     switch (uart2_rx_message.shoot_mode)
    //     {
    //     case SHOOT_DISABLE:
    //         shoot_cmd.mode = SHOOT_DISABLE;
    //         break;

    //     case SHOOT_ENABLE:
    //         shoot_cmd.mode = SHOOT_ENABLE;
    //         shoot_cmd.shoot_speed_set = SHOOT_SPEED_12MPS;
    //         break;

    //     case SHOOT_AUTO_AIMING:
    //         shoot_cmd.mode = SHOOT_AUTO_AIMING;
    //         shoot_cmd.shoot_speed_set = SHOOT_SPEED_16MPS;
    //         break;

    //     case SHOOT_STOP:
    //         shoot_cmd.mode = SHOOT_STOP;
    //         break;

    //     default:
    //         shoot_cmd.mode = SHOOT_STOP;
    //         break;
    //     }

    //     if (shoot_cmd.mode == SHOOT_ENABLE || shoot_cmd.mode == SHOOT_STOP || shoot_cmd.mode == SHOOT_AUTO_AIMING)
    //     {
    //         Shoot_Enable();
    //     }
    //     else
    //     {
    //         Shoot_Disable();
    //     }
    // #elif defined(RMUL)
    // 不再通过拨杆组合直接判断发射模式，底盘主控直接下发发射模式
    switch (uart2_rx_message.shoot_mode)
    {
    case SHOOT_DISABLE:
        shoot_cmd.mode = SHOOT_DISABLE;
        break;

    case SHOOT_ENABLE:
        shoot_cmd.mode = SHOOT_ENABLE;
        // shoot_cmd.shoot_speed_set = SHOOT_SPEED_12MPS;
        if (uart2_rx_message.rc_switch == 0b00100100)
        {
            shoot_cmd.shoot_speed_set = SHOOT_SPEED_12MPS;
        }
        else if (uart2_rx_message.rc_switch == 0b00100010)
        {
            shoot_cmd.shoot_speed_set = SHOOT_SPEED_16MPS;
        }
        break;

    case SHOOT_AUTO_AIMING:
        shoot_cmd.mode = SHOOT_AUTO_AIMING;
        if (uart2_rx_message.rc_switch == 0b00001100)
        {
            shoot_cmd.shoot_speed_set = SHOOT_SPEED_12MPS;
        }
        else if (uart2_rx_message.rc_switch == 0b00001010)
        {
            shoot_cmd.shoot_speed_set = SHOOT_SPEED_16MPS;
        }
        break;

    default:
        shoot_cmd.mode = SHOOT_DISABLE;
        break;
    }

    if (shoot_cmd.mode == SHOOT_ENABLE || shoot_cmd.mode == SHOOT_AUTO_AIMING)
    {
        Shoot_Enable();
    }
    else
    {
        Shoot_Disable();
    }
    // #endif
}

float shoot_kp_test;
float shoot_ki_test;
float shoot_kd_test;
float shoot_speed_test_0;
float shoot_speed_test_1;
float shoot_speed_test_2;

void Shoot_Reference(void)
{
    static uint8_t last_key_cnt[16] = {0};
    static uint8_t key_mode_last = 0;

#define KEY_CLICK(k) (rc_data->key_count[KEY_PRESS][(k)] != last_key_cnt[(k)])
#define KEY_ACK(k) (last_key_cnt[(k)] = rc_data->key_count[KEY_PRESS][(k)])

#ifdef RMUC

    for (int i = 0; i < 3; i++)
    {
        // shoot_m3508_motor[i]->motor_controller.speed_PID->kp = shoot_kp_test;
        // shoot_m3508_motor[i]->motor_controller.speed_PID->ki = shoot_ki_test;
        // shoot_m3508_motor[i]->motor_controller.speed_PID->kd = shoot_kd_test;
    }
    shoot_speed_test_0 = shoot_m3508_motor[0]->receive_data.speed;
    shoot_speed_test_1 = shoot_m3508_motor[1]->receive_data.speed;
    shoot_speed_test_2 = -shoot_m3508_motor[2]->receive_data.speed;

    float shoot_speed_average = (shoot_speed_test_0 + shoot_speed_test_1 + shoot_speed_test_2) / 3.0f;

    if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_12MPS)
    {
        if (shoot_speed_average < (shoot_tar_1 - 500.0f))
        {
            shoot_cmd.fire_launched = 1;
        }
        else if (shoot_speed_average > (shoot_tar_1 - 100.0f))
        {
            shoot_cmd.fire_launched = 0;
        }
    }
    else if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_16MPS)
    {
        if (shoot_speed_average < (shoot_tar_16mps_1 - 500.0f))
        {
            shoot_cmd.fire_launched = 1;
        }
        else if (shoot_speed_average > (shoot_tar_16mps_1 - 100.0f))
        {
            shoot_cmd.fire_launched = 0;
        }
    }
    else
    {
        shoot_cmd.fire_launched = 0;
    }

#elif defined(RMUL)
    for (int i = 0; i < 3; i++)
    {
        // shoot_m3508_motor[i]->motor_controller.speed_PID->kp = shoot_kp_test;
        // shoot_m3508_motor[i]->motor_controller.speed_PID->ki = shoot_ki_test;
        // shoot_m3508_motor[i]->motor_controller.speed_PID->kd = shoot_kd_test;
    }
    shoot_speed_test_0 = shoot_m3508_motor[0]->receive_data.speed;
    shoot_speed_test_1 = shoot_m3508_motor[1]->receive_data.speed;
    shoot_speed_test_2 = -shoot_m3508_motor[2]->receive_data.speed;

    float shoot_speed_average = (shoot_speed_test_0 + shoot_speed_test_1 + shoot_speed_test_2) / 3.0f;

    if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_12MPS)
    {
        if (shoot_speed_average < (shoot_tar_1 - 500.0f))
        {
            shoot_cmd.fire_launched = 1;
        }
        else if (shoot_speed_average > (shoot_tar_1 - 100.0f))
        {
            shoot_cmd.fire_launched = 0;
        }
    }
    else if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_16MPS)
    {
        if (shoot_speed_average < (shoot_tar_16mps_1 - 500.0f))
        {
            shoot_cmd.fire_launched = 1;
        }
        else if (shoot_speed_average > (shoot_tar_16mps_1 - 100.0f))
        {
            shoot_cmd.fire_launched = 0;
        }
    }
    else
    {
        shoot_cmd.fire_launched = 0;
    }

#endif
#undef KEY_CLICK
#undef KEY_ACK
}

void Shoot_Console(void)
{
    if (shoot_cmd.mode == SHOOT_ENABLE || shoot_cmd.mode == SHOOT_AUTO_AIMING)
    {
        if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_12MPS)
        {
            DJI_Motor_Set_Ref(shoot_m3508_motor[0], shoot_tar_1);
            DJI_Motor_Set_Ref(shoot_m3508_motor[1], shoot_tar_1);
            DJI_Motor_Set_Ref(shoot_m3508_motor[2], -shoot_tar_1);
            shoot_m3519_motor.tx_buff[0] = ((int16_t)shoot_tar_2) >> 8;
            shoot_m3519_motor.tx_buff[1] = ((int16_t)shoot_tar_2);
            shoot_m3519_motor.tx_buff[2] = ((int16_t)shoot_tar_2) >> 8;
            shoot_m3519_motor.tx_buff[3] = ((int16_t)shoot_tar_2);
            shoot_m3519_motor.tx_buff[4] = ((int16_t)-shoot_tar_2) >> 8;
            shoot_m3519_motor.tx_buff[5] = ((int16_t)-shoot_tar_2);
            shoot_m3519_motor.tx_buff[6] = 0;
            shoot_m3519_motor.tx_buff[7] = 0;
        }
        else if (shoot_cmd.shoot_speed_set == SHOOT_SPEED_16MPS)
        {
            DJI_Motor_Set_Ref(shoot_m3508_motor[0], shoot_tar_16mps_1);
            DJI_Motor_Set_Ref(shoot_m3508_motor[1], shoot_tar_16mps_1);
            DJI_Motor_Set_Ref(shoot_m3508_motor[2], -shoot_tar_16mps_1);
            shoot_m3519_motor.tx_buff[0] = ((int16_t)shoot_tar_16mps_2) >> 8;
            shoot_m3519_motor.tx_buff[1] = ((int16_t)shoot_tar_16mps_2);
            shoot_m3519_motor.tx_buff[2] = ((int16_t)shoot_tar_16mps_2) >> 8;
            shoot_m3519_motor.tx_buff[3] = ((int16_t)shoot_tar_16mps_2);
            shoot_m3519_motor.tx_buff[4] = ((int16_t)-shoot_tar_16mps_2) >> 8;
            shoot_m3519_motor.tx_buff[5] = ((int16_t)-shoot_tar_2);
            shoot_m3519_motor.tx_buff[6] = 0;
            shoot_m3519_motor.tx_buff[7] = 0;
        }
    }
    else if (shoot_cmd.mode == SHOOT_DISABLE)
    {
        DJI_Motor_Set_Ref(shoot_m3508_motor[0], 0);
        DJI_Motor_Set_Ref(shoot_m3508_motor[1], 0);
        DJI_Motor_Set_Ref(shoot_m3508_motor[2], 0);
        for (int i = 0; i < 8; i++)
        {
            shoot_m3519_motor.tx_buff[i] = 0;
        }
    }
}

void Shoot_Send_Cmd(void)
{
    DJI_Motor_Control(NULL);
    CAN_Transmit(&shoot_m3519_motor, 2);
}

static void Shoot_Enable(void)
{
    for (int i = 0; i < 3; i++)
    {
        DJI_Motor_Enable(shoot_m3508_motor[i]);
    }
    // DJI_Motor_Enable(shoot_m3508_motor);
}

static void Shoot_Disable(void)
{
    for (int i = 0; i < 3; i++)
    {
        DJI_Motor_Disable(shoot_m3508_motor[i]);
    }
    for (int i = 0; i < 8; i++)
    {
        shoot_m3519_motor.tx_buff[i] = 0;
    }
}
