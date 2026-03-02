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

#include "rs485.h"

DJI_motor_instance_t *shoot_stir_motor;
shoot_cmd_t shoot_cmd;

float shoot_stir_tar = -10800; //固定目标转速

// PID_t shoot_stir_angle_pid = {
//     .kp = 0.0f, //5.0f,
//     .ki = 0.0f,
//     .kd = 0.0f, // 0.01f,
//     .output_limit = 10000.0f, 
//     .integral_limit = 5000.0f,
//     .dead_band = 0.0f,
// };

PID_t shoot_stir_speed_pid = {
    .kp = 1.0f, //1.0f,
    .ki = 0.15f, //0.5f,
    .kd = 0.1f,
    .output_limit = 15000.0f, 
    .integral_limit = 7500.0f,
    .dead_band = 0.0f,
};

motor_init_config_t shoot_stir_init = {
    .controller_param_init_config = {
        // .angle_PID = &shoot_stir_angle_pid,
        .angle_PID = NULL,
        .speed_PID = &shoot_stir_speed_pid,
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
        // .outer_loop_type = ANGLE_LOOP,
        // .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
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
        .tx_id = 0x05,
        .rx_id = 0x05,
    },
};

static void Shoot_Enable(void);
static void Shoot_Disable(void);

void Shoot_Init(void)
{
    shoot_stir_motor = DJI_Motor_Init(&shoot_stir_init);
    shoot_stir_motor->motor_feedback = DEGREE;
    // shoot_stir_tar = shoot_stir_motor->receive_data.total_angle;
    shoot_cmd.v_shoot_stir = 0.0f;
	
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
}


extern RC_ctrl_t *rc_data;

void Shoot_Set_Mode(void)
{
    if( rc_data -> rc . switch_left == 2 &&  rc_data -> rc . switch_right == 3)
    // if( uart2_rx_message.switch_left == 2 &&  uart2_rx_message.switch_right == 3)
    {
        shoot_cmd.mode = SHOOT_ENABLE;
    }
    // else if( rc_data -> rc . switch_left == 3 &&  rc_data -> rc . switch_right == 3)
    // {
    //     shoot_cmd.mode = SHOOT_STOP;
    // }
    else 
    {
        shoot_cmd.mode = SHOOT_DISABLE;
    }

    if( shoot_cmd.mode == SHOOT_ENABLE)
    {
        Shoot_Enable();
    }
    else
    {
        Shoot_Disable();
    }
}

// float shoot_kp_test;
// float shoot_ki_test;
// float shoot_kd_test;
// float shoot_stir_speed_test;

float stir_state ;

void Shoot_Reference(void)
{
    // shoot_stir_motor->motor_controller.speed_PID->kp = shoot_kp_test;
    // shoot_stir_motor->motor_controller.speed_PID->ki = shoot_ki_test;
    // shoot_stir_motor->motor_controller.speed_PID->kd = shoot_kd_test;
    // shoot_stir_speed_test = shoot_stir_motor->receive_data.speed;

	GPIO_PinState stir_read_temp;	
    stir_read_temp = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);

    shoot_cmd.fire_arrive_last = shoot_cmd.fire_arrive_current;
    if(stir_read_temp == GPIO_PIN_SET)
    {
        shoot_cmd.fire_arrive_current = 1;
    }
    else
    {
        shoot_cmd.fire_arrive_current = 0;
    }
    
    if(shoot_cmd.mode == SHOOT_ENABLE)
    {
        if( rc_data -> rc .dial == 660) 
        // if( uart2_rx_message.dial == 660)
        {
            if(shoot_cmd.fire_single == 1)
            {
                shoot_cmd.v_shoot_stir = shoot_stir_tar ;
            }
            shoot_cmd.fire_single = 0;
        }
        else if( rc_data -> rc .dial == 0)
        // else if( uart2_rx_message.dial == 0)
        {
            shoot_cmd.fire_single = 1;
            shoot_cmd.v_shoot_stir = 0;
        }

        if(shoot_cmd.fire_arrive_current == 1 && shoot_cmd.fire_arrive_last == 0)
        {
            shoot_cmd.v_shoot_stir = 0 ;
        }

        //shoot pid test

        // shoot_cmd.v_shoot_stir = shoot_stir_tar ;

    }
    else if( shoot_cmd.mode == SHOOT_DISABLE)
    {
        shoot_cmd.v_shoot_stir = 0;
    }

}

void Shoot_Console(void)
{
    DJI_Motor_Set_Ref(shoot_stir_motor, shoot_cmd.v_shoot_stir);
}

void Shoot_Send_Cmd(void)
{
    DJI_Motor_Control(shoot_stir_motor);
}

static void Shoot_Enable(void)
{
    DJI_Motor_Enable(shoot_stir_motor);
}

static void Shoot_Disable(void)
{   
    DJI_Motor_Disable(shoot_stir_motor);
}