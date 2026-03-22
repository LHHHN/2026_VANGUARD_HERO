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

float shoot_stir_tar = -14400; //固定目标转速

PID_t shoot_stir_angle_pid = {
    .kp = 14.0f, //5.0f,
    .ki = 0.0f,
    .kd = 0.1f, // 0.01f,
    .output_limit = 10800.0f, 
    .integral_limit = 3600.0f,
    .dead_band = 0.0f,
};

PID_t shoot_stir_speed_pid = {
    .kp = 0.5f, //1.0f,
    .ki = 0.01f, //0.5f,
    .kd = 0.1f,
    .kf = 0.1f,
    .output_limit = 15000.0f, 
    .integral_limit = 12000.0f,
    .dead_band = 0.0f,
};

// PID_t shoot_stir_speed_pid = {
//     .kp = 1.0f, //1.0f,
//     .ki = 0.15f, //0.5f,
//     .kd = 0.1f,
//     .output_limit = 15000.0f, 
//     .integral_limit = 7500.0f,
//     .dead_band = 0.0f,
// };

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
    if( rc_data -> rc . switch_left == 3 && (rc_data -> rc . switch_right == 3 || rc_data -> rc . switch_right == 2))
    // if( uart2_rx_message.switch_left == 2 &&  uart2_rx_message.switch_right == 3)
    {
        shoot_cmd.mode = SHOOT_ENABLE;
    }
    else if( rc_data -> rc . switch_left == 1 && (rc_data -> rc . switch_right == 3 || rc_data -> rc . switch_right == 2))
    {
        shoot_cmd.mode = SHOOT_AUTO_AIMING;
    }
    else 
    {
        shoot_cmd.mode = SHOOT_DISABLE;
    }

    if( shoot_cmd.mode == SHOOT_ENABLE || shoot_cmd.mode == SHOOT_AUTO_AIMING)
    {
        Shoot_Enable();
    }
    else
    {
        Shoot_Disable();
    }
}

float speed_kp_test = 0.45f;
float speed_ki_test = 0.02f;
float speed_kd_test = 0.05;
float speed_kf_test = 0.25f;
float speed_output_limit_test = 20000.0f;
float speed_integral_limit_test = 15000.0f;

float shoot_stir_speed_test;
float shoot_stir_angle_test;
float shoot_stir_current_test;

float shoot_stir_angle_tar;
float shoot_stir_speed_tar;
uint8_t shoot_stir_read_state;
uint8_t shoot_stir_read_state_last;

float stir_state ;

float stir_buchang = 240.0f; //发射补偿角度,可以根据实际情况调整

float vs_shoot_cnt = 1.0f;
// float vs_shoot_later = 0.0f;

void Shoot_Reference(void)
{
    static uint8_t shoot_angle_limit_flag = 0;

    shoot_stir_motor->motor_controller.speed_PID->kp = speed_kp_test;
    shoot_stir_motor->motor_controller.speed_PID->ki = speed_ki_test;
    shoot_stir_motor->motor_controller.speed_PID->kd = speed_kd_test;
    shoot_stir_motor->motor_controller.speed_PID->kf = speed_kf_test;
    shoot_stir_motor->motor_controller.speed_PID->output_limit = speed_output_limit_test ;
    shoot_stir_motor->motor_controller.speed_PID->integral_limit = speed_integral_limit_test;

    // shoot_stir_motor->motor_controller.angle_PID->kp = angle_kp_test;
    // shoot_stir_motor->motor_controller.angle_PID->ki = angle_ki_test;
    // shoot_stir_motor->motor_controller.angle_PID->kd = angle_kd_test;
    // shoot_stir_motor->motor_controller.angle_PID->output_limit = angle_output_limit_test ;

	GPIO_PinState stir_read_temp;	
    stir_read_temp = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);

    shoot_cmd.fire_arrive_last = shoot_cmd.fire_arrive_current;

    if(stir_read_temp == GPIO_PIN_SET) {
        shoot_cmd.fire_arrive_current = 1;
    } else {
        shoot_cmd.fire_arrive_current = 0;
    }

    // if(shoot_cmd.fire_arrive_last)
    // {
    //     if(!shoot_cmd.fire_arrive_current) 
    //     {
    //         shoot_cmd.angle_shoot_stir = shoot_stir_motor->receive_data.total_angle - stir_buchang; // 5度的补偿,可以根据实际情况调整

    //         // shoot_cmd.angle_shoot_stir = 0.0f;
    //         // shoot_stir_motor->receive_data.total_angle = 0.0f;
    //     }
    // }

    static float shoot_angle_last = 0.0f;
    static float shoot_angle_now = 0.0f;
    
    if(shoot_cmd.mode == SHOOT_ENABLE)
    {
        if(uart2_rx_message.shoot_launched == 0)
        {
            shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
            if( rc_data -> rc .dial == 660) 
            // if( uart2_rx_message.dial == 660)
            {
                if(shoot_cmd.fire_single == 1)
                {
                    if(shoot_angle_limit_flag == 0)
                    {
                        shoot_angle_last = shoot_stir_motor->receive_data.total_angle;
                        shoot_angle_limit_flag = 1;
                    }
                    shoot_cmd.v_shoot_stir = shoot_stir_tar ;

                    // shoot_cmd.angle_shoot_stir -= SHOOT_SINGLE_ANGLE_DEF * 19.8f;
                }
                shoot_cmd.fire_single = 0;
            }
            else if( rc_data -> rc .dial == 0)
            // else if( uart2_rx_message.dial == 0)
            {
                shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
                if(shoot_angle_limit_flag == 1)
                {
                    shoot_angle_limit_flag = 0;
                }
                shoot_cmd.fire_single = 1;

                shoot_cmd.v_shoot_stir = 0;

                // shoot_cmd.angle_shoot_stir = shoot_stir_motor->receive_data.total_angle; 
            }

        }
        
        // if(shoot_cmd.fire_arrive_current == 1 && shoot_cmd.fire_arrive_last == 0)
        // {
        //     shoot_cmd.v_shoot_stir = 0 ;

        //     // shoot_cmd.angle_shoot_stir = 0 ;
        //     // shoot_stir_motor->receive_data.total_angle = 0; 
        // }

        if(uart2_rx_message.shoot_launched == 1)
        {
            shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
            shoot_cmd.v_shoot_stir = 0 ;
        }

        if(((shoot_angle_now - shoot_angle_last) < -63.0f * 19.8f))
        {
            shoot_cmd.v_shoot_stir = 0;
        }

    }
    else if( shoot_cmd.mode == SHOOT_AUTO_AIMING)
    {
        if(uart2_rx_message.vs_mode == 2 && rc_data->rc.dial == 660)
        {				
            if(vs_shoot_cnt <= 0)
            {
								shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
                vs_shoot_cnt = 0.0f;
            }
            else
            {
                shoot_angle_limit_flag = 0;
                vs_shoot_cnt --;
            }
            if(uart2_rx_message.shoot_launched == 0 && vs_shoot_cnt == 0.0f)
            {
                if(shoot_angle_limit_flag == 0)
                {
                    shoot_angle_last = shoot_stir_motor->receive_data.total_angle;
                    shoot_angle_limit_flag = 1;
                }
								
                shoot_cmd.v_shoot_stir = shoot_stir_tar ;
            }
            
            if(uart2_rx_message.shoot_launched == 1)
            {
                shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
                shoot_cmd.v_shoot_stir = 0 ;
                if(vs_shoot_cnt == 0.0f)
                {
                    vs_shoot_cnt = 1000.0f;
                }
            }

            if(((shoot_angle_now - shoot_angle_last) < -63.0f * 19.8f))
            {
                shoot_cmd.v_shoot_stir = 0;
                if(vs_shoot_cnt == 0.0f)
                {
                    vs_shoot_cnt = 1000.0f;
                }
            }
        }
        else
        {
            shoot_cmd.v_shoot_stir = 0;
        }
    }
    else if( shoot_cmd.mode == SHOOT_DISABLE)
    {
        shoot_cmd.v_shoot_stir = 0;

        // shoot_cmd.angle_shoot_stir = shoot_stir_motor->receive_data.total_angle;

        // shoot_cmd.angle_shoot_stir = 0;
        // shoot_stir_motor->receive_data.total_angle = 0;
    }

    if(vs_shoot_cnt <= 0)
    {
        shoot_angle_now = shoot_stir_motor->receive_data.total_angle;
        vs_shoot_cnt = 0.0f;
    }
    else
    {
        shoot_angle_limit_flag = 0;
        vs_shoot_cnt --;
    }

    
    shoot_stir_current_test = shoot_stir_motor->receive_data.real_current;

    shoot_stir_speed_test = shoot_stir_motor->receive_data.speed_aps;
    shoot_stir_angle_test = shoot_stir_motor->receive_data.total_angle;
    shoot_stir_angle_tar = shoot_cmd.angle_shoot_stir;
    shoot_stir_speed_tar = shoot_cmd.v_shoot_stir;
    shoot_stir_read_state = shoot_cmd.fire_arrive_current;
    shoot_stir_read_state_last = shoot_cmd.fire_arrive_last;

}

void Shoot_Console(void)
{
    DJI_Motor_Set_Ref(shoot_stir_motor, shoot_cmd.v_shoot_stir);
    // DJI_Motor_Set_Ref(shoot_stir_motor, shoot_cmd.angle_shoot_stir);
}

void Shoot_Send_Cmd(void)
{
//    DJI_Motor_Control(shoot_stir_motor);
	DJI_Motor_Control(NULL);
}

static void Shoot_Enable(void)
{
    DJI_Motor_Enable(shoot_stir_motor);
}

static void Shoot_Disable(void)
{   
    DJI_Motor_Disable(shoot_stir_motor);
}