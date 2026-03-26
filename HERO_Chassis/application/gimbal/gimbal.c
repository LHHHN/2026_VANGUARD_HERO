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
#include "user_lib.h"

#include "pid.h"
#include "rs485.h"
#include "INS.h"

#include "kalman_one_filter.h"

LK_motor_instance_t *gimbal_MF9025_motor;
gimbal_cmd_t gimbal_cmd;

extern INS_behaviour_t INS;

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

//imu参数
PID_t gimbal_angle_pid = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .kf = 0.0f,
    .output_limit = 20.0f, 
    .fout_limit = 10.0f,
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};


PID_t gimbal_speed_pid = {
    .kp = 750.0f,
    .ki = 2.0f,
    .kd = 1500.0f,
	.kf = 10.0f,
    .output_limit = 2048.0f, 
    .fout_limit = 1000.0f,
    .integral_limit = 100000.0f,
    .dead_band = 0.0f,
};

//电机反馈参数
// PID_t gimbal_angle_pid = {
//     .kp = 100.0f,
//     .ki = 0.0f,
//     .kd = 20.0f,
//     .output_limit = 100.0f, 
//     .integral_limit = 10.0f,
//     .dead_band = 0.0f,
// };

// PID_t gimbal_speed_pid = {
//     .kp = 200.0f,
//     .ki = 0.5f,
//     .kd = 0.0f,
// 	  .kf = 100.0f,
//     .output_limit = 2048.0f, 
//     .integral_limit = 100000.0f,
//     .dead_band = 0.0f,
// };

motor_init_config_t gimbal_motor_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_angle_pid,
        .speed_PID = &gimbal_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &(uart2_rx_message.angle_yaw),
        .other_speed_feedback_ptr = &(uart2_rx_message.speed_yaw),
        // .other_angle_feedback_ptr = NULL,
        // .other_speed_feedback_ptr = NULL,

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

PID_t gimbal_follow_pid = {
    .kp = 0.005f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 0.075f, 
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};



static void Gimbal_Enable(void);
static void Gimbal_Disable(void);
static void Gimbal_Stop(void);

// float target_yaw = 2.166f;
float target_yaw = 0.0f;

#include "kalman_filter.h"
#include "bsp_dwt.h"

// KalmanFilter_t vaEstimateKF;

// static float vaEstimateKF_F[4] = {
// 	1.0f, 0.001f,
// 	0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

// static float vaEstimateKF_P[4] = {
// 	1.0f, 0.0f,
// 	0.0f, 1.0f}; // 后验估计协方差初始值

// static float vaEstimateKF_Q[4] = {
// 	0.05f, 0.0f,
// 	0.0f, 0.01f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

// static float vaEstimateKF_R[4] = {
// 	1200.0f, 0.0f,
// 	0.0f, 2000.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

// static const float vaEstimateKF_H[4] = {
// 	1.0f, 0.0f,
// 	0.0f, 1.0f}; // 设置矩阵H为常量

// static float vaEstimateKF_K[4];

// static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
// {
// 	Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

// 	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
// 	memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
// 	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
// 	memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
// 	memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
// }

kalman_one_filter_t mess_kf; //yaw的卡尔曼滤波

void Messkf_Init(void)
{
	Kalman_One_Init(&mess_kf, 0.01f, 1.3f);
}

//yaw斜坡函数
ramp_function_source_t *yaw_speed_ramp; 

ramp_init_config_t yaw_speed_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.0f,            // 最大输出
    .min_value = 0.0f,             // 最小输出
    .increase_value = 0.0005f,        // 加速度
    .decrease_value = 0.0005f,     // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};

void Gimbal_Init(void)
{
    gimbal_MF9025_motor = LK_Motor_Init(&gimbal_motor_init);
    gimbal_MF9025_motor->lk_ctrl_mode = TORQUE_MODE;
    // gimbal_MF9025_motor->motor_feedback = LK_MOTOR_DIFF;
    gimbal_MF9025_motor->motor_feedback = LK_MOTOR_ABSOLUTE;
    // xvEstimateKF_Init(&vaEstimateKF);
    Gimbal_Enable();
    LK_Motor_GetData(gimbal_MF9025_motor);

    Messkf_Init();
    yaw_speed_ramp = ramp_init(&yaw_speed_ramp_init);
}

extern RC_ctrl_t *rc_data;

#include "omni_mecanum_chassis.h"
void Gimbal_Set_Mode()
{
    // if( rc_data -> rc . switch_left == 3 || (rc_data -> rc . switch_left == 2 && rc_data -> rc . switch_right == 2 ))
    // // if(uart2_rx_message.switch_right == 3 )
    // {
    //     gimbal_cmd.mode = GIMBAL_ENABLE;
    // }
    // else if( rc_data -> rc . switch_left == 1 && (rc_data -> rc . switch_right == 2 || rc_data -> rc . switch_right == 3))
    // {
    //     gimbal_cmd.mode = GIMBAL_AUTO_AIMING;
    // }
    // else if( rc_data -> rc . switch_left == 1 &&  rc_data -> rc . switch_right == 1)
    // // else if( uart2_rx_message.switch_left == 1 &&  uart2_rx_message.switch_right == 1)
    // {
    //     gimbal_cmd.mode = GIMBAL_DISABLE;
    // }
    // else if( rc_data -> rc . switch_left == 2 &&  (rc_data -> rc . switch_right == 1 || rc_data -> rc . switch_right == 3))
    // {
    //     gimbal_cmd.mode = GIMBAL_ZERO;
    // }
    // else
    // {
    //     gimbal_cmd.mode = GIMBAL_STOP;
    // }


    if(rc_data -> online == 0)
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
                // gimbal_cmd.mode = GIMBAL_ENABLE;
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
            if (rc_data->mouse.press_r == 1)
            {
                gimbal_cmd.mode = GIMBAL_AUTO_AIMING;
            }
            else if(rc_data->mouse.press_r == 0 && gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
            {
                gimbal_cmd.mode = GIMBAL_ENABLE;
            }
            
            if(rc_data->key->q == 1)
            {
                gimbal_cmd.mode = GIMBAL_ZERO;
            }
            else if(rc_data->key->q ==0 && gimbal_cmd.mode == GIMBAL_ZERO)
            {
                gimbal_cmd.mode = GIMBAL_ENABLE;
            }
        }
        
    }

    if( gimbal_cmd.mode == GIMBAL_ENABLE && gimbal_MF9025_motor->motor_state_flag != MOTOR_ENABLE)
    // if( gimbal_cmd.mode == GIMBAL_ENABLE )
    {
        target_yaw = uart2_rx_message.angle_yaw;
        // target_yaw = gimbal_MF9025_motor->receive_data.RAD_single_round;
        Gimbal_Enable();
    }
    else if( gimbal_cmd.mode == GIMBAL_AUTO_AIMING && gimbal_MF9025_motor->motor_state_flag != MOTOR_ENABLE)
    // else if( gimbal_cmd.mode == GIMBAL_STOP )
    {
        target_yaw = uart2_rx_message.angle_yaw;
        // target_yaw = gimbal_MF9025_motor->receive_data.RAD_single_round;
        Gimbal_Enable();
    }
//    else if( gimbal_cmd.mode == GIMBAL_STOP && gimbal_MF9025_motor->motor_state_flag != MOTOR_ENABLE)
//    // else if( gimbal_cmd.mode == GIMBAL_STOP )
//    {
//        // target_yaw = gimbal_MF9025_motor->receive_data.RAD_single_round;
//        Gimbal_Stop();
//    }
    else if ( gimbal_cmd.mode == GIMBAL_DISABLE && gimbal_MF9025_motor->motor_state_flag != MOTOR_DISABLE)
    // else if ( gimbal_cmd.mode == GIMBAL_DISABLE )
    {
        // target_yaw = gimbal_MF9025_motor->receive_data.RAD_single_round;
        Gimbal_Disable();
    }
}

// static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel, float fusion_v_data[])
// {
// 	// 卡尔曼滤波器测量值更新
// 	EstimateKF->MeasuredVector[0] = vel; // 测量速度
// 	EstimateKF->MeasuredVector[1] = acc; // 测量加速度

// 	// 卡尔曼滤波器更新函数
// 	Kalman_Filter_Update(EstimateKF);

// 	// 提取估计值
// 	for (uint8_t i = 0 ; i < 2 ; i++)
// 	{
// 		fusion_v_data[i] = EstimateKF->FilteredValue[i];
// 	}
// }

// #define MAX_LMS_FILTER_NUM 20

// float w_ob = 0.0f;
// float d_gyro = 0.0F;
// float new_x = 0.0f;
// float INS_yaw_speed = 0.0f;

// float Q_1 = 0.2f, Q_3 = 0.01f, R_1 = 100.0f, R_3 = 3000.0f;

// float fusion_v_data[2];

// uint8_t test_cnt = 1;

// static void Gimbal_Speed_Update(void)
// {


//     static float x[MAX_LMS_FILTER_NUM] = {0.0f}; // 输入队列
// 	static uint32_t observe_dwt = 0;
// 	static float dt             = 0.0F;

//     new_x = gimbal_MF9025_motor->receive_data.speed_rps; // 转速转换为线速度，假设轮子半径为0.033m
//     INS_yaw_speed = uart2_rx_message.speed_yaw;

//     memmove(&x[0], &x[1], (MAX_LMS_FILTER_NUM - 1) * sizeof(float));
// 	x[MAX_LMS_FILTER_NUM - 1] = (float) INS_yaw_speed;

// 	dt = DWT_GetDeltaT(&observe_dwt);

//     d_gyro = ((x[test_cnt] - x[0]) * 0.8f) / (dt * 10) + d_gyro * 0.2f; // 计算差值

// 	vaEstimateKF_F[1] = dt;

//     vaEstimateKF_Q[0] = Q_1;
// 	vaEstimateKF_Q[3] = Q_3;
// 	vaEstimateKF_R[0] = R_1;
// 	vaEstimateKF_R[3] = R_3;

// 	memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
// 	memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));

// 	xvEstimateKF_Update(&vaEstimateKF, d_gyro, gimbal_MF9025_motor->receive_data.speed_rps, fusion_v_data);

// 	w_ob = fusion_v_data[0];
// }

void Gimbal_Observer(void)
{
    // Gimbal_Speed_Update();
}

//云台pid调整临时变量
float gimbal_speed_kp_test;
float gimbal_speed_ki_test;
float gimbal_speed_kd_test;
float gimbal_speed_kf_test;
float gimbal_speed_test;
float gimbal_speed_motor_test;

float gimbal_angle_kp_test;
float gimbal_angle_ki_test;
float gimbal_angle_kd_test;
float gimbal_angle_kf_test;
float gimbal_angle_test;
float gimbal_angle_motor_test;

float gimbal_angle_tar_afterkf ;

float gimbal_vs_tar;

// uint16_t sin_cnt = 0;
// float frq = 0.0f;
// float val = 0.0f;

void Gimbal_Reference( )
{
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
    // gimbal_MF9025_motor->motor_controller.speed_PID->kf = gimbal_speed_kf_test;
    gimbal_speed_test = uart2_rx_message.speed_yaw;
    gimbal_speed_motor_test = gimbal_MF9025_motor->receive_data.speed_rps;

    // gimbal_MF9025_motor->motor_controller.angle_PID->kp = gimbal_angle_kp_test;
    // gimbal_MF9025_motor->motor_controller.angle_PID->ki = gimbal_angle_ki_test;
    // gimbal_MF9025_motor->motor_controller.angle_PID->kd = gimbal_angle_kd_test; 
    // gimbal_MF9025_motor->motor_controller.angle_PID->kf = gimbal_angle_kf_test;
    gimbal_angle_test = uart2_rx_message.angle_yaw;
    gimbal_angle_motor_test = gimbal_MF9025_motor->receive_data.RAD_single_round;

    // gimbal_angle_tar_afterkf = Kalman_One_Filter(&mess_kf , uart2_rx_message.vs_yaw_tar) ;
    gimbal_vs_tar = uart2_rx_message.vs_yaw_tar;
    
}

float target_yaw_pro;
float target_yaw_promax ;
void Gimbal_Console( )
{

    if(gimbal_cmd.mode == GIMBAL_ENABLE)
    {
        if(gimbal_cmd.key_state.key_EN_state == 0)
        {
            gimbal_cmd.v_yaw = -(float) rc_data -> rc . rocker_r_ * REMOTE_YAW_SEN;
        }
        else if(gimbal_cmd.key_state.key_EN_state == 1)
        {
            gimbal_cmd.v_yaw = ramp_calc(yaw_speed_ramp , -(float)rc_data->mouse.x * KEY_YAW_SEN);
            yaw_speed_ramp->real_value = gimbal_cmd.v_yaw ;
        }
        
        // gimbal_cmd.v_yaw = -(float) uart2_rx_message.rocker_r_ * REMOTE_YAW_SEN;
        target_yaw += gimbal_cmd.v_yaw ;
    }
    else if(gimbal_cmd.mode == GIMBAL_AUTO_AIMING)
    {
        if(uart2_rx_message.vs_mode == 0)
        {
            // gimbal_cmd.v_yaw = -(float) rc_data -> rc . rocker_r_ * REMOTE_YAW_SEN;
            if(gimbal_cmd.key_state.key_EN_state == 0)
            {
                gimbal_cmd.v_yaw = -(float) rc_data -> rc . rocker_r_ * REMOTE_YAW_SEN;
            }
            else if(gimbal_cmd.key_state.key_EN_state == 1)
            {
                gimbal_cmd.v_yaw = ramp_calc(yaw_speed_ramp , -(float)rc_data->mouse.x * KEY_YAW_SEN);
                yaw_speed_ramp->real_value = gimbal_cmd.v_yaw ;
            }
            target_yaw += gimbal_cmd.v_yaw ;
        }
        else if(uart2_rx_message.vs_mode == 1 || uart2_rx_message.vs_mode == 2)
        {
            if(uart2_rx_message.vs_yaw_tar != 0)
            {
                gimbal_cmd.v_yaw = 0.0f;
                target_yaw = uart2_rx_message.vs_yaw_tar;
            }
            else
            {
                target_yaw = target_yaw;
            }
            
            // target_yaw = gimbal_angle_tar_afterkf;
        }
        else
        {
            gimbal_cmd.v_yaw = 0.0f;
            target_yaw =  target_yaw ;
        }
        
    }
//    else if(gimbal_cmd.mode == GIMBAL_STOP)
//    {
//        gimbal_cmd.v_yaw = 0.0f;
//        target_yaw =  target_yaw;
//    }
    else if(gimbal_cmd.mode == GIMBAL_ZERO)
    {        
        gimbal_cmd.v_yaw = PID_Position(&gimbal_follow_pid, gimbal_MF9025_motor->receive_data.RAD_single_round, 3.67f);
        target_yaw += gimbal_cmd.v_yaw ;
    }
    else
    {
        gimbal_cmd.v_yaw = 0.0f ;
        target_yaw = INS.Yaw ;
    }

    // while(target_yaw - uart2_rx_message.angle_yaw> PI)
    //     target_yaw -= 2 * PI ;
    // while(target_yaw - uart2_rx_message.angle_yaw < -PI)
    //     target_yaw += 2 * PI ;

    target_yaw_pro = Kalman_One_Filter(&mess_kf , target_yaw) ;
    while(target_yaw_pro - uart2_rx_message.angle_yaw > PI)
        target_yaw_pro -= 2 * PI ;
    while(target_yaw_pro - uart2_rx_message.angle_yaw < -PI)
        target_yaw_pro += 2 * PI ;

    // while(target_yaw_pro - gimbal_MF9025_motor->receive_data.RAD_single_round> PI)
    //     target_yaw_pro -= 2 * PI ;
    // while(target_yaw_pro - gimbal_MF9025_motor->receive_data.RAD_single_round < -PI)
    //     target_yaw_pro += 2 * PI ;
    LK_Motor_SetTar(gimbal_MF9025_motor, target_yaw_pro);   
//    gimbal_MF9025_motor->receive_data.lk_diff = - gimbal_cmd.v_yaw;
}


void Gimbal_Send_Cmd()
{
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
