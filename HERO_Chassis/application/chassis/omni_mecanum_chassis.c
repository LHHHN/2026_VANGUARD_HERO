/**
******************************************************************************
* @file    omni_mecanum_chassis.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include "omni_mecanum_chassis.h"

#include "INS.h"
#include "gimbal.h"

#include "remote_control.h"
#include "rs485.h"

#include "user_lib.h"
#include "pid.h"

#include "power_ctrl.h"

#include "bsp_dwt.h"

// 底板轮毂电机初始化参数

float LEG_INIT = 0.26f ;  

DJI_motor_instance_t *chassis_m3508[4];
Chassis_CmdTypedef chassis_cmd;

PID_t chassis_3508_speed_pid = {
    .kp = 50.0f,
    .ki = 5.0f,
    .kd = 0.0f,
    .kf = 20.0f,
    .output_limit = 20000.0f,
    .integral_limit = 20000.0f,
    .dead_band = 0.0f,
};

PID_t track_speed_pid = {
    .kp = 75.0f,
    .ki = 0.25f,
    .kd = 7.5f,
    .kf = 10.0f,
    .output_limit = 20000.0f,
    .integral_limit = 20000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_hold_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 4.0f,
    .output_limit = 20.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.005f,
};

motor_init_config_t chassis_3508_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
};

// 底盘轮毂电机斜坡函数
ramp_function_source_t *vx_speed_ramp;
ramp_function_source_t *vy_speed_ramp;
ramp_function_source_t *wz_speed_ramp;

ramp_init_config_t wheel_speed_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.0f,             // 最大输出
    .min_value = 0.0f,             // 最小输出
    .increase_value = 0.0025f,     // 加速度
    .decrease_value = 0.0025f,     // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};

ramp_init_config_t round_speed_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.0f,             // 最大输出
    .min_value = 0.0f,             // 最小输出
    .increase_value = 0.05f,       // 加速度
    .decrease_value = 0.05f,       // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};

// 底盘关节电机DM4340初始化参数

DM_motor_instance_t *DM_arthrosis_motor[2]; // 0为左腿id=2 方向+ ，1为右腿id=3 方向-

PID_t *DM_arthrosis_pitch_pid[2];
PID_t *DM_arthrosis_roll_pid[2];
PID_t *DM_arthrosis_length_pid[2];

motor_init_config_t DM_arthrosis_motor_init = {
    .controller_param_init_config = {
        // .angle_PID = &arthrosis_angle_pid,
        // .speed_PID = &arthrosis_speed_pid,
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
    .close_loop_type = TORQUE_LOOP,
    .outer_loop_type = TORQUE_LOOP,
    // .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
    // .outer_loop_type = OPEN_LOOP,
    // .close_loop_type = OPEN_LOOP,
    // .close_loop_type = ANGLE_LOOP,

    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

    .angle_feedback_source = MOTOR_FEED,
    .speed_feedback_source = MOTOR_FEED,

    .feedforward_flag = FEEDFORWARD_NONE,
    .control_button = TORQUE_DIRECT_CONTROL,
    // .control_button = POLYCYCLIC_LOOP_CONTROL
    },

    .motor_type = DM4340,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x12,
    },

};

// 底盘关节电机斜坡函数
ramp_function_source_t *leg_angle_ramp[2]; // 0为左腿，1为右腿

ramp_init_config_t leg_angle_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.88f,            // 最大输出
    .min_value = 0.0f,             // 最小输出
    .increase_value = 0.1f,        // 加速度
    .decrease_value = 0.0005f,     // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};

// // 履带电机DM3519初始化参数
// DM_motor_instance_t *DM_track_motor[2]; // 0为左腿id=1 方向+ ，1为右腿id=4 方向-

// motor_init_config_t DM_track_motor_init = {
//     .controller_param_init_config = {
//         .angle_PID = NULL,
//         .speed_PID = NULL,
//         .current_PID = NULL,
//         .torque_PID = NULL,

//         .other_angle_feedback_ptr = NULL,
//         .other_speed_feedback_ptr = NULL,

//         .angle_feedforward_ptr = NULL,
//         .speed_feedforward_ptr = NULL,
//         .current_feedforward_ptr = NULL,
//         .torque_feedforward_ptr = NULL,

//         .pid_ref = 0.0f,
//     },
//     .controller_setting_init_config = {
//         // .close_loop_type = TORQUE_LOOP,
//         .outer_loop_type = OPEN_LOOP,
//         .close_loop_type = OPEN_LOOP,

//         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
//         .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

//         .angle_feedback_source = MOTOR_FEED,
//         .speed_feedback_source = MOTOR_FEED,

//         .feedforward_flag = FEEDFORWARD_NONE,
//         .control_button = TORQUE_DIRECT_CONTROL,
//         // .control_button = POLYCYCLIC_LOOP_CONTROL
//     },

//     .motor_type = DM3519,

//     .can_init_config = {
//         .can_handle = &hfdcan2,
//         .tx_id = 0x01,
//         .rx_id = 0x11,
//     },

// };

DJI_motor_instance_t *DJI_track_motor[2];

motor_init_config_t DJI_track_motor_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &track_speed_pid,
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
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x03,
    },
};

// 底盘跟随pid
PID_t chasiss_follow_pid = {
    .kp = 5.0f,
    .ki = 0.00f,
    .kd = 2.0f,
    .output_limit = 7.5f,
    .integral_limit = 20.0f,
    .dead_band = 0.00f,
};

void Chassis_Init(void)
{
    // 底盘轮毂3508电机初始化
    for (int i = 0; i < 4; i++)
    {
        chassis_3508_init.can_init_config.tx_id = 0x01 + i;
        chassis_3508_init.can_init_config.rx_id = 0x01 + i;
        chassis_m3508[i] = DJI_Motor_Init(&chassis_3508_init);
        chassis_m3508[i]->motor_feedback = RAD;
    }

    vx_speed_ramp = ramp_init(&wheel_speed_ramp_init);
    vy_speed_ramp = ramp_init(&wheel_speed_ramp_init);
    wz_speed_ramp = ramp_init(&round_speed_ramp_init);

    // 关节角度环pid
    PID_t arthrosis_pitch_pid_init = {
        .kp = 260.0f, //10.0f
        .ki = 0.0f,
        .kd = 80.0f, //5.0f
        .output_limit = 20.0f,
        .integral_limit = 10.0f,
        .dead_band = 0.0f,
    };

    // 关节速度环pid
    PID_t arthrosis_roll_pid_init = {
        .kp = 50.0f, //4.0f
        .ki = 0.0f, //0.2f
        .kd = 2.0f,
        .output_limit = 40.0f,
        .integral_limit = 200.0f,
        .dead_band = 0.0f,
    };

    PID_t arthrosis_length_pid_init = {
        .kp = 80.0f, //4.0f
        .ki = 0.0f, //0.2f
        .kd = 0.05f,
        .output_limit = 40.0f,
        .integral_limit = 200.0f,
        .dead_band = 0.0f,
    };

    // 底盘关节电机DM4340初始化
    for (int i = 0; i < 2; i++)
    {
        DM_arthrosis_motor_init.can_init_config.tx_id = 0x01 + i;
        DM_arthrosis_motor_init.can_init_config.rx_id = 0x11 + i;
        DM_arthrosis_motor[i] = DM_Motor_Init(&DM_arthrosis_motor_init);
        DM_arthrosis_motor[i]->motor_feedback = DM_MOTOR_ABSOLUTE;
        DM_arthrosis_motor[i]->dm_mode = MIT_MODE;
        // DM_arthrosis_motor[i]->dm_mode = POS_MODE;
        DM_arthrosis_motor[i]->contorl_mode_state = SINGLE_TORQUE;
        DM_Motor_Enable(DM_arthrosis_motor[i]);

        leg_angle_ramp[i] = ramp_init(&leg_angle_ramp_init);
        DM_arthrosis_pitch_pid[i] = PID_Init(&arthrosis_pitch_pid_init);
        DM_arthrosis_roll_pid[i] = PID_Init(&arthrosis_roll_pid_init);
        DM_arthrosis_length_pid[i] = PID_Init(&arthrosis_length_pid_init);

        DJI_track_motor_init.can_init_config.tx_id = 0x03 + i;
        DJI_track_motor_init.can_init_config.rx_id = 0x04 + i;
        DJI_track_motor[i] = DJI_Motor_Init(&DJI_track_motor_init);
        DJI_track_motor[i]->motor_feedback = RAD;
    }

    chassis_cmd.leg_length_target = LEG_INIT;

    // 底盘关节角度斜坡函数初始化
    //  leg_angle_ramp[0] = ramp_init(&leg_angle_ramp_init);
    //  leg_angle_ramp[1] = ramp_init(&leg_angle_ramp_init);

    // 底盘履带电机DM3519初始化
    // DM_track_motor[0] = DM_Motor_Init(&DM_track_motor_init);
    // DM_track_motor_init.can_init_config.tx_id = 0x04;
    // DM_track_motor_init.can_init_config.rx_id = 0x14;
    // DM_track_motor[1] = DM_Motor_Init(&DM_track_motor_init);
    // DM_track_motor[0]->dm_mode = SPEED_MODE;
    // DM_track_motor[1]->dm_mode = SPEED_MODE;
    // DM_Motor_Enable(DM_track_motor[0]);
    // DM_Motor_Enable(DM_track_motor[1]);

    chassis_cmd.key_state.keyboard_control = 0;
    chassis_cmd.key_state.keyboard_armed = 0;
}

extern INS_behaviour_t INS;
extern RC_ctrl_t *rc_data;

extern LK_motor_instance_t *gimbal_MF9025_motor; // 用来计算底盘跟随时的角速度

static inline float Normalize_Rad(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

static void Chassis_Enable(void);
static void Leg_Start(void);
static void Leg_Stop(void);
static void Chassis_Disable(void);

float leg_cnt = 0.0f; // 离地时间计数

void Chassis_Set_Mode(void)
{
    chassis_cmd.mode = rs485_rx_message.chassis_mode;

    if (chassis_cmd.mode == CHASSIS_UPSTEP)
    {
        leg_angle_ramp[0]->decrease_value = 0.5f;
        leg_angle_ramp[1]->decrease_value = 0.5f;
    }
    else
    {
        leg_angle_ramp[0]->decrease_value = 0.0005f;
        leg_angle_ramp[1]->decrease_value = 0.0005f;
    }

    chassis_cmd.leg_state = LEG_NORMAL;

    //TODO(GUATAI):
    // // 离地检查
    // if (DM_arthrosis_motor[0]->receive_data.torque < -3.0f &&
    //     DM_arthrosis_motor[1]->receive_data.torque > 3.0f)
    // {
    //     chassis_cmd.leg_state = LEG_LIFTOFF;
    // }
    // else if (chassis_cmd.leg_state == LEG_LIFTOFF)
    // {
    //     if (DM_arthrosis_motor[0]->receive_data.torque > 10.0f &&
    //         DM_arthrosis_motor[1]->receive_data.torque < -10.0f)
    //     {
    //         if (leg_cnt > 20)
    //         {
    //             leg_cnt = 20;
    //         }
    //         else
    //         {
    //             leg_cnt++;
    //         }

    //         if (leg_cnt >= 20)
    //         {
    //             leg_cnt = 0;
    //             chassis_cmd.leg_state = LEG_NORMAL;
    //         }
    //     }
    // }
}

float leg_angle_test_L;
float leg_angle_test_R;
float leg_angle_kp = 0.0f;
float leg_angle_ki = 0.0f;
float leg_angle_kd = 0.0f;
float leg_speed_test_L;
float leg_speed_test_R;
float leg_speed_kp = 0.0f;
float leg_speed_ki = 0.0f;
float leg_speed_kd = 0.0f;
float leg_torque_test_L;
float leg_torque_test_R;
float leg_angle_tar;
float leg_output_limit_test;

float wheel_speed_kp_test;
float wheel_speed_ki_test;
float wheel_speed_kd_test;
float wheel_speed_kf_test;
float wheel_speed_outputlimit_test;

float wheel_speed_tar0;
float wheel_speed_tar1;
float wheel_speed_tar2;
float wheel_speed_tar3;
float wheel_speed_fb0;
float wheel_speed_fb1;
float wheel_speed_fb2;
float wheel_speed_fb3;

void Chassis_Observer(void)
{
    // leg_angle_test_L = DM_arthrosis_motor[0]->receive_data.position;
    // leg_angle_test_R = -DM_arthrosis_motor[1]->receive_data.position;
    // leg_speed_test_L = DM_arthrosis_motor[0]->receive_data.velocity;
    // leg_speed_test_R = -DM_arthrosis_motor[1]->receive_data.velocity; 
    // leg_torque_test_L = DM_arthrosis_motor[0]->receive_data.torque;
    // leg_torque_test_R = -DM_arthrosis_motor[1]->receive_data.torque;
    // leg_angle_tar = chassis_cmd.leg_angle;

    // leg_speed_test_L = DJI_track_motor[0]->receive_data.speed / 60 * 2 * PI;
    // leg_speed_test_R = -DJI_track_motor[1]->receive_data.speed / 60 * 2 * PI;

    // for(int i = 0; i < 2; i++)
    // {
    //     DM_arthrosis_motor[i]->motor_controller.angle_PID->kp = leg_angle_kp;
    //     DM_arthrosis_motor[i]->motor_controller.angle_PID->ki = leg_angle_ki;
    //     DM_arthrosis_motor[i]->motor_controller.angle_PID->kd = leg_angle_kd;
    //     DM_arthrosis_motor[i]->motor_controller.speed_PID->kp = leg_speed_kp;
    //     DM_arthrosis_motor[i]->motor_controller.speed_PID->ki = leg_speed_ki;
    //     DM_arthrosis_motor[i]->motor_controller.speed_PID->kd = leg_speed_kd;
    //     DM_arthrosis_motor[i]->motor_controller.speed_PID->output_limit = leg_output_limit_test ;
    // }

    //    for(int i = 0; i < 4; i++)
    //    {
    //        chassis_m3508[i]->motor_controller.speed_PID->kp = wheel_speed_kp_test;
    //        chassis_m3508[i]->motor_controller.speed_PID->ki = wheel_speed_ki_test;
    //        chassis_m3508[i]->motor_controller.speed_PID->kd = wheel_speed_kd_test;
    //        chassis_m3508[i]->motor_controller.speed_PID->kf = wheel_speed_kf_test;
    //        chassis_m3508[i]->motor_controller.speed_PID->output_limit = wheel_speed_outputlimit_test ;
    //    }

    // for(int i = 0; i < 2; i++)
    // {
    //        DJI_track_motor[i]->motor_controller.speed_PID->kp = wheel_speed_kp_test;
    //        DJI_track_motor[i]->motor_controller.speed_PID->ki = wheel_speed_ki_test;
    //        DJI_track_motor[i]->motor_controller.speed_PID->kd = wheel_speed_kd_test;
    //        DJI_track_motor[i]->motor_controller.speed_PID->kf = wheel_speed_kf_test;
    //        DJI_track_motor[i]->motor_controller.speed_PID->output_limit = wheel_speed_outputlimit_test ;
    // }

    //    wheel_speed_tar0 = chassis_cmd.wheel_target[0];
    //    wheel_speed_tar1 = chassis_cmd.wheel_target[1];
    //    wheel_speed_tar2 = chassis_cmd.wheel_target[2];
    //    wheel_speed_tar3 = chassis_cmd.wheel_target[3];
    //    wheel_speed_fb0 = DJI_Motor_GetVal(chassis_m3508[0], MOTOR_SPEED, RAD);
    //    wheel_speed_fb1 = DJI_Motor_GetVal(chassis_m3508[1], MOTOR_SPEED, RAD);
    //    wheel_speed_fb2 = DJI_Motor_GetVal(chassis_m3508[2], MOTOR_SPEED, RAD);
    //    wheel_speed_fb3 = DJI_Motor_GetVal(chassis_m3508[3], MOTOR_SPEED, RAD);
}

static float Rabbit_Leg_Length(float theta)
{
    static float L1 = 0.2062f;
    static float L2 = 0.2049f;
    static float L3 = 0.195f;
    static float L4 = 0.0987f;
    static float L23 = 0.0525f;

    float angBAD = PI / 4.0f + theta;
    float BD = sqrtf(L2 * L2 + L4 * L4 - 2.0 * L2 * L4 * cosf(angBAD));
    float angADC = acosf((BD * BD + L2 * L2 - L4 * L4) / (2.0 * BD * L2)) + 
                   acosf((BD * BD + L23 * L23 - L3 * L3) / (2.0 * BD * L23));
    float beta = PI - angADC - theta;

    float leg_length = -1.0f * L2 * sin(theta) - 1.0f * L1 * sin(beta);

    if(leg_length > 0.01f)
    {
        leg_length = 0.01f;
    }

    return leg_length;
}

float leg_test = 0.0f;
float kp_leg_test0 = 360.0f;
float kp_leg_test1 = 360.0f;
float kd_leg_test = 100.0f;
float kp_roll_test = 180.0f;
float kd_roll_test = 100.0f;
float kp_length_test = 120.0f;
float kd_length_test = 10.0f;

float leg_left_length = 0.0f;
float leg_right_length = 0.0f;

float pitch_left_leg_forward_torque;
float pitch_right_leg_forward_torque;

float forward_diff_torque = 0.0f;
float forward_diff_pitch = 0.05f ;
// 更新目标值
void Chassis_Reference(void)
{
    static float chassis_yaw_target = 0.0f;
    static uint8_t chassis_hold_flag = 0;

    static float forward_pitch_torque[2] = {0.0f, 0.0f};
    static float forward_roll_torque[2] = {0.0f, 0.0f};

    if (chassis_cmd.mode != CHASSIS_DISABLE)
    {
        Chassis_Enable();
        Leg_Start();
    }
    else
    {
        Leg_Stop();
        Chassis_Disable();
    }

    if (chassis_cmd.mode == CHASSIS_SPIN)
    {
        chassis_cmd.omega_follow = 0;
        if (rs485_rx_message.control_remote_flag == 0)
        {
            chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
            chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
            chassis_cmd.omega_z = rs485_rx_message.chassis_target_wz;
            ramp_clear(vx_speed_ramp);
            ramp_clear(vy_speed_ramp);
            ramp_clear(wz_speed_ramp);
        }
        else if (rs485_rx_message.control_remote_flag == 1)
        {
            chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
            chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
            chassis_cmd.omega_z = rs485_rx_message.chassis_target_wz;
        }
        chassis_cmd.v_track = 0.0f;
    }
    else if (chassis_cmd.mode == CHASSIS_FOLLOW)
    {
        if (rs485_rx_message.control_remote_flag == 0)
        {
            chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
            chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
            // chassis_cmd.omega_z = rs485_rx_message.chassis_target_wz;
            ramp_clear(vx_speed_ramp);
            ramp_clear(vy_speed_ramp);
            ramp_clear(wz_speed_ramp);
        }
        else if (rs485_rx_message.control_remote_flag == 1)
        {
             chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
             chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
        }
        chassis_cmd.omega_z = 0.0f;
        chassis_cmd.v_track = 0.0f;
        //TODO
        chassis_cmd.omega_follow = 0.0f;
        chassis_cmd.omega_follow = PID_Position(&chasiss_follow_pid, gimbal_MF9025_motor->receive_data.RAD_single_round, FOLLOW_OMEGA_Z);
    }
    else if (chassis_cmd.mode == CHASSIS_UPSTEP)
    {
        if (rs485_rx_message.control_remote_flag == 0)
        {
            chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
            chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
            chassis_cmd.omega_z = rs485_rx_message.chassis_target_wz;
            ramp_clear(vx_speed_ramp);
            ramp_clear(vy_speed_ramp);
            ramp_clear(wz_speed_ramp);
        }
        else if (rs485_rx_message.control_remote_flag == 1)
        {
            chassis_cmd.vx = rs485_rx_message.chassis_target_vx;
            chassis_cmd.vy = rs485_rx_message.chassis_target_vy;
            chassis_cmd.omega_z = rs485_rx_message.chassis_target_wz;
        }
        chassis_cmd.omega_follow = 0.0f;
        // if (rs485_rx_message.chassis_target_leg_angle >= 0 && chassis_cmd.leg_state == LEG_NORMAL)
        // {
        //     chassis_cmd.leg_angle = rs485_rx_message.chassis_target_leg_angle; // 660换算成弧度0-0.88rad
        // }
        chassis_cmd.v_track = 100.0f;
    }
    else if (chassis_cmd.mode == CHASSIS_STOP)
    {
        chassis_cmd.omega_z = 0.0f;
        chassis_cmd.omega_follow = 0.0f;
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        // chassis_cmd.leg_angle = 0.0f;
    }
    if (rs485_rx_message.control_remote_flag == 1)
    {
        chassis_cmd.vx_real = ramp_calc(vx_speed_ramp, chassis_cmd.vx);
        chassis_cmd.vy_real = ramp_calc(vy_speed_ramp, chassis_cmd.vy);
        chassis_cmd.omega_follow = ramp_calc(wz_speed_ramp, (float)chassis_cmd.omega_follow);
        vx_speed_ramp->real_value = chassis_cmd.vx_real;
        vy_speed_ramp->real_value = chassis_cmd.vy_real;
        wz_speed_ramp->real_value = chassis_cmd.omega_follow;
    }
    else
    {
        chassis_cmd.vx_real = chassis_cmd.vx; 
        chassis_cmd.vy_real = chassis_cmd.vy;
    }      

    DM_arthrosis_pitch_pid[0]->kp = kp_leg_test0;
    DM_arthrosis_pitch_pid[1]->kp = kp_leg_test1;
    DM_arthrosis_pitch_pid[0]->kd = kd_leg_test;
    DM_arthrosis_pitch_pid[1]->kd = kd_leg_test;
    DM_arthrosis_roll_pid[0]->kp = kp_roll_test;
    DM_arthrosis_roll_pid[1]->kp = kp_roll_test;
    DM_arthrosis_roll_pid[0]->kd = kd_roll_test;
    DM_arthrosis_roll_pid[1]->kd = kd_roll_test;
    DM_arthrosis_length_pid[0]->kp = kp_length_test;
    DM_arthrosis_length_pid[1]->kd = kd_length_test;

    if(rs485_rx_message.recovery_leg_flag == 1)
    {
        chassis_cmd.leg_length_target = - LEG_INIT ; //收腿
    }
    else
    {
        chassis_cmd.leg_length_target = LEG_INIT;
    }
    
    // if(chassis_cmd.mode == CHASSIS_UPSTEP)
    // {
    //     chassis_cmd.leg_length_target = LEG_INIT;
    // }
    // else
    // {
    //     float change_length = 0.0f;
    //     float angle_delta = 0.0f;
    //     angle_delta = -INS.Pitch;
    //     if(user_abs(angle_delta) < 0.0125f)
    //     {
    //         angle_delta = 0.0f;
    //     }
    //     change_length = float_constrain(angle_delta,-0.0005f,0.0005f);
    //     chassis_cmd.leg_length_target += change_length;
    //     chassis_cmd.leg_length_target = float_constrain(chassis_cmd.leg_length_target,0.05f,0.25f);
    // }

    leg_left_length = -Rabbit_Leg_Length(DM_arthrosis_motor[0]->receive_data.position);
    leg_right_length = -Rabbit_Leg_Length(-DM_arthrosis_motor[1]->receive_data.position);

    DM_arthrosis_motor[0]->motor_controller.pid_ref = DM_arthrosis_length_pid[0]->kp * (chassis_cmd.leg_length_target - leg_left_length) - DM_arthrosis_length_pid[0]->kd * DM_arthrosis_motor[0]->receive_data.velocity;
    DM_arthrosis_motor[1]->motor_controller.pid_ref = -(DM_arthrosis_length_pid[1]->kp * (chassis_cmd.leg_length_target - leg_right_length) + DM_arthrosis_length_pid[1]->kd * DM_arthrosis_motor[1]->receive_data.velocity);

    forward_pitch_torque[0] = DM_arthrosis_pitch_pid[0]->kp * (0.0f - INS.Pitch) - DM_arthrosis_pitch_pid[0]->kd * INS.Gyro[0];
    forward_pitch_torque[1] = -(DM_arthrosis_pitch_pid[1]->kp * (0.0f - INS.Pitch) - DM_arthrosis_pitch_pid[1]->kd * INS.Gyro[0]);

    forward_pitch_torque[0] = float_constrain(forward_pitch_torque[0], -15.0f, 15.0f);
    forward_pitch_torque[1] = float_constrain(forward_pitch_torque[1], -15.0f, 15.0f);

    forward_roll_torque[0] = DM_arthrosis_roll_pid[0]->kp * (-forward_diff_pitch + INS.Roll) + DM_arthrosis_roll_pid[0]->kd * INS.Gyro[1];
    forward_roll_torque[1] = (DM_arthrosis_roll_pid[1]->kp * (-forward_diff_pitch + INS.Roll) + DM_arthrosis_roll_pid[1]->kd * INS.Gyro[1]);

    forward_roll_torque[0] = float_constrain(forward_roll_torque[0], -15.0f, 15.0f);
    forward_roll_torque[1] = float_constrain(forward_roll_torque[1], -15.0f, 15.0f);

    pitch_left_leg_forward_torque = 16.0f + forward_pitch_torque[0] + forward_roll_torque[0];
    pitch_right_leg_forward_torque = -(16.0f) + forward_pitch_torque[1] + forward_roll_torque[1];

    DM_arthrosis_motor[0]->motor_controller.pid_ref += pitch_left_leg_forward_torque;
    DM_arthrosis_motor[1]->motor_controller.pid_ref += pitch_right_leg_forward_torque;

    // forward_diff_torque = user_abs(DM_arthrosis_motor[0]->motor_controller.pid_ref + DM_arthrosis_motor[1]->motor_controller.pid_ref);

    // if(forward_diff_torque> 5.0f)
    // {
    //     forward_diff_torque = DM_arthrosis_motor[0]->motor_controller.pid_ref + DM_arthrosis_motor[1]->motor_controller.pid_ref;
    //     forward_diff_torque = float_constrain(forward_diff_torque, -20.0f, 20.0f);
    //     DM_arthrosis_motor[0]->motor_controller.pid_ref -= forward_diff_torque;
    //     DM_arthrosis_motor[1]->motor_controller.pid_ref -= forward_diff_torque;
    // }

    // if(chassis_cmd.mode == CHASSIS_UPSTEP)
    // {   
    //     if(leg_left_length < 0.18f)
    //     {
    //         DM_arthrosis_motor[0]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[0]->motor_controller.pid_ref, 25.0f, 55.0f);
    //     }
    //     else
    //     {
    //         DM_arthrosis_motor[0]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[0]->motor_controller.pid_ref, -10.0f, 55.0f);
    //     }

    //     if(leg_left_length < 0.18f)
    //     {
    //         DM_arthrosis_motor[1]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[1]->motor_controller.pid_ref, -55.0f, -25.0f);
    //     }
    //     else
    //     {
    //         DM_arthrosis_motor[1]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[1]->motor_controller.pid_ref, -55.0f, 10.0f);
    //     }
    // }
    // else
    // {
    //     DM_arthrosis_motor[0]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[0]->motor_controller.pid_ref, -10.0f, 55.0f);
    //     DM_arthrosis_motor[1]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[1]->motor_controller.pid_ref, -55.0f, 10.0f);;
    // }

    DM_arthrosis_motor[0]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[0]->motor_controller.pid_ref, -10.0f, 55.0f);
    DM_arthrosis_motor[1]->motor_controller.pid_ref = float_constrain(DM_arthrosis_motor[1]->motor_controller.pid_ref, -55.0f, 10.0f);

    // if (chassis_cmd.leg_state == LEG_NORMAL && chassis_cmd.mode != CHASSIS_UPSTEP)
    // {
    //     chassis_cmd.leg_angle = 0.0f;
    // }
    // else if (chassis_cmd.leg_state == LEG_LIFTOFF)
    // {
    //     chassis_cmd.leg_angle = 0.40f;
    // }
}

/*
 * @brief  	全麦面包运动学逆解算(3508电机)
 * @param	底盘控制结构体指针
 * @retval 	float[4], 各轮角速度
 * @note:   仅考虑麦轮俯视打叉，仰视画圆且轴长不等的情况,以辊子与地面接触为准
 */
void Chassis_Console(void)
{
    /*         vx
                ^
                |
           2//      \\3
           // \    / \\
                top      -->vy
           \\ /    \ //
           0\\      //1
    */
    float vx_gimbal;
    float vy_gimbal;
    float yaw_rel;

    vx_gimbal = chassis_cmd.vx_real;
    vy_gimbal = chassis_cmd.vy_real;
    yaw_rel = Normalize_Rad(gimbal_MF9025_motor->receive_data.RAD_single_round - FOLLOW_OMEGA_Z);

    // 将云台坐标系下的平移指令旋转到底盘坐标系，使 SPIN 下 W/S 仍沿云台朝向运动。
    chassis_cmd.vx_real = vx_gimbal * cosf(yaw_rel) + vy_gimbal * sinf(yaw_rel);
    chassis_cmd.vy_real = -vx_gimbal * sinf(yaw_rel) + vy_gimbal * cosf(yaw_rel);

    float omega_z = chassis_cmd.omega_z + chassis_cmd.omega_follow;

    chassis_cmd.wheel_target[1] = ((-chassis_cmd.vx_real - chassis_cmd.vy_real + omega_z * ((MECANUM_WIDTH_B ) / 2 + LENGTH_B) ) / WHEEL_RADIUS) * M3508_REDUCTION_RATIO;
    chassis_cmd.wheel_target[0] = ((chassis_cmd.vx_real - chassis_cmd.vy_real + omega_z * ((MECANUM_WIDTH_B ) / 2 + LENGTH_B) ) / WHEEL_RADIUS) * M3508_REDUCTION_RATIO;
    chassis_cmd.wheel_target[2] = ((chassis_cmd.vx_real + chassis_cmd.vy_real + omega_z * ((MECANUM_WIDTH_F ) / 2 + LENGTH_F) ) / WHEEL_RADIUS) * M3508_REDUCTION_RATIO;
    chassis_cmd.wheel_target[3] = ((-chassis_cmd.vx_real + chassis_cmd.vy_real + omega_z * ((MECANUM_WIDTH_F ) / 2 + LENGTH_F) ) / WHEEL_RADIUS) * M3508_REDUCTION_RATIO;

}

// float leg_tar = 0.0f; //调试用

// float leg0_pos;
float leg_angle_ramp_L;
float leg_angle_ramp_R;
//*89/ float legkp;
// float legki;
// float legkd;
// float legoutputlimit;
// float legioutlimit;
void Chassis_Send_Cmd()
{
    for (int i = 0; i < 4; i++)
    {
        // wheel_speed_ramp[i]->real_value = DJI_Motor_GetVal(chassis_m3508[i], MOTOR_SPEED, RAD);
        // DJI_Motor_Set_Ref(chassis_m3508[i], ramp_calc(wheel_speed_ramp[i] , chassis_cmd.wheel_target[i]));
        DJI_Motor_Set_Ref(chassis_m3508[i], chassis_cmd.wheel_target[i]);
    }

        // chassis_cmd.v_track = leg_angle_tar;

        DJI_Motor_Set_Ref(DJI_track_motor[0],chassis_cmd.v_track);
        DJI_Motor_Set_Ref(DJI_track_motor[1],-chassis_cmd.v_track);

    // leg0_pos = DM_arthrosis_motor[0]->receive_data.position;
    // leg0_torque = DM_arthrosis_motor[0]->receive_data.torque;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->kp = legkp ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->ki = legki ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->kd = legkd ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->output_limit = legoutputlimit ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->integral_limit = legioutlimit ;

    // leg_angle_ramp[0]->real_value = DM_arthrosis_motor[0]->receive_data.position; // 斜坡函数当前值更新
    // DM_Motor_SetTar(DM_arthrosis_motor[0], ramp_calc(leg_angle_ramp[0], -chassis_cmd.leg_angle));
    // leg_angle_ramp_L = ramp_calc(leg_angle_ramp[0], -chassis_cmd.leg_angle);
    // DM_Motor_SetTar(DM_arthrosis_motor[0], chassis_cmd.leg_angle);

    // leg_angle_ramp[1]->real_value = DM_arthrosis_motor[1]->receive_data.position; // 斜坡函数当前值更新
    // DM_Motor_SetTar(DM_arthrosis_motor[1], ramp_calc(leg_angle_ramp[1], chassis_cmd.leg_angle));
    // leg_angle_ramp_R = ramp_calc(leg_angle_ramp[1], chassis_cmd.leg_angle);
    // DM_Motor_SetTar(DM_arthrosis_motor[1], -chassis_cmd.leg_angle);

    // DM_Motor_SetTar(DM_track_motor[0], -chassis_cmd.v_track);
    // DM_Motor_Control(DM_track_motor[0]);
    // DM_Motor_SetTar(DM_track_motor[1], chassis_cmd.v_track);
    // DM_Motor_Control(DM_track_motor[1]);

    // Chassis_Disable();

    DM_Motor_Control(NULL);
}

static void Chassis_Enable(void)
{
    for (int i = 0; i < 4; i++)
    {
        DJI_Motor_Enable(chassis_m3508[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        // if (DM_track_motor[i]->receive_data.state == 0)
        // {
        //     DM_Motor_Enable(DM_track_motor[i]);
        // }
        DJI_Motor_Enable(DJI_track_motor[i]);
        if (DM_arthrosis_motor[i]->receive_data.state == 0)
        {
            DM_Motor_Enable(DM_arthrosis_motor[i]);
        }
    }
}

static void Leg_Start(void)
{
    for (int i = 0; i < 2; i++)
    {
        DM_Motor_Start(DM_arthrosis_motor[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        // DM_Motor_Start(DM_track_motor[i]);
        DJI_Motor_Enable(DJI_track_motor[i]);
    }
}

static void Leg_Stop(void)
{
    for (int i = 0; i < 2; i++)
    {
        DM_Motor_Stop(DM_arthrosis_motor[i]);
        if(DM_arthrosis_motor[i]->motor_settings.control_button == TORQUE_DIRECT_CONTROL)
        {
            PID_Clear(DM_arthrosis_pitch_pid[i]);
            PID_Clear(DM_arthrosis_roll_pid[i]);
        }

    }
    for (int i = 0; i < 2; i++)
    {
        // DM_Motor_Stop(DM_track_motor[i]);
        DJI_Motor_Disable(DJI_track_motor[i]);
    }
}

static void Chassis_Disable(void)
{
    for (int i = 0; i < 4; i++)
    {
        DJI_Motor_Disable(chassis_m3508[i]);
    }
    for (int i = 0; i < 2; i++)
    {
		DJI_Motor_Disable(DJI_track_motor[i]);
        if (DM_arthrosis_motor[i]->receive_data.state == 1)
        {
            // DM_Motor_Disable(DM_track_motor[i]);
            DM_Motor_Disable(DM_arthrosis_motor[i]);
            if(DM_arthrosis_motor[i]->motor_settings.control_button == TORQUE_DIRECT_CONTROL)
            {
                PID_Clear(DM_arthrosis_pitch_pid[i]);
                PID_Clear(DM_arthrosis_roll_pid[i]);
            }
        }
    }
}
