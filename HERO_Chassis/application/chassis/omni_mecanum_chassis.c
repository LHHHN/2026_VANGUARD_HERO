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
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "rs485.h"
#include "gimbal.h"

//底板轮毂电机初始化参数

DJI_motor_instance_t *chassis_m3508[4];
Chassis_CmdTypedef chassis_cmd;

PID_t chassis_3508_speed_pid = {
    .kp = 40.0f,
    .ki = 0.3f,
    .kd = 0.05f,
    .output_limit = 5000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
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

//底盘关节电机DM4340初始化参数

DM_motor_instance_t *DM_arthrosis_motor[2]; // 0为左腿id=2 方向+ ，1为右腿id=3 方向-

PID_t arthrosis_angle_pid = {
    .kp = 500.0f,
    .ki = 1.0f,
    .kd = 750.0f,
    .output_limit = 28.0f, 
    .integral_limit = 1.0f,
    .dead_band = 0.0f,
};

motor_init_config_t DM_arthrosis_motor_init = {
	.controller_param_init_config = {
		.angle_PID = &arthrosis_angle_pid,
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
		// .close_loop_type = TORQUE_LOOP,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		// .control_button = TORQUE_DIRECT_CONTROL,		
        .control_button = POLYCYCLIC_LOOP_CONTROL
	},

	.motor_type = DM4340,

	.can_init_config = {
		.can_handle = &hfdcan2,
		.tx_id = 0x02,
		.rx_id = 0x12,
	},

};

//底盘关节电机斜坡函数
ramp_function_source_t *leg_angle_ramp[2]; // 0为左腿 1为右腿

ramp_init_config_t leg_angle_ramp_init = {
    .frame_period = 0.001f,        // 1ms控制周期
    .max_value = 0.92f,           // 最大输出
    .min_value = -0.92f,          // 最小输出
    .increase_value = 0.003f,        // 加速度
    .decrease_value = 0.0015f,        // 减速度
    .ramp_state = SLOPE_FIRST_REAL // 工作模式
};


//履带电机DM3519初始化参数
DM_motor_instance_t *DM_track_motor[2]; // 0为左腿id=1 方向+ ，1为右腿id=4 方向-

motor_init_config_t DM_track_motor_init = {
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
		// .close_loop_type = TORQUE_LOOP,
        .outer_loop_type = OPEN_LOOP,
        .close_loop_type = OPEN_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,		
        // .control_button = POLYCYCLIC_LOOP_CONTROL
	},

	.motor_type = DM3519,

	.can_init_config = {
		.can_handle = &hfdcan2,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},

};

//底盘跟随pid
PID_t chasiss_follow_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 2.5f,
    .output_limit = 25.0f, 
    .integral_limit = 5.0f,
    .dead_band = 0.1f,
};

void Chassis_Init(void)
{
    //底盘轮毂3508电机初始化
    for(int i = 0; i <4; i++)
    {
        chassis_3508_init.can_init_config.tx_id = 0x01 + i;
        chassis_3508_init.can_init_config.rx_id = 0x01 + i;
        chassis_m3508[i] = DJI_Motor_Init(&chassis_3508_init);
        chassis_m3508[i]->motor_feedback = RAD;
    }

    //底盘关节电机DM4340初始化 & 斜坡函数初始化    
    for(int i = 0 ; i< 2 ; i++)
    {
        DM_arthrosis_motor_init.can_init_config.tx_id = 0x02 + i ;
        DM_arthrosis_motor_init.can_init_config.rx_id = 0x12 + i ;
        DM_arthrosis_motor[i] = DM_Motor_Init(&DM_arthrosis_motor_init) ;
        DM_arthrosis_motor[i]->motor_feedback = DM_MOTOR_ABSOLUTE ;
        DM_arthrosis_motor[i]->dm_mode = MIT_MODE;
        DM_arthrosis_motor[i]->contorl_mode_state = SINGLE_TORQUE ;
        DM_Motor_Enable(DM_arthrosis_motor[i]);

        leg_angle_ramp[i] = ramp_init(&leg_angle_ramp_init);
    }

    //底盘关节角度斜坡函数初始化
    

    //底盘履带电机DM3519初始化
    DM_track_motor[0] = DM_Motor_Init(&DM_track_motor_init) ;
    DM_track_motor_init.can_init_config.tx_id = 0x04 ;
    DM_track_motor_init.can_init_config.rx_id = 0x14 ;
    DM_track_motor[1] = DM_Motor_Init(&DM_track_motor_init) ;
    DM_track_motor[0]->dm_mode = SPEED_MODE;
    DM_track_motor[1]->dm_mode = SPEED_MODE;
    DM_Motor_Enable(DM_track_motor[0]);
    DM_Motor_Enable(DM_track_motor[1]);
}

extern INS_behaviour_t INS;
extern RC_ctrl_t *rc_data;

extern LK_motor_instance_t *gimbal_MF9025_motor; //用来计算底盘跟随时的角速度

static void Chassis_Enable(void);
static void Chassis_Disable(void);

float leg_cnt = 0.0f; //离地时间计数

void Chassis_Set_Mode(void)
{
    //遥控器控制
    if( rc_data -> rc . switch_left == 1 && rc_data -> rc . switch_right == 1)
    // if( uart2_rx_message.switch_left == 1 && uart2_rx_message.switch_right == 1)
	{
        chassis_cmd.mode = CHASSIS_DISABLE;
	}
    else if( rc_data -> rc . switch_left == 1 && rc_data -> rc . switch_right == 3)
	// else if( uart2_rx_message.switch_left == 1 &&  uart2_rx_message.switch_right == 3)
	{
        chassis_cmd.mode = CHASSIS_STOP_C;
	}
    else if( rc_data -> rc . switch_left == 1 && rc_data -> rc . switch_right == 2)
    // else if( uart2_rx_message.switch_left == 1 &&  uart2_rx_message.switch_right == 2)
	{
        chassis_cmd.mode = CHASSIS_UPSTEP;//上台阶
	}
    else if( rc_data -> rc . switch_left == 3 && rc_data -> rc . switch_right == 1)
    // else if( uart2_rx_message.switch_left == 3 &&  uart2_rx_message.switch_right == 1)
	{
        chassis_cmd.mode = CHASSIS_SPIN;
	}
    else if( rc_data -> rc . switch_left == 3 && rc_data -> rc . switch_right == 2)
    // else if( uart2_rx_message.switch_left == 3 &&  uart2_rx_message.switch_right == 2)
	{
        chassis_cmd.mode = CHASSIS_FOLLOW;
	}
    else 
    {
        chassis_cmd.mode = CHASSIS_DISABLE;
    }

    // //离地检查
    // if(DM_arthrosis_motor[0]->receive_data.torque < -3.5f &&
    //    DM_arthrosis_motor[1]->receive_data.torque >  3.5f)
    // {
    //     // if(DM_arthrosis_motor[0]->receive_data.position < 0.1f &&
    //     //     DM_arthrosis_motor[1]->receive_data.position > -0.1f)
    //     // {
    //         chassis_cmd.leg_state = LEG_LIFTOFF;
    //     // }

    // }
    // else if(chassis_cmd.leg_state == LEG_LIFTOFF)
    // {
    //     if(DM_arthrosis_motor[0]->receive_data.torque > 6.0f &&
    //         DM_arthrosis_motor[1]->receive_data.torque <  -6.0f)
    //         {
    //             leg_cnt ++;
    //             if(leg_cnt >= 200)
    //             {
    //                 leg_cnt = 0;
    //                 chassis_cmd.leg_state = LEG_NORMAL;
    //             }
    //             chassis_cmd.leg_state = LEG_NORMAL;
    //         }
    // }
}


//更新目标值
void Chassis_Reference(void)
{
    if(chassis_cmd.mode == CHASSIS_DISABLE)
    {
        Chassis_Disable();
    }
    else
    {
        Chassis_Enable();
    }
    
    chassis_cmd. vx = (float) rc_data -> rc . rocker_l1 * REMOTE_X_SEN ;
	chassis_cmd. vy = (float) rc_data -> rc . rocker_l_ * REMOTE_Y_SEN ;
    // chassis_cmd. vx = (float) uart2_rx_message.rocker_l1 * REMOTE_X_SEN ;
    // chassis_cmd. vy = (float) uart2_rx_message.rocker_l_ * REMOTE_Y_SEN ;
    chassis_cmd.v_track = 0.0f;
    // chassis_cmd.leg_angle = 0.0f ;
    if(chassis_cmd.leg_state == LEG_NORMAL)
    {
        chassis_cmd.leg_angle = 0.0f ;
    }

    if(chassis_cmd.mode == CHASSIS_SPIN)
    {
        chassis_cmd.omega_z = 8.0f;
    }
    else if(chassis_cmd. mode == CHASSIS_FOLLOW)
    {  
        chassis_cmd.omega_follow = PID_Position(&chasiss_follow_pid, gimbal_MF9025_motor->receive_data.RAD_single_round, FOLLOW_OMEGA_Z);
        // chassis_cmd. omega_z = rc_data->rc . rocker_r_ * REMOTE_OMEGA_Z_SEN ;
        // chassis_cmd. omega_z = uart2_rx_message.rocker_r_ * REMOTE_OMEGA_Z_SEN ;
    }
    else if(chassis_cmd.mode == CHASSIS_UPSTEP)
    {
        if(rc_data->rc . rocker_r1 > 0 && chassis_cmd.leg_state == LEG_NORMAL)
        {
            chassis_cmd.leg_angle = rc_data->rc . rocker_r1 * 0.001394f ; //660换算成弧度
        }
        // if(uart2_rx_message.rocker_r1 > 0 && chassis_cmd.leg_state == LEG_NORMAL)
        // {
        //     chassis_cmd.leg_angle = uart2_rx_message.rocker_r1 * 0.001394f ;
        // }
        chassis_cmd.v_track = 15.0f ;
    }
    else if(chassis_cmd.mode == CHASSIS_STOP_C)
    {
        chassis_cmd.omega_z = 0.0f;
        chassis_cmd.omega_follow = 0.0f;
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vy = 0.0f;
        chassis_cmd.leg_angle = 0.0f ; 
    }

    if(chassis_cmd.leg_state == LEG_LIFTOFF)
    {
        chassis_cmd.leg_angle = 0.0f ;
    }
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
		   || /    \ || 
		   1||       ||0
					
	*/
    float omega_z = chassis_cmd.omega_z + chassis_cmd.omega_follow;

    // chassis_cmd.wheel_target[0] = ((- chassis_cmd.vx + omega_z * OMNI_WIDTH /2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    // chassis_cmd.wheel_target[1] = ((  chassis_cmd.vx + omega_z * OMNI_WIDTH /2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    // chassis_cmd.wheel_target[2] = ((  chassis_cmd.vx + chassis_cmd.vy - omega_z * (MECANUM_WIDTH + LENGTH) / 2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    // chassis_cmd.wheel_target[3] = ((- chassis_cmd.vx + chassis_cmd.vy - omega_z * (MECANUM_WIDTH + LENGTH) / 2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    chassis_cmd.wheel_target[0] = ((- chassis_cmd.vx + omega_z * OMNI_WIDTH /2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    chassis_cmd.wheel_target[1] = ((  chassis_cmd.vx + omega_z * OMNI_WIDTH /2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    chassis_cmd.wheel_target[2] = ((  chassis_cmd.vx + chassis_cmd.vy + omega_z * (MECANUM_WIDTH + LENGTH) / 2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;
    chassis_cmd.wheel_target[3] = ((- chassis_cmd.vx + chassis_cmd.vy + omega_z * (MECANUM_WIDTH + LENGTH) / 2)/ WHEEL_RADIUS) * (RPM_2_RAD_PER_SEC * 60 / 2 / PI / WHEEL_RADIUS) ;

}

// float leg_tar = 0.0f; //调试用

// float leg0_pos;
float leg0_torque;
// float legkp;
// float legki;
// float legkd;
// float legoutputlimit;
// float legioutlimit;
void Chassis_Send_Cmd()
{
    for(int i = 0; i < 4; i++)
    {
        DJI_Motor_Set_Ref(chassis_m3508[i], chassis_cmd.wheel_target[i]);
        DJI_Motor_Control(chassis_m3508[i]);
    }

    // leg0_pos = DM_arthrosis_motor[0]->receive_data.position;
    // leg0_torque = DM_arthrosis_motor[0]->receive_data.torque;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->kp = legkp ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->ki = legki ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->kd = legkd ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->output_limit = legoutputlimit ;
    // DM_arthrosis_motor[0]->motor_controller.angle_PID->integral_limit = legioutlimit ;

    leg_angle_ramp[0]->real_value = DM_arthrosis_motor[0]->receive_data.position ; //斜坡函数当前值更新
    DM_Motor_SetTar(DM_arthrosis_motor[0], ramp_calc(leg_angle_ramp[0],chassis_cmd.leg_angle));
    // DM_Motor_SetTar(DM_arthrosis_motor[0], chassis_cmd.leg_angle);
    DM_Motor_Control(DM_arthrosis_motor[0]);

    leg_angle_ramp[1]->real_value = DM_arthrosis_motor[1]->receive_data.position ; //斜坡函数当前值更新
    DM_Motor_SetTar(DM_arthrosis_motor[1], -ramp_calc(leg_angle_ramp[1],chassis_cmd.leg_angle));
    // DM_Motor_SetTar(DM_arthrosis_motor[1], -chassis_cmd.leg_angle);
    DM_Motor_Control(DM_arthrosis_motor[1]);

    DM_Motor_SetTar(DM_track_motor[0], -chassis_cmd.v_track);
    DM_Motor_Control(DM_track_motor[0]);
    DM_Motor_SetTar(DM_track_motor[1], chassis_cmd.v_track);
    DM_Motor_Control(DM_track_motor[1]);
    
		
}

static void Chassis_Enable(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Enable(chassis_m3508[i]);
    }
    for(int i = 0; i <2; i++)
    {
        DM_Motor_Enable(DM_arthrosis_motor[i]);
        DM_Motor_Start(DM_arthrosis_motor[i]);
    }
    for(int i = 0; i <2; i++)
    {
        DM_Motor_Enable(DM_track_motor[i]);
        DM_Motor_Start(DM_track_motor[i]);
    }
}

static void Chassis_Disable(void)
{
    for(int i = 0; i <4; i++)
    {
        DJI_Motor_Disable(chassis_m3508[i]);
    }
    for(int i = 0; i <2; i++)
    {
        DM_Motor_Stop(DM_track_motor[i]);
        DM_Motor_Disable(DM_track_motor[i]);
    }
}