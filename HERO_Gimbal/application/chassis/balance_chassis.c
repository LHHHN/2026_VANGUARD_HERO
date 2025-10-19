/**
******************************************************************************
 * @file    balance_chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "balance_chassis.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)

#include "INS.h"
#include "digital_pid.h"
#include "kalman_filter.h"
#include "wfly_control.h"
#include "user_lib.h"
#include "bsp_dwt.h"

const float MG = 12.43f;// 质量*重力加速度*高度

float Poly_Coefficient_Speed[12][4] = {

	{-211.1604, 262.4733, -142.3981, 0.4066},
	{3.4911, -1.5272, -9.4294, 0.3226},
	{-35.4574, 38.2174, -14.8582, -0.2048},
	{-56.7793, 61.7068, -25.0284, -0.3671},
	{-25.5730, 42.9355, -28.1365, 8.9481},
	{-3.5400, 7.1552, -5.3137, 1.9401},
	{238.2745, -215.5877, 51.8643, 9.2288},
	{23.4550, -25.7294, 9.9475, 0.2743},
	{-17.5001, 27.4665, -16.9699, 4.9790},
	{-26.1363, 41.9664, -26.5049, 8.0921},
	{180.6515, -195.8207, 76.9185, 0.0872},
	{38.3391, -41.9819, 16.7751, -0.2964}

};

digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_roll_speed_pid; // 横滚角补偿pd

digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

/******************************底盘电机初始化结构体*****************************/

motor_init_config_t DM_right_back_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x00,
		.rx_id = 0x10,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

motor_init_config_t DM_right_front_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

motor_init_config_t DM_left_front_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x02,
		.rx_id = 0x12,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

motor_init_config_t DM_left_back_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		.can_handle = &hfdcan1,
		.tx_id = 0x03,
		.rx_id = 0x13,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

motor_init_config_t DJI_right_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x01,
		.rx_id = 0x00,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

motor_init_config_t DJI_left_motor_init = {
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
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x02,
		.rx_id = 0x00,
	},

	.motor_control_type = TORQUE_LOOP_CONTROL,
};

/******************************底盘电机初始化结构体*****************************/

/******************************底盘电机结构体*****************************/

DM_motor_instance_t *DM_right_front_motor_instance;
DM_motor_instance_t *DM_right_back_motor_instance;
DM_motor_instance_t *DM_left_front_motor_instance;
DM_motor_instance_t *DM_left_back_motor_instance;
DJI_motor_instance_t *DJI_right_motor_instance;
DJI_motor_instance_t *DJI_left_motor_instance;

/******************************底盘电机结构体*****************************/

/******************************底盘PID结构体*****************************/

digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

/******************************底盘PID结构体*****************************/

/******************************底盘模型结构体*****************************/

vmc_leg_t chassis_right_leg;
vmc_leg_t chassis_left_leg;

balance_chassis_t balance_chassis;

/******************************底盘模型结构体*****************************/


extern INS_behaviour_t INS;
extern wfly_t *rc_data;

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);

void Chassis_Init(void)
{
	/******************************底盘电机初始化*****************************/
	DM_right_back_motor_instance  = DM_Motor_Init(&DM_right_back_motor_init);
	DM_right_front_motor_instance = DM_Motor_Init(&DM_right_front_motor_init);
	DM_left_front_motor_instance  = DM_Motor_Init(&DM_left_front_motor_init);
	DM_left_back_motor_instance   = DM_Motor_Init(&DM_left_back_motor_init);
	DJI_right_motor_instance      = DJI_Motor_Init(&DJI_right_motor_init);
	DJI_left_motor_instance       = DJI_Motor_Init(&DJI_left_motor_init);

	balance_chassis.joint_motor[0] = DM_right_back_motor_instance;
	balance_chassis.joint_motor[1] = DM_right_front_motor_instance;
	balance_chassis.joint_motor[2] = DM_left_front_motor_instance;
	balance_chassis.joint_motor[3] = DM_left_back_motor_instance;

	balance_chassis.wheel_motor[0] = DJI_right_motor_instance;
	balance_chassis.wheel_motor[1] = DJI_left_motor_instance;
	/******************************底盘电机初始化*****************************/

	/******************************底盘模型初始化*****************************/
	VMC_Init(&chassis_right_leg);
	VMC_Init(&chassis_left_leg);
	/******************************底盘模型初始化*****************************/

	/******************************底盘预测器初始化*****************************/
	xvEstimateKF_Init(&vaEstimateKF);
	/******************************底盘预测器初始化*****************************/

	/******************************底盘运动pid初始化*****************************/
	digital_PID_t digital_PID_temp = {
		.Kp = 80.0f,
		.Ki = 0.0f,
		.Kd = 4.8f,
		.Kf = 0.0f,

		.output_LPF_RC = 0.0f,

		.improve = PID_IMPROVE_NONE_MOTION,

		.integral_limit = 0.0f,

		.output_max = 12.0f,
	};

	leg_roll_position_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 0.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 0.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 0.0f;

	leg_Tp_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 12.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 0.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 8.1f;

	leg_turn_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 100.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 12.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 12.0f;

	leg_l_length_pid = digital_PID_temp;
	leg_r_length_pid = digital_PID_temp;
	/******************************底盘运动pid初始化*****************************/
}

static void Chassis_Speed_Update(void);

static void Chassis_Leg_Update(void);

void Chassis_Observer(void)
{
	Chassis_Leg_Update( );
	Chassis_Speed_Update( );

	chassis_left_leg.v  = balance_chassis.v_filter;
	chassis_left_leg.x  = balance_chassis.x_filter;
	chassis_right_leg.v = -balance_chassis.v_filter;
	chassis_right_leg.x = -balance_chassis.x_filter;
}

void Chassis_Handle_Exception(void)
{
	;
}

void Chassis_Set_Mode(void)
{
	if ((rc_data->toggle.SF == WFLY_SW_UP) && (rc_data->online == 1))
	{
		balance_chassis.start_flag   = 1;
		balance_chassis.chassis_mode = CHASSIS_STAND_UP;
	}
	else
	{
		balance_chassis.start_flag   = 0;
		balance_chassis.x_filter     = 0.0f;
		balance_chassis.x_set        = balance_chassis.x_filter;
		balance_chassis.leg_set      = LEG_LENGTH_INIT;
		balance_chassis.turn_set     = balance_chassis.total_yaw;
		balance_chassis.chassis_mode = CHASSIS_ZERO_FORCE;
	}
}

void Chassis_Reference(void)
{
	switch (rc_data->toggle.SA)
	{
		case WFLY_SW_UP:
			if (rc_data->rc.ch[WFLY_CH_RY])
			{
				balance_chassis.leg_set += (float) rc_data->rc.ch[WFLY_CH_RY] * 0.000005f; // 调腿长
				balance_chassis.leg_length_change_flag = 1;
			}
			else
			{
				balance_chassis.leg_length_change_flag = 0;
			}

			if (balance_chassis.leg_set > LEG_LENGTH_MAX)
			{
				balance_chassis.leg_set = LEG_LENGTH_MAX;
			}
			else if (balance_chassis.leg_set < LEG_LENGTH_MIN)
			{
				balance_chassis.leg_set = LEG_LENGTH_MIN;
			}

			if (rc_data->toggle.SC == WFLY_SW_MID)
			{
				balance_chassis.leg_set = LEG_LENGTH_INIT;
			}

			break;

		case WFLY_SW_MID:
			balance_chassis.v_set = (float) rc_data->rc.ch[WFLY_CH_RY] * 0.0025f; // 调速度

			balance_chassis.x_set = 0.0f;

			balance_chassis.wz_set = -(float) rc_data->rc.ch[WFLY_CH_RX] * 0.00125f; // 调角速度

			if (user_abs(balance_chassis.wz_set) < 0.025f)
			{
				balance_chassis.wz_set = 0.0f;
			}

			if (balance_chassis.wz_set < -0.6f)
			{
				balance_chassis.wz_set = -0.6f;
			}
			else if (balance_chassis.wz_set > 0.6f)
			{
				balance_chassis.wz_set = 0.6f;
			}

			if (balance_chassis.wz_set != 0)
			{
				balance_chassis.turn_set = balance_chassis.total_yaw + balance_chassis.wz_set;
			}
			break;
		case WFLY_SW_DOWN:
			switch (rc_data->toggle.SC)
			{
				case WFLY_SW_UP:

					break;
				case WFLY_SW_MID:
					DM_Motor_DISABLE(balance_chassis.joint_motor[0]);
					DM_Motor_DISABLE(balance_chassis.joint_motor[1]);
					DM_Motor_DISABLE(balance_chassis.joint_motor[2]);
					DM_Motor_DISABLE(balance_chassis.joint_motor[3]);
					break;
				case WFLY_SW_DOWN:
					DM_Motor_ENABLE(balance_chassis.joint_motor[0]);
					DM_Motor_ENABLE(balance_chassis.joint_motor[1]);
					DM_Motor_ENABLE(balance_chassis.joint_motor[2]);
					DM_Motor_ENABLE(balance_chassis.joint_motor[3]);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	chassis_left_leg.v_tar  = balance_chassis.v_set;
	chassis_left_leg.x_tar  = balance_chassis.x_set;
	chassis_right_leg.v_tar = -balance_chassis.v_set;
	chassis_right_leg.x_tar = -balance_chassis.x_set;
}

static float LQR_K_L[12];
static float LQR_K_R[12];

static void Leg_Saturate_Torque(float *val, float min, float max);

void Chassis_Console(void)
{
	static float chassis_err[6] = {0.0f};

	balance_chassis.turn_T = leg_turn_pid.Kp * (balance_chassis.turn_set - balance_chassis.total_yaw) -
	                         leg_turn_pid.Kd * INS.Gyro[2]; // 这样计算更稳一点
	Leg_Saturate_Torque(&balance_chassis.turn_T, -1.0f * leg_turn_pid.output_max, leg_turn_pid.output_max);

	if (balance_chassis.standup_start_cnt > 300)
	{
		balance_chassis.roll_f0 = leg_roll_position_pid.Kp * (balance_chassis.roll_set - balance_chassis.roll)
		                          - leg_roll_position_pid.Kd * INS.Gyro[1];
		Leg_Saturate_Torque(&balance_chassis.roll_f0,
		                    -1.0f * leg_roll_position_pid.output_max,
		                    leg_roll_position_pid.output_max);
	}
	else
	{
		balance_chassis.roll_f0 =
				leg_roll_position_pid.Kp * (balance_chassis.roll_set - balance_chassis.roll) -
				leg_roll_position_pid.Kd * INS.Gyro[1];
		balance_chassis.roll_f0 = float_constrain(balance_chassis.roll_f0, (-0.25f), (0.25f));
	}

	balance_chassis.leg_tp = Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f); // 防劈叉pid计算


	if (chassis_left_leg.off_ground_flag == 1 && chassis_right_leg.off_ground_flag == 1)
	{
		balance_chassis.body_offground_flag = 1;
		balance_chassis.x_filter            = 0.0f;
		balance_chassis.x_set               = 0.0f;
		balance_chassis.turn_set            = balance_chassis.total_yaw;
	}
	else
	{
		balance_chassis.body_offground_flag = 0;
	}

	for (int i = 0 ; i < 12 ; i++)
	{
		LQR_K_L[i] = LQR_K_Calc(&Poly_Coefficient_Speed[i][0], chassis_left_leg.L0);
	}

	for (int i = 0 ; i < 12 ; i++)
	{
		LQR_K_R[i] = LQR_K_Calc(&Poly_Coefficient_Speed[i][0], chassis_right_leg.L0);
	}

	chassis_err[0] = 0.0f - chassis_right_leg.theta;
	chassis_err[1] = 0.0f - chassis_right_leg.d_theta;
	chassis_err[2] = chassis_right_leg.x_tar - chassis_right_leg.x;
	chassis_err[3] = chassis_right_leg.v_tar - chassis_right_leg.v;
	chassis_err[4] = 0.0f - chassis_right_leg.pitch;
	chassis_err[5] = 0.0f - chassis_right_leg.d_pitch;

	balance_chassis.wheel_motor[0]->transmit_data.current = 0.0f;

	for( int i = 0; i < 6; i++ )
	{
		balance_chassis.wheel_motor[0]->transmit_data.current += LQR_K_R[i] * chassis_err[i];
	}

	// 转向环
	balance_chassis.wheel_motor[0]->transmit_data.current += balance_chassis.turn_T;

	chassis_right_leg.Tp = 0.0f;

	for( int i = 6; i < 12; i++ )
	{
		chassis_right_leg.Tp -= LQR_K_R[i] * chassis_err[i - 6];
	}

	chassis_right_leg.Tp += balance_chassis.leg_tp;
	// 防劈腿
	chassis_right_leg.F_0_forward = MG / arm_cos_f32( chassis_right_leg.theta );
	// 支持力
	chassis_right_leg.F0 = MG / arm_cos_f32( chassis_right_leg.theta ) +
						   Digital_PID_Position( &leg_r_length_pid, chassis_right_leg.L0, balance_chassis.leg_set );

	if( balance_chassis.standup_start_cnt < 250 )
	{
		chassis_right_leg.F0 = float_constrain( chassis_right_leg.F0, ( -10.0f ), ( 10.0f ) );
	}					   

	chassis_right_leg.T1 = balance_chassis.joint_motor[2]->receive_data.torque;
	chassis_right_leg.T2 = balance_chassis.joint_motor[3]->receive_data.torque;

	Leg_Force_Calc( &chassis_right_leg );

	// chassis_right_leg.F_N = 140.05f + LowPass_Filter1p_Update( &right_F_N_lpf, chassis_right_leg.F_N );
	
	// VMC_FN_Ground_Detection_R( &chassis_right_leg, &right_F_N_maf, 20.0f );

	if( balance_chassis.recover_flag == 0 && chassis_right_leg.F_N < 20.0f && balance_chassis.leg_length_change_flag == 0 )
	{ // 离地了
		chassis_right_leg.off_ground_flag = 1;
		balance_chassis.wheel_motor[0]->transmit_data.current = 0.0f;
		chassis_right_leg.Tp = 0.0f;
		chassis_right_leg.Tp -= LQR_K_R[6] * chassis_err[0];
		chassis_right_leg.Tp -= LQR_K_R[7] * chassis_err[1];
		chassis_right_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
	chassis_right_leg.off_ground_flag = 0;
	chassis_right_leg.F0 -= balance_chassis.roll_f0;
	}

	Leg_Saturate_Torque( &chassis_right_leg.F0, -1.0f * MAX_F0, MAX_F0 );

	chassis_right_leg.j11 = ( chassis_right_leg.l1 * arm_sin_f32( chassis_right_leg.phi0 - chassis_right_leg.phi3 ) *
							  arm_sin_f32( chassis_right_leg.phi1 - chassis_right_leg.phi2 ) ) /
							arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 );
	chassis_right_leg.j12 = ( chassis_right_leg.l1 * arm_cos_f32( chassis_right_leg.phi0 - chassis_right_leg.phi3 ) *
							  arm_sin_f32( chassis_right_leg.phi1 - chassis_right_leg.phi2 ) ) /
							( chassis_right_leg.L0 * arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 ) );
	chassis_right_leg.j21 = ( chassis_right_leg.l4 * arm_sin_f32( chassis_right_leg.phi0 - chassis_right_leg.phi2 ) *
							  arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi4 ) ) /
							arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 );
	chassis_right_leg.j22 = ( chassis_right_leg.l4 * arm_cos_f32( chassis_right_leg.phi0 - chassis_right_leg.phi2 ) *
							  arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi4 ) ) /
							( chassis_right_leg.L0 * arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 ) );

	if( balance_chassis.standup_start_cnt < 250 )
	{
		chassis_right_leg.Tp = float_constrain( chassis_right_leg.Tp, ( -1.0f ), ( 1.0f ) );
	}

	chassis_right_leg.torque_set[0] =
			chassis_right_leg.j11 * chassis_right_leg.F0 + chassis_right_leg.j12 * chassis_right_leg.Tp;
	chassis_right_leg.torque_set[1] =
			chassis_right_leg.j21 * chassis_right_leg.F0 + chassis_right_leg.j22 * chassis_right_leg.Tp;

	//	Leg_Saturate_Torque (&chassis_move.wheel_motor[0].wheel_T, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);
	Leg_Saturate_Torque( &chassis_right_leg.torque_set[0], -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P );
	Leg_Saturate_Torque( &chassis_right_leg.torque_set[1], -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P );

	chassis_err[0] = 0.0f - chassis_left_leg.theta;
	chassis_err[1] = 0.0f - chassis_left_leg.d_theta;
	chassis_err[2] = chassis_left_leg.x_tar - chassis_left_leg.x;
	chassis_err[3] = chassis_left_leg.v_tar - chassis_left_leg.v;
	chassis_err[4] = 0.0f - chassis_left_leg.pitch;
	chassis_err[5] = 0.0f - chassis_left_leg.d_pitch;

	balance_chassis.wheel_motor[1]->transmit_data.current = 0.0f;

	for( int i = 0; i < 6; i++ )
	{
		balance_chassis.wheel_motor[1]->transmit_data.current += LQR_K_L[i] * chassis_err[i];
	}

// 转向环
	balance_chassis.wheel_motor[1]->transmit_data.current += balance_chassis.turn_T;

	chassis_left_leg.Tp = 0.0f;

	for( int i = 6; i < 12; i++ )
	{
		chassis_left_leg.Tp -= LQR_K_L[i] * chassis_err[i - 6];
	}

	chassis_left_leg.Tp += balance_chassis.leg_tp;

	chassis_left_leg.F_0_forward = MG / arm_cos_f32( chassis_left_leg.theta );
//	chassis_left_leg.F_0_forward = ramp_calc( left_F_0_ramp, chassis_left_leg.F_0_forward );

	// 支持力
	chassis_left_leg.F0 = chassis_left_leg.F_0_forward +
						  Digital_PID_Position( &leg_l_length_pid, chassis_left_leg.L0, balance_chassis.leg_set );

	if( balance_chassis.standup_start_cnt < 250 )
	{
		chassis_left_leg.F0 = float_constrain( chassis_left_leg.F0, ( -10.0f ), ( 10.0f ) );
	}

	chassis_left_leg.T1 = balance_chassis.joint_motor[0]->receive_data.torque;
	chassis_left_leg.T2 = balance_chassis.joint_motor[1]->receive_data.torque;

	Leg_Force_Calc( &chassis_left_leg );

	// chassis_left_leg.F_N = 140.05f + LowPass_Filter1p_Update( &left_F_N_lpf, chassis_left_leg.F_N );

	// VMC_FN_Ground_Detection_L( &chassis_left_leg, &left_F_N_maf, 20.0f );

	if( balance_chassis.recover_flag == 0 && chassis_left_leg.F_N < 20.0f && balance_chassis.leg_length_change_flag == 0 )
	{
		// 离地了
		chassis_left_leg.off_ground_flag = 1;
		balance_chassis.wheel_motor[1]->transmit_data.current = 0.0f;
		chassis_left_leg.Tp = 0.0f;
		chassis_left_leg.Tp -= LQR_K_L[6] * chassis_err[0];
		chassis_left_leg.Tp -= LQR_K_L[7] * chassis_err[1];
		chassis_left_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
	chassis_left_leg.off_ground_flag = 0;
	chassis_left_leg.F0 += balance_chassis.roll_f0;
	}

	Leg_Saturate_Torque( &chassis_left_leg.F0, -1.0f * MAX_F0, MAX_F0 );

	chassis_left_leg.j11 = ( chassis_left_leg.l1 * arm_sin_f32( chassis_left_leg.phi0 - chassis_left_leg.phi3 ) *
							 arm_sin_f32( chassis_left_leg.phi1 - chassis_left_leg.phi2 ) ) /
						   arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi2 );
	chassis_left_leg.j12 = ( chassis_left_leg.l1 * arm_cos_f32( chassis_left_leg.phi0 - chassis_left_leg.phi3 ) *
							 arm_sin_f32( chassis_left_leg.phi1 - chassis_left_leg.phi2 ) ) /
						   ( chassis_left_leg.L0 * arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi2 ) );
	chassis_left_leg.j21 = ( chassis_left_leg.l4 * arm_sin_f32( chassis_left_leg.phi0 - chassis_left_leg.phi2 ) *
							 arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi4 ) ) /
						   arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi2 );
	chassis_left_leg.j22 = ( chassis_left_leg.l4 * arm_cos_f32( chassis_left_leg.phi0 - chassis_left_leg.phi2 ) *
							 arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi4 ) ) /
						   ( chassis_left_leg.L0 * arm_sin_f32( chassis_left_leg.phi3 - chassis_left_leg.phi2 ) );

	if( balance_chassis.standup_start_cnt < 250 )
	{
		chassis_left_leg.Tp = float_constrain( chassis_left_leg.Tp, ( -1.0f ), ( 1.0f ) );
	}

	chassis_left_leg.torque_set[0] =
			chassis_left_leg.j11 * chassis_left_leg.F0 + chassis_left_leg.j12 * chassis_left_leg.Tp;
	chassis_left_leg.torque_set[1] =
			chassis_left_leg.j21 * chassis_left_leg.F0 + chassis_left_leg.j22 * chassis_left_leg.Tp;

	Leg_Saturate_Torque( &chassis_left_leg.torque_set[1], -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P );
	Leg_Saturate_Torque( &chassis_left_leg.torque_set[0], -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P );

}

void Chassis_Send_Cmd(void)
{
	//	static uint32_t chassis_cnt = 0;
	//	chassis_cnt++;
	//	if ((chassis_cnt % 1500) == 0) // 100Hz
	//	{
	//		DM_Motor_ENABLE(NULL);
	//	}
	//	else if ((chassis_cnt % 1000) == 0)
	//	{
	//		DM_Motor_DISABLE(NULL);
	//	}
}

#endif

static float vaEstimateKF_F[4] = {
	1.0f, 0.001f,
	0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

static float vaEstimateKF_P[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 后验估计协方差初始值

static float vaEstimateKF_Q[4] = {
	0.05f, 0.0f,
	0.0f, 0.01f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

static float vaEstimateKF_R[4] = {
	1200.0f, 0.0f,
	0.0f, 2000.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

static const float vaEstimateKF_H[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 设置矩阵H为常量

static float vaEstimateKF_K[4];

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
	Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
	memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel, float fusion_v_data[])
{
	// 卡尔曼滤波器测量值更新
	EstimateKF->MeasuredVector[0] = vel; // 测量速度
	EstimateKF->MeasuredVector[1] = acc; // 测量加速度

	// 卡尔曼滤波器更新函数
	Kalman_Filter_Update(EstimateKF);

	// 提取估计值
	for (uint8_t i = 0 ; i < 2 ; i++)
	{
		fusion_v_data[i] = EstimateKF->FilteredValue[i];
	}
}

static void Chassis_Speed_Update(void)
{
	static float wheel_z_e_right, wheel_z_e_left = 0.0F;
	static float v_right_b_axis, v_left_b_axis   = 0.0F;
	static float aver_v                          = 0.0F;

	static float fusion_v_data[2];

	static float last_vel_acc   = 0.0F;
	static uint32_t observe_dwt = 0;
	static float dt             = 0.0F;

	dt = DWT_GetDeltaT(&observe_dwt);

	balance_chassis.total_yaw = INS.YawTotalAngle;
	balance_chassis.roll      = INS.Roll;
	balance_chassis.theta_err = 0.0f - (chassis_right_leg.theta + chassis_left_leg.theta);

	vaEstimateKF_F[1] = dt;

	wheel_z_e_right = -balance_chassis.wheel_motor[0]->receive_data.speed * 2.0f * PI + INS.Gyro[0] + chassis_right_leg.d_theta;// 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
	wheel_z_e_left  = -balance_chassis.wheel_motor[1]->receive_data.speed * 2.0f * PI + INS.Gyro[0] + chassis_left_leg.d_theta;// 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正

	v_right_b_axis = wheel_z_e_right * WHEEL_RADIUS +
	                 chassis_right_leg.L0 * chassis_right_leg.d_theta * arm_cos_f32(chassis_right_leg.theta) +
	                 chassis_right_leg.d_L0 * arm_sin_f32(chassis_right_leg.theta); // 机体b系的速度
	v_left_b_axis = wheel_z_e_left * WHEEL_RADIUS +
	                chassis_left_leg.L0 * chassis_left_leg.d_theta * arm_cos_f32(chassis_left_leg.theta) +
	                chassis_left_leg.d_L0 * arm_sin_f32(chassis_left_leg.theta); // 机体b系的速度

	aver_v = (v_right_b_axis - v_left_b_axis) / 2.0f;

	xvEstimateKF_Update(&vaEstimateKF, -INS.MotionAccel_b[1], aver_v, fusion_v_data);

	if (balance_chassis.observe_flag == 0)
	{
		if (fabs(fusion_v_data[1] - last_vel_acc) / dt < 0.0001f)
		{
			balance_chassis.observe_flag = 1;
			balance_chassis.x_filter     = 0.0f;
		}
		else
		{
			last_vel_acc = fusion_v_data[1];
		}
	}
	else if (balance_chassis.observe_flag == 1)
	{
		;
	}

	if (balance_chassis.start_flag == 1)
	{
		balance_chassis.standup_start_cnt++;

		balance_chassis.v_filter = fusion_v_data[0];

		if (balance_chassis.v_set != 0.0f)
		{
			balance_chassis.nomotion_start_cnt = 0;
			balance_chassis.x_filter           = 0.0f;
		}
		else
		{
			balance_chassis.nomotion_start_cnt++;
			if (balance_chassis.nomotion_start_cnt > 150)
			{
				balance_chassis.x_filter +=
						fusion_v_data[0] * dt;
			}
		}
	}
	else
	{
		balance_chassis.nomotion_start_cnt = 0;
		balance_chassis.standup_start_cnt  = 0;
		balance_chassis.v_filter           = 0.0f;
		balance_chassis.x_filter           = 0.0f;
	}
}

static void Chassis_Leg_Update(void)
{
	chassis_left_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[2]->receive_data.position + 1.88106048f;
	chassis_left_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[3]->receive_data.position - 1.88106048f;

	chassis_right_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[0]->receive_data.position + 1.88106048f;
	chassis_right_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[1]->receive_data.position - 1.88106048f;

	chassis_left_leg.pitch   = (-INS.Pitch) * 0.1f + chassis_left_leg.pitch * 0.9f;
	chassis_left_leg.d_pitch = (-INS.Gyro[0]) * 0.1f + chassis_left_leg.d_pitch * 0.9f;

	chassis_right_leg.pitch   = (INS.Pitch) * 0.1f + chassis_right_leg.pitch * 0.9f;
	chassis_right_leg.d_pitch = (INS.Gyro[0]) * 0.1f + chassis_right_leg.d_pitch * 0.9f;

	balance_chassis.pitch_l      = chassis_left_leg.pitch;
	balance_chassis.pitch_gyro_l = chassis_left_leg.d_pitch;

	balance_chassis.pitch_r      = chassis_right_leg.pitch;
	balance_chassis.pitch_gyro_r = chassis_right_leg.d_pitch;

	static uint32_t leg_dwt = 0;
	static float dt         = 0.0F;

	if (chassis_left_leg.pitch > 0.34f || chassis_left_leg.pitch < -0.34f)
	{
		balance_chassis.recover_flag = 1;
	}
	else
	{
		balance_chassis.recover_flag = 0;
	}

	dt = DWT_GetDeltaT(&leg_dwt);

	VMC_Calc_Base_Data(&chassis_left_leg, &INS, dt);
	VMC_Calc_Base_Data(&chassis_right_leg, &INS, dt);
}

static void Leg_Saturate_Torque(float *val, float min, float max)
{
	if (*val > max)
	{
		*val = max;
	}
	else if (*val < min)
	{
		*val = min;
	}
	else
	{
		return;
	}
}

static void Chassis_Control_Leg(void)
{
	// 	balance_chassis.leg_tp = Digital_PID_Position(&leg_r_length_pid, chassis_right_leg.L0, balance_chassis.leg_set);
	// }
	// else
	// {
	// 	balance_chassis.leg_tp = 0.0f;
	// }

	// leg_roll_position_pid.ref_p = balance_chassis.roll_set;
	// leg_roll_position_pid.fab_p = INS.Roll;
	// leg_roll_position_pid.dt = 0.001f;
	// balance_chassis.roll_f0 = Digital_PID_Position(&leg_roll_position_pid, INS.Roll, balance_chassis.roll_set);

	// leg_Tp_pid.ref_p = 0.0f;
	// leg_Tp_pid.fab_p = balance_chassis.theta_err;
	// leg_Tp_pid.dt = 0.001f;
	// balance_chassis.leg_tp += Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f);

	// leg_turn_pid.ref_p = balance_chassis.turn_set;
	// leg_turn_pid.fab_p = INS.YawTotalAngle;
	// leg_turn_pid.dt = 0.001f;
	// balance_chassis.turn_T = Digital_PID_Position(&leg_turn_pid, INS.YawTotalAngle, balance_chassis.turn_set);
}
