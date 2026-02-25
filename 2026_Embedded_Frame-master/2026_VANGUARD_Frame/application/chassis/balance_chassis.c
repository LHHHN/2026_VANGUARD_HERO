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
#include "lms.h"

#include "kalman_one_filter.h" 

#include "buzzer.h"

#include "bsp_dwt.h"

/******************************测试变量和宏定义开关*****************************/
#define OFF_GROUND_TEST 0
#define WHEEL_POLARITY_TEST 0 //速度解算更换
#define JOINT_POLARITY_TEST 1
#define TP_POLARITY_TEST 0
#define LENGTH_TEST 1

float F_0_forward_test;
float F_N_forward_test[2];
float DD_W_forward_test;

float chassis_err_left_test[6]  = {0.0f};
float chassis_err_right_test[6] = {0.0f};
/******************************测试变量和宏定义开关*****************************/

#define OFFSET_IK_Y 0.04f
#define OFFSET_IK_X 0.055f

const float half_MG_real = 42.183f;//（此用3.624kg）5.93kg * 9.8m/s^2
const float MG_FF        = 41.89f;//55.86f;//1/4

float Poly_Coefficient_STOOL[12][4] = {

{82.4009,-39.4696,-22.0446,-0.9738},
{21.9039,-16.4822,-2.2514,0.0146},
{25.1136,-9.8773,-4.9499,-0.1946},
{29.4887,-14.3236,-4.1164,-0.4055},
{51.1093,-42.7048,4.2856,4.3785},
{9.2675,-7.8761,0.7344,0.9713},
{2701.6583,-2596.5383,658.9719,61.4568},
{379.4068,-393.3438,110.3478,14.3670},
{690.3630,-569.7314,73.5116,39.8718},
{558.5091,-441.2106,28.6938,47.6886},
{-256.5285,-228.0370,355.0629,-33.0917},
{-49.2645,-54.6030,79.4233,-8.9775},
	// {64.7156,-9.4505,-41.5705,0.1322},
	// {21.8036,-15.6433,-4.2333,0.1733},
	// {7.3186,12.1067,-14.0296,0.4227},
	// {12.8813,7.1738,-13.7336,0.2330},
	// {107.3381,-82.2312,7.6330,6.9684},
	// {22.1850,-17.2518,1.7336,1.5411},
	// {4171.8679,-3782.3738,912.6403,80.5085},
	// {596.6832,-572.8503,156.1832,15.6234},
	// {1083.4482,-814.2487,75.1806,60.2772},
	// {966.2507,-696.6336,30.0413,70.8579},
	// {292.7232,-1169.8230,862.3066,-69.8215},
	// {61.4756,-256.2029,191.8739,-17.7959},

// {113.3229,-74.9517,-20.2015,-2.9993},
// {27.0144,-24.0958,-2.2588,-0.2091},
// {45.4042,-29.1366,-1.0878,-1.1125},
// {52.2102,-36.6208,0.6608,-1.4904},
// {30.6691,-29.7802,0.1144,7.3871},
// {0.9671,-1.2560,-0.9756,1.4397},
// {1924.0969,-2107.5843,626.6529,80.3823},
// {182.7320,-252.7903,92.7877,20.7504},
// {457.4128,-427.2208,57.6128,47.4735},
// {282.6602,-261.0433,3.4892,56.5663},
// {-304.3217,-64.0462,285.9043,-24.4333},
// {-20.2340,-43.0353,62.1460,-8.0999},
};

float Poly_Coefficient_Leg[12][4] = {
	
{-116.1018,140.5951,-78.2089,2.4691},
{4.3428,-3.0514,-6.4732,0.4411},
{-91.8140,94.9564,-34.5810,1.9547},
{-90.9764,95.9217,-37.1137,2.0732},
{-18.3876,70.1069,-55.4021,15.4498},
{-2.2273,8.6459,-7.1029,1.9745},
{970.5407,-793.7433,154.9374,38.8354},
{153.4441,-151.1563,51.3794,1.4084},
{31.5038,81.6229,-97.9826,33.7156},
{93.1688,22.5320,-80.3805,34.3412},
{3157.3550,-3236.4966,1159.7420,-43.8911},
{408.5641,-417.8815,149.2261,-9.5342},

// {-82.7164,100.3916,-61.8783,1.0022},
// {0.6833,-0.7882,-6.5215,0.2473},
// {-55.8382,56.3827,-19.9958,-0.0893},
// {-58.3586,59.9242,-22.9471,-0.1044},
// {-103.0565,135.9682,-69.4196,16.9430},
// {-6.7869,10.3099,-5.9989,1.8142},
// {458.2294,-381.1845,74.7631,29.0330},
// {67.4772,-66.7864,23.5415,2.7409},
// {-221.0904,283.1637,-139.7140,31.9432},
// {-223.9796,288.7902,-144.3984,34.9153},
// {2439.4882,-2473.0642,883.7802,-8.8836},
// {260.0660,-267.7673,98.2371,-4.4445},

};

/***********************************************************底盘初始化参数配置变量与函数***********************************************************/

/******************************底盘电机初始化结构体*****************************/

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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		// .can_mode = FDCAN_FD_CAN,
		.can_handle = &hfdcan1,
		.tx_id = 0x00,
		.rx_id = 0x10,
	},

};

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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		// .can_mode = FDCAN_FD_CAN,
		.can_handle = &hfdcan1,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},

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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		// .can_mode = FDCAN_FD_CAN,	
		.can_handle = &hfdcan1,
		.tx_id = 0x02,
		.rx_id = 0x12,
	},

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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = DM8009P,

	.can_init_config = {
		// .can_mode = FDCAN_FD_CAN,		
		.can_handle = &hfdcan1,
		.tx_id = 0x03,
		.rx_id = 0x13,
	},
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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_mode = FDCAN_CLASSIC_CAN,		
		.can_handle = &hfdcan3,
		.tx_id = 0x01,
		.rx_id = 0x00,
	},

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
		.outer_loop_type = TORQUE_LOOP,
		.close_loop_type = TORQUE_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
		.control_button = TORQUE_DIRECT_CONTROL,
	},

	.motor_type = M3508,

	.can_init_config = {
		.can_mode = FDCAN_CLASSIC_CAN,		
		.can_handle = &hfdcan3,
		.tx_id = 0x02,
		.rx_id = 0x00,
	},

};

/******************************底盘电机初始化结构体*****************************/

/******************************底盘电机结构体*****************************/

DM_motor_instance_t *DM_right_back_motor_instance;
DM_motor_instance_t *DM_right_front_motor_instance;
DM_motor_instance_t *DM_left_front_motor_instance;
DM_motor_instance_t *DM_left_back_motor_instance;
DJI_motor_instance_t *DJI_right_motor_instance;
DJI_motor_instance_t *DJI_left_motor_instance;

/******************************底盘电机结构体*****************************/

/******************************底盘PID结构体*****************************/

#if LENGTH_TEST 
digital_PID_t leg_r_length_position_pid;
digital_PID_t leg_l_length_position_pid;
digital_PID_t leg_r_length_speed_pid;
digital_PID_t leg_l_length_speed_pid;
digital_PID_t leg_length_pid;
#else
digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd
#endif

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

/******************************底盘PID结构体*****************************/

/******************************底盘模型结构体*****************************/

vmc_leg_t chassis_right_leg;
vmc_leg_t chassis_left_leg;

balance_chassis_t balance_chassis;

/******************************底盘模型结构体*****************************/

/******************************底盘滤波器初始化*****************************/
ramp_init_config_t left_leg_v_target_ramp_init = {
	.decrease_value = 0.002f,
	.increase_value = 0.0025f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_init_config_t right_leg_v_target_ramp_init = {
	.decrease_value = 0.002f,
	.increase_value = 0.0025f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_function_source_t *left_leg_v_target_ramp;
ramp_function_source_t *right_leg_v_target_ramp;

ramp_init_config_t left_wheel_torque_ramp_init = {
	.decrease_value = 0.25f,
	.increase_value = 0.25f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_init_config_t right_wheel_torque_ramp_init = {
	.decrease_value = 0.25f,
	.increase_value = 0.25f,
	.frame_period = 0.001f,
	.max_value = 0.0f,
	.min_value = 0.0f,
	.ramp_state = SLOPE_FIRST_REAL,
};

ramp_function_source_t *left_wheel_torque_ramp;
ramp_function_source_t *right_wheel_torque_ramp;

nlms_t left_leg_F_N_nlms;
nlms_t right_leg_F_N_nlms;

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体

kalman_one_filter_t dd_w_kf[2];
kalman_one_filter_t leg_FN[2];
/******************************底盘滤波器初始化*****************************/

extern INS_behaviour_t INS;
extern wfly_t *rc_data;

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);

uint32_t watch_test = 0;

void Chassis_Init(void)
{
	/******************************底盘电机初始化*****************************/
	DM_right_front_motor_instance  = DM_Motor_Init(&DM_right_front_motor_init);
	DM_right_back_motor_instance = DM_Motor_Init(&DM_right_back_motor_init);
	DM_left_front_motor_instance  = DM_Motor_Init(&DM_left_front_motor_init);
	DM_left_back_motor_instance   = DM_Motor_Init(&DM_left_back_motor_init);
	DJI_right_motor_instance      = DJI_Motor_Init(&DJI_right_motor_init);
	DJI_left_motor_instance       = DJI_Motor_Init(&DJI_left_motor_init);

	balance_chassis.joint_motor[0] = DM_right_front_motor_instance;
	balance_chassis.joint_motor[1] = DM_right_back_motor_instance;
	balance_chassis.joint_motor[2] = DM_left_front_motor_instance;
	balance_chassis.joint_motor[3] = DM_left_back_motor_instance;

	balance_chassis.wheel_motor[0] = DJI_right_motor_instance;
	balance_chassis.wheel_motor[1] = DJI_left_motor_instance;

	for (uint8_t i = 0 ; i < 4 ; i++)
	{
		balance_chassis.joint_motor[i]->dm_mode            = MIT_MODE;
		balance_chassis.joint_motor[i]->contorl_mode_state = SINGLE_TORQUE;
	}
	/******************************底盘电机初始化*****************************/

	/******************************底盘模型初始化*****************************/
	VMC_Init(&chassis_right_leg);
	VMC_Init(&chassis_left_leg);
	/******************************底盘模型初始化*****************************/

	/******************************底盘预测器初始化*****************************/
	xvEstimateKF_Init(&vaEstimateKF);
	/******************************底盘预测器初始化*****************************/

	/******************************底盘控制器初始化*****************************/
	digital_PID_t digital_PID_temp = {
		.Kp = 120.0f,
		.Ki = 0.0f,
		.Kd = 2.4f,
		.Kf = 0.0f,

		.dead_band = 0.015f,

		.output_LPF_RC = 0.0f,

		.improve = PID_IMPROVE_NONE_MOTION,

		.integral_limit = 0.0f,

		.output_max = 42.183f,
	};

	leg_roll_position_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 48.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 2.4f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.dead_band      = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_RAMP_TARGET;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 8.1f;

	leg_Tp_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 4.8f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 1.25f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.dead_band      = 0.0025f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 2.207f;

	leg_turn_pid = digital_PID_temp;

#if LENGTH_TEST	
	digital_PID_temp.Kp             = 120.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 15.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 27.93f;

	leg_length_pid = digital_PID_temp;

	digital_PID_temp.Kp             = 8.4f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 1.2f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.4f;
	digital_PID_temp.improve        = PID_OUTPUT_FILTER;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.dead_band 		= 0.005f;
	digital_PID_temp.output_max     = 2.0f;
#else
	digital_PID_temp.Kp             = 120.0f;
	digital_PID_temp.Ki             = 0.0f;
	digital_PID_temp.Kd             = 2.4f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.output_max     = 27.93f;
#endif	

#if LENGTH_TEST	
	leg_r_length_position_pid = digital_PID_temp;
	leg_l_length_position_pid = digital_PID_temp;
	digital_PID_temp.Kp             = 8.1f;
	digital_PID_temp.Ki             = 4.8f;
	digital_PID_temp.Kd             = 0.0f;
	digital_PID_temp.Kf             = 0.0f;
	digital_PID_temp.output_LPF_RC  = 0.0f;
	digital_PID_temp.improve        = PID_IMPROVE_NONE_MOTION;
	digital_PID_temp.integral_limit = 0.0f;
	digital_PID_temp.dead_band 		= 0.0f;
	digital_PID_temp.output_max     = 27.93f;
	leg_r_length_speed_pid = digital_PID_temp;
	leg_l_length_speed_pid = digital_PID_temp;
#else
	leg_l_length_pid = digital_PID_temp;
	leg_r_length_pid = digital_PID_temp;
#endif	
	/******************************底盘控制器初始化*****************************/

	/******************************底盘滤波器初始化*****************************/
	left_leg_v_target_ramp  = ramp_init(&left_leg_v_target_ramp_init);
	right_leg_v_target_ramp = ramp_init(&right_leg_v_target_ramp_init);

	left_wheel_torque_ramp  = ramp_init(&left_wheel_torque_ramp_init);
	right_wheel_torque_ramp = ramp_init(&right_wheel_torque_ramp_init);

	Nlms_Init(&left_leg_F_N_nlms, 50, 0);
	Nlms_Init(&right_leg_F_N_nlms, 50, 0);

	Kalman_One_Init(&dd_w_kf[0], 0.01f, 20.0f);
	Kalman_One_Init(&dd_w_kf[1], 0.01f, 20.0f);

	Kalman_One_Init(&leg_FN[0], 0.01f, 20.0f);
	Kalman_One_Init(&leg_FN[1], 0.01f, 20.0f);
	/******************************底盘滤波器初始化*****************************/

	/******************************底盘初始姿态初始化*****************************/
	balance_chassis.roll_set = 0.014f;
	balance_chassis.prejump_flag = 0;
	balance_chassis.jump_flag = 0;
	balance_chassis.postjump_flag = 0;
	balance_chassis.stop_cnt = 1000;
	balance_chassis.leg_set  = LEG_LENGTH_INIT;
	balance_chassis.chassis_mode = CHASSIS_ZERO_FORCE;
	/******************************底盘初始姿态初始化*****************************/
	
	watch_test = LEG_JUMP_POST_TIME;
}

/***********************************************************底盘初始化参数配置变量与函数***********************************************************/


/***********************************************************底盘状态观测变量与函数***********************************************************/
static void Chassis_Speed_Update(void);

static void Chassis_Leg_Update(void);

void Chassis_Observer(void)
{
	static uint32_t chassis_ob_cnt = 0;
	chassis_ob_cnt++;

	balance_chassis.roll  = INS.Roll;
	balance_chassis.pitch = INS.Pitch;
	balance_chassis.yaw   = INS.Yaw;

	balance_chassis.d_roll  = INS.Gyro[0];
	balance_chassis.d_pitch = INS.Gyro[1];
	balance_chassis.d_yaw   = INS.Gyro[2];

	Chassis_Leg_Update( );

	if((chassis_ob_cnt % 2) == 0)
	{
		Chassis_Speed_Update( );
	}

#if WHEEL_POLARITY_TEST
	chassis_left_leg.v  = balance_chassis.v_filter;
	chassis_left_leg.x  = balance_chassis.x_filter;
	chassis_right_leg.v = -balance_chassis.v_filter;
	chassis_right_leg.x = -balance_chassis.x_filter;
#else
	chassis_left_leg.v  = -balance_chassis.v_filter;
	chassis_left_leg.x  = -balance_chassis.x_filter;
	chassis_right_leg.v = balance_chassis.v_filter;
	chassis_right_leg.x = balance_chassis.x_filter;
#endif
}
/***********************************************************底盘状态观测变量与函数***********************************************************/

/***********************************************************底盘异常处理变量与函数***********************************************************/
void Chassis_Handle_Exception(void)
{
	static uint16_t exception_cnt = 0;
#if TP_POLARITY_TEST

#else
	if (balance_chassis.pitch > 0.5f || balance_chassis.pitch < -0.5f)
	{
		if(balance_chassis.chassis_mode != CHASSIS_DEBUG || balance_chassis.chassis_mode != CHASSIS_CALIBRATE)
		{
			balance_chassis.chassis_mode = CHASSIS_EXCEPTION;
		}
		balance_chassis.recover_flag = 1;
	}

	if(balance_chassis.roll > 0.5f || balance_chassis.pitch < -0.5f)
	{
		if(balance_chassis.chassis_mode != CHASSIS_DEBUG || balance_chassis.chassis_mode != CHASSIS_CALIBRATE)
		{
			balance_chassis.chassis_mode = CHASSIS_EXCEPTION;
		}
		balance_chassis.recover_flag = 1;
	}

	if((balance_chassis.v_set == 0.0f && (user_abs(balance_chassis.v_filter) > 2.0f)) || (balance_chassis.x_set == 0.0f && (user_abs(balance_chassis.x_filter) > 2.0f)))
	{
		if(exception_cnt > 2000)
		{
			balance_chassis.chassis_mode = CHASSIS_EXCEPTION;
			balance_chassis.recover_flag = 1;
		}
		else
		{
			exception_cnt++;
		}
	}
	else
	{
		if(exception_cnt > 150)
		{
			if(exception_cnt > 1000)
			{
				exception_cnt = 999;
			}
			exception_cnt--;
		}
		else
		{
			exception_cnt = 0;
		}
	}
#endif	
}
/***********************************************************底盘异常处理变量与函数***********************************************************/

/***********************************************************底盘控制模式变量与函数***********************************************************/
void Chassis_Set_Mode(void)
{
	if(balance_chassis.recover_flag == 1)
	{
		DJI_Motor_Disable(balance_chassis.wheel_motor[0]);
		DJI_Motor_Disable(balance_chassis.wheel_motor[1]);
		DM_Motor_Stop(balance_chassis.joint_motor[0]);
		DM_Motor_Stop(balance_chassis.joint_motor[1]);
		DM_Motor_Stop(balance_chassis.joint_motor[2]);
		DM_Motor_Stop(balance_chassis.joint_motor[3]);		
	}
	else
	{
		switch (rc_data->toggle.SD)
		{
			case WFLY_SW_UP:
				if(balance_chassis.chassis_mode == CHASSIS_ZERO_FORCE || 
					balance_chassis.chassis_mode == CHASSIS_CALIBRATE || 
					balance_chassis.chassis_mode == CHASSIS_DEBUG ||
					balance_chassis.start_flag == 0)
				{
					balance_chassis.standup_start_cnt = 0;
					balance_chassis.chassis_mode = CHASSIS_CUSHIONING;
				}
				else
				{
					balance_chassis.chassis_mode = balance_chassis.chassis_mode;
				}
				break;
			case WFLY_SW_MID:
				if(rc_data->toggle.SF == WFLY_SW_DOWN)
				{
					balance_chassis.chassis_mode = CHASSIS_CALIBRATE;
				}
				break;
			case WFLY_SW_DOWN:
				if(rc_data->toggle.SF == WFLY_SW_DOWN)
				{
					balance_chassis.chassis_mode = CHASSIS_DEBUG;
				}
				break;
			default:
				break;
		}

		if ((rc_data->toggle.SF == WFLY_SW_UP) && (rc_data->online == 1) && (balance_chassis.chassis_mode != CHASSIS_EXCEPTION))
		{
			balance_chassis.start_flag = 1;

			switch(balance_chassis.chassis_mode)
			{
				case CHASSIS_CUSHIONING:
					chassis_right_leg.theta_target = 0.0f;
					chassis_left_leg.theta_target  = 0.0f;
					DJI_Motor_Enable(balance_chassis.wheel_motor[0]);
					DJI_Motor_Enable(balance_chassis.wheel_motor[1]);
					DM_Motor_Stop(balance_chassis.joint_motor[0]);
					DM_Motor_Stop(balance_chassis.joint_motor[1]);
					DM_Motor_Stop(balance_chassis.joint_motor[2]);
					DM_Motor_Stop(balance_chassis.joint_motor[3]);
					if(user_abs(balance_chassis.pitch) < 0.05f)
					{
						if(balance_chassis.standup_start_cnt > 100)
						{
							balance_chassis.chassis_mode = CHASSIS_STAND_UP;
						}
						else
						{
							balance_chassis.standup_start_cnt++;
						}
					}
					break;
				case CHASSIS_STAND_UP:
						chassis_right_leg.theta_target = 0.0f;
						chassis_left_leg.theta_target  = 0.0f;
						DJI_Motor_Enable(balance_chassis.wheel_motor[0]);
						DJI_Motor_Enable(balance_chassis.wheel_motor[1]);
						DM_Motor_Start(balance_chassis.joint_motor[0]);
						DM_Motor_Start(balance_chassis.joint_motor[1]);
						DM_Motor_Start(balance_chassis.joint_motor[2]);
						DM_Motor_Start(balance_chassis.joint_motor[3]);
						if((user_abs(balance_chassis.leg_set - chassis_left_leg.L0) < 0.03f) || user_abs(balance_chassis.leg_set - chassis_right_leg.L0) < 0.03f)
						{
							balance_chassis.chassis_mode = CHASSIS_AUTO;
							balance_chassis.leg_length_change_flag = 0;
						}
						else
						{
							balance_chassis.leg_length_change_flag = 1;
						}
					break;
				case CHASSIS_AUTO:
						DJI_Motor_Enable(balance_chassis.wheel_motor[0]);
						DJI_Motor_Enable(balance_chassis.wheel_motor[1]);
						DM_Motor_Start(balance_chassis.joint_motor[0]);
						DM_Motor_Start(balance_chassis.joint_motor[1]);
						DM_Motor_Start(balance_chassis.joint_motor[2]);
						DM_Motor_Start(balance_chassis.joint_motor[3]);
					break;
				default:
					break;
			}

			if(rc_data->toggle.SH == WFLY_SW_DOWN)
			{
				if(balance_chassis.jump_start_cnt > LEG_JUMP_DECICE_TIME)
				{
					balance_chassis.postjump_flag = 0;
					if(balance_chassis.prejump_flag == 0)
					{
						Buzzer_Play(Init_sound, 0);
					}
					balance_chassis.prejump_flag = 1;
				}
				else
				{
					balance_chassis.jump_start_cnt++;
				}
			}
		}
		else
		{
			balance_chassis.start_flag = 0;
			balance_chassis.x_filter   = 0.0f;
			balance_chassis.x_set      = 0.0f;
			balance_chassis.leg_set    = LEG_LENGTH_INIT;
			balance_chassis.turn_set   = balance_chassis.total_yaw;
			chassis_right_leg.theta_target = 0.0f;
			chassis_left_leg.theta_target  = 0.0f;
			if (balance_chassis.chassis_mode == CHASSIS_DEBUG)
			{
				DJI_Motor_Disable(balance_chassis.wheel_motor[0]);
				DJI_Motor_Disable(balance_chassis.wheel_motor[1]);
				DM_Motor_Start(balance_chassis.joint_motor[0]);
				DM_Motor_Start(balance_chassis.joint_motor[1]);
				DM_Motor_Start(balance_chassis.joint_motor[2]);
				DM_Motor_Start(balance_chassis.joint_motor[3]);
			}
			else if(balance_chassis.chassis_mode == CHASSIS_CALIBRATE)
			{
				DJI_Motor_Disable(balance_chassis.wheel_motor[0]);
				DJI_Motor_Disable(balance_chassis.wheel_motor[1]);
				DM_Motor_Start(balance_chassis.joint_motor[0]);
				DM_Motor_Start(balance_chassis.joint_motor[1]);
				DM_Motor_Start(balance_chassis.joint_motor[2]);
				DM_Motor_Start(balance_chassis.joint_motor[3]);
			}
			else
			{
				DJI_Motor_Disable(balance_chassis.wheel_motor[0]);
				DJI_Motor_Disable(balance_chassis.wheel_motor[1]);
				DM_Motor_Stop(balance_chassis.joint_motor[0]);
				DM_Motor_Stop(balance_chassis.joint_motor[1]);
				DM_Motor_Stop(balance_chassis.joint_motor[2]);
				DM_Motor_Stop(balance_chassis.joint_motor[3]);
			}
		}
	}
}
/***********************************************************底盘控制模式变量与函数***********************************************************/

/***********************************************************底盘控制目标变量与函数***********************************************************/
void Chassis_Reference(void)
{
	chassis_left_leg.v_tar  = 0.0f;
	chassis_left_leg.x_tar  = 0.0f;
	chassis_right_leg.v_tar = 0.0f;
	chassis_right_leg.x_tar = 0.0f;

	if (balance_chassis.chassis_mode == CHASSIS_DEBUG)
	{
		chassis_right_leg.theta_target = 0.0f;
		chassis_left_leg.theta_target  = 0.0f;
		switch (rc_data->toggle.SA)
		{
			case WFLY_SW_UP:
				chassis_left_leg.ik_x_c  = OFFSET_IK_X + 0.03f * (float) rc_data->rc.ch[WFLY_CH_LX] / WFLY_RC_THRESHOLDS;
				chassis_left_leg.ik_y_c  = OFFSET_IK_Y + trans_thresholds((float) rc_data->rc.ch[WFLY_CH_LY], -WFLY_RC_THRESHOLDS, WFLY_RC_THRESHOLDS, LEG_LENGTH_MIN, LEG_LENGTH_MAX);
				chassis_right_leg.ik_x_c = OFFSET_IK_X + 0.03f * (float) rc_data->rc.ch[WFLY_CH_LX] / WFLY_RC_THRESHOLDS;
				chassis_right_leg.ik_y_c = OFFSET_IK_Y + trans_thresholds((float) rc_data->rc.ch[WFLY_CH_LY], -WFLY_RC_THRESHOLDS, WFLY_RC_THRESHOLDS, LEG_LENGTH_MIN, LEG_LENGTH_MAX);
				break;
			case WFLY_SW_MID:
				chassis_right_leg.front_joint_torque = 0.0f;
				chassis_right_leg.back_joint_torque  = 0.0f;
				chassis_left_leg.front_joint_torque  = 0.0f;
				chassis_left_leg.back_joint_torque   = 0.0f;
				break;
			case WFLY_SW_DOWN:
				chassis_right_leg.front_joint_torque = 0.0f;
				chassis_right_leg.back_joint_torque  = 0.0f;
				chassis_left_leg.front_joint_torque  = 0.0f;
				chassis_left_leg.back_joint_torque   = 0.0f;
				break;
			default:

				break;
		}
	}
	else if (balance_chassis.chassis_mode == CHASSIS_CALIBRATE)
	{
		chassis_right_leg.theta_target = 0.0f;
		chassis_left_leg.theta_target  = 0.0f;
		switch (rc_data->toggle.SA)
		{
			case WFLY_SW_UP:
				//torque在此暂时为速度目标值
				chassis_right_leg.front_joint_torque = -5.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS; // balance_chassis_right_leg.front_joint_torque
				chassis_right_leg.back_joint_torque  = 5.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				chassis_left_leg.front_joint_torque  = -5.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				chassis_left_leg.back_joint_torque   = 5.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				break;
			case WFLY_SW_MID:
			    chassis_right_leg.front_joint_torque = 0.0f;
				chassis_right_leg.back_joint_torque  = 0.0f;
				chassis_left_leg.front_joint_torque  = 0.0f;
				chassis_left_leg.back_joint_torque   = 0.0f;
				break;
			case WFLY_SW_DOWN:
				chassis_right_leg.front_joint_torque = 0.0f;
				chassis_right_leg.back_joint_torque  = 0.0f;
				chassis_left_leg.front_joint_torque  = 0.0f;
				chassis_left_leg.back_joint_torque   = 0.0f;
				// chassis_right_leg.front_joint_torque = -6.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS; // balance_chassis_right_leg.front_joint_torque
				// chassis_right_leg.back_joint_torque  = -6.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				// chassis_left_leg.front_joint_torque  = 6.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				// chassis_left_leg.back_joint_torque   = 6.0f * (float) rc_data->rc.ch[WFLY_CH_RY] / WFLY_RC_THRESHOLDS;
				break;
			default:

				break;
		}
	}
	else
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

				if (rc_data->toggle.SC == WFLY_SW_DOWN)
				{
					balance_chassis.leg_set = LEG_LENGTH_INIT;
				}

				if((user_abs(balance_chassis.leg_set - chassis_left_leg.L0) < 0.025f) || user_abs(balance_chassis.leg_set - chassis_right_leg.L0) < 0.025f)
				{
					balance_chassis.leg_length_change_flag = 0;
				}
				else
				{
					balance_chassis.leg_length_change_flag = 1;
				}

				break;

			case WFLY_SW_MID:
				balance_chassis.leg_length_change_flag = 0;

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
				DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[0]);
				DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[1]);
				DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[2]);
				DM_Motor_Set_Zeropoint(balance_chassis.joint_motor[3]);
				break;
			default:
				break;
		}
	}

#if WHEEL_POLARITY_TEST	
	chassis_left_leg.v_tar  = balance_chassis.v_set;
	chassis_left_leg.x_tar  = balance_chassis.x_set;
	chassis_right_leg.v_tar = -balance_chassis.v_set;
	chassis_right_leg.x_tar = -balance_chassis.x_set;
#else
	right_leg_v_target_ramp->real_value = chassis_right_leg.v;
	left_leg_v_target_ramp->real_value = chassis_left_leg.v;
	ramp_calc(right_leg_v_target_ramp, balance_chassis.v_set);
	ramp_calc(left_leg_v_target_ramp, -balance_chassis.v_set);
	chassis_right_leg.v_tar = right_leg_v_target_ramp->plan_value;
	chassis_right_leg.x_tar = balance_chassis.x_set;
	chassis_left_leg.v_tar  = left_leg_v_target_ramp->plan_value;
	chassis_left_leg.x_tar  = -balance_chassis.x_set;
#endif	
}
/***********************************************************底盘控制目标变量与函数***********************************************************/

/***********************************************************底盘控制运算器变量与函数***********************************************************/
static float LQR_K_L[12];
static float LQR_K_R[12];

/**
 * @brief 
 * 
 * @param val 
 * @param min 
 * @param max 
 */
static void Leg_Saturate_Torque(float *val, float min, float max);

/**
 * @brief 
 * 
 */
static void Chassis_Normal_Control(void);

/**
 * @brief 
 * 
 */
static void Chassis_Exception_Control(void);

/**
 * @brief 
 * 
 */
static void Chassis_Zero_Control(void);

/**
 * @brief 
 * 
 */
static void Chassis_Calibrate_Control(void);

/**
 * @brief 
 * 
 */
static void Chassis_Debug_Control(void);

/**
 * @brief 
 * 
 */
void Chassis_Console(void)
{
	switch (balance_chassis.chassis_mode)
	{
		case CHASSIS_EXCEPTION:
			Chassis_Exception_Control( );
		case CHASSIS_ZERO_FORCE:
			Chassis_Zero_Control( );
			break;
		case CHASSIS_STAND_UP:
		case CHASSIS_FLOATING:
		case CHASSIS_CUSHIONING:
		case CHASSIS_FOLLOW_GIMBAL_YAW:
		case CHASSIS_FREE:
		case CHASSIS_SPIN:
		case CHASSIS_AUTO:
			Chassis_Normal_Control( );
			break;
		case CHASSIS_DEBUG:
			Chassis_Debug_Control( );
			break;
		case CHASSIS_CALIBRATE:
			Chassis_Calibrate_Control( );
	}
}
/***********************************************************底盘控制运算器变量与函数***********************************************************/

float up_torque_test = 8.0f;
float down_torque_test = 8.0f;

uint8_t up_cnt = 0;
uint8_t down_cnt = 0;

/***********************************************************底盘最终执行命令变量与函数***********************************************************/
void Chassis_Send_Cmd(void)
{
	static uint32_t chassis_send_cnt = 0;

	chassis_send_cnt++;
	/******************************底盘部分測試*****************************/

	right_wheel_torque_ramp->real_value = ramp_calc(right_wheel_torque_ramp, chassis_right_leg.wheel_torque);
	left_wheel_torque_ramp->real_value = ramp_calc(left_wheel_torque_ramp, chassis_left_leg.wheel_torque);
	balance_chassis.wheel_motor[0]->motor_controller.pid_ref = right_wheel_torque_ramp->real_value;
	balance_chassis.wheel_motor[1]->motor_controller.pid_ref = left_wheel_torque_ramp->real_value;
	DJI_Motor_Control(NULL);

	if ((chassis_send_cnt % 2) == 0) // 500Hz
	{
		if(rc_data->toggle.SA == WFLY_SW_DOWN)//
		{
			DJI_Motor_Disable(NULL);
			DM_Motor_Disable(NULL);
		}
		else
		{
			if (balance_chassis.chassis_mode == CHASSIS_EXCEPTION)
			{
				//异常处理模式下不遵循代码编写规则
				balance_chassis.recover_start_cnt++;
				if(balance_chassis.recover_start_cnt < 1000)
				{
					for (uint8_t i = 0 ; i < 4 ; i++)
					{
						if (balance_chassis.joint_motor[i]->contorl_mode_state != TARCE_STATE)
						{
							balance_chassis.joint_motor[i]->contorl_mode_state = TARCE_STATE;
						}
						if (balance_chassis.joint_motor[i]->receive_data.state == 0)
						{
							DM_Motor_Enable(balance_chassis.joint_motor[i]);
						}
					}
					DM_Motor_Start(balance_chassis.joint_motor[0]);
					DM_Motor_Start(balance_chassis.joint_motor[1]);
					DM_Motor_Start(balance_chassis.joint_motor[2]);
					DM_Motor_Start(balance_chassis.joint_motor[3]);
					if((balance_chassis.recover_start_cnt % 500) == 0)
					{
						Buzzer_Play(Warming_sound, 0);
					}

					//torque在此暂时为速度目标值
					//right_leg
					balance_chassis.joint_motor[0]->motor_controller.pid_ref = chassis_right_leg.front_joint_torque;
					balance_chassis.joint_motor[1]->motor_controller.pid_ref = chassis_right_leg.back_joint_torque;
					//left_leg
					balance_chassis.joint_motor[2]->motor_controller.pid_ref = chassis_left_leg.front_joint_torque;
					balance_chassis.joint_motor[3]->motor_controller.pid_ref = chassis_left_leg.back_joint_torque;

					balance_chassis.joint_motor[0]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[0]->motor_controller.pid_ref, -1.0f * 10.0f , 10.0f );
					balance_chassis.joint_motor[1]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[1]->motor_controller.pid_ref, -1.0f * 10.0f , 10.0f );

					balance_chassis.joint_motor[2]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[2]->motor_controller.pid_ref, -1.0f * 10.0f , 10.0f );
					balance_chassis.joint_motor[3]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[3]->motor_controller.pid_ref, -1.0f * 10.0f , 10.0f );
					
					DM_Motor_Control(NULL);
					DM_Motor_Stop(balance_chassis.joint_motor[0]);
					DM_Motor_Stop(balance_chassis.joint_motor[1]);
					DM_Motor_Stop(balance_chassis.joint_motor[2]);
					DM_Motor_Stop(balance_chassis.joint_motor[3]);
				}
				else
				{
					if(user_abs(balance_chassis.pitch) < 0.4f && user_abs(balance_chassis.roll) < 0.2f)
					{
						balance_chassis.recover_flag = 0;
						balance_chassis.standup_start_cnt = 0;
						balance_chassis.chassis_mode = CHASSIS_ZERO_FORCE;
					}
					else
					{
						if((balance_chassis.recover_start_cnt % 500) == 0)
						{
							Buzzer_Play(Err_sound, 0);
						}
						if(balance_chassis.recover_start_cnt > 2000)
						{
							balance_chassis.recover_start_cnt = 0;
						}
					}
					DJI_Motor_Disable(NULL);
					DM_Motor_Disable(NULL);
				}
			}
			else if (balance_chassis.chassis_mode == CHASSIS_ZERO_FORCE)
			{
				balance_chassis.recover_start_cnt = 0;
				DJI_Motor_Disable(NULL);
				DM_Motor_Disable(NULL);
			}
			else if (balance_chassis.chassis_mode == CHASSIS_CALIBRATE)
			{
				balance_chassis.recover_start_cnt = 0;
				for (uint8_t i = 0 ; i < 4 ; i++)
				{
					if (balance_chassis.joint_motor[i]->contorl_mode_state != TARCE_STATE)
					{
						balance_chassis.joint_motor[i]->contorl_mode_state = TARCE_STATE;
					}
					if (balance_chassis.joint_motor[i]->receive_data.state == 0)
					{
						DM_Motor_Enable(balance_chassis.joint_motor[i]);
					}
				}
				//torque在此暂时为速度目标值
				//right_leg
				balance_chassis.joint_motor[0]->motor_controller.pid_ref = chassis_right_leg.front_joint_torque;
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = chassis_right_leg.back_joint_torque;
				//left_leg
				balance_chassis.joint_motor[2]->motor_controller.pid_ref = chassis_left_leg.front_joint_torque;
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = chassis_left_leg.back_joint_torque;

				balance_chassis.joint_motor[0]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[0]->motor_controller.pid_ref, -1.0f * MAX_TORQUE_DM8009P , MAX_TORQUE_DM8009P );
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[1]->motor_controller.pid_ref, -1.0f * MAX_TORQUE_DM8009P , MAX_TORQUE_DM8009P );

				balance_chassis.joint_motor[2]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[2]->motor_controller.pid_ref, -1.0f * MAX_TORQUE_DM8009P , MAX_TORQUE_DM8009P );
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[3]->motor_controller.pid_ref, -1.0f * MAX_TORQUE_DM8009P , MAX_TORQUE_DM8009P );
			
				DM_Motor_Control(NULL);
			}
			else if (balance_chassis.chassis_mode == CHASSIS_DEBUG)
			{
				balance_chassis.recover_start_cnt = 0;
#if TP_POLARITY_TEST
				for (uint8_t i = 0 ; i < 4 ; i++)
				{
					if (balance_chassis.joint_motor[i]->contorl_mode_state != SINGLE_TORQUE)
					{
						balance_chassis.joint_motor[i]->contorl_mode_state = SINGLE_TORQUE;
					}
					if (balance_chassis.joint_motor[i]->receive_data.state == 0)
					{
						DM_Motor_Enable(balance_chassis.joint_motor[i]);
					}
				}

				//動力學建模時Tp為正時擺杆為順時針旋轉趨勢，而實際電機順時針轉Tp為負，故需要在最終發送時轉換極性，现在已经在之前翻转了一次，所以不改变符号
				//right_leg
				balance_chassis.joint_motor[0]->motor_controller.pid_ref = chassis_right_leg.front_joint_torque;
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = chassis_right_leg.back_joint_torque;
				//left_leg
				balance_chassis.joint_motor[2]->motor_controller.pid_ref = chassis_left_leg.front_joint_torque;
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = chassis_left_leg.back_joint_torque;		
#else
				for (uint8_t i = 0 ; i < 4 ; i++)
				{
					if (balance_chassis.joint_motor[i]->contorl_mode_state != ABSOLUTE_STATE)
					{
						balance_chassis.joint_motor[i]->contorl_mode_state = ABSOLUTE_STATE;
					}
					if (balance_chassis.joint_motor[i]->receive_data.state == 0)
					{
						DM_Motor_Enable(balance_chassis.joint_motor[i]);
					}
				}
				//right_leg
				balance_chassis.joint_motor[0]->motor_controller.pid_ref = chassis_right_leg.ik_phi1 - PI / 2.0f - 1.88106048f;
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = chassis_right_leg.ik_phi4 - PI / 2.0f + 1.88106048f;
				//left_leg
				balance_chassis.joint_motor[2]->motor_controller.pid_ref = chassis_left_leg.ik_phi1 - PI / 2.0f - 1.88106048f;
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = chassis_left_leg.ik_phi4 - PI / 2.0f + 1.88106048f;

				balance_chassis.joint_motor[0]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[0]->motor_controller.pid_ref, MIN_JOINT0_POS_LIMIT, MAX_JOINT0_POS_LIMIT);
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[1]->motor_controller.pid_ref, MIN_JOINT1_POS_LIMIT, MAX_JOINT1_POS_LIMIT);

				balance_chassis.joint_motor[2]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[2]->motor_controller.pid_ref, MIN_JOINT2_POS_LIMIT, MAX_JOINT2_POS_LIMIT);
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = float_constrain(balance_chassis.joint_motor[3]->motor_controller.pid_ref, MIN_JOINT3_POS_LIMIT, MAX_JOINT3_POS_LIMIT);
#endif
				DM_Motor_Control(NULL);
			}
			else if (balance_chassis.chassis_mode == CHASSIS_CUSHIONING || 
						balance_chassis.chassis_mode == CHASSIS_STAND_UP || 
						balance_chassis.chassis_mode == CHASSIS_FREE || 
						balance_chassis.chassis_mode == CHASSIS_AUTO || 
						balance_chassis.chassis_mode == CHASSIS_FLOATING || 
						balance_chassis.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW || 
						balance_chassis.chassis_mode == CHASSIS_SPIN)
			{
				balance_chassis.recover_start_cnt = 0;
				if(balance_chassis.start_flag == 1)
				{
					for (uint8_t i = 0 ; i < 4 ; i++)
					{
						if (balance_chassis.joint_motor[i]->contorl_mode_state != SINGLE_TORQUE)
						{
							balance_chassis.joint_motor[i]->contorl_mode_state = SINGLE_TORQUE;
						}
						if (balance_chassis.joint_motor[i]->receive_data.state == 0)
						{
							DM_Motor_Enable(balance_chassis.joint_motor[i]);
						}
					}
				}
				else
				{
					DM_Motor_Disable(NULL);
				}

				if(balance_chassis.jump_flag == 1)
				{
					for (uint8_t i = 0 ; i < 4 ; i++)
					{
						if (balance_chassis.joint_motor[i]->contorl_mode_state != TARCE_STATE)
						{
							balance_chassis.joint_motor[i]->contorl_mode_state = TARCE_STATE;
						}
					}
					if(balance_chassis.jump_start_cnt > (LEG_JUMP_FLOATING_TIME + up_cnt))
					{
						Buzzer_Play((char*)Note_Freq, 0);
						balance_chassis.prejump_flag = 0;
						balance_chassis.jump_flag = 0;
						balance_chassis.postjump_flag = 1;
					}
					else
					{
						chassis_right_leg.front_joint_torque = -up_torque_test ; // balance_chassis_right_leg.front_joint_torque
						chassis_right_leg.back_joint_torque  = up_torque_test ;
						chassis_left_leg.front_joint_torque  = -up_torque_test ;
						chassis_left_leg.back_joint_torque   = up_torque_test ;	
						balance_chassis.jump_start_cnt++;
					}
				}
				else if(balance_chassis.postjump_flag == 1)
				{
					for (uint8_t i = 0 ; i < 4 ; i++)
					{
						if (balance_chassis.joint_motor[i]->contorl_mode_state != TARCE_STATE)
						{
							balance_chassis.joint_motor[i]->contorl_mode_state = TARCE_STATE;
						}
					}
					if(balance_chassis.jump_start_cnt > (LEG_JUMP_POST_TIME + down_cnt))
					{
						balance_chassis.jump_start_cnt = 0;
						balance_chassis.postjump_flag = 0;
					}
					else
					{
						if(balance_chassis.jump_start_cnt > (LEG_JUMP_POST_TIME + 5))
						{
							chassis_right_leg.front_joint_torque = 0.0f ; // balance_chassis_right_leg.front_joint_torque
							chassis_right_leg.back_joint_torque  = 0.0f ;
							chassis_left_leg.front_joint_torque  = 0.0f ;
							chassis_left_leg.back_joint_torque   = 0.0f ;	
						}
						chassis_right_leg.front_joint_torque = down_torque_test ; // balance_chassis_right_leg.front_joint_torque
						chassis_right_leg.back_joint_torque  = -down_torque_test ;
						chassis_left_leg.front_joint_torque  = down_torque_test ;
						chassis_left_leg.back_joint_torque   = -down_torque_test ;	
						balance_chassis.jump_start_cnt++;
					}
				}

				//動力學建模時Tp為正時擺杆為順時針旋轉趨勢，而實際電機順時針轉Tp為負，故需要在最終發送時轉換極性，现在已经在之前翻转了一次，所以不改变符号
				//right_leg
				balance_chassis.joint_motor[0]->motor_controller.pid_ref = chassis_right_leg.front_joint_torque;
				balance_chassis.joint_motor[1]->motor_controller.pid_ref = chassis_right_leg.back_joint_torque;
				//left_leg
				balance_chassis.joint_motor[2]->motor_controller.pid_ref = chassis_left_leg.front_joint_torque;
				balance_chassis.joint_motor[3]->motor_controller.pid_ref = chassis_left_leg.back_joint_torque;					

				DM_Motor_Control(NULL);
			}
			else
			{
				balance_chassis.recover_start_cnt = 0;
				DM_Motor_Disable(NULL);
			}
		}
	}
	/******************************底盘部分測試*****************************/
}
/***********************************************************底盘最终执行命令变量与函数***********************************************************/


/***********************************************************平衡步兵打滑检测***********************************************************/
static float vaEstimateKF_F[4] = {
	1.0f, 0.001f,
	0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

static float vaEstimateKF_P[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 后验估计协方差初始值

// static float vaEstimateKF_Q[4] = {
// 	0.0001f, 0.0f,
// 	0.0f, 70.0f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

// static float vaEstimateKF_R[4] = {
// 	250.0f, 0.0f,
// 	0.0f, 500.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

static float vaEstimateKF_Q[4] = {
	0.1f, 0.0f,
	0.0f, 0.005f}; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

static float vaEstimateKF_R[4] = {
	150.0f, 0.0f,
	0.0f, 1000.0f}; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

static const float vaEstimateKF_H[4] = {
	1.0f, 0.0f,
	0.0f, 1.0f}; // 设置矩阵H为常量

static float vaEstimateKF_K[4];

#define SPEED_CHISQUARE_KF 0

#if SPEED_CHISQUARE_KF == 1
/* ---------------- 卡方检验相关变量定义 ---------------- */
// 卡方检验阈值：越小越灵敏。
// 对于2维状态，阈值建议 5.0 ~ 15.0。
// 如果发现稍微一动就拒绝更新（波形发直），调大这个值。
static float Speed_ChiSquareTestThreshold = 10.0f;

// 存储计算出来的卡方值（调试用，观察它来调整阈值）
volatile float Speed_ChiSquare_Val = 0.0f;

// 连续错误计数器：防止滤波器永久拒绝更新（假死）
static uint16_t Speed_ErrorCount = 0;

// 临时矩阵数据缓存（利用KF结构体里已有的temp矩阵，防止malloc）
// 不需要额外定义，直接复用 kf->temp_vector 等
/* ---------------------------------------------------- */

/**
 * @brief 自定义的卡尔曼更新函数，带卡方检验 (Chi-Square Test)
 * 用于检测并剔除异常的观测值（如轮子打滑）
 * @param kf 卡尔曼滤波器结构体指针
 */
/**
 * @brief 自定义的卡尔曼更新函数 (V2.0 修复维度Bug版)
 * 修复了之前矩阵乘法维度不匹配导致计算失败的问题
 */
static void Speed_KF_Check_And_Update(KalmanFilter_t *kf)
{
	// ================== 1. 计算 S = H * P' * H^T + R ==================
	// 2x2 * 2x2 = 2x2
	kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
	// 2x2 * 2x2 = 2x2
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
	// S 存放在 temp_matrix1 (为了省一步加法目标内存，先算逆前的准备)
	// 注意：这里需确保 S 矩阵正确累加 R
	kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);

	// ================== 2. 计算 S的逆矩阵 ==================
	// temp_matrix1 = S^-1
	kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

	// ================== 3. 计算残差 r = z - H * x' ==================
	// temp_vector = H * x' (2x1)
	kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);

	// temp_vector1 = z - temp_vector (残差 r) (2x1)
	kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

	// ================== 4. 计算卡方值 (修复点) ==================
	// 目标: ChiSquare = r^T * S^-1 * r

	// Step A: 计算 vec = S^-1 * r
	// S^-1 (2x2) * r (2x1) -> Result (2x1)
	// 【关键修复】使用 temp_vector (2x1) 作为接收容器，而不是 temp_matrix (2x2)
	kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_vector);

	// Step B: 手动点积 (Scalar = r^T * vec)
	float *r_ptr   = kf->temp_vector1.pData;   // 残差 r
	float *vec_ptr = kf->temp_vector.pData;  // S^-1 * r

	// 直接计算 1x1 结果 (r[0]*vec[0] + r[1]*vec[1])
	Speed_ChiSquare_Val = (r_ptr[0] * vec_ptr[0]) + (r_ptr[1] * vec_ptr[1]);

	// ================== 5. 判断逻辑 ==================
	uint8_t update_flag = 1;

	// 保护：如果计算出的卡方值是NaN或负数(数学上不可能，但计算溢出可能)，强制归零
	if (isnan(Speed_ChiSquare_Val) || Speed_ChiSquare_Val < 0)
		Speed_ChiSquare_Val = 0.0f;

	if (Speed_ChiSquare_Val > Speed_ChiSquareTestThreshold)
	{
		Speed_ErrorCount++;
		if (Speed_ErrorCount > 50) // 50ms 连续异常则重置
		{
			update_flag      = 1;
			Speed_ErrorCount = 0;
		}
		else
		{
			update_flag = 0; // 拒绝更新
		}
	}
	else
	{
		Speed_ErrorCount = 0;
		update_flag      = 1;
	}

	// ================== 6. 执行更新 ==================
	if (update_flag)
	{
		// K = P' * H^T * S^-1
		// temp_matrix = P' * H^T
		kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
		// K = temp_matrix * S^-1 (S^-1 在 temp_matrix1)
		kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

		// x = x' + K * r
		// 这里的 r 还在 temp_vector1 中
		// temp_vector = K * r (2x1)
		kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

		// x = x' + temp_vector
		kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);

		kf->SkipEq5 = 0; // 执行P更新
	}
	else
	{
		// 拒绝更新：直接继承预测值
		memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
		memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
		kf->SkipEq5 = 1; // 跳过P更新
	}
}
#else
#endif

static void Slip_Detect(void)
{
	//未验证极性
	float theta_l   = chassis_left_leg.theta;
	float theta_r   = -chassis_right_leg.theta;
	float d_theta_l = chassis_left_leg.d_theta;
	float d_theta_r = -chassis_right_leg.d_theta;
	// float delta_L0_cos = chassis_left_leg.L0 * arm_cos_f32(theta_l) - chassis_right_leg.L0 * arm_cos_f32(theta_r);
	// float delta_L0_sin = chassis_left_leg.L0 * arm_sin_f32(theta_l) - chassis_right_leg.L0 * arm_sin_f32(theta_r);

	float v_r_leg_x = chassis_right_leg.d_L0 * arm_sin_f32(theta_r) - chassis_left_leg.d_L0 * arm_sin_f32(theta_l) + chassis_right_leg.L0 * arm_cos_f32(theta_r) * d_theta_r - chassis_left_leg.L0 * arm_cos_f32(theta_l) * d_theta_l;

	// float v_r_leg_z = chassis_right_leg.d_L0 * arm_cos_f32(theta_r) - chassis_left_leg.d_L0 * arm_cos_f32(theta_l) - chassis_right_leg.L0 * arm_sin_f32(theta_r) * d_theta_r + chassis_left_leg.L0 * arm_sin_f32(theta_l) * d_theta_l;

	float v_r_delta_wheel         = (chassis_right_leg.wheel_w - chassis_left_leg.wheel_w) * WHEEL_RADIUS;
	balance_chassis.v_delta_wheel = v_r_delta_wheel;
	// float v_spin_x = balance_chassis.d_pitch * delta_L0_cos - balance_chassis.d_yaw * BODY_LENGTH;
	// float v_spin_y = delta_L0_cos * (balance_chassis.d_yaw - balance_chassis.d_roll);
	// float v_spin_z = balance_chassis.d_roll * BODY_LENGTH - balance_chassis.d_pitch * delta_L0_sin;

	float v_r_delta_leg         = v_r_leg_x;
	balance_chassis.v_delta_leg = v_r_delta_leg;
	// float delta_v_x = v_r_delta_wheel + v_r_leg_x + v_r_leg_z;
	// float delta_v_y = v_spin_y;
	// float delta_v_z = v_r_leg_z + v_spin_z;

	float k                = 3.0f;
	float v_gyro           = balance_chassis.d_yaw * BODY_LENGTH / 2 * k;
	balance_chassis.v_gyro = v_gyro;
	// balance_chassis.delta_v = sqrt(pow(delta_v_x,2) + pow(delta_v_y,2) + pow(delta_v_z,2));

	balance_chassis.delta_v = v_r_delta_leg + v_r_delta_wheel - v_gyro;

	if (user_abs(balance_chassis.delta_v) >= 0.15f)
	{
		if (balance_chassis.slip_start_cnt < 200)
		{
			balance_chassis.slip_start_cnt++;
		}
		else
		{
			balance_chassis.slip_flag = 1;
		}
	}
	else
	{
		balance_chassis.slip_start_cnt  = 0;
		balance_chassis.slip_flag = 0;
	}
}

static void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
	Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
	memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

#if SPEED_CHISQUARE_KF == 1
	// 【新增】注册自定义更新函数
	// User_Func3_f 对应标准库中 SetK 之后的位置，这里我们将替代 SetK 和 xhatUpdate
	EstimateKF->User_Func3_f = Speed_KF_Check_And_Update;

	// 【新增】设置跳过标志位
	// 告诉标准库：不要执行标准的计算增益(Eq3)和状态更新(Eq4)
	// 因为我们在 Speed_KF_Check_And_Update 里自己算了
	EstimateKF->SkipEq3 = 1;
	EstimateKF->SkipEq4 = 1;
	// SkipEq5 (P更新) 的标志位会在我们的自定义函数里动态决定
#else
#endif
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

/*1. Q_acc (过程噪声 - 加速度分量)
 *定义：代表你认为“系统的物理状态发生突变”的可能性有多大。
 *通俗理解：“机器人的变轨能力”。数值小的意义（如 1.0 ~ 5.0）：
 *含义：你告诉滤波器：“这个机器人是个大胖子，惯性很大，速度不可能瞬间改变。”
 *效果：滤波器会非常信任上一时刻的速度。即使传感器数据突然跳变，
 *滤波器也会觉得“不可能变这么快”，从而平滑过渡。
 *副作用：滞后。真实的加速开始时，滤波器反应会慢半拍。
 *针对你（防滑）：必须调小。因为打滑就是“不可能的瞬间加速”，调小 Q 可以让滤波器无视这种突变。
 *数值大的意义（如 50.0+）：
 *含义：你告诉滤波器：“这个机器人像苍蝇一样灵活，速度可以瞬间突变。”
 *效果：滤波器会迅速跟随任何变化，响应极快。
 *副作用：抖动。一点点噪声都会被当成真实的运动。
 *
 *2.R_vel (测量噪声 - 速度/编码器分量)
 *定义：代表你认为“轮子转速传感器（编码器）”有多不靠谱。
 *通俗理解：“对轮子数据的怀疑程度”。数值大的意义（如 1000.0+）：
 *含义：你告诉滤波器：“编码器经常撒谎（打滑、震动），别太信它。”
 *效果：抗干扰/防打滑。当轮子打滑空转，编码器数值飙升时，
 *滤波器会因为 R 很大而降低这次测量的权重（Gain 变小），更多地维持原判。
 *针对你（防滑）：必须调大。这是防打滑最直接的手段。数值小的意义（如 10.0）：
 *含义：你告诉滤波器：“编码器非常精准，它测多少就是多少。”
 *效果：死跟传感器。轮子一打滑，计算出的速度立马跟着飙升，系统瞬间失稳。
 *
 *3. R_acc (测量噪声 - 加速度/IMU分量)
 *定义：代表你认为“加速度计（IMU）”有多吵。
 *通俗理解：“对震动的过滤程度”。数值大的意义（如 2000.0）：
 *含义：机器人在地面跑时震动很大，IMU 的数据里混杂了很多“路面噪音”。
 *效果：滤波器会忽略高频震动，输出一条平滑的速度曲线。
 *针对你：保持较大值，有助于在防打滑的同时，让速度波形好看、平稳。
 *数值小的意义（如 10.0）：含义：认为 IMU 数据极度纯净。
 *效果：速度曲线会出现很多毛刺（锯齿状），因为每一丁点路面颠簸都被算进速度里了。
 *4.Q_vel
 *物理意义：拒绝“瞬移”物理公式：v_k = v_k-1 + a_k-1 \Delta t
 *含义：它代表速度在没有加速度驱动的情况下，自行发生突变的概率。
 *现实情况：机器人的速度变化一定是由力（即加速度）产生的。
 *速度不会凭空增加或减少。
 *防滑逻辑：如果你把 Q_vel 设大（比如 1.0），
 *你就在告诉滤波器：“这个机器人的速度有时候会莫名其妙地突变，跟加速度无关。”
 *一旦轮子打滑（编码器速度剧变），滤波器会想：“嗯，虽然 IMU 没检测到加速，
 *但既然允许 Q_vel 突变，那这可能是真的。” —— 结果：防滑失败，速度跟着轮子飞了。
 *如果你把 $Q_{vel}$ 设为 0，滤波器会想：“速度的变化必须严格遵守 v + a \Delta t 的公式。
 *如果没有加速度，速度就不该变。” —— 结果：哪怕轮子转得飞快，只要 IMU 没动，滤波器就拒绝更新速度。
 *这正是你要的！
 *数学博弈：逼迫滤波器依赖积分当你把 Q_vel 设为 0，而保留一定的 Q_acc 时，
 *你实际上是在强迫滤波器建立这样的逻辑：“我相信速度的计算主要靠积分（上一刻速度 + 加速度 \ 时间），
 *而不是靠观测。”这会极大地增加系统的惯性，从而滤除掉所有的轮速高频噪声和打滑尖峰。
 */
// float Q_1 = 0.0f, Q_3 = 20.0f, R_1 = 100.0f, R_3 = 1500.0f;
// float Q_1 = 25.0f, Q_3 = 800.0f, R_1 = 2000.0f, R_3 = 0.01f;
// float Q_1 = 0.0001f, Q_3 = 25.0f, R_1 = 250.0f, R_3 = 1000.0f;
// uint32_t speed_kf_test_cnt;
float Q_1 = 0.1f, Q_3 = 0.05f, R_1 = 150.0f, R_3 = 1000.0f;

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
	balance_chassis.theta_err = chassis_right_leg.theta + chassis_left_leg.theta;

	vaEstimateKF_F[1] = dt;
#if WHEEL_POLARITY_TEST
	chassis_right_leg.wheel_w = -balance_chassis.wheel_motor[0]->receive_data.speed_rps * 2.0f * PI + INS.Gyro[0] + chassis_right_leg.d_phi0;
	chassis_left_leg.wheel_w  = -balance_chassis.wheel_motor[1]->receive_data.speed_rps * 2.0f * PI + INS.Gyro[0] + chassis_left_leg.d_phi0;
#else
	chassis_right_leg.wheel_w = balance_chassis.wheel_motor[0]->receive_data.speed_rps * 2.0f * PI - (chassis_right_leg.d_phi0 - INS.Gyro[0]) * 0.6f;
	chassis_left_leg.wheel_w  = -balance_chassis.wheel_motor[1]->receive_data.speed_rps * 2.0f * PI + (chassis_left_leg.d_phi0 - INS.Gyro[0]) * 0.6f;
#endif	
	wheel_z_e_right           = chassis_right_leg.wheel_w;// 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
	wheel_z_e_left            = chassis_left_leg.wheel_w;// 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正

	Slip_Detect( );

	v_right_b_axis = wheel_z_e_right * WHEEL_RADIUS +
	                 (chassis_right_leg.L0 * chassis_right_leg.d_theta * arm_cos_f32(chassis_right_leg.theta) +
	                 chassis_right_leg.d_L0 * arm_sin_f32(chassis_right_leg.theta)) * 0.6f; // 机体b系的速度
	v_left_b_axis = wheel_z_e_left * WHEEL_RADIUS -
	                (chassis_left_leg.L0 * chassis_left_leg.d_theta * arm_cos_f32(chassis_left_leg.theta) -
	                chassis_left_leg.d_L0 * arm_sin_f32(chassis_left_leg.theta)) * 0.6f; // 机体b系的速度

	chassis_right_leg.leg_b_v = v_right_b_axis;
	chassis_left_leg.leg_b_v = v_left_b_axis;

#if WHEEL_POLARITY_TEST
	aver_v = ( v_right_b_axis - v_left_b_axis ) / 2.0f; // 取平均
#else	
	aver_v = ((v_right_b_axis + v_left_b_axis) / 2.0f) * 0.8f + aver_v * 0.2f;
#endif

	/******************************QR矩阵调试*****************************/
//	vaEstimateKF_Q[0] = Q_1;
//	vaEstimateKF_Q[3] = Q_3;
//	vaEstimateKF_R[0] = R_1;
//	vaEstimateKF_R[3] = R_3;

//	memcpy(vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
//	memcpy(vaEstimateKF.R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	/******************************QR矩阵调试*****************************/

#if WHEEL_POLARITY_TEST
	float user_acc = -INS.MotionAccel_b[1];
#else
	float user_acc = INS.MotionAccel_b[1];
#endif
	float user_v   = aver_v;

	// if(fabsf(aver_v) < 0.03f && fabsf(-INS.MotionAccel_b[1]) < 0.15f)
	// {
	// 	user_v = 0.0f;
	// 	user_acc = 0.0f;

	// 	// 【新增关键代码】
	// 	// 当判断物理静止时，直接暴力将滤波器内部状态清零
	// 	// 防止因为之前的异常导致滤波器卡在某个很大的速度上，被卡方检验一直拒绝
	// 	vaEstimateKF.xhat_data[0] = 0.0f; // 速度置0
	// 	vaEstimateKF.xhat_data[1] = 0.0f; // 加速度置0
	// 	// speed_kf_test_cnt ++;
	// }

	xvEstimateKF_Update(&vaEstimateKF, user_acc, user_v, fusion_v_data);

	balance_chassis.v_filter = fusion_v_data[0];
	balance_chassis.a_filter = fusion_v_data[1];

	if (fabsf(balance_chassis.v_filter) < 0.0025f)
	{
		balance_chassis.v_filter = 0.0f;
	}

	if (balance_chassis.start_flag == 1)
	{
		if (balance_chassis.v_set != 0.0f && balance_chassis.observe_flag == 0)
		{
			balance_chassis.nomotion_start_cnt = 0;
			balance_chassis.x_filter           = 0.0f;
		}
		else
		{
			balance_chassis.nomotion_start_cnt++;
			if (balance_chassis.nomotion_start_cnt > balance_chassis.stop_cnt)
			{
				balance_chassis.x_filter +=
						balance_chassis.v_filter * dt;
				if (balance_chassis.observe_flag == 1)
				{
					balance_chassis.v_set = 0.0f;
					balance_chassis.observe_flag = 0;
				}		
				// balance_chassis.x_filter = 0.0f;
			}
			else
			{
				if(balance_chassis.nomotion_start_cnt > 10)
				{
					if(balance_chassis.observe_flag == 0)
					{
						if(user_abs(balance_chassis.v_filter) > 0.5f)
						{
							balance_chassis.stop_cnt = 1000;
						}
						else
						{
							balance_chassis.stop_cnt = 150;
						}
					}
					balance_chassis.observe_flag = 1;
					balance_chassis.v_set = -balance_chassis.v_filter;
				}
				// if(user_abs(balance_chassis.a_filter) > 0.08f)
				// {
					// chassis_right_leg.theta_target = -balance_chassis.a_filter * 0.4f;
					// chassis_left_leg.theta_target  = balance_chassis.a_filter * 0.4f;
				// }
			}
		}
	}
	else
	{
		balance_chassis.nomotion_start_cnt = 0;
	}
}
/***********************************************************平衡步兵打滑检测***********************************************************/

/***********************************************************平衡步兵单腿状态量更新函数***********************************************************/
static void Chassis_Leg_Update(void)
{
	chassis_right_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[0]->receive_data.position + 1.88106048f;
	chassis_right_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[1]->receive_data.position - 1.88106048f;

	chassis_left_leg.phi1 = PI / 2.0f + balance_chassis.joint_motor[2]->receive_data.position + 1.88106048f;
	chassis_left_leg.phi4 = PI / 2.0f + balance_chassis.joint_motor[3]->receive_data.position - 1.88106048f;

	chassis_right_leg.pitch   = (INS.Pitch) * 0.8f + chassis_right_leg.pitch * 0.2f;
	chassis_right_leg.d_pitch = INS.Gyro[0];

	chassis_left_leg.pitch   = (-INS.Pitch) * 0.8f + chassis_left_leg.pitch * 0.2f;
	chassis_left_leg.d_pitch = -INS.Gyro[0];

	balance_chassis.pitch = INS.Pitch;

	static uint32_t leg_dwt = 0;
	static float dt         = 0.0F;

	dt = DWT_GetDeltaT(&leg_dwt);

	VMC_Calc_Base_Data(&chassis_right_leg, &INS, dt);
	VMC_Calc_Base_Data(&chassis_left_leg, &INS, dt);

#if OFF_GROUND_TEST
	DD_W_forward_test = chassis_left_leg.dd_z_wheel;

	chassis_right_leg.dd_z_wheel = Kalman_One_Filter(&dd_w_kf[0],chassis_right_leg.dd_z_wheel);
	chassis_left_leg.dd_z_wheel = Kalman_One_Filter(&dd_w_kf[1],chassis_left_leg.dd_z_wheel);
#else
	chassis_right_leg.dd_z_wheel = Kalman_One_Filter(&dd_w_kf[0],chassis_right_leg.dd_z_wheel);
	chassis_left_leg.dd_z_wheel = Kalman_One_Filter(&dd_w_kf[1],chassis_left_leg.dd_z_wheel);
#endif
}
/***********************************************************平衡步兵单腿状态量更新函数***********************************************************/

/***********************************************************平衡步兵力矩强制限幅函数***********************************************************/
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
/***********************************************************平衡步兵力矩强制限幅函数***********************************************************/

/***********************************************************平衡步兵缺陷补偿控制函数***********************************************************/
static void Chassis_Control_Leg(void)
{
	balance_chassis.turn_T = leg_turn_pid.Kp * (balance_chassis.turn_set - balance_chassis.total_yaw) -
	                         leg_turn_pid.Kd * INS.Gyro[2]; // 这样计算更稳一点
	Leg_Saturate_Torque(&balance_chassis.turn_T, -1.0f * leg_turn_pid.output_max, leg_turn_pid.output_max);

	balance_chassis.roll_f0 = leg_roll_position_pid.Kp * (balance_chassis.roll_set - balance_chassis.roll)
		                          - leg_roll_position_pid.Kd * INS.Gyro[1];
	Leg_Saturate_Torque(&balance_chassis.roll_f0,
		                    -1.0f * leg_roll_position_pid.output_max,
		                    leg_roll_position_pid.output_max);

	balance_chassis.leg_tp = Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f); // 防劈叉pid计算

	if(balance_chassis.prejump_flag == 1)
	{
		balance_chassis.leg_set = LEG_LENGTH_INIT;
		if((user_abs(balance_chassis.leg_set - chassis_left_leg.L0) < 0.03f) || user_abs(balance_chassis.leg_set - chassis_right_leg.L0) < 0.03f)
		{
			balance_chassis.leg_length_change_flag = 0;
			if(balance_chassis.jump_start_cnt > LEG_JUMP_PRE_TIME)
			{
				if(balance_chassis.jump_flag == 0)
				{
					Buzzer_Play(Ready_sound, 0);
					balance_chassis.jump_flag = 1;
				}
			}
			else
			{
				balance_chassis.jump_start_cnt++;
			}
		}
		else
		{
			balance_chassis.leg_length_change_flag = 1;
		}
	}

	/******************************補償量測試*****************************/
	// balance_chassis.leg_tp = 0.0f;
	// balance_chassis.roll_f0	= 0.0f;
	// balance_chassis.turn_T = 0.0f;
	/******************************補償量測試*****************************/

	if (chassis_left_leg.off_ground_flag == 1 && chassis_right_leg.off_ground_flag == 1)
	{
		balance_chassis.body_offground_flag = 1;
		balance_chassis.x_filter            = 0.0f;
		balance_chassis.x_set               = 0.0f;
		balance_chassis.v_set               = 0.0f;
		balance_chassis.turn_set            = balance_chassis.total_yaw;
	}
	else
	{		
		balance_chassis.body_offground_flag = 0;		
	}
}
/***********************************************************平衡步兵缺陷补偿控制函数***********************************************************/

/***********************************************************平衡步兵正运动学解算函数***********************************************************/
static void Forward_Kinematics(vmc_leg_t *chassis_leg)
{
	//vmc已完成
	;
}
/***********************************************************平衡步兵正运动学解算函数***********************************************************/

/***********************************************************平衡步兵逆运动学解算函数***********************************************************/
static void Inverse_Kinematics(float x, float y, vmc_leg_t *chassis_leg)
{
	float a = 0.0f, b = 0.0f, c = 0.0f;
	float e = 0.0f, f = 0.0f, g = 0.0f;

	float phi1_1 = 0.0f, phi1_2 = 0.0f;
	float phi4_1 = 0.0f, phi4_2 = 0.0f;

	float phi1 = 0.0f, phi4 = 0.0f;

	a = 2 * x * chassis_leg->l1;
	b = 2 * y * chassis_leg->l1;
	c = x * x + y * y + chassis_leg->l1 * chassis_leg->l1 - chassis_leg->l2 * chassis_leg->l2;

	e = 2 * (x - chassis_left_leg.l5) * chassis_leg->l1;
	f = 2 * y * chassis_leg->l1;
	g = (x - chassis_leg->l5) * (x - chassis_leg->l5) + y * y + chassis_leg->l1 * chassis_leg->l1 - chassis_leg->l2 * chassis_leg->l2;

	phi1_1 = 2 * atan2(b + sqrtf(a * a + b * b - c * c), a + c);
	// phi1_2 = 2 * atan2(b - sqrtf(a * a + b * b - c * c), a + c);

	// phi4_1 = 2 * atan2(f - sqrtf(e * e + f * f - g * g), e + g);
	phi4_2 = 2 * atan2(f + sqrtf(e * e + f * f - g * g), e + g);

	phi1_1 = (phi1_1 < 0) ? (phi1_1 + 2.0f * PI) : phi1_1;
	// phi1_2 = (phi1_2 < 0) ? (phi1_2 + 2.0f * PI) : phi1_2;

	// phi4_1 = (phi4_1 < 0) ? (phi4_1 + 2.0f * PI) : phi4_1;
	phi4_2 = (phi4_2 < 0) ? (phi4_2 + 2.0f * PI) : phi4_2;

	// if (phi1_1 > (PI / 2.0f))
	// {
	phi1 = phi1_1;//
	// }
	// else
	// {
	// 	phi1 = phi1_2;
	// }

	// if (phi4_1 >= 0.0f && phi4_1 < (PI / 2.0f))
	// {
	// 	phi4 = phi4_1;
	// }
	// else
	// {
	phi4 = phi4_2;//
	// }

	if (!(Judge_IF_NAN(phi1) || Judge_IF_NAN(phi4)))
	{
		chassis_leg->ik_phi1 = phi1;
		chassis_leg->ik_phi4 = phi4;
	}

	if (chassis_leg->ik_phi4 > (3 * PI / 2))
	{
		chassis_leg->ik_phi4 = chassis_leg->ik_phi4 - 2 * PI;
	}
}
/***********************************************************平衡步兵逆运动学解算函数***********************************************************/

/***********************************************************平衡步兵DEBUG模式函数（测试极性和VMC可行性与运动学解算可行性）***********************************************************/
static void Chassis_Debug_Control(void)
{
	chassis_right_leg.wheel_torque = 0.0f;
	chassis_left_leg.wheel_torque  = 0.0f;
#if TP_POLARITY_TEST 	
	balance_chassis.leg_tp = Digital_PID_Position(&leg_Tp_pid, balance_chassis.theta_err, 0.0f); // 防劈叉pid计算

	chassis_right_leg.Tp = 0.0f;
	chassis_left_leg.Tp = 0.0f;

	// chassis_right_leg.F0 = chassis_right_leg.F_0_forward;
	// chassis_left_leg.F0 = chassis_left_leg.F_0_forward;

	chassis_right_leg.Tp += balance_chassis.leg_tp;
	chassis_left_leg.Tp += balance_chassis.leg_tp;

#if JOINT_POLARITY_TEST
	chassis_right_leg.Tp = -chassis_right_leg.Tp;
	chassis_left_leg.Tp = -chassis_left_leg.Tp;
#endif

	VMC_Calc_T_Joint(&chassis_right_leg);
	VMC_Calc_T_Joint(&chassis_left_leg);

#else
	//right_leg
	Inverse_Kinematics(chassis_right_leg.ik_x_c, chassis_right_leg.ik_y_c, &chassis_right_leg);
	//left_leg
	Inverse_Kinematics(chassis_left_leg.ik_x_c, chassis_left_leg.ik_y_c, &chassis_left_leg);
#endif	
}
/***********************************************************平衡步兵DEBUG模式函数（测试极性和VMC可行性与运动学解算可行性）***********************************************************/

/***********************************************************平衡步兵CALIBRATE模式函数（测试關節電機可行性）***********************************************************/
static void Chassis_Calibrate_Control(void)
{
	chassis_right_leg.wheel_torque = 0.0f;
	chassis_left_leg.wheel_torque  = 0.0f;
}
/***********************************************************平衡步兵CALIBRATE模式函数（测试關節電機可行性）***********************************************************/

uint8_t touch_test_normal = 10;
uint8_t off_test_normal = 20;
uint8_t off_test_jump = 50;
uint8_t touch_test_jump = 80;

/***********************************************************平衡步兵正常模式函数***********************************************************/
static void Chassis_Normal_Control(void)
{
	static float chassis_err[6] = {0.0f};

	Chassis_Control_Leg();

	if(balance_chassis.chassis_mode == CHASSIS_FREE || balance_chassis.chassis_mode == CHASSIS_AUTO || balance_chassis.chassis_mode == CHASSIS_FLOATING || balance_chassis.chassis_mode == CHASSIS_STAND_UP ) 
	{
		for (int i = 0 ; i < 12 ; i++)
		{
			LQR_K_L[i] = LQR_K_Calc(&Poly_Coefficient_Leg[i][0], chassis_left_leg.L0);
		}

		for (int i = 0 ; i < 12 ; i++)
		{
			LQR_K_R[i] = LQR_K_Calc(&Poly_Coefficient_Leg[i][0], chassis_right_leg.L0);
		}
	}	
 	else if(balance_chassis.chassis_mode == CHASSIS_CUSHIONING)
	{
		for (int i = 0 ; i < 12 ; i++)
		{
			LQR_K_L[i] = LQR_K_Calc(&Poly_Coefficient_STOOL[i][0], chassis_left_leg.L0);
		}

		for (int i = 0 ; i < 12 ; i++)
		{
			LQR_K_R[i] = LQR_K_Calc(&Poly_Coefficient_STOOL[i][0], chassis_right_leg.L0);
		}
	}

	static uint16_t jump_end_cnt = 0;
	if(balance_chassis.jump_start_cnt == 0)
	{
		if(jump_end_cnt > 1000)
		{
			touch_threshold = touch_test_normal;
			off_threshold = off_test_normal;
		}
		else
		{
			jump_end_cnt++;
		}
	}
	else
	{
		jump_end_cnt = 0;
		touch_threshold = touch_test_jump;
		off_threshold = off_test_jump;
	}

	/******************************右腿控制输出量计算*****************************/

	chassis_err[0] = chassis_right_leg.theta_target - chassis_right_leg.theta;
	chassis_err[1] = 0.0f - chassis_right_leg.d_theta;
	chassis_err[2] = chassis_right_leg.x_tar - chassis_right_leg.x;//
	chassis_err[3] = chassis_right_leg.v_tar - chassis_right_leg.v;//
	chassis_err[4] = 0.0f - chassis_right_leg.pitch;
	chassis_err[5] = 0.0f - chassis_right_leg.d_pitch;

	for (uint8_t i = 0 ; i < 6 ; i++)
	{
		chassis_err[i] = float_deadband(chassis_err[i], -0.0001f, 0.0001f);
	}

	/******************************平衡误差测试*****************************/
	chassis_err_right_test[0] = chassis_err[0];
	chassis_err_right_test[1] = chassis_err[1];
	chassis_err_right_test[2] = chassis_err[2];
	chassis_err_right_test[3] = chassis_err[3];
	chassis_err_right_test[4] = chassis_err[4];
	chassis_err_right_test[5] = chassis_err[5];
	/******************************平衡误差测试*****************************/

	balance_chassis.wheel_motor[0]->motor_controller.pid_ref = 0.0f;
	chassis_right_leg.wheel_torque                           = 0.0f;

#if WHEEL_POLARITY_TEST
	chassis_right_leg.wheel_torque += LQR_K_R[0] * chassis_err[0];
	chassis_right_leg.wheel_torque += LQR_K_R[1] * chassis_err[1];
	chassis_right_leg.wheel_torque += LQR_K_R[2] * chassis_err[2];
	chassis_right_leg.wheel_torque += LQR_K_R[3] * chassis_err[3];
	chassis_right_leg.wheel_torque += LQR_K_R[4] * chassis_err[4];
	chassis_right_leg.wheel_torque += LQR_K_R[5] * chassis_err[5];
#else
	for (int i = 0 ; i < 6 ; i++)
	{                                                       
		chassis_right_leg.wheel_torque += LQR_K_R[i] * chassis_err[i];
	}
#endif	

	// 转向环
	chassis_right_leg.wheel_torque += balance_chassis.turn_T;

	chassis_right_leg.Tp = 0.0f;

#if JOINT_POLARITY_TEST
	chassis_right_leg.Tp += LQR_K_R[6] * chassis_err[0];
	chassis_right_leg.Tp += LQR_K_R[7] * chassis_err[1];
	chassis_right_leg.Tp += LQR_K_R[8] * chassis_err[2];
	chassis_right_leg.Tp += LQR_K_R[9] * chassis_err[3];
	chassis_right_leg.Tp += LQR_K_R[10] * chassis_err[4];
	chassis_right_leg.Tp += LQR_K_R[11] * chassis_err[5];
#else
	for (int i = 6 ; i < 12 ; i++)
	{
		chassis_right_leg.Tp += LQR_K_R[i] * chassis_err[i - 6];
	}
#endif

	// 防劈腿
	chassis_right_leg.F_0_forward = MG_FF / arm_cos_f32(chassis_right_leg.theta);

#if LENGTH_TEST
	if(balance_chassis.leg_length_change_flag == 1)
	{
		chassis_right_leg.F0 = Digital_PID_Position(&leg_r_length_position_pid, chassis_right_leg.L0, balance_chassis.leg_set);
		chassis_right_leg.F0 = chassis_right_leg.F_0_forward + Digital_PID_Increment(&leg_r_length_speed_pid, chassis_right_leg.d_L0, chassis_right_leg.F0);
	}
	else
	{
		chassis_right_leg.F0 = chassis_right_leg.F_0_forward + 
								leg_length_pid.Kp * (balance_chassis.leg_set - chassis_right_leg.L0) -
								leg_length_pid.Kd * chassis_right_leg.d_L0;
	}
#else
	// 支持力
	chassis_right_leg.F0 = chassis_right_leg.F_0_forward +
	                       Digital_PID_Position(&leg_r_length_pid, chassis_right_leg.L0, balance_chassis.leg_set);
#endif	

#if OFF_GROUND_TEST
	/******************************虛擬離地測試*****************************/
	chassis_right_leg.T1 = chassis_right_leg.front_joint_torque;
	chassis_right_leg.T2 = chassis_right_leg.back_joint_torque;
	/******************************虛擬離地測試*****************************/
#else
	chassis_right_leg.T1 = balance_chassis.joint_motor[0]->receive_data.torque;
	chassis_right_leg.T2 = balance_chassis.joint_motor[1]->receive_data.torque;
#endif

	Leg_Force_Calc(&chassis_right_leg);

#if OFF_GROUND_TEST
	/******************************支持力滤波測試*****************************/
	F_N_forward_test[0] = chassis_right_leg.F_N;
	/******************************支持力滤波測試*****************************/

	Nlms_Filter(&right_leg_F_N_nlms, chassis_right_leg.F_N);

	chassis_right_leg.F_N = right_leg_F_N_nlms.y;
	// + LowPass_Filter1p_Update( &right_F_N_lpf, chassis_right_leg.F_N );
#else
	Nlms_Filter(&right_leg_F_N_nlms, chassis_right_leg.F_N);

	chassis_right_leg.F_N = right_leg_F_N_nlms.y;//half_MG_real + right_leg_F_N_nlms.y;
#endif	

	chassis_right_leg.off_ground_flag = VMC_FN_Ground_Detection(&chassis_right_leg, 20.0f);

	// F_N_forward_test[0] = Kalman_One_Filter(&leg_FN[0],chassis_right_leg.F_N);

	if (chassis_right_leg.off_ground_flag == 1 && balance_chassis.leg_length_change_flag == 0)
	{ // 离地了
		chassis_right_leg.off_ground_flag                        = 1;
		if(balance_chassis.chassis_mode != CHASSIS_CUSHIONING && balance_chassis.chassis_mode != CHASSIS_STAND_UP)
		{
			chassis_right_leg.wheel_torque = 0.0f;
		}
		chassis_right_leg.Tp                                     = 0.0f;//重置
		chassis_right_leg.Tp += LQR_K_R[6] * chassis_err[0];
		chassis_right_leg.Tp += LQR_K_R[7] * chassis_err[1];
		chassis_right_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
		chassis_right_leg.Tp += balance_chassis.leg_tp;
	}

	chassis_right_leg.F0 -= balance_chassis.roll_f0;

	Leg_Saturate_Torque(&chassis_right_leg.F0, -1.0f * MAX_F0, MAX_F0);

#if JOINT_POLARITY_TEST
	chassis_right_leg.Tp = -chassis_right_leg.Tp;
#endif

	VMC_Calc_T_Joint(&chassis_right_leg);

	Leg_Saturate_Torque(&chassis_right_leg.wheel_torque, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);

	Leg_Saturate_Torque(&chassis_right_leg.back_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	Leg_Saturate_Torque(&chassis_right_leg.front_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);

	/******************************右腿控制输出量计算*****************************/

	/******************************左腿控制输出量计算*****************************/
	chassis_err[0] = chassis_left_leg.theta_target - chassis_left_leg.theta;
	chassis_err[1] = 0.0f - chassis_left_leg.d_theta;
	chassis_err[2] = chassis_left_leg.x_tar - chassis_left_leg.x;
	chassis_err[3] = chassis_left_leg.v_tar - chassis_left_leg.v;
	chassis_err[4] = 0.0f - chassis_left_leg.pitch;
	chassis_err[5] = 0.0f - chassis_left_leg.d_pitch;

	for (uint8_t i = 0 ; i < 6 ; i++)
	{
		chassis_err[i] = float_deadband(chassis_err[i], -0.0001f, 0.0001f);
	}

	/******************************平衡误差测试*****************************/
	chassis_err_left_test[0] = chassis_err[0];
	chassis_err_left_test[1] = chassis_err[1];
	chassis_err_left_test[2] = chassis_err[2];
	chassis_err_left_test[3] = chassis_err[3];
	chassis_err_left_test[4] = chassis_err[4];
	chassis_err_left_test[5] = chassis_err[5];
	/******************************平衡误差测试*****************************/

	balance_chassis.wheel_motor[1]->motor_controller.pid_ref = 0.0f;
	chassis_left_leg.wheel_torque                            = 0.0f;
    
#if WHEEL_POLARITY_TEST	
	chassis_left_leg.wheel_torque += LQR_K_L[0] * chassis_err[0];
	chassis_left_leg.wheel_torque += LQR_K_L[1] * chassis_err[1];
	chassis_left_leg.wheel_torque += LQR_K_L[2] * chassis_err[2];
	chassis_left_leg.wheel_torque += LQR_K_L[3] * chassis_err[3];
	chassis_left_leg.wheel_torque += LQR_K_L[4] * chassis_err[4];
	chassis_left_leg.wheel_torque += LQR_K_L[5] * chassis_err[5];
#else
	for (int i = 0 ; i < 6 ; i++)
	{
		chassis_left_leg.wheel_torque += LQR_K_L[i] * chassis_err[i];
	}
#endif

	// 转向环
	chassis_left_leg.wheel_torque += balance_chassis.turn_T;

	chassis_left_leg.Tp = 0.0f;

#if JOINT_POLARITY_TEST	
	chassis_left_leg.Tp += LQR_K_L[6] * chassis_err[0];
	chassis_left_leg.Tp += LQR_K_L[7] * chassis_err[1];
	chassis_left_leg.Tp += LQR_K_L[8] * chassis_err[2];
	chassis_left_leg.Tp += LQR_K_L[9] * chassis_err[3];
	chassis_left_leg.Tp += LQR_K_L[10] * chassis_err[4];
	chassis_left_leg.Tp += LQR_K_L[11] * chassis_err[5];
#else
	for (int i = 6 ; i < 12 ; i++)
	{
		chassis_left_leg.Tp += LQR_K_L[i] * chassis_err[i - 6];
	}
#endif

	chassis_left_leg.F_0_forward = MG_FF / arm_cos_f32(chassis_left_leg.theta);

#if LENGTH_TEST
	if(balance_chassis.leg_length_change_flag == 1)
	{
		chassis_left_leg.F0 = Digital_PID_Position(&leg_l_length_position_pid, chassis_left_leg.L0, balance_chassis.leg_set);
		chassis_left_leg.F0 = chassis_left_leg.F_0_forward + Digital_PID_Increment(&leg_l_length_speed_pid, chassis_left_leg.d_L0, chassis_left_leg.F0);
	}
	else
	{
		chassis_left_leg.F0 = chassis_left_leg.F_0_forward + 
								leg_length_pid.Kp * (balance_chassis.leg_set - chassis_left_leg.L0) -
								leg_length_pid.Kd * chassis_left_leg.d_L0;
	}						
#else 
	// 支持力
	chassis_left_leg.F0 = chassis_left_leg.F_0_forward +
	                      Digital_PID_Position(&leg_l_length_pid, chassis_left_leg.L0, balance_chassis.leg_set);
#endif	

#if OFF_GROUND_TEST	
	/******************************虛擬離地測試*****************************/
	chassis_left_leg.T1 = chassis_left_leg.front_joint_torque;
	chassis_left_leg.T2 = chassis_left_leg.back_joint_torque;
	/******************************虛擬離地測試*****************************/
#else
	chassis_left_leg.T1 = balance_chassis.joint_motor[2]->receive_data.torque;                         
	chassis_left_leg.T2 = balance_chassis.joint_motor[3]->receive_data.torque;
#endif

	Leg_Force_Calc(&chassis_left_leg);

#if OFF_GROUND_TEST	
	/******************************支持力滤波測試*****************************/
	F_N_forward_test[1] = chassis_left_leg.F_N;
	/******************************支持力滤波測試*****************************/

	Nlms_Filter(&left_leg_F_N_nlms, chassis_left_leg.F_N);

	chassis_left_leg.F_N = left_leg_F_N_nlms.y;
	//+ LowPass_Filter1p_Update( &left_F_N_lpf, chassis_left_leg.F_N );
#else
	Nlms_Filter(&left_leg_F_N_nlms, chassis_left_leg.F_N);

	chassis_left_leg.F_N = left_leg_F_N_nlms.y;//half_MG_real + left_leg_F_N_nlms.y;
#endif

	chassis_left_leg.off_ground_flag = VMC_FN_Ground_Detection(&chassis_left_leg, 20.0f);

	// F_N_forward_test[1] = Kalman_One_Filter(&leg_FN[1],chassis_left_leg.F_N);

	if (chassis_left_leg.off_ground_flag == 1 && balance_chassis.leg_length_change_flag == 0)
	{
		// 离地了
		chassis_left_leg.off_ground_flag                         = 1;
		if(balance_chassis.chassis_mode != CHASSIS_CUSHIONING && balance_chassis.chassis_mode != CHASSIS_STAND_UP)
		{
			chassis_left_leg.wheel_torque = 0.0f;
		}
		chassis_left_leg.Tp                                      = 0.0f;//重置
		chassis_left_leg.Tp += LQR_K_L[6] * chassis_err[0];
		chassis_left_leg.Tp += LQR_K_L[7] * chassis_err[1];
		chassis_left_leg.Tp += balance_chassis.leg_tp;
	}
	else
	{
		chassis_left_leg.Tp += balance_chassis.leg_tp;
	}

	chassis_left_leg.F0 += balance_chassis.roll_f0;

	Leg_Saturate_Torque(&chassis_left_leg.F0, -1.0f * MAX_F0, MAX_F0);

#if JOINT_POLARITY_TEST
	chassis_left_leg.Tp = -chassis_left_leg.Tp;
#endif

	VMC_Calc_T_Joint(&chassis_left_leg);

	Leg_Saturate_Torque(&chassis_left_leg.wheel_torque, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);

	Leg_Saturate_Torque(&chassis_left_leg.front_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);
	Leg_Saturate_Torque(&chassis_left_leg.back_joint_torque, -1.0f * MAX_TORQUE_DM8009P, MAX_TORQUE_DM8009P);

	/******************************左腿控制输出量计算*****************************/
}
/***********************************************************平衡步兵正常模式函数***********************************************************/

static void Chassis_Zero_Control(void)
{
	chassis_right_leg.wheel_torque = 0.0f;
	chassis_left_leg.wheel_torque  = 0.0f;
}

static void Chassis_Exception_Control(void)
{
	chassis_right_leg.wheel_torque = 0.0f;
	chassis_left_leg.wheel_torque  = 0.0f;

	//torque在此暂时为速度目标值，收腿系数
	chassis_right_leg.front_joint_torque = 2.0f ; // balance_chassis_right_leg.front_joint_torque
	chassis_right_leg.back_joint_torque  = -2.0f ;
	chassis_left_leg.front_joint_torque  = 2.0f ;
	chassis_left_leg.back_joint_torque   = -2.0f ;	
}

#endif//（CHASSIS_BALANCE）