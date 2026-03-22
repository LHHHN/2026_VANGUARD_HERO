/**
* @file balance_chassis.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __BALANCE_CHASSIS_H__
#define __BALANCE_CHASSIS_H__

#include "robot_frame_config.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)

#include <stdint.h>

#include "dm_motor.h"
#include "dji_motor.h"
#include "vmc.h"

#define WHEEL_RADIUS (0.116f / 2)               // 轮子半径

#define BODY_LENGTH 0.3f

#define MAX_F0 112.04f           // 最大前馈力
#define MAX_TORQUE_DM8009P 10.0f // 最大扭矩
#define MAX_TORQUE_DJI3508 4.414f // 最大扭矩

#define MIN_JOINT0_POS_LIMIT -1.7209f   // 最小关节位置限制
#define MAX_JOINT0_POS_LIMIT 0.0f       // 最大关节位置限制
#define MIN_JOINT1_POS_LIMIT 0.0f       // 最小关节位置限制
#define MAX_JOINT1_POS_LIMIT 1.7259f    // 最大关节位置限制

#define MIN_JOINT2_POS_LIMIT -1.7233f   // 最小关节位置限制
#define MAX_JOINT2_POS_LIMIT 0.0f       // 最大关节位置限制
#define MIN_JOINT3_POS_LIMIT 0.0f       // 最小关节位置限制
#define MAX_JOINT3_POS_LIMIT 1.7317f    // 最大关节位置限制

#define BODY_HEIGHT_INIT  0.200f
#define BODY_HEIGHT_MAX  0.383f
#define BODY_HEIGHT_MIN  0.142f

#define HALF_BODY_WHEEL_LENGTH 0.22f

#define JUMP_TEST 0

#if JUMP_TEST
#define LEG_JUMP_DECICE_TIME 50 //1000
#define LEG_JUMP_PRE_TIME LEG_JUMP_DECICE_TIME + 1000 
#else
#define LEG_JUMP_DECICE_TIME 1000 //1000
#define LEG_JUMP_PRE_TIME LEG_JUMP_DECICE_TIME + 1000 
#endif
#define LEG_JUMP_FLOATING_TIME LEG_JUMP_PRE_TIME + 80
#define LEG_JUMP_POST_TIME LEG_JUMP_FLOATING_TIME + 60

typedef enum 
{
    CHASSIS_EXCEPTION,   // 底盘异常处理阶段
    CHASSIS_ZERO_FORCE,  // 底盘关闭无力，所有控制量置0
    CHASSIS_CALIBRATE,   // 底盘校准
    CHASSIS_DEBUG,       // 调试模式
    CHASSIS_CUSHIONING,  // 底盘缓冲状态
    CHASSIS_STAND_UP,    // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_AUTO,        // 底盘自动模式
    CHASSIS_FREE,        // 底盘不跟随云台
    CHASSIS_FLOATING,    // 底盘悬空状态
    CHASSIS_SPIN,        // 底盘小陀螺模式
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
}chassis_mode_e;

typedef struct
{
    DM_motor_instance_t *joint_motor[4];
    DJI_motor_instance_t *wheel_motor[2];

    float v_set;    // 期望速度，单位是m/s
    float x_set;    // 期望位置，单位是m
	float wz_set;	// 期望角速度，单位是rad/s

    float turn_set; // 期望yaw轴弧度
    float roll_set; // 期望roll轴弧度
    float height_set;// 期望机体高度，单位是m

    float a_filter; // 滤波后的车体加速度，单位是m/s^2
    float v_filter; // 滤波后的车体速度，单位是m/s
    float x_filter; // 滤波后的车体位置，单位是m
    
    float roll;
    float pitch;
    float yaw;

    float body_height;
    float delta_height_set;
    float jump_height_set;
    
    float d_roll;
    float d_pitch;
    float d_yaw;

    float total_yaw;

    float delta_v;
    float theta_err; // 两腿夹角误差
    float tan_slope_rad; // 坡角

    float turn_T;  // yaw轴补偿
    float roll_f0; // roll轴补偿
    float leg_tp;  // 防劈叉补偿
    float roll_f0_forward;

    float v_delta_wheel;
    float v_delta_leg;
    float v_gyro;

    chassis_mode_e chassis_mode;

    uint8_t start_flag;                 // 启动标志
    uint8_t body_offground_flag;        // 车体离地标志
    uint8_t leg_length_change_flag;     // 腿长改变标志

    uint8_t steps_flag;
    uint8_t steps_cnt;

    uint8_t slip_flag;

    uint8_t prejump_flag;   // 跳跃前标志
    uint8_t jump_flag;      // 跳跃标志
    uint8_t postjump_flag;  // 跳跃后标志

    uint8_t recover_flag; // 一种情况下的倒地自起标志

    uint8_t observe_flag; // 观测标志

    uint16_t stop_cnt;
    uint32_t slip_start_cnt;
    uint32_t jump_start_cnt;
    uint32_t recover_start_cnt;
	uint32_t nomotion_start_cnt;
	uint32_t standup_start_cnt;
} balance_chassis_t;

extern float offset_theta_set;
extern float offset_pitch_set;
extern float offset_x_filter;

extern vmc_leg_t chassis_right_leg;
extern vmc_leg_t chassis_left_leg;

extern balance_chassis_t balance_chassis;

extern const float MG;// 质量*重力加速度*高度

extern uint8_t right_off_ground_flag;
extern uint8_t left_off_ground_flag;

extern float F_0_forward_test[2];
extern float F_N_forward_test[2];
extern float DD_W_forward_test;
extern float chassis_err_left_test[6];
extern float chassis_err_right_test[6];

extern void Leg_Pensation_Init(void);
 
#endif

#endif /* __BALANCE_CHASSIS_H__ */