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

#define WHEEL_RADIUS (0.20f / 2)               // 轮子半径

#define MAX_F0 64.0f           // 最大前馈力
#define MAX_TORQUE_DM8009P 15.0f // 最大扭矩
#define MAX_TORQUE_DJI3508 5.8f // 最大扭矩

#define LEG_LENGTH_INIT  0.162f
#define LEG_LENGTH_MAX  0.333f
#define LEG_LENGTH_MIN  0.139f

typedef enum 
{
    CHASSIS_OFF,         // 底盘关闭
    CHASSIS_ZERO_FORCE,  // 底盘无力，所有控制量置0
    CHASSIS_STAND_UP,    // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_CALIBRATE,   // 底盘校准
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_FLOATING,    // 底盘悬空状态
    CHASSIS_CUSHIONING,  // 底盘缓冲状态
    CHASSIS_FREE,        // 底盘不跟随云台
    CHASSIS_SPIN,        // 底盘小陀螺模式
    CHASSIS_AUTO,        // 底盘自动模式
    CHASSIS_DEBUG        // 调试模式
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
    float leg_set;  // 期望腿长，单位是m
    float last_leg_set;

    float v_filter; // 滤波后的车体速度，单位是m/s
    float x_filter; // 滤波后的车体位置，单位是m

    float pitch_r;
    float pitch_gyro_r;

    float pitch_l;
    float pitch_gyro_l;
    
    float roll;
    float total_yaw;
    float theta_err; // 两腿夹角误差

    float turn_T;  // yaw轴补偿
    float roll_f0; // roll轴补偿
    float leg_tp;  // 防劈叉补偿

    chassis_mode_e chassis_mode;

    uint8_t start_flag;          // 启动标志
    uint8_t body_offground_flag; // 车体离地标志
    uint8_t leg_length_change_flag;     // 腿长改变标志
	uint8_t leg_control_flag;

    uint8_t jump_flag;  // 跳跃标志
    uint8_t jump_flag2; // 跳跃标志

    uint8_t prejump_flag; // 预跳跃标志
    uint8_t recover_flag; // 一种情况下的倒地自起标志

    uint8_t observe_flag; // 观测标志

	uint32_t nomotion_start_cnt;
	uint32_t standup_start_cnt;
} balance_chassis_t;

extern float offset_theta_set;
extern float offset_pitch_set;
extern float offset_x_filter;
extern float offset_v_set;

extern float leg_init_set;
extern float leg_max_set;
extern float leg_min_set;

extern vmc_leg_t chassis_right_leg;
extern vmc_leg_t chassis_left_leg;

extern balance_chassis_t balance_chassis;

extern const float MG;// 质量*重力加速度*高度

extern uint8_t right_off_ground_flag;
extern uint8_t left_off_ground_flag;

// 保持腿长
extern float leg_length_pid_kp;
extern float leg_length_pid_ki;
extern float leg_length_pid_kd;
extern float leg_length_pid_max_out;

extern const uint8_t ps_flag;

extern void Leg_Pensation_Init(void);

#endif

#endif /* __BALANCE_CHASSIS_H__ */