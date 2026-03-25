/**
 * @file omni_mecanum_chassis.h
 * @author lHHHn
 * @brief
 * @version 0.1
 * @date 2025-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __OMNI_MECANUM_CHASSIS_H__
#define __OMNI_MECANUM_CHASSIS_H__

#include "robot_frame_config.h"

#if (CHASSIS_TYPE == CHASSIS_OMNI_MECANUM)

#include <stdint.h>

#include "dm_motor.h"
#include "dji_motor.h"
#include "vmc.h"

#include "rs485.h"

#define LENGTH 0.405f         // 轴距（wheelbase） 即前后轮轴间距离
#define MECANUM_WIDTH 0.4238f // 麦轮轮距（track） 即左右驱动轮中心距离
#define OMNI_WIDTH 0.435f     // 全向轮轮距（track） 即左右驱动轮中心距离
#define WHEEL_RADIUS 0.077f   // 驱动轮半径（diameter）

#define SPIN_SET 3.0f 

/*遥控器参数*/
#define REMOTE_X_SEN 0.001 // 660 ~ -660
#define REMOTE_Y_SEN 0.0005
#define REMOTE_OMEGA_Z_SEN 0.00001f // 6.6

/*键鼠参数*/
#define KEY_X_SEN 1.5
#define KEY_Y_SEN 1
#define KEY_OMEGA_Z_SEN 0.005f // 6.6

// #define REMOTE_YAW_SEN 0.000015f
#define REMOTE_PITCH_SEN 0.000002f

#define FOLLOW_OMEGA_Z 3.67f // 底盘跟随0点

typedef enum
{
    CHASSIS_DISABLE = 0, // 底盘失能
    CHASSIS_STOP_C = 1,  // 底盘不动
    CHASSIS_UPSTEP = 2,  // 底盘上台阶
    CHASSIS_FOLLOW = 3,  // 底盘跟随
    CHASSIS_SPIN = 4,    // 底盘小陀螺
} chassis_mode_e;

typedef struct
{
    uint8_t key_EN_state;
    uint8_t chassis_EN_state;
} chassis_key_state_e;

typedef enum
{
    LEG_NORMAL = 0,  // 正常行驶
    LEG_LIFTOFF = 1, // 离地
} leg_state_e;

typedef struct
{
    float vx;
    float vy;

    float omega_z;      // 底盘小陀螺时的角速度(rad/s)
    float omega_follow; // 底盘跟随时的角速度  (rad/s)

    float leg_angle; // 前腿电机角度
    float v_track;   // 履带速度

    float wheel_target[4]; // 四轮目标速度(rad/s)

    chassis_mode_e mode;
    chassis_key_state_e key_state;

    // leg_mode_e leg_mode;
    leg_state_e leg_state;

} __attribute__((__packed__)) Chassis_CmdTypedef;

extern DJI_motor_instance_t *chassis_m3508[4];

extern Chassis_CmdTypedef chassis_cmd;

extern void Chassis_Init(void);

#endif

#endif