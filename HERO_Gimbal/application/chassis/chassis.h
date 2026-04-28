/**
* @file chassis.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>

#define SPIN_SET 3.0f
#define SPIN_SET_PRO 5.0f

/*遥控器参数*/
#define REMOTE_X_SEN 0.007 // 660 ~ -660
#define REMOTE_Y_SEN 0.005
#define REMOTE_OMEGA_Z_SEN 0.001f // 6.6

/*键鼠参数*/
#define KEY_X_SEN 1.5
#define KEY_Y_SEN 1.3
#define KEY_X_SEN_PRO 2.25
#define KEY_Y_SEN_PRO 1.7
#define KEY_OMEGA_Z_SEN 0.005f // 6.6

// #define REMOTE_YAW_SEN 0.000015f
//#define REMOTE_PITCH_SEN 0.000002f

typedef enum
{
    CHASSIS_DISABLE = 0, // 底盘失能
    CHASSIS_STOP = 1,    // 底盘不动
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
	/* data */
}__attribute__((packed)) chassis_behaviour_t;

typedef struct
{
	/* data */
	float target_vx;
    float target_vy;
    float target_wz;      // 底盘小陀螺时的角速度(rad/s)
    float target_leg_angle;

    chassis_mode_e mode;
    chassis_key_state_e key_state;
    leg_state_e leg_state;
}__attribute__((packed)) chassis_cmd_t;

extern chassis_cmd_t chassis_cmd;

extern void Chassis_Init(void);

#endif /* __CHASSIS_H__ */
