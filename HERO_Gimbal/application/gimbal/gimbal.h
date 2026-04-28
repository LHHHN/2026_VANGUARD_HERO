/**
 * @file gimbal.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include <stdint.h>

#include "DM_motor.h"

#define REMOTE_PITCH_SEN 0.000005f
#define KEY_PITCH_SEN 0.000008f
#define REMOTE_YAW_SEN 0.000005f
#define KEY_YAW_SEN 0.00001f
#define PITCH_VELOCITY_MAX 2.0F
#define PTICH_MAX_ANGLE 0.750f // 云台俯仰最大角度，单位：弧度
#define PTICH_MIN_ANGLE -0.35f // 云台俯仰最小角度，单位：弧度

typedef struct
{
	/* data */
} __attribute__((packed)) gimbal_behaviour_t;

typedef enum
{
	GIMBAL_DISABLE = 0,		/* 失能不管模式 */
	GIMBAL_ENABLE = 1,		/* 使能转动模式 */
	GIMBAL_AUTO_AIMING = 2, /* 自瞄模式 */
	GIMBAL_ZERO = 3,		/* 回中模式 */
} gimbal_mode_e;

typedef struct 
{
	uint8_t key_EN_state;
    uint8_t gimbal_EN_state;
	/* data */
}gimbal_key_state_e;

typedef struct
{
	/* data */
	gimbal_mode_e mode;
	gimbal_key_state_e key_state;
	
	float pitch_diff;
	float pitch_v;
	float pitch_target; // PITCH目标
	float yaw_v;
	float yaw_target;
} __attribute__((packed)) gimbal_cmd_t;

extern gimbal_cmd_t gimbal_cmd;

void Gimbal_Init(void);

#endif /* __GIMBAL_H__ */
