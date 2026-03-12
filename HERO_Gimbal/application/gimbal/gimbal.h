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
#define PITCH_VELOCITY_MAX 2.0f
#define PTICH_MAX_ANGLE 0.796f // 云台俯仰最大角度，单位：弧度
#define PTICH_MIN_ANGLE -0.36f  // 云台俯仰最小角度，单位：弧度

typedef struct
{
	/* data */
}__attribute__((packed)) gimbal_behaviour_t;

typedef enum
{
	GIMBAL_DISABLE = 0,
	GIMBAL_STOP = 1,
	GIMBAL_ENABLE = 2,
	GIMBAL_AUTO_AIMING = 3,
}gimbal_mode_e;

typedef struct
{
	/* data */
	gimbal_mode_e mode;
	float pitch_diff;
	float pitch_v;
	float pitch_target;          //PITCH目标
}__attribute__((packed)) gimbal_cmd_t;

void Gimbal_Init(void);


#endif /* __GIMBAL_H__ */
