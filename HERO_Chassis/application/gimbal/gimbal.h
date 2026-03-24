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

#include "LK_motor.h"

#define REMOTE_YAW_SEN 0.000005f

#define GIMBAL_FEEDBACK_TYPE IMU_FEEDBACK
//#define GIMBAL_FEEDBACK_TYPE MOTOR_FEEDBACK_P

typedef struct
{
	/* data */
}__attribute__((packed)) gimbal_behaviour_t;

typedef enum
{
	GIMBAL_DISABLE = 0,
	GIMBAL_ENABLE  = 1,
	GIMBAL_STOP	= 2,
	GIMBAL_AUTO_AIMING = 3,
	GIMBAL_ZERO = 4,
}gimbal_mode_e;

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
	gimbal_key_state_e key_state ;

	float yaw_diff;
	float v_yaw;          //YAW_角速度
}__attribute__((packed)) gimbal_cmd_t;

extern float fusion_v_data[2];
extern float target_yaw;
extern float target_yaw_pro;

void Gimbal_Init(void);

#endif /* __GIMBAL_H__ */
