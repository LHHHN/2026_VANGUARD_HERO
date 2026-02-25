/**
* @file shoot.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include <stdint.h>

#include "dji_motor.h"
#include "dm_motor.h"

#define SHOOT_SINGLE_ANGLE_DEF 360 / 6 // 单次发射角度

typedef struct
{
	/* data */
}__attribute__((packed)) shoot_behaviour_t;

typedef enum
{
	SHOOT_DISABLE = 0,
	SHOOT_STOP = 1,
	SHOOT_ENABLE  = 2,
}shoot_mode_e;

typedef struct
{
	/* data */
	shoot_mode_e mode;
	uint8_t fire_allow;
	float shoot_angle_def;
}__attribute__((packed)) shoot_cmd_t;

extern DJI_motor_instance_t *shoot_m3508_motor[3];

void Shoot_Init(void);
	
#endif /* __SHOOT_H__ */
