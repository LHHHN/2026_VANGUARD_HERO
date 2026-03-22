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

#include "robot_frame_config.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "balance_chassis.h"
#endif

typedef struct
{
	/* data */
}__attribute__((packed)) chassis_behaviour_t;

typedef struct
{
	/* data */
	
}__attribute__((packed)) chassis_cmd_t;

extern void Chassis_Init(void);

#endif /* __CHASSIS_H__ */
