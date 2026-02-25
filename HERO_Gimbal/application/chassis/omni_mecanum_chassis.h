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


typedef struct
{

}__attribute__((__packed__))Chassis_CmdTypedef;

extern void Chassis_Init(void);

#endif

#endif