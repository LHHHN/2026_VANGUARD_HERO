/**
******************************************************************************
* @file    robot_frame_init.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include "robot_frame_init.h"

#include "robot_frame_config.h"

// TODO(GUATAI):daemon_task是否需要看个人理解，可以把daemon_task去掉，
// 替换成其他功能任务，任务最好不要超过8个
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "daemon_task.h"
#include "INS_task.h"
#include "procotol_task.h"
#include "power_control_task.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "daemon.h"
#include "INS.h"
#include "procotol.h"

#include "referee_task.h"

#include "bmi088.h"
#include "ws2812.h"
#include "buzzer.h"
#include "remote_control.h"
#include "wfly_control.h"
#include "referee.h"
#include "vofa.h"

#include "bsp_dwt.h"
#include "bsp_usart.h"

#include "balance_chassis.h"

#include "BMI088driver.h"

float init_time;

#if (REMOTE_TYPE == WFLY_SBUS)

wfly_t *rc_data;

#elif (REMOTE_TYPE == DT7)

RC_ctrl_t *rc_data;

#endif

// Referee_InfoTypedef *referee_data;

static void Frame_MCU_Init(void)
{
	__disable_irq();

	DWT_Init(480);

	__enable_irq();
}

static void Frame_Device_Init(void)
{
	/******************************module模块初始化*****************************/

	Buzzer_Register();

	ws2812_instance = WS2812_Register(&ws2812_config);

	// BMI088_Init(&hspi2,0);
	bmi088_h7 = BMI088_Register(&bmi088_init_h7);

#if (REMOTE_TYPE == DT7)
	rc_data = Remote_Control_Init(&huart5);
#elif (REMOTE_TYPE == WFLY_SBUS)
	rc_data = WFLY_SBUS_Register();
#endif

	referee_outer_info = UI_Task_Init(&huart1, &referee_outer_interactive);
	// referee_data = Referee_Init(&huart1);

	// VOFA_Register( );

	/******************************module模块初始化*****************************/

	/******************************application组件初始化*****************************/

	Chassis_Init();

	Gimbal_Init();

	Shoot_Init();
	/******************************application组件初始化*****************************/
}

static void Frame_Task_Init(void)
{
	/******************************测试任务初始化时间*****************************/

	// TIME_ELAPSE(init_time, Buzzer_Task_Init( );
	// Chassis_Task_Init( );
	// Gimbal_Task_Init( );
	// Shoot_Task_Init( );
	// INS_Task_Init( );
	// Procotol_Task_Init();
	// )
	// ;

	/******************************测试任务初始化时间*****************************/

	/******************************任务初始化*****************************/

	Buzzer_Task_Init();

	Daemon_Task_Init();

	Chassis_Task_Init();

	Gimbal_Task_Init();

	Shoot_Task_Init();

	INS_Task_Init();

	Procotol_Task_Init();

	// Power_Control_Task_Init();

	/******************************任务初始化*****************************/
}

void Robot_Frame_Init(void)
{
	// __disable_irq( );

	Frame_MCU_Init();

	Frame_Device_Init();

	Frame_Task_Init();

	// __enable_irq( );
}
