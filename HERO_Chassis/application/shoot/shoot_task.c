/**
******************************************************************************
 * @file    shoot_task.c
 * @brief
 * @author
******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "shoot_task.h"
#include "control_task.h"
#include "shoot.h"

#include "bsp_dwt.h"

#include "message_center.h"

#define SHOOT_TASK_PERIOD 1 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t shoot_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void Shoot_Publish(void);
__weak void Shoot_Init(void);
__weak void Shoot_Handle_Exception(void);
__weak void Shoot_Set_Mode(void);
__weak void Shoot_Observer(void);
__weak void Shoot_Reference(void);
__weak void Shoot_Console(void);
__weak void Shoot_Send_Cmd(void);

osThreadId_t shoot_task_handel;

static publisher_t *shoot_publisher;
static subscriber_t *shoot_subscriber;

static void Shoot_Task(void *argument);

void Shoot_Task_Init(void)
{
	const osThreadAttr_t attr = {
		.name = "Shoot_Task",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityRealtime4,
	};
	shoot_task_handel = osThreadNew(Shoot_Task, NULL, &attr);

	shoot_publisher  = Publisher_Register("shoot_transmit_feed", sizeof(shoot_behaviour_t));
	shoot_subscriber = Subscriber_Register("shoot_receive_cmd", sizeof(shoot_cmd_t));
}

uint32_t shoot_task_diff;
float shoot_time;

static void Shoot_Task(void *argument)
{
	Shoot_Publish( );

	uint32_t time = osKernelGetTickCount( );

	osDelay(2);

	for (; ;)
	{
		// 更新状态量
		Shoot_Observer( );
		// 处理异常
		Shoot_Handle_Exception( );
		// 设置射击模式
		Shoot_Set_Mode( );
		// 设置目标量
		Shoot_Reference( );
		// 计算控制量
		Shoot_Console( );
		// 发送控制量
		Shoot_Send_Cmd( );

		shoot_task_diff = osKernelGetTickCount( ) - time;
		time            = osKernelGetTickCount( );
		osDelayUntil(time + SHOOT_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
		shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
// 	static const control_task_hook_t shoot_steps[] = {
// 		Shoot_Observer,
// 		Shoot_Handle_Exception,
// 		Shoot_Set_Mode,
// 		Shoot_Reference,
// 		Shoot_Console,
// 		Shoot_Send_Cmd,
// 	};
// 	static const control_task_config_t shoot_task_config = {
// 		.publish = Shoot_Publish,
// 		.start = NULL,
// 		.steps = shoot_steps,
// 		.step_count = (uint8_t)(sizeof(shoot_steps) / sizeof(shoot_steps[0])),
// 		.period_ms = SHOOT_TASK_PERIOD,
// 		.startup_delay_ms = 2,
// 		.diff_ms = &shoot_task_diff,
// #if INCLUDE_uxTaskGetStackHighWaterMark
// 		.stack_high_water = &shoot_high_water,
// #endif
// 	};

// 	(void)argument;
// 	Control_Task_Run(&shoot_task_config);
}

__weak void Shoot_Publish(void)
{
}

__weak void Shoot_Init(void)
{
}

__weak void Shoot_Handle_Exception(void)
{
}

__weak void Shoot_Set_Mode(void)
{
}

__weak void Shoot_Observer(void)
{
}

__weak void Shoot_Reference(void)
{
}

__weak void Shoot_Console(void)
{
}

__weak void Shoot_Send_Cmd(void)
{
}
