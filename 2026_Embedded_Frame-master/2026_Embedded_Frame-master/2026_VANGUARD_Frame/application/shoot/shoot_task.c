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
#include "shoot.h"

#include "message_center.h"

#define SHOOT_TASK_PERIOD 5 // ms

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
		.priority = (osPriority_t) osPriorityRealtime3,
	};
	shoot_task_handel = osThreadNew(Shoot_Task, NULL, &attr);

	shoot_publisher  = Publisher_Register("shoot_transmit_feed", sizeof(shoot_behaviour_t));
	shoot_subscriber = Subscriber_Register("shoot_receive_cmd", sizeof(shoot_cmd_t));
}

uint32_t shoot_task_diff;

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
}

__weak void Shoot_Publish(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Init(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Handle_Exception(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Set_Mode(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Observer(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Reference(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Console(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Shoot_Send_Cmd(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}
