/**
******************************************************************************
 * @file    chassis_task.c
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

#include "chassis_task.h"
#include "chassis.h"

#include "message_center.h"

#include "DM_motor.h"

#include "bsp_dwt.h"

#define CHASSIS_TASK_PERIOD 1 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void Chassis_Publish(void);

__weak void Chassis_Init(void);

__weak void Chassis_Handle_Exception(void);

__weak void Chassis_Set_Mode(void);

__weak void Chassis_Observer(void);

__weak void Chassis_Reference(void);

__weak void Chassis_Console(void);

__weak void Chassis_Send_Cmd(void);

osThreadId_t robot_cmd_task_handel;

static publisher_t *chassis_publisher;
static subscriber_t *chassis_subscriber;

static void Chassis_Task(void *argument);

void Chassis_Task_Init(void)
{
	const osThreadAttr_t attr = {
		.name = "Chassis_Task",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityRealtime4,
	};
	robot_cmd_task_handel = osThreadNew(Chassis_Task, NULL, &attr);

	chassis_publisher  = Publisher_Register("chassis_transmit_feed", sizeof(chassis_behaviour_t));
	chassis_subscriber = Subscriber_Register("chassis_receive_cmd", sizeof(chassis_cmd_t));
}

uint32_t chassis_task_diff;

float total_time;

static void Chassis_Task(void *argument)
{
	Chassis_Publish( );

	uint32_t time = osKernelGetTickCount( );

	osDelay(2);

	for (; ;)
	{
		/******************************底盘测试达妙收发代码*****************************/

		//		static uint32_t chassis_cnt = 0;
		//		chassis_cnt++;
		//		if((chassis_cnt % 1500) == 0) // 100Hz
		//		{
		//			DM_Motor_ENABLE(NULL);
		//		}
		//		else if((chassis_cnt % 1000) == 0)
		//		{
		//			DM_Motor_DISABLE(NULL);
		//		}

		/******************************底盘测试达妙收发代码*****************************/

		/******************************底盘测试运行总时长代码*****************************/
		TIME_ELAPSE(total_time, Chassis_Observer( );
		Chassis_Handle_Exception( );
		Chassis_Set_Mode( );
		Chassis_Reference( );
		Chassis_Console( );
		Chassis_Send_Cmd( );
		)
		;

		/******************************底盘测试运行总时长代码*****************************/
		// // 更新状态量
		// Chassis_Observer( );
		// // 处理异常
		// Chassis_Handle_Exception( );
		// // 设置底盘模式
		// Chassis_Set_Mode( );
		// // 更新目标量
		// Chassis_Reference( );
		// // 计算控制量
		// Chassis_Console( );
		// // 发送控制量
		// Chassis_Send_Cmd( );

		chassis_task_diff = osKernelGetTickCount( ) - time;
		time              = osKernelGetTickCount( );
		osDelayUntil(time + CHASSIS_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

__weak void Chassis_Publish(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Init(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Handle_Exception(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Set_Mode(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Observer(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Reference(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Console(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}

__weak void Chassis_Send_Cmd(void)
{
	/*
	 NOTE : 在其他文件中定义具体内容
	*/
}
