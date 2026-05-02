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
#include "control_task.h"
#include "chassis.h"

#include "message_center.h"

#include "DM_motor.h"

#include "bsp_dwt.h"

#include "bsp_usart.h"
#include "rs485.h"

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
static void Chassis_Task_Start(void);

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
float chassis_time;

static void Chassis_Task_Start(void)
{
#if RS485_CHA
	HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
#endif
}

static void Chassis_Task(void *argument)
{
	static const control_task_hook_t chassis_steps[] = {
		Chassis_Observer,
		Chassis_Handle_Exception,
		Chassis_Set_Mode,
		Chassis_Reference,
		Chassis_Console,
		Chassis_Send_Cmd,
	};
	static const control_task_config_t chassis_task_config = {
		.publish = Chassis_Publish,
		.start = Chassis_Task_Start,
		.steps = chassis_steps,
		.step_count = (uint8_t)(sizeof(chassis_steps) / sizeof(chassis_steps[0])),
		.period_ms = CHASSIS_TASK_PERIOD,
		.startup_delay_ms = 2,
		.diff_ms = &chassis_task_diff,
#if INCLUDE_uxTaskGetStackHighWaterMark
		.stack_high_water = &chassis_high_water,
#endif
	};

	(void)argument;
	Control_Task_Run(&chassis_task_config);
}

__weak void Chassis_Publish(void)
{
}

__weak void Chassis_Init(void)
{
}

__weak void Chassis_Handle_Exception(void)
{
}

__weak void Chassis_Set_Mode(void)
{
}

__weak void Chassis_Observer(void)
{
}

__weak void Chassis_Reference(void)
{
}

__weak void Chassis_Console(void)
{
}

__weak void Chassis_Send_Cmd(void)
{
}
