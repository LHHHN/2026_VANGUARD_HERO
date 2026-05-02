/**
 * @file control_task.h
 * @brief Shared runner for periodic control tasks.
 */

#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

typedef void (*control_task_hook_t)(void);

typedef struct
{
	control_task_hook_t publish;
	control_task_hook_t start;
	const control_task_hook_t *steps;
	uint8_t step_count;
	uint32_t period_ms;
	uint32_t startup_delay_ms;
	uint32_t *diff_ms;
#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t *stack_high_water;
#endif
} control_task_config_t;

static inline void Control_Task_Call(control_task_hook_t hook)
{
	if (hook != NULL)
	{
		hook();
	}
}

static inline void Control_Task_Run(const control_task_config_t *config)
{
	uint8_t i;
	uint32_t time;

	if (config == NULL)
	{
		for (;;)
		{
			osDelay(1000);
		}
	}

	Control_Task_Call(config->publish);
	Control_Task_Call(config->start);

	time = osKernelGetTickCount();

	if (config->startup_delay_ms > 0U)
	{
		osDelay(config->startup_delay_ms);
	}

	for (;;)
	{
		for (i = 0U; i < config->step_count; i++)
		{
			Control_Task_Call(config->steps[i]);
		}

		if (config->diff_ms != NULL)
		{
			*config->diff_ms = osKernelGetTickCount() - time;
		}

		time = osKernelGetTickCount();
		osDelayUntil(time + config->period_ms);

#if INCLUDE_uxTaskGetStackHighWaterMark
		if (config->stack_high_water != NULL)
		{
			*config->stack_high_water = uxTaskGetStackHighWaterMark(NULL);
		}
#endif
	}
}

#endif /* __CONTROL_TASK_H__ */
