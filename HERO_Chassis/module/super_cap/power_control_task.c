// /**
//  ******************************************************************************
//  * @file    power_control_task.c
//  * @brief
//  * @author
//  ******************************************************************************
//  * Copyright (c) 2023 Team
//  * All rights reserved.
//  ******************************************************************************
//  */

// #include "FreeRTOS.h"
// #include "task.h"
// #include "cmsis_os.h"
// #include "cmsis_os2.h"

// #include "power_control_task.h"
// #include "power_control.h"

// #define POWER_CONTROL_TASK_PERIOD 2 // ms,由超电内部3ms最小发送间隔做最终限频

// osThreadId_t power_control_task_handle;

// static void Power_Control_Task(void *argument);

// void Power_Control_Task_Init(void)
// {
//     const osThreadAttr_t attr = {
//         .name = "PowerCtrl_Task",
//         .stack_size = 128 * 4,
//         .priority = (osPriority_t)osPriorityNormal4,
//     };

//     power_control_task_handle = osThreadNew(Power_Control_Task, NULL, &attr);
// }

// uint32_t power_control_task_diff;

// static void Power_Control_Task(void *argument)
// {
//     (void)argument;

//     // 上电后先完成超电模块初始化和默认控制帧准备
//     power_control_init();

//     uint32_t time = osKernelGetTickCount();

//     for (;;)
//     {
//         // 周期调用发送接口，实际发送频率由超电内部最小发送间隔限制
//         power_control_SetPowerState();

//         power_control_task_diff = osKernelGetTickCount() - time;
//         time = osKernelGetTickCount();
//         osDelay(73);
//     }
// }
