/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/******************************这里是塞💩的地方，请随便塞💩*****************************/
#include "rng.h"
#include "robot_frame_init.h"
#include "buzzer.h"
#include "ws2812.h"
#include "wfly_control.h"

/******************************这里是塞💩的地方，请随便塞💩*****************************/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/******************************这里是塞💩的地方，请随便塞💩*****************************/ 
extern wfly_t *rc_data;
uint32_t beat = 0;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	WS2812_Control(ws2812_instance, GREEN_WS2812_COLOR);
	Buzzer_Play(StartUP_sound, 0);
	osDelay(1500);
  /* Infinite loop */
  for(;;)
  {
    static uint8_t music_lock = 0;
    if(rc_data->online == 0)
    {
        if(music_lock == 0)
        {
            Buzzer_Play(No_RC_sound, 0);
            music_lock = 1;
        }
    }
    else if(rc_data->online == 1)
    {
        if(music_lock == 1)
        {
            Buzzer_Play(Yes_RC_sound, 0);
            music_lock = 0;
        }
    }

		beat++;
		if((beat % 120000) == 0)
		{
			Buzzer_Play(Super_Mario_sound, 0);
		}
		else if((beat % 30000) == 0)
		{
			Buzzer_Play(Heartbeat_sound, 0);
		}
		else if((beat % 55555) == 0)
		{
			Buzzer_Play(Call_Airsupport_sound, 0);
		}
		
//有💩		
//    uint32_t rand_data = 0;
//    HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t*)&rand_data);
//	
//		if(rand_data == 0xFFFFFFFF)
//		{
//			Buzzer_Play(Call_Airsupport_sound, 0);
//		}
		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/******************************这里是塞💩的地方，请随便塞💩*****************************/
/* USER CODE END Application */

