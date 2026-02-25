// 接口头文件是ui.h
// 为了使消息能够发送给裁判系统，必须要自定义ui_interface.c中的ui_self_id变量（该变量也可以使用其他代码从裁判系统读取后由程序修改）
// 由于裁判系统接收限制，可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数在ui_interface.h中替换掉SEND_MESSAGE的宏定义HAL_UART_Transmit_DMA
#include "ui.h"
#include "math.h"
#include "main.h"
#include "gimbal.h"
#include "remote_control.h"
#include "chassis.h"
#include "referee.h"
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "message_center.h"

#include "DM_motor.h"

#include "bsp_dwt.h"
#define  UI_TASK_PERIOD 1 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ui_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

osThreadId_t ui_task_handel;
static publisher_t *ui_publisher;
static subscriber_t *ui_subscriber;

static void UI_Task(void *argument);

uint32_t ui_diff = 0;
uint8_t fric_flag;
uint8_t auto_flag;

uint8_t supercap_energy = 0;
float delta_gim;

void UI_Task_Init(void) // 创建所有元素的函数，调用一次即可，如果只能显示部分图案，就尝试重新创建
{
	  const osThreadAttr_t attr = {
		.name = "UI_Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t)osPriorityNormal,
	};

	ui_task_handel = osThreadNew(UI_Task, NULL, &attr);
	ui_publisher  = Publisher_Register("ui_transmit_feed", sizeof(ui_behaviour_t));
	ui_subscriber = Subscriber_Register("ui_receive_cmd", sizeof(ui_cmd_t));
}

void ui_init() // 创建所有元素的函数，调用一次即可，如果只能显示部分图案，就尝试重新创建
{
	_ui_init_Infantry_group1_0();
	HAL_Delay(115);
	_ui_init_Infantry_group1_1();
	HAL_Delay(115);
	_ui_init_Infantry_group1_2();
	HAL_Delay(115);
	_ui_init_Infantry_group1_3();
	HAL_Delay(115);
	_ui_init_Infantry_group1_4();
	HAL_Delay(115);
	_ui_init_Infantry_group2_0();
	HAL_Delay(115);
}

void ui_reinit() // 重新创建所有元素的函数，由于下位机上电时主控不一定连接到服务器，建议设置一个遥控按键，在按下按键后重新创建，避免由于丢包等各种因素ui没有显示，如果还是只能显示部分图案，可尝试在每行后面适当加上延时
{
	_ui_remove_Infantry_group1_0();
	HAL_Delay(115);
	_ui_remove_Infantry_group1_1();
	HAL_Delay(115);
	_ui_remove_Infantry_group1_2();
	HAL_Delay(115);
	_ui_remove_Infantry_group1_3();
	HAL_Delay(115);
	_ui_remove_Infantry_group1_4();
	HAL_Delay(115);
	_ui_remove_Infantry_group2_0();
	HAL_Delay(115);

	_ui_init_Infantry_group1_0();
	HAL_Delay(115);
	_ui_init_Infantry_group1_1();
	HAL_Delay(115);
	_ui_init_Infantry_group1_2();
	HAL_Delay(115);
	_ui_init_Infantry_group1_3();
	HAL_Delay(115);
	_ui_init_Infantry_group1_4();
	HAL_Delay(115);
	_ui_init_Infantry_group2_0();
	HAL_Delay(115);
}


void update_shooter_ui(float gimbal_delta, uint8_t cap_level, uint8_t auto_status, uint8_t fric_status, uint8_t error_num) // 更新所有UI的函数，gimbal_delta为（云台角度-底盘角度），正中间为0度，顺时针为正，弧度制；cap_level为超电剩余百分比；auto_status为自瞄状态，2为自瞄中，1为自瞄已准备好，否则为0；fric_status为摩擦轮状态，开启为1，否则为0
{

	// 更新云台方向

	ui_Infantry_group2_Front->end_x = ui_Infantry_group2_Front->start_x + (int)(85 * sin(gimbal_delta));
	ui_Infantry_group2_Front->end_y = ui_Infantry_group2_Front->start_y + (int)(85 * cos(gimbal_delta));

	// 更新超电剩余电量

	ui_Infantry_group2_electricity->end_y = ui_Infantry_group2_electricity->start_y + 4 * cap_level;

	// 更新自瞄状态

	if (auto_status == 2)
	{
		ui_Infantry_group2_auto_point->color = 0;
	}
	else if (auto_status == 1)
	{
		ui_Infantry_group2_auto_point->color = 2;
	}
	else
	{
		ui_Infantry_group2_auto_point->color = 8;
	}

	// 更新摩擦轮状态

	if (fric_status == 1)
	{
		ui_Infantry_group2_fric_point->color = 2;
	}
	else
	{
		ui_Infantry_group2_fric_point->color = 8;
	}

	// 更新错误码

	ui_Infantry_group2_error_num->number = error_num;

	// 更新UI组2
	_ui_update_Infantry_group2_0();


}



void UI_Task(void *argument)//此处根据自己代码的结构体自行更改id及参数
{

	uint32_t time = osKernelGetTickCount();

	ui_init();

	for (;;)
	{

//		if (gimbal_cmd.friction_state)
//			fric_flag = 1;
//		else
//			fric_flag = 0;

//		if (gimbal_cmd.shooter == AUTO_FIRE)
//			auto_flag = 2;					 // 云台跟随+火控开启标志位
//		else if (gimbal_cmd.shooter == AUTO) // 云台跟随
//			auto_flag = 1;
//		else
//			auto_flag = 0;

//		delta_gim = -chassis.error / 8192 * 2 * PI;
//		supercap_energy = (uint8_t)(supercap_rx_data.capEnergy * 100 / 255);

//		update_shooter_ui(delta_gim, supercap_energy, auto_flag, fric_flag, supercap_rx_data.errorCode); // 更新UI

//	    if(refree_info.Game_Robot_state.robot_id > 0 && refree_info.Game_Robot_state.robot_id < 12)
//			    ui_self_id = 3;
//		else if(refree_info.Game_Robot_state.robot_id > 100 && refree_info.Game_Robot_state.robot_id < 112)
//				ui_self_id = 103;
		ui_diff = osKernelGetTickCount() - time;
		time = osKernelGetTickCount();
		osDelayUntil(time + UI_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
		ui_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
