/**
******************************************************************************
 * @file    wfly_control.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "wfly_control.h"

#include "defense_center.h"

#include "buzzer.h"

#define WFLY_SBUS_USART_HANDLE &huart5

static wfly_t wfly_instance;

static USART_instance_t *wfly_usart_instance;

// static supervisor_t *wfly_supervisor_instance; // 监视器实例

void wfly_to_rc(const uint8_t *buf)
{
  /******************************wfly*****************************/    
	// USART_Error_Lost(wfly_usart_instance);

	if (wfly_usart_instance->lost_flag == 1)
	{
		memset(&wfly_instance, 0, sizeof(wfly_t));
		wfly_instance.online = 0;
		return;
	}

	if ((buf[0] != WFLY_SBUS_HEAD) || (buf[24] != WFLY_SBUS_END))
	{
		return;
	}

	if ((buf[23] == 0x0C) || (buf[23] == 0x0F))
	{
		wfly_instance.online = 0;
		memset(&wfly_instance, 0, sizeof(wfly_t));
		return;
	}
	else
	{
		wfly_instance.online = 1;
	}
  /******************************wfly*****************************/

	wfly_instance.rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF) - WFLY_RC_OFFSET;
	wfly_instance.rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF) - WFLY_RC_OFFSET;
	wfly_instance.rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF) - WFLY_RC_OFFSET;
	wfly_instance.rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF) - WFLY_RC_OFFSET - 23;
	wfly_instance.rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
	wfly_instance.rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
	wfly_instance.rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
	wfly_instance.rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
	wfly_instance.rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
	wfly_instance.rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

	//    for (uint8_t i = 0; i < 4; i++)
	//    {
	//        if (wfly_instance.rc.ch[i] < 5 || wfly_instance.rc.ch[i] > -5)
	//        {
	//            wfly_instance.rc.ch[i] = 0;
	//        }
	//    }

	switch (wfly_instance.rc.ch[4])
	{
		case 0x161:
			wfly_instance.toggle.SA = WFLY_SW_UP;
			break;
		case 0x0400:
			wfly_instance.toggle.SA = WFLY_SW_MID;
			break;
		case 0x069F:
			wfly_instance.toggle.SA = WFLY_SW_DOWN;
			break;
	}
	switch (wfly_instance.rc.ch[5])
	{
		case 0x161:
			wfly_instance.toggle.SB = WFLY_SW_UP;
			break;
		case 0x0400:
			wfly_instance.toggle.SB = WFLY_SW_MID;
			break;
		case 0x069F:
			wfly_instance.toggle.SB = WFLY_SW_DOWN;
			break;
	}
	switch (wfly_instance.rc.ch[6])
	{
		case 0x161:
			wfly_instance.toggle.SC = WFLY_SW_UP;
			break;
		case 0x0400:
			wfly_instance.toggle.SC = WFLY_SW_MID;
			break;
		case 0x069F:
			wfly_instance.toggle.SC = WFLY_SW_DOWN;
			break;
	}
	switch (wfly_instance.rc.ch[7])
	{
		case 0x161:
			wfly_instance.toggle.SD = WFLY_SW_UP;
			break;
		case 0x0400:
			wfly_instance.toggle.SD = WFLY_SW_MID;
			break;
		case 0x069F:
			wfly_instance.toggle.SD = WFLY_SW_DOWN;
			break;
	}
	switch (wfly_instance.rc.ch[8])
	{
		case 0x161:
			wfly_instance.toggle.SF = WFLY_SW_UP;
			break;
		case 0x069F:
			wfly_instance.toggle.SF = WFLY_SW_DOWN;
			break;
	}
	switch (wfly_instance.rc.ch[9])
	{
		case 0x161:
			wfly_instance.toggle.SH = WFLY_SW_UP;
			break;
		case 0x069F:
			wfly_instance.toggle.SH = WFLY_SW_DOWN;
			break;
	}
}

static void WFLY_SBUS_Rx_Callback(void)
{
	wfly_to_rc(wfly_usart_instance->recv_buff);
}

// static void WFLY_SBUS_Lost_Callback(void *id)
// {
//     ;
// }

wfly_t *WFLY_SBUS_Register(void)
{
	__disable_irq( );

	usart_init_config_t config;
	config.module_callback = WFLY_SBUS_Rx_Callback;
	config.usart_handle    = WFLY_SBUS_USART_HANDLE;
	config.recv_buff_size  = BUFF_RC_RECEIVE_SIZE;
	wfly_usart_instance    = USART_Register(&config);

	/******************************wfly接收器在遥控器未开启时仍然会发送串口信息*****************************/
	//似乎监管者无法解决这个问题，暂时先注释掉，只能在解析时判断
	// supervisor_init_config_t supervisor_config = {
	//     .init_count = 100, // 初始化计数器
	//     .reload_count = 100, // 重载计数器
	//     .handler_callback = WFLY_SBUS_Lost_Callback, // 离线回调函数
	//     .owner_id = NULL // 所属者ID
	// };

	// wfly_supervisor_instance = Supervisor_Register(&supervisor_config);

	/******************************wfly接收器在遥控器未开启时仍然会发送串口信息*****************************/

	__enable_irq( );

	return (&wfly_instance);
}
