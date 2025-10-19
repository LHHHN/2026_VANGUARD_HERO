/**
* @file wfly_control.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __WFLY_CONTROL_H__
#define __WFLY_CONTROL_H__

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "bsp_usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define BUFF_RC_RECEIVE_SIZE 25
#define WFLY_SBUS_HEAD 0X0F
#define WFLY_SBUS_END 0X00
#define WFLY_RC_OFFSET 1024

typedef enum
{
	WFLY_SW_UP,
	WFLY_SW_MID,
	WFLY_SW_DOWN
} toggle_e;

typedef enum
{
	WFLY_CH_LY,
	WFLY_CH_LX,
	WFLY_CH_RY,
	WFLY_CH_RX
} rc_ch_e;

typedef struct
{
	uint16_t online;
	struct
	{
		/*
		 *
		 * ch[0] : 左摇杆上下 3535-1024-1695
		 * ch[1] : 左摇杆左右
		 * ch[2] : 右摇杆上下
		 * ch[3] : 右摇杆左右
		 * */
		int16_t ch[10];
	} rc;
	struct
	{
		uint8_t SA;
		uint8_t SB;
		uint8_t SC;
		uint8_t SD;
		uint8_t SF;
		uint8_t SH;
	} toggle;
} wfly_t;

extern wfly_t *WFLY_SBUS_Register(void);

#endif /*__WFLY_CONTROL_H__*/