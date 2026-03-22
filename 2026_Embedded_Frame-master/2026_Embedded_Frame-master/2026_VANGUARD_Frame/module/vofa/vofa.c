/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参）
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		 2024/9/10		 向上位机vofa+发送数据
 *  v1.1		GUATAI	    2025/8/12	    邪恶bsp归一化
 *  v1.2        GUATAI      2025/9/14       修复发送浮点数组bug
 *  v1.3        GUATAI      2025/9/21       把简单的vofa函数变得复杂，功能不变让其晦涩难懂，当一个实在的cs
 */

#include <stdlib.h>
#include <string.h>

#include <stdarg.h>
#include <stdio.h>

#include "vofa.h"

#define MAX_BUFFER_SIZE 128

#define VOFA_USART_HANDLE &huart7

uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

USART_instance_t *vofa_usart_instance;

float vofa_data_view[20] = {0};

void VOFA_Register(void)
{
	usart_init_config_t config;
	config.module_callback = NULL; // 该模块不需要接收数据
	config.usart_handle    = VOFA_USART_HANDLE;
	config.recv_buff_size  = 1;

	vofa_usart_instance = USART_Register(&config);
}

/**
***********************************************************************
* @brief:      VOFA_Transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void VOFA_Transmit(uint8_t *buf, uint16_t len,usart_transfer_e mode)
{
	USART_Send(vofa_usart_instance, buf, len, mode);
}

// 按printf格式写，最后必须加\r\n
void VOFA_FireWater(const char *format, ...)
{
    uint8_t txBuffer[100];
    uint32_t n;
    va_list args;
    va_start(args, format);
    n = vsnprintf((char *)txBuffer, 100, format, args);

    //....在此替换你的串口发送函数...........
    VOFA_Transmit((uint8_t *)txBuffer, n, USART_TRANSFER_DMA);
    //......................................

    va_end(args);
}

// uint8_t tempData[MAX_BUFFER_SIZE] = {0};

// // 输入个数和数组地址
// void VOFA_JustFloat(float *_data, uint8_t _num)
// {
//     uint8_t temp_end[4] = {0, 0, 0x80, 0x7F};
//     float temp_copy[_num];

//     memcpy(&temp_copy, _data, sizeof(float) * _num);

//     memcpy(tempData, (uint8_t *)&temp_copy, sizeof(temp_copy));
//     memcpy(&tempData[_num * 4], &temp_end[0], 4);

//     //....在此替换你的串口发送函数...........
//     VOFA_Transmit( tempData, (_num + 1) * 4, USART_TRANSFER_DMA);
//     //......................................
// }

typedef union
{
    float float_t;
    uint8_t uint8_t[4];
} send_float;

uint8_t send_data[MAX_BUFFER_SIZE]; //定义通过串口传出去的数组，数量是所传数据的字节数加上4个字节的尾巴

void VOFA_JustFloat(float *_data, uint8_t _num)
{
    static uint8_t i = 0;
    send_float temp[_num];			//定义缓冲区数组
    
    for (i = 0; i < _num; i++)
    {
        temp[i].float_t = _data[i]; //将所传数据移到缓冲区数组
    }
    for (i = 0; i < _num; i++)
    {
        send_data[4 * i] = temp[i].uint8_t[0];
        send_data[4 * i + 1] = temp[i].uint8_t[1];
        send_data[4 * i + 2] = temp[i].uint8_t[2];
        send_data[4 * i + 3] = temp[i].uint8_t[3]; //将缓冲区数组内的浮点型数据转成4个字节的无符号整型，之后传到要通过串口传出的数组里
    }
    send_data[4 * _num] = 0x00;
    send_data[4 * _num + 1] = 0x00;
    send_data[4 * _num + 2] = 0x80;
    send_data[4 * _num + 3] = 0x7f; //加上协议要求的4个尾巴

    VOFA_Transmit((uint8_t *)send_data, 4 * _num + 4, USART_TRANSFER_DMA);
}

/**
 * @brief vofa发送不动长数据
 * @param buf 数组指针
 * @param len 数组长度
 */

void VOFA_Send_Data(float *buf, uint8_t len)
{
	cnt = len * 4;

	static uint8_t buf_end[4] = {0x00, 0x00, 0x80, 0x7f};

	memcpy(send_buf, buf, cnt);
	memcpy(send_buf + cnt, buf_end, 4);
	VOFA_Transmit((uint8_t *) send_buf, cnt + 4, USART_TRANSFER_DMA);
}

