#ifndef SERIAL_TEST_UI_INTERFACE_H
#define SERIAL_TEST_UI_INTERFACE_H

#include "main.h"

#include "stm32h7xx_hal.h"
#include "ui_types.h"
#include "usart.h"
#include "bsp_usart.h"

extern USART_instance_t *referee_usart_instance;
extern int ui_self_id;
extern uint8_t seq;
#define SEND_MESSAGE(message, len)  USART_Send(referee_usart_instance,message,len,USART_TRANSFER_DMA);
//#define SEND_MESSAGE(message, len)  HAL_UART_Transmit_DMA(&huart1, message, len);

//由于裁判系统接收限制，你可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数替换掉HAL_UART_Transmit_DMA
unsigned char calc_crc8(unsigned char *pchMessage, unsigned int dwLength);
uint16_t calc_crc16(uint8_t *pchMessage, uint32_t dwLength);
void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void autonomous_sentinel_decision(sentry_cmd_t *msg);
void autonomous_radar_decision(radar_cmd_t *msg);

#endif //SERIAL_TEST_UI_INTERFACE_H
