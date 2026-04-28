#ifndef __RS485_H
#define __RS485_H

#include <string.h>
#include "main.h"

#define RS485_CHA 0

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

typedef struct
{
    uint8_t frame_header; // 帧头
    uint8_t chassis_mode; // 底盘模式
    uint8_t gimbal_mode;  // 云台模式
    uint8_t shoot_mode;   // 发射模式

    float chassis_target_vx;
    float chassis_target_vy;
    float chassis_target_wz;
	float chassis_target_leg_angle;
    float gimbal_target_yaw;
    float gimbal_target_yaw_speed;
    float gimbal_measure_yaw;
    float gimbal_measure_yaw_speed;
    float gimbal_measure_pitch;

    uint8_t shoot_fire_en_flag;
    uint8_t shoot_launched_flag;

    uint8_t auto_aiming_flag;
    uint8_t control_remote_flag;
    uint8_t crc;    // 校验和
    uint8_t frame_tailer; // 帧尾
} __attribute__((packed)) tx_gimbal_t;

typedef struct
{
    uint8_t frame_header;
    uint8_t chassis_beat;
    uint8_t chassis_flag;
    uint8_t crc;
    uint8_t frame_tailer;
} __attribute__((packed)) rx_chassis_t;

typedef struct
{
    uint8_t online;
} __attribute__((packed)) rs485_command_t;

extern rs485_command_t rs485_command;

extern tx_gimbal_t rs485_tx_message;
extern rx_chassis_t rs485_rx_message;

rs485_command_t RS485_Register(UART_HandleTypeDef *rs485_handle);
void RS485_Handle_Rx_Data(void);
void RS485_Handle_Tx_Data(void);

#endif
