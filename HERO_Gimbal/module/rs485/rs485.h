#ifndef __RS485_H
#define __RS485_H

#include <stdint.h>
#include <string.h>
#include "main.h"

#define RS485_CHA 0

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

#define RS485_PROTOCOL_VERSION 1U
#define RS485_FRAME_TYPE_GIMBAL_CMD 0x11U
#define RS485_FRAME_TYPE_CHASSIS_STATUS 0x21U

typedef struct
{
    uint8_t frame_header;
    uint8_t version;
    uint8_t frame_type;
    uint8_t seq;
    uint8_t ack_seq;
    uint8_t payload_len;
} __attribute__((packed)) rs485_frame_header_t;

typedef struct
{
    uint8_t mode_flags;
    float chassis_target_vx;
    float chassis_target_vy;
    float chassis_target_wz;
    float gimbal_target_yaw;
    float gimbal_target_yaw_speed;
    float gimbal_measure_yaw;
    float gimbal_measure_yaw_speed;
    uint8_t gimbal_measure_pitch;
    uint8_t control_flags;
} __attribute__((packed)) rs485_gimbal_cmd_payload_t;

typedef struct
{
    uint8_t chassis_beat;
    uint8_t chassis_flag;
} __attribute__((packed)) rs485_chassis_status_payload_t;

typedef struct
{
    uint8_t chassis_mode : 4;
    uint8_t gimbal_mode : 2;
    uint8_t shoot_mode : 2;

    float chassis_target_vx;
    float chassis_target_vy;
    float chassis_target_wz;
    float gimbal_target_yaw;
    float gimbal_target_yaw_speed;
    float gimbal_measure_yaw;
    float gimbal_measure_yaw_speed;

    uint8_t gimbal_measure_pitch;

    uint8_t shoot_fire_en_flag : 2;
    uint8_t shoot_launched_flag : 1;
    uint8_t recovery_leg_flag: 1;
    uint8_t control_remote_flag : 1;
    uint8_t ui_refresh_flag : 1;
    uint8_t auto_aiming_flag : 2;

    uint8_t seq;
    uint8_t ack_seq;
} tx_gimbal_t;

typedef struct
{
    uint8_t chassis_beat;
    uint8_t chassis_flag;
    uint8_t seq;
    uint8_t ack_seq;
} rx_chassis_t;

typedef struct
{
    uint8_t online;
    uint8_t last_rx_seq;
    uint8_t last_tx_seq;
    uint32_t rx_ok_count;
    uint32_t crc_error_count;
    uint32_t seq_lost_count;
    uint32_t tx_busy_count;
    uint32_t recover_count;
} rs485_command_t;

extern rs485_command_t rs485_command;

extern tx_gimbal_t rs485_tx_message;
extern rx_chassis_t rs485_rx_message;

rs485_command_t RS485_Register(UART_HandleTypeDef *rs485_handle);
void RS485_Handle_Rx_Data(void);
void RS485_Handle_Tx_Data(void);

#endif
