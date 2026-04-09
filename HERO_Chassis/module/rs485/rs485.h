#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "gimbal.h"
#include "shoot.h"

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

typedef struct
{
    uint8_t frame_header;

    // float chassis_omega_speed;

    // int16_t rocker_r_; // 右水平
    // int16_t rocker_r1; // 右竖直
    // uint8_t rc_switch;  // 两侧拨杆
    // float angle_tar;

    int16_t rocker_r1; // pitch 遥控器输入
    int16_t mouse_y;   // pitch 键鼠输入

    uint8_t rc_switch;         // 两侧拨杆
    uint8_t control_src;       // 0: remote, 1: keymouse
    gimbal_mode_e gimbal_mode; // 云台模式
    shoot_mode_e shoot_mode;   // 发射模式

    float angle_tar; // yaw 目标角

    uint8_t frame_tailer;
    uint8_t check_sum;
} __attribute__((packed)) Tx_packed_t;

typedef struct
{
    uint8_t frame_header;

    // uint8_t chassis_mode;
    // uint8_t shoot_mode;
    // float delta_target_angle_yaw;
    // float target_x_speed;
    // float target_y_speed;
    // float target_omega_speed;

    float speed_yaw;
    float angle_yaw;

    uint8_t vs_mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float vs_yaw_tar; // 视觉给定的yaw目标角度

    uint8_t shoot_launched; // 0: 没有发射，1: 发射了

    uint8_t frame_tailer;
    uint8_t check_sum;
} __attribute__((packed)) Rx_packed_t;

extern Tx_packed_t uart2_tx_message;
extern Rx_packed_t uart2_rx_message;

extern uint8_t uart2_receive_buffer[sizeof(Rx_packed_t) * 3];
extern uint8_t uart2_transmit_buffer[sizeof(Tx_packed_t)];

extern uint8_t ready_to_transmit;
extern uint8_t ready_to_receive;

extern uint8_t uart2_status;
extern uint32_t last_uart2_uwTick;
extern uint8_t uart2_current_byte;
extern uint8_t rs485_status;

void uart2_send_data(Tx_packed_t *data_p);
void uart2_transmit_control(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void uart2_online_check(void);

#endif
