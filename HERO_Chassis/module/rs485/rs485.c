#include "rs485.h"

#include "defense_center.h"

#include "CRC.h"

#include "bsp_usart.h"

#define BUFF_RS485_RECEIVE_SIZE 128

rs485_command_t rs485_command;

static USART_instance_t *rs485_usart_instance;
static supervisor_t *rs485_supervisor_instance;

tx_chassis_t rs485_tx_message;
rx_gimbal_t rs485_rx_message;

uint8_t rs485_tx_message_temp[sizeof(tx_chassis_t)];
uint8_t rs485_rx_message_temp[sizeof(rx_gimbal_t) * 2];

uint8_t rs485_receive_en_flag = 0;

static uint8_t rs485_parse_buff[sizeof(rx_gimbal_t)];
static uint16_t rs485_parse_index = 0;

uint32_t rs485_cnt ;

static void RS485_Reset_Parse_State(void)
{
    rs485_parse_index = 0;
    memset(rs485_parse_buff, 0, sizeof(rs485_parse_buff));
}

static void RS485_Recover_Parse_State(void)
{
    for(uint16_t i = 1; i < sizeof(rx_gimbal_t); i++)
    {
        if(rs485_parse_buff[i] == FRAME_HEADER)
        {
            rs485_parse_index = sizeof(rx_gimbal_t) - i;
            memmove(rs485_parse_buff, &rs485_parse_buff[i], rs485_parse_index);
            return;
        }
    }
    RS485_Reset_Parse_State();
}

static void RS485_Parse_Byte(uint8_t data)
{
    if(rs485_parse_index == 0)
    {
        if(data != FRAME_HEADER)
        {
            return;
        }
    }

    rs485_parse_buff[rs485_parse_index++] = data;

    if(rs485_parse_index < sizeof(rx_gimbal_t))
    {
        return;
    }

    if(rs485_parse_buff[sizeof(rx_gimbal_t) - 1] == FRAME_TAILER &&
       Verify_CRC8_Check_Sum(rs485_parse_buff + 1, sizeof(rx_gimbal_t) - 2))
    {
        memcpy(rs485_rx_message_temp, rs485_parse_buff, sizeof(rx_gimbal_t));
        rs485_receive_en_flag = 1;
        RS485_Reset_Parse_State();
        return;
    }

    RS485_Recover_Parse_State();
}

static void RS485_Parse_Bytes(uint8_t *data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        RS485_Parse_Byte(data[i]);
    }
}

static void RS485_Parse_Circular_Rx_Data(void)
{
    uint16_t start = rs485_usart_instance->rx_start_pos;
    uint16_t end = rs485_usart_instance->current_size;
    uint16_t buff_size = rs485_usart_instance->recv_buff_size;

    if(start >= buff_size)
    {
        start = 0;
    }

    if(start == end)
    {
        return;
    }

    if(start < end)
    {
        RS485_Parse_Bytes(&rs485_usart_instance->recv_buff[start], end - start);
    }
    else
    {
        RS485_Parse_Bytes(&rs485_usart_instance->recv_buff[start], buff_size - start);
        if(end > 0)
        {
            RS485_Parse_Bytes(rs485_usart_instance->recv_buff, end);
        }
    }
}

static void RS485_Rx_Callback(void)
{
    if(rs485_usart_instance->usart_handle->hdmarx != NULL &&
       rs485_usart_instance->usart_handle->hdmarx->Init.Mode == DMA_CIRCULAR)
    {
        RS485_Parse_Circular_Rx_Data();
        rs485_tx_message.chassis_beat++;
        return;
    }

    if(rs485_usart_instance->current_size > rs485_usart_instance->recv_buff_size)
    {
        rs485_usart_instance->current_size = rs485_usart_instance->recv_buff_size;
    }

    RS485_Parse_Bytes(rs485_usart_instance->recv_buff, rs485_usart_instance->current_size);
}

static void RS485_Lost_Callback(void *id)
{
    if(rs485_usart_instance->usart_handle->hdmarx != NULL &&
       rs485_usart_instance->usart_handle->hdmarx->Init.Mode == DMA_CIRCULAR)
    {
        memset(&rs485_rx_message, 0, sizeof(rs485_rx_message));
        memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
        RS485_Reset_Parse_State();
        rs485_receive_en_flag = 0;
        rs485_command.online = 0;
        return;
    }
    
	memset(&rs485_rx_message, 0, sizeof(rs485_rx_message)); // 清空遥控器数据
    HAL_UARTEx_ReceiveToIdle_DMA(rs485_usart_instance->usart_handle,
			                             rs485_usart_instance->recv_buff,
			                             rs485_usart_instance->recv_buff_size);
	__HAL_DMA_DISABLE_IT(rs485_usart_instance->usart_handle->hdmarx, DMA_IT_HT);    
	rs485_receive_en_flag = 0;
    rs485_command.online = 0; // 遥控器离线
}

void RS485_Handle_Rx_Data(void)
{
    if(rs485_receive_en_flag == 0)
    {
        return;
    }

    if(rs485_rx_message_temp[0] != FRAME_HEADER || rs485_rx_message_temp[sizeof(rx_gimbal_t) - 1] != FRAME_TAILER)
    {
        memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
        rs485_receive_en_flag = 0;
    }
    else
    {
        if(rs485_receive_en_flag == 1)
        {
            if (Verify_CRC8_Check_Sum(rs485_rx_message_temp + 1, sizeof(rx_gimbal_t) - 2))
            {
                rs485_cnt ++;   
                memcpy(&rs485_rx_message, rs485_rx_message_temp, sizeof(rx_gimbal_t));
                Supervisor_Reload(rs485_supervisor_instance); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
                rs485_command.online = 1;
            }
            memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
            rs485_receive_en_flag = 0;
        }
    }
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
}

void RS485_Handle_Tx_Data(void)
{
    rs485_tx_message.frame_header = FRAME_HEADER;
    rs485_tx_message.frame_tailer = FRAME_TAILER;
    memcpy(&rs485_tx_message_temp, &rs485_tx_message, sizeof(tx_chassis_t));
    Append_CRC8_Check_Sum(rs485_tx_message_temp + 1, sizeof(tx_chassis_t) - 2);
    USART_Send(rs485_usart_instance, (uint8_t *)rs485_tx_message_temp, sizeof(rs485_tx_message_temp), USART_TRANSFER_DMA);
	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // 发送回调函数
{
    if (huart == rs485_usart_instance->usart_handle)
    {
        // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    }
}

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
rs485_command_t RS485_Register(UART_HandleTypeDef *rs485_handle)
{	
	usart_init_config_t conf;
	conf.module_callback = RS485_Rx_Callback;
	conf.usart_handle    = rs485_handle;
	conf.recv_buff_size  = BUFF_RS485_RECEIVE_SIZE;
	rs485_usart_instance    = USART_Register(&conf);

	// 进行守护进程的注册,用于定时检查遥控器是否正常工作
	supervisor_init_config_t supervisor_conf = {
		.reload_count = 3,
		.handler_callback = RS485_Lost_Callback,
		.owner_id = NULL,
	};
	rs485_supervisor_instance = Supervisor_Register(&supervisor_conf);

	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    return rs485_command;
}

/*ai修改后的快慢包发送*/
// #include "rs485.h"

// #include "defense_center.h"

// #include "CRC.h"

// #include "bsp_usart.h"

// #define BUFF_RS485_RECEIVE_SIZE 128

// rs485_command_t rs485_command;

// static USART_instance_t *rs485_usart_instance;
// static supervisor_t *rs485_supervisor_instance;

// tx_chassis_t rs485_tx_message;
// rx_gimbal_t rs485_rx_message;

// uint8_t rs485_tx_message_temp[sizeof(tx_chassis_t)];
// uint8_t rs485_rx_message_temp[BUFF_RS485_RECEIVE_SIZE];

// uint8_t rs485_receive_en_flag = 0;
// static uint8_t rs485_rx_message_size = 0;

// static float RS485_Int16_To_Float(int16_t value, float scale)
// {
//     return (float)value / scale;
// }

// static uint8_t RS485_Verify_Frame(uint8_t *frame, uint16_t frame_size)
// {
//     if(frame[0] != FRAME_HEADER || frame[frame_size - 1] != FRAME_TAILER)
//     {
//         return 0;
//     }

//     return Verify_CRC8_Check_Sum(frame + 1, frame_size - 2);
// }

// static void RS485_Decode_Gimbal_Fast_Frame(rs485_gimbal_fast_t *frame)
// {
//     uint8_t mode_flags = frame->mode_flags;

//     rs485_rx_message.frame_header = FRAME_HEADER;
//     rs485_rx_message.gimbal_mode = mode_flags & 0x03U;
//     rs485_rx_message.auto_aiming_flag = (mode_flags >> 2) & 0x03U;
//     rs485_rx_message.control_remote_flag = (mode_flags >> 4) & 0x01U;
//     rs485_rx_message.shoot_fire_en_flag = (mode_flags >> 5) & 0x01U;
//     rs485_rx_message.shoot_launched_flag = (mode_flags >> 6) & 0x01U;
//     rs485_rx_message.gimbal_target_yaw = RS485_Int16_To_Float(frame->yaw_target, RS485_ANGLE_SCALE);
//     rs485_rx_message.gimbal_target_yaw_speed = RS485_Int16_To_Float(frame->yaw_step, RS485_YAW_STEP_SCALE);
//     rs485_rx_message.gimbal_measure_yaw = RS485_Int16_To_Float(frame->yaw_measure, RS485_ANGLE_SCALE);
//     rs485_rx_message.gimbal_measure_yaw_speed = RS485_Int16_To_Float(frame->yaw_gyro, RS485_GYRO_SCALE);
//     rs485_rx_message.frame_tailer = FRAME_TAILER;
// }

// static void RS485_Decode_Gimbal_Slow_Frame(rs485_gimbal_slow_t *frame)
// {
//     uint8_t mode_flags = frame->mode_flags;

//     rs485_rx_message.frame_header = FRAME_HEADER;
//     rs485_rx_message.chassis_mode = mode_flags & 0x07U;
//     rs485_rx_message.shoot_mode = (mode_flags >> 3) & 0x03U;
//     rs485_rx_message.shoot_fire_en_flag = (mode_flags >> 5) & 0x01U;
//     rs485_rx_message.shoot_launched_flag = (mode_flags >> 6) & 0x01U;
//     rs485_rx_message.control_remote_flag = (mode_flags >> 7) & 0x01U;
//     rs485_rx_message.chassis_target_vx = RS485_Int16_To_Float(frame->chassis_vx, RS485_SPEED_SCALE);
//     rs485_rx_message.chassis_target_vy = RS485_Int16_To_Float(frame->chassis_vy, RS485_SPEED_SCALE);
//     rs485_rx_message.chassis_target_wz = RS485_Int16_To_Float(frame->chassis_wz, RS485_SPEED_SCALE);
//     rs485_rx_message.chassis_target_leg_angle = RS485_Int16_To_Float(frame->leg_angle, RS485_ANGLE_SCALE);
//     rs485_rx_message.frame_tailer = FRAME_TAILER;
// }

// static void RS485_Rx_Callback(void)
// {
// 		HAL_UART_DMAStop(rs485_usart_instance->usart_handle);
//     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
//     if(rs485_usart_instance->current_size > 0 && rs485_usart_instance->current_size <= sizeof(rs485_rx_message_temp))
//     {
//         memcpy(rs485_rx_message_temp, rs485_usart_instance->recv_buff, rs485_usart_instance->current_size);
//         rs485_rx_message_size = rs485_usart_instance->current_size;
//         rs485_receive_en_flag = 1;
//         rs485_command.online = 1;
//         Supervisor_Reload(rs485_supervisor_instance);
//         return;
//     }
//     Supervisor_Reload(rs485_supervisor_instance);// 先喂狗
//     if(rs485_usart_instance->recv_buff[0] == FRAME_HEADER && rs485_usart_instance->recv_buff[sizeof(rx_gimbal_t) - 1] == FRAME_TAILER)
//     {
//         memcpy(&rs485_rx_message_temp, rs485_usart_instance->recv_buff, sizeof(rx_gimbal_t));
//         rs485_receive_en_flag = 1;
//         rs485_command.online = 1; // 遥控器在线
//     }
// }

// static void RS485_Lost_Callback(void *id)
// {
// 	memset(&rs485_rx_message, 0, sizeof(rs485_rx_message)); // 清空遥控器数据
//     HAL_UARTEx_ReceiveToIdle_DMA(rs485_usart_instance->usart_handle,
// 			                             rs485_usart_instance->recv_buff,
// 			                             rs485_usart_instance->recv_buff_size);
// 	__HAL_DMA_DISABLE_IT(rs485_usart_instance->usart_handle->hdmarx, DMA_IT_HT);
// 	rs485_receive_en_flag = 0;
//     rs485_command.online = 0; // 遥控器离线
// }

// void RS485_Handle_Rx_Data(void)
// {
//     if(rs485_receive_en_flag == 1)
//     {
//         uint16_t offset = 0;
//         uint8_t decode_success = 0;

//         while(offset < rs485_rx_message_size)
//         {
//             uint8_t frame_type;
//             uint16_t frame_size;

//             if(rs485_rx_message_temp[offset] != FRAME_HEADER)
//             {
//                 offset++;
//                 continue;
//             }

//             if(offset + sizeof(rs485_gimbal_fast_t) > rs485_rx_message_size)
//             {
//                 break;
//             }

//             frame_type = rs485_rx_message_temp[offset + 1];
//             if(frame_type == RS485_GIMBAL_FRAME_FAST)
//             {
//                 frame_size = sizeof(rs485_gimbal_fast_t);
//             }
//             else if(frame_type == RS485_GIMBAL_FRAME_SLOW)
//             {
//                 frame_size = sizeof(rs485_gimbal_slow_t);
//             }
//             else
//             {
//                 offset++;
//                 continue;
//             }

//             if(offset + frame_size > rs485_rx_message_size)
//             {
//                 break;
//             }

//             if(RS485_Verify_Frame(&rs485_rx_message_temp[offset], frame_size))
//             {
//                 if(frame_type == RS485_GIMBAL_FRAME_FAST)
//                 {
//                     RS485_Decode_Gimbal_Fast_Frame((rs485_gimbal_fast_t *)&rs485_rx_message_temp[offset]);
//                 }
//                 else
//                 {
//                     RS485_Decode_Gimbal_Slow_Frame((rs485_gimbal_slow_t *)&rs485_rx_message_temp[offset]);
//                 }

//                 decode_success = 1;
//                 offset += frame_size;
//             }
//             else
//             {
//                 offset++;
//             }
//         }

//         if(decode_success == 1)
//         {
//             Supervisor_Reload(rs485_supervisor_instance);
//         }

//         memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
//         rs485_rx_message_size = 0;
//         rs485_receive_en_flag = 0;
//         return;
//     }

//     if(rs485_rx_message_temp[0] != FRAME_HEADER || rs485_rx_message_temp[sizeof(rx_gimbal_t) - 1] != FRAME_TAILER)
//     {
//         memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
//         rs485_receive_en_flag = 0;
//     }
//     else
//     {
//         if(rs485_receive_en_flag == 1)
//         {
//             if (Verify_CRC8_Check_Sum(rs485_rx_message_temp + 1, sizeof(rx_gimbal_t) - 2))
//             {
//                 memcpy(&rs485_rx_message, rs485_rx_message_temp, sizeof(rx_gimbal_t));
//                 Supervisor_Reload(rs485_supervisor_instance); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
//             }
//             memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
//             rs485_receive_en_flag = 0;
//         }
//     }
// //    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
// }

// void RS485_Handle_Tx_Data(void)
// {
//     rs485_tx_message.frame_header = FRAME_HEADER;
//     rs485_tx_message.frame_tailer = FRAME_TAILER;
//     memcpy(&rs485_tx_message_temp, &rs485_tx_message, sizeof(tx_chassis_t));
//     Append_CRC8_Check_Sum(rs485_tx_message_temp + 1, sizeof(tx_chassis_t) - 2);
//     USART_Send(rs485_usart_instance, (uint8_t *)rs485_tx_message_temp, sizeof(rs485_tx_message_temp), USART_TRANSFER_DMA);
// 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
// }

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // 发送回调函数
// {
//     if (huart == rs485_usart_instance->usart_handle)
//     {
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
//     }
// }

// /**
//  * @brief 初始化遥控器,该函数会将遥控器注册到串口
//  *
//  * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
//  *
//  */
// rs485_command_t RS485_Register(UART_HandleTypeDef *rs485_handle)
// {
// 	usart_init_config_t conf;
// 	conf.module_callback = RS485_Rx_Callback;
// 	conf.usart_handle    = rs485_handle;
// 	conf.recv_buff_size  = BUFF_RS485_RECEIVE_SIZE;
// 	rs485_usart_instance    = USART_Register(&conf);

// 	// 进行守护进程的注册,用于定时检查遥控器是否正常工作
// 	supervisor_init_config_t supervisor_conf = {
// 		.reload_count = 10,
// 		.handler_callback = RS485_Lost_Callback,
// 		.owner_id = NULL,
// 	};
// 	rs485_supervisor_instance = Supervisor_Register(&supervisor_conf);

// 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
//     return rs485_command;
// }
