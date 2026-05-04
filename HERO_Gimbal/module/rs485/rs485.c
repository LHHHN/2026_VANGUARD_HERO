#include "rs485.h"

#include "defense_center.h"

#include "CRC.h"

#include "bsp_usart.h"

#define BUFF_RS485_RECEIVE_SIZE 128

rs485_command_t rs485_command;

static USART_instance_t *rs485_usart_instance;
static supervisor_t *rs485_supervisor_instance;

tx_gimbal_t rs485_tx_message;
rx_chassis_t rs485_rx_message;

uint8_t rs485_tx_message_temp[sizeof(tx_gimbal_t)];
uint8_t rs485_rx_message_temp[sizeof(rx_chassis_t) * 2];

uint8_t rs485_receive_en_flag = 0;

static uint8_t rs485_parse_buff[sizeof(rx_chassis_t)];
static uint16_t rs485_parse_index = 0;

static void RS485_Reset_Parse_State(void)
{
    rs485_parse_index = 0;
    memset(rs485_parse_buff, 0, sizeof(rs485_parse_buff));
}

static void RS485_Recover_Parse_State(void)
{
    for(uint16_t i = 1; i < sizeof(rx_chassis_t); i++)
    {
        if(rs485_parse_buff[i] == FRAME_HEADER)
        {
            rs485_parse_index = sizeof(rx_chassis_t) - i;
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

    if(rs485_parse_index < sizeof(rx_chassis_t))
    {
        return;
    }

    if(rs485_parse_buff[sizeof(rx_chassis_t) - 1] == FRAME_TAILER &&
       Verify_CRC8_Check_Sum(rs485_parse_buff + 1, sizeof(rx_chassis_t) - 2))
    {
        memcpy(rs485_rx_message_temp, rs485_parse_buff, sizeof(rx_chassis_t));
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

    if(rs485_rx_message_temp[0] != FRAME_HEADER || rs485_rx_message_temp[sizeof(rx_chassis_t) - 1] != FRAME_TAILER)
    {
        memset(&rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
        rs485_receive_en_flag = 0;
    }
    else
    {
        if(rs485_receive_en_flag == 1)
        {
            if (Verify_CRC8_Check_Sum(rs485_rx_message_temp + 1, sizeof(rx_chassis_t) - 2))
            {   
                memcpy(&rs485_rx_message, rs485_rx_message_temp, sizeof(rx_chassis_t));
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
    memcpy(&rs485_tx_message_temp, &rs485_tx_message, sizeof(tx_gimbal_t));
    Append_CRC8_Check_Sum(rs485_tx_message_temp + 1, sizeof(tx_gimbal_t) - 2);
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
		.reload_count = 10, 
		.handler_callback = RS485_Lost_Callback,
		.owner_id = NULL,
	};
	rs485_supervisor_instance = Supervisor_Register(&supervisor_conf);

	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	return rs485_command;
}

