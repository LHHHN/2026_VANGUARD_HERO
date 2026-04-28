#include "rs485.h"

#include "defense_center.h"

#include "CRC.h"

#include "bsp_usart.h"

#define BUFF_RS485_RECEIVE_SIZE 16

rs485_command_t rs485_command;

static USART_instance_t *rs485_usart_instance;
static supervisor_t *rs485_supervisor_instance;

tx_gimbal_t rs485_tx_message;
rx_chassis_t rs485_rx_message;

uint8_t rs485_tx_message_temp[sizeof(tx_gimbal_t)];
uint8_t rs485_rx_message_temp[sizeof(rx_chassis_t) * 2];

uint8_t rs485_receive_en_flag = 0;

static void RS485_Rx_Callback(void)
{
	HAL_UART_DMAStop(rs485_usart_instance->usart_handle);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);    
    Supervisor_Reload(rs485_supervisor_instance);         // 先喂狗
    if(rs485_usart_instance->recv_buff[0] == FRAME_HEADER && rs485_usart_instance->recv_buff[sizeof(rx_chassis_t) - 1] == FRAME_TAILER)
    {
        memcpy(&rs485_rx_message_temp, rs485_usart_instance->recv_buff, sizeof(rx_chassis_t));
        rs485_receive_en_flag = 1;
        rs485_command.online = 1; // 遥控器在线
    }
}

static void RS485_Lost_Callback(void *id)
{
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
            }
            memset(&rs485_tx_message_temp, 0, sizeof(rs485_tx_message_temp));
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
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // 发送回调函数
{
    if (huart == rs485_usart_instance->usart_handle)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
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

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	return rs485_command;
}