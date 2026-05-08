#include "rs485.h"

#include "CRC.h"
#include "bsp_usart.h"
#include "defense_center.h"

#define BUFF_RS485_RECEIVE_SIZE 128U
#define RS485_GIMBAL_FRAME_SIZE (sizeof(rs485_frame_header_t) + sizeof(rs485_gimbal_cmd_payload_t) + sizeof(uint16_t) + sizeof(uint8_t))
#define RS485_CHASSIS_FRAME_SIZE (sizeof(rs485_frame_header_t) + sizeof(rs485_chassis_status_payload_t) + sizeof(uint16_t) + sizeof(uint8_t))
#define RS485_GIMBAL_MODE_FLAGS(chassis, gimbal, shoot) \
    ((uint8_t)(((chassis) & 0x0FU) | (((gimbal) & 0x03U) << 4U) | (((shoot) & 0x03U) << 6U)))
#define RS485_GIMBAL_CONTROL_FLAGS(fire, launched, remote, ui, auto_aim) \
    ((uint8_t)(((fire) & 0x03U) | (((launched) & 0x03U) << 2U) | (((remote) & 0x01U) << 4U) | (((ui) & 0x01U) << 5U) | (((auto_aim) & 0x03U) << 6U)))

typedef struct
{
    rs485_frame_header_t header;
    rs485_gimbal_cmd_payload_t payload;
    uint16_t crc16;
    uint8_t frame_tailer;
} __attribute__((packed)) rs485_gimbal_wire_t;

typedef struct
{
    rs485_frame_header_t header;
    rs485_chassis_status_payload_t payload;
    uint16_t crc16;
    uint8_t frame_tailer;
} __attribute__((packed)) rs485_chassis_wire_t;

rs485_command_t rs485_command;

static USART_instance_t *rs485_usart_instance;
static supervisor_t *rs485_supervisor_instance;

tx_gimbal_t rs485_tx_message;
rx_chassis_t rs485_rx_message;

static uint8_t rs485_tx_message_temp[RS485_GIMBAL_FRAME_SIZE];
static uint8_t rs485_tx_pending_message[RS485_GIMBAL_FRAME_SIZE];
static uint8_t rs485_rx_message_temp[RS485_CHASSIS_FRAME_SIZE];

static volatile uint8_t rs485_receive_en_flag = 0;

static uint8_t rs485_parse_buff[RS485_CHASSIS_FRAME_SIZE];
static uint16_t rs485_parse_index = 0;
static volatile uint8_t rs485_tx_pending_flag = 0U;

static void RS485_Reset_Parse_State(void)
{
    rs485_parse_index = 0;
    memset(rs485_parse_buff, 0, sizeof(rs485_parse_buff));
}

static void RS485_Recover_Parse_State(uint16_t valid_size)
{
    if (valid_size > RS485_CHASSIS_FRAME_SIZE)
    {
        valid_size = RS485_CHASSIS_FRAME_SIZE;
    }

    for (uint16_t i = 1; i < valid_size; i++)
    {
        if (rs485_parse_buff[i] == FRAME_HEADER)
        {
            rs485_parse_index = valid_size - i;
            memmove(rs485_parse_buff, &rs485_parse_buff[i], rs485_parse_index);
            return;
        }
    }

    RS485_Reset_Parse_State();
}

static uint8_t RS485_Verify_Chassis_Frame(const uint8_t *frame)
{
    const rs485_chassis_wire_t *wire = (const rs485_chassis_wire_t *)frame;

    if (wire->header.frame_header != FRAME_HEADER ||
        wire->header.version != RS485_PROTOCOL_VERSION ||
        wire->header.frame_type != RS485_FRAME_TYPE_CHASSIS_STATUS ||
        wire->header.payload_len != sizeof(rs485_chassis_status_payload_t) ||
        wire->frame_tailer != FRAME_TAILER)
    {
        return 0;
    }

    return Verify_CRC16_Check_Sum((uint8_t *)frame, RS485_CHASSIS_FRAME_SIZE - 1U) ? 1U : 0U;
}

static uint8_t RS485_Verify_Chassis_Header(const uint8_t *frame)
{
    const rs485_frame_header_t *header = (const rs485_frame_header_t *)frame;

    if (header->frame_header != FRAME_HEADER ||
        header->version != RS485_PROTOCOL_VERSION ||
        header->frame_type != RS485_FRAME_TYPE_CHASSIS_STATUS ||
        header->payload_len != sizeof(rs485_chassis_status_payload_t))
    {
        return 0U;
    }

    return 1U;
}

static void RS485_Update_Seq_Status(uint8_t seq)
{
    uint8_t expected;

    if (rs485_command.rx_ok_count > 0U)
    {
        expected = (uint8_t)(rs485_command.last_rx_seq + 1U);
        if (seq != expected)
        {
            rs485_command.seq_lost_count++;
        }
    }

    rs485_command.last_rx_seq = seq;
}

static void RS485_Copy_Chassis_Payload(const rs485_chassis_wire_t *wire)
{
    rs485_rx_message.chassis_beat = wire->payload.chassis_beat;
    rs485_rx_message.chassis_flag = wire->payload.chassis_flag;
    rs485_rx_message.seq = wire->header.seq;
    rs485_rx_message.ack_seq = wire->header.ack_seq;
}

static void RS485_Fill_Gimbal_Payload(rs485_gimbal_cmd_payload_t *payload)
{
    payload->mode_flags = RS485_GIMBAL_MODE_FLAGS(rs485_tx_message.chassis_mode,
                                                  rs485_tx_message.gimbal_mode,
                                                  rs485_tx_message.shoot_mode);
    payload->chassis_target_vx = rs485_tx_message.chassis_target_vx;
    payload->chassis_target_vy = rs485_tx_message.chassis_target_vy;
    payload->chassis_target_wz = rs485_tx_message.chassis_target_wz;
    payload->chassis_target_leg_angle = rs485_tx_message.chassis_target_leg_angle;
    payload->gimbal_target_yaw = rs485_tx_message.gimbal_target_yaw;
    payload->gimbal_target_yaw_speed = rs485_tx_message.gimbal_target_yaw_speed;
    payload->gimbal_measure_yaw = rs485_tx_message.gimbal_measure_yaw;
    payload->gimbal_measure_yaw_speed = rs485_tx_message.gimbal_measure_yaw_speed;
    payload->gimbal_measure_pitch = rs485_tx_message.gimbal_measure_pitch;
    payload->control_flags = RS485_GIMBAL_CONTROL_FLAGS(rs485_tx_message.shoot_fire_en_flag,
                                                        rs485_tx_message.shoot_launched_flag,
                                                        rs485_tx_message.control_remote_flag,
                                                        rs485_tx_message.ui_refresh_flag,
                                                        rs485_tx_message.auto_aiming_flag);
}

static void RS485_Parse_Byte(uint8_t data)
{
    if (rs485_parse_index == 0U)
    {
        if (data != FRAME_HEADER)
        {
            return;
        }
    }

    rs485_parse_buff[rs485_parse_index++] = data;

    if (rs485_parse_index == sizeof(rs485_frame_header_t) &&
        RS485_Verify_Chassis_Header(rs485_parse_buff) == 0U)
    {
        RS485_Recover_Parse_State(rs485_parse_index);
        return;
    }

    if (rs485_parse_index < RS485_CHASSIS_FRAME_SIZE)
    {
        return;
    }

    if (RS485_Verify_Chassis_Frame(rs485_parse_buff))
    {
        memcpy(rs485_rx_message_temp, rs485_parse_buff, RS485_CHASSIS_FRAME_SIZE);
        rs485_receive_en_flag = 1U;
        RS485_Reset_Parse_State();
        return;
    }

    rs485_command.crc_error_count++;
    RS485_Recover_Parse_State(rs485_parse_index);
}

static void RS485_Parse_Bytes(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        RS485_Parse_Byte(data[i]);
    }
}

static void RS485_Parse_Circular_Rx_Data(void)
{
    uint16_t start = rs485_usart_instance->rx_start_pos;
    uint16_t end = rs485_usart_instance->current_size;
    uint16_t buff_size = rs485_usart_instance->recv_buff_size;

    if (start >= buff_size)
    {
        start = 0;
    }

    if (start == end)
    {
        return;
    }

    if (start < end)
    {
        RS485_Parse_Bytes(&rs485_usart_instance->recv_buff[start], end - start);
    }
    else
    {
        RS485_Parse_Bytes(&rs485_usart_instance->recv_buff[start], buff_size - start);
        if (end > 0U)
        {
            RS485_Parse_Bytes(rs485_usart_instance->recv_buff, end);
        }
    }
}

static void RS485_Rx_Callback(void)
{
    if (rs485_usart_instance->usart_handle->hdmarx != NULL &&
        rs485_usart_instance->usart_handle->hdmarx->Init.Mode == DMA_CIRCULAR)
    {
        RS485_Parse_Circular_Rx_Data();
        return;
    }

    if (rs485_usart_instance->current_size > rs485_usart_instance->recv_buff_size)
    {
        rs485_usart_instance->current_size = rs485_usart_instance->recv_buff_size;
    }

    RS485_Parse_Bytes(rs485_usart_instance->recv_buff, rs485_usart_instance->current_size);
}

static void RS485_Lost_Callback(void *id)
{
    (void)id;

    memset(&rs485_rx_message, 0, sizeof(rs485_rx_message));
    memset(rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
    RS485_Reset_Parse_State();
    rs485_receive_en_flag = 0U;
    rs485_command.online = 0U;
    rs485_command.recover_count++;

    if (rs485_usart_instance != NULL)
    {
        HAL_UART_AbortReceive(rs485_usart_instance->usart_handle);
        rs485_usart_instance->rx_start_pos = 0;
        rs485_usart_instance->rx_last_pos = 0;
        rs485_usart_instance->current_size = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(rs485_usart_instance->usart_handle,
                                     rs485_usart_instance->recv_buff,
                                     rs485_usart_instance->recv_buff_size);
        __HAL_DMA_DISABLE_IT(rs485_usart_instance->usart_handle->hdmarx, DMA_IT_HT);
    }
}

static void RS485_Store_Pending_Frame(const uint8_t *frame)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(rs485_tx_pending_message, frame, RS485_GIMBAL_FRAME_SIZE);
    rs485_tx_pending_flag = 1U;
    __set_PRIMASK(primask);
}

static uint8_t RS485_Send_Frame(const uint8_t *frame)
{
    HAL_StatusTypeDef status;
    uint32_t primask;

    if (rs485_usart_instance == NULL || frame == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (USART_Is_Ready(rs485_usart_instance) == 0U)
    {
        __set_PRIMASK(primask);
        rs485_command.tx_busy_count++;
        return 0U;
    }

    memcpy(rs485_tx_message_temp, frame, RS485_GIMBAL_FRAME_SIZE);
    status = USART_Send(rs485_usart_instance, rs485_tx_message_temp, RS485_GIMBAL_FRAME_SIZE, USART_TRANSFER_DMA);
    __set_PRIMASK(primask);

    if (status != HAL_OK)
    {
        rs485_command.tx_busy_count++;
        return 0U;
    }

    return 1U;
}

void RS485_Handle_Rx_Data(void)
{
    rs485_chassis_wire_t wire;

    if (rs485_receive_en_flag == 0U)
    {
        return;
    }

    __disable_irq();
    memcpy(&wire, rs485_rx_message_temp, sizeof(wire));
    memset(rs485_rx_message_temp, 0, sizeof(rs485_rx_message_temp));
    rs485_receive_en_flag = 0U;
    __enable_irq();

    if (RS485_Verify_Chassis_Frame((const uint8_t *)&wire) == 0U)
    {
        rs485_command.crc_error_count++;
        return;
    }

    RS485_Update_Seq_Status(wire.header.seq);
    rs485_command.rx_ok_count++;
    RS485_Copy_Chassis_Payload(&wire);
    Supervisor_Reload(rs485_supervisor_instance);
    rs485_command.online = 1U;
}

void RS485_Handle_Tx_Data(void)
{
    rs485_gimbal_wire_t wire;
    uint8_t tx_frame[RS485_GIMBAL_FRAME_SIZE];

    if (rs485_usart_instance == NULL)
    {
        return;
    }

    memset(&wire, 0, sizeof(wire));
    wire.header.frame_header = FRAME_HEADER;
    wire.header.version = RS485_PROTOCOL_VERSION;
    wire.header.frame_type = RS485_FRAME_TYPE_GIMBAL_CMD;
    wire.header.seq = (uint8_t)(rs485_command.last_tx_seq + 1U);
    wire.header.ack_seq = rs485_command.last_rx_seq;
    wire.header.payload_len = sizeof(rs485_gimbal_cmd_payload_t);
    RS485_Fill_Gimbal_Payload(&wire.payload);
    wire.frame_tailer = FRAME_TAILER;

    memcpy(tx_frame, &wire, sizeof(wire));
    Append_CRC16_Check_Sum(tx_frame, RS485_GIMBAL_FRAME_SIZE - 1U);

    if (RS485_Send_Frame(tx_frame))
    {
        rs485_tx_pending_flag = 0U;
        rs485_command.last_tx_seq = wire.header.seq;
        rs485_tx_message.seq = wire.header.seq;
        rs485_tx_message.ack_seq = wire.header.ack_seq;
        return;
    }

    RS485_Store_Pending_Frame(tx_frame);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t tx_frame[RS485_GIMBAL_FRAME_SIZE];
    uint8_t seq;
    uint8_t ack_seq;

    if (rs485_usart_instance == NULL ||
        huart != rs485_usart_instance->usart_handle ||
        rs485_tx_pending_flag == 0U)
    {
        return;
    }

    memcpy(tx_frame, rs485_tx_pending_message, RS485_GIMBAL_FRAME_SIZE);
    rs485_tx_pending_flag = 0U;

    if (RS485_Send_Frame(tx_frame))
    {
        seq = ((const rs485_gimbal_wire_t *)tx_frame)->header.seq;
        ack_seq = ((const rs485_gimbal_wire_t *)tx_frame)->header.ack_seq;
        rs485_command.last_tx_seq = seq;
        rs485_tx_message.seq = seq;
        rs485_tx_message.ack_seq = ack_seq;
        return;
    }

    RS485_Store_Pending_Frame(tx_frame);
}

rs485_command_t RS485_Register(UART_HandleTypeDef *rs485_handle)
{
    usart_init_config_t conf;
    conf.module_callback = RS485_Rx_Callback;
    conf.usart_handle = rs485_handle;
    conf.recv_buff_size = BUFF_RS485_RECEIVE_SIZE;
    rs485_usart_instance = USART_Register(&conf);

    supervisor_init_config_t supervisor_conf = {
        .reload_count = 3,
        .handler_callback = RS485_Lost_Callback,
        .owner_id = NULL,
    };
    rs485_supervisor_instance = Supervisor_Register(&supervisor_conf);

    return rs485_command;
}
