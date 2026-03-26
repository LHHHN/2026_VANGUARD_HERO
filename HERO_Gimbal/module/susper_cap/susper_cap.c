/**
 ******************************************************************************
 * @file    susper_cap.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>

#include "susper_cap.h"
#include "bsp_dwt.h"

static uint8_t idx;
static susper_cap_instance_t *super_cap_instances[SUPER_CAP_MX_CNT];

// CAN 服务在收到匹配 rx_id 的报文后会回调到这里
static void Super_cap_Decode(CAN_instance_t *super_cap_can);
static void Super_cap_Lost_Callback(void *super_cap_ptr);
// 将结构体形式的控制量打包成 8 字节 CAN 数据区
static void Super_cap_Pack_Transmit_Data(susper_cap_instance_t *instance);

static void Super_cap_Decode(CAN_instance_t *super_cap_can)
{
    uint8_t *rxbuff = super_cap_can->rx_buff;
    susper_cap_instance_t *instance = (susper_cap_instance_t *)super_cap_can->id;
    super_cap_callback_t *receive_data = &instance->receive_data;

    Supervisor_Reload(instance->supervisor);
    instance->dt = DWT_GetDeltaT(&instance->feed_cnt);

    // 0x051: 超电状态上报帧，低两位为错误码，其余字节按文档固定布局解析
    receive_data->statusCode = rxbuff[0];
    receive_data->errorCode = (super_cap_error_e)(rxbuff[0] & 0x03);
    // 文档明确 chassisPower 为 float 类型，这里按原始 4 字节拷贝
    memcpy(&receive_data->chassisPower, &rxbuff[1], sizeof(receive_data->chassisPower));
    receive_data->chassisPowerLimit = (uint16_t)(rxbuff[5] | (rxbuff[6] << 8));
    receive_data->capEnergy = rxbuff[7];

    instance->error_code = receive_data->errorCode;
    instance->error_beat = 0;
}

static void Super_cap_Lost_Callback(void *super_cap_ptr)
{
    susper_cap_instance_t *instance = (susper_cap_instance_t *)super_cap_ptr;
    // 当前先只做计数，后续如果需要可在这里追加离线保护动作
    instance->error_beat++;
}

static void Super_cap_Pack_Transmit_Data(susper_cap_instance_t *instance)
{
    super_cap_fillmessage_t *transmit_data = &instance->transmit_data;
    uint8_t ctrl_flags = 0;

    // 0x061: DATA[0] 是三个 1-bit 控制位，其他字段按低字节在前打包
    if (transmit_data->enableDCDC)
    {
        ctrl_flags |= SUPER_CAP_CTRL_ENABLE_DCDC;
    }
    if (transmit_data->systemRestart)
    {
        ctrl_flags |= SUPER_CAP_CTRL_SYSTEM_RESTART;
    }
    if (transmit_data->clearError)
    {
        ctrl_flags |= SUPER_CAP_CTRL_CLEAR_ERROR;
    }

    memset(instance->motor_can_instance->tx_buff, 0, sizeof(instance->motor_can_instance->tx_buff));
    instance->motor_can_instance->tx_buff[0] = ctrl_flags;
    instance->motor_can_instance->tx_buff[1] = (uint8_t)(transmit_data->refereePowerLimit & 0xFF);
    instance->motor_can_instance->tx_buff[2] = (uint8_t)(transmit_data->refereePowerLimit >> 8);
    instance->motor_can_instance->tx_buff[3] = (uint8_t)(transmit_data->refereeEnergyBuffer & 0xFF);
    instance->motor_can_instance->tx_buff[4] = (uint8_t)(transmit_data->refereeEnergyBuffer >> 8);
}

susper_cap_instance_t *Super_cap_Init(super_cap_init_config_t *config)
{
    susper_cap_instance_t *instance;
    supervisor_init_config_t supervisor_config;
    uint16_t reload_count;

    if (config == NULL || idx >= SUPER_CAP_MX_CNT)
    {
        return NULL;
    }

    instance = (susper_cap_instance_t *)malloc(sizeof(susper_cap_instance_t));
    if (instance == NULL)
    {
        return NULL;
    }
    memset(instance, 0, sizeof(susper_cap_instance_t));

    // 与电机模块一致：在 Init 内部绑定接收回调，并交给 CAN 服务统一分发
    config->can_init_config.can_module_callback = Super_cap_Decode;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CAN_Register(&config->can_init_config);

    // 离线检测逻辑与其他 CAN 模块保持一致：周期内没收到状态帧就触发 lost callback
    reload_count = config->supervisor_reload_count ? config->supervisor_reload_count : 20U;
    supervisor_config.reload_count = reload_count;
    supervisor_config.init_count = 0;
    supervisor_config.handler_callback = Super_cap_Lost_Callback;
    supervisor_config.owner_id = instance;
    instance->supervisor = Supervisor_Register(&supervisor_config);

    instance->error_code = SUPER_CAP_ERROR_NONE;
    DWT_GetDeltaT(&instance->feed_cnt);

    super_cap_instances[idx++] = instance;
    return instance;
}

void Super_cap_Set_Transmit_Message(susper_cap_instance_t *instance, const super_cap_fillmessage_t *message)
{
    if (instance == NULL || message == NULL)
    {
        return;
    }

    // 先缓存到实例，真正发送时再统一打包到 tx_buff
    memcpy(&instance->transmit_data, message, sizeof(instance->transmit_data));
}

uint8_t Super_cap_Transmit(susper_cap_instance_t *instance, float timeout)
{
    if (instance == NULL || instance->motor_can_instance == NULL)
    {
        return 0;
    }

    // 发送前始终以结构体当前值重新打包，避免 tx_buff 和 transmit_data 脱节
    Super_cap_Pack_Transmit_Data(instance);
    return CAN_Transmit(instance->motor_can_instance, timeout);
}
