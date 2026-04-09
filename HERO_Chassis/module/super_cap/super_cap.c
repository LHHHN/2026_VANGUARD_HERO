/**
 ******************************************************************************
 * @file    super_cap.c
 * @brief   超电控制
 * @author  Bale
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "super_cap.h"
#include "bsp_dwt.h"
#include "task.h"

#define SUPER_CAP_MAX_ENERGY_J 2000.0f
#define SUPER_CAP_REFEREE_POWER_LIMIT_W 100U
#define SUPER_CAP_REFEREE_BUFFER_MAX_J 60.0f
#define SUPER_CAP_AUTO_RECOVER_HOLDOFF_MS 100.0f
#define SUPER_CAP_MIN_SEND_INTERVAL_US 3000ULL
#define SUPER_CAP_TX_RETRY_CNT 3U
#define SUPER_CAP_TX_RETRY_DELAY_MS 3U

/******************************************************************************
 * 联盟赛裁判系统缓冲能量计算
 * 假设当前剩余缓冲能量为Z,裁判系统缓冲能量上限为60J
 * Z = Z - (当前功率 - 限制功率) * 0.1（这里的0.1是因为裁判系统0.1s检查一次）
 * 当Z小于等于0时直接断电
 *****************************************************************************/

static uint8_t idx;
static super_cap_instance_t *super_cap_instances[SUPER_CAP_MX_CNT];
super_cap_instance_t *super_cap_instance;

// CAN 服务在收到匹配 rx_id 的报文后会回调到这里
static void Super_cap_Decode(CAN_instance_t *super_cap_can);
static void Super_cap_Lost_Callback(void *super_cap_ptr);
// 将结构体形式的控制量打包成 8 字节 CAN 数据区
static inline void Super_cap_Pack_Transmit_Data(super_cap_instance_t *instance, const super_cap_fillmessage_t *message);
static inline void Super_cap_Set_Data_Trust(super_cap_callback_t *receive_data, uint8_t trusted_supercap_power, uint8_t trusted_supercap_limit, uint8_t trusted_supercap_store);
static inline void Super_cap_Update_Measurements(super_cap_callback_t *receive_data, const uint8_t *rxbuff);
static inline void Super_cap_Handle_Error(super_cap_instance_t *instance, const uint8_t *rxbuff);
static inline uint16_t Super_cap_Calc_Referee_Energy_Buffer(super_cap_instance_t *instance, double elapsed_s);
static inline void Super_cap_Prepare_Transmit_Message(super_cap_instance_t *instance,
                                                      const super_cap_fillmessage_t *message,
                                                      super_cap_fillmessage_t *prepared_message,
                                                      double elapsed_s);
static inline uint8_t Super_cap_Transmit_Frame(super_cap_instance_t *instance, const super_cap_fillmessage_t *message);
static inline uint8_t Super_cap_Transmit_With_Recovery(super_cap_instance_t *instance,
                                                       const super_cap_fillmessage_t *normal_message);

static inline void Super_cap_Set_Data_Trust(super_cap_callback_t *receive_data, uint8_t trusted_supercap_power, uint8_t trusted_supercap_limit, uint8_t trusted_supercap_store)
{
    receive_data->chassisPowerTrusted = trusted_supercap_power;
    receive_data->chassisPowerLimitTrusted = trusted_supercap_limit;
    receive_data->capEnergyTrusted = trusted_supercap_store;
}

static inline void Super_cap_Update_Measurements(super_cap_callback_t *receive_data, const uint8_t *rxbuff)
{
    // 文档明确 chassisPower 为 float 类型，这里按原始 4 字节拷贝
    memcpy(&receive_data->chassisPower, &rxbuff[1], sizeof(receive_data->chassisPower));
    receive_data->chassisPowerLimit = (uint16_t)(rxbuff[5] | (rxbuff[6] << 8));
    receive_data->capEnergy = rxbuff[7];
    receive_data->capEnergyJ = ((float)receive_data->capEnergy / 255.0f) * SUPER_CAP_MAX_ENERGY_J;
}

static inline void Super_cap_Handle_Error(super_cap_instance_t *instance, const uint8_t *rxbuff)
{
    super_cap_callback_t *receive_data = &instance->receive_data;
    float now_ms = DWT_GetTimeline_ms();

    receive_data->statusCode = rxbuff[0];
    receive_data->errorCode = (super_cap_error_e)(rxbuff[0] & 0x03);
    instance->error_code = receive_data->errorCode;

    switch (receive_data->errorCode)
    {
    case SUPER_CAP_ERROR_NONE:
        if (now_ms < instance->auto_recover_release_time_ms)
        {
            // 自动恢复阶段内不读取超电反馈，底盘功率控制需要退回软件侧兜底。
            return;
        }
        Super_cap_Update_Measurements(receive_data, rxbuff);
        Super_cap_Set_Data_Trust(receive_data, 1U, 1U, 1U);
        return;

    case SUPER_CAP_ERROR_AUTO_RECOVERABLE:
        instance->auto_recover_release_time_ms = now_ms + SUPER_CAP_AUTO_RECOVER_HOLDOFF_MS;
        // 自动恢复错误期间保留上一帧数据除了功率，等待超电恢复后再重新采信。
        Super_cap_Set_Data_Trust(receive_data, 1U, 0U, 0U);
        return;

    case SUPER_CAP_ERROR_CMD_RECOVERABLE:
        // 本帧数据除了功率测试直接丢弃，继续保留上一帧完整测量值。
        Super_cap_Set_Data_Trust(receive_data, 1U, 0U, 0U);
        return;

    case SUPER_CAP_ERROR_UNRECOVERABLE:
    default:
        // 不可恢复错误下，底盘功率/功率上限/电容能量全部标记为不可信。
        Super_cap_Set_Data_Trust(receive_data, 0U, 0U, 0U);
        return;
    }
}

/// @brief 计算剩余缓冲能量
/// @param instance 超电实例
/// @return 剩余缓冲能量数据
static inline uint16_t Super_cap_Calc_Referee_Energy_Buffer(super_cap_instance_t *instance, double elapsed_s)
{
    super_cap_callback_t *receive_data = &instance->receive_data;

    if (receive_data->chassisPowerTrusted)
    {
        instance->refereeEnergyBufferJ = (float)((double)instance->refereeEnergyBufferJ -
                                                 ((double)receive_data->chassisPower - (double)SUPER_CAP_REFEREE_POWER_LIMIT_W) * elapsed_s);
    }

    if (instance->refereeEnergyBufferJ < 0.0f)
    {
        instance->refereeEnergyBufferJ = 0.0f;
    }
    else if (instance->refereeEnergyBufferJ > SUPER_CAP_REFEREE_BUFFER_MAX_J)
    {
        instance->refereeEnergyBufferJ = SUPER_CAP_REFEREE_BUFFER_MAX_J;
    }

    return (uint16_t)(instance->refereeEnergyBufferJ - 0.5f);
}

static inline void Super_cap_Prepare_Transmit_Message(super_cap_instance_t *instance,
                                                      const super_cap_fillmessage_t *message,
                                                      super_cap_fillmessage_t *prepared_message,
                                                      double elapsed_s)
{
    memcpy(prepared_message, message, sizeof(*prepared_message));

    // 当前阶段先固定按 100W 裁判功率上限下发，缓冲量按实际发送间隔积分估算。
    prepared_message->refereePowerLimit = SUPER_CAP_REFEREE_POWER_LIMIT_W;
    prepared_message->refereeEnergyBuffer = Super_cap_Calc_Referee_Energy_Buffer(instance, elapsed_s);

    switch (instance->error_code)
    {
    case SUPER_CAP_ERROR_CMD_RECOVERABLE:
        break;

    case SUPER_CAP_ERROR_UNRECOVERABLE:
        instance->refereeEnergyBufferJ = SUPER_CAP_REFEREE_BUFFER_MAX_J;
        prepared_message->refereeEnergyBuffer = (uint16_t)(SUPER_CAP_REFEREE_BUFFER_MAX_J);
        break;

    case SUPER_CAP_ERROR_NONE:
    case SUPER_CAP_ERROR_AUTO_RECOVERABLE:
    default:
        break;
    }
}

static inline uint8_t Super_cap_Transmit_Frame(super_cap_instance_t *instance, const super_cap_fillmessage_t *message)
{
    uint8_t retry;

    memcpy(&instance->transmit_data, message, sizeof(instance->transmit_data));
    Super_cap_Pack_Transmit_Data(instance, message);

    for (retry = 0; retry < SUPER_CAP_TX_RETRY_CNT; retry++)
    {
        if (CAN_Transmit(instance->super_cap_can_instance, 10.0f))
        {
            return 1U;
        }

        if (retry + 1U < SUPER_CAP_TX_RETRY_CNT)
        {
            vTaskDelay(pdMS_TO_TICKS(SUPER_CAP_TX_RETRY_DELAY_MS));
        }
    }

    return 0U;
}

static inline uint8_t Super_cap_Transmit_With_Recovery(super_cap_instance_t *instance,
                                                       const super_cap_fillmessage_t *normal_message)
{
    super_cap_fillmessage_t recovery_message = *normal_message;

    switch (instance->error_code)
    {
    case SUPER_CAP_ERROR_CMD_RECOVERABLE:
        recovery_message.enableDCDC = 0U;
        recovery_message.systemRestart = 0U;
        recovery_message.clearError = 0U;
        if (!Super_cap_Transmit_Frame(instance, &recovery_message))
        {
            return 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(3));

        recovery_message.clearError = 1U;
        if (!Super_cap_Transmit_Frame(instance, &recovery_message))
        {
            return 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(3));
        return Super_cap_Transmit_Frame(instance, normal_message);

    case SUPER_CAP_ERROR_UNRECOVERABLE:
        recovery_message.enableDCDC = 0U;
        recovery_message.systemRestart = 1U;
        recovery_message.clearError = 1U;
        recovery_message.refereeEnergyBuffer = (uint16_t)(SUPER_CAP_REFEREE_BUFFER_MAX_J);
        instance->refereeEnergyBufferJ = SUPER_CAP_REFEREE_BUFFER_MAX_J;
        if (!Super_cap_Transmit_Frame(instance, &recovery_message))
        {
            return 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        return Super_cap_Transmit_Frame(instance, normal_message);

    case SUPER_CAP_ERROR_NONE:
    case SUPER_CAP_ERROR_AUTO_RECOVERABLE:
    default:
        return Super_cap_Transmit_Frame(instance, normal_message);
    }
}

static void Super_cap_Decode(CAN_instance_t *super_cap_can)
{
    uint8_t *rxbuff = super_cap_can->rx_buff;
    super_cap_instance_t *instance = (super_cap_instance_t *)super_cap_can->id;

    Supervisor_Reload(instance->supervisor);
    instance->dt = DWT_GetDeltaT(&instance->feed_cnt);

    // 0x051: 超电状态上报帧，先按错误等级决定这一帧数据是否采信。
    Super_cap_Handle_Error(instance, rxbuff);
    instance->error_beat = 0;
}

static void Super_cap_Lost_Callback(void *super_cap_ptr)
{
    super_cap_instance_t *instance = (super_cap_instance_t *)super_cap_ptr;
    super_cap_callback_t *receive_data = &instance->receive_data;

    instance->error_code = SUPER_CAP_ERROR_UNRECOVERABLE;
    receive_data->errorCode = SUPER_CAP_ERROR_UNRECOVERABLE;
    receive_data->statusCode = (uint8_t)SUPER_CAP_ERROR_UNRECOVERABLE;
    Super_cap_Set_Data_Trust(receive_data, 0U, 0U, 0U);
    instance->error_beat++;
}

static inline void Super_cap_Pack_Transmit_Data(super_cap_instance_t *instance, const super_cap_fillmessage_t *message)
{
    uint8_t ctrl_flags = 0;

    // 0x061: DATA[0] 是三个 1-bit 控制位，其他字段按低字节在前打包
    if (message->enableDCDC)
    {
        ctrl_flags |= SUPER_CAP_CTRL_ENABLE_DCDC;
    }
    if (message->systemRestart)
    {
        ctrl_flags |= SUPER_CAP_CTRL_SYSTEM_RESTART;
    }
    if (message->clearError)
    {
        ctrl_flags |= SUPER_CAP_CTRL_CLEAR_ERROR;
    }

    memset(instance->super_cap_can_instance->tx_buff, 0, sizeof(instance->super_cap_can_instance->tx_buff));
    instance->super_cap_can_instance->tx_buff[0] = ctrl_flags;
    instance->super_cap_can_instance->tx_buff[1] = (uint8_t)(message->refereePowerLimit & 0xFF);
    instance->super_cap_can_instance->tx_buff[2] = (uint8_t)(message->refereePowerLimit >> 8);
    instance->super_cap_can_instance->tx_buff[3] = (uint8_t)(message->refereeEnergyBuffer & 0xFF);
    instance->super_cap_can_instance->tx_buff[4] = (uint8_t)(message->refereeEnergyBuffer >> 8);
}

/// @brief 超电初始化
/// @param 无
/// @return 超电控制can实例
void Super_cap_Config_Init(void)
{
    super_cap_init_config_t config = {
        .can_init_config.can_handle = &hfdcan2,
        .can_init_config.can_mode = FDCAN_CLASSIC_CAN,
        .can_init_config.rx_id = 0x05A,
        .can_init_config.tx_id = 0x06A,
        .supervisor_reload_count = 100};

    supervisor_init_config_t supervisor_config;

    if (idx >= SUPER_CAP_MX_CNT)
    {
        return;
    }

    super_cap_instance = (super_cap_instance_t *)malloc(sizeof(super_cap_instance_t));
    memset(super_cap_instance, 0, sizeof(super_cap_instance_t));

    config.can_init_config.can_module_callback = Super_cap_Decode;
    config.can_init_config.id = super_cap_instance;
    super_cap_instance->super_cap_can_instance = CAN_Register(&config.can_init_config);

    // 周期内没收到状态帧就触发 lost callback
    supervisor_config.reload_count = config.supervisor_reload_count;
    supervisor_config.init_count = 0;
    supervisor_config.handler_callback = Super_cap_Lost_Callback;
    supervisor_config.owner_id = super_cap_instance;
    super_cap_instance->supervisor = Supervisor_Register(&supervisor_config);

    super_cap_instance->error_code = SUPER_CAP_ERROR_NONE;
    Super_cap_Set_Data_Trust(&super_cap_instance->receive_data, 1U, 1U, 1U);
    super_cap_instance->last_send_time_us = 0U;
    super_cap_instance->refereeEnergyBufferJ = SUPER_CAP_REFEREE_BUFFER_MAX_J;
    DWT_GetDeltaT(&super_cap_instance->feed_cnt);

    super_cap_instances[idx++] = super_cap_instance;
}

/// @brief 超电数据帧发送
/// @param instance 超电实例
/// @param message 本次要下发的控制帧内容
/// @return 0==>失败,1==>成功
uint8_t Super_cap_Send_internal(super_cap_instance_t *instance, const super_cap_fillmessage_t *message)
{
    static super_cap_fillmessage_t prepared_message;
    uint64_t now_us;
    uint64_t elapsed_us = 0U;
    double elapsed_s = 0.0;
    uint8_t transmit_ok;

    if (instance == NULL || message == NULL || instance->super_cap_can_instance == NULL)
    {
        return 0;
    }

    now_us = DWT_GetTimeline_us();
    if (instance->last_send_time_us != 0U)
    {
        elapsed_us = now_us - instance->last_send_time_us;
        if (elapsed_us < SUPER_CAP_MIN_SEND_INTERVAL_US)
        {
            return 0U;
        }
        elapsed_s = (double)elapsed_us * 1e-6;
    }

    Super_cap_Prepare_Transmit_Message(instance, message, &prepared_message, elapsed_s);
    transmit_ok = Super_cap_Transmit_With_Recovery(instance, &prepared_message);
    if (transmit_ok)
    {
        instance->last_send_time_us = now_us;
    }
    return transmit_ok;
}

uint8_t Super_cap_Send_Auto(uint8_t en_DCDC, uint8_t sys_restart, uint8_t clear_err)
{
    super_cap_fillmessage_t auto_message = {
        .enableDCDC = 1U,
        .systemRestart = 0U,
        .clearError = 0U,
        .refereePowerLimit = SUPER_CAP_REFEREE_POWER_LIMIT_W,
        .refereeEnergyBuffer = (uint16_t)SUPER_CAP_REFEREE_BUFFER_MAX_J,
    };

    return Super_cap_Send_internal(super_cap_instance, &auto_message);
}

void Super_cap_Init(void)
{
    Super_cap_Config_Init();
    Super_cap_Send_Auto(1, 0, 0);
}
