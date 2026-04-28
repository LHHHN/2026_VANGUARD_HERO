// #ifndef __SUPER_CAP_H_
// #define __SUPER_CAP_H_

// #include "defense_center.h"
// #include "bsp_can.h"

// #define SUPER_CAP_MX_CNT 1
// #define ENABLE_DCDC 1
// #define SYS_RESTART 1
// #define CLEAR_ERR 1

// // 超电上报状态中的错误等级，仅取 statusCode 的低两位
// typedef enum
// {
//     SUPER_CAP_ERROR_NONE = 0,             // 无错误
//     SUPER_CAP_ERROR_AUTO_RECOVERABLE = 1, // 错误，可自动恢复
//     SUPER_CAP_ERROR_CMD_RECOVERABLE = 2,  // 错误，可通过发信息恢复
//     SUPER_CAP_ERROR_UNRECOVERABLE = 3,    // 错误，不可恢复
// } super_cap_error_e;

// typedef struct
// {
//     // 对应超电发送给主控的 0x051 状态帧
//     uint8_t statusCode;               // 原始状态码，保留完整字节便于后续扩展
//     super_cap_error_e errorCode;      // 由 statusCode 低两位解析出的错误等级
//     float chassisPower;               // DATA[1..4]，底盘功率，单位 W
//     int16_t chassisPowerLimit;        // DATA[5..6]，底盘最大可用功率，单位 W
//     uint8_t capEnergy;                // DATA[7]，电容能量映射值 0-255
//     float capEnergyJ;                 // 将 0-255 的能量映射为 0-2000J 后的结果
//     uint8_t chassisPowerTrusted;      // 1 表示该值可信
//     uint8_t chassisPowerLimitTrusted; // 1 表示该值可信
//     uint8_t capEnergyTrusted;         // 1 表示该值可信
// } super_cap_callback_t;

// typedef struct
// {
//     // 对应主控发送给超电的 0x061 控制帧
//     uint8_t enableDCDC;           // DATA[0] bit0，DCDC 使能
//     uint8_t systemRestart;        // DATA[0] bit1，系统重启
//     uint8_t clearError;           // DATA[0] bit2，清除可恢复错误
//     uint16_t refereePowerLimit;   // DATA[1..2]，裁判限制功率，单位 W
//     uint16_t refereeEnergyBuffer; // DATA[3..4]，裁判能量缓冲，单位 J
// } super_cap_fillmessage_t;

// typedef enum
// {
//     // 直接对应 0x061 控制帧 DATA[0] 的 bit 定义
//     SUPER_CAP_CTRL_ENABLE_DCDC = 0x01,
//     SUPER_CAP_CTRL_SYSTEM_RESTART = 0x02,
//     SUPER_CAP_CTRL_CLEAR_ERROR = 0x04,
// } super_cap_control_flag_e;

// typedef struct
// {
//     can_init_config_t can_init_config;
//     uint16_t supervisor_reload_count; // 0 时使用默认离线检测计数
// } super_cap_init_config_t;

// typedef struct
// {
//     super_cap_error_e error_code;

//     CAN_instance_t *super_cap_can_instance; // 超电注册到 CAN 服务后的实例
//     super_cap_callback_t receive_data;      // 最近一次收到的状态帧
//     super_cap_fillmessage_t transmit_data;  // 最近一次下发的控制帧内容

//     supervisor_t *supervisor;

//     uint32_t feed_cnt;
//     uint32_t error_beat;                // 离线回调累计触发次数
//     float dt;                           // 两次收到状态帧的时间间隔
//     float auto_recover_release_time_ms; // 自动恢复错误后的数据屏蔽截止时刻
//     uint64_t last_send_time_us;         // 上一次成功发送控制帧的时间戳(us)
//     float refereeEnergyBufferJ;         // 软件侧估算的裁判系统缓冲能量

//     uint8_t sender_group;
//     uint8_t message_num;
// } super_cap_instance_t;

// extern super_cap_instance_t *super_cap_instance;

// // 初始化超电 CAN 设备，内部会完成 CAN_Register 和离线监测注册
// void Super_cap_Config_Init(void);

// void Super_cap_Init(void);

// // 参考电机模块发送模式：调用时一次性打包并下发 CAN 报文
// uint8_t Super_cap_Send_internal(super_cap_instance_t *instance, const super_cap_fillmessage_t *message);

// // 无参发送接口：默认控制帧并调用 Super_cap_Send
// uint8_t Super_cap_Send_Auto(uint8_t en_DCDC, uint8_t sys_restart, uint8_t clear_err);

// #endif
