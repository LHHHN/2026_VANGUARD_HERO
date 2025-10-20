/**
* @file LK_motor.h
 * @author LHHHN (896047872@qq.com)
 * @brief 该电机驱动主要是LK电机的MF9025
 * @version 0.1
 * @date 2025-10-19
 *
 * @copyright Copyright (c) 2023
 *
*/

#ifndef __LK_MOTOR_H__
#define __LK_MOTOR_H__

#include "drv_motor.h"
#include "defense_center.h"

#define LK_MOTOR_MAX_CNT 4 // 最多允许4个LK电机使用多电机指令,挂载在一条总线上
#define I_MIN -2000 // 最小电流设定值
#define I_MAX 2000      // 最大电流设定值
#define CURRENT_SMOOTH_COEF 0.9f    // 电流滤波系数
#define SPEED_SMOOTH_COEF 0.85f     // 速度滤波系数
#define REDUCTION_RATIO_DRIVEN 1  // 电机齿轮减速比
#define ECD_TO_ANGLE_COEF_LK (360.0f / 65536.0f) // LK电机编码器转角系数
#define CURRENT_TORQUE_COEF_LK 0.00512f  // 电流设定值转换成扭矩的系数，这里对应的是16T

typedef enum
{
	LK_ERROR_NONE               = 00000000,     //无错误
	LK_LOW_VOLTAGE_STATE        = 00000001,     //低电压状态 
    LK_HIGH_VOLTAGE_STATE       = 00000010,     //高电压状态
	LK_DRIVE_OVER_HEATING       = 00000100,     //驱动过热
    LK_MOTOR_OVER_HEATING       = 00001000,     //电机过热
    LK_MOTOR_OVER_CURRENT       = 00010000,     //电机过流
    LK_MOTOR_SHORT_CIRCUIT      = 00100000,     //电机短路
    LK_MOTOR_LOCKED_ROTOR       = 01000000,     //电机堵转
    LK_MOTOR_LOSS               = 10000000,     //电机失联
} LK_error_e;

typedef enum
{
    LK_CMD_DISABLE = 0x80, //电机失能，能回复控制命令但不执行
    LK_CMD_ENABLE = 0x88, //电机使能，能正常工作
    LK_CMD_STOP = 0x81,    //电机停止，再次发送命令可工作
    LK_CMD_ZERO_POSITION = 0x19, //电机设置零点（需要重新上电，不建议使用）
    LK_CMD_READ_ERROR = 0x9A, //读取错误码
    LK_CMD_CLEAR_ERROR = 0x9B, //清除错误码
}LK_CMD_mode_e;

typedef enum
{
    LK_CONTROL_IQ = 0xA1, // 转矩闭环控制模式
    LK_CONTROL_SPEED_IQ= 0xA2,     // 速度闭环控制模式
    LK_CONTROL_MULTI_ANGLE = 0xA3,  // 多圈位置闭环控制模式
    LK_CONTROL_MULTI_SPEEDANGLE = 0xA4, // 多圈位置速度闭环控制模式
    LK_CONTROL_SINGLE_ANGLE = 0xA5, // 单圈位置闭环控制模式
    LK_CONTROL_SINGLE_SPEEDANGLE = 0xA6,    // 单圈位置速度闭环控制模式
    LK_CONTROL_INCREMENT_ANGlE = 0xA7, // 位置增量闭环控制模式
    LK_CONTROL_INCREMENT_SPEEDANGLE = 0xA8, // 位置增量速度闭环控制模式
}LK_control_mode_e;

typedef struct
{
	float position_des;
	float velocity_des;
    float iq_des;
} LK_motor_fillmessage_t; // 电机目标值结构体

typedef struct // 9025
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float lk_diff;
} LK_motor_callback_t;

typedef struct
{
    LK_motor_callback_t receive_data; // 电机测量值
    LK_motor_fillmessage_t transmit_data; // 电机目标值

    motor_reference_e motor_reference; // 电机参考系

    motor_control_setting_t motor_settings; // 电机设置
	motor_controller_t motor_controller;    // 电机控制器

    CAN_instance_t *motor_can_instance;     // 电机CAN实例

    motor_working_type_e motor_state_flag; // 启停标志
    motor_error_detection_type_e motor_error_detection; // 
    
    LK_control_mode_e control_mode; // 电机控制模式

    supervisor_t *supervisor;   // 守护线程

    uint32_t feed_cnt;        // 计数器
	float dt;

    LK_error_e error_code;      // 错误码
    uint32_t error_beat;      // 错误计数

} LK_motor_instance_t;



#endif