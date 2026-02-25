/**
* @file LK_motor.h
 * @author lHHHn
 * @brief
 * @version 0.1
 * @date 2025-12-24
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __LK_MOTOR_H__
#define __LK_MOTOR_H__

#include "drv_motor.h"

#include "defense_center.h"

#define LK_MOTOR_MX_CNT 4 // 最多允许4个LK电机使用多电机指令,挂载在一条总线上

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define ECD_RAD_COEF_LK (2.0f * PI / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.00512f  // 电流设定值转换成扭矩的系数，这里对应的是16T

typedef struct // 9025
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float RAD_single_round;   // 单圈弧度
    float speed_dps;        // 度/s
    float speed_rps;         // speed rad/s
    float real_current;     // 实际电流A
    int16_t current;     // 电流-2048~2048 对应 -16.5A~16A
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float lk_diff;
} LK_motor_callback_t;

typedef enum
{
	LK_MOTOR_DIFF     = 0,
	LK_MOTOR_ABSOLUTE = 1,
} LK_motor_feedback_data_e;

typedef struct
{
	float position_des;
	float velocity_des;
	float torque_des;
} LK_motor_fillmessage_t;

typedef enum
{
    TORQUE_MODE = 0xA1,
}LK_ctrl_mode_e;

typedef struct
{
    motor_model_e motor_type;        // 电机类型
    motor_reference_e motor_reference;

	motor_control_setting_t motor_settings; // 电机设置
	motor_controller_t motor_controller;    // 电机控制器

	motor_error_e error_code;

	CAN_instance_t *motor_can_instance;

	motor_working_type_e motor_state_flag; // 启停标志

	LK_motor_callback_t receive_data;		// 电机反馈值
	LK_motor_fillmessage_t transmit_data;	// 电机目标值
    LK_motor_feedback_data_e motor_feedback;

	supervisor_t *supervisor;

    LK_ctrl_mode_e lk_ctrl_mode;

	uint32_t feed_cnt;
	uint32_t error_beat;
	float dt;

} LK_motor_instance_t;

typedef enum
{
	LK_CMD_ENABLE_MODE   = 0x88, // 使能,会响应指令
	LK_CMD_DISABLE_MODE  = 0x80, // 停止
	LK_CMD_CLEAR_ERROR   = 0x9B 	// 清除电机错误
} LK_motor_mode_e;

void LK_Motor_Start(LK_motor_instance_t *motor);

void LK_Motor_Stop(LK_motor_instance_t *motor);

void LK_Motor_Enable(LK_motor_instance_t *motor);

void LK_Motor_Disable(LK_motor_instance_t *motor);

void LK_Motor_SetTar(LK_motor_instance_t *motor, float val);

void LK_Motor_GetData(LK_motor_instance_t *motor);

LK_motor_instance_t *LK_Motor_Init(motor_init_config_t *config);

void LK_Motor_Control(LK_motor_instance_t *motor_s);
#endif