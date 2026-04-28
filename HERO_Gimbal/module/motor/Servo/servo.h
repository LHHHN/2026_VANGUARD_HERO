/*
 * @Author: GUATAI 2508588132@qq.com
 * @Date: 2026-04-20 23:13:36
 * @LastEditors: GUATAI 2508588132@qq.com
 * @LastEditTime: 2026-04-21 00:29:50
 * @FilePath: \HERO_Gimbal\module\motor\Servo\servo.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
/**
* @file servo.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "drv_motor.h"

#include "bsp_pwm.h"

#include "defense_center.h"

#define SERVO_CNT 2

typedef struct
{
	int16_t pwm_radio;        //电流
} servo_fillmessage_t;

typedef struct
{
	motor_model_e motor_type;        // 电机类型
	// motor_reference_e motor_reference;

	motor_control_setting_t motor_settings; // 电机控制设置
	motor_controller_t motor_controller;    // 电机控制器

	motor_error_e error_code;

	PWM_instance_t *servo_pwm_instance; // 电机PWM实例

	motor_working_type_e motor_state_flag; // 启停标志

	// servo_callback_t receive_data;            // 电机测量值
	servo_fillmessage_t transmit_data;  // 电机设定值
	// motor_feedback_data_e motor_feedback;

	supervisor_t *supervisor;
} servo_instance_t;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return servo_instance_t*
 */
servo_instance_t *Servo_Init(motor_init_config_t *config, pwm_init_config_t *pwm_config);

void Servo_Set_Ref(servo_instance_t *motor, float ref);

void Servo_Set_Tar(servo_instance_t *motor, float val, motor_reference_e type);

void Servo_Control(servo_instance_t *motor_s);

void Servo_Disable(servo_instance_t *motor);

void Servo_Enable(servo_instance_t *motor);

uint8_t Servo_Error_Judge(servo_instance_t *motor);

#endif /* __SERVO_H__ */
