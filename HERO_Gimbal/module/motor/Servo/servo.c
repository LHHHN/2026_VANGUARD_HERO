/**
******************************************************************************
 * @file    servo.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include "servo.h"

#include "bsp_dwt.h"

static uint8_t idx = 0;
static servo_instance_t *servo_instances[SERVO_CNT] = {NULL};

static void Decode_Servo(PWM_instance_t *servo_pwm)
{
    ;
}

static void Servo_Lost_Callback(void *motor_ptr)
{
    ;
}

// 电机初始化,返回一个电机实例
servo_instance_t *Servo_Init(motor_init_config_t *config, pwm_init_config_t *pwm_config)
{
	servo_instance_t *instance = (servo_instance_t *) malloc(sizeof(servo_instance_t));
	memset(instance, 0, sizeof(servo_instance_t));

	if (instance == NULL)
	{
		return NULL;
	}

	// motor basic setting 电机基本设置
	instance->motor_type     = config->motor_type;                     // 6020 or 2006 or 3508
	instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

	// motor controller init 电机控制器初始化
	instance->motor_controller.current_PID = PID_Init(config->controller_param_init_config.current_PID);
	instance->motor_controller.speed_PID   = PID_Init(config->controller_param_init_config.speed_PID);
	instance->motor_controller.angle_PID   = PID_Init(config->controller_param_init_config.angle_PID);
	instance->motor_controller.torque_PID  = PID_Init(config->controller_param_init_config.torque_PID);

	//电机控制闭环时的非电机本身反馈数据指针
	instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
	instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

	//电机控制闭环时的前馈控制器或前馈控制量指针
	instance->motor_controller.torque_feedforward_ptr = config->controller_param_init_config.torque_feedforward_ptr;
	instance->motor_controller.speed_feedforward_ptr  = config->controller_param_init_config.speed_feedforward_ptr;

    pwm_config->id = instance;
    instance->servo_pwm_instance = PWM_Register(pwm_config);

	// 注册守护线程
	supervisor_init_config_t supervisor_config = {
		.handler_callback = Servo_Lost_Callback,
		.owner_id = instance,
		.reload_count = 20, // 20ms未收到数据则丢失
	};
	instance->supervisor = Supervisor_Register(&supervisor_config);

	Servo_Disable(instance);
	servo_instances[idx++] = instance;
	return instance;
}

void Servo_Disable(servo_instance_t *motor)
{
	motor->motor_state_flag = MOTOR_DISABLE;
}

void Servo_Enable(servo_instance_t *motor)
{
	motor->motor_state_flag = MOTOR_ENABLE;
}

// 设置参考值
void Servo_Set_Ref(servo_instance_t *motor, float ref)
{
	motor->motor_controller.pid_ref = ref;
}

// 设置参考值
void Servo_Set_Tar(servo_instance_t *motor, float val, motor_reference_e type)
{
	if (type == ABS)
	{
		motor->motor_controller.pid_ref = val;
	}
	else if (type == INCR)
	{
		motor->motor_controller.pid_ref += val;
	}
}

// 异常检测
uint8_t Servo_Error_Judge(servo_instance_t *motor)
{
    return 0;
}

// 为所有电机实例计算三环PID,发送控制报文
void Servo_Control(servo_instance_t *motor_s)
{
    if (motor_s == NULL)
    {
        for (size_t i = 0 ; i < idx ; ++i)
        {
            if(servo_instances[i]->motor_controller.pid_ref < 0.0f || servo_instances[i]->motor_controller.pid_ref > 1.0f)
            {
                servo_instances[i]->motor_controller.pid_ref = 0.0f; // 超出范围直接置零,保护电机
            }
            PWM_Set_DutyRatio(servo_instances[i]->servo_pwm_instance, servo_instances[i]->motor_controller.pid_ref);
        }
    }
    else
    {
        if(motor_s->motor_controller.pid_ref < 0.0f || motor_s->motor_controller.pid_ref > 1.0f)
        {
            motor_s->motor_controller.pid_ref = 0.0f; // 超出范围直接置零,保护电机
        }
        motor_s->servo_pwm_instance->dutyratio = motor_s->motor_controller.pid_ref;
        PWM_Set_DutyRatio(motor_s->servo_pwm_instance, motor_s->servo_pwm_instance->dutyratio);
    }
}
