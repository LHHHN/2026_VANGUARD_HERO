/**
******************************************************************************
 * @file    LK_motor.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "LK_motor.h"

#include "bsp_dwt.h"

static uint8_t idx;
static LK_motor_instance_t *lk_motor_instances[LK_MOTOR_MX_CNT];


static void LK_Motor_Set_Mode(LK_motor_mode_e cmd, LK_motor_instance_t *motor)
{
	memset(motor->motor_can_instance->tx_buff, 0x00, 8); // 发送电机指令的时候后面7bytes都是0x00
	motor->motor_can_instance->tx_buff[0] = (uint8_t) cmd; // 第一位是命令id
	CAN_Transmit(motor->motor_can_instance, 2);
}
void LK_Motor_Start(LK_motor_instance_t *motor)
{
    if (motor == NULL)
	{
		for (size_t i = 0 ; i < idx ; ++i)
		{
			lk_motor_instances[i]->motor_state_flag = MOTOR_ENABLE;
		}
	}
	else
	{
		motor->motor_state_flag = MOTOR_ENABLE;
	}
}

void LK_Motor_Stop(LK_motor_instance_t *motor)
{
	if (motor == NULL)
	{
		for (size_t i = 0 ; i < idx ; ++i)
		{
			lk_motor_instances[i]->motor_state_flag = MOTOR_DISABLE;
		}
	}
	else
	{
		motor->motor_state_flag = MOTOR_DISABLE;
	}
}

void LK_Motor_Enable(LK_motor_instance_t *motor)
{
	if (motor == NULL)
	{
		for (size_t i = 0 ; i < idx ; ++i)
		{
			LK_Motor_Set_Mode(LK_CMD_ENABLE_MODE, lk_motor_instances[i]);
		}
	}
	else
	{
		LK_Motor_Set_Mode(LK_CMD_ENABLE_MODE, motor);
	}
}

void LK_Motor_Disable(LK_motor_instance_t *motor)
{
	if (motor == NULL)
	{
		for (size_t i = 0 ; i < idx ; ++i)
		{
			LK_Motor_Set_Mode(LK_CMD_DISABLE_MODE, lk_motor_instances[i]);
		}
	}
	else
	{
		LK_Motor_Set_Mode(LK_CMD_DISABLE_MODE, motor);
	}
}

void LK_Motor_Clear_Error(LK_motor_instance_t *motor)
{
	LK_Motor_Set_Mode(LK_CMD_CLEAR_ERROR, motor);
}

void LK_Motor_SetTar(LK_motor_instance_t *motor, float val)
{
	motor->motor_controller.pid_ref = val;
}

void LK_Motor_GetData(LK_motor_instance_t *motor)
{
	memset(motor->motor_can_instance->tx_buff, 0x00, 8); // 发送电机指令的时候后面7bytes都是0x00
	motor->motor_can_instance->tx_buff[0] = 0x9C; // 第一位是命令id
	CAN_Transmit(motor->motor_can_instance, 2);
}

//int16_t iq_temp = 0;
void LK_Ctrl(LK_motor_instance_t *motor)
{
    if(motor->lk_ctrl_mode == TORQUE_MODE)
    {
		int16_t iq_temp = 0;
		iq_temp = (int16_t)motor->transmit_data.torque_des;
        memset(motor->motor_can_instance->tx_buff, 0x00, 8);
        motor->motor_can_instance->tx_buff[0] = TORQUE_MODE;          // 控制命令字
		motor->motor_can_instance->tx_buff[4] = iq_temp;     // 低八位
        motor->motor_can_instance->tx_buff[5] = (iq_temp >> 8); // 高八位
        CAN_Transmit(motor->motor_can_instance, 1);
    }
    
}

static void LK_Motor_Decode(CAN_instance_t *motor_can)
{
	uint8_t *rxbuff                   = motor_can->rx_buff;
	LK_motor_instance_t *motor        = (LK_motor_instance_t *) motor_can->id;
	LK_motor_callback_t *receive_data = &(motor->receive_data); // 将can实例中保存的id转换成电机实例的指针

    Supervisor_Reload(motor->supervisor);
	motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    if (motor->error_code & MOTOR_LOST_ERROR)
	{
		motor->error_code &= ~(MOTOR_LOST_ERROR);
	}

    if(rxbuff[0] == 0x9C || rxbuff[0] == 0xA1)
    {
         receive_data->last_ecd = receive_data->ecd;

        receive_data->ecd = (uint16_t)((rxbuff[7] << 8) | rxbuff[6]);
        receive_data->angle_single_round = ECD_ANGLE_COEF_LK * receive_data->ecd;
        receive_data->RAD_single_round = ECD_RAD_COEF_LK * receive_data->ecd;

        receive_data->speed_dps = (1 - SPEED_SMOOTH_COEF) * receive_data->speed_dps +
                            SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[5] << 8 | rxbuff[4]));
        receive_data->speed_rps = DEGREE_2_RAD * receive_data->speed_dps;

        receive_data->current = (1 - CURRENT_SMOOTH_COEF) * receive_data->current +
                                CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[3] << 8 | rxbuff[2]));
        receive_data->temperature = rxbuff[1];
    }
   
}

static void LK_Motor_Lost_Callback(void *motor_ptr)
{
	LK_motor_instance_t *motor = (LK_motor_instance_t *) motor_ptr;
	motor->error_code |= MOTOR_LOST_ERROR;
}


LK_motor_instance_t *LK_Motor_Init(motor_init_config_t *config)
{
	LK_motor_instance_t *instance = (LK_motor_instance_t *) malloc(sizeof(LK_motor_instance_t));
	memset(instance, 0, sizeof(LK_motor_instance_t));

	if (instance == NULL)
	{
		return NULL;
	}

	instance->motor_type     = config->motor_type;
	instance->motor_settings = config->controller_setting_init_config;

	instance->motor_controller.torque_PID = PID_Init(
	                                                 config->controller_param_init_config.torque_PID);
	instance->motor_controller.speed_PID = PID_Init(
	                                                config->controller_param_init_config.speed_PID);
	instance->motor_controller.angle_PID = PID_Init(
	                                                config->controller_param_init_config.angle_PID);

	instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
	instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
	instance->motor_controller.torque_feedforward_ptr   = config->controller_param_init_config.torque_feedforward_ptr;
	instance->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;


	// instance->dm_tx_id = config->can_init_config.tx_id;
	// instance->dm_rx_id = config->can_init_config.rx_id;

	config->can_init_config.can_module_callback = LK_Motor_Decode;
	config->can_init_config.id                  = instance;
	instance->motor_can_instance                = CAN_Register(&config->can_init_config);

	// 注册守护线程
	supervisor_init_config_t supervisor_config = {
		.handler_callback = LK_Motor_Lost_Callback,
		.owner_id = instance,
		.reload_count = 20, // 20ms未收到数据则丢失
	};
	instance->supervisor = Supervisor_Register(&supervisor_config);

	instance->error_code = MOTOR_ERROR_NONE;

	DWT_GetDeltaT(&instance->feed_cnt);

	LK_Motor_Stop(instance);
	DWT_Delay(0.1);
	lk_motor_instances[idx++] = instance;
	return instance;
}

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
// 为所有电机实例计算三环PID,发送控制报文
void LK_Motor_Control(LK_motor_instance_t *motor_s)
{
	// 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
	LK_motor_instance_t *motor;
	motor_control_setting_t *motor_setting; // 电机控制参数
	motor_controller_t *motor_controller;   // 电机控制器
	LK_motor_callback_t *receive_data;     // 电机测量值
	float pid_fab, pid_ref;		  // 电机PID测量值和设定值

	uint8_t j = 0;

	if (motor_s == NULL)
	{
		j = idx;
	}
	else
	{
		j = 1;
	}
	// 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
	for (size_t i = 0 ; i < j ; ++i)
	{ // 减小访存开销,先保存指针引用
		if (motor_s == NULL)
		{
			motor = lk_motor_instances[i];
		}
		else
		{
			motor = motor_s;
		}
		motor_setting    = &motor->motor_settings;
		motor_controller = &motor->motor_controller;
		receive_data     = &motor->receive_data;
		pid_ref          = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
		// 多环目标值是上环输出为下环输入

		/* ------------------------------digital_pid------------------------------------*/
		if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
		{
			pid_ref *= -1; // 目标值设置反转
		}

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        // dm_diff的值需要自己另外做处理，比如使其指向视觉发送的目标装甲板的偏差值
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && (motor_setting->outer_loop_type == ANGLE_LOOP))
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_angle_feedback_ptr;
            }
            else
            {
                if (motor->motor_feedback == LK_MOTOR_ABSOLUTE)
                {
                    pid_fab = motor->receive_data.RAD_single_round;
                }
                else if (motor->motor_feedback == LK_MOTOR_DIFF)
                {
                    pid_fab = motor->receive_data.lk_diff;
                }
            }
            // 更新pid_ref进入下一个环
            pid_ref = PID_Position(motor_controller->angle_PID,
                                    pid_fab,
                                    pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            {
                pid_ref += *motor_controller->speed_feedforward_ptr;
            }

            if (motor_setting->speed_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_speed_feedback_ptr;
            }
						else
						{	
								if (motor->motor_feedback == LK_MOTOR_ABSOLUTE)
								{
										pid_fab = motor->receive_data.speed_rps;
								}
						}

            // 更新pid_ref进入下一个环
            pid_ref = PID_Increment(motor_controller->speed_PID,
                                    pid_fab,
                                    pid_ref);
        }

        // 计算扭矩环,目前只要启用了扭矩环就计算,不管外层闭环是什么,并且扭矩只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & TORQUE_FEEDFORWARD)
        {
            pid_ref += *motor_controller->torque_feedforward_ptr;
        }
        if (motor_setting->close_loop_type & TORQUE_LOOP)
        {
            pid_ref = PID_Position(motor_controller->torque_PID,
                                    receive_data->current,
                                    pid_ref);
        }

		if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
		{
			pid_ref *= -1; //输出值设置反转
		}

        if(motor -> lk_ctrl_mode == TORQUE_MODE)
        {
            if(motor->motor_state_flag == MOTOR_DISABLE)
            {
                motor->transmit_data.position_des = motor->receive_data.RAD_single_round;
                motor->transmit_data.velocity_des = 0.0f;
								motor->transmit_data.torque_des = 0.0f;
                pid_ref = 0.0f;
            }
            else
            {
                motor->transmit_data.torque_des = pid_ref;
            }
            
            LK_Ctrl(motor);
        }
    }
}