/**
* @file LK_motor.c
 * @author LHHHN (896047872@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "LK_motor.h"
#include "bsp_dwt.h"

static uint8_t idx;
static LK_motor_instance_t *lk_motor_instances[LK_MOTOR_MAX_CNT] = {NULL};

/**
 ************************************************************************
 * @brief:      	float_to_uint: 浮点数转换为无符号整数函数
 * @param[in]:   x_float:	待转换的浮点数
 * @param[in]:   x_min:		范围最小值
 * @param[in]:   x_max:		范围最大值
 * @param[in]:   bits: 		目标无符号整数的位数
 * @retval:     	无符号整数结果
 * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
 ************************************************************************
 **/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span   = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

//设置电机命令
static void LK_Motor_Set_CMD_Mode( LK_motor_instance_t *motor ,LK_CMD_mode_e cmd_mode)
{
    memset(motor->motor_can_instance->tx_buff, 0, 8); // 发送电机指令的时候后面7bytes都是0
    motor->motor_can_instance->tx_buff[0] = (uint8_t) cmd_mode; // 第一位是命令id
    CAN_Transmit(motor->motor_can_instance, 10);
}

//设置控制模式
static void LK_Motor_Set_Control_Mode(LK_motor_instance_t *motor ,LK_control_mode_e control_mode)
{
    motor->control_mode = control_mode; // 第一位是控制模式id
}

//电机启动
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

//电机停止
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

//电机使能
void LK_Motor_Enable(LK_motor_instance_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0 ; i < idx ; ++i)
        {
            LK_Motor_Set_CMD_Mode(lk_motor_instances[i], LK_CMD_ENABLE);
        }
    }
    else
    {
        LK_Motor_Set_CMD_Mode(motor, LK_CMD_ENABLE);
    }
}

//电机失能
void LK_Motor_Disable(LK_motor_instance_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0 ; i < idx ; ++i)
        {
            LK_Motor_Set_CMD_Mode(lk_motor_instances[i], LK_CMD_DISABLE);
        }
    }
    else
    {
        LK_Motor_Set_CMD_Mode(motor, LK_CMD_DISABLE);
    }
}

//清除错误码
void LK_Motor_Clear_Error(LK_motor_instance_t *motor)
{
    LK_Motor_Set_CMD_Mode(motor, LK_CMD_CLEAR_ERROR);
}

//设置目标值
void LK_Motor_SetTar(LK_motor_instance_t *motor, float val)
{
    motor->motor_controller.pid_ref = val;
}

//电机转矩闭环模式控制
//控制值iq为int16_t类型，数值范围-2048~ 2048
//对应MF电机实际转矩电流范围-16.5A~16.5A
//命令字节 0xA1
//DATA[0] = 0xA1
//DATA[4] = *(uint8_t *)(&iq)
//DATA[5] = *((uint8_t *)(&iq)+1) 
//其余都为0
void LK_IQ_Control(LK_motor_instance_t *motor , float iq)
{
    if(motor->control_mode == LK_CONTROL_IQ)
    {
        int16_t iq_data = (int16_t)(iq / 16.5f * 2048.0f);
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_IQ; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&iq_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&iq_data)+1);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机速度闭环模式控制
//控制值speed为int32_t类型，对应实际转速为0.01dps/LSB
//控制值iq为int16_t类型，数值范围-2048~ 2048
//对应MF电机实际转矩电流范围-16.5A~16.5A
//命令字节 0xA2
//DATA[0] = 0xA2
//DATA[2] = *(uint8_t *)(&iq)
//DATA[3] = *((uint8_t *)(&iq)+1) 
//DATA[4] = *(uint8_t *)(&speed)
//DATA[5] = *((uint8_t *)(&speed)+1)
//DATA[6] = *((uint8_t *)(&speed)+2)
//DATA[7] = *((uint8_t *)(&speed)+3)
//其余都为0
void LK_Speed_IQ_Control(LK_motor_instance_t *motor , float speed, float iq)
{
    if(motor->control_mode == LK_CONTROL_SPEED_IQ)
    {
        int16_t iq_data = (int16_t)(iq / 16.5f * 2048.0f);
        int32_t speed_data = (int32_t)(speed * 100.0f); // 转速单位转换为0.01dps
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_SPEED_IQ; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[2] = *(uint8_t *)(&iq_data);
        motor->motor_can_instance->tx_buff[3] = *((uint8_t *)(&iq_data)+1);
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&speed_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&speed_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&speed_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&speed_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机多圈位置闭环模式控制
//控制值angle为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°
//命令字节 0xA3
//DATA[0] = 0xA3
//DATA[4] = *(uint8_t *)(&angle)
//DATA[5] = *((uint8_t *)(&angle)+1)
//DATA[6] = *((uint8_t *)(&angle)+2)
//DATA[7] = *((uint8_t *)(&angle)+3)
//其余都为0
void LK_Multi_Angle_Control(LK_motor_instance_t *motor , float angle)
{
    if(motor->control_mode == LK_CONTROL_MULTI_ANGLE)
    {
        int32_t angle_data = (int32_t)(angle * 100.0f); // 位置单位转换为0.01degree
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_MULTI_ANGLE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angle_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angle_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angle_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angle_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机多圈位置速度闭环模式控制
//控制值angle为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°
//电机转动方向由目标位置和当前位置的差值决定
//控制值maxSpeed限制了电机转动的最大速度，为uint16_t类型，对应实际转速为1dps/LSB
//命令字节 0xA4
//DATA[0] = 0xA4
//DATA[2] = *(uint8_t *)(&maxSpeed)
//DATA[3] = *((uint8_t *)(&maxSpeed)+1)
//DATA[4] = *(uint8_t *)(&angle)
//DATA[5] = *((uint8_t *)(&angle)+1)
//DATA[6] = *((uint8_t *)(&angle)+2)
//DATA[7] = *((uint8_t *)(&angle)+3)
//其余都为0
void LK_Multi_SpeedAngle_Control(LK_motor_instance_t *motor , float angle, float maxSpeed)
{
    if(motor->control_mode == LK_CONTROL_MULTI_SPEEDANGLE)
    {
        int32_t angle_data = (int32_t)(angle * 100.0f); // 位置单位转换为0.01degree
        uint16_t maxSpeed_data = (uint16_t)(maxSpeed); // 转速单位转换为1dps
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_MULTI_SPEEDANGLE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[2] = *(uint8_t *)(&maxSpeed_data);
        motor->motor_can_instance->tx_buff[3] = *((uint8_t *)(&maxSpeed_data)+1);
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angle_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angle_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angle_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angle_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机单圈位置闭环模式控制
//控制值spinDirection 设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针
//控制值angle为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°
//命令字节 0xA5
//DATA[0] = 0xA5
//DATA[1] = (uint8_t)spinDirection
//DATA[4] = *(uint8_t *)(&angle)
//DATA[5] = *((uint8_t *)(&angle)+1)
//DATA[6] = *((uint8_t *)(&angle)+2)
//DATA[7] = *((uint8_t *)(&angle)+3)
//其余都为0
void LK_Single_Angle_Control(LK_motor_instance_t *motor , uint8_t spinDirection, float angle)
{
    if(motor->control_mode == LK_CONTROL_SINGLE_ANGLE)
    {
        int32_t angle_data = (int32_t)(angle * 100.0f); // 位置单位转换为0.01degree
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_SINGLE_ANGLE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[1] = spinDirection;
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angle_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angle_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angle_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angle_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机单圈位置速度闭环模式控制
//控制值spinDirection 设置电机转动的方向，为uint8_t类型，0x00代表顺时针，0x01代表逆时针
//控制值angle为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°
//电机转动方向由目标位置和当前位置的差值决定
//控制值maxSpeed限制了电机转动的最大速度，为uint16_t类型，对应实际转速为1dps/LSB
//命令字节 0xA6
//DATA[0] = 0xA6
//DATA[1] = (uint8_t)spinDirection
//DATA[2] = *(uint8_t *)(&maxSpeed)
//DATA[3] = *((uint8_t *)(&maxSpeed)+1)
//DATA[4] = *(uint8_t *)(&angle)
//DATA[5] = *((uint8_t *)(&angle)+1)
//DATA[6] = *((uint8_t *)(&angle)+2)
//DATA[7] = *((uint8_t *)(&angle)+3)
//其余都为0
void LK_Single_SpeedAngle_Control(LK_motor_instance_t *motor , uint8_t spinDirection, float angle, float maxSpeed)
{
    if(motor->control_mode == LK_CONTROL_SINGLE_SPEEDANGLE)
    {
        int32_t angle_data = (int32_t)(angle * 100.0f); // 位置单位转换为0.01degree
        uint16_t maxSpeed_data = (uint16_t)(maxSpeed); // 转速单位转换为1dps
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_SINGLE_SPEEDANGLE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[1] = spinDirection;
        motor->motor_can_instance->tx_buff[2] = *(uint8_t *)(&maxSpeed_data);
        motor->motor_can_instance->tx_buff[3] = *((uint8_t *)(&maxSpeed_data)+1);
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angle_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angle_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angle_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angle_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机增量位置闭环
//控制值angleIncrement 为 int32_t 类型，对应实际位置为0.01degree/LSB，即36000 代表360°
//转动方向由该参数的符号决定
//命令字节 0xA7
//DATA[0] = 0xA7
//DATA[4] = *(uint8_t *)(&angleIncrement)
//DATA[5] = *((uint8_t *)(&angleIncrement)+1) 
//DATA[6] = *((uint8_t *)(&angleIncrement)+2)
//DATA[7] = *((uint8_t *)(&angleIncrement)+3)   
//其余都为0
void LK_Increment_Angle_Control(LK_motor_instance_t *motor , float angleIncrement)
{
    if(motor->control_mode == LK_CONTROL_INCREMENT_ANGlE)
    {
        int32_t angleIncrement_data = (int32_t)(angleIncrement * 100.0f); // 位置单位转换为0.01degree
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_INCREMENT_ANGlE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angleIncrement_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angleIncrement_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angleIncrement_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angleIncrement_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机增量速度位置速度闭环
//控制值angleIncrement 为 int32_t 类型，对应实际位置为0.01degree/LSB，即36000 代表360°
//转动方向由该参数的符号决定
//控制值maxSpeed限制了电机转动的最大速度，为uint32_t类型，对应实际转速为1dps/LSB
//命令字节 0xA8
//DATA[0] = 0xA8
//DATA[2] = *(uint8_t *)(&maxSpeed)
//DATA[3] = *((uint8_t *)(&maxSpeed)+1)
//DATA[4] = *(uint8_t *)(&angleIncrement)
//DATA[5] = *((uint8_t *)(&angleIncrement)+1)
//DATA[6] = *((uint8_t *)(&angleIncrement)+2)
//DATA[7] = *((uint8_t *)(&angleIncrement)+3)
//其余都为0
void LK_Increment_SpeedAngle_Control(LK_motor_instance_t *motor , float angleIncrement, float maxSpeed)
{
    if(motor->control_mode == LK_CONTROL_INCREMENT_SPEEDANGLE)
    {
        int32_t angleIncrement_data = (int32_t)(angleIncrement * 100.0f); // 位置单位转换为0.01degree
        uint32_t maxSpeed_data = (uint32_t)(maxSpeed); // 转速单位转换为1dps
        memset(motor->motor_can_instance->tx_buff, 0, 8); 
        motor->motor_can_instance->tx_buff[0] = (uint8_t) LK_CONTROL_INCREMENT_SPEEDANGLE; // 第一位是控制模式id
        motor->motor_can_instance->tx_buff[2] = *(uint8_t *)(&maxSpeed_data);
        motor->motor_can_instance->tx_buff[3] = *((uint8_t *)(&maxSpeed_data)+1);
        motor->motor_can_instance->tx_buff[4] = *(uint8_t *)(&angleIncrement_data);
        motor->motor_can_instance->tx_buff[5] = *((uint8_t *)(&angleIncrement_data)+1);
        motor->motor_can_instance->tx_buff[6] = *((uint8_t *)(&angleIncrement_data)+2);
        motor->motor_can_instance->tx_buff[7] = *((uint8_t *)(&angleIncrement_data)+3);
        CAN_Transmit(motor->motor_can_instance, 1);
    }
}

//电机数据解析
static void LK_Motor_Decode(CAN_instance_t *motor_can)
{
    LK_motor_instance_t *motor = (LK_motor_instance_t *)motor_can->id;  
    LK_motor_callback_t *receive_data = &(motor->receive_data) ; // 将can实例中保存的id转换成电机实例的指针
    uint8_t *rxdata = motor_can->rx_buff;
    
    Supervisor_Reload(motor->supervisor); // 喂狗
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    if(motor->error_code & LK_MOTOR_LOSS)
    {
        motor->error_code &= ~LK_MOTOR_LOSS; // 清除掉掉线错误码
    }

    receive_data->last_ecd = receive_data->ecd;
    receive_data->ecd = (uint16_t)(rxdata[7] << 8 | rxdata[6]); // 电机回传的编码器值
    receive_data->angle_single_round = ECD_TO_ANGLE_COEF_LK * receive_data->ecd; // 单圈位置，单位degree

    receive_data->speed_rads = (1 - SPEED_SMOOTH_COEF) * receive_data->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rxdata[5] << 8 | rxdata[4]));// 电机回传的速度值，单位rads

    receive_data->real_current = (1 - CURRENT_SMOOTH_COEF) * receive_data->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxdata[3] << 8 | rxdata[2])); // 电机回传的电流值，单位A

    receive_data->temperature = rxdata[1];

}.