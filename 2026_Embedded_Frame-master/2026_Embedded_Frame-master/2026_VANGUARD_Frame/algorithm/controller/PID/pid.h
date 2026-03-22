/**
* @file pid.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __PID_H__
#define __PID_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "arm_math.h"

#ifndef pid_abs
#define pid_abs(x) ((x > 0) ? x : -x)
#endif

/* PID结构体 */
typedef struct
{
	//---------------------------------- init config block
	// config parameter
	float kp;
	float ki;
	float kd;

	float output_limit;
	float integral_limit;     // 积分限幅
	float dead_band;

	//-----------------------------------
	// for calculating
	float target;
	float last_target;
	float measure;
	float last_measure;
	float error;
	float last_error;
	float pre_error;

	float abs_error;
	float sum_error;
	float max_limit_error;

	float p_out;
	float i_term;
	float i_out;
	float d_out;

	float output;
	float last_output;
	float last_d_out;
} PID_t;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
	// basic parameter
	float kp;
	float ki;
	float kd;
	float output_limit;   // 输出限幅
	float dead_band; // 死区

	float integral_limit; // 积分限幅
} pid_init_config_t;

typedef struct
{
	uint8_t pid_mode;
	float integral_max_error;				// 积分项误差死区范围上限
	float integral_min_error;				// 积分项误差死区范围下限
	float ki_index;				// 积分缩放系数
	float output_deadband;			// 输出项死区
	float online_k1;				// 强作用比率
	float online_k2;				// 弱作用比率
} PID_professional_t;			// 专家PID规则设置

/**
 * @brief 初始化PID实例
 * @attention 该函数为旧版本，后续修改为PIDRegister函数
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
PID_t *PID_Init(PID_t *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param target     设定值
 * @return float  PID计算输出
 */
float PID_Position(PID_t *pid, float measure, float target);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param target     设定值
 * @return float  PID计算输出
 */
float PID_Increment(PID_t *pid, float measure, float target);

/**
  * @brief          专家PID 改进
  * @param[out]		pid : PID结构数据指针
  * @param[in]		ref : 当前值
  * @param[in]		set : 期望值
  */
void PID_Professional(PID_t *pid, PID_professional_t *pp, float measure, float target);

#endif /* __PID_H__ */
