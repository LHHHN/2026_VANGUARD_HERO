/**
******************************************************************************
 * @file    pid.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "pid.h"

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 * @param config PID初始化设置
 * @return PID_t* PID实例指针
 */
PID_t *PID_Init(PID_t *config)
{
	if (config == NULL)
	{
		return NULL;
	}

	PID_t *pid = (PID_t *) malloc(sizeof(PID_t));
	memset(pid, 0, sizeof(PID_t));

	pid->kp             = config->kp;
	pid->ki             = config->ki;
	pid->kd             = config->kd;
	pid->integral_limit = config->integral_limit;
	pid->output_limit   = config->output_limit;
	pid->dead_band      = config->dead_band;

	return pid;
}

static float Value_Limit(float value, float min, float max)
{
	if (value < min)
	{
		return min;
	}
	else if (value > max)
	{
		return max;
	}
	else
	{
		return value;
	}
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Position(PID_t *pid, float measure, float target)
{
	// 保存上次的测量值和误差,计算当前error
	pid->measure = measure;
	pid->target  = target;
	pid->error   = pid->target - pid->measure;

	// 如果在死区外,则计算PID
	if (pid_abs(pid->error) > pid->dead_band)
	{
		// 基本的pid计算,使用位置式
		pid->p_out  = pid->kp * pid->error;
		pid->i_term = pid->ki * pid->error;
		pid->i_out += pid->i_term;
		pid->d_out = pid->kd * (pid->error - pid->last_error);

		pid->i_out = Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit); // 积分限幅

		pid->output = pid->p_out + pid->i_out + pid->d_out; // 计算输出

		pid->output = Value_Limit(pid->output, -pid->output_limit, pid->output_limit); // 输出限幅
	}
	else // 进入死区, 则清空积分和输出
	{
		pid->output = 0;
		pid->i_term = 0;
	}

	// 保存当前数据,用于下次计算
	pid->last_target  = pid->target;
	pid->last_measure = pid->measure;
	pid->last_output  = pid->output;
	pid->last_d_out   = pid->d_out;
	pid->last_error   = pid->error;

	return pid->output;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Increment(PID_t *pid, float measure, float target)
{
	// 保存上次的测量值和误差,计算当前error
	pid->measure = measure;
	pid->target  = target;
	pid->error   = pid->target - pid->measure;

	// 如果在死区外,则计算PID
	if (pid_abs(pid->error) > pid->dead_band)
	{
		// 基本的pid计算,使用增量式
		pid->p_out  = pid->kp * (pid->error - pid->last_error);
		pid->i_term = pid->ki * pid->error;
		pid->i_out  = pid->i_term;
		pid->d_out  = pid->kd * (pid->error - 2.0f * pid->last_error + pid->pre_error);

		pid->i_out = Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit); // 积分限幅

		pid->output += (pid->p_out + pid->i_out + pid->d_out); // 计算输出

		pid->output = Value_Limit(pid->output, -pid->output_limit, pid->output_limit); // 输出限幅
	}
	else // 进入死区, 则清空积分和输出，若使用此来控制速度时请设置死区为0
	{
		pid->output = 0;
		pid->i_term = 0;
	}

	// 保存当前数据,用于下次计算
	pid->last_target  = pid->target;
	pid->last_measure = pid->measure;
	pid->last_output  = pid->output;
	pid->last_d_out   = pid->d_out;
	pid->pre_error    = pid->last_error;
	pid->last_error   = pid->error;

	return pid->output;
}

/**
  * @brief          专家PID(模糊pid簡化版)
  * @param[out]		pid : PID结构数据指针
  * @param[in]		ref : 当前值
  * @param[in]		set : 期望值
  */
void PID_Professional(PID_t *pid, PID_professional_t *pp, float measure, float target)
{
	if (pid == NULL || pp == NULL)
	{
		return;
	}

	// 递推旧值
	pid->pre_error    = pid->last_error;
	pid->last_error   = pid->error;
	pid->abs_error    = (fabsf)(pid->error);
	pid->last_measure = pid->measure;
	pid->last_target  = pid->target;
	pid->last_output  = pid->output;
	pid->last_d_out   = pid->d_out;

	pid->measure = measure;
	// 输出微分先行
	pid->measure = pid->measure * 0.950f + pid->last_measure * (1.000f - 0.950f);
	pid->target  = target;
	pid->error   = pid->target - pid->measure;
	// 偏差微分先行
	pid->error = pid->error * 0.850f + pid->last_error * (1.000f - 0.850f);
	pid->sum_error += pid->error;

	if (pp->pid_mode == 0)
	{
		// 位置式 PID 计算
		// 变速积分（线性积分分离)
		if (pid->abs_error > pp->integral_max_error)
		{
			pp->ki_index = 0.000f;
		}
		else if (pid->abs_error < pp->integral_min_error)
		{
			pp->ki_index = 1.000f;
		}
		else
		{
			pp->ki_index = (pp->integral_max_error - pid->abs_error) / (pp->integral_max_error - pp->integral_min_error);
		}
		pid->p_out = pid->kp * pid->error;
		pid->i_out = pp->ki_index * (pid->ki * pid->sum_error);
		pid->d_out = pid->kd * (pid->error - pid->last_error);
		// 微分项不完全微分
		pid->d_out = pid->d_out * 0.850f + pid->last_d_out * (1.000f - 0.850f);
		Value_Limit(pid->sum_error, -pid->max_limit_error, pid->max_limit_error);
		Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit);
		// 遇限削弱积分
		if (fabsf(pid->last_output) > pp->output_deadband)
		{
			if ((pid->sum_error > 0.0f && pid->error < 0.0f) || (pid->sum_error < 0.0f && pid->error > 0.0f))
			{
				pid->sum_error += pid->error;
			}
		}
		else
		{
			pid->sum_error += pid->error;
		}
		// 位置式 PID 计算
		pid->output = pid->p_out + pid->i_out + pid->d_out;
		// 有效偏差
		// 专家PID规则
		if (pid->abs_error > pp->integral_min_error && pid->abs_error < pp->integral_max_error)
		{
			// 误差不大不小 在正常范围内
			if ((pid->error * (pid->error - pid->last_error) > 0 && pid->last_error * (pid->last_error - pid->pre_error) > 0) || (pid->error - pid->last_error) == 0)
			{
				// 偏差在朝向偏差绝对值增大的方向变化(偏差越来越大), 或者偏差一直为某一固定值
				if (pid->abs_error > (pp->integral_max_error + pp->integral_min_error) / 2.0f)
				{
					// 控制器实施较强的控制作用
					pid->output = pp->online_k1 * pid->output;
				}
				else
				{
					// 但是偏差绝对值本身并不是很大
					pid->output = pid->output + 0.0f;
				}
				//		} else if ((pid->error * (pid->error - pid->last_error) < 0 && (pid->error - pid->last_error) * (pid->last_error - pid->pre_error) > 0) || (pid->error == 0 && pid->last_error == 0)) {
				// 偏差的绝对值向减小的方向变化，或者已经达到平衡状态
				// 此时可以保持控制器输出不变
			}
			else if (pid->error * (pid->error - pid->last_error) < 0 && ((pid->error - pid->last_error) * (pid->last_error - pid->pre_error) < 0))
			{
				// 偏差处于极值极限状态
				if (pid->abs_error > (pp->integral_max_error + pp->integral_min_error) / 2.0f)
				{
					pid->output = pp->online_k1 * pid->output;
				}
				else
				{
					// 但是偏差绝对值本身并不是很大
					pid->output = pid->output + 0;
				}
			}
		}
	}
	else if (pp->pid_mode == 1)
	{
		if (pid->abs_error > pp->integral_max_error)
		{
			pid->output += pp->output_deadband;
		}
		else if ((pid->abs_error < pp->integral_max_error) && (pid->abs_error > pp->integral_min_error))
		{
			// 增量式 PID 计算
			pid->p_out  = pid->kp * (pid->error - pid->last_error);
			pid->i_term = pid->ki * pid->error;
			pid->i_out  = pid->i_term;
			pid->d_out  = pid->kd * (pid->error - 2.0f * pid->last_error + pid->pre_error);
			// 微分项不完全微分
			pid->d_out = pid->d_out * 0.850f + pid->last_d_out * (1.000f - 0.850f);
			Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit);
			//偏差在朝向偏差绝对值增大的方向变化，或者偏差为某一固定值
			if ((pid->error * (pid->error - pid->last_error) < 0) || ((pid->error - pid->last_error) == 0))
			{
				//|e(k)|>Mmid，说明偏差也较大，可考虑由控制器实施较强的控制作用，以达到扭转偏差绝对值向减小的方向变化，并迅速减小偏差的绝对值
				if (pid->abs_error > (pp->integral_max_error + pp->integral_min_error) / 2.0f)
				{
					pid->output += pp->online_k1 * pid->p_out + pid->i_out + pid->d_out;
				}
				//|e(k)|≤Mmid，说明尽管偏差是向绝对值增大的方向变化，但是偏差绝对值本身并不是很大，可以考虑控制器实施一般的控制作用，只需要扭转偏差的变化趋势，使其向偏差绝对值减小的方向变化即可
				else
				{
					pid->output += pid->p_out + pid->i_out + pid->d_out;
				}
			}
			else if (((pid->error * (pid->error - pid->last_error) < 0) && ((pid->error - pid->last_error) * (pid->last_error - pid->pre_error) > 0)) || (pid->error == 0))
			{
				//偏差的绝对值向减小的方向变化，或者已经达到平衡状态
				//此时可以保持控制器输出不变
				pid->output = pid->last_output;
			}
			else if ((pid->error * (pid->error - pid->last_error) < 0) && (pid->last_error * (pid->last_error - pid->pre_error) < 0))
			{
				//偏差处于极值极限状态
				if (pid->abs_error > (pp->integral_max_error + pp->integral_min_error) / 2.0f)
				{
					pid->output += pp->online_k1 * pid->p_out;
				}
				else
				{
					// 但是偏差绝对值本身并不是很大
					pid->output += pp->online_k2 * pid->p_out;
				}
			}
		}
		else if (pid->abs_error < pp->integral_min_error)
		{
			pid->p_out  = pid->kp * (pid->error - pid->last_error);
			pid->i_term = pid->ki * pid->error;
			pid->i_out  = pid->i_term;
			Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit);
			pid->output += pid->p_out + pid->i_out;
		}
	}
	// 输出不完全微分
	pid->output = pid->output * 0.850f + pid->last_output * (1.000f - 0.850f);
	Value_Limit(pid->output, -pid->output_limit, pid->output_limit);
}
