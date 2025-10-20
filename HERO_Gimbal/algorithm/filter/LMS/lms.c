/**
******************************************************************************
 * @file    lms.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "lms.h"

#include "stm32h7xx_hal.h"

/**
 * @brief 初始化LMS滤波器权重（线性递减权重）
 * @param afd 指向滤波器结构体的指针（需确保非NULL）
 * @note 权重分配策略：距离当前时刻越近的样本权重越大
 */
void Nlms_Init(nlms_t *afd, uint8_t flag)
{
	/* 参数有效性检查（硬件开发必备） */
	if (afd == NULL)
	{
		return;  /* 或触发硬件看门狗复位 */
	}

	memset(afd, 0, sizeof(nlms_t));

	/* 初始化计数器 */
	afd->cnt = 0;
    afd->adapt_flag = flag;

	/* 计算线性权重（使用float累加以避免double精度浪费） */
	float w_sum = 0.0f;
	for (int i = 0 ; i < MAX_LMS_FILTER_NUM ; i++)
	{
		const float wi = (float) (i + 1);  /* 显式类型转换 */
		w_sum += (float) wi;                 /* 累加阶段使用float */
		afd->w[i] = wi;                     /* 暂存原始权重 */
	}

	/* 权重归一化（最终结果保持double精度） */
	float w_sum_inv = 1.0f / w_sum;  /* 除法转乘法优化 */
	for (int i = 0 ; i < MAX_LMS_FILTER_NUM ; i++)
	{
		afd->w[i] = (float) w_sum_inv * afd->w[i];  /* 最终权重计算 */
	}

	/* 硬件寄存器保护（防止初始化被中断打断） */
	__disable_irq( );
	memset(afd->x, 0, sizeof(afd->x));  /* 清空输入队列 */
	__enable_irq();
}

/**
 * @brief 更新NLMS滤波器状态（支持变长度滤波）
 * @param afd     指向滤波器结构体的指针
 * @param a       学习率参数（0 < a ≤ 2）
 * @param mu      正则化参数（防止除以零）
 * @param new_x   最新输入样本
 * @param new_d   期望输出（参考值）
 *
 * 算法特性：
 * 1. 动态切换模式：
 *    - 启动阶段（cnt < MAX）：线性加权均值滤波
 *    - 稳态阶段（cnt == MAX）：归一化LMS滤波
 * 2. 边界保护：防止除以零和数值溢出
 */
void Nlms_Filter(nlms_t *afd, float new_x)
{
	/* 参数有效性检查（硬件实时系统建议使用assert） */
	if (afd->a <= 0.0 || afd->a > 2.0)
	{
		afd->a = 1.0;  // 限制学习率范围
	}
	if (afd->mu < 1e-6)
	{
		afd->mu = 1e-6;         // 最小正则化值
	}

	/* 样本队列维护（环形缓冲区优化） */
	if (afd->cnt < MAX_LMS_FILTER_NUM)
	{
		afd->x[afd->cnt++] = (float) new_x;  // 启动阶段填充缓冲区
	}
	else
	{
		/* 使用memmove实现高效数据移位（比逐元素拷贝更快） */
		memmove(&afd->x[0], &afd->x[1], (MAX_LMS_FILTER_NUM - 1) * sizeof(float));
		afd->x[MAX_LMS_FILTER_NUM - 1] = (float) new_x;
	}

	/* 动态滤波处理 */
	if ((afd->cnt == MAX_LMS_FILTER_NUM) && (afd->adapt_flag == 1))
	{
		/* NLMS滤波核心计算 */
		float x_norm = afd->mu;  // 初始化正则化项
		afd->y        = 0.0;

		/* 第一遍循环：计算能量项和预测输出 */
		for (int i = 0 ; i < MAX_LMS_FILTER_NUM ; i++)
		{
			afd->y += afd->w[i] * afd->x[i];       // 计算预测输出 y = w'*x
			x_norm += afd->x[i] * afd->x[i];              // 计算输入能量 x'*x
		}

		/* 误差计算与权重更新 */
		float error     = afd->d - afd->y;
		float step_size = afd->a / fmax(x_norm, afd->mu);  // 防止除以零

		/* 第二遍循环：并行更新权重（考虑定点数优化潜力） */
		for (int i = 0 ; i < MAX_LMS_FILTER_NUM ; i++)
		{
			afd->w[i] += step_size * error * afd->x[i];
		}
	}
	else
	{
		/* 启动阶段线性加权滤波（近期数据权重更高） */
		float weighted_sum = 0.0f;
		float weight_sum   = 0.0f;

		for (int i = 0 ; i < afd->cnt ; i++)
		{
			const float wi = (float) (i + 1);  // 线性递增权重
			weighted_sum += afd->x[i] * wi;
			weight_sum += wi;
		}

		if (weight_sum == 0.0f)
		{
			weight_sum = 1.0f; // 防止除以零
		}

		afd->y = weighted_sum / weight_sum;
	}
}

#define BUFFER_SIZE 20       // 缓冲区大小
#define GAUSSIAN_STD 2.0f    // 高斯分布标准差（控制权重衰减速度）

float Nlms_Filter_Sensor(nlms_t *afd, float new_x)
{
    // 环形缓冲区实现延迟参考信号
	static float buffer[BUFFER_SIZE] = {0};
    static uint8_t head = 0;
    static uint8_t ref_index = 0;
    uint8_t delay     = 9;  // 延迟阶数（根据信号特性调整）

    // 计算参考信号位置
    ref_index = (head + delay) % BUFFER_SIZE;
    afd->d = buffer[ref_index];

    // 更新队列
    buffer[head] = new_x;
    head = (head + 1) % BUFFER_SIZE;

	Nlms_Filter(afd, new_x);

	return afd->y;
}
