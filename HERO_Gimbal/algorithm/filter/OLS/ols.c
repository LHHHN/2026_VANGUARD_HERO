/**
******************************************************************************
 * @file    ols.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include <stdlib.h>
#include <string.h>

#include "ols.h"

/**
 * @brief          普通最小二乘法初始化
 * @param[in]      普通最小二乘法结构体
 * @param[in]      样本数
 * @retval         返回空
 */
void OLS_Init(ordinary_least_squares_t *OLS, uint16_t order)
{
	OLS->order = order;
	OLS->count = 0;
	OLS->x     = (float *) user_malloc(sizeof(float) * order);
	OLS->y     = (float *) user_malloc(sizeof(float) * order);
	OLS->k     = 0;
	OLS->b     = 0;
	memset((void *) OLS->x, 0, sizeof(float) * order);
	memset((void *) OLS->y, 0, sizeof(float) * order);
	memset((void *) OLS->t, 0, sizeof(float) * 4);
}

/**
 * @brief          普通最小二乘法拟合
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 */
void OLS_Update(ordinary_least_squares_t *OLS, float deltax, float y)
{
	static float temp = 0;
	temp              = OLS->x[1];
	for (uint16_t i = 0 ; i < OLS->order - 1 ; ++i)
	{
		OLS->x[i] = OLS->x[i + 1] - temp;
		OLS->y[i] = OLS->y[i + 1];
	}
	OLS->x[OLS->order - 1] = OLS->x[OLS->order - 2] + deltax;
	OLS->y[OLS->order - 1] = y;

	if (OLS->count < OLS->order)
	{
		OLS->count++;
	}
	memset((void *) OLS->t, 0, sizeof(float) * 4);
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->t[0] += OLS->x[i] * OLS->x[i];
		OLS->t[1] += OLS->x[i];
		OLS->t[2] += OLS->x[i] * OLS->y[i];
		OLS->t[3] += OLS->y[i];
	}

	OLS->k = (OLS->t[2] * OLS->order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->order - OLS->t[1] * OLS->t[1]);
	OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->order - OLS->t[1] * OLS->t[1]);

	OLS->standard_deviation = 0;
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->standard_deviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
	}
	OLS->standard_deviation /= OLS->order;
}

/**
 * @brief          普通最小二乘法提取信号微分
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回斜率k
 */
float OLS_Derivative(ordinary_least_squares_t *OLS, float deltax, float y)
{
	static float temp = 0;
	temp              = OLS->x[1];
	for (uint16_t i = 0 ; i < OLS->order - 1 ; ++i)
	{
		OLS->x[i] = OLS->x[i + 1] - temp;
		OLS->y[i] = OLS->y[i + 1];
	}
	OLS->x[OLS->order - 1] = OLS->x[OLS->order - 2] + deltax;
	OLS->y[OLS->order - 1] = y;

	if (OLS->count < OLS->order)
	{
		OLS->count++;
	}

	memset((void *) OLS->t, 0, sizeof(float) * 4);
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->t[0] += OLS->x[i] * OLS->x[i];
		OLS->t[1] += OLS->x[i];
		OLS->t[2] += OLS->x[i] * OLS->y[i];
		OLS->t[3] += OLS->y[i];
	}

	OLS->k = (OLS->t[2] * OLS->order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->order - OLS->t[1] * OLS->t[1]);

	OLS->standard_deviation = 0;
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->standard_deviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
	}
	OLS->standard_deviation /= OLS->order;

	return OLS->k;
}

/**
 * @brief          获取普通最小二乘法提取信号微分
 * @param[in]      普通最小二乘法结构体
 * @retval         返回斜率k
 */
float Get_OLS_Derivative(ordinary_least_squares_t *OLS)
{
	return OLS->k;
}

/**
 * @brief          普通最小二乘法平滑信号
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回平滑输出
 */
float OLS_Smooth(ordinary_least_squares_t *OLS, float deltax, float y)
{
	static float temp = 0;
	temp              = OLS->x[1];
	for (uint16_t i = 0 ; i < OLS->order - 1 ; ++i)
	{
		OLS->x[i] = OLS->x[i + 1] - temp;
		OLS->y[i] = OLS->y[i + 1];
	}
	OLS->x[OLS->order - 1] = OLS->x[OLS->order - 2] + deltax;
	OLS->y[OLS->order - 1] = y;

	if (OLS->count < OLS->order)
	{
		OLS->count++;
	}

	memset((void *) OLS->t, 0, sizeof(float) * 4);
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->t[0] += OLS->x[i] * OLS->x[i];
		OLS->t[1] += OLS->x[i];
		OLS->t[2] += OLS->x[i] * OLS->y[i];
		OLS->t[3] += OLS->y[i];
	}

	OLS->k = (OLS->t[2] * OLS->order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->order - OLS->t[1] * OLS->t[1]);
	OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->order - OLS->t[1] * OLS->t[1]);

	OLS->standard_deviation = 0;
	for (uint16_t i = OLS->order - OLS->count ; i < OLS->order ; ++i)
	{
		OLS->standard_deviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
	}
	OLS->standard_deviation /= OLS->order;

	return OLS->k * OLS->x[OLS->order - 1] + OLS->b;
}

/**
 * @brief          获取普通最小二乘法平滑信号
 * @param[in]      普通最小二乘法结构体
 * @retval         返回平滑输出
 */
float Get_OLS_Smooth(ordinary_least_squares_t *OLS)
{
	return OLS->k * OLS->x[OLS->order - 1] + OLS->b;
}