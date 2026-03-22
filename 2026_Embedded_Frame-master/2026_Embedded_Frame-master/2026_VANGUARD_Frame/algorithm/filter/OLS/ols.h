/**
* @file ols.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __OLS_H__
#define __OLS_H__

#include <stdint.h>
#include <stdbool.h>

#include "user_lib.h"

typedef struct
{
	uint16_t order;
	uint32_t count;

	float *x;
	float *y;

	float k;
	float b;

	float standard_deviation;

	float t[4];
} __attribute__((__packed__)) ordinary_least_squares_t;

void OLS_Init(ordinary_least_squares_t *OLS, uint16_t order);

void OLS_Update(ordinary_least_squares_t *OLS, float deltax, float y);

float OLS_Derivative(ordinary_least_squares_t *OLS, float deltax, float y);

float OLS_Smooth(ordinary_least_squares_t *OLS, float deltax, float y);

float Get_OLS_Derivative(ordinary_least_squares_t *OLS);

float Get_OLS_Smooth(ordinary_least_squares_t *OLS);

#endif /* __OLS_H__ */