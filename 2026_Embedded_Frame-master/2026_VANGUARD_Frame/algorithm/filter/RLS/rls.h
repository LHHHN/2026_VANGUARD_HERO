/**
* @file rls.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __RLS_H__
#define __RLS_H__

#include <stdint.h>
#include <stdbool.h>

#include "user_lib.h"

#define mat             arm_matrix_instance_f32
#define Matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64

typedef struct
{
	uint8_t sizeof_float;

	uint8_t X_Size;
	uint8_t Y_Size;
	uint8_t P_Size;
	float Lamda; //forgetting factor

	struct
	{
		mat X;
		mat XT;
		mat Lamda;
		mat E;
		mat Z;
		mat K;
		mat W;
		mat P;
		mat Y;
		mat U;
		mat K_Numerator;
		mat K_Denominator;
		mat Cache_Matrix[2];
		mat Cache_Vector[2];
		mat Output;
	} Mat;

	arm_status MatStatus;

	struct
	{
		float *X;
		float *XT;
		float *Lamda;
		float *E;
		float *Z;
		float *K;
		float *W;
		float *Y;
		float *U;
		float *P;
		float *K_Numerator;
		float *K_Denominator;
		float *Cache_Matrix[2];
		float *Cache_Vector[2];
		float *Output;
	} Data;
} recursive_least_squares_t;

extern void RLS_Update(recursive_least_squares_t *RLS);

extern void RLS_Init(recursive_least_squares_t *RLS, uint8_t X_Size, uint8_t P_Size, uint8_t Y_Size);

#endif /* __RLS_H__ */
