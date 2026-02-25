/**
* @file lms.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __LMS_H__
#define __LMS_H__

#include <stdint.h>
#include <stdbool.h>

//归一化最小均方（NLMS）算法通常需要参考信号进行自适应滤波

#ifndef MAX_LMS_FILTER_NUM
#define MAX_LMS_FILTER_NUM 50  /* 最大滤波器阶数 */
#endif

typedef struct
{
	uint8_t cnt;                 /* 样本计数器 */
	float y;                   /* 滤波输出值 */
	float x[MAX_LMS_FILTER_NUM];  /* 输入数据队列 */
	float w[MAX_LMS_FILTER_NUM];  /* 权重系数数组 */

	float a;									/* 学习率参数 */
	float mu;									/* 正则化参数 */

	float d;								/* 虚拟滞后参考 */

	uint8_t num;

  uint8_t adapt_flag;
} nlms_t;

void Nlms_Init(nlms_t *afd, uint8_t num , uint8_t flag);

void Nlms_Filter(nlms_t *afd, float new_x);

float Nlms_Filter_Sensor(nlms_t *afd, float new_x);

#endif /* __LMS_H__ */
