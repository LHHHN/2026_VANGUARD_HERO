#ifndef __POWER_CONTROL_H_
#define __POWER_CONTROL_H_

#include "super_cap.h"

// 3508电机功率模型参数

#define K0 1.5214e-05f // k0  ---  (20/16384)*(0.3)*(187/3591)/9.55    //转矩常数 = 0.3 ； 9.55 = 360/(2*PI)
                       //           9.55 * P = n *T;  T = 0.3 *I;  n = n1*187/3591
#define K1 1.7131e-05f // k1
#define K2 2.592e-07f  // k2

#define constant 2.6259f // 常量  底盘静态功率/电机数量(4个)

void chassis_power_control(void);
void power_control_init();
void power_control_SetPowerState(void);

extern float chassis_max_power;
extern float P_total;
extern float I_test[4];
extern float P_test;

#endif
