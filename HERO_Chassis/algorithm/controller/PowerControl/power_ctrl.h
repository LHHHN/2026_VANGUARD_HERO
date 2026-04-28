#ifndef POWER_CTRL_H
#define POWER_CTRL_H

// 3508电机功率模型参数

// #define K0 1.5214e-05f  // k0  ---  (20/16384)*(0.3)*(187/3591)/9.55    //转矩常数 = 0.3 ； 9.55 = 360/(2*PI)    
//           9.55 * P = n *T;  T = 0.3 *I;  n = n1*187/3591
// #define K1 1.7131e-05f  // k1
// #define K2 2.592e-07f  // k2

// #define constant 2.6259f  // 常量  底盘静态功率/电机数量(4个)

#define K0 1.996889994e-06f  // k0  ---  (20/16384)*(0.3)*(187/3591)/9.55    //转矩常数 = 0.3 ； 9.55 = 360/(2*PI)    
                             //           9.55 * P = n *T;  T = 0.3 *I;  n = n1*187/3591
//#define K1 1.23e-07f  // k1
//#define K2 1.453e-07f  // k2

#define K1 2.0326e-07f  // k1
#define K2 1.453e-07f  // k2

#define constant 0.75f  // 常量  底盘静态功率/电机数量(4个)

void chassis_power_control(void);
void Power_Control();

extern float chassis_max_power;
extern float P_total;
extern float I_test[4];
extern float P_test;

typedef enum
{
	SUPERCAP_CONTROL = 0,
	CODE_CONTROL = 1,
} power_control_mode_e;//功率控制模式

#endif
