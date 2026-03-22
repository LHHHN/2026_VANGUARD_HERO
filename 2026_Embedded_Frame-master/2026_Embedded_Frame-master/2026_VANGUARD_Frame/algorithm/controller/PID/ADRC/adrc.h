/**
  ******************************************************************************
  * @file    adrc.c
  * @author  张曦梁
  * @version V1.0
  * @date    2021/10/24
  * @brief   ADRC计算函数
  ******************************************************************************
  * @attention
  *	
  *
  ******************************************************************************
  */
	
#include "universal.h"
typedef struct
{
/*******安排过渡过程*******/
float x1;//跟踪微分器状态量
float x2;//跟踪微分期状态量微分项
float r;//时间尺度
float h;//ADRC系统积分时间
u16 N0;//跟踪微分期解决速度超调h0=N*h

float h0;
float fh;//最速微分加速度跟踪量
float fst;//最速微分加速度跟踪量

/*******扩张状态观测器*******/
/******已知输出y和输入u******/
float z1;
float z2;
float z3;//根据控制对象的输入输出，提取的扰动信息
float e;//系统状态误差
float y;//系统输出量
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b;


/**********系统状态误差反馈率*********/
float e0;//状态误差积分项
float e1;//状态偏差
float e2;//状态微分项
float u0;//非线性组合系统输出
float u;//扰动补偿后的输出
float b0;//扰动补偿

/*********第一种组合形式*********/
float beta_0;//线性
float beta_1;//非线性组合参数
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0)
/*********第二种组合形式*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
float alpha2;//0<alpha1<1<alpha2
float zeta;//线性段的区间长度
/*********等三种组合形式*********/
float h1;//u0=-fhan(e1,e2,r,h1)
u16 N1;//跟踪微分期解决速度超调h0=N*h
/*********第四种组合形式*********/
float c;//u0=-fhan(e1,c*e2*e2,r,h1)

}AdrcTypeDef;



void ADRC_Init(AdrcTypeDef *fhan_Input1,AdrcTypeDef *fhan_Input2);
void Fhan_ADRC(AdrcTypeDef *fhan_Input,float expect_ADRC);
void ADRC_Control(AdrcTypeDef *fhan_Input,float feedback_ADRC,float expect_ADRC);

extern AdrcTypeDef Gimbal_adrc[2];

#ifndef __LADRC_FEEDFORWARD_H
#define __LADRC_FEEDFORWARD_H

#include "struct_typedef.h"
#include "ladrc.h"

#ifdef __cplusplus
extern "C"{
#endif
		
#ifdef __cplusplus
typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} differ_type_def;	

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} lpf_type_def;

class	LADRC_FDW_t:public LADRC_t 
{
  private:
	  differ_type_def differ1;
	  differ_type_def differ2;
	  fp32 dif1;
	  fp32 dif2; 
	  fp32 own_w;//前馈带宽
	  fp32 own_gain;//前馈增益
		
	  //输入
	  fp32 own_set_last;
	
		uint32_t DWT_CNT;
    float dt;
	public:
		void Init(fp32 ladrc_fdw_param[5],fp32 max_out);
		void MaxOutInit(fp32 input_max_out);
	  fp32 FDW_Calc(fp32 measure, fp32 set, fp32 gyro);
};
	
extern fp32 LPF(lpf_type_def *lpf ,fp32 time_cons,fp32 input,fp32 w);
#endif

#ifdef __cplusplus
}
#endif

#endif

#ifndef __LADRC_H
#define __LADRC_H

#include "struct_typedef.h"
#include "user_lib.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class LADRC_t
{
	public:
		fp32 time_cons; //时间常数
	
	  fp32 own_wo;//观测器带宽
		fp32 own_b0;//输出增益
		fp32 z1;
		fp32 z2;
	
	  fp32 own_wc;//控制器带宽
	
    fp32 own_max_out;  //最大输出
	  //输入
    fp32 own_set;//设定值
    fp32 own_fdb;//反馈值
		fp32 own_gyro;//角速度
		fp32 own_err;
	
		fp32 u;
		uint8_t init_flag;
		void Init(fp32 wc,fp32 b0,fp32 wo,fp32 max_out);
		void MaxOutInit(fp32 input_max_out);
	  fp32 Calc(fp32 measure, fp32 set, fp32 gyro);
};
	
	
#endif

#ifdef __cplusplus
}
#endif

#endif


