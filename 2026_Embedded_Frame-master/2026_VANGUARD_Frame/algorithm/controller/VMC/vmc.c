#include "vmc.h"

#include "arm_math.h"

void VMC_Init(vmc_leg_t *vmc) // 给杆长赋值
{
	vmc->l5      = 0.11f; // AE长度 //单位为m
	vmc->l1      = 0.15f; // 单位为m
	vmc->l2      = 0.27f; // 单位为m
	vmc->l3      = 0.27f; // 单位为m
	vmc->l4      = 0.15f; // 单位为m
	vmc->wheel_m = 0.65f;
	vmc->theta_target = 0.0f;
}

void VMC_Calc_Base_Data(vmc_leg_t *vmc, INS_behaviour_t *ins, float dt) // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
	if (dt == 0.0f )//|| _isnanf(dt))
	{
		dt = 1.0f;
	}
	else if (dt > 0.005f)
	{
		dt = 0.001f;
	}

	if (vmc->first_init_flag == 0)
	{
		vmc->dd_L0 = 0.0f;
		vmc->dd_theta = 0.0f;
		vmc->dd_z_M = 0.0f;
		vmc->dd_z_wheel = 0.0f;
		vmc->last_L0 = vmc->L0;
		vmc->last_phi0       = vmc->phi0;
		vmc->last_theta = vmc->theta;
		vmc->last_d_phi0 = vmc->d_phi0;
		vmc->last_d_theta = vmc->d_theta;
		vmc->last_alpha = vmc->alpha;
		vmc->last_d_alpha = vmc->d_alpha;
		vmc->first_init_flag = 1;
	}

	vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4);           // D的y坐标
	vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1);           // B的y坐标
	vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4); // D的x坐标
	vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1);           // B的x坐标

	vmc->l_BD = sqrt(
	                 (vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

	vmc->A0   = 2 * vmc->l2 * (vmc->XD - vmc->XB);
	vmc->B0   = 2 * vmc->l2 * (vmc->YD - vmc->YB);
	vmc->C0   = vmc->l2 * vmc->l2 + vmc->l_BD * vmc->l_BD - vmc->l3 * vmc->l3;
	vmc->phi2 = 2 *
	            atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)),
	                   vmc->A0 + vmc->C0);
	vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2),
	                   vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));
	// C点直角坐标
	//	vmc->XC = vmc->l1 * arm_cos_f32( vmc->phi1 ) + vmc->l2 * arm_cos_f32( vmc->phi2 );
	//	vmc->YC = vmc->l1 * arm_sin_f32( vmc->phi1 ) + vmc->l2 * arm_sin_f32( vmc->phi2 );
	vmc->XC = vmc->XB + vmc->l2 * arm_cos_f32(vmc->phi2);
	vmc->YC = vmc->YB + vmc->l2 * arm_sin_f32(vmc->phi2);
	// C点极坐标
	vmc->L0 = (sqrt((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + (vmc->YC * vmc->YC))) * 0.85f +
	          vmc->L0 * 0.15f;

	vmc->phi0 = (atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f))) * 0.8f +
	            vmc->phi0 * 0.2f;//phi0用于计算lqr需要的theta
	vmc->alpha  = (PI / 2.0f - vmc->phi0) * 0.8f + vmc->alpha * 0.2f;
	vmc->theta  = (PI / 2.0f - vmc->pitch - vmc->phi0) * 0.8f + vmc->theta * 0.2f; // 得到状态变量1
	vmc->height = vmc->L0 * arm_cos_f32(vmc->theta) + 0.1f;
	vmc->dd_z_M = ins->MotionAccel_n[2] * 0.8f + vmc->dd_z_M * 0.2f;

	vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt * 0.8f + vmc->last_d_phi0 * 0.2f;
	vmc->d_alpha = (vmc->alpha - vmc->last_alpha) / dt * 0.8f + vmc->last_d_alpha * 0.2f;
	vmc->d_theta = (vmc->theta - vmc->last_theta) / dt * 0.8f + vmc->last_d_theta * 0.2f;
	vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt * 0.8f + vmc->last_d_L0 * 0.2f;
	// vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / (dt * 10) +
	//               (vmc->last_d_phi0 * 0.008f / (dt * 10)); // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
	// vmc->d_alpha = (0.0f - vmc->d_phi0) * 0.8f + vmc->d_alpha * 0.2f;
	// vmc->d_theta = (vmc->theta - vmc->last_theta) / (dt * 10) + (vmc->last_d_theta * 0.008f / (dt * 10));
	// 			// 	(-vmc->d_pitch - vmc->d_phi0) * 0.8f +
	//             //    vmc->last_d_theta * 0.2f;   // 得到状态变量2
	// vmc->d_L0     = (vmc->L0 - vmc->last_L0) / (dt * 10) + (vmc->last_d_L0) * 0.008f / (dt * 10);      // 腿长L0的一阶导数
	vmc->d_height = vmc->d_L0 * arm_cos_f32(vmc->theta) - vmc->L0 * arm_sin_f32(vmc->theta) * vmc->d_theta;

	vmc->last_L0   = vmc->L0;
	vmc->last_phi0 = vmc->phi0;
	vmc->last_theta = vmc->theta;
	vmc->last_alpha = vmc->alpha;

	vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt * 0.8f + vmc->dd_L0 * 0.2f;
	vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt * 0.8f + vmc->dd_theta * 0.2f;
	// vmc->dd_L0    = (vmc->d_L0 - vmc->last_d_L0) / (dt * 10) + (vmc->dd_L0 * 0.008f / (dt * 10)); // 腿长L0的二阶导数
	// vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / (dt * 10) + (vmc->dd_theta * 0.008f / (dt * 10));

	vmc->dd_z_wheel = (vmc->dd_z_M - vmc->dd_L0 * arm_cos_f32(vmc->theta) +
	                   2.0f * vmc->d_L0 * vmc->d_theta * arm_sin_f32(vmc->theta) +
	                   vmc->L0 * vmc->dd_theta *
	                   arm_sin_f32(vmc->theta) +
	                   vmc->L0 * vmc->d_theta * vmc->d_theta * arm_cos_f32(vmc->theta)) * 0.8f +
	                  vmc->dd_z_wheel * 0.2f;

	vmc->last_d_L0    = vmc->d_L0;
	vmc->last_d_phi0  = vmc->d_phi0;
	vmc->last_d_theta = vmc->d_theta;
	vmc->last_d_alpha = vmc->d_alpha;
}

void VMC_Calc_T_Joint(vmc_leg_t *vmc) // 计算期望的关节输出力矩
{
	vmc->j11 = (vmc->l1 * arm_sin_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) /
	           arm_sin_f32(vmc->phi3 - vmc->phi2);
	vmc->j12 = (vmc->l1 * arm_cos_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) /
	           (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));
	vmc->j21 = (vmc->l4 * arm_sin_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) /
	           arm_sin_f32(vmc->phi3 - vmc->phi2);
	vmc->j22 = (vmc->l4 * arm_cos_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) /
	           (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));

	vmc->front_joint_torque  = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp; // F0为五连杆机构末端沿腿的推力
	vmc->back_joint_torque = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp; // Tp为沿中心轴的力矩
}
     

uint8_t touch_threshold = 20;
uint8_t off_threshold = 30;

uint8_t VMC_FN_Ground_Detection(vmc_leg_t *vmc, float dead_line)
{
	if (vmc->F_N < dead_line)
	{ // 离地了
		if (vmc->off_cnt < off_threshold)
		{
			vmc->off_cnt++;
			return 0;
		}
		else
		{
			vmc->touch_cnt = 0;
			vmc->off_cnt   = off_threshold;
			return 1;
		}
	}
	else
	{
		if (vmc->touch_cnt < touch_threshold)
		{
			vmc->touch_cnt++;
			return 1;
		}
		else
		{
			vmc->off_cnt   = 0;
			vmc->touch_cnt = touch_threshold;
			return 0;
		}
	}
}

float LQR_K_Calc(float *coe, float len)
{
	return ((coe[0] * (len * len * len)) + (coe[1] * (len * len)) + (coe[2] * len) + coe[3]);
}

void Leg_Force_Calc(vmc_leg_t *vmc)
{
	float det = 0;

	det = vmc->j11 * vmc->j22 - vmc->j12 * vmc->j21;

	if (det == 0)
	{
		return;
	}

	vmc->inv_j11 = vmc->j22 / det;
	vmc->inv_j12 = -vmc->j12 / det;
	vmc->inv_j21 = -vmc->j21 / det;
	vmc->inv_j22 = vmc->j11 / det;

	vmc->mea_F  = vmc->inv_j11 * vmc->T1 + vmc->inv_j12 * vmc->T2;
	vmc->mea_Tp = vmc->inv_j21 * vmc->T1 + vmc->inv_j22 * vmc->T2;

	// //具有可行性
	// vmc->mea_F = (vmc->T2 * arm_cos_f32(vmc->phi0 - vmc->phi3) / vmc->l4 * arm_sin_f32(vmc->phi3 - vmc->phi4)) - (vmc->T1 * arm_cos_f32(vmc->phi0 - vmc->phi2) / vmc->l1 * arm_sin_f32(vmc->phi1 - vmc->phi2)); 
	// vmc->mea_Tp = vmc->L0 * ((vmc->T1 * arm_sin_f32(vmc->phi0 - vmc->phi2) / vmc->l1 * arm_sin_f32(vmc->phi1 - vmc->phi2)) - (vmc->T2 * arm_sin_f32(vmc->phi0 - vmc->phi3) / vmc->l4 * arm_sin_f32(vmc->phi3 - vmc->phi4)));
 
	vmc->p = vmc->mea_F * arm_cos_f32(vmc->theta)
	         + vmc->mea_Tp * arm_sin_f32(vmc->theta) / vmc->L0;
	vmc->F_N = (vmc->p + vmc->wheel_m * (9.8f + vmc->dd_z_wheel) * 0.6f) * 0.8f + vmc->F_N * 0.2f;
}
