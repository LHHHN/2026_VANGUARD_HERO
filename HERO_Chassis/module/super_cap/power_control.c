#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "power_control.h"
#include "omni_mecanum_chassis.h"
#include "referee.h"

float P_cal[4] = {0};   // 电机功率模型计算出的功率
float P_total = 0;      // 测量总功率
float P_total_temp = 0; // 临时总功率

float P_cmd[4] = {0};          // 以最大功率上线分配所得的功率
float chassis_max_power = 100; // 底盘最大功率 , 5~8W 的误差

float I_temp[4] = {0};

float I_test[4];
float P_test;

float speed_rad;
float current;

// 与大P分配有关的参数---------
float error[4] = {0};
float error_sum = 0.0f;
float weight[4] = {0};
float k_coe = 0.0f;
float E_lower = 5.0f;  // 误差下阈值（需要调）
float E_upper = 30.0f; // 误差上阈值（需要调）
//--------------------------

extern super_cap_instance_t *super_cap_instance;
static super_cap_fillmessage_t super_cap_control_msg = {0};

void chassis_power_control()
{

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float delta = 0.0f;
    float E_k = 0.0f;

    // 功率模型
    for (uint8_t i = 0; i < 4; i++)
    {
        if (chassis_m3508[i] == NULL)
        {
            // 记录或跳过
            continue;
        }
        speed_rad = chassis_m3508[i]->receive_data.speed; // motor_speed ---> rad
        current = chassis_m3508[i]->transmit_data.current;

        P_cal[i] = K0 * current * speed_rad +
                   K1 * speed_rad * speed_rad +
                   K2 * current * current +
                   constant; // 静态功耗

        if (P_cal[i] < 0) // 负功率不计入（过渡性）
            continue;

        P_total_temp += P_cal[i]; // 输出总功率
    }

    P_total = P_total_temp; // 用于观察计算出来的底盘总功率
    P_total_temp = 0;       // 防止累加

    if (P_total > chassis_max_power) // 总功率超过
    {
        if (P_total > chassis_max_power)
        {
            // 计算误差
            error_sum = 0.0f;

            for (uint8_t i = 0; i < 4; i++)
            {
                // float target = target_speed[i]; // 底盘电机的目标值
                float real = chassis_m3508[i]->receive_data.speed;

                // error[i] = fabsf(target - real);

                error_sum += error[i];
            }

            // 计算置信度 k_coe
            if (error_sum <= E_lower)
                k_coe = 0.0f;
            else if (error_sum >= E_upper)
                k_coe = 1.0f;
            else
            {
                float x = (error_sum - E_lower) / (E_upper - E_lower);

                if (x < 0.0f)
                    x = 0.0f;
                if (x > 1.0f)
                    x = 1.0f;

                k_coe = x * x * (3.0f - 2.0f * x);
            }

            // 计算权重
            for (uint8_t i = 0; i < 4; i++)
            {
                float w_error = 0.0f;
                float w_power = 0.0f;

                if (error_sum > 1e-6f)
                    w_error = error[i] / error_sum;

                if (P_total > 1e-6f)
                    w_power = P_cal[i] / P_total;

                weight[i] = k_coe * w_error + (1.0f - k_coe) * w_power;
            }

            // 重新分配功率
            for (uint8_t i = 0; i < 4; i++)
            {
                P_cmd[i] = chassis_max_power * weight[i];
            }

            // 根据 P_cmd 反算电流
            for (uint8_t i = 0; i < 4; i++)
            {
                speed_rad = chassis_m3508[i]->receive_data.speed;
                // current = chassis_m3508[i]->target.current;

                float a = K2;
                float b = K0 * speed_rad;
                float c = K1 * speed_rad * speed_rad + constant - P_cmd[i];

                float delta = b * b - 4 * a * c;

                if (delta < 0)
                    continue;

                if (current > 0)
                    I_temp[i] = (-b + sqrtf(delta)) / (2 * a);
                else
                    I_temp[i] = (-b - sqrtf(delta)) / (2 * a);

                // 电流限幅
                if (I_temp[i] > 16000.0f)
                    I_temp[i] = 16000.0f;
                if (I_temp[i] < -16000.0f)
                    I_temp[i] = -16000.0f;

                // chassis_m3508[i]->target.current = I_temp[i];
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////调试使用，后面可以注释这一段
    for (uint8_t i = 0; i < 4; i++)
    {
        speed_rad = chassis_m3508[i]->receive_data.speed; // motor_list[i]->data.speed ---> rpm
        // current = chassis_m3508[i]->target.current;

        P_cal[i] = K0 * current * speed_rad +
                   K1 * speed_rad * speed_rad +
                   K2 * current * current +
                   constant; // 静态功耗

        if (P_cal[i] < 0) // 负功率不计入（过渡性）
            continue;

        P_total_temp += P_cal[i]; // 输出总功率
    }

    P_test = P_total_temp; // 用于观察功率控制后的底盘总功率
    P_total_temp = 0;      // 防止累加
}

void power_control_init()
{
    Super_cap_Init();
    vTaskDelay(1);
    super_cap_control_msg.clearError = 0;
    super_cap_control_msg.enableDCDC = 1;
    super_cap_control_msg.refereeEnergyBuffer = 60;
    super_cap_control_msg.refereePowerLimit = 100;
    super_cap_control_msg.systemRestart = 0;
    vTaskDelay(1);
}

void power_control_SetPowerState(void)
{
    // TODO:完善超电的数据读取之后的处理版本
    // 读取超电的电容组数据以及当前能量进行判断
    // if (super_cap_instance.receive_data.chassisPower == 100)
    // {
    // 毛胚房版本
    // Super_cap_Send(&super_cap_instance, &super_cap_control_msg);
    Super_cap_Send_Auto(1, 0, 0);
    // }
}
