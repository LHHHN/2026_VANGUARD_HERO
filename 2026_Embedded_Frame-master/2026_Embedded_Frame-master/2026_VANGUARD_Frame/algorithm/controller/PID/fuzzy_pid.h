#ifndef FUZZY_PID_H
#define FUZZY_PID_H
#include <stdint.h>

typedef enum
{
    NB = -3, // Negative Big
    NM = -2, // Negative Medium
    NS = -1, // Negative Small
    ZO = 0,  // Zero
    PS = 1,  // Positive Small
    PM = 2,  // Positive Medium
    PL = 3   // Positive Large
} fuzzy_set_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float e_min, e_max;
    float ec_min, ec_max;
    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;

    float e_normalized;  // 归一化后的误差t
    float ec_normalized; // 归一化后的误差变化率

    int8_t rule_kp[7][7];
    int8_t rule_ki[7][7];
    int8_t rule_kd[7][7];

    float error_history[3];

} fuzzy_pid_t;

void Fuzzy_PID_Init(fuzzy_pid_t *fpid);
void Fuzzy_PID_Set_Parameters(fuzzy_pid_t *fpid, float Kp, float Ki, float Kd,
                            float e_min, float e_max, float ec_min, float ec_max,
                            float kp_min, float kp_max, float ki_min, float ki_max,
                            float kd_min, float kd_max);
float Fuzzy_PID_Compute(fuzzy_pid_t *fpid, float setpoint, float actual);

#endif
