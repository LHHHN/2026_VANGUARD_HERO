/**
******************************************************************************
 * @file    chassis.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "chassis.h"
#include "remote_control.h"
#include "remote_vt03.h"

#include "rs485.h"

// 底板轮毂电机初始化参数
chassis_cmd_t chassis_cmd;

void Chassis_Init(void)
{
    memset(&chassis_cmd, 0, sizeof(chassis_cmd_t));
}

extern RC_ctrl_t *rc_data;
extern VT03_ctrl_t *vt03_data;

void Chassis_Set_Mode(void)
{
   static uint8_t last_key_cnt[16] = {0};
   static uint8_t key_mode_last = 0;

#define KEY_CLICK(k) (vt03_data->key_count[KEY_PRESS][(k)] != last_key_cnt[(k)])
#define KEY_ACK(k) (last_key_cnt[(k)] = vt03_data->key_count[KEY_PRESS][(k)])            
       if (rc_data->online == 0 && vt03_data->online == 0)
       {
           chassis_cmd.mode = CHASSIS_DISABLE;
       }
       else
       {    
           chassis_cmd.key_state.key_EN_state = 0;
           // 遥控器控制
           if (rc_data->rc.switch_left == 1)
           {
               switch (rc_data->rc.switch_right)
               {
               case 1:
                   /* code */
                   chassis_cmd.mode = CHASSIS_DISABLE;
                   break;
               case 3:
                   /* code */
                   chassis_cmd.mode = CHASSIS_STOP;
                   break;
               case 2:
                   /* code */
                   chassis_cmd.mode = CHASSIS_STOP;
                   break;
               default:
                   chassis_cmd.mode = CHASSIS_DISABLE;
                   break;
               }
           }
           else if (rc_data->rc.switch_left == 3)
           {
               switch (rc_data->rc.switch_right)
               {
               case 1:
                   /* code */
                   chassis_cmd.mode = CHASSIS_FOLLOW;
                   break;
               case 3:
                   /* code */
                   chassis_cmd.mode = CHASSIS_STOP;
                   break;
               case 2:
                   /* code */
                   chassis_cmd.mode = CHASSIS_STOP;
                   break;
               default:
                   chassis_cmd.mode = CHASSIS_DISABLE;
                   break;
               }
           }
           else if (rc_data->rc.switch_left == 2)
           {
               switch (rc_data->rc.switch_right)
               {
               case 1:
                   /* code */
                   chassis_cmd.mode = CHASSIS_UPSTEP;
                   break;
               case 3:
                   /* code */
                   chassis_cmd.mode = CHASSIS_SPIN;
                   break;
               case 2:
                   /* code */
                   chassis_cmd.key_state.key_EN_state = 1;
                   break;
               default:
                   chassis_cmd.mode = CHASSIS_DISABLE;
                   break;
               }
           }
           else
           {
               chassis_cmd.mode = CHASSIS_DISABLE;
           }
       }

   if (chassis_cmd.key_state.key_EN_state == 1 && key_mode_last == 0)
   {
       for (uint8_t i = 0; i < 16; i++)
       {
           last_key_cnt[i] = vt03_data->key_count[KEY_PRESS][i];
       }
   }
   key_mode_last = chassis_cmd.key_state.key_EN_state;

   if (chassis_cmd.key_state.key_EN_state == 1)
   {
       if (KEY_CLICK(Key_B))
       {
           if (chassis_cmd.mode == CHASSIS_DISABLE)
           {
               chassis_cmd.mode = CHASSIS_FOLLOW;
               chassis_cmd.key_state.chassis_EN_state = 1;
           }
           else
           {
               chassis_cmd.mode = CHASSIS_DISABLE;
               chassis_cmd.key_state.chassis_EN_state = 0;
           }
           KEY_ACK(Key_B);
       }

       if (chassis_cmd.key_state.chassis_EN_state == 1)
       {
           /* 蹬腿键鼠 */
           if (vt03_data->key->q == 1)
           {
               chassis_cmd.mode = CHASSIS_UPSTEP;
           }
           else if (vt03_data->key->q == 0 && chassis_cmd.mode == CHASSIS_UPSTEP)
           {
               chassis_cmd.mode = CHASSIS_FOLLOW;
           }

           if (KEY_CLICK(Key_E))
           {
               if (chassis_cmd.mode == CHASSIS_FOLLOW)
               {
                   chassis_cmd.mode = CHASSIS_SPIN;
               }
               else if (chassis_cmd.mode == CHASSIS_SPIN)
               {
                   chassis_cmd.mode = CHASSIS_FOLLOW;
               }
           }
           KEY_ACK(Key_E);
       }      
   }
#undef KEY_CLICK
#undef KEY_ACK      
}

void Chassis_Observer(void)
{
   ;
}

// 更新目标值
void Chassis_Reference(void)
{
   static float chassis_yaw_target = 0.0f;

   if (chassis_cmd.mode == CHASSIS_SPIN)
   {
       chassis_cmd.target_wz = SPIN_SET;
       if (chassis_cmd.key_state.key_EN_state == 0)
       {
           chassis_cmd.target_vx = (float)rc_data->rc.rocker_l1 * REMOTE_X_SEN;
           chassis_cmd.target_vy = (float)rc_data->rc.rocker_l_ * REMOTE_Y_SEN;
       }
       else if (chassis_cmd.key_state.key_EN_state == 1)
       {
           chassis_cmd.target_vx = (float)(vt03_data->key->w - vt03_data->key->s) * KEY_X_SEN;
           chassis_cmd.target_vy = (float)(vt03_data->key->a - vt03_data->key->d) * KEY_Y_SEN;
           if (vt03_data->key->shift == 1)
           {
               chassis_cmd.target_wz = SPIN_SET_PRO;
           }
       }
   }
   else if (chassis_cmd.mode == CHASSIS_FOLLOW)
   {
       if (chassis_cmd.key_state.key_EN_state == 0)
       {
           chassis_cmd.target_vx = (float)rc_data->rc.rocker_l1 * REMOTE_X_SEN;
           chassis_cmd.target_vy = (float)rc_data->rc.rocker_l_ * REMOTE_Y_SEN;
       }
       else if (chassis_cmd.key_state.key_EN_state == 1)
       {
           // if ((vt03_data->key->shift == 1) && (super_cap_instance->receive_data.capEnergyJ >= 100))
           if (vt03_data->key->shift == 1)
           {
               chassis_cmd.target_vx = (float)(vt03_data->key->w - vt03_data->key->s) * KEY_X_SEN_PRO;
               chassis_cmd.target_vy = (float)(vt03_data->key->a - vt03_data->key->d) * KEY_Y_SEN_PRO;
           }
           else
           {
               chassis_cmd.target_vx = (float)(vt03_data->key->w - vt03_data->key->s) * KEY_X_SEN;
               chassis_cmd.target_vy = (float)(vt03_data->key->a - vt03_data->key->d) * KEY_Y_SEN;
           }
       }
       chassis_cmd.target_wz = 0.0f;
   }
   else if (chassis_cmd.mode == CHASSIS_UPSTEP)
   {
       if (chassis_cmd.key_state.key_EN_state == 0)
       {
           chassis_cmd.target_vx = (float)rc_data->rc.rocker_l1 * REMOTE_X_SEN;
           chassis_cmd.target_vy = (float)rc_data->rc.rocker_l_ * REMOTE_Y_SEN;
           chassis_cmd.target_wz = rc_data->rc.rocker_r_ * REMOTE_OMEGA_Z_SEN;
       }
       else if (chassis_cmd.key_state.key_EN_state == 1)
       {
           chassis_cmd.target_vx = (float)(vt03_data->key->w - vt03_data->key->s) * KEY_X_SEN;
           chassis_cmd.target_vy = (float)(vt03_data->key->a - vt03_data->key->d) * KEY_Y_SEN;
           chassis_cmd.target_wz = rc_data->mouse.x * KEY_OMEGA_Z_SEN;
       }
       if (rc_data->rc.rocker_r1 >= 0 && chassis_cmd.leg_state == LEG_NORMAL)
       {
           chassis_cmd.target_leg_angle = rc_data->rc.rocker_r1 * 0.0013f; // 660换算成弧度0-0.88rad
       }
   }
   else if (chassis_cmd.mode == CHASSIS_STOP)
   {
       chassis_cmd.target_wz = 0.0f;
       chassis_cmd.target_vx = 0.0f;
       chassis_cmd.target_vy = 0.0f;
       chassis_cmd.target_leg_angle = 0.0f;
   }
}

void Chassis_Console(void)
{
   ;
}

void Chassis_Send_Cmd(void)
{
    ;
}
