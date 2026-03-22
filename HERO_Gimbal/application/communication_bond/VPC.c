#include "VPC.h"
#include "Serial.h"
#include "INS.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"
#include "usbd_cdc_if.h"

#include "shoot.h"

uint8_t frame_buf[1024];

extern shoot_cmd_t shoot_cmd ;

void VPC_Init(void)
{
  // NV_Send_Packet_Init(&nv_aim_packet_to_nuc);
  VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
  VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);
  // Send_Packet_Init(&aim_packet_to_nuc);
}

/* 更新发送给上位机的数据包 */
void VPC_UpdatePackets(void)
{
  /*导航传输数据区*/
//   nv_aim_packet_to_nuc.header = 0x5A; // 帧头赋值
//   // nv_aim_packet_to_nuc.imu_roll = INS.Roll;
//   nv_aim_packet_to_nuc.imu_pitch = INS.Pitch;
//   nv_aim_packet_to_nuc.imu_yaw = INS.Yaw;

// //   nv_aim_packet_to_nuc.joint_pitch = gimbal_motor_pitch->measure.rad; //lhn:；联调时增加
//   nv_aim_packet_to_nuc.joint_yaw = INS.Yaw;

//   nv_aim_packet_to_nuc.timestamp = 0; // 时间戳
//   nv_aim_packet_to_nuc.robot_hp = 0;  // 血量
//   nv_aim_packet_to_nuc.game_time = 0; // 比赛时间

  /*新视觉传输数据区*/ //(同济大学版本)

  vs_aim_packet_to_nuc.head[0] = 'S';
  vs_aim_packet_to_nuc.head[1] = 'P';
  vs_aim_packet_to_nuc.mode = 1;
  if(shoot_cmd.fire_launched == 1)
  {
    vs_aim_packet_to_nuc.shoot_sign = 1; // 0-不射击 1-射击
  }
  else
  {
    vs_aim_packet_to_nuc.shoot_sign = 0; // 0-不射击 1-射击
  }
  vs_aim_packet_to_nuc.q[0] = INS.q[0];
  vs_aim_packet_to_nuc.q[1] = INS.q[1];
  vs_aim_packet_to_nuc.q[2] = INS.q[2];
  vs_aim_packet_to_nuc.q[3] = INS.q[3];
  vs_aim_packet_to_nuc.yaw = INS.Yaw;
  vs_aim_packet_to_nuc.yaw_vel = 0;   //未定
  vs_aim_packet_to_nuc.pitch = INS.Pitch;
  vs_aim_packet_to_nuc.pitch_vel = 0; //lhn:；联调时增加
  vs_aim_packet_to_nuc.bullet_speed = 15.9; //未定
  vs_aim_packet_to_nuc.bullet_count = 0; //未定

  // vs_aim_packet_to_nuc.mode = 0;
  // vs_aim_packet_to_nuc.q[0] = 0;
  // vs_aim_packet_to_nuc.q[1] = 0;
  // vs_aim_packet_to_nuc.q[2] = 0;
  // vs_aim_packet_to_nuc.q[3] = 0;
  // vs_aim_packet_to_nuc.yaw = 0;
  // vs_aim_packet_to_nuc.yaw_vel = 0;   //未定
  // vs_aim_packet_to_nuc.pitch = 0 ;
  // vs_aim_packet_to_nuc.pitch_vel = 0; //lhn:；联调时增加
  // vs_aim_packet_to_nuc.bullet_speed = 0; //未定
  // vs_aim_packet_to_nuc.bullet_count = 0; //未定

  /*深圳大学版本*/
  // vs_aim_packet_to_nuc.output_data.config = 1.0f;
  // vs_aim_packet_to_nuc.output_data.target_pose[0] = 0.0f;
  // vs_aim_packet_to_nuc.output_data.target_pose[1] = 0.0f;
  // vs_aim_packet_to_nuc.output_data.target_pose[2] = 0.0f;
  // vs_aim_packet_to_nuc.output_data.curr_yaw = INS.Yaw;
  // vs_aim_packet_to_nuc.output_data.curr_pitch = INS.Pitch;
  // vs_aim_packet_to_nuc.output_data.enemy_color = 0; // 0-蓝 1-红
  // vs_aim_packet_to_nuc.output_data.shoot_config = 0x80;

  /*旧视觉传输数据区*/
  //    aim_packet_to_nuc.sof = 0xA6;
  //   aim_packet_to_nuc.detect_color = 1;
  //   aim_packet_to_nuc.task_mode = 1; // 0-auto 1-aim 2-buff
  //   aim_packet_to_nuc.reset_tracker = 1;
  //   aim_packet_to_nuc.is_play = 1;
  //   aim_packet_to_nuc.change_target = 1;
  //   aim_packet_to_nuc.reserved = 1;
  //   aim_packet_to_nuc.roll = INS.Roll;
  //   aim_packet_to_nuc.pitch = INS.Pitch;
  //   aim_packet_to_nuc.yaw = INS.Yaw;
  //   aim_packet_to_nuc.aim_x = 1.23;
  //   aim_packet_to_nuc.aim_y = 4.56;
  //   aim_packet_to_nuc.aim_z = 7.89;
  //   aim_packet_to_nuc.game_time = 0.0f;
  //   aim_packet_to_nuc.timestamp = 0.0f;
}

/*根据帧头选择对应的数据处理*/
void Choose_VPC_Type(void)
{
  uint16_t frame_len = cdc_rx_len; 

  /* 拷贝完整帧 */
  memcpy(frame_buf, cdc_rx_cache, frame_len);

  /*根据帧头来判断接收到的是哪个数据包*/
  /*导航部分*/
  if (frame_buf[0] == 0xA5)
  {
    //memcpy(nv_buf_receive_from_nuc, frame_buf, sizeof(nv_receive_packet_t));
    NV_UnPack_Data_ROS2(frame_buf, &nv_aim_packet_from_nuc, sizeof(nv_receive_packet_t));
  }
  /*视觉部分*/
  else if (frame_buf[0] == 'S' && frame_buf[1] == 'P')
  {
    //memcpy(vs_buf_receive_from_nuc, frame_buf, sizeof(vs_receive_packet_t));
    VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));
  }

  // /* 移除已处理数据 */
  // memset(cdc_rx_cache, 0, sizeof(cdc_rx_cache));
  // memset(frame_buf, 0, sizeof(frame_buf));
  // cdc_rx_len 应该是 CDC_Receive_HS 传入的 *Len
  // uint16_t len = cdc_rx_len;
  // uint8_t *ptr = cdc_rx_cache;

  // 在收到的数据中寻找帧头
  // for (uint16_t i = 0; i < len; i++)
  // {
  //   /* 识别视觉包帧头 */
  //   //if (ptr[i] == 0xA6)
  //   if (ptr[i] == 'S' && ptr[i + 1] == 'P')
  //   {
  //     // 确保剩余长度足够一帧视觉数据
  //     if ((len - i) >= sizeof(vs_receive_packet_t))
  //     {
  //       VS_UnPack_Data_ROS2(&ptr[i], &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));

  //       // 只有解析成功了，才去释放信号量
  //       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //       xSemaphoreGiveFromISR(g_xSemVPC, &xHigherPriorityTaskWoken);
  //       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  //       i += (sizeof(vs_receive_packet_t) - 1); // 跳过已处理字节
  //     }
  //   }
  //   /* 识别导航包帧头 */
  //   else if (ptr[i] == 0xA5)
  //   {
  //     if ((len - i) >= sizeof(nv_receive_packet_t))
  //     {
  //       NV_UnPack_Data_ROS2(&ptr[i], &nv_aim_packet_from_nuc, sizeof(nv_receive_packet_t));
  //       i += (sizeof(nv_receive_packet_t) - 1);
  //     }
  //   }
  // }
}