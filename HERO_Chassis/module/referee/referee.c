/*
 * @file		referee.c/h
 * @brief	裁判系统、图传串口中断
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		姚启杰		2023/4/1		裁判系统、图传链路通信
 *	v1.1		郑煜壅		2023/11/3		更改部分代码风格
 *	V2.0		YQJ			2024/4/1		无需再注释库函数
 */
#include "main.h"
#include "usart.h"
#include "dma.h"
#include "string.h"
#include "referee.h"
#include "remote_control.h"
#include "CRC.h"
#include "ui_interface.h"
#include "defense_center.h"

uint8_t referee_rx_len;    //裁判系统串口idle中断接收数据长度
uint8_t referee_pic_rx_len;
uint8_t referee_rx_buf[REFEREE_RXBUFF_SIZE];           //dma接收区
uint8_t referee_pic_rx_buf[REFEREE_PICBUFF_SIZE];		//图传数据
uint8_t referee_tx_buf[REFEREE_TXBUFF_SIZE];                        //maximum 128 bytes

USART_instance_t *referee_usart_instance;  //串口实例
static supervisor_t *referee_supervisor_instance; // 监视器实例

static USART_instance_t *refereepic_usart_instance;  //串口实例
static supervisor_t *refereepic_supervisor_instance; // 监视器实例

static Referee_InfoTypedef refree_info;             //裁判系统数据
static Referee_PicInfoTypedef referee_picinfo;

static uint8_t refree_init_flag = 0; // 裁判系统初始化标志位
static uint8_t refreepic_init_flag = 0;

/* 键盘/鼠标命令缓存（LEN_keyboard_information == 12） */
static ext_robot_command_t get_vaule;

static Pic_setchannel_data_t     set_channel_data;             //设置图传信道的数据包
static Pic_getchannel_data_t     get_channel_data;             //查询图传信道的数据包
static Robot_CUSTOM_DATA_t       send_custom_data_1;             //向自定义控制器发送自定义数据
static Robot_CUSTOM_DATA_2_t     send_custom_data_2;           //同上
static Radar_send_robotdata_t    send_robot_data;             //雷达发送敌方机器人信息到选手端
static Autorobot_send_mapdata_t  send_route_data;           //哨兵机器人或半自动控制方式的机器人通过常规链路向对应的操作手选手端发送路径坐标数据
static Robot_send_customdata_t   send_custom_data;           //己方机器人可通过常规链路向己方任意选手端发送自定义的消息

void RefereeInit()
{
	if(USART_Is_Ready(refereepic_usart_instance))
	  Set_Pic_Channel(1,&set_channel_data);
}

/* 为方便，以下为单个命令码所设置的函数的crc校验与ui的校验是同一种*/
/* 2026新增命令码0x0F01 */
void Set_Pic_Channel(uint8_t i,Pic_setchannel_data_t *msg){
			msg->header.SOF = 0xA5;
			msg->header.length = 1;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0F01;
			msg->data = i;//设置出图信道为i(i为1-6)
			msg->crc16 = calc_crc16((uint8_t *) msg, 8);
			SEND_PIC_MESSAGE((uint8_t *) &msg, sizeof(msg));
}

/* 获取当前图传信道 0x0F02*/
void Get_Pic_Channel(Pic_getchannel_data_t *msg){
			msg->header.SOF = 0xA5;
			msg->header.length = 0;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0F02;
			msg->crc16 = calc_crc16((uint8_t *) msg, 7);
		  SEND_PIC_MESSAGE((uint8_t *) &msg, sizeof(msg));
}

/* 发送自定义信息函数 命令码为0x0309(发给自定义控制器)或0x0310(发给自定义客户端) 注意此函数的数据段数据没有填写，需按需填写 */
void Robot_Custom_Senddata(uint16_t cmd_id,Robot_CUSTOM_DATA_t *msg,Robot_CUSTOM_DATA_2_t *msg_1){
	if(cmd_id==0x0309)
	{
		  msg->header.SOF = 0xA5;
			msg->header.length = CUSTOM_DATA;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0309;
			//msg->data = data;该数据为最大长度为30的数组，可自行修改函数形参传入参数
			msg->crc16 = calc_crc16((uint8_t *) msg, CUSTOM_DATA+7);
			SEND_PIC_MESSAGE((uint8_t *) &msg, sizeof(msg));
	}
	else if(cmd_id==0x0310){
			msg_1->header.SOF = 0xA5;
			msg_1->header.length = CUSTOM_DATA_2;
			msg_1->header.seq = seq++;
			msg_1->header.crc8 = calc_crc8((uint8_t *) msg_1, 4);
			msg_1->header.cmd_id = 0x0310;
			//msg->data = data;该数据为最大长度为150的数组，自行选择传入参数
			msg_1->crc16 = calc_crc16((uint8_t *) msg_1, CUSTOM_DATA_2+7);
			SEND_PIC_MESSAGE((uint8_t *) &msg_1, sizeof(msg_1));
	}
}

/* 雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据 0x0305 注意此函数的数据段数据没有填写，需根据雷达所得数据填写
uint16_t hero_position_x; uint16_t hero_position_y; 英雄坐标，单位cm，下同
uint16_t engineer_position_x; uint16_t engineer_position_y; 工程坐标 
uint16_t infantry_3_position_x; uint16_t infantry_3_position_y; 3号步兵坐标
uint16_t infantry_4_position_x; uint16_t infantry_4_position_y; 4号
uint16_t infantry_5_position_x; uint16_t infantry_5_position_y; 5号
uint16_t sentry_position_x; uint16_t sentry_position_y;哨兵坐标 */
void Radar_send_player(Radar_send_robotdata_t *msg){
			msg->header.SOF = 0xA5;
			msg->header.length = 24;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0305;
			//所发送数据，msg->map_robot_data.hero_position_x=...;...可为函数增加形参，将数据打包成结构体一同传入函数
			msg->crc16 = calc_crc16((uint8_t *) msg, 31);
		  SEND_MESSAGE((uint8_t *) &msg, sizeof(msg));
}

/* 哨兵机器人或半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，该路径会在小地图上显示 0x0307
uint8_t intention; 1：到目标点攻击 2：到目标点防守 3：移动到目标点
uint16_t start_position_x; 路径起点x轴坐标//单位dm，下同
uint16_t start_position_y; 路径起点y轴坐标
int8_t delta_x[49]; 路径点x轴增量数组
int8_t delta_y[49]; 路径点y轴增量数组，增量相较于上一个点位进行计算，共49个新点位，X与Y轴增量对应组成点位 
uint16_t sender_id; 发送者ID  */
void Autorobot_send_player(Autorobot_send_mapdata_t *msg){
			msg->header.SOF = 0xA5;
			msg->header.length = 105;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0307;
			//所发送数据，可为函数增加形参，将数据打包成结构体一同传入函数
			msg->map_data.sender_id = refree_info.Game_Robot_state.robot_id;
			msg->crc16 = calc_crc16((uint8_t *) msg, 112);
			SEND_MESSAGE((uint8_t *) &msg, sizeof(msg));
}

/* 己方机器人可通过常规链路向己方任意选手端发送自定义的消息 0x0308*/
void Robot_send_customdata(Robot_send_customdata_t *msg){
			msg->header.SOF = 0xA5;
			msg->header.length = 34;
			msg->header.seq = seq++;
			msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
			msg->header.cmd_id = 0x0308;
			msg->custom_info.sender_id = refree_info.Game_Robot_state.robot_id;
			//msg->custom_info.receiver_id = 任意己方选手端id
			//所发送数据
			msg->crc16 = calc_crc16((uint8_t *) msg, 41);
			SEND_MESSAGE((uint8_t *) &msg, sizeof(msg));
}

void RefereeSolve(uint8_t *data)
{

	uint16_t offset_frame_tail = (data[Offset_SOF_DataLength + 1]<<8) + data[Offset_SOF_DataLength]
							+LEN_CMD_ID +LEN_FRAME_HEAD;
	uint16_t cmd_id = ((uint16_t)data[Offset_cmd_ID + 1] << 8) | (uint16_t)data[Offset_cmd_ID];

	if(data[0] != 0xA5 || !Verify_CRC16_Check_Sum(data, offset_frame_tail +2)
			           || !Verify_CRC8_Check_Sum(data, LEN_FRAME_HEAD))
	{
		return;
	}

	switch(cmd_id)
	{
	//0x000
	case ID_game_status:
		memcpy(&(refree_info.Game_Status), (data + Offset_data), LEN_game_state);
		break;
	case ID_game_result:
		memcpy(&(refree_info.Game_Result), (data + Offset_data), LEN_game_result);
		break; 
	case ID_game_robot_hp:
		memcpy(&(refree_info.Game_Robot_HP), (data + Offset_data), LEN_game_robot_hp); //v1.4中有冲突，按sof中为准
		break;

	//0x100
	case ID_event_data:
			memcpy(&(refree_info.Event_Data), (data + Offset_data), LEN_event_data);
			break;
	case ID_referee_warn:
			memcpy(&(refree_info.Referee_Warning), (data + Offset_data), LEN_referee_warn);
			break;
	case ID_dart_shoot_info:
			memcpy(&(refree_info.dart_info), (data + Offset_data), LEN_dart_info);
			break;

	//0x200
	case ID_game_robot_state:
			memcpy(&(refree_info.Game_Robot_state), (data + Offset_data), LEN_game_robot_state);
			break;
	case ID_power_heat_data:
			memcpy(&(refree_info.Power_Heat_Data), (data + Offset_data), LEN_power_heat_data);
			break;
	case ID_game_robot_pos:
			memcpy(&(refree_info.Game_Robot_Pos), (data + Offset_data), LEN_game_robot_pos);
			break;
	case ID_buff_musk:
			memcpy(&(refree_info.Buff_Musk), (data + Offset_data), LEN_buff_musk);
			break;
	case ID_robot_hurt:
			memcpy(&(refree_info.Game_Status), (data + Offset_data), LEN_robot_hurt);
			break;
	case ID_shoot_data:
			memcpy(&(refree_info.Shoot_Data), (data + Offset_data), LEN_shoot_data);//v1.4中冲突，以sof中为准
			break;
	case ID_bullet_remaining:
			memcpy(&(refree_info.bullet_remaining), (data + Offset_data), LEN_bullet_remaining);//v1.4中冲突，以sof中为准
			break;
	case ID_rfid_status:
			memcpy(&(refree_info.rfid_status), (data + Offset_data), LEN_rfid_status);
			break;
	case ID_dart_client_directive:
			memcpy(&(refree_info.dart_client), (data + Offset_data), LEN_dart_client_directive);
			break;
	case ID_ground_robot_position:
	    memcpy(&(refree_info.ground_robot_position), (data + Offset_data), LEN_ground_robot_position);
			break;
	case ID_radar_mark_data:
	    memcpy(&(refree_info.radar_mark_data), (data + Offset_data), LEN_radar_mark_data);
			break;
  case ID_sentry_info:
	    memcpy(&(refree_info.sentry_info), (data + Offset_data), LEN_sentry_info);
			break;
  case ID_radar_info:
	    memcpy(&(refree_info.radar_info), (data + Offset_data), LEN_radar_info);
			break;
	
	//0x0300
  case ID_map_interactive_header_data://0x0303
	    memcpy(&(refree_info.map_cmmmand), (data + Offset_data), LEN_map_interactive_header_data);
			break;
	
	//0x0A00
	case ID_enemy_position:
		  memcpy(&(refree_info.get_map_robot_position), (data + Offset_data), LEN_enemy_position);
			break;
	case ID_enemy_blood:
		  memcpy(&(refree_info.get_map_robot_blood), (data + Offset_data), LEN_enemy_blood);
			break;
	case ID_enemy_bullet:
		  memcpy(&(refree_info.get_map_robot_bullet), (data + Offset_data), LEN_enemy_bullet);
			break;
	case ID_enemy_state:
		  memcpy(&(refree_info.get_enemy_state), (data + Offset_data), LEN_enemy_state);
			break;
	case ID_enemy_buff:
		  memcpy(&(refree_info.get_enemy_buff), (data + Offset_data), LEN_enemy_buff);
			break;
	case ID_enemy_disturbance_key:
		  memcpy(&(refree_info.password), (data + Offset_data), LEN_enemy_disturbance_key);
			break;
	
	
//#ifndef USE_REMOTE_KEYBORAD
//	case ID_keyboard_information:
//		memcpy(&get_vaule, (data + Offset_data), LEN_keyboard_information);
//		break;
//#endif
	}

	if(data[offset_frame_tail+2] == 0xA5)
		RefereeSolve(data +offset_frame_tail +2);
	return;

}

void RefereePicSolve(uint8_t *data)
{

	uint16_t offset_frame_tail = (data[Offset_SOF_DataLength + 1]<<8) + data[Offset_SOF_DataLength]
							+LEN_CMD_ID +LEN_FRAME_HEAD;
	uint16_t cmd_id = ((uint16_t)data[Offset_cmd_ID + 1] << 8) | (uint16_t)data[Offset_cmd_ID];
	if(data[0] != 0xA5 || !Verify_CRC16_Check_Sum(data, offset_frame_tail +2)
			           || !Verify_CRC8_Check_Sum(data, LEN_FRAME_HEAD))
	{
		return;
	}

	switch(cmd_id)
	{
				//0x0300
		case ID_controller_interactive_header_data://0x0302
				memcpy(&(referee_picinfo.custom_robot_data), (data + Offset_data), LEN_controller_interactive_header_data);
				break;
		
				//0x0F00
		case ID_set_channel:
				memcpy(&(referee_picinfo.pic_channel_set), (data + Offset_data), LEN_set_channel);
				break;
		case ID_get_channel:
				memcpy(&(referee_picinfo.pic_channel_get), (data + Offset_data), LEN_get_channel);
				break;
		
#ifndef USE_REMOTE_KEYBORAD
	case ID_keyboard_information:
								memcpy(&get_vaule, (data + Offset_data), LEN_keyboard_information);
								break;
#endif
	}
		if(data[offset_frame_tail+2] == 0xA5)
		RefereePicSolve(data +offset_frame_tail +2);
	  return;
}


static void RefereeSolve_Rx_Callback(void)
{
	Supervisor_Reload(referee_supervisor_instance);       // 先喂狗
	RefereeSolve(referee_usart_instance->recv_buff); // 进行协议解析
}

static void RefereePicSolve_Rx_Callback(void)
{
	Supervisor_Reload(refereepic_supervisor_instance);       // 先喂狗
	RefereePicSolve(refereepic_usart_instance->recv_buff); // 进行协议解析
}

static void Referee_Lost_Callback(void *id)
{
	memset(&refree_info, 0, sizeof(refree_info)); // 清空接收数据
	USART_Service_Init(referee_usart_instance); // 尝试重新启动接收
}

static void Refereepic_Lost_Callback(void *id)
{
	memset(&referee_picinfo, 0, sizeof(referee_picinfo)); // 清空接收数据
	USART_Service_Init(refereepic_usart_instance); // 尝试重新启动接收
}

Referee_InfoTypedef *Referee_Init(UART_HandleTypeDef *referee_usart_handle)//进行常规链路串口的注册
{
	usart_init_config_t conf;
	conf.module_callback = RefereeSolve_Rx_Callback;
	conf.usart_handle    = referee_usart_handle;
	conf.recv_buff_size  = REFEREE_RXBUFF_SIZE;
	referee_usart_instance    = USART_Register(&conf);

	// 进行守护进程的注册,用于定时检查裁判系统是否正常工作
	supervisor_init_config_t supervisor_conf = {
		.reload_count = 1000, // 100ms未收到数据视为离线？
		.handler_callback = Referee_Lost_Callback,
		.owner_id = NULL, 
	};
  referee_supervisor_instance = Supervisor_Register(&supervisor_conf);

	refree_init_flag = 1;
	return (&refree_info);
}

Referee_PicInfoTypedef *RefereePic_Init(UART_HandleTypeDef *refereepic_usart_handle)//进行图传链路串口的注册
{
	usart_init_config_t conf;
	conf.module_callback = RefereePicSolve_Rx_Callback;
	conf.usart_handle    = refereepic_usart_handle;
	conf.recv_buff_size  = REFEREE_PICBUFF_SIZE;
	refereepic_usart_instance    = USART_Register(&conf);

	// 进行守护进程的注册,用于定时检查裁判系统是否正常工作
	supervisor_init_config_t supervisor_conf = {
		.reload_count = 1000, // 100ms未收到数据视为离线？
		.handler_callback = Refereepic_Lost_Callback,
		.owner_id = NULL, 
	};
	refereepic_supervisor_instance = Supervisor_Register(&supervisor_conf);

	refreepic_init_flag = 1;
	return (&referee_picinfo);
}


