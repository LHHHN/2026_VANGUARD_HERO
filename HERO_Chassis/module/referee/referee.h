// /*
//  * @file		referee.c/h
//  * @brief	裁判系统、图传串口中断
//  * @history
//  * 版本			作者			编写日期
//  * v1.0.0		姚启杰		2023/4/1	
//  * v1.1.0       范竣杰      2025/3/10    适配裁判系统v1.7.0
 
//  */
// #ifndef _REFEREE_H_
// #define _REFEREE_H_

// #define REFEREE_RXBUFF_SIZE 256
// #define REFEREE_PICBUFF_SIZE 150
// #define REFEREE_TXBUFF_SIZE 128
// #define Offset_SOF_DataLength 1      //裁判系统数据帧偏移
// #define Offset_CRC8 4
// #define Offset_cmd_ID 5
// #define Offset_data 7
// #define CUSTOM_DATA 30
// #define CUSTOM_DATA_2 150

// #define SEND_PIC_MESSAGE(message, len)  USART_Send(refereepic_usart_instance,message,len,USART_TRANSFER_DMA);
// //#define SEND_MESSAGE(message, len)  USART_Send(referee_usart_instance,message,len,USART_TRANSFER_DMA);
// //由于裁判系统接收限制，你可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数替换掉上述函数

// #include "main.h"
// #include "bsp_usart.h"

// typedef enum{
// 	referee_OK = 0,
// 	referee_solving = 1,
// 	referee_error = 2
// }Referee_StatusTypeDef;
// typedef enum
// {
// 	ID_game_status  = 0x0001,
// 	ID_game_result  = 0x0002,
// 	ID_game_robot_hp  = 0x0003,//比赛机器人存活数据
// 	ID_game_dart_state = 0x0004, //飞镖发射状态(2026无)
// 	ID_game_buff  =0x0005,//buff(2026无)

// 	ID_event_data  = 0x0101,//场地事件数据

// 	ID_referee_warn = 0x0104,//裁判系统警告数据
// 	ID_dart_shoot_info =0x0105  , //飞镖发射

// 	ID_game_robot_state = 0x0201,//机器人状态数据
// 	ID_power_heat_data = 0x0202,//实时功率热量数据
// 	ID_game_robot_pos = 0x0203,//机器人位置数据
// 	ID_buff_musk = 0x0204,//机器人增益数据

// 	ID_robot_hurt = 0x0206,//伤害状态数据
// 	ID_shoot_data = 0x0207,//实时射击数据
// 	ID_bullet_remaining  = 0x0208,//剩余发射数
// 	ID_rfid_status	= 0x0209,//机器人RFID状态，3Hz

// 	ID_dart_client_directive = 0x020A,//飞镖机器人客户端指令书, 3Hz
// 	ID_ground_robot_position = 0x020B,//己方哨兵接受地面机器人数据，1Hz
// 	ID_radar_mark_data =0x020C,//雷达标记进度数据，1Hz
// 	ID_sentry_info = 0x020D,//哨兵自主决策信息同步，1Hz
// 	ID_radar_info =0x020E,//雷达自主决策信息同步,1Hz

// 	ID_robot_interactive_header_data = 0x0301,//机器人交互数据，——发送方触发——发送 上限30Hz
// 	ID_controller_interactive_header_data = 0x0302,//自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
// 	ID_map_interactive_header_data  = 0x0303,//客户端小地图交互数据，——触发发送——
// 	ID_keyboard_information = 0x0304,//键盘、鼠标信息，通过——图传串口——发送
// 	ID_map_robot_data = 0x0305,//雷达向己方选手端发送对方机器人坐标
// 	ID_custom_client_data = 0x0306,//自定义控制器向选手端发送数据(不属于任何链路)
// 	ID_map_data = 0x0307,//哨兵/半自动机器人向己方对应选手端发送路径坐标数据
// 	ID_custom_info = 0x0308,//己方机器人向己方任意选手端发送自定义消息
// 	ID_robot_custom_data  = 0x0309,//机器人通过图传链路向对应选手端所连接的自定义控制器发送数据(RMUL暂不适用)
// 	ID_robot_custom_data_2 = 0x0310,//同上
	
// 	ID_enemy_position = 0x0A01,//雷达接收敌方机器人位置信息
// 	ID_enemy_blood = 0x0A02,//雷达接收敌方机器人血量信息
// 	ID_enemy_bullet = 0x0A03,//雷达接收敌方机器人剩余发弹量信息
// 	ID_enemy_state = 0xA04,//雷达接收敌方宏观状态信息
// 	ID_enemy_buff = 0x0A05,//雷达接收敌方机器人增益信息
// 	ID_enemy_disturbance_key = 0x0A06,//雷达接收敌方干扰波密钥

// 	ID_set_channel = 0x0F01,//设置出图信道(1-6)并获取反馈(0成功，1图传启动中，2设置信道有误)
//   ID_get_channel = 0x0F02,//查询当前信道(0为未设置)
// }CmdID;

// typedef enum
// {
// 	/* Std */
// 	LEN_FRAME_HEAD = 5,	// 帧头长度
// 	LEN_CMD_ID 	= 2,	// 命令码长度
// 	LEN_FRAME_TAIL = 2,	// 帧尾CRC16
// 	/* Ext */

// 	LEN_game_state =  11,	//0x0001
// 	LEN_game_result =  1,	//0x0002
// 	LEN_game_robot_hp =  16,	//0x0003  比赛机器人血量数据(2026数据长度变为16位)

// 	LEN_event_data  =  4,	//0x0101  场地事件数据
// 	LEN_supply_projectile_action =  4,	//0x0102场地补给站动作标识数据(2026没有该命令码)
// 	LEN_referee_warn =3, //裁判系统警告 0x0104
// 	LEN_dart_info =3  , //飞镖发射口倒计时

// 	LEN_game_robot_state = 13,	//0x0201机器人状态数据
// 	LEN_power_heat_data = 14,	//0x0202实时功率热量数据 (2026被改为14位数据)
// 	LEN_game_robot_pos = 12,	//0x0203机器人位置数据(文档有点问题？)
// 	LEN_buff_musk =  8,	//0x0204机器人增益数据(2026改为8位)
// 	LEN_aerial_robot_energy  =  2,	//0x0205空中机器人能量状态数据(2026无该命令码)
// 	LEN_robot_hurt =  1,	//0x0206伤害状态数据
// 	LEN_shoot_data =  7,	//0x0207	实时射击数据
// 	LEN_bullet_remaining = 8,//剩余发射数(文档有点问题？)

// 	LEN_rfid_status	= 5,//0x0209 RFID模块数据长度5位
// 	LEN_dart_client_directive = 6,//0x020A飞镖选手端指令数据
// 	LEN_ground_robot_position = 40,//0x020B地面机器人位置数据
//   LEN_radar_mark_data = 2,//0x020C雷达标记数据
// 	LEN_sentry_info = 6,//0x020D哨兵自主决策信息同步
// 	LEN_radar_info = 1,//0x020E雷达自主决策信息同步

// 	LEN_controller_interactive_header_data=30,//0x0302
// 	LEN_map_interactive_header_data=12,//0x0303(文档中该命令码对应数据长度有问题？)
// 	LEN_keyboard_information = 12,//0x0304
// 	LEN_map_robot_data = 24,//0x0305
// 	LEN_custom_client_data = 8,//0x0306
// 	LEN_map_data = 105,//0x0307
// 	LEN_custom_info = 34,//0x0308
// 	LEN_robot_custom_data = 30,//0x0309
// 	LEN_robot_custom_data_2 = 150,//0x0310

// 	LEN_set_channel = 1,//0x0F01
// 	LEN_get_channel = 1,//0x0F02
	
// 	LEN_enemy_position = 24,//0x0A01
// 	LEN_enemy_blood = 12,//0x0A02
// 	LEN_enemy_bullet = 10,//0x0A03
// 	LEN_enemy_state = 6,//0x0A04
// 	LEN_enemy_buff = 36,//0x0A05
// 	LEN_enemy_disturbance_key = 6,//0x0A06
// }JudgeDataLength;

// /* ID: 0x0001  Byte:  11    比赛状态数据 */
// typedef struct
// {
// 	uint8_t game_type : 4;
// 	uint8_t game_progress : 4;
// 	uint16_t stage_remain_time;
// 	uint64_t SyncTimeStamp;
	
// }__attribute__((packed))game_status_t;

// /* ID: 0x0002  Byte:  1    比赛结果数据 */
// typedef struct
// {
// 	uint8_t winner;
// }__attribute__((packed)) ext_game_result_t;

// /* ID: 0x0003  Byte:  16    比赛机器人血量数据 */ 
// typedef struct
// {
// 	uint16_t ally_1_robot_HP;  
// 	uint16_t ally_2_robot_HP;  
// 	uint16_t ally_3_robot_HP;  
// 	uint16_t ally_4_robot_HP;  
// 	uint16_t reserved;  
// 	uint16_t ally_7_robot_HP;  
// 	uint16_t ally_outpost_HP;  
// 	uint16_t ally_base_HP; 
// }__attribute__((packed)) game_robot_HP_t;

// /* ID: 0x0101  Byte:  4    场地事件数据 */
// typedef struct
// {
// 	uint32_t event_data; //bit 21-22：中心增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领（仅RMUL适用）
// }__attribute__((packed)) ext_event_data_t;


// /* ID: 0x0104  Byte: 3   裁判系统警告信息 */
// typedef struct
// {
// 	uint8_t level;
// 	uint8_t offending_robot_id;
// 	uint8_t count;
// }__attribute__((packed)) ext_referee_warning_t;

// /* ID: 0x0105  Byte:3  飞镖发射口倒计时 */
// typedef struct
// {
// 	uint8_t dart_remaining_time;
// 	uint16_t dart_info;
// } __attribute__((packed)) dart_info_t;

// /* ID: 0X0201  Byte: 13    机器人状态数据 */
// typedef struct
// {
// 	uint8_t robot_id;
// 	uint8_t robot_level;
// 	uint16_t current_HP;
// 	uint16_t maximum_HP;
// 	uint16_t shooter_barrel_cooling_value;//机器人枪口热量每秒冷却值

// 	uint16_t shooter_barrel_heat_limit; //枪口热量上限   
// 	uint16_t chassis_power_limit;       //底盘功率上限
	
// 	uint8_t power_management_gimbal_output : 1;
// 	uint8_t power_management_chassis_output : 1;
// 	uint8_t power_management_shooter_output : 1;//可用上
// }__attribute__((packed)) ext_game_robot_state_t;

// /* ID: 0X0202  Byte: 14    实时功率热量数据 */
// typedef struct
// {
// 	uint16_t reserved_1; //改为保留位
// 	uint16_t reserved_2; //改为保留位
// 	float    reserved_3; //改为保留位
// 	uint16_t buffer_energy;
// 	uint16_t shooter_17mm_1_barrel_heat;
// 	uint16_t shooter_42mm_barrel_heat;
// }__attribute__((packed)) ext_power_heat_data_t;

// /* ID: 0x0203  Byte: 16    机器人位置数据 */
// typedef struct
// {
// 	float x;
// 	float y;
// 	float angle;
// }__attribute__((packed)) ext_game_robot_pos_t;

// /* ID: 0x0204  Byte:  8    机器人增益数据 */
// typedef struct
// {
// 	uint8_t recovery_buff;
// 	uint8_t cooling_buff;
// 	uint8_t defence_buff;
// 	uint8_t vulnerability_buff;
// 	uint16_t attack_buff;
// 	uint8_t remaining_energy; 
// }__attribute__((packed)) ext_buff_musk_t;

// /* ID: 0x0206  Byte:  1    伤害状态数据 */
// typedef struct
// {
// 	uint8_t armor_id : 4;
// 	uint8_t hurt_type : 4;
// }__attribute__((packed)) ext_robot_hurt_t;

// /* ID: 0x0207  Byte:  7    实时射击数据 */
// typedef struct
// {
// 	uint8_t bullet_type;
// 	uint8_t shooter_number;
// 	uint8_t launching_frequency;
// 	float initial_speed;
// } __attribute__((packed)) ext_shoot_data_t;


// /* ID: 0x0208  Byte:  8    子弹剩余数量 */
// typedef struct
// {
// 	uint16_t bullet_remaining_num_17mm;
// 	uint16_t bullet_remaining_num_42mm;
// 	uint16_t coin_remaining_num;//金币剩余
// 	uint16_t projectile_allowance_fortress; 
// } __attribute__((packed)) ext_bullet_remaining_t;

// /* ID: 0x0209  Byte:  5 	机器人RFID状态 */
// typedef struct
// {
// 	uint32_t rfid_status;
// 	uint8_t rfid_status_2; 
// }__attribute__((packed)) ext_rfid_status_t;
// /* ID: 0x020A  Byte:  6 */
// typedef struct{
// 	uint8_t dart_launch_opening_status;
// 	uint8_t reserved;
// 	uint16_t target_change_time;
// 	uint16_t latest_launch_cmd_time;
// } __attribute__((packed)) ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  表3-19

// /* ID: 0x020B Byte:  40 地面机器人数据*/
// typedef struct 
// { 
// 	float hero_x;  
// 	float hero_y;  
// 	float engineer_x;  
// 	float engineer_y;  
// 	float standard_3_x;  
// 	float standard_3_y;  
// 	float standard_4_x;  
// 	float standard_4_y;  
// 	float reserved_1;  
// 	float reserved_2; 
// } __attribute__((packed)) ext_ground_robot_position_t; 

// /* ID：0x020C Byte: 2 雷达标记进度数据*/
// typedef struct 
// { 
//     uint16_t mark_progress;  
// } __attribute__((packed)) ext_radar_mark_data_t; 

// /* ID：0x020D Byte :6 哨兵自主决策信息同步*/
// typedef struct 
// {  
// 	uint32_t sentry_info; 
// 	uint16_t sentry_info_2; 
// } __attribute__((packed)) ext_sentry_info_t; 

// /* ID： 0x020E Byte : 1 雷达自主决策信息同步*/
// typedef struct 
// { 
// 	uint8_t radar_info; 
// } __attribute__((packed)) ext_radar_info_t; 

// /* ID : 0x0302 Byte : 30 自定义控制器与机器人交互数据*/
// typedef struct 
// { 
// 	uint8_t data[30]; 
// } __attribute__((packed)) ext_custom_robot_data_t; 

// /* ID：0x0303 Byte ： 15 选手端小地图交互数据*/
// typedef struct 
// { 
// 	float target_position_x; 
// 	float target_position_y; 
// 	uint8_t cmd_keyboard; 
// 	uint8_t target_robot_id; 
// 	uint16_t cmd_source; 
// } __attribute__((packed)) ext_map_command_t; 

// /* ID: 0x0305 Byte ： 24 选手端小地图接收雷达数据，*/
// typedef struct 
// {  
// 	uint16_t hero_position_x; 
// 	uint16_t hero_position_y; 
// 	uint16_t engineer_position_x; 
// 	uint16_t engineer_position_y; 
// 	uint16_t infantry_3_position_x; 
// 	uint16_t infantry_3_position_y; 
// 	uint16_t infantry_4_position_x; 
// 	uint16_t infantry_4_position_y; 
// 	uint16_t infantry_5_position_x; 
// 	uint16_t infantry_5_position_y; 
// 	uint16_t sentry_position_x; 
// 	uint16_t sentry_position_y; 
// } __attribute__((packed)) ext_map_robot_data_t; 

// /* ID : 0x0306 Byte : 8 自定义控制器与选手端交互数据*/
// typedef struct 
// { 
// 	uint16_t key_value; 
// 	uint16_t x_position:12; 
// 	uint16_t mouse_left:4; 
// 	uint16_t y_position:12; 
// 	uint16_t mouse_right:4; 
// 	uint16_t reserved; 
// } __attribute__((packed)) ext_custom_client_data_t; //在这里没有对该命令码的任何操作

// /* ID: 0x0307 Byte : 105 选手端小地图接收哨兵/半自动机器人路径数据*/
// typedef struct 
// { 
// 	uint8_t intention; 
// 	uint16_t start_position_x; //单位dm，下同
// 	uint16_t start_position_y; 
// 	int8_t delta_x[49]; 
// 	int8_t delta_y[49]; 
// 	uint16_t sender_id; 
// } __attribute__((packed)) ext_map_data_t; 

// /* ID: 0x0308 Byte : 34 选手端小地图接收机器人数据，*/
// typedef struct 
// {  
// 	uint16_t sender_id; 
// 	uint16_t receiver_id; 
// 	uint8_t user_data[30]; 
// } __attribute__((packed)) custom_info_t; 

// /* ID: 0x0F01 Byte : 1 设置图传出图信道*/
// typedef struct{
//   uint8_t set_channel_back;
// } __attribute__((packed)) ext_pic_channel_set_t;

// /* ID: 0x0F02 Byte : 1 查询当前出图信道*/
// typedef struct{
//   uint8_t get_channel_back;
// } __attribute__((packed)) ext_pic_channel_get_t;

// /* ID: 0x0A01 Byte : 24 对方机器人的位置坐标*/
// typedef struct 
// {  
// 	uint16_t hero_position_x; 
// 	uint16_t hero_position_y; 
// 	uint16_t engineer_position_x; 
// 	uint16_t engineer_position_y; 
// 	uint16_t infantry_3_position_x; 
// 	uint16_t infantry_3_position_y; 
// 	uint16_t infantry_4_position_x; 
// 	uint16_t infantry_4_position_y; 
// 	uint16_t aerial_position_x; 
// 	uint16_t aerial_position_y; 
// 	uint16_t sentry_position_x; 
// 	uint16_t sentry_position_y; 
// } __attribute__((packed)) ext_map_robot_position_t; 

// /* ID: 0x0A02 Byte : 12 对方机器人的血量信息*/
// typedef struct 
// {  
// 	uint16_t hero_blood;  
// 	uint16_t engineer_blood; 
// 	uint16_t infantry_3_blood; 
// 	uint16_t infantry_4_blood; 
// 	uint16_t reserved;
// 	uint16_t sentry_blood; 
// } __attribute__((packed)) ext_map_robot_blood_t; 

// /* ID: 0x0A03 Byte : 10 对方机器人的剩余发弹量信息*/
// typedef struct 
// {  
// 	uint16_t hero_bullet;  
// 	uint16_t infantry_3_bullet; 
// 	uint16_t infantry_4_bullet; 
// 	uint16_t aerial_bullet;
// 	uint16_t sentry_bullet; 
// } __attribute__((packed)) ext_map_robot_bullet_t; 

// /* ID: 0x0A04 Byte : 6 对方队伍的宏观状态信息*/
// typedef struct 
// {  
// 	uint16_t restcoin;
// 	uint16_t totalcoin;
// 	uint16_t site_state;
// } __attribute__((packed)) ext_enemy_state_t; 

// /* ID: 0x0A05 Byte : 36 对方各机器人当前增益效果*/
// typedef struct 
// {  
// 	uint8_t hero_blood_buff; 
// 	uint16_t hero_shoot_buff; 
// 	uint8_t hero_defense_buff; 
// 	uint8_t hero_neg_defense_buff;
// 	uint16_t hero_attack_buff;
// 	uint8_t engineer_blood_buff; 
// 	uint16_t engineer_shoot_buff; 
// 	uint8_t engineer_defense_buff; 
// 	uint8_t engineer_neg_defense_buff;
// 	uint16_t engineer_attack_buff; 
// 	uint8_t infantry_3_blood_buff; 
// 	uint16_t infantry_3_shoot_buff; 
// 	uint8_t infantry_3_defense_buff; 
// 	uint8_t infantry_3_neg_defense_buff;
// 	uint16_t infantry_3_attack_buff; 
// 	uint8_t infantry_4_blood_buff; 
// 	uint16_t infantry_4_shoot_buff; 
// 	uint8_t infantry_4_defense_buff; 
// 	uint8_t infantry_4_neg_defense_buff;
// 	uint16_t infantry_4_attack_buff;
// 	uint8_t sentry_blood_buff; 
// 	uint16_t sentry_shoot_buff; 
// 	uint8_t sentry_defense_buff; 
// 	uint8_t sentry_neg_defense_buff;
// 	uint16_t sentry_attack_buff; 
// 	uint8_t sentry_state;
// } __attribute__((packed)) ext_map_robot_buff_t;

// /* ID: 0x0A06 Byte : 6 对方干扰波密钥*/
// typedef struct 
// {
// 	uint8_t password1;
// 	uint8_t password2;
// 	uint8_t password3;
// 	uint8_t password4;
// 	uint8_t password5;
// 	uint8_t password6;
// } __attribute__((packed)) ext_disturb_password_t;

// enum judge_robot_ID{
// 	hero_red       = 1,
// 	engineer_red   = 2,
// 	infantry3_red  = 3,
// 	infantry4_red  = 4,
// 	infantry5_red  = 5,
// 	plane_red      = 6,

// 	hero_blue      = 101,
// 	engineer_blue  = 102,
// 	infantry3_blue = 103,
// 	infantry4_blue = 104,
// 	infantry5_blue = 105,
// 	plane_blue     = 106,
// };
// typedef struct{
// 	uint16_t teammate_hero;
// 	uint16_t teammate_engineer;
// 	uint16_t teammate_infantry3;
// 	uint16_t teammate_infantry4;
// 	uint16_t teammate_infantry5;
// 	uint16_t teammate_plane;
// 	uint16_t teammate_sentry;

// 	uint16_t client_hero;
// 	uint16_t client_engineer;
// 	uint16_t client_infantry3;
// 	uint16_t client_infantry4;
// 	uint16_t client_infantry5;
// 	uint16_t client_plane;
// } ext_interact_id_t;
// typedef struct
// {
// 	uint8_t data[113]; //数据段,n需要小于113
// } __attribute__((packed))robot_interactive_data_t;


// typedef struct{

// 	game_status_t 		Game_Status;				// 0x0001           比赛状态数据
// 	ext_game_result_t 		Game_Result;				// 0x0002         比赛结果数据
// 	game_robot_HP_t 	Game_Robot_HP;			// 0x0003         机器人血量数据

// 	ext_event_data_t			Event_Data;					// 0x0101         场地事件数据
// 	ext_referee_warning_t		Referee_Warning;		// 0x0104         裁判警告信息
// 	dart_info_t	dart_info;// 0x0105         飞镖发射口倒计时

// 	ext_game_robot_state_t	Game_Robot_state;	// 0x0201         比赛机器人状态
// 	ext_power_heat_data_t	Power_Heat_Data;		// 0x0202         实时功率热量数据
// 	ext_game_robot_pos_t	Game_Robot_Pos;			// 0x0203         机器人位置
// 	ext_buff_musk_t		    Buff_Musk;						// 0x0204     机器人增益
// 	ext_robot_hurt_t		Robot_Hurt;					// 0x0206         伤害状态
// 	ext_shoot_data_t		Shoot_Data;					// 0x0207         实时射击信息(射频  射速  子弹信息)
// 	ext_bullet_remaining_t	bullet_remaining;		// 0x0208	        子弹剩余发射数
// 	ext_rfid_status_t		rfid_status;				// 0x0209	        RFID信息
// 	ext_dart_client_cmd_t   dart_client;        // 0x020A         飞镖客户端
//   ext_ground_robot_position_t ground_robot_position;   //0x020B      地面机器人位置数据
// 	ext_radar_mark_data_t       radar_mark_data ;     //0x020C    雷达标记数据
// 	ext_sentry_info_t           sentry_info;              //0x020D    哨兵自主决策信息同步
// 	ext_radar_info_t            radar_info;                //0x020E    雷达自主决策信息同步

// 	ext_map_command_t               map_cmmmand;               //0x0303    选手端下发数据 
	
// 	ext_map_robot_position_t    get_map_robot_position;          
// 	ext_map_robot_blood_t       get_map_robot_blood;
// 	ext_map_robot_bullet_t      get_map_robot_bullet;
// 	ext_enemy_state_t           get_enemy_state;
// 	ext_map_robot_buff_t        get_enemy_buff;
// 	ext_disturb_password_t      password;
// 	ext_interact_id_t		    ids;								//与本机交互的机器人id
// 	uint16_t                    self_client;        //本机客户端

// }Referee_InfoTypedef;

// typedef struct{
// 		ext_custom_robot_data_t     custom_robot_data;            //0x0302    操作手可使用自定义控制器通过图传链路向对应的机器人发送数据
	  
// 		ext_pic_channel_set_t       pic_channel_set;              //0x0F01    设置出图信道
//     ext_pic_channel_get_t       pic_channel_get;              //0x0F02    查询出图信道
// }Referee_PicInfoTypedef;

// typedef struct {
// 	uint8_t SOF;
// 	uint16_t length;
// 	uint8_t seq, crc8;
// 	uint16_t cmd_id;
// } __attribute__((packed)) frame_header_t;

// typedef struct {                        
// 	frame_header_t header;                  
// 	uint8_t data;                      
// 	uint16_t crc16;                         
// } __attribute__((packed)) Pic_setchannel_data_t;

// typedef struct {                        
// 	frame_header_t header;                                      
// 	uint16_t crc16;                         
// } __attribute__((packed)) Pic_getchannel_data_t;

// typedef struct {                        
// 	frame_header_t header;                  
// 	uint8_t data[CUSTOM_DATA];                      
// 	uint16_t crc16;                         
// } __attribute__((packed)) Robot_CUSTOM_DATA_t;

// typedef struct {                        
// 	frame_header_t header;                  
// 	uint8_t data[CUSTOM_DATA_2];                      
// 	uint16_t crc16;                         
// } __attribute__((packed)) Robot_CUSTOM_DATA_2_t;

// typedef struct{
//   frame_header_t header;                  
// 	ext_map_robot_data_t map_robot_data;                     
// 	uint16_t crc16; 
// } __attribute__((packed)) Radar_send_robotdata_t;

// typedef struct{
//   frame_header_t header;                  
// 	ext_map_data_t map_data;                     
// 	uint16_t crc16; 
// } __attribute__((packed)) Autorobot_send_mapdata_t;

// typedef struct{
//   frame_header_t header;                  
// 	custom_info_t custom_info;                     
// 	uint16_t crc16; 
// } __attribute__((packed)) Robot_send_customdata_t;

// typedef struct
// {
// 	int16_t mouse_x;
// 	int16_t mouse_y;
// 	int16_t mouse_z;
// 	int8_t left_button_down;
// 	int8_t right_button_down;
// 	uint16_t keyboard_value;
// 	uint16_t reserved;
// }__attribute__((packed))ext_robot_command_t;

// void RefereeInit();
// void RefereeSolve(uint8_t *data);
// void Set_Pic_Channel(uint8_t i,Pic_setchannel_data_t *msg);
// void Get_Pic_Channel(Pic_getchannel_data_t *msg);
// void Robot_Custom_Senddata(uint16_t cmd_id,Robot_CUSTOM_DATA_t *msg,Robot_CUSTOM_DATA_2_t *msg_1);
// void Radar_send_player(Radar_send_robotdata_t *msg);
// void Autorobot_send_player(Autorobot_send_mapdata_t *msg);
// void Robot_send_customdata(Robot_send_customdata_t *msg);
// Referee_InfoTypedef *Referee_Init(UART_HandleTypeDef *referee_usart_handle);
// Referee_PicInfoTypedef *RefereePic_Init(UART_HandleTypeDef *refereepic_usart_handle);

// extern USART_instance_t *referee_usart_instance;
// extern uint8_t referee_tx_buf[REFEREE_TXBUFF_SIZE];
// extern uint8_t referee_rx_buf[REFEREE_RXBUFF_SIZE];           //dma接收区
// extern uint8_t referee_pic_rx_buf[REFEREE_PICBUFF_SIZE];
// extern uint8_t referee_rx_len;    //裁判系统串口idle中断接收数据长度
// extern uint8_t referee_pic_rx_len;

// #endif
