/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "rm_referee.h"
#include "referee_UI.h"

#if REFEREE_RAW
#else
referee_info_t *referee_outer_info;
Referee_Interactive_info_t referee_outer_interactive;
#endif

#if REFEREE_RAW
#else
static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID(void)
{
  // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
  referee_recv_info->referee_id.Robot_Color =
      referee_recv_info->RobotPerformance.robot_id > 7 ?
	  Robot_Blue :
	  Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->RobotPerformance.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info,
			Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t* UI_Task_Init(UART_HandleTypeDef *referee_usart_handle,
			   Referee_Interactive_info_t *UI_data)
{
  referee_recv_info = Referee_Register(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
  Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
  referee_recv_info->init_flag = 1;
  return referee_recv_info;
}

void UI_Task(void)
{
#if referee_outer_info
  RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
#else
#endif
  MyUIRefresh(referee_recv_info, Interactive_data);
}

static String_Data_t UI_sign_logo[7];
static Graph_Data_t UI_measure_digital[14];
static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_shoot_arrow[14];
static Graph_Data_t UI_Energy[14];      // 电容能量条
static Graph_Data_t UI_energy_line[5];
static String_Data_t UI_State_Sta_String[7];  // 机器人状态,静态只需画一次
static Graph_Data_t UI_State_dyn_Graph[7];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[7] = {
    540 ,
    960 ,
    490 ,
    515 ,
    565 ,
    400 ,
    300 , };

static void UI_Shoot_Line(void)
{
  // 绘制发射基准线
  UI_Line_Draw(&UI_shoot_line[0],
	     "sa0",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     3,
	     710,
	     shoot_line_location[0],
	     1210,
	     shoot_line_location[0]);
  UI_Line_Draw(&UI_shoot_line[1],
	     "sa1",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     3,
	     shoot_line_location[1],
	     340,
	     shoot_line_location[1],
	     740);
  UI_Line_Draw(&UI_shoot_line[2],
	     "sa2",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     2,
	     985,
	     0,
	     985,
	     shoot_line_location[2]);
  UI_Line_Draw(&UI_shoot_line[3],
	     "sa3",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     2,
	     935,
	     0,
	     935,
	     shoot_line_location[2]);
  UI_Line_Draw(&UI_shoot_line[4],
	     "sa4",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     2,
	     810,
	     shoot_line_location[4],
	     1110,
	     shoot_line_location[4]);
  UI_Line_Draw(&UI_shoot_line[5],
	     "sa5",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     2,
	     810,
	     shoot_line_location[5],
	     1110,
	     shoot_line_location[5]);
  UI_Line_Draw(&UI_shoot_line[6],
	     "sa6",
	     UI_Graph_ADD,
	     9,
	     UI_Color_White,
	     2,
	     810,
	     shoot_line_location[6],
	     1110,
	     shoot_line_location[6]);

  UI_Graph_Refresh(&referee_recv_info->referee_id,
		 7,
		 UI_shoot_line[0],
		 UI_shoot_line[1],
		 UI_shoot_line[2],
		 UI_shoot_line[3],
		 UI_shoot_line[4],
		 UI_shoot_line[5],
		 UI_shoot_line[6]);
}

static void UI_Energy_Line(void)
{
      UI_Arc_Draw(&UI_energy_line[0],
		"sb0",
		UI_Graph_ADD,
		9,
		UI_Color_Black,
		10,
		75,
		15,
		960,
		540,
		300,
		300);
      UI_Graph_Refresh(&referee_recv_info->referee_id,
		     1,
		     UI_energy_line[0]);
}

static void UI_State_Mode_Data(void)
{
 // 绘制车辆状态标志指示
 UI_Char_Draw(&UI_State_Sta_String[0],
	     "ss0",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     750,
	     "chassis:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[0]);
 UI_Char_Draw(&UI_State_Sta_String[1],
	     "ss1",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     700,
	     "gimbal:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[1]);
 UI_Char_Draw(&UI_State_Sta_String[2],
	     "ss2",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     650,
	     "shoot:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[2]);
 UI_Char_Draw(&UI_State_Sta_String[3],
	     "ss3",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     600,
	     "frict:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[3]);
 UI_Char_Draw(&UI_State_Sta_String[4],
	     "ss4",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     550,
	     "ammo:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[4]);
 UI_Char_Draw(&UI_State_Sta_String[5],
	     "ss5",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     850,
	     "none:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[6]);
 UI_Char_Draw(&UI_State_Sta_String[6],
	     "ss6",
	     UI_Graph_ADD,
	     8,
	     UI_Color_Cyan,
	     15,
	     2,
	     150,
	     800,
	     "control:");
 UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[6]);

 UI_Arc_Draw(&UI_State_dyn_Graph[0],
		"sr0",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		790,
		10,
		10);
 UI_Arc_Draw(&UI_State_dyn_Graph[1],
		"sr1",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		740,
		10,
		10);
 UI_Arc_Draw(&UI_State_dyn_Graph[2],
		"sr2",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		690,
		10,
		10);
 UI_Arc_Draw(&UI_State_dyn_Graph[3],
		"sr3",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		640,
		10,
		10);
 UI_Arc_Draw(&UI_State_dyn_Graph[4],
		"sr4",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		590,
		10,
		10);
 UI_Arc_Draw(&UI_State_dyn_Graph[5],
		"sr5",
		UI_Graph_ADD,
		8,
		UI_Color_Purplish_red,
		0,
		360,
		5,
		275,
		540,
		10,
		10);	
 // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
 UI_Arc_Draw(&UI_State_dyn_Graph[6],
	    "sr6",
		UI_Graph_ADD,
		8,
		UI_Color_Main,
		0,
		360,
		5,
		20,
		900,
		10,
		10);	
 UI_Graph_Refresh(&referee_recv_info->referee_id,
		 7,
		 UI_State_dyn_Graph[0],
		 UI_State_dyn_Graph[1],
		 UI_State_dyn_Graph[2],
		 UI_State_dyn_Graph[3],
		 UI_State_dyn_Graph[4],
		 UI_State_dyn_Graph[5],
		 UI_State_dyn_Graph[6]);
}

void User_UI_Init()
{
  if (!referee_recv_info->init_flag)
  {
    osDelay(10000); // 如果没有初始化裁判系统则直接删除ui任务
  }
  else
  {
    while (referee_recv_info->RobotPerformance.robot_id == 0)
    {
      osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
    }

    DeterminRobotID();                                         // 确定ui要发送到的目标客户端
    UI_Delete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    UI_Shoot_Line();

    UI_State_Mode_Data();

  }
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
  count++;
  if (count >= 50)
  {
    count = 0;
    count1++;
  }
  switch (count1 % 2)
  {
    case 0:
    {
    //   _Interactive_data->chassis_mode = CHASSIS_STOP;
    //   _Interactive_data->gimbal_mode = GIMBAL_STOP;
    //   _Interactive_data->shoot_mode = SHOOTER_STOP;
    //   _Interactive_data->friction_mode = FRICTION_ON;
    //   _Interactive_data->ammo_mode = AMMO_OPEN;
      _Interactive_data->Chassis_Power_Limit += 1.5;
      if (_Interactive_data->Chassis_Power_Limit >= 70)
      {
	_Interactive_data->Chassis_Power_Limit = 0;
      }
      break;
    }
    case 1:
    {
    //   _Interactive_data->chassis_mode = CHASSIS_NORMAL;
    //   _Interactive_data->gimbal_mode = GIMBAL_NORMAL;
    //   _Interactive_data->shoot_mode = SHOOTER_AUTO;
    //   _Interactive_data->friction_mode = FRICTION_OFF;
    //   _Interactive_data->ammo_mode = AMMO_CLOSE;
      _Interactive_data->Chassis_Power_Limit -= 0.1;
      if (_Interactive_data->Chassis_Power_Limit <= 0)
      {
	_Interactive_data->Chassis_Power_Limit = 25;
      }
      break;
    }
    case 2:
    {
    //   _Interactive_data->chassis_mode = CHASSIS_SPIN;
    //   _Interactive_data->gimbal_mode = GIMBAL_MAINTAIN;
    //   _Interactive_data->shoot_mode = SHOOTER_SINGLE;
    //   _Interactive_data->friction_mode = FRICTION_ON;
    //   _Interactive_data->ammo_mode = AMMO_OPEN;
      _Interactive_data->Chassis_Power_Limit += 1.5;
      if (_Interactive_data->Chassis_Power_Limit >= 70)
      {
	_Interactive_data->Chassis_Power_Limit = 0;
      }
      break;
    }
    case 3:
    {
    //   _Interactive_data->chassis_mode = CHASSIS_FOLLOW;
    //   _Interactive_data->gimbal_mode = GIMBAL_AUTO;
    //   _Interactive_data->shoot_mode = SHOOTER_AUTO;
    //   _Interactive_data->friction_mode = FRICTION_OFF;
    //   _Interactive_data->ammo_mode = AMMO_CLOSE;
      _Interactive_data->Chassis_Power_Limit -= 0.1;
      if (_Interactive_data->Chassis_Power_Limit <= 0)
      {
	_Interactive_data->Chassis_Power_Limit = 25;
      }
      break;
    }
    default:
      break;
  }
}

static void MyUIRefresh(referee_info_t *referee_recv_info,
			Referee_Interactive_info_t *_Interactive_data)
{
  static uint32_t refresh_cnt;
  static uint8_t refresh_flag;

  static uint8_t error_code = 0;
  static uint8_t error_last_code = 0;
  static uint8_t error_flag = 0;

  refresh_flag = 0;
  refresh_cnt++;

  _Interactive_data->Chassis_Power_Limit = 0.0f;//referee_outer_info->RobotPerformance.chassis_power_limit;

  _Interactive_data->control_mode = 0;//control_cmd.control;
  _Interactive_data->ammo_mode = 0;//gimbal_message_chat.ammo;
  _Interactive_data->friction_mode = 0;//gimbal_message_chat.friction_state;
  _Interactive_data->chassis_mode = 0;//chassis_cmd.chassis;
  _Interactive_data->gimbal_mode = 0;//gimbal_cmd.gimbal;
  _Interactive_data->shoot_mode = 0;//shoot_cmd.shooter;

  UIChangeCheck(_Interactive_data);

  if (_Interactive_data->Referee_Interactive_Flag.control_flag == 1)
  {
    switch (_Interactive_data->control_mode)
    {
      case 0://CONTROL_NONE:

	break;
      case 1://CONTROL_REMOTE:

	// 此处注意字数对齐问题，字数相同才能覆盖掉
	break;
      case 2://CONTROL_PC:

	break;
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[6]);

    _Interactive_data->Referee_Interactive_Flag.control_flag = 0;
  }

  // chassis
  if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
  {
    switch (_Interactive_data->chassis_mode)
    {
      case 0://CHASSIS_STOP:

	break;
      case 1://CHASSIS_NORMAL:
	// 此处注意字数对齐问题，字数相同才能覆盖掉
	break;
      case 2://CHASSIS_FOLLOW:

	break;
      case 3://CHASSIS_SPIN:

	break;
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[0]);

    _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
  }
  // gimbal
  if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
  {
    switch (_Interactive_data->gimbal_mode)
    {
      case 0://GIMBAL_STOP:
      {
	break;
      }
      case 1://GIMBAL_NORMAL:
      {
	break;
      }
      case 2://GIMBAL_AUTO:
      {
	break;
      }
      case 3://GIMBAL_MAINTAIN:
      {
	break;
      }
      case 4://GIMBAL_ZERO:
      {
	break;
      }
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[1]);
    _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
  }
  // shoot
  if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
  {
    switch (_Interactive_data->shoot_mode)
    {
      case 0://SHOOTER_STOP:
	break;
      case 1://SHOOTER_SINGLE:
	break;
      case 2://SHOOTER_AUTO:
	break;
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[2]);
    _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
  }
  // friction
  if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
  {
    switch (_Interactive_data->friction_mode)
    {
      case 0://FRICTION_OFF:
	break;
      case 1://FRICTION_ON:
	break;
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[3]);
    _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
  }
  // ammo
  if (_Interactive_data->Referee_Interactive_Flag.ammo_flag == 1)
  {
    switch (_Interactive_data->ammo_mode)
    {
      case 0://AMMO_CLOSE:
	break;
      case 1://AMMO_OPEN:
	break;
    }
    // UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_dyn_Graph[4]);
    _Interactive_data->Referee_Interactive_Flag.ammo_flag = 0;
  }

  // power
  if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
  {
    UI_Float_Draw(&UI_Energy[1],
		"sf1",
		UI_Graph_Change,
		7,
		UI_Color_Green,
		18,
		2,
		2,
		750,
		230,
		_Interactive_data->Chassis_Power_Limit * 1000);
    UI_Line_Draw(&UI_Energy[2],
	       "sf2",
	       UI_Graph_Change,
	       7,
	       UI_Color_Pink,
	       30,
	       720,
	       160,
	       (uint32_t) 650 + _Interactive_data->Chassis_Power_Limit * 5,
	       160);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_Energy[1],
		   UI_Energy[2]);
    _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
  }

  error_code = 0;//super_cap->receive_data.errorCode;
  if (error_code != error_last_code)
  {
    error_flag = 1;
  }
  else
  {
    error_flag = 0;
  }

  if ((refresh_cnt % 3) == 0)
  {
    if (error_flag == 1)
    {
    }

    //   UI_Arc_Draw(&UI_energy_line[0],
	// 	"sc0",
	// 	UI_Graph_Change,
	// 	9,
	// 	UI_Color_Black,
	// 	15,
	// 	75,
	// 	15,
	// 	960,
	// 	540,
	// 	300,
	// 	300);
    //   UI_Arc_Draw(&UI_energy_line[1],
	// 	"sc1",
	// 	UI_Graph_Change,
	// 	9,
	// 	UI_Color_Black,
	// 	285,
	// 	345,
	// 	15,
	// 	960,
	// 	540,
	// 	300,
	// 	300);
    //   UI_Arc_Draw(&UI_energy_line[2],
	// 	"sc2",
	// 	UI_Graph_Change,
	// 	9,
	// 	UI_Color_Black,
	// 	135,
	// 	225,
	// 	15,
	// 	960,
	// 	540,
	// 	300,
	// 	300);
    //   UI_Arc_Draw(&UI_energy_line[3],
	// 	"sc3",
	// 	UI_Graph_Change,
	// 	9,
	// 	UI_Color_Black,
	// 	105,
	// 	165,
	// 	15,
	// 	960,
	// 	540,
	// 	400,
	// 	400);
    //   UI_Arc_Draw(&UI_energy_line[4],
	// 	"sc4",
	// 	UI_Graph_Change,
	// 	9,
	// 	UI_Color_Black,
	// 	195,
	// 	255,
	// 	15,
	// 	960,
	// 	540,
	// 	400,
	// 	400);
    //   UI_Graph_Refresh(&referee_recv_info->referee_id,
	// 	     5,
	// 	     UI_energy_line[0],
	// 	     UI_energy_line[1],
	// 	     UI_energy_line[2],
	// 	     UI_energy_line[3],
	// 	     UI_energy_line[4]);
  }

  if ((refresh_cnt & 4) == 0)
  {
    if (1)//gimbal_message_chat.fire_advice == 1)
    {;
    }
    else
    {;
	}
  }

  if ((refresh_cnt % 5) == 0)
  {
    UI_Line_Draw(&UI_measure_digital[4],
	       "sw4",
	       UI_Graph_Change,
	       6,
	       UI_Color_Main,
	       10,
	       1360,
	       305,
	       (1360 ),//+ (int8_t) (-65 * sin(gimbal_cmd.absolute_yaw))),
	       (305 ));//+ (int8_t) (65 * cos(-gimbal_cmd.absolute_yaw))));

    UI_Rectangle_Draw(&UI_measure_digital[5],
		    "sw5",
		    UI_Graph_Change,
		    6,
		    UI_Color_Green,
		    10,
		    1320,
		    255,
		    1400,
		    355);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_measure_digital[4],
		   UI_measure_digital[5]);

    UI_Line_Draw(&UI_measure_digital[2],
	       "sw2",
	       UI_Graph_Change,
	       6,
	       UI_Color_Green,
	       20,
	       1540,
	       710,
	       1540 ,//+ user_abs(chassis_cmd.vx * 1000) + user_abs(chassis_cmd.vy * 1000),
	       710);

    UI_Line_Draw(&UI_measure_digital[3],
	       "sw3",
	       UI_Graph_Change,
	       6,
	       UI_Color_Green,
	       20,
	       1540,
	       630,
	       1540 ,//+ user_abs(chassis_cmd.omega_z_now * 60),
	       630);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_measure_digital[2],
		   UI_measure_digital[3]);

    UI_Float_Draw(&UI_Energy[12],
		"sg2",
		UI_Graph_Change,
		6,
		UI_Color_White,
		10,
		2,
		3,
		1480,
		620,
		0);//chassis_cmd.omega_z_now * 10000);

    UI_Float_Draw(&UI_Energy[10],
		"sg0",
		UI_Graph_Change,
		6,
		UI_Color_White,
		10,
		2,
		3,
		1480,
		700,
		0);//(user_abs(chassis_cmd.vx) + user_abs(chassis_cmd.vy)) * 10000);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_Energy[10],
		   UI_Energy[12]);

    UI_Line_Draw(&UI_measure_digital[6],
	       "sw6",
	       UI_Graph_Change,
	       6,
	       UI_Color_Main,
	       20,
	       1540,
	       550,
	       (1540 + referee_outer_info->PowerHeatData.shooter_42mm_barrel_heat),
	       550);

    UI_Line_Draw(&UI_measure_digital[7],
	       "sw7",
	       UI_Graph_Change,
	       6,
	       UI_Color_Main,
	       20,
	       1540,
	       470,
	       (1540 + referee_outer_info->ShootData.bullet_speed),
	       470);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_measure_digital[6],
		   UI_measure_digital[7]);

    UI_Line_Draw(&UI_Energy[13],
	       "sg3",
	       UI_Graph_Change,
	       7,
	       UI_Color_Green,
	       20,
	       1440,
	       150 ,//+ (-gimbal_message_chat.gimbal_pitch + 0.23f) * 300,
	       1460,
	       150 );//+ (-gimbal_message_chat.gimbal_pitch + 0.23f) * 300);
    UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[13]);

    UI_Float_Draw(&UI_measure_digital[8],
		"sw8",
		UI_Graph_Change,
		6,
		UI_Color_Main,
		10,
		2,
		3,
		1460,
		530,
		referee_outer_info->PowerHeatData.shooter_42mm_barrel_heat * 1000);

    UI_Float_Draw(&UI_measure_digital[9],
		"sw9",
		UI_Graph_Change,
		6,
		UI_Color_Main,
		10,
		2,
		3,
		1460,
		450,
		referee_outer_info->ShootData.bullet_speed * 1000);
    UI_Graph_Refresh(&referee_recv_info->referee_id,
		   2,
		   UI_measure_digital[8],
		   UI_measure_digital[9]);
  }

  if ((refresh_cnt % 6) == 0)
  {

  }

  error_last_code = error_code;
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
  if (_Interactive_data->control_mode != _Interactive_data->control_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.control_flag = 1;
    _Interactive_data->control_last_mode = _Interactive_data->control_mode;
  }

  if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
  }

  if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
  }

  if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
  }

  if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
  }

  if (_Interactive_data->ammo_mode != _Interactive_data->ammo_last_mode)
  {
    _Interactive_data->Referee_Interactive_Flag.ammo_flag = 1;
    _Interactive_data->ammo_last_mode = _Interactive_data->ammo_mode;
  }
  if (_Interactive_data->Chassis_Power_Limit != _Interactive_data->Chassis_last_Power_Limit)
  {
    _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    _Interactive_data->Chassis_last_Power_Limit = _Interactive_data->Chassis_Power_Limit;
  }
//  if (_Interactive_data->Super_Power != _Interactive_data->Super_last_Power)
//  {
//    if (user_abs(_Interactive_data->Super_Power - _Interactive_data->Super_last_Power) < 1)
//    {
//      _Interactive_data->Referee_Interactive_Flag.Super_flag = 1;
//    }
//    _Interactive_data->Super_last_Power = _Interactive_data->Super_Power;
//  }
}
#endif

// static void UI_Shoot_Center(void)
// {
//   UI_Circle_Draw(&UI_shoot_center[0],
// 	       "sz0",
// 	       UI_Graph_ADD,
// 	       9,
// 	       UI_Color_Main,
// 	       5,
// 	       960,
// 	       540,
// 	       25);
//   UI_Circle_Draw(&UI_shoot_center[1],
// 	       "sz1",
// 	       UI_Graph_ADD,
// 	       9,
// 	       UI_Color_Yellow,
// 	       5,
// 	       960,
// 	       540,
// 	       50);
//   UI_Arc_Draw(&UI_shoot_center[2],
// 	    "sz2",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_Green,
// 	    315,
// 	    45,
// 	    5,
// 	    960,
// 	    540,
// 	    75,
// 	    75);
//   UI_Arc_Draw(&UI_shoot_center[3],
// 	    "sz3",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_Orange,
// 	    310,
// 	    50,
// 	    5,
// 	    960,
// 	    540,
// 	    100,
// 	    100);
//   UI_Arc_Draw(&UI_shoot_center[4],
// 	    "sz4",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_Purplish_red,
// 	    90,
// 	    270,
// 	    5,
// 	    960,
// 	    540,
// 	    125,
// 	    125);
//   UI_Arc_Draw(&UI_shoot_center[5],
// 	    "sz5",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_Pink,
// 	    295,
// 	    75,
// 	    5,
// 	    960,
// 	    540,
// 	    150,
// 	    150);
//   UI_Arc_Draw(&UI_shoot_center[6],
// 	    "sz6",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_Cyan,
// 	    135,
// 	    225,
// 	    5,
// 	    960,
// 	    540,
// 	    175,
// 	    175);
//   UI_Graph_Refresh(&referee_recv_info->referee_id,
// 		 7,
// 		 UI_shoot_center[0],
// 		 UI_shoot_center[1],
// 		 UI_shoot_center[2],
// 		 UI_shoot_center[3],
// 		 UI_shoot_center[4],
// 		 UI_shoot_center[5],
// 		 UI_shoot_center[6]);

//   UI_Arc_Draw(&UI_shoot_crosshair[0],
// 	    "sc0",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_White,
// 	    15,
// 	    75,
// 	    15,
// 	    960,
// 	    540,
// 	    300,
// 	    300);
//   UI_Arc_Draw(&UI_shoot_crosshair[1],
// 	    "sc1",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_White,
// 	    285,
// 	    345,
// 	    15,
// 	    960,
// 	    540,
// 	    300,
// 	    300);
//   UI_Arc_Draw(&UI_shoot_crosshair[2],
// 	    "sc2",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_White,
// 	    135,
// 	    225,
// 	    15,
// 	    960,
// 	    540,
// 	    300,
// 	    300);
//   UI_Arc_Draw(&UI_shoot_crosshair[3],
// 	    "sc3",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_White,
// 	    105,
// 	    165,
// 	    15,
// 	    960,
// 	    540,
// 	    400,
// 	    400);
//   UI_Arc_Draw(&UI_shoot_crosshair[4],
// 	    "sc4",
// 	    UI_Graph_ADD,
// 	    9,
// 	    UI_Color_White,
// 	    195,
// 	    255,
// 	    15,
// 	    960,
// 	    540,
// 	    400,
// 	    400);
//   UI_Graph_Refresh(&referee_recv_info->referee_id,
// 		 5,
// 		 UI_shoot_crosshair[0],
// 		 UI_shoot_crosshair[1],
// 		 UI_shoot_crosshair[2],
// 		 UI_shoot_crosshair[3],
// 		 UI_shoot_crosshair[4]);
// }

// static void UI_Chassis_Limit(void)
// {
//   // 底盘功率显示，静态
//   UI_Char_Draw(&UI_State_Sta_String[5],
// 	     "ss5",
// 	     UI_Graph_ADD,
// 	     7,
// 	     UI_Color_Green,
// 	     18,
// 	     2,
// 	     620,
// 	     230,
// 	     "BUFF :");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_State_Sta_String[5]);
//   // 能量条框
//   UI_Rectangle_Draw(&UI_Energy[0],
// 		  "sf0",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  720,
// 		  140,
// 		  1220,
// 		  180);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

//   // 底盘功率显示,动态
//   UI_Float_Draw(&UI_Energy[1],
// 	      "sf1",
// 	      UI_Graph_ADD,
// 	      7,
// 	      UI_Color_Green,
// 	      18,
// 	      2,
// 	      2,
// 	      750,
// 	      230,
// 	      referee_outer_info->PowerHeatData.buffer_energy * 1000);
//   // 能量条初始状态
//   UI_Line_Draw(&UI_Energy[2],
// 	     "sf2",
// 	     UI_Graph_ADD,
// 	     7,
// 	     UI_Color_Pink,
// 	     30,
// 	     720,
// 	     160,
// 	     1020,
// 	     160);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

//   UI_Rectangle_Draw(&UI_Energy[3],
// 		  "sf3",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  860,
// 		  1920,
// 		  880);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[3]);

//   UI_Rectangle_Draw(&UI_Energy[4],
// 		  "sf4",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  780,
// 		  1920,
// 		  800);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[4]);

//   UI_Rectangle_Draw(&UI_Energy[5],
// 		  "sf5",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  700,
// 		  1920,
// 		  720);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[5]);

//   UI_Rectangle_Draw(&UI_Energy[6],
// 		  "sf6",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  620,
// 		  1920,
// 		  640);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[6]);

//   UI_Rectangle_Draw(&UI_Energy[7],
// 		  "sf7",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  540,
// 		  1920,
// 		  560);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[7]);

//   UI_Rectangle_Draw(&UI_Energy[8],
// 		  "sf8",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Green,
// 		  2,
// 		  1540,
// 		  460,
// 		  1920,
// 		  480);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[8]);

//   UI_Rectangle_Draw(&UI_Energy[9],
// 		  "sf9",
// 		  UI_Graph_ADD,
// 		  7,
// 		  UI_Color_Cyan,
// 		  2,
// 		  1440,
// 		  150,
// 		  1460,
// 		  400);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[9]);

//   UI_Line_Draw(&UI_Energy[13],
// 	     "sg3",
// 	     UI_Graph_ADD,
// 	     7,
// 	     UI_Color_Green,
// 	     20,
// 	     1440,
// 	     150,
// 	     1460,
// 	     150);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[13]);

//   UI_Float_Draw(&UI_Energy[10],
// 	      "sg0",
// 	      UI_Graph_ADD,
// 	      6,
// 	      UI_Color_White,
// 	      10,
// 	      2,
// 	      3,
// 	      1480,
// 	      700,
// 	      0);//(user_abs(chassis_cmd.vx) + user_abs(chassis_cmd.vy)) * 10000);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_Energy[10]);
// }

// static void UI_Logo_Sign(void)
// {
//   UI_Char_Draw(&UI_sign_logo[1],
// 	     "sq1",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     880,
// 	     "SuperPower");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[1]);

//   UI_Char_Draw(&UI_sign_logo[2],
// 	     "sq2",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     800,
// 	     "ammo");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[2]);

//   UI_Char_Draw(&UI_sign_logo[3],
// 	     "sq3",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     720,
// 	     "CHA_SPEED");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[3]);

//   UI_Char_Draw(&UI_sign_logo[4],
// 	     "sq4",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     640,
// 	     "CHA_SPIN");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[4]);

//   UI_Char_Draw(&UI_sign_logo[5],
// 	     "sq5",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     550,
// 	     "SHOOT_HEAT");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[5]);

//   UI_Char_Draw(&UI_sign_logo[6],
// 	     "sq6",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_White,
// 	     10,
// 	     2,
// 	     1440,
// 	     470,
// 	     "SHOOT_SPEED");
//   UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[6]);

//   UI_Float_Draw(&UI_measure_digital[0],
// 	      "sw0",
// 	      UI_Graph_ADD,
// 	      6,
// 	      UI_Color_White,
// 	      10,
// 	      2,
// 	      3,
// 	      1480,
// 	      860,
// 	      0);//super_cap->receive_data.errorCode * 1000);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_measure_digital[0]);

//   UI_Line_Draw(&UI_measure_digital[1],
// 	     "sw1",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Green,
// 	     20,
// 	     1540,
// 	     870,
// 	     1540 + 0,//super_cap->receive_data.capEnergy,
// 	     870);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_measure_digital[1]);

//   UI_Line_Draw(&UI_measure_digital[2],
// 	     "sw2",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Green,
// 	     20,
// 	     1540,
// 	     710,
// 	     1540 ,//+ chassis_cmd.vx * 50 + chassis_cmd.vy * 50,
// 	     710);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_measure_digital[2]);

//   UI_Line_Draw(&UI_measure_digital[3],
// 	     "sw3",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Green,
// 	     20,
// 	     1540,
// 	     630,
// 	     1540 ,//+ chassis_cmd.omega_z_now * 60,
// 	     630);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_measure_digital[3]);

//   UI_Line_Draw(&UI_measure_digital[4],
// 	     "sw4",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Main,
// 	     10,
// 	     1360,
// 	     305,
// 	     (1360 + (int8_t) (65 * sin(0.45))),
// 	     (305 + (int8_t) (65 * cos(0.45))));

//   UI_Rectangle_Draw(&UI_measure_digital[5],
// 		  "sw5",
// 		  UI_Graph_ADD,
// 		  6,
// 		  UI_Color_Green,
// 		  10,
// 		  1320,
// 		  255,
// 		  1400,
// 		  355);
//   UI_Graph_Refresh(&referee_recv_info->referee_id,
// 		 2,
// 		 UI_measure_digital[4],
// 		 UI_measure_digital[5]);

//   UI_Line_Draw(&UI_measure_digital[6],
// 	     "sw6",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Main,
// 	     20,
// 	     1540,
// 	     550,
// 	     (1540 + referee_outer_info->PowerHeatData.shooter_42mm_barrel_heat),
// 	     550);

//   UI_Line_Draw(&UI_measure_digital[7],
// 	     "sw7",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Main,
// 	     20,
// 	     1540,
// 	     470,
// 	     (1540 + referee_outer_info->ShootData.bullet_speed),
// 	     470);
//   UI_Graph_Refresh(&referee_recv_info->referee_id,
// 		 2,
// 		 UI_measure_digital[6],
// 		 UI_measure_digital[7]);

//   UI_Float_Draw(&UI_measure_digital[8],
// 	      "sw8",
// 	      UI_Graph_ADD,
// 	      6,
// 	      UI_Color_Main,
// 	      10,
// 	      2,
// 	      3,
// 	      1480,
// 	      540,
// 	      referee_outer_info->PowerHeatData.shooter_42mm_barrel_heat * 1000);

//   UI_Float_Draw(&UI_measure_digital[9],
// 	      "sw9",
// 	      UI_Graph_ADD,
// 	      6,
// 	      UI_Color_Main,
// 	      10,
// 	      2,
// 	      3,
// 	      1480,
// 	      460,
// 	      referee_outer_info->ShootData.bullet_speed * 1000);
//   UI_Graph_Refresh(&referee_recv_info->referee_id,
// 		 2,
// 		 UI_measure_digital[8],
// 		 UI_measure_digital[9]);

//   UI_Line_Draw(&UI_measure_digital[10],
// 	     "se0",
// 	     UI_Graph_ADD,
// 	     6,
// 	     UI_Color_Main,
// 	     20,
// 	     1540,
// 	     790,
// 	     1540,
// 	     790);
//   UI_Graph_Refresh(&referee_recv_info->referee_id, 1, UI_measure_digital[10]);
// }