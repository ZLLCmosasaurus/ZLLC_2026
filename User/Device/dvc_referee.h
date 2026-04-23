/**
 * @file dvc_referee.h
 * @author lez by yssickjgd
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "limits.h"
#include "string.h"
#include "config.h"
/* Exported macros -----------------------------------------------------------*/

class Class_Chariot;

//设定当前车车

#define HERO_1            1
#define ENGINEER_2        2
#define INFANTRY_3        3
#define INFANTRY_4        4
#define INFANTRY_5        5
#define AERIAL_6          6
#define SENTRY_7          7
#define DART_8            8
#define RADAR_9           9
#define BASE_10           10
#define OUTPOST_11        11

#define RED 0
#define BLUE 100

//只需修改这里 UI发送者ID
#define ROBOT_ID    (INFANTRY_3 + BLUE)

const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 
	0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 
	0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 
	0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 
	0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 
	0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 
	0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 
	0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11,
	0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1,
	0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e,
	0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28,
	0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5,
	0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88,
	0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

const uint16_t CRC16_INIT = 0xffff;
const uint16_t CRC16_Table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,	
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* Exported types ------------------------------------------------------------*/

/**
 * @brief UI图层
 *
 */
enum Enum_Referee_UI_Layer_Number
{
    Referee_UI_Layer_Zero = 0,
    Referee_UI_Layer_One,
    Referee_UI_Layer_Two,
    Referee_UI_Layer_Three,
    Referee_UI_Layer_Four,
    Referee_UI_Layer_Five,
    Referee_UI_Layer_Six,
    Referee_UI_Layer_Seven,
    Referee_UI_Layer_Eight,
    Referee_UI_Layer_Nine,
};

/**
 * @brief UI颜色
 *
 */
enum Enum_Referee_UI_Color
{
    Referee_UI_Color_Red_Blue = 0,
    Referee_UI_Color_Yellow,
    Referee_UI_Color_Green,
    Referee_UI_Color_Orange,
    Referee_UI_Color_Purple, //紫色
    Referee_UI_Color_Pink,  //粉色
    Referee_UI_Color_Cyan,  //青色
    Referee_UI_Color_Black,
    Referee_UI_Color_White,
};

/**
 * @brief 同一UI组 不同字符的序号
 *
 */
enum Enum_Referee_UI_Group_Index
{
    Referee_UI_Zero = 0,
    Referee_UI_One,
    Referee_UI_Two,
    Referee_UI_Three,
    Referee_UI_Four,
    Referee_UI_Five,
    Referee_UI_Six,
};

/**
 * @brief UI组序号
 *
 */
enum Enum_Referee_UI_Group
{
    Referee_UI_Group_Zero = 0,
    Referee_UI_Group_One,
    Referee_UI_Group_Two,
    Referee_UI_Group_Three,
    Referee_UI_Group_Four,
    Referee_UI_Group_Five,
    Referee_UI_Group_Six,
};

/**
 * @brief UI绘制操作
 *
 */
enum Enum_Referee_UI_Operate_Type
{
    Referee_UI_NULL = 0,
    Referee_UI_ADD,
    Referee_UI_CHANGE,
    Referee_UI_DELETE,
};

/**
 * @brief 裁判系统状态
 *
 */
enum Enum_Referee_Status
{
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_Referee_Data_Status : uint8_t
{
    Referee_Data_Status_DISABLE = 0,
    Referee_Data_Status_ENABLE,
};

/**
 * @brief 裁判系统命令码类型
 *
 */
enum Enum_Referee_Command_ID : uint16_t
{
    Referee_Command_ID_GAME_STATUS = 0x0001,
    Referee_Command_ID_GAME_RESULT,
    Referee_Command_ID_GAME_ROBOT_HP,
    Referee_Command_ID_GAME_DART_STATUS_DISABLED,
    Referee_Command_ID_GAME_AI_DISABLED,
    Referee_Command_ID_EVENT_DATA = 0x0101,
    Referee_Command_ID_EVENT_SUPPLY,
    Referee_Command_ID_EVENT_SUPPLY_REQUEST_DISABLED,
    Referee_Command_ID_EVENT_REFEREE_WARNING,
    Referee_Command_ID_EVENT_DART_REMAINING_TIME,
    Referee_Command_ID_ROBOT_STATUS = 0x0201,
    Referee_Command_ID_ROBOT_POWER_HEAT,
    Referee_Command_ID_ROBOT_POSITION,
    Referee_Command_ID_ROBOT_BUFF,
    Referee_Command_ID_ROBOT_AERIAL_ENERGY,
    Referee_Command_ID_ROBOT_DAMAGE,
    Referee_Command_ID_ROBOT_BOOSTER,
    Referee_Command_ID_ROBOT_REMAINING_AMMO,
    Referee_Command_ID_ROBOT_RFID,
    Referee_Command_ID_ROBOT_DART_COMMAND,
    Referee_Command_ID_INTERACTION = 0x0301,
    Referee_Command_ID_INTERACTION_CUSTOM_CONTROLLER,
    Referee_Command_ID_INTERACTION_RADAR_SEND,
    Referee_Command_ID_INTERACTION_REMOTE_CONTROL,
    Referee_Command_ID_INTERACTION_Client_RECEIVE,
};

/**
 * @brief 通用单方机器人ID
 *
 */
enum Enum_Referee_Data_Robot_ID : uint8_t
{
    Referee_Data_Robot_ID_NULL = 0,
    Referee_Data_Robot_ID_HERO_1,
    Referee_Data_Robot_ID_ENGINEER_2,
    Referee_Data_Robot_ID_INFANTRY_3,
    Referee_Data_Robot_ID_INFANTRY_4,
    Referee_Data_Robot_ID_INFANTRY_5,
    Referee_Data_Robot_ID_AERIAL_6,
    Referee_Data_Robot_ID_SENTRY_7,
    Referee_Data_Robot_ID_DART_8,
    Referee_Data_Robot_ID_RADAR_9,
    Referee_Data_Robot_ID_BASE_10,
    Referee_Data_Robot_ID_OUTPOST_11,
};

/**
 * @brief 通用双方机器人ID
 *
 */
enum Enum_Referee_Data_Robots_ID : uint8_t
{
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1,
    Referee_Data_Robots_ID_RED_ENGINEER_2,
    Referee_Data_Robots_ID_RED_INFANTRY_3,
    Referee_Data_Robots_ID_RED_INFANTRY_4,
    Referee_Data_Robots_ID_RED_INFANTRY_5,
    Referee_Data_Robots_ID_RED_AERIAL_6,
    Referee_Data_Robots_ID_RED_SENTRY_7,
    Referee_Data_Robots_ID_RED_DART_8,
    Referee_Data_Robots_ID_RED_RADAR_9,
    Referee_Data_Robots_ID_RED_BASE_10,
    Referee_Data_Robots_ID_RED_OUTPOST_11,
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
    Referee_Data_Robots_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_ID_BLUE_SENTRY_7,
    Referee_Data_Robots_ID_BLUE_DART_8,
    Referee_Data_Robots_ID_BLUE_RADAR_9,
    Referee_Data_Robots_ID_BLUE_BASE_10,
    Referee_Data_Robots_ID_BLUE_OUTPOST_11,
};
/**
 * @brief 通用双方机器人ID
 *
 */
enum Enum_Referee_Data_Robots_Client_ID : uint16_t
{
    Referee_Data_Robots_Client_ID_NO = 0,
    Referee_Data_Robots_Client_ID_RED_HERO_1 = 0x0101,
    Referee_Data_Robots_Client_ID_RED_ENGINEER_2,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_3,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_4,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_5,
    Referee_Data_Robots_Client_ID_RED_AERIAL_6,
    Referee_Data_Robots_Client_ID_BLUE_HERO_1 = 0x0165,
    Referee_Data_Robots_Client_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_Client_ID_BLUE_AERIAL_6,
};

/**
 * @brief 比赛类型
 *
 */
enum Enum_Referee_Game_Status_Type
{
    Referee_Game_Status_Type_RMUC = 1,
    Referee_Game_Status_Type_SINGLE,
    Referee_Game_Status_Type_ICRA,
    Referee_Game_Status_Type_RMUL_3V3,
    Referee_Game_Status_Type_RMUL_1V1,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_Referee_Game_Status_Stage
{
    Referee_Game_Status_Stage_NOT_STARTED = 0,
    Referee_Game_Status_Stage_READY,
    Referee_Game_Status_Stage_SELF_TESTING,
    Referee_Game_Status_Stage_5S_COUNTDOWN,
    Referee_Game_Status_Stage_BATTLE,
    Referee_Game_Status_Stage_SETTLEMENT,
};

/**
 * @brief 比赛结果
 *
 */
enum Enum_Referee_Game_Result : uint8_t
{
    Referee_Game_Result_DRAW = 0,
    Referee_Game_Result_RED_WIN,
    Referee_Game_Result_BLUE_WIN,
};

/**
 * @brief 补给站状态
 *
 */
enum Enum_Referee_Data_Event_Supply_Status : uint8_t
{
    Referee_Data_Event_Supply_Status_CLOSED = 0,
    Referee_Data_Event_Supply_Status_READY,
    Referee_Data_Event_Supply_Status_DROPPING,
};

/**
 * @brief 补给站提供子弹数量
 *
 */
enum Enum_Referee_Data_Event_Supply_Ammo_Number : uint8_t
{
    Referee_Data_Event_Supply_Ammo_Number_50 = 50,
    Referee_Data_Event_Supply_Ammo_Number_100 = 100,
    Referee_Data_Event_Supply_Ammo_Number_150 = 150,
    Referee_Data_Event_Supply_Ammo_Number_200 = 200,
};

/**
 * @brief 裁判警告等级
 *
 */
enum Enum_Referee_Data_Event_Referee_Warning_Level : uint8_t
{
    Referee_Data_Referee_Warning_Level_Both_YELLOW= 1,//双方黄牌
    Referee_Data_Referee_Warning_Level_YELLOW,
    Referee_Data_Referee_Warning_Level_RED,
    Referee_Data_Referee_Warning_Level_FAIL,
};

/**
 * @brief 伤害类型
 *
 */
enum Enum_Referee_Data_Event_Robot_Damage_Type
{
    Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED = 0,
    Referee_Data_Robot_Damage_Type_MODULE_OFFLINE,
    Referee_Data_Robot_Damage_Type_BOOSTER_SPEED,
    Referee_Data_Robot_Damage_Type_BOOSTER_HEAT,
    Referee_Data_Robot_Damage_Type_CHASSIS_POWER,
    Referee_Data_Robot_Damage_Type_ARMOR_COLLISION,
};

/**
 * @brief 子弹类型
 *
 */
enum Enum_Referee_Data_Robot_Ammo_Type : uint8_t
{
    Referee_Data_Robot_Ammo_Type_BOOSTER_17MM = 1,
    Referee_Data_Robot_Ammo_Type_BOOSTER_42mm,
};

/**
 * @brief 发射机构类型
 *
 */
enum Enum_Referee_Data_Robot_Booster_Type : uint8_t
{
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1 = 1,
    Referee_Data_Robot_Booster_Type_BOOSTER_17MM_2,
    Referee_Data_Robot_Booster_Type_BOOSTER_42mm,
};

/**
 * @brief 飞镖发射口状态
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Status : uint8_t
{
    Referee_Data_Robot_Dart_Command_Status_OPEN = 0,
    Referee_Data_Robot_Dart_Command_Status_CLOSED,
    Referee_Data_Robot_Dart_Command_Status_EXECUTING,
};

/**
 * @brief 飞镖击打目标
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Target : uint8_t
{
    Referee_Data_Robot_Dart_Command_Target_OUTPOST = 0,
    Referee_Data_Robot_Dart_Command_Target_BASE,
    Referee_Data_Robot_Dart_Command_Target_Randam_BASE
};

/**
 * @brief 图形操作交互信息
 *
 */
enum Enum_Referee_Data_Interaction_Layer_Delete_Operation : uint8_t
{
    Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR,
};

/**
 * @brief 图形操作
 *
 */
enum Enum_Graphic_Operation
{
    Graphic_Operation_NULL = 0,
    Graphic_Operation_ADD,
    Graphic_Operation_CHANGE,
    Graphic_Operation_DELETE,
};

/**
 * @brief 图形类型
 *
 */
enum Enum_Graphic_Type
{
    Graphic_Type_LINE = 0,
    Graphic_Type_RECTANGLE,
    Graphic_Type_CIRCLE,
    Graphic_Type_OVAL,
    Graphic_Type_ARC,
    Graphic_Type_FLOAT,
    Graphic_Type_INTEGER,
    Graphic_Type_STRING,
};

/**
 * @brief 图形颜色
 *
 */
enum Enum_Graphic_Color
{
    Graphic_Color_MAIN = 0,
    Graphic_Color_YELLOW,
    Graphic_Color_GREEN,
    Graphic_Color_ORANGE,
    Graphic_Color_PURPLE,
    Graphic_Color_PINK,
    Graphic_Color_CYAN,
    Graphic_Color_BLACK,
    Graphic_Color_WHITE,
};

/**
 * @brief 直线图形结构体
 *
 */
struct Struct_Graphic_Line
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t End_X : 11;
    uint32_t End_Y : 11;
} __attribute__((packed));

/**
 * @brief 矩形图形结构体
 *
 */
struct Struct_Graphic_Rectangle
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t End_X : 11;
    uint32_t End_Y : 11;
} __attribute__((packed));

/**
 * @brief 圆形图形结构体
 *
 */
struct Struct_Graphic_Circle
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Radius : 10;
    uint32_t Reserved_3 : 11;
    uint32_t Reserved_4 : 11;
} __attribute__((packed));

/**
 * @brief 椭圆图形结构体
 *
 */
struct Struct_Graphic_Oval
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Reserved_1 : 9;
    uint32_t Reserved_2 : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Reserved_3 : 10;
    uint32_t Half_Length_X : 11;
    uint32_t Half_Length_Y : 11;
} __attribute__((packed));

/**
 * @brief 圆弧图形结构体
 * Angle单位是角度制而非弧度制
 *
 */
struct Struct_Graphic_Arc
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Start_Angle : 9;
    uint32_t End_Angle : 9;
    uint32_t Line_Width : 10;
    uint32_t Center_X : 11;
    uint32_t Center_Y : 11;
    uint32_t Reserved : 10;
    uint32_t Half_Length_X : 11;
    uint32_t Half_Length_Y : 11;
} __attribute__((packed));

/**
 * @brief 浮点数图形结构体
 * Decimal_Number 小数点后位数, 最多3位
 * Float *1000后的32位有符号整数
 *
 */
struct Struct_Graphic_Float
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Reserved : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    int32_t Float;
} __attribute__((packed));

/**
 * @brief 整型数图形结构体
 *
 */
struct Struct_Graphic_Integer
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Reserved : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    int32_t Integer;
} __attribute__((packed));

/**
 * @brief 字符串图形结构体
 *
 */
struct Struct_Graphic_String
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Serial : 4;
    uint32_t Color_Enum : 4;
    uint32_t Font_Size : 9;
    uint32_t Length : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Reserved;
} __attribute__((packed));

/**
 * @brief 各种图形的联合体
 *
 */
union Union_Graphic
{
    Struct_Graphic_Line Line;
    Struct_Graphic_Rectangle Rectangle;
    Struct_Graphic_Circle Circle;
    Struct_Graphic_Oval Oval;
    Struct_Graphic_Arc Arc;
    Struct_Graphic_Float Float;
    Struct_Graphic_Integer Integer;
    Struct_Graphic_String String;
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_Referee_UART_Data
{
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    Enum_Referee_Command_ID Referee_Command_ID;
    uint8_t Data[121];
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0001比赛状态, 3Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Status
{
    uint8_t Type_Enum : 4;
    uint8_t Stage_Enum : 4;
    uint16_t Remaining_Time;
    uint64_t Timestamp;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0002比赛结果, 比赛结束后发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Result
{
    Enum_Referee_Game_Result Result;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0003机器人血量, 1Hz
 *
 */
struct Struct_Referee_Rx_Data_Game_Robot_HP
{
    uint16_t Red_Hero_1;
    uint16_t Red_Engineer_2;
    uint16_t Red_Infantry_3;
    uint16_t Red_Infantry_4;
    uint16_t Red_Infantry_5;
    uint16_t Red_Sentry_7;
    uint16_t Red_Outpost_11;
    uint16_t Red_Base_10;
    uint16_t Blue_Hero_1;
    uint16_t Blue_Engineer_2;
    uint16_t Blue_Infantry_3;
    uint16_t Blue_Infantry_4;
    uint16_t Blue_Infantry_5;
    uint16_t Blue_Sentry_7;
    uint16_t Blue_Outpost_11;
    uint16_t Blue_Base_10;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0101场地事件, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Data
{
    uint32_t Supply_1_Status_Enum : 1;
    uint32_t Supply_2_Status_Enum : 1;
    uint32_t Supply_3_Status_Enum : 1;
    uint32_t Energy_Status_Enum : 1;
    uint32_t Energy_Small_Status_Enum : 1;
    uint32_t Energy_Big_Status_Enum : 1;
    uint32_t Highland_2_Status_Enum : 2;
    uint32_t Highland_3_Status_Enum : 2;
    uint32_t Highland_4_Status_Enum : 2;
    uint32_t Base_Shield_Remain_HP : 7; //己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
    uint32_t Dart_Outpost_Base_Time : 9; //飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0)
    uint32_t Dart_Outpost_Base_Obeject :2; //飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
    uint32_t Centre_Enable_Status : 2; //中心增益是否占领 0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0102补给站状态, 补给请求后对应发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Supply
{
    uint8_t Reserved;
    Enum_Referee_Data_Robots_ID Robot;
    Enum_Referee_Data_Event_Supply_Status Status;
    Enum_Referee_Data_Event_Supply_Ammo_Number Ammo_Number;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0104裁判警告信息, 判罚发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Referee_Warning
{
    Enum_Referee_Data_Event_Referee_Warning_Level Level;
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Count;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0105飞镖15s倒计时, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Dart_Remaining_Time
{
    uint8_t Dart_Remaining_Time;
    uint16_t Dart_Info : 5;
    uint16_t Dart_Target_Enum : 2;
    uint16_t Reserved : 9;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0201机器人状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Status
{
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Level;
    uint16_t HP;
    uint16_t HP_Max;
    uint16_t Shooter_Barrel_Cooling_Value;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Chassis_Power_Limit; 
    uint8_t PM01_Gimbal_Status_Enum : 1;
    uint8_t PM01_Chassis_Status_Enum : 1;
    uint8_t PM01_Booster_Status_Enum : 1;
    uint8_t Reserved : 5;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0202当前机器人实时功率热量, 50Hz发送
 * 电压mV, 电流mA
 *
 */
struct Struct_Referee_Rx_Data_Robot_Power_Heat
{
    uint16_t Chassis_Voltage;
    uint16_t Chassis_Current;
    float Chassis_Power;
    uint16_t Chassis_Energy_Buffer;
    uint16_t Booster_17mm_1_Heat;
    uint16_t Booster_17mm_2_Heat;
    uint16_t Booster_42mm_Heat;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0203当前机器人实时位置, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Position
{
    float Location_X;
    float Location_Y;
    float Location_Yaw;  //本机器人测速模块的朝向，单位：度。正北为 0 度
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0204当前机器人增益, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Buff
{
    uint8_t HP_Recovery_Buff_Rate ;
    uint8_t Booster_Cooling_Buff_Rate ;
    uint8_t Defend_Buff_Rate ; //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t Vulnerability_Buff_Rate; //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint8_t Damage_Buff_Rate ; //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0205无人机可攻击时间, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Aerial_Energy
{
    uint8_t Airforce_status;  //空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
    uint8_t Remaining_Time;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0206伤害情况, 伤害发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Damage
{
    uint8_t Armor_ID : 4;
    uint8_t Type_Enum : 4;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0207子弹信息, 射击发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Booster
{
    Enum_Referee_Data_Robot_Ammo_Type Ammo_Type;
    Enum_Referee_Data_Robot_Booster_Type Booster_Type;
    uint8_t Frequency;
    float Speed;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0208子弹剩余信息, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo
{
    uint16_t Booster_Allowance_17mm;
    uint16_t Booster_Allowance_42mm;
    uint16_t Money;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0209RFID状态信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_RFID
{
    uint32_t Self_Base_Status_Enum : 1;
    uint32_t Self_Highland_Status_Enum : 1;
    uint32_t Opposite_Highland_Status_Enum : 1;
    uint32_t Self_R3_B3_Status_Enum : 1;
    uint32_t Opposite_R3_B3_Status_Enum : 1;
    uint32_t Self_R4_B4_Status_Enum : 1;
    uint32_t Opposite_R4_B4_Status_Enum : 1;
    uint32_t Self_Energy_Status_Enum : 1;
    uint32_t Self_Flyover_Status_Enum_Before : 1;
    uint32_t Self_Flyover_Status_Enum_After : 1;
    uint32_t Opposite_Flyover_Status_Enum_Before : 1;
    uint32_t Opposite_Flyover_Status_Enum_After : 1;
    uint32_t Self_Outpost_Status_Enum : 1;
    uint32_t Self_HP_Status_Enum : 1;
    uint32_t Self_Sentry_Patrol_Area_Enum : 1;
    uint32_t Opposite_Sentry_Patrol_Area_Enum : 1;
    uint32_t Self_Engineer_Status_Enum : 1;
    uint32_t Opposite_Engineer_Status_Enum : 1;
    uint32_t Self_Exchange_Area_Status_Enum : 1;
    uint32_t Centre_Status_Enum : 1;
    uint32_t Reserved : 12;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020a飞镖状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Dart_Command
{
    Enum_Referee_Data_Robot_Dart_Command_Status Status;
    uint8_t Reserved;
    uint16_t Change_Target_Timestamp;
    uint16_t Last_Confirm_Timestamp;
    uint16_t CRC_16;
} __attribute__((packed));

/**
 * @brief 裁判系统发送或接收的数据, 0x0301机器人间交互信息, 用户自主发送
 * TODO 视情况启用
 * Header 0x0200~0x02ff
 * Data 最大112
 *
 */
struct Struct_Referee_Data_Interaction_Students
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved_1;
    Enum_Referee_Data_Robots_ID Receiver;
    uint8_t Reserved_2;
    uint8_t Data[112];
    uint16_t CRC_16;
}__attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301图形删除交互信息, 用户自主发送
 * Header 0x0100
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Layer_Delete
{
    uint16_t Header = 0x0100;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;   
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Enum_Referee_Data_Interaction_Layer_Delete_Operation Operation;
    uint8_t Delete_Serial;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画一个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_1
{
    uint16_t Header = 0x0101;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画两个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_2
{
    uint16_t Header = 0x0102;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    Union_Graphic Graphic_2;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画五个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_5
{
    uint16_t Header = 0x0103;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_1;
    Union_Graphic Graphic_2;
    Union_Graphic Graphic_3;
    Union_Graphic Graphic_4;
    Union_Graphic Graphic_5;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画七个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_7
{
    uint16_t Header = 0x0104;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic[6];
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0301画字符图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_String
{
    uint16_t Header = 0x0110;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Union_Graphic Graphic_String;
    uint8_t String[30];
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0302自定义控制器交互信息, 用户自主发送
 * TODO 视情况赋予Data含义
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Custom_Controller
{
    uint8_t Data[30];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0303雷达发送小地图交互信息, 用户自主最高30Hz发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Radar_Send
{
    float Coordinate_X;
    float Coordinate_Y;
    float Coordinate_Z;
    uint8_t Keyboard;
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0304图传遥控交互信息, 30Hz发送
 * TODO 等待扩展
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Remote_Control
{
    uint16_t Mouse_X;
    uint16_t Mouse_Y;
    uint16_t Mouse_Z;
    Enum_Referee_Data_Status Mouse_Left_Key_Status;
    Enum_Referee_Data_Status Mouse_Right_Key_Status;
    uint16_t Keyboard_Key_W : 1;
    uint16_t Keyboard_Key_S : 1;
    uint16_t Keyboard_Key_A : 1;
    uint16_t Keyboard_Key_D : 1;
    uint16_t Keyboard_Key_Shift : 1;
    uint16_t Keyboard_Key_Ctrl : 1;
    uint16_t Keyboard_Key_Q : 1;
    uint16_t Keyboard_Key_E : 1;
    uint16_t Keyboard_Key_R : 1;
    uint16_t Keyboard_Key_F : 1;
    uint16_t Keyboard_Key_G : 1;
    uint16_t Keyboard_Key_Z : 1;
    uint16_t Keyboard_Key_X : 1;    
    uint16_t Keyboard_Key_C : 1;
    uint16_t Keyboard_Key_V : 1;
    uint16_t Keyboard_Key_B : 1;
    uint16_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0305客户端接收小地图交互信息, 用户自主最高30Hz发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Client_Receive
{
    Enum_Referee_Data_Robots_ID Robot_ID;
    uint8_t Reserved_1;
    float Coordinate_X;
    float Coordinate_Y;
    uint8_t Reserved_2;
} __attribute__((packed));

/**
 * @brief Specialized, 裁判系统
 *
 */
class Class_Referee
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xa5);

    inline uint16_t Get_Circle_Index(uint16_t index){return (index%UART_Manage_Object->Rx_Buffer_Length);}
    inline Enum_Referee_Status Get_Referee_Status();
    inline Enum_Referee_Game_Status_Type Get_Game_Type();
    inline Enum_Referee_Game_Status_Stage Get_Game_Stage();
    inline uint16_t Get_Remaining_Time();
    inline uint64_t Get_Timestamp();
    inline Enum_Referee_Game_Result Get_Result();
    inline uint16_t Get_HP(Enum_Referee_Data_Robots_ID Robots_ID);
    inline Enum_Referee_Data_Status Get_Event_Supply_Status(uint8_t Supply_ID);
    inline Enum_Referee_Data_Status Get_Event_Energy_Status();
    inline Enum_Referee_Data_Status Get_Event_Energy_Small_Activate_Status();
    inline Enum_Referee_Data_Status Get_Event_Energy_Big_Activate_Status();
    inline Enum_Referee_Data_Status Get_Event_Highland_Status(uint8_t Highland_ID);
    inline uint32_t Get_Event_Base_Shield_Remain_HP();
    inline uint8_t Get_Supply_ID();
    inline Enum_Referee_Data_Robots_ID Get_Supply_Request_Robot();
    inline Enum_Referee_Data_Event_Supply_Status Get_Supply_Request_Status();
    inline Enum_Referee_Data_Event_Supply_Ammo_Number Get_Supply_Ammo_Number();
    inline Enum_Referee_Data_Event_Referee_Warning_Level Get_Referee_Warning();
    inline Enum_Referee_Data_Robots_ID Get_Referee_Warning_Robot();
    inline uint8_t Get_Dart_Remaining_Time();
    inline Enum_Referee_Data_Robots_ID Get_ID();
    inline uint8_t Get_Level();
    inline uint16_t Get_HP();
    inline uint16_t Get_HP_Max();
    inline uint16_t Get_Booster_17mm_1_Heat_CD();
    inline uint16_t Get_Booster_17mm_1_Heat_Max();
    inline uint16_t Get_Booster_17mm_2_Heat_CD();
    inline uint16_t Get_Booster_17mm_2_Heat_Max();
    inline uint16_t Get_Booster_42mm_Heat_CD();
    inline uint16_t Get_Booster_42mm_Heat_Max();
    inline uint16_t Get_Chassis_Power_Max();
    inline Enum_Referee_Data_Status Get_PM01_Gimbal_Status();
    inline Enum_Referee_Data_Status Get_PM01_Chassis_Status();
    inline Enum_Referee_Data_Status Get_PM01_Booster_Status();
    inline float Get_Chassis_Voltage();
    inline float Get_Chassis_Current();
    inline float Get_Chassis_Power();
    inline uint16_t Get_Chassis_Energy_Buffer();
    inline uint16_t Get_Booster_17mm_1_Heat();
    inline uint16_t Get_Booster_17mm_2_Heat();
    inline uint16_t Get_Booster_42mm_Heat();
    inline float Get_Location_X();
    inline float Get_Location_Y();
    inline float Get_Location_Yaw();
    inline uint8_t Get_HP_Recovery_Buff_Rate();
    inline uint8_t Get_Booster_Cooling_Buff_Rate();
    inline uint8_t Get_Defend_Buff_Rate();
    inline uint8_t Get_Damage_Buff_Rate();
    inline uint8_t Get_Aerial_Remaining_Time();
    inline uint8_t Get_Armor_Attacked_ID();
    inline Enum_Referee_Data_Event_Robot_Damage_Type Get_Attacked_Type();
    inline Enum_Referee_Data_Robot_Ammo_Type Get_Shoot_Ammo_Type();
    inline Enum_Referee_Data_Robot_Booster_Type Get_Shoot_Booster_Type();
    inline uint8_t Get_Shoot_Frequency();
    inline float Get_Shoot_Speed();
    inline uint16_t Get_17mm_Remaining();
    inline uint16_t Get_42mm_Remaining();
    inline uint16_t Get_Money_Remaining();
    inline Enum_Referee_Data_Status Get_Self_Base_RFID_Status();
    inline Enum_Referee_Data_Status Get_Self_Highland_RFID_Status();
    inline Enum_Referee_Data_Status Get_Self_Energy_RFID_Status();
    inline Enum_Referee_Data_Status Get_Self_Flyover_RFID_Status_Before();
    inline Enum_Referee_Data_Status Get_Self_Flyover_RFID_Status_After();
    inline Enum_Referee_Data_Status Get_Self_HP_RFID_Status();
    inline Enum_Referee_Data_Status Get_Self_Engineer_RFID_Status();
    inline Enum_Referee_Data_Status Get_Self_Outpost_RFID_Status();
    inline Enum_Referee_Data_Robot_Dart_Command_Status Get_Dart_Command_Status();
    inline Enum_Referee_Data_Robot_Dart_Command_Target Get_Dart_Command_Target();
    inline uint16_t Get_Dart_Change_Target_Timestamp();
    inline uint16_t Get_Dart_Last_Confirm_Timestamp();
    inline Enum_Referee_Data_Robots_ID Get_Radar_Send_Robot_ID();
    inline float Get_Radar_Send_Coordinate_X();
    inline float Get_Radar_Send_Coordinate_Y();


    //裁判系统状态
    Enum_Referee_Status Referee_Status = Referee_Status_DISABLE;
		
    #ifdef GIMBAL
    inline void Set_Robot_ID(Enum_Referee_Data_Robots_ID __Robot_ID);
    inline void Set_Game_Stage(Enum_Referee_Game_Status_Stage __Game_Stage);
    inline void Set_Booster_17mm_1_Heat(uint16_t __Booster_17mm_1_Heat);
    inline void Set_Booster_17mm_1_Heat_CD(uint16_t __Booster_17mm_1_Heat_CD);
    inline void Set_Booster_17mm_1_Heat_Max(uint16_t __Booster_17mm_1_Heat_Max);
    inline void Set_Booster_Speed(float __booster_speed);
		
    #endif

    void Referee_UI_Draw_Line(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y,  uint32_t __End_X, uint32_t __End_Y,Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_Rectangle(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y,  uint32_t __End_X, uint32_t __End_Y,Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_Oval(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Line_Width, uint32_t __Center_X, uint32_t __Center_Y, uint32_t __X_Length, uint32_t __Y_Length, Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_Circle(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Line_Width, uint32_t __Center_X, uint32_t __Center_Y, uint32_t __Radius, Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_Float(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, float __Number, Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_Integer(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, int32_t __Number, Enum_Referee_UI_Operate_Type __Operate_Type);
    void Referee_UI_Draw_String(uint8_t __Robot_ID, Enum_Referee_UI_Group_Index __Group_Index, uint32_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, char *__String ,uint32_t __String_Length, Enum_Referee_UI_Operate_Type __Operate_Type);

    void Referee_UI_Packed_String();

    template <typename T>
    void Referee_UI_Packed_Data(T* __data);

    void UART_Tx_Referee_UI();

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Size);
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    //数据包头标
    uint8_t Frame_Header;

    //常量
    uint8_t seq = 0;  //包序号
    const uint8_t frameheader_len = 5; // 帧头长度
    const uint8_t cmd_len = 2;         // 命令码长度
    const uint8_t crc_len = 2;         // CRC16校验
    // 内部变量

    //当前时刻的裁判系统接收flag
    uint32_t Flag = 0;
    //前一时刻的裁判系统接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //比赛状态
    Struct_Referee_Rx_Data_Game_Status Game_Status;
    //比赛结果
    Struct_Referee_Rx_Data_Game_Result Game_Result;
    //机器人血量
    Struct_Referee_Rx_Data_Game_Robot_HP Game_Robot_HP;
    //场地事件
    Struct_Referee_Rx_Data_Event_Data Event_Data;
    //补给站状态
    Struct_Referee_Rx_Data_Event_Supply Event_Supply;
    //裁判警告信息
    Struct_Referee_Rx_Data_Event_Referee_Warning Event_Referee_Warning;
    //飞镖15s倒计时
    Struct_Referee_Rx_Data_Event_Dart_Remaining_Time Event_Dart_Remaining_Time;
    //机器人状态
    Struct_Referee_Rx_Data_Robot_Status Robot_Status;
    //当前机器人实时功率热量
    Struct_Referee_Rx_Data_Robot_Power_Heat Robot_Power_Heat;
    //当前机器人实时位置
    Struct_Referee_Rx_Data_Robot_Position Robot_Position;
    //当前机器人增益
    Struct_Referee_Rx_Data_Robot_Buff Robot_Buff;
    //无人机可攻击时间
    Struct_Referee_Rx_Data_Robot_Aerial_Energy Robot_Aerial_Energy;
    //伤害情况
    Struct_Referee_Rx_Data_Robot_Damage Robot_Damage;
    //子弹信息
    Struct_Referee_Rx_Data_Robot_Booster Robot_Booster;
    //子弹剩余信息
    Struct_Referee_Rx_Data_Robot_Remaining_Ammo Robot_Remaining_Ammo;
    // RFID状态信息
    Struct_Referee_Rx_Data_Robot_RFID Robot_RFID;
    //飞镖状态
    Struct_Referee_Rx_Data_Robot_Dart_Command Robot_Dart_Command;
    //客户端接收小地图交互信息
    Struct_Referee_Tx_Data_Interaction_Client_Receive Interaction_Client_Receive;

    Struct_Referee_Data_Interaction_Students Interaction_Students;
    // 写变量

    // 图形删除交互信息
    Struct_Referee_Tx_Data_Interaction_Layer_Delete Interaction_Layer_Delete;
    //画一个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_1 Interaction_Graphic_1;
    //画两个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_2 Interaction_Graphic_2;
    //画五个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_5 Interaction_Graphic_5;
    //画七个图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_7 Interaction_Graphic_7;
    //画字符图形交互信息
    Struct_Referee_Tx_Data_Interaction_Graphic_String Interaction_Graphic_String;
    //雷达发送小地图交互信息
    Struct_Referee_Tx_Data_Interaction_Radar_Send Interaction_Radar_Send;

    //读写变量

    //内部函数 

    void Data_Process();
    
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

unsigned char Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


/**
 * @brief 裁判系统数据打包
 *
 */
template <typename T>
void Class_Referee::Referee_UI_Packed_Data(T* __data)
{
    uint16_t frame_length,data_len,cmd_id;
    
    cmd_id = 0x0301;    //子内容ID
    data_len = sizeof(T);      //字符操作数据长度
	frame_length = frameheader_len + cmd_len + data_len + crc_len;   //数据帧长度	

	memset(UART_Manage_Object->Tx_Buffer,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	UART_Manage_Object->Tx_Buffer[0] = Frame_Header;//数据帧起始字节
	memcpy(&UART_Manage_Object->Tx_Buffer[1],(uint8_t*)&data_len, 2);//数据帧中data的长度
	UART_Manage_Object->Tx_Buffer[3] = seq;//包序号
	Append_CRC8_Check_Sum(UART_Manage_Object->Tx_Buffer,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&UART_Manage_Object->Tx_Buffer[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&UART_Manage_Object->Tx_Buffer[frameheader_len+cmd_len], __data, sizeof(T));
	Append_CRC16_Check_Sum(UART_Manage_Object->Tx_Buffer,frame_length);  //一帧数据校验CRC16

    UART_Manage_Object->Tx_Length = frame_length;

    seq++;
}

/**
 * @brief 获取裁判系统状态
 *
 * @return Enum_Referee_Status 裁判系统状态
 */
Enum_Referee_Status Class_Referee::Get_Referee_Status()
{
    return (Referee_Status);
}

/**
 * @brief 获取比赛类型
 *
 * @return Enum_Referee_Game_Status_Type 比赛类型
 */
Enum_Referee_Game_Status_Type Class_Referee::Get_Game_Type()
{
    return (static_cast<Enum_Referee_Game_Status_Type>(Game_Status.Type_Enum));
}

/**
 * @brief 获取比赛阶段
 *
 * @return Enum_Referee_Game_Status_Stage 比赛阶段
 */
Enum_Referee_Game_Status_Stage Class_Referee::Get_Game_Stage()
{
    return (static_cast<Enum_Referee_Game_Status_Stage>(Game_Status.Stage_Enum));
}

/**
 * @brief 获取当前阶段剩余时间
 *
 * @return uint16_t 当前阶段剩余时间
 */
uint16_t Class_Referee::Get_Remaining_Time()
{
    return (Game_Status.Remaining_Time);
}

/**
 * @brief 获取系统时间戳
 *
 * @return uint64_t 系统时间戳
 */
uint64_t Class_Referee::Get_Timestamp()
{
    return (Game_Status.Timestamp);
}

/**
 * @brief 获取比赛结果
 *
 * @return Enum_Referee_Game_Result 比赛结果
 */
Enum_Referee_Game_Result Class_Referee::Get_Result()
{
    return (Game_Result.Result);
}

/**
 * @brief 获取机器人血量
 *
 * @param Robots_ID 通用双方机器人ID
 * @return uint16_t 机器人血量
 */
uint16_t Class_Referee::Get_HP(Enum_Referee_Data_Robots_ID Robots_ID)
{
    switch (Robots_ID)
    {
    case (Referee_Data_Robots_ID_RED_HERO_1):
        return (Game_Robot_HP.Red_Hero_1);
    case (Referee_Data_Robots_ID_RED_ENGINEER_2):
        return (Game_Robot_HP.Red_Engineer_2);
    case (Referee_Data_Robots_ID_RED_INFANTRY_3):
        return (Game_Robot_HP.Red_Infantry_3);
    case (Referee_Data_Robots_ID_RED_INFANTRY_4):
        return (Game_Robot_HP.Red_Infantry_4);
    case (Referee_Data_Robots_ID_RED_INFANTRY_5):
        return (Game_Robot_HP.Red_Infantry_5);
    case (Referee_Data_Robots_ID_RED_SENTRY_7):
        return (Game_Robot_HP.Red_Sentry_7);
    case (Referee_Data_Robots_ID_RED_OUTPOST_11):
        return (Game_Robot_HP.Red_Outpost_11);
    case (Referee_Data_Robots_ID_RED_BASE_10):
        return (Game_Robot_HP.Red_Base_10);
    case (Referee_Data_Robots_ID_BLUE_HERO_1):
        return (Game_Robot_HP.Blue_Hero_1);
    case (Referee_Data_Robots_ID_BLUE_ENGINEER_2):
        return (Game_Robot_HP.Blue_Engineer_2);
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_3):
        return (Game_Robot_HP.Blue_Infantry_3);
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_4):
        return (Game_Robot_HP.Blue_Infantry_4);
    case (Referee_Data_Robots_ID_BLUE_INFANTRY_5):
        return (Game_Robot_HP.Blue_Infantry_5);
    case (Referee_Data_Robots_ID_BLUE_SENTRY_7):
        return (Game_Robot_HP.Blue_Sentry_7);
    case (Referee_Data_Robots_ID_BLUE_OUTPOST_11):
        return (Game_Robot_HP.Blue_Outpost_11);
    case (Referee_Data_Robots_ID_BLUE_BASE_10):
        return (Game_Robot_HP.Blue_Base_10);
    default:
        return (Game_Robot_HP.Blue_Base_10); // 或者返回一个其他合适的默认值
    }
}

/**
 * @brief 获取补给站占领状态
 *
 * @param Supply_ID 补给站ID, 1~3
 * @return Enum_Referee_Data_Status 补给站占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Supply_Status(uint8_t Supply_ID)
{
    switch (Supply_ID)
    {
    case (1):
        return static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_1_Status_Enum);
    case (2):
        return static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_2_Status_Enum);
    case (3):
        return static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_3_Status_Enum);
    default:
        return static_cast<Enum_Referee_Data_Status>(Event_Data.Supply_1_Status_Enum); // 
    }
}

/**
 * @brief 获取能量机关占领状态
 *
 * @return Enum_Referee_Data_Status 能量机关占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Status_Enum));
}

/**
 * @brief 获取小能量机关激活状态
 *
 * @return Enum_Referee_Data_Status 小能量机关激活状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Small_Activate_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Small_Status_Enum));
}

/**
 * @brief 获取大能量机关激活状态
 *
 * @return Enum_Referee_Data_Status 大能量机关激活状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Energy_Big_Activate_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Energy_Big_Status_Enum));
}

/**
 * @brief 获取高地占领状态
 *
 * @param Highland_ID 高地ID, 2~4
 * @return Enum_Referee_Data_Status 高地占领状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Event_Highland_Status(uint8_t Highland_ID)
{
    switch (Highland_ID)
    {
    case (2):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_2_Status_Enum));
    }
    break;
    case (3):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_3_Status_Enum));
    }
    break;
    case (4):
    {
        return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_4_Status_Enum));
    }
    break;
    default :
	    return (static_cast<Enum_Referee_Data_Status>(Event_Data.Highland_4_Status_Enum));
    }
}

/**
 * @brief 获取基地护盾状态
 *
 * @return Enum_Referee_Data_Status 基地护盾状态
 */
uint32_t Class_Referee::Get_Event_Base_Shield_Remain_HP()
{
    return ((uint32_t)Event_Data.Base_Shield_Remain_HP);
}


/**
 * @brief 获取补给点ID
 *
 * @return uint8_t 补给点ID
 */
uint8_t Class_Referee::Get_Supply_ID()
{
    return (Event_Supply.Robot);
}

/**
 * @brief 获取请求补给的机器人ID
 *
 * @return Enum_Referee_Data_Robots_ID 请求补给的机器人ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_Supply_Request_Robot()
{
    return (Event_Supply.Robot);
}

/**
 * @brief 获取补给站的补给状态
 *
 * @return Enum_Referee_Data_Event_Supply_Status 补给站的补给状态
 */
Enum_Referee_Data_Event_Supply_Status Class_Referee::Get_Supply_Request_Status()
{
    return (Event_Supply.Status);
}

/**
 * @brief 获取补给子弹数量
 *
 * @return Enum_Referee_Data_Event_Supply_Ammo_Number 补给子弹数量
 */
Enum_Referee_Data_Event_Supply_Ammo_Number Class_Referee::Get_Supply_Ammo_Number()
{
    return (Event_Supply.Ammo_Number);
}

/**
 * @brief 获取裁判判罚信息
 *
 * @return Enum_Referee_Data_Event_Referee_Warning_Level 裁判判罚信息
 */
Enum_Referee_Data_Event_Referee_Warning_Level Class_Referee::Get_Referee_Warning()
{
    return (Event_Referee_Warning.Level);
}

/**
 * @brief 获取裁判判罚机器人
 *
 * @return Enum_Referee_Data_Robot_ID 裁判判罚机器人
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_Referee_Warning_Robot()
{
    return (Event_Referee_Warning.Robot_ID);
}

/**
 * @brief 获取飞镖剩余时间
 *
 * @return uint8_t 飞镖剩余时间
 */
uint8_t Class_Referee::Get_Dart_Remaining_Time()
{
    return (Event_Dart_Remaining_Time.Dart_Remaining_Time);
}

/**
 * @brief 获取自身ID
 *
 * @return Enum_Referee_Data_Robots_ID 自身ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_ID()
{
    return (Robot_Status.Robot_ID);
}

/**
 * @brief 获取自身等级
 *
 * @return uint8_t 自身等级
 */
uint8_t Class_Referee::Get_Level()
{
    return (Robot_Status.Level);
}

/**
 * @brief 获取自身血量
 *
 * @return uint16_t 自身血量
 */
uint16_t Class_Referee::Get_HP()
{
    return (Robot_Status.HP);
}

/**
 * @brief 获取自身血量上限
 *
 * @return uint16_t 自身血量上限
 */
uint16_t Class_Referee::Get_HP_Max()
{
    return (Robot_Status.HP_Max);
}

/**
 * @brief 获取17mm1枪口冷却速度
 *
 * @return uint16_t 17mm1枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat_CD()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_1_Heat_CD == 0)
    {
        return (40);
    }
#endif
    return (Robot_Status.Shooter_Barrel_Cooling_Value);
}

/**
 * @brief 获取17mm1枪口热量上限
 *
 * @return uint16_t 17mm1枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_1_Heat_Max == 0)
    {
        return (240);
    }
#endif
    return (Robot_Status.Shooter_Barrel_Heat_Limit);
}


/**
 * @brief 获取17mm2枪口冷却速度
 *
 * @return uint16_t 17mm2枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat_CD()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_2_Heat_CD == 0)
    {
        return (40);
    }
#endif
    return (Robot_Status.Shooter_Barrel_Cooling_Value);
}

/**
 * @brief 获取17mm2枪口热量上限
 *
 * @return uint16_t 17mm2枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Booster_17mm_2_Heat_Max == 0)
    {
        return (240);
    }
#endif
    return (Robot_Status.Shooter_Barrel_Heat_Limit);
}


/**
 * @brief 获取42mm枪口冷却速度
 *
 * @return uint16_t 42mm枪口冷却速度
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat_CD()
{
    return (Robot_Status.Shooter_Barrel_Cooling_Value);
}

/**
 * @brief 获取42mm枪口热量上限
 *
 * @return uint16_t 42mm枪口热量上限
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat_Max()
{
    return (Robot_Status.Shooter_Barrel_Heat_Limit);
}


/**
 * @brief 获取底盘功率上限
 *
 * @return uint16_t 底盘功率上限
 */
uint16_t Class_Referee::Get_Chassis_Power_Max()
{
#ifdef Robot_SENTRY_7
    if (Robot_Status.Chassis_Power_Max == 0)
    {
        //裁判系统如果寄了
        return (80);
        // //高校联盟赛
        // return (100);
        // //超级对抗赛
        // return (150);
    }
#endif
    return (Robot_Status.Chassis_Power_Limit);
}

/**
 * @brief 获取Gimbal供电状态
 *
 * @return Enum_Referee_Data_Status Gimbal供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Gimbal_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Gimbal_Status_Enum));
}

/**
 * @brief 获取Chassis供电状态
 *
 * @return Enum_Referee_Data_Status Chassis供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Chassis_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Chassis_Status_Enum));
}

/**
 * @brief 获取Booster供电状态
 *
 * @return Enum_Referee_Data_Status Booster供电状态
 */
Enum_Referee_Data_Status Class_Referee::Get_PM01_Booster_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_Status.PM01_Booster_Status_Enum));
}

/**
 * @brief 获取底盘电压
 *
 * @return float 底盘电压
 */
float Class_Referee::Get_Chassis_Voltage()
{
    return (Robot_Power_Heat.Chassis_Voltage / 1000.0f);
}

/**
 * @brief 获取底盘电流
 *
 * @return float 底盘电流
 */
float Class_Referee::Get_Chassis_Current()
{
    return (Robot_Power_Heat.Chassis_Current / 1000.0f);
}

/**
 * @brief 获取底盘功率
 *
 * @return float 底盘功率
 */
float Class_Referee::Get_Chassis_Power()
{
    return (Robot_Power_Heat.Chassis_Power);
}

/**
 * @brief 获取底盘能量缓冲
 *
 * @return uint16_t 底盘能量缓冲
 */
uint16_t Class_Referee::Get_Chassis_Energy_Buffer()
{
    return (Robot_Power_Heat.Chassis_Energy_Buffer);
}

/**
 * @brief 获取17mm1热量
 *
 * @return uint16_t 17mm1热量
 */
uint16_t Class_Referee::Get_Booster_17mm_1_Heat()
{
    return (Robot_Power_Heat.Booster_17mm_1_Heat);
}

/**
 * @brief 获取17mm2热量
 *
 * @return uint16_t 17mm2热量
 */
uint16_t Class_Referee::Get_Booster_17mm_2_Heat()
{
    return (Robot_Power_Heat.Booster_17mm_2_Heat);
}

/**
 * @brief 获取42mm热量
 *
 * @return uint16_t 42mm热量
 */
uint16_t Class_Referee::Get_Booster_42mm_Heat()
{
    return (Robot_Power_Heat.Booster_42mm_Heat);
}

/**
 * @brief 获取自身位置x
 *
 * @return float 自身位置x
 */
float Class_Referee::Get_Location_X()
{
    return (Robot_Position.Location_X);
}

/**
 * @brief 获取自身位置y
 *
 * @return float 自身位置y
 */
float Class_Referee::Get_Location_Y()
{
    return (Robot_Position.Location_Y);
}


/**
 * @brief 获取自身方向yaw
 *
 * @return float 自身方向yaw
 */
float Class_Referee::Get_Location_Yaw()
{
    return (Robot_Position.Location_Yaw);
}

/**
 * @brief 获取补血buff状态
 *
 * @return Enum_Referee_Data_Status 补血buff状态
 */
uint8_t Class_Referee::Get_HP_Recovery_Buff_Rate()
{
    return (Robot_Buff.HP_Recovery_Buff_Rate);
}

/**
 * @brief 获取冷却缩减buff状态
 *
 * @return Enum_Referee_Data_Status 冷却缩减buff状态
 */
uint8_t Class_Referee::Get_Booster_Cooling_Buff_Rate()
{
    return (Robot_Buff.Booster_Cooling_Buff_Rate);
}

/**
 * @brief 获取防御加成buff状态
 *
 * @return Enum_Referee_Data_Status 防御加成buff状态
 */
uint8_t Class_Referee::Get_Defend_Buff_Rate()
{
    return (Robot_Buff.Defend_Buff_Rate);
}

/**
 * @brief 获取攻击加成buff状态
 *
 * @return Enum_Referee_Data_Status 攻击加成buff状态
 */
uint8_t Class_Referee::Get_Damage_Buff_Rate()
{
    return (Robot_Buff.Damage_Buff_Rate);
}

/**
 * @brief 获取无人机时间
 *
 * @return uint8_t 无人机时间
 */
uint8_t Class_Referee::Get_Aerial_Remaining_Time()
{
    return (Robot_Aerial_Energy.Remaining_Time);
}

/**
 * @brief 获取受击装甲板ID
 *
 * @return uint8_t 受击装甲板ID
 */
uint8_t Class_Referee::Get_Armor_Attacked_ID()
{
    return (Robot_Damage.Armor_ID);
}

/**
 * @brief 获取受击类型
 *
 * @return Enum_Referee_Data_Event_Robot_Damage_Type 受击类型
 */
Enum_Referee_Data_Event_Robot_Damage_Type Class_Referee::Get_Attacked_Type()
{
    return (static_cast<Enum_Referee_Data_Event_Robot_Damage_Type>(Robot_Damage.Type_Enum));
}

/**
 * @brief 获取射击子弹类型
 *
 * @return Enum_Referee_Data_Robot_Ammo_Type 射击子弹类型
 */
Enum_Referee_Data_Robot_Ammo_Type Class_Referee::Get_Shoot_Ammo_Type()
{
    return (Robot_Booster.Ammo_Type);
}

/**
 * @brief 获取发射机构类型
 *
 * @return Enum_Referee_Data_Robot_Booster_Type 发射机构类型
 */
Enum_Referee_Data_Robot_Booster_Type Class_Referee::Get_Shoot_Booster_Type()
{
    return (Robot_Booster.Booster_Type);
}

/**
 * @brief 获取射频, Hz
 *
 * @return uint8_t 射频, Hz
 */
uint8_t Class_Referee::Get_Shoot_Frequency()
{
    return (Robot_Booster.Frequency);
}

/**
 * @brief 获取射速
 *
 * @return float 射速
 */
float Class_Referee::Get_Shoot_Speed()
{
    return (Robot_Booster.Speed);
}

/**
 * @brief 获取17mm弹丸剩余数
 *
 * @return uint16_t 17mm弹丸剩余数
 */
uint16_t Class_Referee::Get_17mm_Remaining()
{
    return (Robot_Remaining_Ammo.Booster_Allowance_17mm);
}

/**
 * @brief 获取42mm弹丸剩余数
 *
 * @return uint16_t 42mm弹丸剩余数
 */
uint16_t Class_Referee::Get_42mm_Remaining()
{
    return (Robot_Remaining_Ammo.Booster_Allowance_42mm);
}

/**
 * @brief 获取金币剩余数
 *
 * @return uint16_t 金币剩余数
 */
uint16_t Class_Referee::Get_Money_Remaining()
{
    return (Robot_Remaining_Ammo.Money);
}

/**
 * @brief 获取基地增益RFID状态
 *
 * @return Enum_Referee_Data_Status 基地增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Base_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Base_Status_Enum));
}

/**
 * @brief 获取高地增益RFID状态
 *
 * @return Enum_Referee_Data_Status 高地增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Highland_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Highland_Status_Enum));
}

/**
 * @brief 获取能量机关增益RFID状态
 *
 * @return Enum_Referee_Data_Status 能量机关增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Energy_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Energy_Status_Enum));
}

/**
 * @brief 获取飞坡增益RFID状态
 *
 * @return Enum_Referee_Data_Status 飞坡增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Flyover_RFID_Status_Before()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Flyover_Status_Enum_Before));
}

/**
 * @brief 获取飞坡增益RFID状态
 *
 * @return Enum_Referee_Data_Status 飞坡增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Flyover_RFID_Status_After()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Flyover_Status_Enum_After));
}

/**
 * @brief 获取前哨站增益RFID状态
 *
 * @return Enum_Referee_Data_Status 前哨站增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Outpost_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Outpost_Status_Enum));
}

/**
 * @brief 获取补血点增益RFID状态
 *
 * @return Enum_Referee_Data_Status 补血点增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_HP_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_HP_Status_Enum));
}

/**
 * @brief 获取工程复活卡增益RFID状态
 *
 * @return Enum_Referee_Data_Status 工程复活卡增益RFID状态
 */
Enum_Referee_Data_Status Class_Referee::Get_Self_Engineer_RFID_Status()
{
    return (static_cast<Enum_Referee_Data_Status>(Robot_RFID.Self_Engineer_Status_Enum));
}

/**
 * @brief 获取飞镖发射口状态
 *
 * @return Enum_Referee_Data_Robot_Dart_Command_Status 飞镖发射口状态
 */
Enum_Referee_Data_Robot_Dart_Command_Status Class_Referee::Get_Dart_Command_Status()
{
    return (Robot_Dart_Command.Status);
}

/**
 * @brief 获取飞镖击打目标
 *
 * @return Enum_Referee_Data_Robot_Dart_Command_Target 飞镖击打目标
 */
Enum_Referee_Data_Robot_Dart_Command_Target Class_Referee::Get_Dart_Command_Target()
{
    return  (static_cast<Enum_Referee_Data_Robot_Dart_Command_Target>(Event_Dart_Remaining_Time.Dart_Target_Enum));
}

/**
 * @brief 获取击打目标改变的时间戳
 *
 * @return uint16_t 击打目标改变的时间戳
 */
uint16_t Class_Referee::Get_Dart_Change_Target_Timestamp()
{
    return (Robot_Dart_Command.Change_Target_Timestamp);
}

/**
 * @brief 获取云台手下达指令的时间戳
 *
 * @return uint16_t 云台手下达指令的时间戳
 */
uint16_t Class_Referee::Get_Dart_Last_Confirm_Timestamp()
{
    return (Robot_Dart_Command.Last_Confirm_Timestamp);
}

/**
 * @brief 获取雷达发送目标机器人ID
 *
 * @return Enum_Referee_Data_Robots_ID 雷达发送目标的机器人ID
 */
Enum_Referee_Data_Robots_ID Class_Referee::Get_Radar_Send_Robot_ID()
{
    return (Interaction_Client_Receive.Robot_ID);
}

/**
 * @brief 获取雷达发送目标机器人位置x
 *
 * @return float 雷达发送目标机器人位置x
 */
float Class_Referee::Get_Radar_Send_Coordinate_X()
{
    return (Interaction_Client_Receive.Coordinate_X);
}

/**
 * @brief 获取雷达发送目标机器人位置y
 *
 * @return float 雷达发送目标机器人位置y
 */
float Class_Referee::Get_Radar_Send_Coordinate_Y()
{
    return (Interaction_Client_Receive.Coordinate_Y);
}


/**
 * @brief 设置机器人ID
 *
 * @param __Robot_ID 机器人ID
 */
#ifdef GIMBAL
void Class_Referee::Set_Robot_ID(Enum_Referee_Data_Robots_ID __Robot_ID)
{
    this->Robot_Status.Robot_ID = __Robot_ID;
}
#endif


/**
 * @brief 设置机器人等级
 *
 * @param __Level 机器人等级
 */
#ifdef GIMBAL
void Class_Referee::Set_Game_Stage(Enum_Referee_Game_Status_Stage __Game_Stage)
{
    this->Game_Status.Stage_Enum = __Game_Stage;
}
#endif



#ifdef GIMBAL
void Class_Referee::Set_Booster_17mm_1_Heat(uint16_t __Booster_17mm_1_Heat)
{
    this->Robot_Power_Heat.Booster_17mm_1_Heat = __Booster_17mm_1_Heat;
}
#endif


/**
 * @brief 设置17mm枪管热量上限
 *
 * @param __Booster_17mm_1_Heat_Max 枪管热量上限
 */
#ifdef GIMBAL
void Class_Referee::Set_Booster_17mm_1_Heat_Max(uint16_t __Booster_17mm_1_Heat_Max)
{
    this->Robot_Status.Shooter_Barrel_Heat_Limit = __Booster_17mm_1_Heat_Max;
}

void Class_Referee::Set_Booster_Speed(float __booster_speed)
{
    Robot_Booster.Speed=__booster_speed;
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
