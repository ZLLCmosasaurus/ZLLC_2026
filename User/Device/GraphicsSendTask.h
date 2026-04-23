//#ifndef __GRAPHICS_SEND_TASK_H
//#define __GRAPHICS_SEND_TASK_H

//#include "stdint.h"
//#include "main.h"
//#include "string.h"
//#include "dvc_referee.h"

//#define SEND_MAX_SIZE    128    //上传数据最大的长度
//#define frameheader_len  5       //帧头长度
//#define cmd_len          2       //命令码长度
//#define crc_len          2       //CRC16校验
///*屏幕宽度*/
//#define SCREEN_WIDTH 1080
//#define SCREEN_LENGTH 1920			//屏幕分辨率
///*操作类型*/
//#define Op_None 0
//#define Op_Add 1
//#define Op_Change 2
//#define Op_Delete 3
//#define Op_Init		1		//初始化，也就是增加图层
///*颜色*/
//#define Red_Blue 0
//#define Yellow   1
//#define Green    2
//#define Orange   3
//#define Purple	 4
//#define Pink     5
//#define Cyan		 6
//#define Black    7
//#define White    8


////功率限制
//enum POWERSTATE_Typedef
//{
//	BAT = 0,
//	CAP,
//	HalfCAP
//};

//enum CHARGESTATE_Typedef
//{
//	ChargeOn = 0,
//	ChargeOff
//};

//typedef struct{
//	char SuperPowerLimit;	  //0为超级电容关闭，不为0则开启使用超级电容
//	char Chassis_Flag;			//模式见上
//	char Mag_Flag;					//0表示弹仓盖关闭，1为打开
//	char Laser_Flag;				//0表示激光关闭，1为打开
//	short Gimbal_100;				//pitch角度,乘了100之后发
//	char Gimbal_Flag;				//模式见上
//	char Graphic_Init_Flag;	//0为进入初始化模式，1为初始化结束
//	char Freq_state;			  //射频状态，0表示正常射频，1表示高射频
//	/*打包数据*/
//	char Send_Pack1;	
//	char Fric_Flag;
//}F405_typedef;

//typedef struct
//{
//  char HeatUpdate_NoUpdate;
//	char SpeedUpdate_NoUpdate;
//	
//	//0x0201
//	uint8_t robot_id;
//	uint8_t RobotLevel;
//	uint16_t remainHP;
//	uint16_t maxHP;
//	uint16_t HeatCool17;		//17mm枪口每秒冷却值
//	uint16_t HeatMax17;			//17mm枪口热量上限
//	uint16_t BulletSpeedMax17;	//17mm枪口上限速度
//	uint16_t MaxPower;			//底盘功率限制上限

//	//0x0202
//	uint16_t realChassisOutV;
//	uint16_t realChassisOutA;
//	float realChassispower;
//	uint16_t remainEnergy;       //剩余能量
//	short shooterHeat17;
//	
//	//0x0207
//	uint8_t bulletFreq;		//射击频率
//	uint8_t ShootCpltFlag; //已射出一发子弹标志位
//	
//	//flag
//	short HeatUpdateFlag;	
//	
//	//not in use
//	uint8_t cardType;
//	uint8_t CardIdx;

//	float bulletSpeed;		//当前射速
//	float LastbulletSpeed;
//}
//JudgeReceive_t;

//typedef struct 
//{
//  short carSpeedx;
//	short carSpeedy;
//	short carSpeedw;
//	
//	short Last_carSpeedx;
//	short Last_carSpeedy;
//	short Last_carSpeedw;
//	
//	short ABSLastcarSpeedx;
//	short ABSLastcarSpeedy;
//	short ABSLastcarSpeedw;
//} ChassisSpeed_t;

////图形数据
//typedef __packed struct
//{
//	uint8_t graphic_name[3];
//	uint32_t operate_tpye:3;
//	uint32_t graphic_tpye:3;
//	uint32_t layer:4;
//	uint32_t color:4;
//	uint32_t start_angle:9;
//	uint32_t end_angle:9;
//	uint32_t width:10;
//	uint32_t start_x:11;
//	uint32_t start_y:11;
//	uint32_t radius:10;
//	uint32_t end_x:11;
//	uint32_t end_y:11;
//}graphic_data_struct_t;	

///*裁判系统发送信息库*详情见裁判系统串口协议*/
//typedef __packed struct
//{
//	uint8_t data[113];
//}robot_interactive_data_t;//交互数据

//typedef __packed struct
//{
//	uint8_t operate_tpye;		//0空操作  1删除单个图层  2删除所有图层
//	uint8_t layer;					//图层号  0~9
//}ext_client_custom_graphic_delete_t;//客户端删除图形

//typedef __packed struct
//{
//	graphic_data_struct_t grapic_data_struct;
//}ext_client_custom_graphic_single_t;//客户端绘制一个图形

//typedef __packed struct
//{
//	graphic_data_struct_t grapic_data_struct[2];
//}ext_client_custom_graphic_double_t;//客户端绘制两个图形

//typedef __packed struct
//{
//	graphic_data_struct_t grapic_data_struct[5];
//}ext_client_custom_graphic_five_t;//客户端绘制五个图形

//typedef __packed struct
//{
//	graphic_data_struct_t grapic_data_struct[7];
//}ext_client_custom_graphic_seven_t;//客户端绘制七个图形

//typedef __packed struct
//{
//	graphic_data_struct_t grapic_data_struct;
//	char data[30];
//}ext_client_custom_character_t;//客户端绘制字符

////交互数据信息
//typedef __packed struct         //帧头帧尾9B
//{
//	uint16_t data_cmd_id;	//数据段内容ID  :2B
//	uint16_t sender_ID;	//发送者ID        :2B
//	uint16_t receiver_ID;	//接受者ID      :2B
//	ext_client_custom_graphic_seven_t graphic_custom;//自定义图形数据: 客户端绘制七个图形  ：105B
//}ext_student_interactive_header_data_t;	

//typedef __packed struct
//{
//	uint16_t data_cmd_id;	//数据段内容ID                      :2B
//	uint16_t sender_ID;	//发送者ID														:2B
//	uint16_t receiver_ID;	//接受者ID													:2B
//	ext_client_custom_character_t char_custom;//自定义字符串数据   :45B
//}ext_student_interactive_char_header_data_t;

//extern JudgeReceive_t JudgeReceive;

//void JudgementDataSend(void);
//void JudgementCustomizeGraphics(int Op_type);
//void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
//void referee_data_load_Graphic(int Op_type);

//void referee_data_load_shootUI(uint8_t operate_type,uint8_t robot_level);
//void referee_data_load_NumberUI(void);

//void GraphicSendtask();

//#endif
