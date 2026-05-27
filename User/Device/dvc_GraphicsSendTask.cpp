/**********************************************************************************************************
 * @文件     Graphics_Send.c
 * @说明     裁判系统图形发送
 * @版本     V2.0
 * @作者     黄志雄
 * @日期     2023.5.1
 **********************************************************************************************************/
#include "dvc_GraphicsSendTask.h"
#include <stm32f4xx.h>
#include <string.h>
#include "usart.h"
#include <stdio.h>
#include "dvc_pilotlamp.h"

#define CAP_GRAPHIC_NUM 9 // 超级电容的电量显示细分个数
#define Robot_ID 46
unsigned char JudgeSend[SEND_MAX_SIZE];
JudgeReceive_t JudgeReceiveData;
JudgeReceive_t Last_JudgeReceiveData;
// extern SuperPower superpower;
F405_typedef F405;
#define Robot_ID 46

int pitch_change_flag;
int cap_percent_change_flag;
int BigFrictSpeed_change_flag;
int Pitch_change_flag;
int vol_change_array[CAP_GRAPHIC_NUM];
float last_cap_vol;
short lastBigFrictSpeed;

/**********************************************************************************************************
 * @文件     Graphics_Send.c
 * @日期     2023.4


参考：Robomaster 裁判协议附录v1.4



裁判系统通信协议

	帧头部					命令id(绘制UI为0x0301)		数据段（头部+数据）			尾部2字节校验位 CRC16
*********************		*********************		*********************		*********************
*					*		*					*		*					*		*					*
*	frame_header	*		*	cmd_id			*		*	data			*		*	frame_tail		*
*	(5 bytes)		*	+	*	(2 bytes)		*	+	*	(n bytes)		*	+	*	(2 bytes)		*
*					*		*					*		*					*		*	  				*
*********************		*********************		*********************		*********************



**********************************************************************************************************/

/*			变量定义				*/
uint8_t Transmit_Pack[128];				   // 裁判系统发送帧
uint8_t data_pack[DRAWING_PACK * 7] = {0}; // 数据段部分
uint8_t DMAsendflag;
/**********************************************************************************************************
 *函 数 名: Send_UIPack
 *功能说明: 发送自定义UI数据包（数据段头部和数据）
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/

void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID, uint16_t receiverID, uint8_t *data, uint16_t pack_len)
{
	student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = data_cmd_id;
	custom_interactive_header.send_ID = SendID;
	custom_interactive_header.receiver_ID = receiverID;

	uint8_t header_len = sizeof(custom_interactive_header); // 数据段头部长度

	memcpy((void *)(Transmit_Pack + 7), &custom_interactive_header, header_len); // 将数据段的数据段进行封装（封装头部）
	memcpy((void *)(Transmit_Pack + 7 + header_len), data, pack_len);			 // 将整个帧的数据段进行封装（封装数据）

	Send_toReferee(0x0301, pack_len + header_len); // 发送整个数据帧数据
}

/**********************************************************************************************************
 *函 数 名: Send_toReferee
 *功能说明: 将整个帧数据发送给裁判系统
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Send_toReferee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len;

	// 帧头部封装
	{
		Transmit_Pack[0] = 0xA5;
		memcpy(&Transmit_Pack[1], (uint8_t *)&data_len, sizeof(data_len)); // 数据段即data的长度
		Transmit_Pack[3] = seq++;
		Append_CRC8_Check_Sum(Transmit_Pack, HEADER_LEN); // 帧头校验CRC8
	}

	// 命令ID
	memcpy(&Transmit_Pack[HEADER_LEN], (uint8_t *)&cmd_id, CMD_LEN);

	// 尾部添加校验CRC16
	Append_CRC16_Check_Sum(Transmit_Pack, Frame_Length);

	// 对于状态变化类消息，增加发送次数为3次，提高可靠性
	uint8_t send_cnt = (cmd_id == Drawing_Char_ID) ? 3 : 1;
	while (send_cnt)
	{
		send_cnt--;
		// 将超时时间从5ms增加到50ms，提高通信稳定性
		HAL_UART_Transmit(&huart1, (uint8_t *)Transmit_Pack, Frame_Length, 50);
		DMAsendflag = 1;

		// 添加短暂延时，避免连续发送导致丢包
		if (send_cnt > 0)
		{
			for (volatile uint16_t i = 0; i < 1000; i++)
				; // 简单延时
		}
	}
}

/**********************************************************************************************************
 *函 数 名: Deleta_Layer
 *功能说明: 清空图层
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Deleta_Layer(uint8_t layer, uint8_t deleteType)
{
	static client_custom_graphic_delete_t Delete_Graphic; // 定义为静态变量，避免函数调用时重复分配该变量内存
	Delete_Graphic.layer = layer;
	Delete_Graphic.operate_tpye = deleteType;
	Send_UIPack(Drawing_Delete_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, (uint8_t *)&Delete_Graphic, sizeof(Delete_Graphic)); // 发字符
}

/**********************************************************************************************************
 *函 数 名: CharGraphic_Draw
 *功能说明: 得到字符图形数据结构体
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
graphic_data_struct_t *CharGraphic_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[])
{

	static graphic_data_struct_t drawing;  // 定义为静态变量，避免函数调用时重复分配该变量内存
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CHAR; // 7为字符类型
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;

	drawing.start_angle = size; // 字体大小
	drawing.end_angle = len;	// 字符长度
	drawing.width = line_width;

	for (uint8_t i = DRAWING_PACK; i < DRAWING_PACK + 30; i++)
		data_pack[i] = 0;
	return &drawing;
}

/**********************************************************************************************************
 *函 数 名: Char_Draw
 *功能说明: 绘制字符
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Char_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[], uint8_t *str_data)
{
	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = CharGraphic_Draw(0, Op_Type, startx, starty, size, len, line_width, color, name);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	memset(&data_pack[DRAWING_PACK], 0, 30);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)str_data, len);
	Send_UIPack(Drawing_Char_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK + 30); // 发送字符
}

/**********************************************************************************************************
 *函 数 名: FloatData_Draw
 *功能说明: 得到绘制浮点图形结构体
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
graphic_data_struct_t *FloatData_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, float data_f, uint8_t size, uint8_t valid_bit, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing; // 定义为静态变量，避免函数调用时重复分配该变量内存
	static int32_t Data1000;
	Data1000 = (int32_t)(data_f * 1000);
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_FLOAT; // 5为浮点数据
	drawing.width = line_width;		   // 线宽
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;	   // 字体大小
	drawing.end_angle = valid_bit; // 有效位数

	drawing.radius = Data1000 & 0x03ff;
	drawing.end_x = (Data1000 >> 10) & 0x07ff;
	drawing.end_y = (Data1000 >> 21) & 0x07ff;
	return &drawing;
}

/**********************************************************************************************************
 *函 数 名: Line_Draw
 *功能说明: 直线图形数据结构体
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
graphic_data_struct_t *Line_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // 定义为静态变量，避免函数调用时重复分配该变量内存
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_LINE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.end_x = endx;
	drawing.end_y = endy;
	return &drawing;
}

/**********************************************************************************************************
 *函 数 名: Rectangle_Draw
 *功能说明: 矩形图形数据结构体
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
graphic_data_struct_t *Rectangle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // 定义为静态变量，避免函数调用时重复分配该变量内存
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_RECTANGLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.end_x = endx;
	drawing.end_y = endy;
	return &drawing;
}

/**********************************************************************************************************
 *函 数 名: Circle_Draw
 *功能说明: 圆形图形数据结构体
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
graphic_data_struct_t *Circle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint32_t radius, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // 定义为静态变量，避免函数调用时重复分配该变量内存
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CIRCLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.radius = radius;
	return &drawing;
}

/**
 * @brief 画圆弧
 *
 * @param layer 图层
 * @param Op_Type 操作类型
 * @param startx 圆心x坐标
 * @param starty 圆心y坐标
 * @param start_angle 圆弧起始角度（deg）
 * @param end_angle 圆弧终止角度（deg）
 * @param radius	圆弧半径
 * @param line_width 圆弧线宽
 * @param color 圆弧颜色
 * @param name
 * @return graphic_data_struct_t*
 */
graphic_data_struct_t *Arc_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t start_angle, uint16_t end_angle, uint32_t x_len, uint32_t y_len, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // 定义为静态变量，避免函数调用时重复分配该变量内存
	memcpy(drawing.graphic_name, name, 3); // 图形名称，3位
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_ARC;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = start_angle;
	drawing.end_angle = end_angle;
	drawing.end_x = x_len;
	drawing.end_y = y_len;
	return &drawing;
}

/**********************************************************************************************************
 *函 数 名: Lanelines_Init
 *功能说明: 车道线初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	graphic_data_struct_t *P_graphic_data;
	// 第一条车道线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.41, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.31, 0, 4, Orange, LaneLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	// 第二条车道线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.59, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.69, 0, 4, Orange, LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 发送两个图形
}

/**********************************************************************************************************
 *函 数 名: Shootlines_Init
 *功能说明: 枪口初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void ShootLines_Init(void)
{
	static uint8_t ShootLineName1[] = "SL1";
	static uint8_t ShootLineName2[] = "SL2";
	static uint8_t ShootLineName3[] = "SL3";
	static uint8_t ShootLineName4[] = "SL4";
	static uint8_t ShootLineName5[] = "SL5";
	static uint8_t ShootLineName6[] = "SL6";
	static uint8_t ShootLineName7[] = "SL7";
	graphic_data_struct_t *P_graphic_data;

	float x_bias = 0;
	float y_bias = 0;
	// 横向线条
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, SCREEN_LENGTH * 0.5 + 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, 1, Green, ShootLineName3);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, SCREEN_LENGTH * 0.5 + 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, 1, Green, ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, SCREEN_LENGTH * 0.5 + 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 纵向线条
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 40 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 10 + x_bias, SCREEN_WIDTH * 0.5 - 132 + y_bias, SCREEN_LENGTH * 0.5 + 10 + x_bias, SCREEN_WIDTH * 0.5 - 132 + y_bias, 1, Green, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5); // 发送五个图形
}

/**********************************************************************************************************
 *函 数 名: CarPosture_Change
 *功能说明: 车体姿态绘制
 *形    参: theta: 云台底盘夹角(rad)，Init_Cnt: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
uint16_t RectCenterX = SCREEN_LENGTH * 0.5; // 修改为屏幕正中心
uint16_t RectCenterY = SCREEN_WIDTH * 0.5;	// 修改为屏幕正中心
uint16_t startX, startY, endX, endY;
float angle;
float angle1;

/**
 * @brief 绘制车体位置
 *
 * @param theta 云台底盘夹角（rad）
 * @param Init_Cnt 初始化标志
 */
void CarPosture_Change(float theta, uint8_t Init_Cnt)
{
	static uint8_t CarPostureName[] = "cpt";
	static uint8_t optype;
	uint16_t start_angle;
	uint16_t end_angle;

	uint16_t angle = 15;

	// 将弧度转换为角度
	float angle_deg = theta * 180.0f / 3.14159f;

	// 计算圆弧的起始和终止角度
	// 随着夹角变化，圆弧整体旋转

	start_angle = (uint16_t)(345 + angle_deg) % 360;
	end_angle = (uint16_t)(15 + angle_deg) % 360;

	// 圆弧半径
	uint32_t radius = 100;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	uint8_t indicatorColor = (JudgeReceiveData.Minipc_Status > 0) ? Green : Orange;
	// 绘制圆弧
	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = Arc_Draw(1, optype, RectCenterX, RectCenterY, start_angle, end_angle, radius, radius, 10, indicatorColor, CarPostureName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 发送图形数据
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

void FrictSpeed_Draw(uint16_t omega_left, uint16_t omega_right, uint8_t Init_Cnt)
{
	static uint8_t FricSpeedValueName_left[] = "fsl";  // 数值的独立名称
	static uint8_t FricSpeedValueName_right[] = "fsr"; // 数值的独立名称
	static uint16_t last_omega_left = 0;			   // 添加静态变量记录上次的值
	static uint16_t last_omega_right = 0;
	uint8_t optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 处理低转速情况，当转速低于阈值时认为已停止
	if (omega_left < 200)
		omega_left = 0;
	if (omega_right < 200)
		omega_right = 0;

	// 恢复条件更新逻辑，确保数值变化时才更新显示
	if (abs((int)last_omega_left - (int)omega_left) > 10 ||
		abs((int)last_omega_right - (int)omega_right) > 10 ||
		(last_omega_left > 0 && omega_left == 0) ||
		(last_omega_left == 0 && omega_left > 0) ||
		(last_omega_right > 0 && omega_right == 0) ||
		(last_omega_right == 0 && omega_right > 0) ||
		Init_Cnt > 0)
	{
		// 将整数转换为字符串
		uint8_t speed_str_left[8];
		snprintf((char *)speed_str_left, sizeof(speed_str_left), "%d", omega_left);
		uint8_t speed_str_right[8];
		snprintf((char *)speed_str_right, sizeof(speed_str_right), "%d", omega_right);

		// 动态更新数值
		if (omega_left < 600)
		{
			Char_Draw(0, optype,
					  0.9 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, // 数值位置 (X, Y)
					  20, strlen((char *)speed_str_left), 2,
					  Green, FricSpeedValueName_left, speed_str_left);
		}
		else
		{
			Char_Draw(0, optype,
					  0.9 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, // 数值位置 (X, Y)
					  20, strlen((char *)speed_str_left), 2,
					  Orange, FricSpeedValueName_left, speed_str_left);
		}

		// if (omega_right < 600)
		// {
		// 	Char_Draw(0, optype, 0.933 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, 20, strlen((char *)speed_str_right), 2, Green, FricSpeedValueName_right, speed_str_right);
		// }
		// else
		// {
		// 	Char_Draw(0, optype, 0.933 * SCREEN_LENGTH, 0.35 * SCREEN_WIDTH, 20, strlen((char *)speed_str_right), 2, Orange, FricSpeedValueName_right, speed_str_right);
		// }

		// 更新记录的值
		last_omega_left = omega_left;
		last_omega_right = omega_right;
	}
}
void BulletNum_Draw(uint16_t bullet_num, uint8_t Init_Cnt)
{
	static uint8_t BulletNumName[] = "bun"; // 数值的独立名称
	static uint16_t last_bullet_num = 0;
	uint8_t optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 恢复条件更新逻辑，确保数值变化时才更新显示
	if (bullet_num != last_bullet_num || Init_Cnt > 0)
	{
		// 将整数转换为字符串
		uint8_t num_str[8];
		snprintf((char *)num_str, sizeof(num_str), "%d", bullet_num);

		Char_Draw(0, optype,
				  0.9 * SCREEN_LENGTH, 0.60 * SCREEN_WIDTH, // 数值位置 (X, Y)
				  20, strlen((char *)num_str), 2,
				  Green, BulletNumName, num_str);
	}

	// 更新记录的值
	last_bullet_num = bullet_num;
}
/*�������ݵ�������*/
void CapDraw(float CapVolt, uint8_t Init_Flag)
{
	static float Length;
	static uint8_t CapName1[] = "Out";
	static uint8_t CapName2[] = "In";

	graphic_data_struct_t *P_graphic_data;
	if (Init_Flag)
	{
		P_graphic_data = Rectangle_Draw(0, Op_Add, 0.3495 * SCREEN_LENGTH, 0.1125 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.1385 * SCREEN_WIDTH, 5, Cyan, CapName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, Op_Add, 0.35 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 0.65 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 27, Green, CapName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2);
	}
	else
	{
		Length = CapVolt * (0.3 * SCREEN_LENGTH);
		P_graphic_data = Line_Draw(0, Op_Change, 0.35 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 0.35 * SCREEN_LENGTH + Length, 0.125 * SCREEN_WIDTH, 27, Green, CapName2);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
	}
}

/*字符变化发送*/
void FricChange(uint8_t Init_Flag)
{

	uint8_t FOLLOW[] = "ENABLE";
	uint8_t Chassis_Off[] = "DISABLE";

	static uint8_t optype;
	/*底盘状态改变*/
	static uint8_t ChassisChangeName[] = "cha";
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	switch (JudgeReceiveData.Fric_Status)
	{
	case 0:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.70 * SCREEN_WIDTH, 20, sizeof(Chassis_Off), 2, Pink, ChassisChangeName, Chassis_Off);
		break;
	case 1:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.70 * SCREEN_WIDTH, 20, sizeof(FOLLOW), 2, Green, ChassisChangeName, FOLLOW);
		break;
	}
}

/**********************************************************************************************************
 *�� �� ��: Char_Init
 *����˵��: �ַ����ݳ�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Char_Init(void)
{
	static uint8_t FrictionName[] = "fri";
	static uint8_t AutoName[] = "aim";
	static uint8_t FricStatusName[] = "fsp";
	static uint8_t GimbalStatusLabelName[] = "gsl"; // 云台状态标签名称
	static uint8_t BoosterModeLabelName[] = "bml";	// 发射机构模式标签名称
	static uint8_t MiniPCTargetLabelName[] = "mpl"; // MiniPC模式标签名称
	static uint8_t BulletNumName[] = "bnu";			// 弹丸已发射数量
	static uint8_t AntispinType[] = "ant";
	static uint8_t PliotStatusName[] = "psn";
	/*              FIREMODE字符*/

	uint8_t fric_status_label[] = "FRIC :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.70 * SCREEN_WIDTH, 20, sizeof(fric_status_label), 2, Yellow, FricStatusName, fric_status_label);

	uint8_t orin_alive_label[] = "ORIN :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.65 * SCREEN_WIDTH, 20, sizeof(orin_alive_label), 2, Yellow, BulletNumName, orin_alive_label);
	/*              MINIPC MODE字符*/
	uint8_t minipc_target_label[] = "TARGET :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.60 * SCREEN_WIDTH, 20, sizeof(minipc_target_label), 2, Yellow, MiniPCTargetLabelName, minipc_target_label);

	/*              BOOSTER MODE字符*/
	uint8_t booster_mode_label[] = "BOOSTER:";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(booster_mode_label), 2, Yellow, BoosterModeLabelName, booster_mode_label);

	/*              GIMBAL状态标签            */
	uint8_t gimbal_status_label[] = "GIMBAL :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(gimbal_status_label), 2, Yellow, GimbalStatusLabelName, gimbal_status_label);

	uint8_t pliot_status_label[] = "PLIOT :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(pliot_status_label), 2, Yellow, PliotStatusName, pliot_status_label);
}

void MiniPC_Aim_Change(uint8_t Init_Cnt)
{
	/*自瞄获取状态*/
	static uint8_t Auto_Aim_ChangeName[] = "Aim";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.MiniPC_Aim_Status)
	{
	case 1:
		P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.25 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.75 * SCREEN_WIDTH, 5, Green, Auto_Aim_ChangeName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
	case 0:
		P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.25 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.75 * SCREEN_WIDTH, 5, Pink, Auto_Aim_ChangeName);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
	}
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: PitchUI_Change
 *功能说明: Pitch角度显示（圆弧方式）
 *形    参: Pitch角度，初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void PitchUI_Change(float Pitch, uint8_t Init_Cnt)
{
	static uint8_t PitchBackgroundName[] = "PBG"; // Pitch背景圆弧名称
	static uint8_t PitchIndicatorName[] = "PIN";  // Pitch指示器圆弧名称
	static uint8_t optype;

	// 圆弧位置和大小参数
	uint16_t centerX = 0.6 * SCREEN_LENGTH;
	uint16_t centerY = 0.5 * SCREEN_WIDTH;
	uint16_t radiusX = 200; // X轴半径
	uint16_t radiusY = 300; // Y轴半径（大于X轴半径，形成竖直方向的椭圆）

	float pitchMin = -30.0f;
	float pitchMax = 30.0f;

	uint16_t bgStartAngle = 30;
	uint16_t bgEndAngle = 150;

	// 计算当前Pitch对应的角度位置
	float pitchRatio = (Pitch - pitchMin) / (pitchMax - pitchMin);		 // 归一化到0-1范围
	pitchRatio = pitchRatio < 0 ? 0 : (pitchRatio > 1 ? 1 : pitchRatio); // 限制在0-1范围内

	// 计算指示器圆弧的角度范围（短弧，宽度为10度）
	uint16_t indicatorAngle = bgStartAngle + (uint16_t)(pitchRatio * (bgEndAngle - bgStartAngle));
	uint16_t indicatorStartAngle = indicatorAngle - 5;
	uint16_t indicatorEndAngle = indicatorAngle + 5;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	graphic_data_struct_t *P_graphic_data;

	// if (optype == Op_Add)
	// {
	// 	// 绘制背景圆弧（细线）
	// 	P_graphic_data = Arc_Draw(1, optype, centerX, centerY, bgStartAngle, bgEndAngle, radiusX, radiusY, 4, Yellow, PitchBackgroundName);
	// 	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	// 	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
	// }

	// 绘制指示器圆弧（粗线）
	uint8_t indicatorColor = (Pitch > 0) ? Green : Orange;
	P_graphic_data = Arc_Draw(1, optype, centerX, centerY, indicatorStartAngle, indicatorEndAngle, radiusX, radiusY, 12, indicatorColor, PitchIndicatorName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: RadarDoubleDamage_Draw
 *功能说明: 显示雷达双倍易伤状态
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void RadarDoubleDamage_Draw(uint8_t Init_Cnt)
{
	static uint8_t RadarDamageChangeName[] = "rdd";
	static uint8_t optype;
	static uint8_t READY2[] = "READY2";
	static uint8_t READY1[] = "READY1";
	static uint8_t EMPTY[] = "     ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Radar_Double_Damage_Flag)
	{
	case 1:
		Char_Draw(0, optype, 0.5 * SCREEN_LENGTH, 0.7 * SCREEN_WIDTH, 30, sizeof(READY1), 3, Green, RadarDamageChangeName, READY1);
		break;
	case 2:
		Char_Draw(0, optype, 0.5 * SCREEN_LENGTH, 0.7 * SCREEN_WIDTH, 30, sizeof(READY2), 3, Green, RadarDamageChangeName, READY2);
		break;
	default:
		break;
	}
}

/**********************************************************************************************************
 *函 数 名: GimbalStatus_Draw
 *功能说明: 显示云台状态
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void GimbalStatus_Draw(uint8_t Init_Cnt)
{
	static uint8_t GimbalStatusName[] = "gst";
	static uint8_t optype;
	static uint8_t DISABLE[] = "DISABLE";
	static uint8_t NORMAL[] = "NORMAL ";
	static uint8_t MINIPC[] = "MINIPC ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Gimbal_Control_Type)
	{
	case 0: // Gimbal_Control_Type_DISABLE
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(DISABLE), 2, Pink, GimbalStatusName, DISABLE);
		break;
	case 1: // Gimbal_Control_Type_NORMAL
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(NORMAL), 2, Green, GimbalStatusName, NORMAL);
		break;
	case 2: // Gimbal_Control_Type_MINIPC
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(MINIPC), 2, Orange, GimbalStatusName, MINIPC);
		break;
	default:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(DISABLE), 2, Pink, GimbalStatusName, DISABLE);
		break;
	}
}

void PliotLampChange(uint8_t Init_Cnt)
{
	static uint8_t PliotLampChangeName[] = "pcn";
	static uint8_t optype;
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	switch (JudgeReceiveData.PilotLamp_Mode)
	{
	case Enum_PilotLamp_Type_Left45Deg:
	{
		uint8_t str[] = "L45";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
	case Enum_PilotLamp_Type_Right45Deg:
	{
		uint8_t str[] = "R45";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
	case Enum_PilotLamp_Type_Position1:
	{
		uint8_t str[] = "P1";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
	case Enum_PilotLamp_Type_Position2:
	{
		uint8_t str[] = "P2";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
	case Enum_PilotLamp_Type_Position3:
	{
		uint8_t str[] = "P3";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
	case Enum_PilotLamp_Type_Position4:
	{
		uint8_t str[] = "P4";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}

	default:
		uint8_t str[] = "NONE";
		Char_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(str), 2, Green, PliotLampChangeName, str);
		break;
	}
}

/**********************************************************************************************************
 *函 数 名: BoosterMode_Draw
 *功能说明: 显示发射机构用户控制类型
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void BoosterMode_Draw(uint8_t Init_Cnt)
{
	static uint8_t BoosterModeStatusName[] = "bms";
	static uint8_t optype;
	static uint8_t SINGLE[] = "DISABLE";
	static uint8_t MULTI[] = "ENABLE ";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	if (JudgeReceiveData.Booster_status == 0)
	{
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(SINGLE), 2, Green, BoosterModeStatusName, SINGLE);
	}
	else
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(MULTI), 2, Orange, BoosterModeStatusName, MULTI);
}

/**********************************************************************************************************
 *函 数 名: MiniPCMode_Draw
 *功能说明: 显示MiniPC模式UI显示
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void MiniPCMode_Draw(uint8_t Init_Cnt)
{
	static uint8_t MiniPCModeStatusName[] = "mpm";
	static uint8_t optype;
	static uint8_t ARMOR1[] = "OUTPOST_1";
	static uint8_t ARMOR2[] = "OUTPOST_2";
	static uint8_t WINDMILL[] = "BASE";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Minipc_Mode)
	{
	case 0: // MiniPC_Mode_ARMOR
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.60 * SCREEN_WIDTH, 20, sizeof(ARMOR1), 2, Green, MiniPCModeStatusName, ARMOR1);
		break;
	case 1: // MiniPC_Mode_WINDMILL
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.60 * SCREEN_WIDTH, 20, sizeof(WINDMILL), 2, Orange, MiniPCModeStatusName, WINDMILL);
		break;
	default:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.60 * SCREEN_WIDTH, 20, sizeof(ARMOR2), 2, Green, MiniPCModeStatusName, ARMOR2);
		break;
	}
}
/**********************************************************************************************************
 *函 数 名: MiniPC_Alive_Draw
 *功能说明: 显示是否开启反小陀螺UI显示
 *形    参: 初始化标志
 *返 回 值: 无
 **********************************************************************************************************/
void MiniPC_Alive_Draw(uint8_t Init_Cnt)
{

	static uint8_t AntispinTypeName[] = "atn";
	static uint8_t optype;
	static uint8_t On[] = "ON  ";
	static uint8_t Off[] = "OFF";

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Minipc_Status)
	{
	case 0:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.65 * SCREEN_WIDTH, 20, sizeof(Off), 2, Green, AntispinTypeName, Off);
		break;
	case 1:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.65 * SCREEN_WIDTH, 20, sizeof(On), 2, Orange, AntispinTypeName, On);
		break;
	default:
		Char_Draw(0, optype, 0.9 * SCREEN_LENGTH, 0.65 * SCREEN_WIDTH, 20, sizeof(On), 2, Green, AntispinTypeName, Off);
		break;
	}
}
/**********************************************************************************************************
 *函 数 名: GraphicSendtask
 *功能说明: ͼ�η�������
 *形    参: ��
 *返 回 值: ��
 **********************************************************************************************************/
uint8_t Init_Cnt = 15;
// 添加UI更新频率控制计数器
static uint32_t ui_update_counter = 0;

// 添加状态变化标志
static uint8_t status_changed = 0;

// 添加UI更新状态枚举
typedef enum
{
	UI_STATE_IDLE = 0,		// 空闲状态
	UI_STATE_STATUS_UPDATE, // 状态更新状态
	UI_STATE_VALUE_UPDATE	// 数值更新状态
} UI_Update_State_t;

void GraphicSendtask(void)
{
	static UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
	static uint8_t status_update_retry = 0;			   // 状态更新重试次数
	static uint8_t last_status_type = 0;			   // 上次变化的状态类型
	static uint32_t last_update_time = 0;			   // 上次更新时间
	static uint32_t current_time = 0;				   // 当前时间

	// 获取当前时间（假设有HAL_GetTick函数）
	current_time = HAL_GetTick();

	// 初始化阶段发送所有UI元素
	if (Init_Cnt > 0)
	{
		Init_Cnt--;

		Char_Init();				 // 字符
		MiniPC_Aim_Change(Init_Cnt); // 自瞄框
		FricChange(Init_Cnt);
		MiniPC_Alive_Draw(Init_Cnt);
		MiniPCMode_Draw(Init_Cnt);
		BoosterMode_Draw(Init_Cnt);
		GimbalStatus_Draw(Init_Cnt); // Pilot模式
		PliotLampChange(Init_Cnt);

		// 初始化完成后，保存当前数据作为比较基准
		memcpy(&Last_JudgeReceiveData, &JudgeReceiveData, sizeof(JudgeReceive_t));

		return;
	}

	// 状态机处理
	switch (ui_state)
	{
	case UI_STATE_IDLE:
		// 检查是否有状态变化
		if (Last_JudgeReceiveData.Fric_Status != JudgeReceiveData.Fric_Status)
		{
			// 底盘控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 1;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Minipc_Status != JudgeReceiveData.Minipc_Status)
		{
			// MiniPC状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 2;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Minipc_Mode != JudgeReceiveData.Minipc_Mode)
		{
			// 发射机构用户控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 3;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Booster_status != JudgeReceiveData.Booster_status)
		{
			// 云台控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 4;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Gimbal_Control_Type != JudgeReceiveData.Gimbal_Control_Type)
		{
			// 雷达双倍易伤状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 5;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.PilotLamp_Mode != JudgeReceiveData.PilotLamp_Mode)
		{

			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 6;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}
		if (Last_JudgeReceiveData.MiniPC_Aim_Status != JudgeReceiveData.Antispin_Type)
		{

			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 7;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}
		// 如果没有状态变化，且距离上次数值更新已经过去足够时间，则进入数值更新状态
		if (current_time - last_update_time > 10) // 10ms更新一次数值
		{
			ui_state = UI_STATE_VALUE_UPDATE;
			last_update_time = current_time;
		}
		break;

	case UI_STATE_STATUS_UPDATE:
		// 根据状态类型发送对应的状态更新
		switch (last_status_type)
		{
		case 1: // 摩擦轮状态
			FricChange(0);
			Last_JudgeReceiveData.Fric_Status = JudgeReceiveData.Fric_Status;
			break;
		case 2: // MiniPC在线状态
			MiniPC_Alive_Draw(0);
			Last_JudgeReceiveData.Minipc_Status = JudgeReceiveData.Minipc_Status;
			break;
		case 3: // 云台手自瞄打击目标 前哨/基地
			MiniPCMode_Draw(0);
			Last_JudgeReceiveData.Minipc_Mode = JudgeReceiveData.Minipc_Mode;
			break;
		case 4: // 拨弹盘状态
			BoosterMode_Draw(0);
			Last_JudgeReceiveData.Booster_status = JudgeReceiveData.Booster_status;
			break;
		case 5: // Gimbal状态
			GimbalStatus_Draw(0);
			Last_JudgeReceiveData.Gimbal_Control_Type = JudgeReceiveData.Gimbal_Control_Type;
			break;
		case 6: // Pilot模式
			PliotLampChange(0);
			Last_JudgeReceiveData.PilotLamp_Mode = JudgeReceiveData.PilotLamp_Mode;
			break;
		case 7: // 自瞄识别框
			MiniPC_Aim_Change(0);
			Last_JudgeReceiveData.MiniPC_Aim_Status = JudgeReceiveData.MiniPC_Aim_Status;
		}

		// 增加重试次数
		status_update_retry++;

		// 如果重试次数达到上限或者已经成功发送，则回到空闲状态
		if (status_update_retry >= 5)
		{
			ui_state = UI_STATE_IDLE;
			last_update_time = current_time;
		}
		else
		{
			// 设置下次重试时间
			last_update_time = current_time;
		}
		break;

	case UI_STATE_VALUE_UPDATE:
		// 只更新一个数值，避免占用太多通信资源
		static uint8_t value_update_index = 0;

		// switch (value_update_index)
		// {
		// case 0: // 更新Pitch角度
		// 	if (fabs(Last_JudgeReceiveData.Pitch_Angle - JudgeReceiveData.Pitch_Angle) > 0.01f)
		// 	{
		// 		PitchUI_Change(JudgeReceiveData.Pitch_Angle, 0);
		// 		Last_JudgeReceiveData.Pitch_Angle = JudgeReceiveData.Pitch_Angle;
		// 	}
		// 	break;

		// case 1: // 更新超级电容电压
		// 	if (fabs(Last_JudgeReceiveData.Supercap_Voltage - JudgeReceiveData.Supercap_Voltage) >= 0.01f)
		// 	{
		// 		CapDraw(JudgeReceiveData.Supercap_Voltage, 0);
		// 		// CapUI_Change(JudgeReceiveData.Supercap_Voltage, 0);
		// 		Last_JudgeReceiveData.Supercap_Voltage = JudgeReceiveData.Supercap_Voltage;
		// 	}
		// 	break;

		// case 2: // 更新摩擦轮转速
		// 	//FrictSpeed_Draw(JudgeReceiveData.booster_fric_omega_left, JudgeReceiveData.booster_fric_omega_right, 0);
		// 	break;

		// case 3: // 更新云台底盘夹角与自瞄状态
		// 	if (fabs(Last_JudgeReceiveData.Chassis_Gimbal_Diff - JudgeReceiveData.Chassis_Gimbal_Diff) >= PI / 180.0)
		// 	{
		// 		//CarPosture_Change(JudgeReceiveData.Chassis_Gimbal_Diff, 0); // 直接传入弧度值
		// 		//Last_JudgeReceiveData.Chassis_Gimbal_Diff = JudgeReceiveData.Chassis_Gimbal_Diff;
		// 		//Last_JudgeReceiveData.Minipc_Status = JudgeReceiveData.Minipc_Status;
		// 	}
		// 	break;
		// }

		// 更新索引，循环遍历所有数值
		value_update_index = (value_update_index + 1) % 4;

		// 回到空闲状态
		ui_state = UI_STATE_IDLE;
		last_update_time = current_time;
		break;
	}
}