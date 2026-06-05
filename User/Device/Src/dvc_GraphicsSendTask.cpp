/**********************************************************************************************************
 * @文件     Graphics_Send.c
 * @说明     裁判系统图形发送
 * @版本     V2.0
 * @作者     黄志雄
 * @日期     2023.5.1
 **********************************************************************************************************/
#include "dvc_GraphicsSendTask.h"
#include <stm32h7xx.h>
#include <string.h>
#include "usart.h"
#include <stdio.h>

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

#define REFEREE_DMA_TX_QUEUE_DEPTH 40
#define REFEREE_DMA_MAX_PACKET_LEN SEND_MAX_SIZE

static RefereeDMAPacket_t referee_dma_queue[REFEREE_DMA_TX_QUEUE_DEPTH];
static uint8_t referee_dma_head = 0;
static uint8_t referee_dma_tail = 0;
uint8_t referee_dma_count = 0;
volatile uint8_t referee_dma_busy = 0;

static inline uint8_t Referee_DMA_QueueFull(void)
{
	return referee_dma_count >= REFEREE_DMA_TX_QUEUE_DEPTH;
}

static inline uint8_t Referee_DMA_QueueEmpty(void)
{
	return referee_dma_count == 0;
}

static void Referee_DMA_Dequeue(void)
{
	if (Referee_DMA_QueueEmpty())
	{
		return;
	}
	referee_dma_head = (referee_dma_head + 1) % REFEREE_DMA_TX_QUEUE_DEPTH;
	referee_dma_count--;
}

static void Referee_DMA_StartNext(void)
{
	if (referee_dma_busy || Referee_DMA_QueueEmpty())
	{
		return;
	}

	uint16_t len = referee_dma_queue[referee_dma_head].len;
	if (HAL_UART_Transmit_DMA(&huart10, referee_dma_queue[referee_dma_head].data, len) == HAL_OK)
	{
		referee_dma_busy = 1;
	}
}

void Referee_DMA_EnqueuePacket(const uint8_t *data, uint16_t len)
{
	if (len == 0 || len > REFEREE_DMA_MAX_PACKET_LEN)
	{
		return;
	}

	// if (Referee_DMA_QueueFull()) {
	//     return;
	// }

	memcpy(referee_dma_queue[referee_dma_tail].data, data, len);
	referee_dma_queue[referee_dma_tail].len = len;
	referee_dma_tail = (referee_dma_tail + 1) % REFEREE_DMA_TX_QUEUE_DEPTH;
	referee_dma_count++;
	Referee_DMA_StartNext();
}
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
	// uint8_t send_cnt = 3;
	while (send_cnt)
	{
		send_cnt--;
		Referee_DMA_EnqueuePacket(Transmit_Pack, Frame_Length);
	}
}
uint32_t lastcnt;
float dtw;
#ifdef __cplusplus
extern "C"
{
#endif

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
	{
		if (huart == &huart10)
		{
			dtw = 1.0f / DWT_GetDeltaT(&lastcnt);
			// referee_dma_busy = 0;
			// Referee_DMA_Dequeue();
			// Referee_DMA_StartNext();
		}
	}

#ifdef __cplusplus
}
#endif

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
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 第一条车道线
	P_graphic_data = Line_Draw(1, optype, SCREEN_LENGTH * 0.41, SCREEN_WIDTH * 0.3, SCREEN_LENGTH * 0.25, 0, 4, Orange, LaneLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	// 第二条车道线
	P_graphic_data = Line_Draw(1, optype, SCREEN_LENGTH * 0.59, SCREEN_WIDTH * 0.3, SCREEN_LENGTH * 0.75, 0, 4, Orange, LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 发送两个图形
}
/**********************************************************************************************************
 *函 数 名: Shootlines_Init
 *功能说明: 枪口初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void ShootLines_Init_1(void)
{
	static uint8_t ShootLineName1[] = "SL1";
	static uint8_t ShootLineName2[] = "SL2";
	static uint8_t ShootLineName3[] = "SL3";
	static uint8_t ShootLineName4[] = "SL4";
	static uint8_t ShootLineName5[] = "SL5";
	static uint8_t ShootLineName6[] = "SL6";
	static uint8_t ShootLineName7[] = "SL7";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;
	// 左侧纵向虚线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 135 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 135 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 120 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 120 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 105 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 105 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 90 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 90 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 75 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 75 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 60 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 60 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName6);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 45 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 45 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // 发送七个图形
}
void ShootLines_Init_2(void)
{
	static uint8_t ShootLineName1[] = "SL8";
	static uint8_t ShootLineName2[] = "SL9";
	static uint8_t ShootLineName3[] = "SL0";
	static uint8_t ShootLineName4[] = "SLA";
	static uint8_t ShootLineName5[] = "SLB";
	static uint8_t ShootLineName6[] = "SLC";
	static uint8_t ShootLineName7[] = "SLD";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;
	// 右侧纵向虚线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 135 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 135 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 120 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 120 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 105 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 105 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 90 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 90 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 75 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 75 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 60 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 60 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName6);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 45 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 45 + x_bias, SCREEN_WIDTH * 0.5 - 10 + y_bias, 1, White, ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // 发送七个图形
}
void ShootLines_Init_3(void)
{
	static uint8_t ShootLineName1[] = "S31";
	static uint8_t ShootLineName2[] = "S32";
	static uint8_t ShootLineName3[] = "S33";
	static uint8_t ShootLineName4[] = "S34";
	static uint8_t ShootLineName5[] = "S35";
	static uint8_t ShootLineName6[] = "S36";
	static uint8_t ShootLineName7[] = "S37";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// 外侧轮廓瞄准线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 150 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 150 + x_bias, SCREEN_WIDTH * 0.5 - 20 + y_bias, 1, White, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 150 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 150 + x_bias, SCREEN_WIDTH * 0.5 - 20 + y_bias, 1, White, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 150 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 190 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, 1, White, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 150 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 190 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, 1, White, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 + 60 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 + 95 + y_bias, 1, White, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 50 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 210 + y_bias, 1, White, ShootLineName6);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 50 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 210 + y_bias, 1, White, ShootLineName7);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // 发送七个图形
}
void ShootLines_Init_4(void)
{
	static uint8_t ShootLineName1[] = "S37";
	static uint8_t ShootLineName2[] = "S38";
	static uint8_t ShootLineName3[] = "S39";
	static uint8_t ShootLineName4[] = "S3A";
	static uint8_t ShootLineName5[] = "S3B";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// 内侧轮廓瞄准线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 19 + x_bias, SCREEN_WIDTH * 0.5 - 82 + y_bias, SCREEN_LENGTH * 0.5 + 20 + x_bias, SCREEN_WIDTH * 0.5 - 82 + y_bias, 1, White, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 39 + x_bias, SCREEN_WIDTH * 0.5 - 114 + y_bias, SCREEN_LENGTH * 0.5 + 40 + x_bias, SCREEN_WIDTH * 0.5 - 114 + y_bias, 1, White, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 59 + x_bias, SCREEN_WIDTH * 0.5 - 146 + y_bias, SCREEN_LENGTH * 0.5 + 60 + x_bias, SCREEN_WIDTH * 0.5 - 146 + y_bias, 1, White, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 79 + x_bias, SCREEN_WIDTH * 0.5 - 178 + y_bias, SCREEN_LENGTH * 0.5 + 80 + x_bias, SCREEN_WIDTH * 0.5 - 178 + y_bias, 1, White, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 4); // 发送四个图形
}
/**********************************************************************************************************
 *函 数 名: Pitch_Line_Init
 *功能说明: Pitch角度刻度线
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Pitch_Line_Init_1(void)
{
	static uint8_t PitchLineName1[] = "PL1";
	static uint8_t PitchLineName2[] = "PL2";
	static uint8_t PitchLineName3[] = "PL3";
	static uint8_t PitchLineName4[] = "PL4";
	static uint8_t PitchLineName5[] = "PL5";
	static uint8_t PitchLineName6[] = "PL6";
	static uint8_t PitchLineName7[] = "PL7";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// Pitch角度刻度线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 350 + x_bias, SCREEN_WIDTH * 0.5 + 62 + y_bias, SCREEN_LENGTH * 0.5 + 365 + x_bias, SCREEN_WIDTH * 0.5 + 64 + y_bias, 1, White, PitchLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 327 + x_bias, SCREEN_WIDTH * 0.5 + 119 + y_bias, SCREEN_LENGTH * 0.5 + 355 + x_bias, SCREEN_WIDTH * 0.5 + 129 + y_bias, 1, White, PitchLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK); // 40

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 308 + x_bias, SCREEN_WIDTH * 0.5 + 178 + y_bias, SCREEN_LENGTH * 0.5 + 321 + x_bias, SCREEN_WIDTH * 0.5 + 185 + y_bias, 1, White, PitchLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 267 + x_bias, SCREEN_WIDTH * 0.5 + 224 + y_bias, SCREEN_LENGTH * 0.5 + 290 + x_bias, SCREEN_WIDTH * 0.5 + 243 + y_bias, 1, White, PitchLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 348 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 + 378 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, 1, White, PitchLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK); // 0

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 350 + x_bias, SCREEN_WIDTH * 0.5 - 62 + y_bias, SCREEN_LENGTH * 0.5 + 365 + x_bias, SCREEN_WIDTH * 0.5 - 64 + y_bias, 1, White, PitchLineName6);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 327 + x_bias, SCREEN_WIDTH * 0.5 - 119 + y_bias, SCREEN_LENGTH * 0.5 + 355 + x_bias, SCREEN_WIDTH * 0.5 - 129 + y_bias, 1, White, PitchLineName7);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // 发送七个图形
}
void Pitch_Line_Init_2(void)
{
	static uint8_t PitchLineName1[] = "PL8";
	static uint8_t PitchLineName2[] = "PL9";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// Pitch角度刻度线
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 308 + x_bias, SCREEN_WIDTH * 0.5 - 178 + y_bias, SCREEN_LENGTH * 0.5 + 321 + x_bias, SCREEN_WIDTH * 0.5 - 185 + y_bias, 1, White, PitchLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + 267 + x_bias, SCREEN_WIDTH * 0.5 - 224 + y_bias, SCREEN_LENGTH * 0.5 + 290 + x_bias, SCREEN_WIDTH * 0.5 - 243 + y_bias, 1, White, PitchLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 发送两个图形
}
void Pitch_Line_Init_3(void)
{
	static uint8_t PitchLineName1[] = "PLA";
	static uint8_t PitchLineName2[] = "PLB";
	static uint8_t PitchLineName3[] = "PLC";
	static uint8_t PitchLineName4[] = "PLD";
	static uint8_t PitchLineName5[] = "PLE";

	static uint8_t ZERO[] = "0";
	static uint8_t MINUS20[] = "-20";
	static uint8_t MINUS40[] = "-40";
	static uint8_t PLUS20[] = "+20";
	static uint8_t PLUS40[] = "+40";

	Char_Draw(1, Op_Add, 1286, 545, 13, sizeof(ZERO), 1, White, PitchLineName1, ZERO);

	Char_Draw(1, Op_Add, 1260, 662, 13, sizeof(PLUS20), 1, White, PitchLineName2, PLUS20);

	Char_Draw(1, Op_Add, 1197, 763, 13, sizeof(PLUS40), 1, White, PitchLineName3, PLUS40);

	Char_Draw(1, Op_Add, 1260, 424, 13, sizeof(MINUS20), 1, White, PitchLineName4, MINUS20);

	Char_Draw(1, Op_Add, 1197, 325, 13, sizeof(MINUS40), 1, White, PitchLineName5, MINUS40);
}

/**********************************************************************************************************
 *函 数 名: GIMLine_Init
 *功能说明: 云台线初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void GIMLine_Init(void)
{
	static uint8_t GIMLineName1[] = "GL1";
	static uint8_t GIMLineName2[] = "GL2";
	static uint8_t GIMLineName3[] = "GL3";
	static uint8_t GIMLineName4[] = "GL4";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	uint8_t N[] = "N";
	uint8_t R[] = "R";
	uint8_t A[] = "A";

	P_graphic_data = Arc_Draw(1, Op_Add, 960, 540, 170, 190, 300, 260, 10, White, GIMLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);

	Char_Draw(1, Op_Add, 886, 270, 20, sizeof(A), 2, White, GIMLineName2, A);

	Char_Draw(1, Op_Add, 952, 261, 20, sizeof(R), 2, White, GIMLineName3, R);

	Char_Draw(1, Op_Add, 1021, 270, 20, sizeof(N), 2, White, GIMLineName4, N);
}
/**********************************************************************************************************
 *函 数 名: SCapLine_Init
 *功能说明: 超级电容初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void SCapLine_Init(void)
{
	static uint8_t PitchLineName1[] = "PLF";
	static uint8_t PitchLineName2[] = "PLG";
	static uint8_t PitchLineName3[] = "PLH";
	static uint8_t PitchLineName4[] = "PLI";
	graphic_data_struct_t *P_graphic_data;

	static uint8_t E[] = "E";
	static uint8_t F[] = "F";

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 347 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, SCREEN_LENGTH * 0.5 - 377 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, 1, White, PitchLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 266 + x_bias, SCREEN_WIDTH * 0.5 + 224 + y_bias, SCREEN_LENGTH * 0.5 - 289 + x_bias, SCREEN_WIDTH * 0.5 + 243 + y_bias, 1, White, PitchLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // 发送两个图形

	Char_Draw(1, Op_Add, 629, 551, 20, sizeof(E), 2, White, PitchLineName3, E);

	Char_Draw(1, Op_Add, 701, 758, 20, sizeof(F), 2, White, PitchLineName4, F);
}

/**********************************************************************************************************
 *函 数 名: LauncherMode_Init
 *功能说明: 发射模式UI初始化（仅圆形指示器，无文字）
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
/**********************************************************************************************************
 *函 数 名: LauncherMode_Init
 *功能说明: 发射模式UI初始化（显示文字"lob"，并创建隐藏的矩形框）
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void LauncherMode_Init(void)
{
	static uint8_t text_name[] = "LOB";
	static uint8_t frame_name[] = "LBF";
	static uint8_t str[] = "lob";

	// 文字位置：X=1693, Y=520，字体大小15，白色
	Char_Draw(2, Op_Add, 1693, 450, 15, sizeof(str) - 1, 1, Purple, text_name, str);

	// 矩形框初始设为不可见（线宽0，颜色黑色）
	// 框包围文字：文字左上角约(1693, 520)，宽约30，高约15，左右留白5，上下留白2
	uint16_t start_x = 1693 - 5;
	uint16_t start_y = 450 - 2;
	uint16_t end_x = 1693 + 30 + 5;
	uint16_t end_y = 450 + 15 + 2;

	graphic_data_struct_t *rect = Rectangle_Draw(2, Op_Add, start_x, start_y, end_x, end_y, 0, Black, frame_name);
	memcpy(data_pack, (uint8_t *)rect, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: RadarStatus_Init
 *功能说明: 雷达状态UI初始化（显示文字"Standby"或"Ready"，字体20，位置Y=500）
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void RadarStatus_Init(void)
{
	static uint8_t text_name[] = "RDS";
	uint8_t *init_str;
	uint8_t len;
	uint8_t color;

	// 根据当前状态确定初始显示内容和颜色
	if (JudgeReceiveData.Radar_if_Ready == 1)
	{
		static uint8_t ready_str[] = "Ready";
		init_str = ready_str;
		len = sizeof(ready_str) - 1;
		color = Purple;
	}
	else
	{
		static uint8_t standby_str[] = "Standby";
		init_str = standby_str;
		len = sizeof(standby_str) - 1;
		color = Yellow;
	}

	Char_Draw(2, Op_Add, 1593, 500, 20, len, 1, color, text_name, init_str);
}

/**********************************************************************************************************
 *函 数 名: SCapLine_Change
 *功能说明: 超级电容容量
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void SCapLine_Change(void)
{
	static uint8_t PitchLineName1[] = "PLJ";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	P_graphic_data = Arc_Draw(0, Op_Add, 960, 540, 180, 225, 357, 357, 30, Green, PitchLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 发送图形数据
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}
/**********************************************************************************************************
 *函 数 名: ChassisLine_Change
 *功能说明: 底盘方向
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint16_t xxx = 632;
uint16_t yyy = 184;
void ChassisLine_Change(float theta, uint8_t Init_Cnt)
{
	static uint8_t ChassisLineName[] = "CLC";
	static uint8_t optype;
	uint16_t start_angle;
	uint16_t end_angle;

	graphic_data_struct_t *P_graphic_data;

	theta = (int16_t)theta % 360;
	theta = theta - Reference_Angle__;

	// 计算圆弧的起始和终止角度
	// 随着夹角变化，圆弧整体旋转

	start_angle = (uint16_t)(345 + theta) % 360;
	end_angle = (uint16_t)(15 + theta) % 360;

	// 圆弧半径
	uint32_t radius = 83;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	uint8_t indicatorColor;
	switch (JudgeReceiveData.Chassis_Control_Type)
	{
	case 0: // Chassis_Control_Type_DISABLE
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Black, ChassisLineName);
		break;
	case 1: // Chassis_Control_Type_FOLLOW
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, start_angle, end_angle, radius, radius, 15, Green, ChassisLineName);
		break;
	case 2: // Chassis_Control_Type_SPIN
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Orange, ChassisLineName);
		break;
	case 3: // Chassis_Control_Type_DRIVE
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Cyan, ChassisLineName);
		break;
	default:
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Black, ChassisLineName);
		break;
	}

	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 发送图形数据
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: BoostLine_Change
 *功能说明: 摩擦轮状态
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void BoostLine_Change(void)
{
	static uint8_t BoostLineName1[] = "BL2";
	static uint8_t BoostLineName2[] = "BL3";
	static uint8_t BoostLineName3[] = "BL4";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Fric_Status)
	{
	case 0: // Booster_Control_Type_DISABLE
		P_graphic_data = Line_Draw(0, optype, 1593, 726, 1593, 659, 6, Green, BoostLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1592, 724, 1534, 759, 6, Green, BoostLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1593, 724, 1651, 759, 6, Green, BoostLineName3);
		memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 发送图形数据
		Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
		break;
	case 1: // Booster_Control_Type_ENABLE
		P_graphic_data = Line_Draw(0, optype, 1593, 726, 1593, 659, 6, Orange, BoostLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1592, 724, 1534, 759, 6, Orange, BoostLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1593, 724, 1651, 759, 6, Orange, BoostLineName3);
		memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 发送图形数据
		Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
		break;
	default:
		P_graphic_data = Line_Draw(0, optype, 1593, 726, 1593, 659, 6, Black, BoostLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1592, 724, 1534, 759, 6, Black, BoostLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1593, 724, 1651, 759, 6, Black, BoostLineName3);
		memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 发送图形数据
		Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
		break;
	}
}

/**********************************************************************************************************
 *函 数 名: GIMLine_Change
 *功能说明: 云台线初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void GIMLine_Change(uint8_t Init_Cnt)
{
	static uint8_t GIMLineName1[] = "GL5";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.Minipc_Mode)
	{
	case 0: // Gimbal_Control_Type_NORMAL
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 170, 176, 300, 260, 10, Green, GIMLineName1);
		break;
	case 2: // Gimbal_Control_Type_MINIPC
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 176, 184, 300, 260, 10, Green, GIMLineName1);
		break;
	case 1: // Gimbal_Control_Type_FOLD
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 184, 190, 300, 260, 10, Green, GIMLineName1);
		break;
	default:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 170, 190, 300, 260, 10, Black, GIMLineName1);
		break;
	}

	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: GIMLine_Change
 *功能说明: 云台线初始化
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void PitchUI_Change(float Pitch, uint8_t Init_Cnt)
{
	// static uint8_t PitchBackgroundName[] = "PBG"; // Pitch背景圆弧名称
	static uint8_t PitchIndicatorName[] = "PIN"; // Pitch指示器圆弧名称
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	float pitchMin = -40.0f;
	float pitchMax = 40.0f;

	uint16_t bgStartAngle = 40;
	uint16_t bgEndAngle = 140;

	// 计算当前Pitch对应的角度位置
	float pitchRatio = (Pitch - pitchMin) / (pitchMax - pitchMin);		 // 归一化到0-1范围
	pitchRatio = pitchRatio < 0 ? 0 : (pitchRatio > 1 ? 1 : pitchRatio); // 限制在0-1范围内

	// 计算指示器圆弧的角度范围（短弧，宽度为10度）
	uint16_t indicatorAngle = bgStartAngle + (uint16_t)(pitchRatio * (bgEndAngle - bgStartAngle));
	uint16_t indicatorStartAngle = indicatorAngle - 1;
	uint16_t indicatorEndAngle = indicatorAngle + 1;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	P_graphic_data = Arc_Draw(0, optype, 960, 540, indicatorStartAngle, indicatorEndAngle, 375, 375, 36, Red_Blue, PitchIndicatorName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: Scap_Change
 *功能说明: 超电容量百分比
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint16_t ababa = 0;
void Scap_Change(float Scap_Percentage, uint8_t Init_Cnt)
{
	static uint8_t ScapLineName[] = "SCP";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	// 圆弧半径
	uint32_t radius = 300;

	// 计算圆弧的起始和终止角度
	uint16_t startAngle = 270;
	uint16_t endAngle = (uint16_t)(startAngle + (Scap_Percentage / 100.0f) * 40); // 根据百分比计算结束角度
	ababa = endAngle;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 + y_bias, startAngle, endAngle, 360, 360, 10, Green, ScapLineName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	// 发送图形数据
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: PitchValue_Change
 *功能说明: 显示Pitch角度数值（使用浮点图形）
 *形    参: pitch       当前俯仰角（度）
 *          Init_Cnt    初始化标志（0：正常更新，非0：强制以添加模式绘制）
 *返 回 值: 无
 **********************************************************************************************************/
void PitchValue_Change(float pitch, uint8_t Init_Cnt)
{
	static uint8_t PitchValueName[] = "PVA";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	if (pitch > 90.0f)
		pitch = 90.0f;
	if (pitch < -90.0f)
		pitch = -90.0f;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 字体大小20，颜色青色(Cyan)
	P_graphic_data = FloatData_Draw(0, optype, 1593, 550, pitch, 20, 1, 1, Cyan, PitchValueName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: YawValue_Change
 *功能说明: 显示Yaw角度数值（使用浮点图形）
 *形    参: yaw         当前偏航角（度）
 *          Init_Cnt    初始化标志（0：正常更新，非0：强制以添加模式绘制）
 *返 回 值: 无
 **********************************************************************************************************/
void YawValue_Change(float yaw, uint8_t Init_Cnt)
{
	static uint8_t YawValueName[] = "YVA";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	if (yaw > 180.0f)
		yaw = 180.0f;
	if (yaw < -180.0f)
		yaw = -180.0f;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	// 字体大小20，颜色青色(Cyan)
	P_graphic_data = FloatData_Draw(0, optype, 1593, 600, yaw, 20, 1, 1, Pink, YawValueName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

/**********************************************************************************************************
 *函 数 名: LauncherMode_Change
 *功能说明: 发射模式状态变化时更新矩形框（吊射模式显示绿框，否则隐藏）
 *形    参: mode        当前发射模式（0:Normal, 1:Aimer, 2:Launcher）
 *          Init_Cnt    初始化标志（0：正常更新，非0：强制以添加模式绘制）
 *返 回 值: 无
 **********************************************************************************************************/
void LauncherMode_Change(uint8_t mode, uint8_t Init_Cnt)
{
	static uint8_t last_mode = 0xFF;
	static uint8_t frame_name[] = "LBF";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	// 确定矩形框是否可见：吊射模式（mode == 2）显示绿色框，否则隐藏
	uint8_t line_width;
	uint8_t color;
	if (mode == 2) // Launcher 模式
	{
		line_width = 2;
		color = Green;
	}
	else
	{
		line_width = 0; // 线宽0使其不可见
		color = Black;
	}

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	if (Init_Cnt == 0 && mode == last_mode)
		return;

	// 矩形框坐标（与初始化时相同）
	uint16_t start_x = 1693 - 5;
	uint16_t start_y = 450 - 2;
	uint16_t end_x = 1693 + 30 + 5;
	uint16_t end_y = 450 + 15 + 2;

	P_graphic_data = Rectangle_Draw(2, optype, start_x, start_y, end_x, end_y, line_width, color, frame_name);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);

	last_mode = mode;
}

/**********************************************************************************************************
 *函 数 名: RadarStatus_Change
 *功能说明: 雷达状态变化时更新显示文字（0→Standby蓝色，1→Ready红色）
 *形    参: status      当前雷达状态（0:Standby, 1:Ready）
 *          Init_Cnt    初始化标志（0：正常更新，非0：强制以添加模式绘制）
 *返 回 值: 无
 **********************************************************************************************************/
void RadarStatus_Change(uint8_t status, uint8_t Init_Cnt)
{
	static uint8_t last_status = 0xFF;
	static uint8_t text_name[] = "RDS";
	static uint8_t optype;
	uint8_t *str;
	uint8_t len;
	uint8_t color;

	if (status == 1)
	{
		static uint8_t ready_str[] = "Ready";
		str = ready_str;
		len = sizeof(ready_str) - 1;
		color = Purple;
	}
	else
	{
		static uint8_t standby_str[] = "Standby";
		str = standby_str;
		len = sizeof(standby_str) - 1;
		color = Yellow;
	}

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;
	if (Init_Cnt == 0 && status == last_status)
		return;

	Char_Draw(2, optype, 1593, 500, 20, len, 1, color, text_name, str);

	last_status = status;
}

/**********************************************************************************************************
 *函 数 名: GraphicSendtask
 *功能说明: ͼ�η�������
 *形    参: ��
 *返 回 值: ��
 **********************************************************************************************************/
uint8_t Init_Cnt = 10;
// 添加UI更新频率控制计数器
static uint32_t ui_update_counter = 0;

// 添加状态变化标志
static uint8_t status_changed = 0;

uint32_t last_update_time_value = 0; // 上次数值更新时间

// 添加UI更新状态枚举
typedef enum
{
	UI_STATE_IDLE = 0,		// 空闲状态
	UI_STATE_STATUS_UPDATE, // 状态更新状态
	UI_STATE_VALUE_UPDATE	// 数值更新状态
} UI_Update_State_t;

uint16_t ssm = 0;
UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
void GraphicSendtask(void)
{
	// static UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
	static uint8_t status_update_retry = 0; // 状态更新重试次数
	static uint8_t last_status_type = 0;	// 上次变化的状态类型
	static uint32_t last_update_time = 0;	// 上次更新时间
	static uint32_t current_time = 0;		// 当前时间

	// 获取当前时间
	current_time = DWT_GetTimeline_ms();

	if (huart10.hdmatx->State == HAL_DMA_STATE_READY)
	{
		referee_dma_busy = 0;
		Referee_DMA_Dequeue();
		Referee_DMA_StartNext();
	}
	// 初始化阶段发送所有UI元素
	if (Init_Cnt > 0)
	{
		Init_Cnt--;
		if (Init_Cnt == 254)
		{
			referee_dma_busy = 0;
			referee_dma_count = 0;
		}

		if (Init_Cnt % 2 == 0)
		{
			// Pitch_Line_Init_1(); // Pitch线
			// Pitch_Line_Init_2();
			// Pitch_Line_Init_3();
			SCapLine_Init(); // 超电容线
			// Lanelines_Init(); // 车道线
			GIMLine_Init();		// 云台线
			RadarStatus_Init(); // 雷达状态
		}
		else
		{
			ShootLines_Init_1(); // 枪口线
			ShootLines_Init_2();
			ShootLines_Init_3();
			ShootLines_Init_4();
			// LauncherMode_Init();
		}

		ChassisLine_Change(0, Init_Cnt); // 底盘方向线
		BoostLine_Change();
		PitchUI_Change(0, Init_Cnt);
		GIMLine_Change(Init_Cnt);
		Scap_Change(100, Init_Cnt);
		RadarStatus_Change(JudgeReceiveData.Radar_if_Ready, 1);
		PitchValue_Change(JudgeReceiveData.Pitch_Angle, 1);
		YawValue_Change(JudgeReceiveData.Yaw_Angle, 1);
		// LauncherMode_Change(JudgeReceiveData.Launcher_Mode, 1);

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
			ssm++;
			// 摩擦轮状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 1;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Minipc_Mode != JudgeReceiveData.Minipc_Mode)
		{
			// 云台用户控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 2;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Chassis_Control_Type != JudgeReceiveData.Chassis_Control_Type)
		{
			// 底盘控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 3;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}
		if (Last_JudgeReceiveData.Launcher_Mode != JudgeReceiveData.Launcher_Mode)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 4;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}
		if (Last_JudgeReceiveData.Radar_if_Ready != JudgeReceiveData.Radar_if_Ready)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 5; // 新类型号，确保未使用（已用1,2,3,4）
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
			BoostLine_Change();
			Last_JudgeReceiveData.Fric_Status = JudgeReceiveData.Fric_Status;
			break;
		case 2: // 云台控制类型
			GIMLine_Change(0);
			Last_JudgeReceiveData.Minipc_Mode = JudgeReceiveData.Minipc_Mode;
			break;
		case 3: // 底盘控制类型
			Lanelines_Init();
			Last_JudgeReceiveData.Chassis_Control_Type = JudgeReceiveData.Chassis_Control_Type;
			break;
		case 4: // 发射模式
			LauncherMode_Change(JudgeReceiveData.Launcher_Mode, 0);
			Last_JudgeReceiveData.Launcher_Mode = JudgeReceiveData.Launcher_Mode;
			break;
		case 5: // 雷达状态
			RadarStatus_Change(JudgeReceiveData.Radar_if_Ready, 0);
			Last_JudgeReceiveData.Radar_if_Ready = JudgeReceiveData.Radar_if_Ready;
			break;
		}

		// 增加重试次数
		status_update_retry++;

		// 如果重试次数达到上限或者已经成功发送，则回到空闲状态
		if (status_update_retry >= 10)
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
		// 更新所有数值，提高发送频率
		// 更新Pitch角度
		if (fabs(Last_JudgeReceiveData.Pitch_Angle - JudgeReceiveData.Pitch_Angle) > 0.01f)
		{
			// PitchUI_Change(JudgeReceiveData.Pitch_Angle, 0);
			// Last_JudgeReceiveData.Pitch_Angle = JudgeReceiveData.Pitch_Angle;
		}

		// 更新超级电容电压
		if (fabs(Last_JudgeReceiveData.Supercap_Voltage - JudgeReceiveData.Supercap_Voltage) >= 2.0f)
		{
			Scap_Change(JudgeReceiveData.Supercap_Voltage, 0);
			Last_JudgeReceiveData.Supercap_Voltage = JudgeReceiveData.Supercap_Voltage;
		}

		if (fabs(Last_JudgeReceiveData.Pitch_Angle - JudgeReceiveData.Pitch_Angle) > 0.01f)
		{
			PitchValue_Change(JudgeReceiveData.Pitch_Angle, 0);
			Last_JudgeReceiveData.Pitch_Angle = JudgeReceiveData.Pitch_Angle;
		}
		if (fabs(Last_JudgeReceiveData.Yaw_Angle - JudgeReceiveData.Yaw_Angle) > 0.01f)
		{
			YawValue_Change(JudgeReceiveData.Yaw_Angle, 0);
			Last_JudgeReceiveData.Yaw_Angle = JudgeReceiveData.Yaw_Angle;
		}

		// 底盘角度及控制类型
		ChassisLine_Change(JudgeReceiveData.Chassis_Gimbal_Diff, 0);

		// 回到空闲状态
		ui_state = UI_STATE_IDLE;
		last_update_time = current_time;
		break;
	}
}
