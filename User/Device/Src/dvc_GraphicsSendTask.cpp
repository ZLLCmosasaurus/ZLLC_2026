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

typedef struct
{
    float uplift_rf_percent;
    float uplift_lf_percent;
    float uplift_rb_percent;
    float uplift_lb_percent;
    float power_percent;
    uint8_t orientation_forehead;
    graph_ui_speed_t speed;
    graph_ui_mode_t mode;
    uint8_t stage;
    graph_ui_wheel_t wheel;
    graph_ui_gripper_t gripper;
    graph_ui_orientation_t input;
} graph_ui_state_t;

static graph_ui_state_t g_graph_ui_state = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f,
    1U,
    GRAPH_UI_SPEED_SLOW,
    GRAPH_UI_MODE_WORKING,
    0U,
    GRAPH_UI_WHEEL_ON,
    GRAPH_UI_GRIPPER_CLOSE,
    GRAPH_UI_ORIENTATION_FOREHEAD
};

static graph_ui_state_t g_graph_ui_last_state = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f,
    1U,
    GRAPH_UI_SPEED_SLOW,
    GRAPH_UI_MODE_WORKING,
    0U,
    GRAPH_UI_WHEEL_ON,
    GRAPH_UI_GRIPPER_CLOSE,
    GRAPH_UI_ORIENTATION_FOREHEAD
};

static graph_ui_sync_t g_graph_ui_remote_state = {
    GRAPH_UI_SPEED_SLOW,
    GRAPH_UI_MODE_WORKING,
    0U,
    GRAPH_UI_GRIPPER_CLOSE,
    GRAPH_UI_ORIENTATION_FOREHEAD,
    0U
};

static uint16_t g_graph_ui_self_id = 0U;

static void GraphUI_SendFigureGroupA(uint8_t op_type);
static void GraphUI_SendFigureGroupB(uint8_t op_type);
static void GraphUI_SendFigureGroupC(uint8_t op_type);
static void GraphUI_SendCoreStrings(uint8_t op_type);
static void GraphUI_SendModeStrings(uint8_t op_type);
static void GraphUI_UpdateLiftBars(uint8_t op_type);
static void GraphUI_UpdatePowerBar(uint8_t op_type);
static void GraphUI_UpdateSpeedCircle(uint8_t op_type);
static void GraphUI_UpdateModeRects(uint8_t op_type);
static void GraphUI_UpdateStage(uint8_t op_type);
static void GraphUI_UpdateOrientation(void);
static void GraphUI_UpdateWheel(void);
static void GraphUI_UpdateGripper(void);

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

static void Referee_DMA_ClearQueue(void)
{
    referee_dma_head = 0;
    referee_dma_tail = 0;
    referee_dma_count = 0;
    referee_dma_busy = 0;
}

static float GraphUI_ClampPercent01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 100.0f)
    {
        return 100.0f;
    }
    return value;
}

static float GraphUI_ClampMinZero(float value)
{
    return (value < 0.0f) ? 0.0f : value;
}

static int GraphUI_RoundPositive(float value)
{
    if (value <= 0.0f)
    {
        return 0;
    }
    return (int)(value + 0.5f);
}

static graph_ui_speed_t GraphUI_SanitizeSpeed(graph_ui_speed_t speed)
{
    return (speed == GRAPH_UI_SPEED_AXEL) ? GRAPH_UI_SPEED_AXEL : GRAPH_UI_SPEED_SLOW;
}

static graph_ui_mode_t GraphUI_SanitizeMode(graph_ui_mode_t mode)
{
    switch (mode)
    {
    case GRAPH_UI_MODE_DISABLE:
    case GRAPH_UI_MODE_WORKING:
    case GRAPH_UI_MODE_MOVING:
    case GRAPH_UI_MODE_UPLIFT:
    case GRAPH_UI_MODE_DOWNLIFT:
    case GRAPH_UI_MODE_SAVELOAD:
        return mode;
    default:
        return GRAPH_UI_MODE_WORKING;
    }
}

static graph_ui_wheel_t GraphUI_SanitizeWheel(graph_ui_wheel_t wheel)
{
    return (wheel == GRAPH_UI_WHEEL_OFF) ? GRAPH_UI_WHEEL_OFF : GRAPH_UI_WHEEL_ON;
}

static graph_ui_gripper_t GraphUI_SanitizeGripper(graph_ui_gripper_t gripper)
{
    return (gripper == GRAPH_UI_GRIPPER_OPEN) ? GRAPH_UI_GRIPPER_OPEN : GRAPH_UI_GRIPPER_CLOSE;
}

static graph_ui_orientation_t GraphUI_SanitizeInput(graph_ui_orientation_t input)
{
    switch (input)
    {
    case GRAPH_UI_ORIENTATION_FOREHEAD:
    case GRAPH_UI_ORIENTATION_REARBACK:
    case GRAPH_UI_ORIENTATION_FOLLOW:
        return input;
    default:
        return GRAPH_UI_ORIENTATION_FOREHEAD;
    }
}

static uint8_t GraphUI_RemoteStateValid(const graph_ui_sync_t *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    if (state->speed != GRAPH_UI_SPEED_SLOW && state->speed != GRAPH_UI_SPEED_AXEL)
    {
        return 0U;
    }

    switch (state->mode)
    {
    case GRAPH_UI_MODE_DISABLE:
    case GRAPH_UI_MODE_WORKING:
    case GRAPH_UI_MODE_MOVING:
    case GRAPH_UI_MODE_UPLIFT:
    case GRAPH_UI_MODE_DOWNLIFT:
    case GRAPH_UI_MODE_SAVELOAD:
        break;
    default:
        return 0U;
    }

    if (state->stage > 9U)
    {
        return 0U;
    }

    if (state->gripper != GRAPH_UI_GRIPPER_CLOSE && state->gripper != GRAPH_UI_GRIPPER_OPEN)
    {
        return 0U;
    }

    if (state->input != GRAPH_UI_ORIENTATION_FOREHEAD &&
        state->input != GRAPH_UI_ORIENTATION_REARBACK &&
        state->input != GRAPH_UI_ORIENTATION_FOLLOW)
    {
        return 0U;
    }

    return 1U;
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

static void GraphUI_TxCompleteInternal(void)
{
    referee_dma_busy = 0;
    Referee_DMA_Dequeue();
    Referee_DMA_StartNext();
}

void GraphUI_OnTxComplete(void)
{
    GraphUI_TxCompleteInternal();
}

void GraphUI_SetLift(float rf_percent, float lf_percent, float rb_percent, float lb_percent)
{
    g_graph_ui_state.uplift_rf_percent = GraphUI_ClampPercent01(rf_percent);
    g_graph_ui_state.uplift_lf_percent = GraphUI_ClampPercent01(lf_percent);
    g_graph_ui_state.uplift_rb_percent = GraphUI_ClampPercent01(rb_percent);
    g_graph_ui_state.uplift_lb_percent = GraphUI_ClampPercent01(lb_percent);
}

void GraphUI_SetPower(float power_percent)
{
    g_graph_ui_state.power_percent = GraphUI_ClampMinZero(power_percent);
}

void GraphUI_SetOrientation(uint8_t forehead)
{
    g_graph_ui_state.orientation_forehead = (forehead != 0U) ? 1U : 0U;
}

void GraphUI_SetWheel(graph_ui_wheel_t status)
{
    g_graph_ui_state.wheel = GraphUI_SanitizeWheel(status);
}

void GraphUI_SetSpeed(graph_ui_speed_t speed)
{
    g_graph_ui_state.speed = GraphUI_SanitizeSpeed(speed);
}

void GraphUI_SetMode(graph_ui_mode_t mode)
{
    g_graph_ui_state.mode = GraphUI_SanitizeMode(mode);
    JudgeReceiveData.UI_Mode = g_graph_ui_state.mode;
}

void GraphUI_SetStage(uint8_t stage)
{
    g_graph_ui_state.stage = (stage > 9U) ? 9U : stage;
}

void GraphUI_SetGripper(graph_ui_gripper_t gripper)
{
    g_graph_ui_state.gripper = GraphUI_SanitizeGripper(gripper);
    JudgeReceiveData.Gripper_Status = g_graph_ui_state.gripper;
}

void GraphUI_SetInput(graph_ui_orientation_t input)
{
    g_graph_ui_state.input = GraphUI_SanitizeInput(input);
    JudgeReceiveData.Orientation_Status = g_graph_ui_state.input;
}

void GraphUI_RemoteSetSpeed(graph_ui_speed_t speed)
{
    g_graph_ui_remote_state.speed = GraphUI_SanitizeSpeed(speed);
}

void GraphUI_RemoteSetMode(graph_ui_mode_t mode)
{
    g_graph_ui_remote_state.mode = GraphUI_SanitizeMode(mode);
}

void GraphUI_RemoteSetStage(uint8_t stage)
{
    g_graph_ui_remote_state.stage = (stage > 9U) ? 9U : stage;
}

void GraphUI_RemoteSetGripper(graph_ui_gripper_t gripper)
{
    g_graph_ui_remote_state.gripper = GraphUI_SanitizeGripper(gripper);
}

void GraphUI_RemoteSetInput(graph_ui_orientation_t input)
{
    g_graph_ui_remote_state.input = GraphUI_SanitizeInput(input);
}

void GraphUI_RemoteRequestFullRefresh(void)
{
    g_graph_ui_remote_state.flags |= GRAPH_UI_SYNC_FLAG_FULL_REFRESH;
}

void GraphUI_RemotePack(uint8_t data[GRAPH_UI_SYNC_DLC])
{
    if (data == NULL)
    {
        return;
    }

    memset(data, 0, GRAPH_UI_SYNC_DLC);
    data[GRAPH_UI_SYNC_IDX_SPEED] = (uint8_t)g_graph_ui_remote_state.speed;
    data[GRAPH_UI_SYNC_IDX_MODE] = (uint8_t)g_graph_ui_remote_state.mode;
    data[GRAPH_UI_SYNC_IDX_STAGE] = g_graph_ui_remote_state.stage;
    data[GRAPH_UI_SYNC_IDX_GRIPPER] = (uint8_t)g_graph_ui_remote_state.gripper;
    data[GRAPH_UI_SYNC_IDX_INPUT] = (uint8_t)g_graph_ui_remote_state.input;
    data[GRAPH_UI_SYNC_IDX_FLAGS] = g_graph_ui_remote_state.flags;
    g_graph_ui_remote_state.flags = 0U;
}

uint8_t GraphUI_RemoteUnpack(const uint8_t data[GRAPH_UI_SYNC_DLC], graph_ui_sync_t *out)
{
    graph_ui_sync_t decoded_state;

    if (data == NULL || out == NULL)
    {
        return 0U;
    }

    decoded_state.speed = (graph_ui_speed_t)data[GRAPH_UI_SYNC_IDX_SPEED];
    decoded_state.mode = (graph_ui_mode_t)data[GRAPH_UI_SYNC_IDX_MODE];
    decoded_state.stage = data[GRAPH_UI_SYNC_IDX_STAGE];
    decoded_state.gripper = (graph_ui_gripper_t)data[GRAPH_UI_SYNC_IDX_GRIPPER];
    decoded_state.input = (graph_ui_orientation_t)data[GRAPH_UI_SYNC_IDX_INPUT];
    decoded_state.flags = data[GRAPH_UI_SYNC_IDX_FLAGS];

    if (GraphUI_RemoteStateValid(&decoded_state) == 0U)
    {
        return 0U;
    }

    *out = decoded_state;
    return 1U;
}

void GraphUI_RemoteApply(const graph_ui_sync_t *state)
{
    if (state == NULL)
    {
        return;
    }

    g_graph_ui_state.speed = GraphUI_SanitizeSpeed(state->speed);
    g_graph_ui_state.mode = GraphUI_SanitizeMode(state->mode);
    JudgeReceiveData.UI_Mode = g_graph_ui_state.mode;
    g_graph_ui_state.stage = (state->stage > 9U) ? 9U : state->stage;
    g_graph_ui_state.gripper = GraphUI_SanitizeGripper(state->gripper);
    JudgeReceiveData.Gripper_Status = g_graph_ui_state.gripper;
    g_graph_ui_state.input = GraphUI_SanitizeInput(state->input);
    JudgeReceiveData.Orientation_Status = g_graph_ui_state.input;

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
	graphic_data_struct_t *P_graphic_data;

	P_graphic_data = Rectangle_Draw(1, Op_Add, 1454, 295, 1482, 445, 25, White, ShootLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Rectangle_Draw(1, Op_Add, 1524, 295, 1552, 445, 25, White, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Rectangle_Draw(1, Op_Add, 1594, 295, 1622, 445, 25, White, ShootLineName3);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Rectangle_Draw(1, Op_Add, 1664, 295, 1692, 445, 25, White, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 4); // 发送四个图形
}
void ShootLines_Init_2(void)
{
}
void ShootLines_Init_3(void)
{
}
void ShootLines_Init_4(void)
{
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
	static uint8_t GIMLineName5[] = "GL6";
	static uint8_t GIMLineName6[] = "GL7";
	static uint8_t GIMLineName7[] = "GL8";
	graphic_data_struct_t *P_graphic_data;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	uint8_t D[] = "D";
	uint8_t W[] = "W";
	uint8_t M[] = "M";
	uint8_t U[] = "U";
	uint8_t L[] = "L";
	uint8_t S[] = "S";

	P_graphic_data = Arc_Draw(1, Op_Add, 960, 540, 150, 210, 300, 260, 10, White, GIMLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);

	Char_Draw(1, Op_Add, 790, 295, 20, sizeof(D), 2, White, GIMLineName2, D);

	Char_Draw(1, Op_Add, 858, 274, 20, sizeof(W), 2, White, GIMLineName3, W);

	Char_Draw(1, Op_Add, 928, 263, 20, sizeof(M), 2, White, GIMLineName4, M);

	Char_Draw(1, Op_Add, 998, 263, 20, sizeof(U), 2, White, GIMLineName5, U);

	Char_Draw(1, Op_Add, 1068, 274, 20, sizeof(L), 2, White, GIMLineName6, L);

	Char_Draw(1, Op_Add, 1136, 295, 20, sizeof(S), 2, White, GIMLineName7, S);
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

	graphic_data_struct_t *P_graphic_data;

	(void)theta;

	// 圆弧半径
	uint32_t radius = 83;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	uint16_t x_bias = 0;
	uint16_t y_bias = 0;

	switch (JudgeReceiveData.Orientation_Status)
	{
	case GRAPH_UI_ORIENTATION_FOREHEAD:
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Green, ChassisLineName);
		break;
	case GRAPH_UI_ORIENTATION_REARBACK:
		P_graphic_data = Arc_Draw(0, optype, SCREEN_LENGTH * 0.5 + 632 + x_bias, SCREEN_WIDTH * 0.5 + 184 + y_bias, 0, 360, radius, radius, 15, Orange, ChassisLineName);
		break;
	case GRAPH_UI_ORIENTATION_FOLLOW:
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

	switch (JudgeReceiveData.Gripper_Status)
	{
	case GRAPH_UI_GRIPPER_CLOSE:
		P_graphic_data = Line_Draw(0, optype, 1593, 726, 1593, 659, 6, Green, BoostLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1592, 724, 1534, 759, 6, Green, BoostLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

		P_graphic_data = Line_Draw(0, optype, 1593, 724, 1651, 759, 6, Green, BoostLineName3);
		memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 发送图形数据
		Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
		break;
	case GRAPH_UI_GRIPPER_OPEN:
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

	switch (JudgeReceiveData.UI_Mode)
	{
	case GRAPH_UI_MODE_DISABLE:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 150, 160, 300, 260, 10, Green, GIMLineName1);
		break;
	case GRAPH_UI_MODE_WORKING:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 160, 170, 300, 260, 10, Green, GIMLineName1);
		break;
	case GRAPH_UI_MODE_MOVING:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 170, 180, 300, 260, 10, Green, GIMLineName1);
		break;
	case GRAPH_UI_MODE_UPLIFT:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 180, 190, 300, 260, 10, Green, GIMLineName1);
		break;
	case GRAPH_UI_MODE_DOWNLIFT:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 190, 200, 300, 260, 10, Green, GIMLineName1);
		break;
	case GRAPH_UI_MODE_SAVELOAD:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 200, 210, 300, 260, 10, Green, GIMLineName1);
		break;
	default:
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 150, 210, 300, 260, 10, Black, GIMLineName1);
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
			Pitch_Line_Init_1(); // Pitch线
			Pitch_Line_Init_2();
			Pitch_Line_Init_3();
			SCapLine_Init();  // 超电容线
			Lanelines_Init(); // 车道线
			GIMLine_Init();	  // 云台线
		}
		else
		{
			ShootLines_Init_1(); // 枪口线
			ShootLines_Init_2();
			ShootLines_Init_3();
			ShootLines_Init_4();
		}

		ChassisLine_Change(0, Init_Cnt); // 底盘方向线
		BoostLine_Change();
		PitchUI_Change(0, Init_Cnt);
		GIMLine_Change(Init_Cnt);
		Scap_Change(100, Init_Cnt);

		PitchValue_Change(JudgeReceiveData.Pitch_Angle, 1);
		YawValue_Change(JudgeReceiveData.Yaw_Angle, 1);

		// 初始化完成后，保存当前数据作为比较基准
		memcpy(&Last_JudgeReceiveData, &JudgeReceiveData, sizeof(JudgeReceive_t));

		return;
	}

	// 状态机处理
	switch (ui_state)
	{
	case UI_STATE_IDLE:
		// 检查是否有状态变化
		if (Last_JudgeReceiveData.Gripper_Status != JudgeReceiveData.Gripper_Status)
		{
			ssm++;
			// 摩擦轮状态变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 1;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.UI_Mode != JudgeReceiveData.UI_Mode)
		{
			// 云台用户控制类型变化
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 2;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Orientation_Status != JudgeReceiveData.Orientation_Status)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 4;
			status_update_retry = 0;
			last_update_time = current_time;
			break;
		}

		if (Last_JudgeReceiveData.Chassis_Control_Type != JudgeReceiveData.Chassis_Control_Type)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 3;
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
			Last_JudgeReceiveData.Gripper_Status = JudgeReceiveData.Gripper_Status;
			break;
		case 2: // 云台控制类型
			GIMLine_Change(0);
			Last_JudgeReceiveData.UI_Mode = JudgeReceiveData.UI_Mode;
			break;
		case 3: // 底盘控制类型
			Lanelines_Init();
			Last_JudgeReceiveData.Chassis_Control_Type = JudgeReceiveData.Chassis_Control_Type;
			break;
		case 4: // 底盘方向状态
			ChassisLine_Change(JudgeReceiveData.Chassis_Gimbal_Diff, 0);
			Last_JudgeReceiveData.Orientation_Status = JudgeReceiveData.Orientation_Status;
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
