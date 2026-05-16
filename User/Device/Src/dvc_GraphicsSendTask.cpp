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

#if defined(GIMBAL) && defined(CHASSIS)
#error "GIMBAL and CHASSIS cannot be defined at the same time"
#endif

#if !defined(GIMBAL) && !defined(CHASSIS)
#error "Either GIMBAL or CHASSIS must be defined"
#endif

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
    graph_ui_input_t input;
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
    GRAPH_UI_INPUT_JOYSTICK
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
    GRAPH_UI_INPUT_JOYSTICK
};

static graph_ui_sync_t g_graph_ui_remote_state = {
    GRAPH_UI_SPEED_SLOW,
    GRAPH_UI_MODE_WORKING,
    0U,
    GRAPH_UI_GRIPPER_CLOSE,
    GRAPH_UI_INPUT_JOYSTICK,
    0U
};

static uint16_t g_graph_ui_self_id = 1U;

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

#define REFEREE_DMA_TX_QUEUE_DEPTH 4
#define REFEREE_DMA_MAX_PACKET_LEN SEND_MAX_SIZE

static RefereeDMAPacket_t referee_dma_queue[REFEREE_DMA_TX_QUEUE_DEPTH];
static uint8_t referee_dma_head = 0;
static uint8_t referee_dma_tail = 0;
static uint8_t referee_dma_count = 0;
static volatile uint8_t referee_dma_busy = 0;

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
    if (Referee_DMA_QueueEmpty()) {
        return;
    }
    referee_dma_head = (referee_dma_head + 1) % REFEREE_DMA_TX_QUEUE_DEPTH;
    referee_dma_count--;
}

static void Referee_DMA_StartNext(void)
{
    if (referee_dma_busy || Referee_DMA_QueueEmpty()) {
        return;
    }

    uint16_t len = referee_dma_queue[referee_dma_head].len;
    if (HAL_UART_Transmit_DMA(&huart10, referee_dma_queue[referee_dma_head].data, len) == HAL_OK) {
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

static graph_ui_input_t GraphUI_SanitizeInput(graph_ui_input_t input)
{
    return (input == GRAPH_UI_INPUT_KEYBOARD) ? GRAPH_UI_INPUT_KEYBOARD : GRAPH_UI_INPUT_JOYSTICK;
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

    if (state->input != GRAPH_UI_INPUT_JOYSTICK && state->input != GRAPH_UI_INPUT_KEYBOARD)
    {
        return 0U;
    }

    return 1U;
}

void Referee_DMA_EnqueuePacket(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > REFEREE_DMA_MAX_PACKET_LEN) {
        return;
    }

    if (Referee_DMA_QueueFull()) {
        return;
    }

    memcpy(referee_dma_queue[referee_dma_tail].data, data, len);
    referee_dma_queue[referee_dma_tail].len = len;
    referee_dma_tail = (referee_dma_tail + 1) % REFEREE_DMA_TX_QUEUE_DEPTH;
    referee_dma_count++;
    Referee_DMA_StartNext();
}

void GraphUI_Init(uint16_t self_id)
{
    g_graph_ui_self_id = self_id;
    JudgeReceiveData.robot_id = (uint8_t)self_id;
    memset(&Last_JudgeReceiveData, 0, sizeof(Last_JudgeReceiveData));
    g_graph_ui_state.uplift_rf_percent = 0.0f;
    g_graph_ui_state.uplift_lf_percent = 0.0f;
    g_graph_ui_state.uplift_rb_percent = 0.0f;
    g_graph_ui_state.uplift_lb_percent = 0.0f;
    g_graph_ui_state.power_percent = 0.0f;
    g_graph_ui_state.orientation_forehead = 1U;
    g_graph_ui_state.speed = GRAPH_UI_SPEED_SLOW;
    g_graph_ui_state.mode = GRAPH_UI_MODE_WORKING;
    g_graph_ui_state.stage = 0U;
    g_graph_ui_state.wheel = GRAPH_UI_WHEEL_ON;
    g_graph_ui_state.gripper = GRAPH_UI_GRIPPER_CLOSE;
    g_graph_ui_state.input = GRAPH_UI_INPUT_JOYSTICK;
    GraphSendTask_ResetInit();
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
}

void GraphUI_SetStage(uint8_t stage)
{
    g_graph_ui_state.stage = (stage > 9U) ? 9U : stage;
}

void GraphUI_SetGripper(graph_ui_gripper_t gripper)
{
    g_graph_ui_state.gripper = GraphUI_SanitizeGripper(gripper);
}

void GraphUI_SetInput(graph_ui_input_t input)
{
    g_graph_ui_state.input = GraphUI_SanitizeInput(input);
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

void GraphUI_RemoteSetInput(graph_ui_input_t input)
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
    decoded_state.input = (graph_ui_input_t)data[GRAPH_UI_SYNC_IDX_INPUT];
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
    g_graph_ui_state.stage = (state->stage > 9U) ? 9U : state->stage;
    g_graph_ui_state.gripper = GraphUI_SanitizeGripper(state->gripper);
    g_graph_ui_state.input = GraphUI_SanitizeInput(state->input);

    if ((state->flags & GRAPH_UI_SYNC_FLAG_FULL_REFRESH) != 0U)
    {
        GraphSendTask_ResetInit();
    }
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
	//uint8_t send_cnt = (cmd_id == Drawing_Char_ID) ? 3 : 1;
	uint8_t send_cnt = 1;
	while (send_cnt)
	{
		send_cnt--;
		Referee_DMA_EnqueuePacket(Transmit_Pack, Frame_Length);
	}
}

void GraphUI_TxCpltCallback_Legacy(UART_HandleTypeDef *huart)
{
    if (huart == &huart10) {
        GraphUI_TxCompleteInternal();
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

#if 0
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	static uint8_t optype;
	graphic_data_struct_t *P_graphic_data;

	// 确定操作类型
	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	if (JudgeReceiveData.Chassis_Control_Type == Chassis_Control_Type_Drive)
	{
		// 第一条车道线
		P_graphic_data = Line_Draw(1, optype, 650, 0, 1145, 450, 4, Orange, LaneLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 第二条车道线
		P_graphic_data = Line_Draw(1, optype, 1920, 180, 1419, 440, 4, Orange, LaneLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);
	}
	else
	{
		// 第一条车道线
		P_graphic_data = Line_Draw(1, optype, SCREEN_LENGTH * 0.41, SCREEN_WIDTH * 0.3, SCREEN_LENGTH * 0.25, 0, 4, Orange, LaneLineName1);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		// 第二条车道线
		P_graphic_data = Line_Draw(1, optype, SCREEN_LENGTH * 0.59, SCREEN_WIDTH * 0.3, SCREEN_LENGTH * 0.75, 0, 4, Orange, LaneLineName2);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);
	}

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
	uint8_t M[] = "M";
	uint8_t F[] = "F";

	P_graphic_data = Arc_Draw(1, Op_Add, 960, 540, 170, 190, 300, 260, 10, White, GIMLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);

	Char_Draw(1, Op_Add, 886, 270, 20, sizeof(N), 2, White, GIMLineName2, N);

	Char_Draw(1, Op_Add, 952, 261, 20, sizeof(M), 2, White, GIMLineName3, M);
	
	Char_Draw(1, Op_Add, 1021, 270, 20, sizeof(F), 2, White, GIMLineName4, F);
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
	uint16_t start_angle;
	uint16_t end_angle;

	graphic_data_struct_t *P_graphic_data;

	theta = (int16_t)theta % 360;
	theta = theta - Reference_Angle;

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

	switch (JudgeReceiveData.Gimbal_Control_Type)
	{
	case 1: // Gimbal_Control_Type_NORMAL
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 170, 176, 300, 260, 10, Green, GIMLineName1);
		break;
	case 2: // Gimbal_Control_Type_MINIPC
		P_graphic_data = Arc_Draw(0, optype, 960, 540, 176, 184, 300, 260, 10, Green, GIMLineName1);
		break;
	case 3: // Gimbal_Control_Type_FOLD
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
 *函 数 名: GraphicSendtask
 *功能说明: ͼ�η�������
 *形    参: ��
 *返 回 值: ��
 **********************************************************************************************************/
#endif
static uint16_t GraphUI_LiftStartY(float percent)
{
	float clamped = GraphUI_ClampPercent01(percent);
	float height_f = 10.0f + clamped * 135.0f / 100.0f;
	int height = GraphUI_RoundPositive(height_f);
	return (uint16_t)(383 - height);
}

static uint16_t GraphUI_PowerEndX(void)
{
	float clamped = GraphUI_ClampMinZero(g_graph_ui_state.power_percent);
	float width_f = 3.0f + clamped * 677.0f / 100.0f;
	int end_x = 606 + GraphUI_RoundPositive(width_f);
	if (end_x > 2047)
	{
		end_x = 2047;
	}
	return (uint16_t)end_x;
}

static void GraphUI_GetModeColors(uint8_t rect_colors[5], uint8_t text_colors[5])
{
	uint8_t i = 0;
	for (i = 0; i < 5; i++)
	{
		rect_colors[i] = Orange;
		text_colors[i] = Orange;
	}

	switch (g_graph_ui_state.mode)
	{
	case GRAPH_UI_MODE_MOVING:
		rect_colors[1] = Green;
		text_colors[1] = Green;
		break;
	case GRAPH_UI_MODE_UPLIFT:
		rect_colors[2] = Green;
		text_colors[2] = Green;
		break;
	case GRAPH_UI_MODE_DOWNLIFT:
		rect_colors[3] = Green;
		text_colors[3] = Green;
		break;
	case GRAPH_UI_MODE_SAVELOAD:
		rect_colors[4] = Green;
		text_colors[4] = Green;
		break;
	case GRAPH_UI_MODE_DISABLE:
		break;
	case GRAPH_UI_MODE_WORKING:
	default:
		rect_colors[0] = Green;
		text_colors[0] = Green;
		break;
	}
}

static void GraphUI_SendFigureGroupA(uint8_t op_type)
{
	static uint8_t n0[] = "F00";
	static uint8_t n1[] = "F01";
	static uint8_t n2[] = "F02";
	static uint8_t n3[] = "F03";
	static uint8_t n4[] = "F04";
	static uint8_t n5[] = "F05";
	static uint8_t n6[] = "F06";
	graphic_data_struct_t *p = NULL;
	uint8_t power_color = (g_graph_ui_state.power_percent > 100.0f) ? 4U : Green;

	p = Rectangle_Draw(0, op_type, 291, 171, 319, 319, 3, White, n0);
	memcpy(&data_pack[DRAWING_PACK * 0], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 356, 171, 384, 319, 3, White, n1);
	memcpy(&data_pack[DRAWING_PACK * 1], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 414, 171, 442, 319, 3, White, n2);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 602, 90, 1329, 140, 3, White, n3);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)p, DRAWING_PACK);
	p = Circle_Draw(0, op_type, 958, 539, 80, 3, (g_graph_ui_state.speed == GRAPH_UI_SPEED_AXEL) ? Orange : Green, n4);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 233, 171, 261, 319, 3, White, n5);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 606, 94, GraphUI_PowerEndX(), 100, 40, power_color, n6);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7);
}

static void GraphUI_SendFigureGroupB(uint8_t op_type)
{
	static uint8_t n0[] = "F07";
	static uint8_t n1[] = "F08";
	static uint8_t n2[] = "F09";
	static uint8_t n3[] = "F10";
	static uint8_t n4[] = "F11";
	static uint8_t n5[] = "F12";
	static uint8_t n6[] = "F13";
	graphic_data_struct_t *p = NULL;
	uint8_t rect_colors[5];
	uint8_t text_colors[5];
	(void)text_colors;
	GraphUI_GetModeColors(rect_colors, text_colors);

	p = Rectangle_Draw(0, op_type, 235, GraphUI_LiftStartY(g_graph_ui_state.uplift_rf_percent), 235, 383, 25, Orange, n0);
	memcpy(&data_pack[DRAWING_PACK * 0], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 294, GraphUI_LiftStartY(g_graph_ui_state.uplift_lf_percent), 294, 383, 25, Orange, n1);
	memcpy(&data_pack[DRAWING_PACK * 1], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 359, GraphUI_LiftStartY(g_graph_ui_state.uplift_rb_percent), 359, 383, 25, Orange, n2);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 418, GraphUI_LiftStartY(g_graph_ui_state.uplift_lb_percent), 418, 383, 25, Orange, n3);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 793, 253, 843, 303, 3, rect_colors[0], n4);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 866, 252, 916, 302, 3, rect_colors[1], n5);
	memcpy(&data_pack[DRAWING_PACK * 5], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 936, 252, 986, 302, 3, rect_colors[2], n6);
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic7_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 7);
}

static void GraphUI_SendFigureGroupC(uint8_t op_type)
{
	static uint8_t n0[] = "F14";
	static uint8_t n1[] = "F15";
	static uint8_t n2[] = "F16";
	graphic_data_struct_t *p = NULL;
	uint8_t rect_colors[5];
	uint8_t text_colors[5];
	(void)text_colors;
	GraphUI_GetModeColors(rect_colors, text_colors);

	p = Rectangle_Draw(0, op_type, 1004, 252, 1054, 302, 3, rect_colors[3], n0);
	memcpy(&data_pack[DRAWING_PACK * 0], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 1072, 252, 1122, 302, 3, rect_colors[4], n1);
	memcpy(&data_pack[DRAWING_PACK * 1], (uint8_t *)p, DRAWING_PACK);
	p = IntData_Draw(0, op_type, 1004, 218, g_graph_ui_state.stage, 20, 2, Yellow, n2);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 3);
}

static void GraphUI_SendCoreStrings(uint8_t op_type)
{
	static uint8_t n0[] = "S00";
	static uint8_t n1[] = "S02";
	static uint8_t n2[] = "S03";
	static uint8_t n3[] = "S04";
	static uint8_t n4[] = "S05";
	static uint8_t n5[] = "S06";
	static uint8_t t_stage[] = "STAGE ";
	static uint8_t t_wheels[] = "WHEELS:";
	static uint8_t t_grip[] = "GRIP:";
	static uint8_t t_on[] = "ON";
	static uint8_t t_off[] = "OFF";
	static uint8_t t_open[] = "OPEN";
	static uint8_t t_close[] = "CLOSE";
	static uint8_t t_fore[] = "FOREHEAD";
	static uint8_t t_rear[] = "REARBACK";

	Char_Draw(0, op_type, 876, 882, 20, (g_graph_ui_state.orientation_forehead ? (uint8_t)sizeof(t_fore) : (uint8_t)sizeof(t_rear)) - 1U, 2, g_graph_ui_state.orientation_forehead ? Green : Orange, n0, g_graph_ui_state.orientation_forehead ? t_fore : t_rear);
	Char_Draw(0, op_type, 895, 218, 20, sizeof(t_stage) - 1U, 2, Yellow, n1, t_stage);
	Char_Draw(0, op_type, 1456, 343, 25, sizeof(t_wheels) - 1U, 3, Cyan, n2, t_wheels);
	Char_Draw(0, op_type, 1644, 343, 25, (g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? (uint8_t)sizeof(t_on) : (uint8_t)sizeof(t_off)) - 1U, 3, g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? Green : Orange, n3, g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? t_on : t_off);
	Char_Draw(0, op_type, 1455, 401, 25, sizeof(t_grip) - 1U, 3, Cyan, n4, t_grip);
	Char_Draw(0, op_type, 1608, 401, 25, (g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? (uint8_t)sizeof(t_open) : (uint8_t)sizeof(t_close)) - 1U, 3, g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? Green : Orange, n5, g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? t_open : t_close);
}

static void GraphUI_SendModeStrings(uint8_t op_type)
{
	static uint8_t n0[] = "S07";
	static uint8_t n1[] = "S08";
	static uint8_t n2[] = "S09";
	static uint8_t n3[] = "S10";
	static uint8_t n4[] = "S11";
	static uint8_t z[] = "Z";
	static uint8_t p[] = "P";
	static uint8_t s[] = "S";
	static uint8_t x[] = "X";
	static uint8_t q[] = "Q";
	uint8_t rect_colors[5];
	uint8_t text_colors[5];
	(void)rect_colors;
	GraphUI_GetModeColors(rect_colors, text_colors);

	Char_Draw(0, op_type, 803, 292, 30, 1, 3, text_colors[0], n0, z);
	Char_Draw(0, op_type, 876, 292, 30, 1, 3, text_colors[1], n1, p);
	Char_Draw(0, op_type, 946, 293, 30, 1, 3, text_colors[2], n2, s);
	Char_Draw(0, op_type, 1014, 293, 30, 1, 3, text_colors[3], n3, x);
	Char_Draw(0, op_type, 1082, 293, 30, 1, 3, text_colors[4], n4, q);
}

static void GraphUI_UpdateLiftBars(uint8_t op_type)
{
	static uint8_t n0[] = "F07";
	static uint8_t n1[] = "F08";
	static uint8_t n2[] = "F09";
	static uint8_t n3[] = "F10";
	graphic_data_struct_t *p = NULL;

	p = Rectangle_Draw(0, op_type, 235, GraphUI_LiftStartY(g_graph_ui_state.uplift_rf_percent), 235, 383, 25, Orange, n0);
	memcpy(&data_pack[DRAWING_PACK * 0], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 294, GraphUI_LiftStartY(g_graph_ui_state.uplift_lf_percent), 294, 383, 25, Orange, n1);
	memcpy(&data_pack[DRAWING_PACK * 1], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 359, GraphUI_LiftStartY(g_graph_ui_state.uplift_rb_percent), 359, 383, 25, Orange, n2);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 418, GraphUI_LiftStartY(g_graph_ui_state.uplift_lb_percent), 418, 383, 25, Orange, n3);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 4);
}

static void GraphUI_UpdatePowerBar(uint8_t op_type)
{
	static uint8_t n0[] = "F06";
	graphic_data_struct_t *p = NULL;
	uint8_t power_color = (g_graph_ui_state.power_percent > 100.0f) ? 4U : Green;
	p = Rectangle_Draw(0, op_type, 606, 94, GraphUI_PowerEndX(), 100, 40, power_color, n0);
	memcpy(data_pack, (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

static void GraphUI_UpdateSpeedCircle(uint8_t op_type)
{
	static uint8_t n0[] = "F04";
	graphic_data_struct_t *p = NULL;
	p = Circle_Draw(0, op_type, 958, 539, 80, 3, (g_graph_ui_state.speed == GRAPH_UI_SPEED_AXEL) ? Orange : Green, n0);
	memcpy(data_pack, (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

static void GraphUI_UpdateModeRects(uint8_t op_type)
{
	static uint8_t n0[] = "F11";
	static uint8_t n1[] = "F12";
	static uint8_t n2[] = "F13";
	static uint8_t n3[] = "F14";
	static uint8_t n4[] = "F15";
	graphic_data_struct_t *p = NULL;
	uint8_t rect_colors[5];
	uint8_t text_colors[5];
	(void)text_colors;
	GraphUI_GetModeColors(rect_colors, text_colors);

	p = Rectangle_Draw(0, op_type, 793, 253, 843, 303, 3, rect_colors[0], n0);
	memcpy(&data_pack[DRAWING_PACK * 0], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 866, 252, 916, 302, 3, rect_colors[1], n1);
	memcpy(&data_pack[DRAWING_PACK * 1], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 936, 252, 986, 302, 3, rect_colors[2], n2);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 1004, 252, 1054, 302, 3, rect_colors[3], n3);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)p, DRAWING_PACK);
	p = Rectangle_Draw(0, op_type, 1072, 252, 1122, 302, 3, rect_colors[4], n4);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5);
	GraphUI_SendModeStrings(op_type);
}

static void GraphUI_UpdateStage(uint8_t op_type)
{
	static uint8_t n0[] = "F16";
	graphic_data_struct_t *p = NULL;
	p = IntData_Draw(0, op_type, 1004, 218, g_graph_ui_state.stage, 20, 2, Yellow, n0);
	memcpy(data_pack, (uint8_t *)p, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
}

static void GraphUI_UpdateOrientation(void)
{
	static uint8_t n0[] = "S00";
	static uint8_t t_fore[] = "FOREHEAD";
	static uint8_t t_rear[] = "REARBACK";
	Char_Draw(0, Op_Change, 876, 882, 20, (g_graph_ui_state.orientation_forehead ? (uint8_t)sizeof(t_fore) : (uint8_t)sizeof(t_rear)) - 1U, 2, g_graph_ui_state.orientation_forehead ? Green : Orange, n0, g_graph_ui_state.orientation_forehead ? t_fore : t_rear);
}

static void GraphUI_UpdateWheel(void)
{
	static uint8_t n0[] = "S04";
	static uint8_t t_on[] = "ON";
	static uint8_t t_off[] = "OFF";
	Char_Draw(0, Op_Change, 1644, 343, 25, (g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? (uint8_t)sizeof(t_on) : (uint8_t)sizeof(t_off)) - 1U, 3, g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? Green : Orange, n0, g_graph_ui_state.wheel == GRAPH_UI_WHEEL_ON ? t_on : t_off);
}

static void GraphUI_UpdateGripper(void)
{
	static uint8_t n0[] = "S06";
	static uint8_t t_open[] = "OPEN";
	static uint8_t t_close[] = "CLOSE";
	Char_Draw(0, Op_Change, 1608, 401, 25, (g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? (uint8_t)sizeof(t_open) : (uint8_t)sizeof(t_close)) - 1U, 3, g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? Green : Orange, n0, g_graph_ui_state.gripper == GRAPH_UI_GRIPPER_OPEN ? t_open : t_close);
}

uint8_t Init_Cnt = 10;
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

#if 0
uint16_t ssm = 0;
UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
void GraphicSendtask(void)
{
	//static UI_Update_State_t ui_state = UI_STATE_IDLE; // UI更新状态
	static uint8_t status_update_retry = 0;			   // 状态更新重试次数
	static uint8_t last_status_type = 0;			   // 上次变化的状态类型
	static uint32_t last_update_time = 0;			   // 上次更新时间
	static uint32_t current_time = 0;				   // 当前时间
	static uint32_t last_update_time_value = 0;			   // 上次数值更新时间

	// 获取当前时间
	current_time = DWT_GetTimeline_ms();

	// 初始化阶段发送所有UI元素
	if (Init_Cnt > 0)
	{
		Init_Cnt--;
		if(Init_Cnt == 254)
		{
			Referee_DMA_ClearQueue();
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
			ShootLines_Init_1();			 // 枪口线
			ShootLines_Init_2();
			ShootLines_Init_3();
			ShootLines_Init_4();
		}

		ChassisLine_Change(0, Init_Cnt); // 底盘方向线
		BoostLine_Change();
		PitchUI_Change(0, Init_Cnt);
		GIMLine_Change(Init_Cnt);

		// 初始化完成后，保存当前数据作为比较基准
		memcpy(&Last_JudgeReceiveData, &JudgeReceiveData, sizeof(JudgeReceive_t));

		return;
	}

	if(referee_dma_busy)
	{
		uint32_t update_time_value = DWT_GetTimeline_ms();
		if(update_time_value - last_update_time_value > 500)
		{
			last_update_time_value = update_time_value;
			Referee_DMA_ClearQueue();
		}
	}
	else
	{
		last_update_time_value = DWT_GetTimeline_ms();
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

		if (Last_JudgeReceiveData.Gimbal_Control_Type != JudgeReceiveData.Gimbal_Control_Type)
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
			Last_JudgeReceiveData.Gimbal_Control_Type = JudgeReceiveData.Gimbal_Control_Type;
			break;
		case 3: // 底盘控制类型
			Lanelines_Init();
			Last_JudgeReceiveData.Chassis_Control_Type = JudgeReceiveData.Chassis_Control_Type;
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
			PitchUI_Change(JudgeReceiveData.Pitch_Angle, 0);
			Last_JudgeReceiveData.Pitch_Angle = JudgeReceiveData.Pitch_Angle;
		}

		// 更新超级电容电压
		if (fabs(Last_JudgeReceiveData.Supercap_Voltage - JudgeReceiveData.Supercap_Voltage) >= 0.01f)
		{
			SCapLine_Change();
			Last_JudgeReceiveData.Supercap_Voltage = JudgeReceiveData.Supercap_Voltage;
		}

		// 底盘角度及控制类型
		ChassisLine_Change(JudgeReceiveData.Chassis_Gimbal_Diff, 0);

		// 回到空闲状态
		ui_state = UI_STATE_IDLE;
		last_update_time = current_time;
		break;
	}	
}

#endif
uint16_t ssm = 0;
UI_Update_State_t ui_state = UI_STATE_IDLE;
void GraphicSendtask(void)
{
	static uint8_t status_update_retry = 0;
	static uint8_t last_status_type = 0;
	static uint32_t last_update_time = 0;
	static uint32_t current_time = 0;
	static uint32_t last_update_time_value = 0;

	current_time = DWT_GetTimeline_ms();

	if (Init_Cnt > 0)
	{
		Init_Cnt--;
		if (Init_Cnt == 254U)
		{
			Referee_DMA_ClearQueue();
		}

		if ((Init_Cnt % 2U) == 0U)
		{
			GraphUI_SendFigureGroupA(Op_Add);
			GraphUI_SendFigureGroupC(Op_Add);
			GraphUI_SendCoreStrings(Op_Add);
		}
		else
		{
			GraphUI_SendFigureGroupB(Op_Add);
			GraphUI_SendModeStrings(Op_Add);
		}

		g_graph_ui_last_state = g_graph_ui_state;
		return;
	}

	if (referee_dma_busy)
	{
		uint32_t update_time_value = DWT_GetTimeline_ms();
		if (update_time_value - last_update_time_value > 500U)
		{
			last_update_time_value = update_time_value;
			Referee_DMA_ClearQueue();
		}
	}
	else
	{
		last_update_time_value = DWT_GetTimeline_ms();
	}

	switch (ui_state)
	{
	case UI_STATE_IDLE:
		if (g_graph_ui_last_state.speed != g_graph_ui_state.speed)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 1U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (g_graph_ui_last_state.mode != g_graph_ui_state.mode)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 2U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (g_graph_ui_last_state.orientation_forehead != g_graph_ui_state.orientation_forehead)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 3U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (g_graph_ui_last_state.wheel != g_graph_ui_state.wheel)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 4U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (g_graph_ui_last_state.gripper != g_graph_ui_state.gripper)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 5U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (g_graph_ui_last_state.stage != g_graph_ui_state.stage)
		{
			ui_state = UI_STATE_STATUS_UPDATE;
			last_status_type = 6U;
			status_update_retry = 0U;
			last_update_time = current_time;
			break;
		}
		if (current_time - last_update_time > 10U)
		{
			ui_state = UI_STATE_VALUE_UPDATE;
			last_update_time = current_time;
		}
		break;

	case UI_STATE_STATUS_UPDATE:
		switch (last_status_type)
		{
		case 1U:
			GraphUI_UpdateSpeedCircle(Op_Change);
			g_graph_ui_last_state.speed = g_graph_ui_state.speed;
			break;
		case 2U:
			GraphUI_UpdateModeRects(Op_Change);
			g_graph_ui_last_state.mode = g_graph_ui_state.mode;
			break;
		case 3U:
			GraphUI_UpdateOrientation();
			g_graph_ui_last_state.orientation_forehead = g_graph_ui_state.orientation_forehead;
			break;
		case 4U:
			GraphUI_UpdateWheel();
			g_graph_ui_last_state.wheel = g_graph_ui_state.wheel;
			break;
		case 5U:
			GraphUI_UpdateGripper();
			g_graph_ui_last_state.gripper = g_graph_ui_state.gripper;
			break;
		case 6U:
			GraphUI_UpdateStage(Op_Change);
			g_graph_ui_last_state.stage = g_graph_ui_state.stage;
			break;
		default:
			break;
		}

		status_update_retry++;
		if (status_update_retry >= 10U)
		{
			ui_state = UI_STATE_IDLE;
			last_update_time = current_time;
		}
		else
		{
			last_update_time = current_time;
		}
		break;

	case UI_STATE_VALUE_UPDATE:
		if (g_graph_ui_last_state.uplift_rf_percent != g_graph_ui_state.uplift_rf_percent ||
			g_graph_ui_last_state.uplift_lf_percent != g_graph_ui_state.uplift_lf_percent ||
			g_graph_ui_last_state.uplift_rb_percent != g_graph_ui_state.uplift_rb_percent ||
			g_graph_ui_last_state.uplift_lb_percent != g_graph_ui_state.uplift_lb_percent)
		{
			GraphUI_UpdateLiftBars(Op_Change);
			g_graph_ui_last_state.uplift_rf_percent = g_graph_ui_state.uplift_rf_percent;
			g_graph_ui_last_state.uplift_lf_percent = g_graph_ui_state.uplift_lf_percent;
			g_graph_ui_last_state.uplift_rb_percent = g_graph_ui_state.uplift_rb_percent;
			g_graph_ui_last_state.uplift_lb_percent = g_graph_ui_state.uplift_lb_percent;
		}
		if (g_graph_ui_last_state.power_percent != g_graph_ui_state.power_percent)
		{
			GraphUI_UpdatePowerBar(Op_Change);
			g_graph_ui_last_state.power_percent = g_graph_ui_state.power_percent;
		}
		ui_state = UI_STATE_IDLE;
		last_update_time = current_time;
		break;
	}
}

void GraphSendTask_ResetInit(void)
{
	Referee_DMA_ClearQueue();
	ui_state = UI_STATE_IDLE;
	memset(&Last_JudgeReceiveData, 0, sizeof(Last_JudgeReceiveData));
	Init_Cnt = 255;
}

graphic_data_struct_t *IntData_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, int32_t data_i, uint8_t size, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	memcpy(drawing.graphic_name, name, 3);
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_INT;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;
	drawing.end_angle = 0;
	drawing.radius = ((uint32_t)data_i) & 0x03ff;
	drawing.end_x = (((uint32_t)data_i) >> 10) & 0x07ff;
	drawing.end_y = (((uint32_t)data_i) >> 21) & 0x07ff;
	return &drawing;
}
