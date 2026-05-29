#ifndef __GRAPHICS_SEND_TASK_H
#define __GRAPHICS_SEND_TASK_H

#include "stdint.h"
#include "crt_chassis.h"
#include "dvc_referee.h"

#define PI 3.14159f
#define Reference_Angle__ 1.2372514f
#define DMA_FLAG_TCIF4 ((uint32_t)0x20000020)
/*��Ļ����*/
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920 // ��Ļ�ֱ���

/* ����ϵͳ���ݰ����� */
#define SEND_MAX_SIZE 128 // �ϴ��������ĳ���
#define HEADER_LEN 5	  // ֡ͷ����
#define CMD_LEN 2		  // �����볤��
#define CRC_LEN 2		  // β��CRC16У��
#define DRAWING_PACK 15	  // ��1��ͼ���ݰ�����

#define REFEREE_DMA_TX_QUEUE_DEPTH 4
#define REFEREE_DMA_MAX_PACKET_LEN SEND_MAX_SIZE

typedef struct {
    uint8_t data[REFEREE_DMA_MAX_PACKET_LEN];
    uint16_t len;
} RefereeDMAPacket_t;

/* ����ϵͳ���ݶ�����ID */
#define Drawing_Delete_ID 0x0100
#define Drawing_Graphic1_ID 0x0101
#define Drawing_Graphic2_ID 0x0102
#define Drawing_Graphic5_ID 0x0103
#define Drawing_Graphic7_ID 0x0104
#define Drawing_Char_ID 0x0110

/*��������*/
#define Op_None 0
#define Op_Add 1
#define Op_Change 2
#define Op_Delete 3
#define Op_Init 1 // ��ʼ����Ҳ��������ͼ��
/* ͼ���������	*/
#define CLEAR_ONE_LAYER 1U
#define CLEAR_ALL 2U

/*��ɫ*/
#define Red_Blue 0
#define Yellow 1
#define Green 2
#define Orange 3
#define Purple 4
#define Pink 5
#define Cyan 6
#define Black 7
#define White 8

#define Chassis_Powerdown_Mode 0
#define Chassis_Act_Mode 1
#define Chassis_SelfProtect_Mode 2
#define Chassis_Solo_Mode 3
#define Chassis_Jump_Mode 4
#define Chassis_Test_Mode 5

#define Gimbal_Powerdown_Mode 7
#define Gimbal_Act_Mode 3
#define Gimbal_Armor_Mode 0
#define Gimbal_BigBuf_Mode 2
#define Gimbal_DropShot_Mode 4
#define Gimbal_SI_Mode 5
#define Gimbal_Jump_Mode 6
#define Gimbal_AntiSP_Mode 7
#define Gimbal_SmlBuf_Mode 1

typedef enum
{
    GRAPH_UI_MODE_DISABLE = 0,
    GRAPH_UI_MODE_WORKING,
    GRAPH_UI_MODE_MOVING,
    GRAPH_UI_MODE_UPLIFT,
    GRAPH_UI_MODE_DOWNLIFT,
    GRAPH_UI_MODE_SAVELOAD,
} graph_ui_mode_t;

typedef enum
{
    GRAPH_UI_SPEED_SLOW = 0,
    GRAPH_UI_SPEED_AXEL,
} graph_ui_speed_t;

typedef enum
{
    GRAPH_UI_WHEEL_OFF = 0,
    GRAPH_UI_WHEEL_ON,
} graph_ui_wheel_t;

typedef enum
{
    GRAPH_UI_GRIPPER_CLOSE = 0,
    GRAPH_UI_GRIPPER_OPEN,
} graph_ui_gripper_t;

typedef enum
{
    GRAPH_UI_ORIENTATION_FOREHEAD = 0,
    GRAPH_UI_ORIENTATION_REARBACK,
	GRAPH_UI_ORIENTATION_FOLLOW,
} graph_ui_orientation_t;

typedef struct
{
    graph_ui_speed_t speed;
    graph_ui_mode_t mode;
    uint8_t stage;
    graph_ui_gripper_t gripper;
    graph_ui_orientation_t input;
    bool flags;
} graph_ui_sync_t;

#define GRAPH_UI_SYNC_DLC 8U
#define GRAPH_UI_SYNC_IDX_SPEED 0U
#define GRAPH_UI_SYNC_IDX_MODE 1U
#define GRAPH_UI_SYNC_IDX_STAGE 2U
#define GRAPH_UI_SYNC_IDX_GRIPPER 3U
#define GRAPH_UI_SYNC_IDX_INPUT 4U
#define GRAPH_UI_SYNC_IDX_FLAGS 5U

typedef struct
{
	char SuperPowerLimit;	// 0Ϊ�������ݹرգ���Ϊ0����ʹ�ó�������
	char Chassis_Flag;		// ģʽ����
	char AutoFire_Flag;		// 0��ʾ�ֶ�����1Ϊ�Զ�����
	char Laser_Flag;		// 0��ʾ����رգ�1Ϊ��
	short Pitch_100;		// pitch�Ƕ�,����100֮��
	short Yaw_100;			// yaw�Ƕ�,����100֮��
	char Gimbal_Flag;		// ģʽ����
	char Graphic_Init_Flag; // 0Ϊ�����ʼ��ģʽ��1Ϊ��ʼ������
	char Freq_state;		// ��Ƶ״̬��0��ʾ������Ƶ��1��ʾ����Ƶ
	char Enemy_ID;
	/*�������*/
	char Send_Pack1;
	char Fric_Flag;
} F405_typedef;

enum ARMOR_ID
{
	ARMOR_AIM_LOST = 0,
	ARMOR_ID_1,
	ARMOR_ID_2,
	ARMOR_ID_3,
	ARMOR_ID_4,
	ARMOR_ID_5,
	ARMOR_ID_Sentry,
};

// ͼ�����ݽṹ��
typedef __PACKED_STRUCT
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;//起点/圆心坐标
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
}
graphic_data_struct_t;

/* ͼ�λ������� */
typedef enum
{
	TYPE_LINE = 0U,
	TYPE_RECTANGLE = 1U,
	TYPE_CIRCLE = 2U,
	TYPE_OVAL = 3U,
	TYPE_ARC = 4U,
	TYPE_FLOAT = 5U,
	TYPE_INT = 6U,
	TYPE_CHAR = 7U,
} graphic_tpye;

typedef __packed struct
{
	uint8_t operate_tpye;		  // 0�ղ���  1ɾ������ͼ��  2ɾ������ͼ��
	uint8_t layer;				  // ͼ���  0~9
} client_custom_graphic_delete_t; // �ͻ���ɾ��ͼ��

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t; // �ͻ��˻���һ��ͼ��

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t; // �ͻ��˻�������ͼ��

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t; // �ͻ��˻������ͼ��

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t; // �ͻ��˻����߸�ͼ��

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	char data[30];
} ext_client_custom_character_t; // �ͻ��˻����ַ�

/*����ϵͳ������Ϣ��*���������ϵͳ����Э��*/
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} student_interactive_header_data_t; // ��������

typedef __packed struct // ֡ͷ֡β9B
{
	uint16_t data_cmd_id;							  // ���ݶ�����ID  :2B
	uint16_t sender_ID;								  // ������ID        :2B
	uint16_t receiver_ID;							  // ������ID      :2B
	ext_client_custom_graphic_seven_t graphic_custom; // �Զ���ͼ������: �ͻ��˻����߸�ͼ��  ��105B
} ext_student_interactive_header_data_t;

typedef __packed struct
{
	uint16_t data_cmd_id;					   // ���ݶ�����ID                      :2B
	uint16_t sender_ID;						   // ������ID														:2B
	uint16_t receiver_ID;					   // ������ID													:2B
	ext_client_custom_character_t char_custom; // �Զ����ַ�������   :45B
} ext_student_interactive_char_header_data_t;

typedef struct
{
	uint8_t robot_id;
	uint8_t Chassis_Control_Type;
	uint8_t Minipc_Status; //自瞄当前运行状态
	uint8_t MiniPC_Aim_Status; //上位机存活状态
	uint8_t Fric_Status;  //摩擦轮状态
	uint8_t Supercap_Energy;  // 超级电容能量百分比
	uint8_t Supercap_State; // 超级电容状态
	uint8_t Minipc_Mode;
	uint8_t Gimbal_Control_Type; // 添加云台控制状态字段
	graph_ui_mode_t UI_Mode;
	graph_ui_gripper_t Gripper_Status;
	graph_ui_orientation_t Orientation_Status;
	uint8_t Booster_User_Control_Type;
	uint16_t booster_fric_omega_left;
	float Supercap_Voltage;
	float Pitch_Angle;
	float Chassis_Gimbal_Diff;
	float Yaw_Angle;

} JudgeReceive_t;

void JudgementDataSend(void);
void JudgementCustomizeGraphics(int Op_type);
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void referee_data_load_Graphic(int Op_type);

void referee_data_load_shootUI(uint8_t operate_type, uint8_t robot_level);
void referee_data_load_NumberUI(void);
void GraphicSendtask(void);

void GraphUI_SetLift(float rf_percent, float lf_percent, float rb_percent, float lb_percent);
void GraphUI_SetPower(float power_percent);
void GraphUI_SetOrientation(uint8_t forehead);
void GraphUI_SetWheel(graph_ui_wheel_t status);
void GraphUI_SetSpeed(graph_ui_speed_t speed);
void GraphUI_SetMode(graph_ui_mode_t mode);
void GraphUI_SetStage(uint8_t stage);
void GraphUI_SetGripper(graph_ui_gripper_t gripper);
void GraphUI_SetInput(graph_ui_orientation_t input);

void GraphUI_RemoteSetSpeed(graph_ui_speed_t speed);
void GraphUI_RemoteSetMode(graph_ui_mode_t mode);
void GraphUI_RemoteSetStage(uint8_t stage);
void GraphUI_RemoteSetGripper(graph_ui_gripper_t gripper);
void GraphUI_RemoteSetInput(graph_ui_orientation_t input);
void GraphUI_RemoteRequestFullRefresh(void);
void GraphUI_RemoteRequsetReset();
void GraphUI_RemotePack(uint8_t data[GRAPH_UI_SYNC_DLC]);
uint8_t GraphUI_RemoteUnpack(const uint8_t data[GRAPH_UI_SYNC_DLC], graph_ui_sync_t *out);
void GraphUI_RemoteApply(const graph_ui_sync_t *state);

void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID, uint16_t receiverID, uint8_t *data, uint16_t pack_len);
void Send_toReferee(uint16_t cmd_id, uint16_t data_len);
void Referee_DMA_EnqueuePacket(const uint8_t *data, uint16_t len);

graphic_data_struct_t *Line_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[]);
graphic_data_struct_t *Rectangle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[]);
graphic_data_struct_t *FloatData_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, float data_f, uint8_t size, uint8_t valid_bit, uint16_t line_width, int color, uint8_t name[]);
graphic_data_struct_t *CharGraphic_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[]);

extern F405_typedef F405;

void ShootLines_Init_1(void);
void ShootLines_Init_2(void);
void ShootLines_Init_3(void);
void ShootLines_Init_4(void);
void Pitch_Line_Init_1(void);
void Pitch_Line_Init_2(void);
void Pitch_Line_Init_3(void);
void GIMLine_Init(void);
void SCapLine_Init(void);
void Lanelines_Init(void);
void SCapLine_Change(void);
void ChassisLine_Change(float theta, uint8_t Init_Cnt);
void BoostLine_Change(void);
void GIMLine_Change(uint8_t Init_Cnt);
void PitchUI_Change(float Pitch, uint8_t Init_Cnt);
//void CharChange(uint8_t Init_Flag);

void PitchValue_Change(float pitch, uint8_t Init_Cnt);
void YawValue_Change(float yaw, uint8_t Init_Cnt);

void Char_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[], uint8_t *str_data);

enum
{
	Gimbal = 0,
	Friction,
	Armor,
	Fire,
	ChangeNum
};

enum
{
	LastState = 0,
	NowState
};

extern unsigned char JudgeSend[SEND_MAX_SIZE];
extern JudgeReceive_t JudgeReceiveData;
extern JudgeReceive_t Last_JudgeReceiveData;
extern uint8_t Init_Cnt;
extern volatile uint8_t referee_dma_busy;
extern uint32_t last_update_time_value;
extern uint8_t referee_dma_count;

#endif
