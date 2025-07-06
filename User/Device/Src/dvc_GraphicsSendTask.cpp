/**
 * @file GraphicsSendTask.cpp
 * @author cjw
 * 
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */
#include "dvc_GraphicsSendTask.h"
#include <stm32h7xx.h>
#include <string.h>
#include "usart.h"

#define CAP_GRAPHIC_NUM 9 // �������ݵ���ͼ����ʾϸ�ָ���
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

*********************		*********************		*********************		*********************
*					*		*					*		*					*		*					*
*	frame_header	*		*	cmd_id			*		*	data			*		*	frame_tail		*
*	(5 bytes)		*	+	*	(2 bytes)		*	+	*	(n bytes)		*	+	*	(2 bytes)		*
*					*		*					*		*					*		*	  				*
*********************		*********************		*********************		*********************

**********************************************************************************************************/

/*			��������				*/
uint8_t Transmit_Pack[128];				   // ����ϵͳ����֡
uint8_t data_pack[DRAWING_PACK * 7] = {0}; // ���ݶβ���
uint8_t DMAsendflag;
/**********************************************************************************************************
 *�� �� ��: Send_UIPack
 *����˵��: ��������UI���ݰ������ݶ��ײ������ݣ�
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/



void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID, uint16_t receiverID, uint8_t *data, uint16_t pack_len)
{
	student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = data_cmd_id;
	custom_interactive_header.send_ID = SendID;
	custom_interactive_header.receiver_ID = receiverID;

	uint8_t header_len = sizeof(custom_interactive_header); // ���ݶ��ײ�����

	memcpy((void *)(Transmit_Pack + 7), &custom_interactive_header, header_len); // �����ݶε����ݶν��з�װ����װ���ף�
	memcpy((void *)(Transmit_Pack + 7 + header_len), data, pack_len);			 // ������֡�����ݶν��з�װ����װ���ݣ�

	Send_toReferee(0x0301, pack_len + header_len); // �����������֡����
}

/**********************************************************************************************************
 *�� �� ��: Send_toReferee
 *����˵��: �������֡�����͸�����ϵͳ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Send_toReferee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len;

	// ֡�ײ���װ
	{
		Transmit_Pack[0] = 0xA5;
		memcpy(&Transmit_Pack[1], (uint8_t *)&data_len, sizeof(data_len)); // ���ݶ���data�ĳ���
		Transmit_Pack[3] = seq++;
		Append_CRC8_Check_Sum(Transmit_Pack, HEADER_LEN); // ֡ͷУ��CRC8
	}

	// ����ID
	memcpy(&Transmit_Pack[HEADER_LEN], (uint8_t *)&cmd_id, CMD_LEN);

	// β������У��CRC16
	Append_CRC16_Check_Sum(Transmit_Pack, Frame_Length);

	uint8_t send_cnt = 3; // ���ʹ���������3��
	while (send_cnt)
	{
		send_cnt--;
		//HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Transmit_Pack, Frame_Length);
		//HAL_UART_Transmit_IT(&huart6, (uint8_t *)Transmit_Pack, Frame_Length);
		// __disable_irq();
		HAL_UART_Transmit(&huart9, (uint8_t *)Transmit_Pack, Frame_Length,15);
		// __enable_irq();
		DMAsendflag = 1; 

		// vTaskDelay(1);
	}
}

/**********************************************************************************************************
 *�� �� ��: Deleta_Layer
 *����˵��: ���ͼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Deleta_Layer(uint8_t layer, uint8_t deleteType)
{
	static client_custom_graphic_delete_t Delete_Graphic; // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	Delete_Graphic.layer = layer;
	Delete_Graphic.operate_tpye = deleteType;
	Send_UIPack(Drawing_Delete_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, (uint8_t *)&Delete_Graphic, sizeof(Delete_Graphic)); // ���ַ�
}

/**********************************************************************************************************
 *�� �� ��: CharGraphic_Draw
 *����˵��: �õ��ַ�ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *CharGraphic_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[])
{

	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CHAR; // 7Ϊ�ַ�����
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;

	drawing.start_angle = size; // �����С
	drawing.end_angle = len;	// �ַ�����
	drawing.width = line_width;

	for (uint8_t i = DRAWING_PACK; i < DRAWING_PACK + 30; i++)
		data_pack[i] = 0;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Char_Draw
 *����˵��: �����ַ�
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Char_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color, uint8_t name[], uint8_t *str_data)
{
	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = CharGraphic_Draw(0, Op_Type, startx, starty, size, len, line_width, color, name);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	memset(&data_pack[DRAWING_PACK], 0, 30);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)str_data, len);
	Send_UIPack(Drawing_Char_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK + 30); // �����ַ�
}

/**********************************************************************************************************
 *�� �� ��: FloatData_Draw
 *����˵��: �õ����Ƹ���ͼ�νṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *FloatData_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, float data_f, uint8_t size, uint8_t valid_bit, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing; // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	static int32_t Data1000;
	Data1000 = (int32_t)(data_f * 1000);
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_FLOAT; // 5Ϊ��������
	drawing.width = line_width;		   // �߿�
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;	   // �����С
	drawing.end_angle = valid_bit; // ��Чλ��

	drawing.radius = Data1000 & 0x03ff;
	drawing.end_x = (Data1000 >> 10) & 0x07ff;
	drawing.end_y = (Data1000 >> 21) & 0x07ff;
	return &drawing;
}

/**********************************************************************************************************
 *�� �� ��: Line_Draw
 *����˵��: ֱ��ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Line_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
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
 *�� �� ��: Rectangle_Draw
 *����˵��: ����ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Rectangle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
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
 *�� �� ��: Circle_Draw
 *����˵��: Բ��ͼ�����ݽṹ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
graphic_data_struct_t *Circle_Draw(uint8_t layer, int Op_Type, uint16_t startx, uint16_t starty, uint32_t radius, uint16_t line_width, int color, uint8_t name[])
{
	static graphic_data_struct_t drawing;  // ����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name, name, 3); // ͼ�����ƣ�3λ
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

/**********************************************************************************************************
 *�� �� ��: Lanelines_Init
 *����˵��: �����߳�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	graphic_data_struct_t *P_graphic_data;
	// ��һ��������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.41, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.31, 0, 4, Orange, LaneLineName1);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	// �ڶ���������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.59, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.69, 0, 4, Orange, LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // ������ͼ��
}

/**********************************************************************************************************
 *�� �� ��: Shootlines_Init
 *����˵��: ǹ�ڳ�ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
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
	// �������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, SCREEN_LENGTH * 0.5 + 40 + x_bias, SCREEN_WIDTH * 0.5 - 72 + y_bias, 1, Green, ShootLineName3);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, SCREEN_LENGTH * 0.5 + 30 + x_bias, SCREEN_WIDTH * 0.5 - 92 + y_bias, 1, Green, ShootLineName1);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, SCREEN_LENGTH * 0.5 + 20 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName4);
	memcpy(&data_pack[DRAWING_PACK * 2], (uint8_t *)P_graphic_data, DRAWING_PACK);

	// �������
	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 40 + y_bias, SCREEN_LENGTH * 0.5 + x_bias, SCREEN_WIDTH * 0.5 - 112 + y_bias, 1, Green, ShootLineName2);
	memcpy(&data_pack[DRAWING_PACK * 3], (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(1, Op_Add, SCREEN_LENGTH * 0.5 - 10 + x_bias, SCREEN_WIDTH * 0.5 - 132 + y_bias, SCREEN_LENGTH * 0.5 + 10 + x_bias, SCREEN_WIDTH * 0.5 - 132 + y_bias, 1, Green, ShootLineName5);
	memcpy(&data_pack[DRAWING_PACK * 4], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic5_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 5); // ������ͼ��
}

/**********************************************************************************************************
 *�� �� ��: CarPosture_Change
 *����˵��: ������̬����
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint16_t RectCenterX = SCREEN_LENGTH * 0.4;
uint16_t RectCenterY = SCREEN_WIDTH * 0.7;
uint16_t startX, startY, endX, endY;
float angle;
float angle1;
void CarPosture_Change(short Yaw_100, uint8_t Init_Cnt)
{
	static uint8_t LaneLineName1[] = "po1";
	static uint8_t LaneLineName2[] = "po2";
	static uint8_t LaneLineName3[] = "po3";
	static uint8_t LaneLineName4[] = "po4";
	static uint8_t LaneLineName5[] = "po5";
	static uint8_t LaneLineName6[] = "po6";
	static uint8_t LaneLineName7[] = "po7";
	graphic_data_struct_t *P_graphic_data;

	static uint16_t len = 50;
	static uint16_t centerx = 200, centery = 700;
	angle = (Yaw_100 / 100.0f) * PI / 180.0f + PI;
	angle1 = (Yaw_100 / 100.0f);

	if (angle1 < 0)
		angle1 += 360;
	uint8_t optype = Init_Cnt == 0 ? Op_Change : Op_Add;
	// ע��
	//	P_graphic_data = Line_Draw(0, optype, centerx,
	//							   centery,
	//							   centerx + len * (-arm_sin_f32(angle)),
	//							   centery + len * (+arm_cos_f32(angle)), 4, Orange, LaneLineName2);
	P_graphic_data = Line_Draw(0, optype, centerx,
							   centery,
							   centerx,
							   centery, 4, Orange, LaneLineName2);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);

	P_graphic_data = Line_Draw(0, optype, 200, 700, 200, 800, 8, Pink, LaneLineName6); // ǹ�ڱ�ʶ��
	memcpy(&data_pack[DRAWING_PACK], (uint8_t *)P_graphic_data, DRAWING_PACK);

	Send_UIPack(Drawing_Graphic2_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // ��7��ͼ��
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
		if(CapVolt>20.0f)
		{
			CapVolt = 20.0f;
		}
		Length = CapVolt / 20.0f *(0.3 * SCREEN_LENGTH);
		P_graphic_data = Line_Draw(0, Op_Change, 0.35 * SCREEN_LENGTH, 0.125 * SCREEN_WIDTH, 0.35 * SCREEN_LENGTH + Length, 0.125 * SCREEN_WIDTH, 27, Green, CapName2);
		memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK);
	}
}

/*字符变化发送*/
void CharChange(uint8_t Init_Flag)
{
	uint8_t BulletOff[] = "OFF";
	uint8_t BulletOn[] = "ON";

	uint8_t FrictionOff[] = "OFF";
	uint8_t FrictionOn[] = "ON";

	uint8_t AutoLost[] = "LOST";
	uint8_t AutoOn[] = "ON";

	uint8_t FireAuto[] = "AUTO";
	uint8_t FireManual[] = "MANUAL";

	uint8_t SPIN[] = "SPIN";
	uint8_t FOLLOW[] = "FOLLOW";
	uint8_t Chassis_Off[] = "OFF";

	uint8_t INIT[] = "INIT";

	uint8_t JAMM[] = "JAMMING!!!";


	/*弹舱状态改变*/
	static uint8_t BulletChangeName[] = "bul";
	if (Init_Flag)
	{
		Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(INIT), 2, Green, BulletChangeName, INIT);
	}
	else
	{
		switch (JudgeReceiveData.Bullet_Status)
		{
		case 1:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(BulletOn), 2, Green, BulletChangeName, BulletOn);
			break;
		case 0:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(BulletOff), 2, Pink, BulletChangeName, BulletOff);
			break;
		}
	}		



	/*摩擦轮状态改变*/
	static uint8_t FrictionChangeName[] = "mcl";
	if (Init_Flag)
	{
	Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOff), 2, Pink, FrictionChangeName, INIT);
	}
	else
	{
		switch (JudgeReceiveData.Fric_Status)
		{
			case 0:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOff), 2, Pink, FrictionChangeName, FrictionOff);
			break;
			case 1:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(FrictionOn), 2, Green, FrictionChangeName, FrictionOn);
			break;
		}
	}		


	/*自瞄连接状态*/
	static uint8_t AutoChangeName[] = "auto";
	if (Init_Flag)
	{
		Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(INIT), 2, Pink, AutoChangeName, INIT);
	}
	else
	{
		switch (JudgeReceiveData.Minipc_Satus)
		{
		case 1:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(AutoOn), 2, Green, AutoChangeName, AutoOn);
			break;
		case 0:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(AutoLost), 2, Pink, AutoChangeName, AutoLost);
			break;
		}
	}		

	/*切换底盘运动模式*/
	static uint8_t ChassisChangeName[] = "fcn";
	if (Init_Flag)
	{
		Char_Draw(0, Op_Add, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(INIT), 2, Green, ChassisChangeName, INIT);
	}
	else
	{
		switch (JudgeReceiveData.Chassis_Control_Type)
		{
		case 0:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(Chassis_Off), 2, Cyan, ChassisChangeName, Chassis_Off);
			break;

		case 1:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(FOLLOW), 2, Green, ChassisChangeName, FOLLOW);
			break;

		case 2:
			Char_Draw(0, Op_Change, 0.9 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(SPIN), 2, Green, ChassisChangeName, SPIN);
			break;
		}
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
	static uint8_t PitchName[] = "pit";
	static uint8_t GimbalName[] = "gim";
	static uint8_t FrictionName[] = "fri";
	static uint8_t AutoName[] = "aim";
	static uint8_t CapStaticName[] = "cpt";
	static uint8_t FireName[] = "frm";
	/*				PITCH�ַ�			*/
	uint8_t pitch_char[] = "PITCH :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, 20, sizeof(pitch_char), 2, Yellow, PitchName, pitch_char);

	/*              GIMBAL�ַ�*/
	uint8_t bullet_char[] = "BULLET :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.55 * SCREEN_WIDTH, 20, sizeof(bullet_char), 2, Yellow, GimbalName, bullet_char);

	/*              FRICTION�ַ�*/
	uint8_t friction_char[] = "FRICTION :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.50 * SCREEN_WIDTH, 20, sizeof(friction_char), 2, Yellow, FrictionName, friction_char);

	/*              ARMOR�ַ�*/
	uint8_t auto_char[] = "AUTO :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.45 * SCREEN_WIDTH, 20, sizeof(auto_char), 2, Yellow, AutoName, auto_char);

	/*              FIREMODE�ַ�*/
	uint8_t fire_char[] = "CHASSIS :";
	Char_Draw(0, Op_Add, 0.80 * SCREEN_LENGTH, 0.40 * SCREEN_WIDTH, 20, sizeof(fire_char), 2, Yellow, FireName, fire_char);

	/*              CAP�ַ�*/
	uint8_t cap_char[] = "CAP :  V";
	Char_Draw(0, Op_Add, 0.40 * SCREEN_LENGTH, 0.1 * SCREEN_WIDTH, 30, sizeof(cap_char), 2, Yellow, CapStaticName, cap_char);
	

}

void MiniPC_Aim_Change(uint8_t Init_Cnt)
{
	/*自瞄获取状态*/
	static uint8_t Auto_Aim_ChangeName[] = "Aim";
	static uint8_t optype;
	graphic_data_struct_t* P_graphic_data;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	switch (JudgeReceiveData.MiniPC_Aim_Status)
	{
		case 1:
			P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.3 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.8 * SCREEN_WIDTH, 2, Green, Auto_Aim_ChangeName);
			memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
		case 0:
			P_graphic_data = Rectangle_Draw(0, optype, 0.3495 * SCREEN_LENGTH, 0.3 * SCREEN_WIDTH, 0.651 * SCREEN_LENGTH, 0.8 * SCREEN_WIDTH, 2, Pink, Auto_Aim_ChangeName);
			memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
		break;
	}	
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK); 		

}


/**********************************************************************************************************
 *�� �� ��: PitchUI_Change
 *����˵��: Pitch�ǶȻ���
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void PitchUI_Change(float Pitch, uint8_t Init_Cnt)
{
	static uint8_t PitchName[] = "Pit";
	static uint8_t optype;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	graphic_data_struct_t *P_graphic_data;

	P_graphic_data = FloatData_Draw(0, optype, 0.90 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, Pitch, 20, 4, 2, Green, PitchName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK); // ���ַ�
}

/**********************************************************************************************************
 *�� �� ��: CapUI_Change
 *����˵��: ���ݵ�������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void CapUI_Change(float CapVolt, uint8_t Init_Cnt)
{
	static uint8_t CapName[] = "cpv";
	static uint8_t optype;

	optype = (Init_Cnt == 0) ? Op_Change : Op_Add;

	graphic_data_struct_t *P_graphic_data;
	P_graphic_data = FloatData_Draw(0, optype, 0.42 * SCREEN_LENGTH + 100, 0.1 * SCREEN_WIDTH, CapVolt, 30, 4, 2, Orange, CapName);
	memcpy(data_pack, (uint8_t *)P_graphic_data, DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceiveData.robot_id, JudgeReceiveData.robot_id + 0x100, data_pack, DRAWING_PACK); // ���ַ�
	
}

/**********************************************************************************************************
 *�� �� ��: GraphicSendtask
 *����˵��: ͼ�η�������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint8_t Init_Cnt = 10;   
void GraphicSendtask(void)
{
	CharChange(Init_Cnt);

	PitchUI_Change(JudgeReceiveData.Pitch_Angle, Init_Cnt);

	CapDraw(JudgeReceiveData.Supercap_Voltage, Init_Cnt); 

	MiniPC_Aim_Change(Init_Cnt);

	CapUI_Change(JudgeReceiveData.Supercap_Voltage, Init_Cnt);

	if (Init_Cnt > 0)
	{
		Init_Cnt--;
		Char_Init(); // �ַ�
		ShootLines_Init(); // ǹ����
		Lanelines_Init();         //������
	}
}
