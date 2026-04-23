#include "ui.h"

unsigned char JudgeSend_ui[SEND_MAX_SIZE];
int Char_Change_Array[7];					//0��ʾû�仯����0��ʾ�б仯
int Graphic_Change_Array[7];
float relative_angle,last_relative_angle;
char	spin_mode,last_spin_mode;
char LowHP_Flag,lastLowHP_Flag;									//��Ѫ������

GIMBAL_CONNECTION_T connection;
JudgeReceive_t JudgeReceive;
ext_student_interactive_char_header_data_t custom_char_draw;
ext_student_interactive_header_data_t custom_grapic_draw;

char change_cnt[7];
	int32_t temp_pitch;
float c_pos_x[12] = {0.01,0.01,0.01,0.9,0.9, 0.87,0.64, 0.54,0.40,0.53,0.3,0.4};
float c_pos_y[12] = {0.8,0.7,0.6,0.8 ,0.75, 0.7,0.1, 0.05,0.1 ,0.15,0.5,0.7};
float g_pos_x[CAP_GRAPHIC_NUM] = {0.05,0.05,0.05,0.5,0.92,0.459,0.62,0.5,0.42};
float g_pos_y[CAP_GRAPHIC_NUM] = {0.74,0.64,0.54,0.5,0.64,0.36,0.1,0.8,0.1};
float g_line_x[20]={0.657,0.541,0.343,0.459,0.5,0.5,0.49,0.51,0.48,0.52};
float g_line_y[20]={0.0,0.36,0.0,0.36,0.0,0.5,0.392,0.392,0.37,0.37};
/*��׼��ƫ����*/
int AIM_bias_x = 0;
int AIM_bias_y = 0;
int placece_x[14]={0  , 50, 30,  30, 30,  10,  7,  7,  7,  10,  7,  7,  7 ,10 };
int placece_y[15]={-80,-320,-80,-100,-120,-140,-160,-180,-200,-220,-240,-260,-280,10, 10 };
int Graphic_Change_Check(void)
{
	int i;
	/*���ڳ�ʼ������ͼ�Σ��糵���ߣ����ߣ��͸�����׼�ߵ�*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{
		return Op_Init;	//����Init,��һֱ����Add����������ͼ��
	}
	
	if(connection.connection_rx.fric.flag!=connection.connection_rx.fric.last_flag)
	{
		Graphic_Change_Array[0]=Op_Change;
		change_cnt[0]=5;
	
	}
	
		if(connection.connection_rx.vision.flag!=connection.connection_rx.vision.last_flag)
	{
		Graphic_Change_Array[1]=Op_Change;
		change_cnt[1]=5;
	
	}
	if(connection.connection_rx.follow.flag!=connection.connection_rx.follow.last_flag)
	{
		Graphic_Change_Array[2]=Op_Change;
		change_cnt[2]=5;
	}
	
		if(fabs(relative_angle-last_relative_angle)>0.1f)
	{
		last_relative_angle=relative_angle;
		Graphic_Change_Array[3]=Op_Change;
		change_cnt[3]=10;
	}
	
	if(connection.connection_rx.mode==CHASSIS_SPIN)
	spin_mode=1;
	else
		spin_mode=0;
	
	
	if(spin_mode!=last_spin_mode)
	{
		Graphic_Change_Array[4]=Op_Change;
		change_cnt[4]=5;
	}
	
	
	last_spin_mode=spin_mode;
	
	for(i = 0;i<7;i++)
	{
		if(Graphic_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	return Op_None;	//���ؿղ���
}

int Char_Change_Check(void)
{
	int i;



	/*����ͼ�ν����ʼ��*/
	if(connection.connection_rx.Graphic_Init.flag == 0)		
	{

		return Op_Init;	//����Init,��ʹһֱ����Add����������ͼ��
	}

	
		
	/*��ȡ��̨���͵ĸ���״̬*/
	
	LowHP_Flag = JudgeReceive.robot_state.maxHP * 0.35 > JudgeReceive.robot_state.remainHP ? 1:0;
	
	
	/*�б仯����־����λ*/

	
	temp_pitch=-connection.connection_rx.pitch_angle*1000;
	if(fabs(connection.connection_rx.pitch_angle-connection.connection_rx.last_pitch_angle)>0.2)
	{
		connection.connection_rx.last_pitch_angle=connection.connection_rx.pitch_angle;
		Char_Change_Array[3]=Op_Change;
		change_cnt[3]=5;
	}
	

	
	/*������α�־���ϴαȽ�*/

	lastLowHP_Flag = LowHP_Flag;
	
	
	
	/*������û�з����仯������б仯�򷵻��޸�ͼ��*/
	for(i = 0;i<7;i++)
	{
		if(Char_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	
	return Op_None;	//���򷵻ؿղ�������ʱ���ᷢ�Ͷ���
}

void referee_data_load_String(int Op_type)
{
	static int tick=0;
	static char Fric_State[2][6] = {"CLOSE","OPEN"};
	static char Vision_State[2][6] = {"CLOSE","OPEN"};
	static char Chassis_State[4][10] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Gimbal_State[4][10] = {"NOForce","Normal","SPIN","PRECISION"};
	static char Power_State[2][4] = {"Bat","Cap"};

	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(tick%11)
		{
			/*��̬�ַ�*/
			
			case 0:
			/*******************************Fric Mode �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 1;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=1;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Fric Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[0]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[0]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Fric Mode:");
			break;
			case 1:
			/*******************************Auto Aim Mode �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 2;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=2;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(" Auto Aim:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[1]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[1]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data," Auto Aim:");
			break;
			case	2:
				/*******************************Follow Mode �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 3;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=3;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Follow Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[2]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[2]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Follow Mode:");
			break;
			case	3:
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 4;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=4;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Pitch:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[3]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[3]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Pitch:");
			break;
			case 4:
				custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 7;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=1;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Spin Mode:");
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[5]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[5]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"Spin Mode:");
				break;
				
			case 5:
			PITCH_:	
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 5;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=5;
			custom_char_draw.char_custom.grapic_data_struct.layer=4;
			custom_char_draw.char_custom.grapic_data_struct.color=Purple; 
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.width=1;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[4]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[4]*SCREEN_WIDTH;
			custom_char_draw.char_custom.grapic_data_struct.radius=temp_pitch&0x3ff;
			custom_char_draw.char_custom.grapic_data_struct.end_x=temp_pitch>>10&0x7ff;
			custom_char_draw.char_custom.grapic_data_struct.end_y=temp_pitch>>21&0x7ff;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			

			break;
			
			
			default:
			break;
		}
		tick++;
		return ;
		
	}else if(Op_type == Op_Change)		//����Ǳ�־Ϊ�޸�
	{
		/*Ѱ�����ĸ������˱仯*/
	if(Char_Change_Array[0] == Op_Change)  
		{
			if(change_cnt[0]>0)
			{
			 change_cnt[0] -- ;
			}
			else
			{
			Char_Change_Array[0] = Op_None;
			}
		}
			if(Char_Change_Array[1] == Op_Change)  
		{
			if(change_cnt[1]>0)
			{
			 change_cnt[1] -- ;
			}
			else
			{
			Char_Change_Array[1] = Op_None;
			}
		}
			if(Char_Change_Array[2] == Op_Change)  
		{
			if(change_cnt[2]>0)
			{
			 change_cnt[2] -- ;
			}
			else
			{
			Char_Change_Array[2] = Op_None;
			}
		}
			if(Char_Change_Array[3] == Op_Change)  
		{
			if(change_cnt[3]>0)
			{
			 change_cnt[3] -- ;
				goto PITCH_;
			}
			else
			{
			Char_Change_Array[3] = Op_None;
			}
		}
		if(Char_Change_Array[4] == Op_Change)  
		{
			if(change_cnt[4]>0)
			{
			 change_cnt[4] -- ;
			
			}
			else
			{
			Char_Change_Array[4] = Op_None;
			}
		}
		if(Char_Change_Array[5] == Op_Change)  
		{
			if(change_cnt[5]>0)
			{
			 change_cnt[5] -- ;
			}
			else
			{
			Char_Change_Array[5] = Op_None;
			}
			
		}
		
	
	}
}

void referee_data_load_Graphic(int Op_type)
{
	static int pack_tick = 0;			//���ݰ�������
	static int i;
	static float start_angle,end_angle;
	int packed_tick = 0;							//װ��������
	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(pack_tick % PACK_NUM)
		{
		case 0:
			/*******************************Fric Mode ͼ��*********************************/
		FRIC_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;
				if(connection.connection_rx.fric.flag==2)
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Purple;		
				else if(connection.connection_rx.fric.flag==1)
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
				else
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[0] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_y[0] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=20;
				if(Op_type == Op_Change) goto CONT_0;	
				/*******************************Auto Aim Mode ͼ��*********************************/
		AUTO_AIM:		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=2;
				if(connection.connection_rx.vision.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=g_pos_x[1] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=g_pos_y[1] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=20;
			if(Op_type == Op_Change) goto CONT_1;
				
				/*��׼�߻�����*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_line_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_line_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x=g_line_x[5] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y=g_line_y[5] * SCREEN_WIDTH;
				/*1m��׼��*//*3m��׼��*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*5m��׼��*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 5;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=g_line_x[8] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=g_line_y[8] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x=g_line_x[9] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y=g_line_y[9] * SCREEN_WIDTH;
				
				/*7m��׼��*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y=g_line_y[7] * SCREEN_WIDTH;
				
				/*11m��׼��*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 7;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=g_line_x[6] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=g_line_y[6] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x=g_line_x[7] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y=g_line_y[7] * SCREEN_WIDTH;
				break;
			case 1:
			
			/*ͣ���߻�����*/
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_line_x[0] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_line_y[0] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=g_line_x[1] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=g_line_y[1] * SCREEN_WIDTH;
			
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Purple;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=g_line_x[2] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=g_line_y[2] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x=g_line_x[3] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y=g_line_y[3] * SCREEN_WIDTH;
			/*Follow Modeͼ��*/
			FOLLOW_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=3;
				if(connection.connection_rx.follow.flag)
			custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=g_pos_x[2] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=g_pos_y[2] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=20;
			if(Op_type == Op_Change) goto CONT_2;
		
			SPIN_MODE:		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=4;
				if(spin_mode)
			custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Green;
			else
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Orange;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=g_pos_x[4] * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=g_pos_y[4] * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=20;
			if(Op_type == Op_Change) goto CONT_4;
				break;
			case 2:
					RELATIVE_:

			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 6;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=4;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=5;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Cyan; 
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=10;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=g_pos_x[3]*SCREEN_LENGTH;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=g_pos_x[3]*SCREEN_WIDTH;
			start_angle=relative_angle-50.f;
			if(start_angle>180.f)start_angle-=360.f;
			if(start_angle<-180.f)start_angle+=360.f;
			end_angle=relative_angle+50.f;
			if(end_angle>180.f)end_angle-=360.f;
			if(end_angle<-180.f)end_angle+=360.f;
			if(start_angle<0)
				start_angle+=360.f;
			if(end_angle<0)
				end_angle+=360.f;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=start_angle;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=end_angle;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=40;
			custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=40;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
				if(Op_type == Op_Change) goto CONT_3;
				break;
			
			default:
				break;
		}
		pack_tick++;
	}
	else if(Op_type == Op_Change)
	{
			if(Graphic_Change_Array[0] == Op_Change)  
		{
			goto FRIC_MODE;
	CONT_0:
			if(change_cnt[0])
				change_cnt[0]--;
			else
			Graphic_Change_Array[0]=Op_None;
		}
			if(Graphic_Change_Array[1] == Op_Change)  
		{
			goto AUTO_AIM;
			CONT_1:
			if(change_cnt[1])
				change_cnt[1]--;
			else
			Graphic_Change_Array[1]=Op_None;
		}
		if(Graphic_Change_Array[2] == Op_Change)  
		{
			goto FOLLOW_MODE;
	CONT_2:
			if(change_cnt[2])
				change_cnt[2]--;
			else
			Graphic_Change_Array[2]=Op_None;
		}
		if(Graphic_Change_Array[3] == Op_Change)  
		{
			goto RELATIVE_;
	CONT_3:
			if(change_cnt[3])
				change_cnt[3]--;
			else
			Graphic_Change_Array[3]=Op_None;
		}
		if(Graphic_Change_Array[4] == Op_Change)  
		{
			goto SPIN_MODE;
	CONT_4:
			if(change_cnt[4])
				change_cnt[4]--;
			else
			Graphic_Change_Array[4]=Op_None;
		}
	}
}

void JudgementCustomizeChar(int Op_type)
{
		custom_char_draw.data_cmd_id=0x0110;//�����ַ�

		custom_char_draw.sender_ID=JudgeReceive.robot_state.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_state.robot_id == 101)
				custom_char_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_state.robot_id == 1)
				custom_char_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_state.robot_id == 103)
				custom_char_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_state.robot_id == 104)
				custom_char_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_state.robot_id == 105)
				custom_char_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_state.robot_id == 3)
				custom_char_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_state.robot_id == 4)
				custom_char_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_state.robot_id == 5)
				custom_char_draw.receiver_ID = 0x0105;

/*********************************�Զ����ַ�����***********************************/
		referee_data_load_String(Op_type);
}

void JudgementCustomizeGraphics(int Op_type)
{
		custom_grapic_draw.data_cmd_id=0x0104;//�����߸�ͼ�Σ�����ID����ѯ����ϵͳ�ֲᣩ

		custom_grapic_draw.sender_ID=JudgeReceive.robot_state.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_state.robot_id == 101)
				custom_grapic_draw.receiver_ID = 0x0165;
		if(JudgeReceive.robot_state.robot_id == 1)
				custom_grapic_draw.receiver_ID = 0x0101;
		if(JudgeReceive.robot_state.robot_id == 103)
				custom_grapic_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_state.robot_id == 104)
				custom_grapic_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_state.robot_id == 105)
				custom_grapic_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_state.robot_id == 3)
				custom_grapic_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_state.robot_id == 4)
				custom_grapic_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_state.robot_id == 5)
				custom_grapic_draw.receiver_ID = 0x0105;
			

/*********************************�Զ���ͼ������***********************************/
		referee_data_load_Graphic(Op_type);
}

uint8_t seq = 0;
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(JudgeSend_ui,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	JudgeSend_ui[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&JudgeSend_ui[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	JudgeSend_ui[3] = seq;//�����
	Append_CRC8_Check_Sum(JudgeSend_ui,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&JudgeSend_ui[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&JudgeSend_ui[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend_ui,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/

//	 HAL_UART_Transmit_DMA(&huart6, JudgeSend,frame_length);	
	__disable_irq();  /* 禁止全局中断*/
	HAL_UART_Transmit(&huart6, JudgeSend_ui,frame_length,50);
	__enable_irq();  /* 使能全局中断*/
}

int char_change_state,graphic_change_state;

void UI_Send_Graphic_Task(void)
{
			graphic_change_state = Graphic_Change_Check();
			if(graphic_change_state)
			{
				JudgementCustomizeGraphics(graphic_change_state);
				if(graphic_change_state != Op_None)
				referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw));
			}
}

void UI_Send_Char_Task(void)
{
		char_change_state = Char_Change_Check();
			if(char_change_state)			//�����û�б仯��û�б仯�Ͳ�������ʡ����
			{
				JudgementCustomizeChar(char_change_state);
				if(char_change_state != Op_None)
					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
			}
}