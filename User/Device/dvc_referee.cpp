/**
 * @file dvc_referee.cpp
 * @author lez by yssickjgd
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_referee.h"

#include "drv_math.h"
#include "dvc_GraphicsSendTask.h" // 添加这一行，包含完整的JudgeReceive_t定义
#include "buzzer.h"
/* Private macros ------------------------------------------------------------*/
// 删除前向声明，因为已经包含了完整定义
// struct JudgeReceive_t;
extern JudgeReceive_t JudgeReceiveData;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 裁判系统初始化
 *
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Class_Referee::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }

    Frame_Header = __Frame_Header;
}


/**
 * @brief 数据处理过程, 为节约性能不作校验但提供了接口
 * 如遇到大规模丢包或错乱现象, 可重新启用校验过程
 *
 */    
uint16_t buffer_index = 0;
uint16_t cmd_id,data_length;
uint16_t buffer_index_max;
void Class_Referee::Data_Process()
{
    buffer_index = 0;
    buffer_index_max = UART_Manage_Object->Rx_Buffer_Length;
    // 遍历整个接收缓冲区寻找帧头
    while (buffer_index < buffer_index_max)
    {
        // 通过校验和帧头
        if (UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index)] == 0xA5)
        {
            // 数据处理过程
            cmd_id = (UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 6)]) & 0xff;
            cmd_id = (cmd_id << 8) | UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 5)];
            data_length = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 2)] & 0xff;
            data_length = (data_length << 8) | UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 1)];
            Math_Constrain(&data_length,(uint16_t)0,(uint16_t)(128));  //限制数据段最大长度
            Enum_Referee_Command_ID CMD_ID = (Enum_Referee_Command_ID)cmd_id;

            uint8_t *data_temp = new uint8_t[5];
            uint8_t *sum_data = new uint8_t[data_length + 9];
            for (int i = 0; i < 5; i++)
            {
                data_temp[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + i)];
            }
            if (Verify_CRC8_Check_Sum(data_temp, 5) == 1) //校验帧头
            {
                for (int i = 0; i < data_length + 9; i++)
                {
                    sum_data[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + i)];
                }
                if (Verify_CRC16_Check_Sum(sum_data, data_length + 9) == 1) //校验整个帧
                {
                    switch (CMD_ID)
                    {
                    case Referee_Command_ID_GAME_STATUS:
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Game_Status)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Game_Status) + 7;
                        // FPS = FPS_Counter_Update();
                    }
                    break;
                    case (Referee_Command_ID_GAME_RESULT):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Game_Result)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Game_Result) + 7;
                    }
                    break;
                    case (Referee_Command_ID_GAME_ROBOT_HP):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Game_Robot_HP)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Game_Robot_HP) + 7;
                    }
                    break;
                    // 不确定为什么解包这里会导致其他数据错乱
                    // case (Referee_Command_ID_EVENT_DATA):
                    // {
                    //     for (int i = 0; i < data_length + 2; i++)
                    //     {
                    //         reinterpret_cast<uint8_t *>(&Event_Data)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                    //     }
                    //     buffer_index += sizeof(Struct_Referee_Rx_Data_Event_Data) + 7;
                    // }
                    // break;
                    case (Referee_Command_ID_EVENT_SUPPLY):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Event_Supply)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Event_Supply) + 7;
                    }
                    break;
                    case (Referee_Command_ID_EVENT_REFEREE_WARNING):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Event_Referee_Warning)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning) + 7;
                    }
                    break;
                    case (Referee_Command_ID_EVENT_DART_REMAINING_TIME):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Event_Dart_Remaining_Time)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_STATUS):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Status)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Status) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_POWER_HEAT):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Power_Heat)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_POSITION):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Position)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Position) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_BUFF):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Buff)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Buff) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_AERIAL_ENERGY):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Aerial_Energy)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_DAMAGE):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Damage)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Damage) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_BOOSTER):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Booster)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Booster) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_REMAINING_AMMO):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Remaining_Ammo)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_RFID):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_RFID)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_RFID) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_DART_COMMAND):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Dart_Command)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command) + 7;
                    }
                    break;
                    case(Referee_Command_ID_INTERACTION):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Interaction_Students)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Data_Interaction_Students) + 7;
                        if (Interaction_Students.Header == 0x0205)
                        {
                            JudgeReceiveData.Radar_Double_Damage_Flag=Interaction_Students.Data[0];
                        }
                    }
                    break;
                    }
                }
            }
            delete[] sum_data;
            delete[] data_temp;
        }
        buffer_index++;
    }
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data,uint16_t Length)
{
    //滑动窗口, 判断裁判系统是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测裁判系统是否存活
 *
 */
void Class_Referee::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过裁判系统数据
    if (Flag == Pre_Flag)
    {
        //裁判系统断开连接
        Referee_Status = Referee_Status_DISABLE;
        // Buzzer.Set_NowTask(BUZZER_DEVICE_OFFLINE_PRIORITY);
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        //裁判系统保持连接
        Referee_Status = Referee_Status_ENABLE;
    }
    Pre_Flag = Flag;
}


/**
 * @brief 裁判系统发送UI绘图数据
 *
 */
void Class_Referee::UART_Tx_Referee_UI()
{
    Referee_UI_Draw_String(Get_ID(), Referee_UI_Zero , 0 , 0x00, 0, 20, 2, 500, 500, "Chassis", (sizeof("chassis")-1),Referee_UI_ADD);    //配置字符信息
    //Referee_UI_Packed_String(); 
    Referee_UI_Packed_Data(&Interaction_Graphic_String); //打包字符数据
    //UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Buffer_Length); //DMA发送
    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length,10); //阻塞发送

    Referee_UI_Draw_String(Get_ID(), Referee_UI_Zero , 0 , 0x00, 0, 20, 2, 500, 800, "Gimbal", (sizeof("Gimbal")-1),Referee_UI_ADD);    //配置字符信息
    //Referee_UI_Packed_String(); 
    Referee_UI_Packed_Data(&Interaction_Graphic_String); //打包字符数据
    //UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Buffer_Length); //DMA发送
    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length,10); //阻塞发送

    Referee_UI_Draw_String(Get_ID(), Referee_UI_Zero , 0 , 0x00, 0, 20, 2, 500, 1200, "Fric", (sizeof("Fric")-1),Referee_UI_ADD);    //配置字符信息
    //Referee_UI_Packed_String(); 
    Referee_UI_Packed_Data(&Interaction_Graphic_String); //打包字符数据
    //UART_Send_Data(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Buffer_Length); //DMA发送
    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length,10); //阻塞发送
}


/**
 * @brief 裁判系统字符数据打包
 *
 */
void Class_Referee::Referee_UI_Packed_String()
{
    uint16_t frame_length,data_len,cmd_id;
    
    cmd_id = 0x0301;    //子内容ID
    data_len = sizeof(Interaction_Graphic_String);      //字符操作数据长度
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
	memcpy(&UART_Manage_Object->Tx_Buffer[frameheader_len+cmd_len], &Interaction_Graphic_String, sizeof(Interaction_Graphic_String));
	Append_CRC16_Check_Sum(UART_Manage_Object->Tx_Buffer,frame_length);  //一帧数据校验CRC16

    UART_Manage_Object->Tx_Length = frame_length;

    seq++;
}




/**
 * @brief 绘制字符串
 *
 */
void Class_Referee::Referee_UI_Draw_String(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint32_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, char *__String ,uint32_t __String_Length, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_String.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_String.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    memcpy(Interaction_Graphic_String.String, __String, __String_Length * sizeof(uint8_t));
    Interaction_Graphic_String.Graphic_String.String.Serial = __Serial;
    Interaction_Graphic_String.Graphic_String.String.Index[0] = __Index;
    Interaction_Graphic_String.Graphic_String.String.Operation_Enum = __Operate_Type;
    Interaction_Graphic_String.Graphic_String.String.Type_Enum = 7;
    Interaction_Graphic_String.Graphic_String.String.Color_Enum = __Color;
    Interaction_Graphic_String.Graphic_String.String.Font_Size = __Font_Size;
    Interaction_Graphic_String.Graphic_String.String.Line_Width = __Line_Width;
    Interaction_Graphic_String.Graphic_String.String.Start_X = __Start_X;
    Interaction_Graphic_String.Graphic_String.String.Start_Y = __Start_Y;
    Interaction_Graphic_String.Graphic_String.String.Length = __String_Length;
}

/**
 * @brief 绘制直线
 *
 * @param __Robot_ID 机器人ID
 * Enum_Referee_UI_Group_Index __Group_Index,
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Line_Width 线宽
 * @param __Start_X 起始点X坐标
 * @param __Start_Y 起始点Y坐标
 * @param __End_X 结束点X坐标
 * @param __End_Y 结束点Y坐标
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Line(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, uint32_t __End_X, uint32_t __End_Y, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Line.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Type_Enum = 0;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Start_X = __Start_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.Start_Y = __Start_Y;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.End_X = __End_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Line.End_Y = __End_Y;
}

/**
 * @brief 绘制矩形
 *
 * @param __Robot_ID 机器人ID
 * @param __data_type 数据类型
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Line_Width 线宽
 * @param __Start_X 起始点X坐标
 * @param __Start_Y 起始点Y坐标
 * @param __End_X 结束点X坐标
 * @param __End_Y 结束点Y坐标
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Rectangle(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y,  uint32_t __End_X, uint32_t __End_Y,Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Type_Enum = 1;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Start_X = __Start_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.Start_Y = __Start_Y; 
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.End_X = __End_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Rectangle.End_Y = __End_Y;     
}

/**
 * @brief 绘制椭圆
 *
 * @param __Robot_ID 机器人ID
 * @param __data_type 数据类型
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Line_Width 线宽
 * @param __Center_X 中心点X坐标
 * @param __Center_Y 中心点Y坐标
 * @param __X_Length X轴半径
 * @param __Y_Length Y轴半径
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Oval(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index,uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Line_Width, uint32_t __Center_X, uint32_t __Center_Y, uint32_t __X_Length, uint32_t __Y_Length, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Type_Enum = 3;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Center_X = __Center_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Center_Y= __Center_Y; 
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Half_Length_X = __X_Length;
    Interaction_Graphic_7.Graphic[__Group_Index].Oval.Half_Length_Y = __Y_Length;  
}

/**
 * @brief 绘制圆形
 *
 * @param __Robot_ID 机器人ID
 * @param __data_type 数据类型
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Line_Width 线宽
 * @param __Center_X 中心点X坐标
 * @param __Center_Y 中心点Y坐标
 * @param __Radius 半径
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Circle(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Line_Width, uint32_t __Center_X, uint32_t __Center_Y, uint32_t __Radius, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Type_Enum = 2;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Center_X = __Center_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Circle.Center_Y= __Center_Y;   
}

/**
 * @brief 绘制浮点数
 *
 * @param __Robot_ID 机器人ID
 * @param __data_type 数据类型
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Font_Size 字体大小
 * @param __Line_Width 线宽
 * @param __Start_X 起始点X坐标
 * @param __Start_Y 起始点Y坐标
 * @param __Number 浮点数
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Float(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, float __Number, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Float.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Type_Enum = 5;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Font_Size = __Font_Size;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Start_X = __Start_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Start_Y= __Start_Y;
    Interaction_Graphic_7.Graphic[__Group_Index].Float.Float = (int32_t)__Number*1000;      
}

/**
 * @brief 绘制整数
 *
 * @param __Robot_ID 机器人ID
 * @param __data_type 数据类型
 * @param __Serial 序列号
 * @param __Index 索引
 * @param __Color 颜色
 * @param __Font_Size 字体大小
 * @param __Line_Width 线宽
 * @param __Start_X 起始点X坐标
 * @param __Start_Y 起始点Y坐标
 * @param __Number 整数
 * @param __Operate_Type 操作类型
 */
void Class_Referee::Referee_UI_Draw_Integer(uint8_t __Robot_ID,Enum_Referee_UI_Group_Index __Group_Index, uint8_t __Serial, uint8_t __Index, uint32_t __Color, uint32_t __Font_Size,uint32_t __Line_Width, uint32_t __Start_X, uint32_t __Start_Y, int32_t __Number, Enum_Referee_UI_Operate_Type __Operate_Type)
{
    Interaction_Graphic_7.Sender = (Enum_Referee_Data_Robots_ID)__Robot_ID;
    Interaction_Graphic_7.Receiver = (Enum_Referee_Data_Robots_Client_ID)(__Robot_ID + 0x0100);

    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Serial = __Serial;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Index[0] = __Index;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Operation_Enum = __Operate_Type;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Type_Enum = 6;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Color_Enum = __Color;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Font_Size = __Font_Size;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Line_Width = __Line_Width;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Start_X = __Start_X;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Start_Y= __Start_Y;
    Interaction_Graphic_7.Graphic[__Group_Index].Integer.Integer = __Number;
}



unsigned char Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;
	while (dwLength--)
	{
	ucIndex = ucCRC8^(*pchMessage++);
	ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return 0;
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ CRC16_Table[((uint16_t)(wCRC)^(uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC16_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
	pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return;
	}
	wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC16_INIT );
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
