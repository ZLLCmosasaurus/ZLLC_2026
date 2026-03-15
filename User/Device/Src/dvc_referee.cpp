/**
 * @file dvc_referee.cpp
 * @author cjw by yssickjgd
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_referee.h"

/* Private macros ------------------------------------------------------------*/

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
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if(huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }

    Frame_Header = __Frame_Header;
}
/**
 * @brief 裁判系统8bit循环冗余检验
 *
 * @param Message 消息
 * @param Length 字节数
 * @return uint8_t 校验码
 */
uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint8_t check = 0xff;

    if (Message == nullptr)
    {
        return (check);
    }

    while (Length--)
    {
        index = *Message;
        Message++;
        check = CRC8_TAB[check ^ index];
    }
    return (check);
}
/**
 * @brief 裁判系统16bit循环冗余检验
 *
 * @param Message 消息
 * @param Length 字节数
 * @return uint8_t 校验码
 */
uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint16_t check = 0xffff;

    if (Message == nullptr)
    {
        return (check);
    }

    while (Length--)
    {
        index = *Message;
        Message++;
        check = ((uint16_t) (check) >> 8) ^ CRC16_Table[((uint16_t) (check) ^ (uint16_t) (index)) & 0xff];
    }
    return (check);
}

/**
 * @brief UART通信发送UI函数
 *
 * @param Graphic_1 图形地址
 */
void Class_Referee::UART_Send_Interaction_UI_Graphic_1(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1) - 2;
    tmp_buffer->Sequence = Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_1 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_1 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    tmp_data->Sender = Robot_Status.Robot_ID;
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);

    // UI发一个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1), 100);

    Sequence++;
}
/**
 * @brief UART通信发送UI函数
 *
 * @param Graphic_1 图形地址
 * @param Graphic_2 图形地址
 */
void Class_Referee::UART_Send_Interaction_UI_Graphic_2(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_2) - 2;
    tmp_buffer->Sequence = Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_2 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_2 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
    tmp_data->Sender = Robot_Status.Robot_ID;
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);

    // UI发两个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_2), 100);

    Sequence++;
}

/**
 * @brief UART通信发送UI函数
 *
 * @param Graphic_1 图形地址
 * @param Graphic_2 图形地址
 * @param Graphic_3 图形地址
 * @param Graphic_4 图形地址
 * @param Graphic_5 图形地址
 */
void Class_Referee::UART_Send_Interaction_UI_Graphic_5(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_5) - 2;
    tmp_buffer->Sequence = Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_5 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_5 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
    tmp_data->Sender = Robot_Status.Robot_ID;
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);

    // UI发五个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;
    tmp_data->Graphic[2] = *Graphic_3;
    tmp_data->Graphic[3] = *Graphic_4;
    tmp_data->Graphic[4] = *Graphic_5;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_5), 100);

    Sequence++;
}

/**
 * @brief UART通信发送UI函数
 *
 * @param Graphic_1 图形地址
 * @param Graphic_2 图形地址
 * @param Graphic_3 图形地址
 * @param Graphic_4 图形地址
 * @param Graphic_5 图形地址
 * @param Graphic_6 图形地址
 * @param Graphic_7 图形地址
 */
void Class_Referee::UART_Send_Interaction_UI_Graphic_7(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_6, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_7)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_7) - 2;
    tmp_buffer->Sequence = Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_7 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_7 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
    tmp_data->Sender = Robot_Status.Robot_ID;
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);

    // UI发七个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;
    tmp_data->Graphic[2] = *Graphic_3;
    tmp_data->Graphic[3] = *Graphic_4;
    tmp_data->Graphic[4] = *Graphic_5;
    tmp_data->Graphic[5] = *Graphic_6;
    tmp_data->Graphic[6] = *Graphic_7;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_7), 100);

    Sequence++;
}

/**
 * @brief UART通信发送UI函数
 *
 * @param Graphic_String 图形地址
 * @param String_Content 字符串地址
 */
void Class_Referee::UART_Send_Interaction_UI_Graphic_String(Struct_Referee_Data_Interaction_Graphic_Config *Graphic_String, const char *String_Content)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xa5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_String) - 2;
    tmp_buffer->Sequence = Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_String *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_String *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
    tmp_data->Sender = Robot_Status.Robot_ID;
    tmp_data->Receiver = static_cast<Enum_Referee_Data_Robots_Client_ID>((int) (Robot_Status.Robot_ID) + 0x100);

    // UI发字符串帧内容
    tmp_data->Graphic_String = *Graphic_String;
   // bzero(tmp_data->String, 30);
    strcpy((char *) tmp_data->String, String_Content);

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_String), 80);

    Sequence++;
}

/**
 * @brief 数据处理过程, 为节约性能不作校验但提供了接口
 * 如遇到大规模丢包或错乱现象, 可重新启用校验过程
 *
 */    
//以前版本
// uint16_t buffer_index = 0;
// uint16_t cmd_id,data_length;
//test
uint16_t buffer_index = 0;
uint16_t cmd_id,data_length;
uint16_t temp_cmd_id;
uint16_t buffer_index_max;
int receive_flag = 0;
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
                        //FPS = FPS_Counter_Update();
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
                    // //不确定为什么解包这里会导致其他数据错乱
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
                    // case (Referee_Command_ID_ROBOT_AERIAL_ENERGY):
                    // {
                    //     for (int i = 0; i < data_length + 2; i++)
                    //     {
                    //         reinterpret_cast<uint8_t *>(&Robot_Aerial_Energy)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                    //     }
                    //     buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Aerial_Energy) + 7;
                    // }
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
                    case (Referee_Command_ID_ROBOT_Robot_Position):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Robot_Ally_Position)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Ally_Position) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_Sentry_Info):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Sentry_Info)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Sentry_Info) + 7;
                    }
                    break;
                    case (Referee_Command_ID_ROBOT_Radar_Info):
                    {
                        for (int i = 0; i < data_length + 2; i++)
                        {
                            reinterpret_cast<uint8_t *>(&Radar_Info)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                        }
                        buffer_index += sizeof(Struct_Referee_Rx_Data_Robot_Radar_Info) + 7;
                    }
                    break;
                    // case (Referee_Command_ID_INTERACTION):
                    // {
                    //     for (int i = 0; i < data_length + 2; i++)
                    //     {
                    //         reinterpret_cast<uint8_t *>(&Interaction_Robot_Receive)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                    //     }
                    //     buffer_index += sizeof(Struct_Referee_Tx_Data_Interaction_Robot_Receive) + 7;
                    // }
                    // break;
                    // case (Referee_Command_ID_INTERACTION_RADAR_SEND):
                    // {
                    //     for (int i = 0; i < data_length + 2; i++)
                    //     {
                    //         reinterpret_cast<uint8_t *>(&Interaction_Radar_Send)[i] = UART_Manage_Object->Rx_Buffer[Get_Circle_Index(buffer_index + 7 + i)];
                    //     }
                    //     buffer_index += sizeof(Struct_Referee_Tx_Data_Interaction_Radar_Send) + 7;
                    // }
                    // break;
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
    }
    else
    {
        //裁判系统保持连接
        Referee_Status = Referee_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief UART定时发送函数
 *
 * @param 
 */

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

