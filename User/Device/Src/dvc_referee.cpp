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
#include "dvc_dwt.h"
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
    else if(huart->Instance == USART10){
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
uint8_t Class_Referee::Verify_CRC_8(uint8_t *Message, uint32_t Length)
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
uint16_t Class_Referee::Verify_CRC_16(uint8_t *Message, uint32_t Length)
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


uint32_t last_cnt[5] = {0};
float Dt0[5] = {0};
    /**
 * @brief 数据处理过程, 为节约性能不作校验但提供了接口
 * 如遇到大规模丢包或错乱现象, 可重新启用校验过程
 *
 */
void Class_Referee::Data_Process(uint16_t Length)
{
    // 数据处理过程
    Struct_Referee_UART_Data *tmp_buffer;

    for (int i = 0; i < Length;)
    {
        tmp_buffer = (Struct_Referee_UART_Data *)&UART_Manage_Object->Rx_Buffer[i];

        // 未通过头校验
        if (tmp_buffer->Frame_Header != Frame_Header)
        {
            i++;
            continue;
        }
        // 未通过CRC8校验, 顺一位继续判断
        if (Verify_CRC_8((uint8_t *)tmp_buffer, 4) != tmp_buffer->CRC_8)
        {
            i++;
            continue;
        }
        // 未通过CRC16校验, 跨过当前包继续判断
        if (Verify_CRC_16((uint8_t *)tmp_buffer, 7 + tmp_buffer->Data_Length) != *(uint16_t *)((uint32_t)tmp_buffer + 7 + tmp_buffer->Data_Length))
        {
            i += 9 + tmp_buffer->Data_Length;
            continue;
        }
        // 通过校验但帧不够长
        if (i + 7 + tmp_buffer->Data_Length + 2 > Length)
        {
            break;
        }

        switch (tmp_buffer->Referee_Command_ID)
        {
        case Referee_Command_ID_GAME_STATUS:
        {
            Game_Status_Flag += 1;

            memcpy(&Game_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Status));
            Dt0[1] = DWT_GetDeltaT(&last_cnt[1]);
            break;
        }

        case (Referee_Command_ID_GAME_RESULT):
        {
            memcpy(&Game_Result, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Result));

            break;
        }
        case (Referee_Command_ID_GAME_ROBOT_HP):
        {
            memcpy(&Game_Robot_HP, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Robot_HP));

            break;
        }
        case (Referee_Command_ID_EVENT_DATA):
        {
            memcpy(&Event_Data, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Data));

            break;
        }
        case (Referee_Command_ID_EVENT_SUPPLY):
        {
            memcpy(&Event_Supply, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Supply));

            break;
        }
        case (Referee_Command_ID_EVENT_REFEREE_WARNING):
        {
            memcpy(&Event_Referee_Warning, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning));

            break;
        }
        case (Referee_Command_ID_ROBOT_STATUS):
        {
            memcpy(&Robot_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Status));
            Dt0[2] = DWT_GetDeltaT(&last_cnt[2]);
            break;
        }
        case (Referee_Command_ID_ROBOT_POWER_HEAT):
        {
            memcpy(&Robot_Power_Heat, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat));
            Dt0[0] = DWT_GetDeltaT(last_cnt);

            break;
        }
        case(Referee_Command_ID_EVENT_DART_REMAINING_TIME):{
            memcpy(&Event_Dart_Remaining_Time, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Dart_Remaining_Time));

            break;
        }
        case (Referee_Command_ID_ROBOT_POSITION):
        {
            memcpy(&Robot_Position, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Position));

            break;
        }
        case (Referee_Command_ID_ROBOT_BUFF):
        {
            memcpy(&Robot_Buff, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Buff));

            break;
        }
        case (Referee_Command_ID_ROBOT_DAMAGE):
        {
            memcpy(&Robot_Damage, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Damage));

            break;
        }
        case (Referee_Command_ID_ROBOT_BOOSTER):
        {
            memcpy(&Robot_Booster, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Booster));

            break;
        }
        case (Referee_Command_ID_ROBOT_REMAINING_AMMO):
        {
            memcpy(&Robot_Remaining_Ammo, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo));

            break;
        }
        case (Referee_Command_ID_ROBOT_RFID):
        {
            memcpy(&Robot_RFID, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_RFID));

            break;
        }
        case (Referee_Command_ID_ROBOT_DART_COMMAND):
        {
            memcpy(&Robot_Dart_Command, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command));

            break;
        }
        case(Referee_Command_ID_INTERACTION_MAP_COMMAND):{
            memcpy(&Robot_Map_Command_Data, tmp_buffer->Data, sizeof(Struct_Referee_Map_Command_Data));

            break;
        }
        case(Referee_Command_ID_Interaction_Robot_Receive):{
            memcpy(&Interaction_Robot_Receive, tmp_buffer->Data, sizeof(Struct_Referee_Tx_Data_Interaction_Robot_Receive));

            break;
        }
        case(Referee_Command_ID_ROBOT_Sentry_Info):{
            memcpy(&Sentry_Info_Bits, tmp_buffer->Data, sizeof(Struct_Sentry_Info_Bits_t));

            break;

        }
}
        // 缓冲区直接推移
        i += 7 + tmp_buffer->Data_Length + 2;
    

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

    Data_Process(Length);
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

void Class_Referee::TIM_Game_Status_Alive_PeriodElapsedCallback()
{
    if(Game_Status_Flag == Game_Status_Pre_Flag)
    {
        Game_Status_Online = 0;
    }
    else
    {
        Game_Status_Online = 1;
    }

    Game_Status_Pre_Flag = Game_Status_Flag;
}

/**
 * @brief UART定时发送函数，发送与雷达通信数据
 *
 * @param 
 */
void Class_Referee::TIM_UART_Tx_PeriodElapsedCallback()
{
    //雷达发送
    Sentry_To_Radar.Sender = Get_ID();
    if(Get_ID() == Referee_Data_Robots_ID_RED_SENTRY_7)
    {
        Sentry_To_Radar.Receiver = Referee_Data_Robots_ID_RED_RADAR_9;
    }
    else if (Get_ID() == Referee_Data_Robots_ID_BLUE_SENTRY_7)
    {
        Sentry_To_Radar.Receiver = Referee_Data_Robots_ID_BLUE_RADAR_9;
    }
    Referee_UI_Packed_Data(&Sentry_To_Radar);
    HAL_UART_Transmit_IT(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length);
}

/**
 * @brief UART定时发送函数，发送哨兵自主决策命令数据
 *
 * @param 
 */
void Class_Referee::Sentry_Auto_cmd_Transmit()
{
    //哨兵自主决策
    Sentry_cmd.Sender = Get_ID();
    Referee_UI_Packed_Data(&Sentry_cmd);
    HAL_UART_Transmit_IT(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length);
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