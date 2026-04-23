/**
 * @file dvc_supercap.cpp
 * @author lez by yssickjgd
 * @brief 超级电容
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_supercap.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化超级电容通信, 切记__CAN_ID避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 收数据绑定的CAN ID
 * @param __Limit_Power_Max 最大限制功率, 0表示不限制
 */
void Class_Supercap::Init(CAN_HandleTypeDef *hcan, float __Limit_Power_Max)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    Supercap_Tx_Data.Limit_Power = __Limit_Power_Max;
    CAN_Tx_Data = CAN_Supercap_Tx_Data;
}
/**
 * @brief 初始化超级电容通信
 *
 * @param __huart 绑定的CAN总线
 * @param __fame_header 收数据绑定的帧头
 * @param __fame_tail 收数据绑定的帧尾
 * @param __Limit_Power_Max 最大限制功率, 0表示不限制
 */
void Class_Supercap::Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header, uint8_t __fame_tail, float __Limit_Power_Max)
{
    if (__huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (__huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (__huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (__huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (__huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (__huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    Supercap_Status = Disconnected;
    Supercap_Tx_Data.Limit_Power = __Limit_Power_Max;
    UART_Manage_Object->UART_Handler = __huart;
    Fame_Header = __fame_header;
    Fame_Tail = __fame_tail;
}

/**
 * @brief
 *
 */
void Class_Supercap::Data_Process()
{
    int16_t temp_chassis_power;
    uint16_t temp_buffer_power;
    uint8_t temp_cap_percent;
    uint16_t temp_used_energy;
    // 数据处理过程
    if (CAN_Manage_Object->Rx_Buffer.Header.StdId == 0x67)
    {
        memcpy(&temp_chassis_power, CAN_Manage_Object->Rx_Buffer.Data, 2);
        memcpy(&temp_buffer_power, CAN_Manage_Object->Rx_Buffer.Data + 2, 2);
        memcpy(&temp_cap_percent, CAN_Manage_Object->Rx_Buffer.Data + 4, 1);

        Supercap_Data.Chassis_Power = temp_chassis_power / 10.0f;
        Supercap_Data.Buffer_Power = (float)temp_buffer_power / 100.0f;
        Supercap_Data.Cap_Percent = temp_cap_percent;
        Supercap_Data.Supercap_Status = static_cast<Enum_Supercap_Status>(CAN_Manage_Object->Rx_Buffer.Data[5]);
        Supercap_Status = Supercap_Data.Supercap_Status;
        // Supercap_Data.Used_Energy = CAN_Manage_Object->Rx_Buffer.Data[6];
        memcpy(&temp_used_energy, CAN_Manage_Object->Rx_Buffer.Data + 6, 2);
        Supercap_Data.Used_Energy = temp_used_energy;
    }

    if (CAN_Manage_Object->Rx_Buffer.Header.StdId == 0x55)
    {
        if (CAN_Manage_Object->Rx_Buffer.Data[0] == 1)
        {
            Supercap_Data.Supercap_Status = Low_Power_Warning;
        }
        memcpy(&Supercap_Data.Overload_Power, CAN_Manage_Object->Rx_Buffer.Data + 1, 4);
    }
}

/**
 * @brief
 *
 */
void Class_Supercap::Output()
{
    memcpy(CAN_Tx_Data, &Supercap_Tx_Data, sizeof(Struct_Supercap_Tx_Data));
}
/**
 * @brief
 *
 */
void Class_Supercap::Output_UART()
{
    // float now_power = Supercap_Data.Now_Power;

    // UART_Manage_Object->Tx_Buffer[0] = Fame_Header;
    // UART_Manage_Object->Tx_Buffer[1] = 13;

    // UART_Manage_Object->Tx_Buffer[2] = (uint8_t)(Limit_Power_Max/100)%10;
    // UART_Manage_Object->Tx_Buffer[3] = (uint8_t)(Limit_Power_Max/10)%10;
    // UART_Manage_Object->Tx_Buffer[4] = (uint8_t)Limit_Power_Max%10;

    // UART_Manage_Object->Tx_Buffer[5] = (uint8_t)(now_power/100)%10;
    // UART_Manage_Object->Tx_Buffer[6] = (uint8_t)(now_power/10)%10;
    // UART_Manage_Object->Tx_Buffer[7] = (uint8_t)now_power%10;
    // UART_Manage_Object->Tx_Buffer[8] = (uint8_t)(now_power*10)%10;

    // UART_Manage_Object->Tx_Buffer[9] = Fame_Tail;
}

/**
 * @brief
 *
 */
void Class_Supercap::Data_Process_UART()
{
    // 数据处理过程
    if (UART_Manage_Object->Rx_Buffer[0] != '*' || UART_Manage_Object->Rx_Buffer[1] != 12 || UART_Manage_Object->Rx_Buffer[10] != ';')
        return;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Supercap::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag++;
    Data_Process();
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Supercap::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag++;
    Data_Process_UART();
}

/**
 * @brief TIM定时器中断定期检测超级电容是否存活
 *
 */
void Class_Supercap::TIM_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过超级电容数据
    if (Flag == Pre_Flag)
    {
        // 超级电容断开连接
        Supercap_Status = Disconnected;
    }

    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器修改发送缓冲区
 *
 */
void Class_Supercap::TIM_UART_Tx_PeriodElapsedCallback()
{
    Output_UART();
}

/**
 * @brief TIM定时器修改发送缓冲区
 *
 */
void Class_Supercap::TIM_Supercap_PeriodElapsedCallback()
{
    Output();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
