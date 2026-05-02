/**
 * @file dvc_supercap.cpp
 * @author cjw by yssickjgd
 * @brief 超级电容
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_supercap.h"
#include "dvc_referee.h"
#include "Config.h"

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
void Class_Supercap::Init(FDCAN_HandleTypeDef *hcan, float __Limit_Power_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    Supercap_Tx_Data.Limit_Power = __Limit_Power_Max;
    CAN_Tx_Data = CAN_Supercap_Tx_Data;

    fsm.Init(2, 0);
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
    else if (__huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }
    Supercap_Status = Supercap_Status_DISABLE;
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
    // 数据处理过程
    memcpy(&Supercap_Data, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Struct_Supercap_CAN_Data));

    Data.Chassis_Actual_Power = (float)Supercap_Data.Chassis_Actual_Power / 10.0f;
    Data.Supercap_Buffer_Power = (float)Supercap_Data.Supercap_Buffer_Power / 100.0f;
    Data.Supercap_Charge_Percentage = (float)Supercap_Data.Supercap_Charge_Percentage;
    Data.Supercup_Control_Level_Status = Supercap_Data.Supercup_Control_Level_Status;
    Data.Supercap_Current_Energy_Consumption = (float)Supercap_Data.Supercap_Current_Energy_Consumption / 10000.0f;

    if (Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        Totol_Energy -= Data.Supercap_Current_Energy_Consumption;

        if (Totol_Energy < 0)
            Totol_Energy = 0;

        if (Totol_Energy <= 1000.0f) // 损失5%的正常20000J功率
        {
            Robot_Power_Status = 1;
        }
        else
        {
            Robot_Power_Status = 0;
        }
    }
    else
    {
        Totol_Energy = 20000.0f;
        Robot_Power_Status = 0;
    }
}
#ifdef OLD
void Class_Supercap::Data_Process()
{
    // 数据处理过程
    switch (CAN_Manage_Object->Rx_Buffer.Header.Identifier)
    {
    case (0x67):
    {
        memcpy(&CAN_Supercap_Rx_Data_Normal, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Supercap_Rx_Data_A));
        Chassis_Power = CAN_Supercap_Rx_Data_Normal.Chassis_Power / 10.f;
        if (Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
        {
            Consuming_Power -= (float)(Get_Consuming_Power_Now() / 10000);
        }
        break;
    }
    case (0x55):
    {
        memcpy(&CAN_Supercap_Rx_Data_Error, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Supercap_Rx_Data_B));
        break;
    }
    }
}
#endif
/**
 * @brief
 *
 */
void Class_Supercap::Output()
{
    switch (PowerLimit_Type)
    {
    case PowerLimit_Type_Referee_BufferPower:
    {
        Chassis_Device_LimitPower = Referee_BufferPower_Output + Referee_MaxPower;
        Limit_Power = Referee_BufferPower_Output + Referee_MaxPower;
    }
    break;
    case PowerLimit_Type_Supercap_BufferPower:
    {
        Chassis_Device_LimitPower = Supercap_BufferPower_Output + Referee_MaxPower + Supercap_LimitBufferPower_Output;
        Limit_Power = Supercap_LimitBufferPower_Output + Referee_MaxPower;
    }
    break;
    }

    if (Chassis_Device_LimitPower < 0.0f)
    {
        Chassis_Device_LimitPower = 0.0f;
    }
    if (Limit_Power < 0.0f)
    {
        Limit_Power = 0.0f;
    }
    // 给超电can发送打包
    Set_Supercap_Control_Status((Enum_Supercap_Control_Status)SuperCap);
    Set_Limit_Power(Limit_Power);
    memcpy(CAN_Tx_Data, &Supercap_Tx_Data, sizeof(Struct_Supercap_Tx_Data));
}

void Class_Supercap::Use_SuperCap_Strategy()
{
    float energyBuffer = 0.0f, normalized = 0.0f, bufferPower = 0.0f;
    // 超级电容策略 超电一直处于使能状态
    // 1.使用裁判系统缓冲环（斜率不易过大 功率控制讲究细水长流）（如果软件功率限制不住，莫慌，超电会帮你补一些）
    // 2.使用超级电容缓冲环并添加缓冲环充能策略（缓冲只加不减 ->充能）
    if (Get_Supercap_Status() == Supercap_Status_ENABLE)
    {
        switch (Supercap_Usage_Stratage)
        {
        case Supercap_Usage_Stratage_Referee_BufferPower:
        {
            energyBuffer = Referee->Get_Chassis_Energy_Buffer();
            // 归一化到[-1, 1]范围，中心点在30J
            normalized = (energyBuffer - 30.0f) / 30.0f;
            // 使用tanh实现平滑过渡，范围[-30, 30]
            bufferPower = 30.0f * tanhf(normalized);

            Limit_Power = Referee->Get_Chassis_Power_Max() + bufferPower;
            Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max() + bufferPower;
        }
        break;
        case Supercap_Usage_Stratage_Supercap_BufferPower:
        {
            Limit_Power = Referee->Get_Chassis_Power_Max();
            if (Data.Supercap_Buffer_Power > 100.0f)
            {
                Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max() + Get_Supercap_Buffer_Power() * 0.6f;
            }
            else if (Data.Supercap_Buffer_Power > 50.0f)
            {
                Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max() + Get_Supercap_Buffer_Power() * 0.3f;
            }
            else
            {
                Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max();
            }
        }
        break;
        }
    }
    else
    {
        energyBuffer = Referee->Get_Chassis_Energy_Buffer();
        // 归一化到[-1, 1]范围，中心点在40J
        normalized = (energyBuffer - 40.0f) / 20.0f;
        // 使用tanh实现平滑过渡，范围[-20, 20]
        bufferPower = 20.0f * tanhf(normalized);

        Limit_Power = Referee->Get_Chassis_Power_Max() + bufferPower;
        Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max() + bufferPower;
    }
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
    if (UART_Manage_Object->Rx_Buffer[0] != '*' && UART_Manage_Object->Rx_Buffer[1] != 12 && UART_Manage_Object->Rx_Buffer[10] != ';')
        return;
    else
    {
        // Supercap_Data.Stored_Energy = (float)(UART_Manage_Object->Rx_Buffer[4]/10.0f);
    }
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
    Data_Process();
    // Data_Process_UART();
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
        Supercap_Status = Supercap_Status_DISABLE;
    }
    else
    {
        // 超级电容保持连接
        Supercap_Status = Supercap_Status_ENABLE;
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
    Use_SuperCap_Strategy();

    Set_Limit_Power(Limit_Power);
    Set_Supercap_Control_Status(Supercap_Control_Status_ENABLE);
    memcpy(CAN_Tx_Data, &Supercap_Tx_Data, sizeof(Struct_Supercap_Tx_Data));
}

void Class_Supercap::TIM_Supercap_Strategy_PeriodElapsedCallback()
{
    Use_SuperCap_Strategy();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
