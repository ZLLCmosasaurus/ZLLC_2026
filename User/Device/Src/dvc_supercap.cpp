/**
 * @file dvc_supercap.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 超级电容
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "dvc_supercap.h"
#include "dvc_dwt.h"
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
// float Fps_Supercap_Rx,E_Sum;
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

        if (Totol_Energy <= 500.0f) // 损失5%的正常20000J功率
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
void Class_Supercap::Get_Referee_MaxPower()
{
    if (Robot_Power_Status == 1)
    {
        Referee_MaxPower = 35;
        return;
    }
#ifdef Forward_Power
    switch (Referee->Get_Level())
    {
    case 1:
        Referee_MaxPower = 60.f;
        break;
    case 2:
        Referee_MaxPower = 65.f;
        break;
    case 3:
        Referee_MaxPower = 70.f;
        break;
    case 4:
        Referee_MaxPower = 75.f;
        break;
    case 5:
        Referee_MaxPower = 80.f;
        break;
    case 6:
        Referee_MaxPower = 85.f;
        break;
    case 7:
        Referee_MaxPower = 90.f;
        break;
    case 8:
        Referee_MaxPower = 95.f;
        break;
    case 9:
        Referee_MaxPower = 100.f;
        break;
    case 10:
        Referee_MaxPower = 100.f;
        break;
    default:
        Referee_MaxPower = 60.f; // 默认等级1的功率
        break;
    }
#endif

#ifdef HP_Power // 血量优先底盘
    switch (Referee->Get_Level())
    {
    case 1:
        Referee_MaxPower = 45.f;
        break;
    case 2:
        Referee_MaxPower = 50.f;
        break;
    case 3:
        Referee_MaxPower = 55.f;
        break;
    case 4:
        Referee_MaxPower = 60.f;
        break;
    case 5:
        Referee_MaxPower = 65.f;
        break;
    case 6:
        Referee_MaxPower = 70.f;
        break;
    case 7:
        Referee_MaxPower = 75.f;
        break;
    case 8:
        Referee_MaxPower = 80.f;
        break;
    case 9:
        Referee_MaxPower = 90.f;
        break;
    case 10:
        Referee_MaxPower = 100.f; // 或 100W？请根据实际表格
        break;
    default:
        Referee_MaxPower = 45.f;
        break;
    }
#endif
}
/**
 * @brief
 *
 */
void Class_Supercap::Output()
{
    float energyBuffer = 0.0f, normalized = 0.0f, bufferPower = 0.0f;
    
    Get_Referee_MaxPower();

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

            Chassis_Device_LimitPower = Referee_MaxPower + bufferPower;
            Limit_Power = Referee_MaxPower + bufferPower;
        }
        break;
        case Supercap_Usage_Stratage_Supercap_BufferPower:
        {
            if (Data.Supercap_Buffer_Power >= 50.0f)
                Chassis_Device_LimitPower = Referee_MaxPower + Data.Supercap_Buffer_Power * 0.7f;
            else
                Chassis_Device_LimitPower = Referee_MaxPower;

            Limit_Power = Referee_MaxPower;
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

        Chassis_Device_LimitPower = Referee->Get_Chassis_Power_Max() + bufferPower;
        Limit_Power = Referee->Get_Chassis_Power_Max() + bufferPower;
    }
}

/**
 * @brief TIM定时器修改发送缓冲区
 *
 */
void Class_Supercap::TIM_Supercap_PeriodElapsedCallback()
{
    Output();

    Set_Limit_Power(Limit_Power);
    Set_Supercap_Control_Status(Supercap_Control_Status_ENABLE);
    memcpy(CAN_Tx_Data, &Supercap_Tx_Data, sizeof(Supercap_Tx_Data));
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
