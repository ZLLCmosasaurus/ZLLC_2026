/**
 * @file dvc_manifold.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 妙算视觉
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_manifold.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 妙算初始化
 *
 * @param huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Class_Manifold::Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header)
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
//    else if (huart->Instance == UART7)
//    {
//        UART_Manage_Object = &UART7_Manage_Object;
//    }
//    else if (huart->Instance == UART8)
//    {
//        UART_Manage_Object = &UART8_Manage_Object;
//    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Manifold::Data_Process()
{
    //数据处理过程
    Struct_Manifold_UART_Data *tmp_buffer = (Struct_Manifold_UART_Data *)UART_Manage_Object->Rx_Buffer;

    Data.Data_Flag = tmp_buffer->Data_Flag;
    Data.Delta_Yaw = tmp_buffer->Delta_Yaw / 18000.0f * PI;
    Data.Delta_Pitch = tmp_buffer->Delta_Pitch / 18000.0f * PI;
    Data.Predicted_Delta_Yaw = tmp_buffer->Predicted_Delta_Yaw / 18000.0f * PI;
    Data.Predicted_Delta_Pitch = tmp_buffer->Predicted_Delta_Pitch / 18000.0f * PI;
    Data.Predicted_Velocity_Yaw = tmp_buffer->Predicted_Velocity_Yaw / 18000.0f * PI;
    Data.Predicted_Velocity_Pitch = tmp_buffer->Predicted_Velocity_Pitch / 18000.0f * PI;
    Data.Distance = tmp_buffer->Distance / 1000.0f;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Manifold::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断妙算是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测妙算是否存活
 *
 */
void Class_Manifold::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过妙算数据
    if (Flag == Pre_Flag)
    {
        //妙算断开连接
        Manifold_Status = Manifold_Status_DISABLE;
    }
    else
    {
        //妙算保持连接
        Manifold_Status = Manifold_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
