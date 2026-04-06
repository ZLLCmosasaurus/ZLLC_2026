/**
 * @file TOFSense.cpp
 * @author hsl
 * @brief Nooploop TOFSense lazer lidar
 * @version 0.1
 * @date 2024-12-18 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_TOFSense.h"

// 协议相关宏定义
#define TOFSENSE_FRAME_LEN 16
#define TOFSense_HEADER_0 0x57
#define TOFSense_HEADER_1 0x00

/**
 * @brief 解析24位有符号整型
 */
static int32_t ParseInt24(const uint8_t *data)
{
    // 左移到最高位再右移实现符号位自动扩展
    return (int32_t)(data[0] << 8 | data[1] << 16 | data[2] << 24) / 256;
}

/**
 * @brief 检查校验和
 */
uint8_t NPVerifyCheckSum(const void *data, size_t data_length)
{
    const uint8_t *byte = (uint8_t *)data;
    uint8_t sum = 0;
    for (size_t i = 0; i < data_length - 1; ++i)
    {
        sum += byte[i];
    }
    return sum == byte[data_length - 1];
}

/**
 * @brief 计算校验和
 */
uint8_t CalculateCheckSum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    // 累加除最后一位（校验位）之外的所有字节 
    for (uint16_t i = 0; i < len - 1; i++)
    {
        sum += data[i];
    }
    return sum;
}

void Class_TOFSense::Init(UART_HandleTypeDef *huart, uint8_t __dvc_id)
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
    else if (huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }

    dvc_id = __dvc_id;
}

void Class_TOFSense::TOFSense_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Length)
{ 
    // 遍历缓冲区寻找帧头
    for (uint16_t i = 0; i <= (Length - TOFSENSE_FRAME_LEN); i++)
    {
        if (Rx_Data[i] == TOFSense_HEADER_0 && Rx_Data[i + 1] == TOFSense_HEADER_1)
        {
            // 找到帧头后进行校验和验证
            if (NPVerifyCheckSum(&Rx_Data[i], TOFSENSE_FRAME_LEN))
            {
                // 将帧首地址传入解包函数
                Data_Process(&Rx_Data[i]);
                Flag++;

                break;
            }
        }
    }
}

void Class_TOFSense::TIM1msMod150_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        TOFSense_Communication_Status = TOFSense_Comm_OFFLINE;
    }
    else
    {
        TOFSense_Communication_Status = TOFSense_Comm_ONLINE;
    }

    Pre_Flag = Flag;
}

void Class_TOFSense::Data_Process(uint8_t* Frame_Ptr)
{
    // 解析测距模块ID
    TOFSense_Data.id = Frame_Ptr[3];
    
    // 解析测距模块系统时间
    memcpy(&(TOFSense_Data.system_time), &Frame_Ptr[4], 4);
    
    // 解析测得的距离 (24位并转换为米)
    int32_t raw_dis = ParseInt24(&Frame_Ptr[8]);
    TOFSense_Data.dis = raw_dis / 1000.0f;
    
    // 解析距离是否有效 (0为有效)
    TOFSense_Data.dis_status = Frame_Ptr[11];
    
    // 解析信号强度
    memcpy(&(TOFSense_Data.signal_strength), &Frame_Ptr[12], 2);
    
    // 解析距离范围/重复精度
    TOFSense_Data.range_precision = Frame_Ptr[14];
}

// 串口模式查询帧，主动模式下无需使用
void Class_TOFSense::TIM_UART_Tx_PeriodElapsedCallback()
{
    uint8_t read_frame[8] = {0x57, 0x10, 0x00, 0x00, dvc_id, 0x00, 0x00, 0x00};
    read_frame[7] = CalculateCheckSum(read_frame, 8);

    HAL_UART_Transmit_DMA(UART_Manage_Object->UART_Handler, read_frame, 8);
}
/************************ COPYRIGHT(C) NEUQ-ZLLC **************************/