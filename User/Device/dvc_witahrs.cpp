/**
 * @file dvc_witahrs.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief WIT姿态传感器
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_witahrs.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief WIT姿态传感器初始化
 * 
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Class_WIT::Init(UART_HandleTypeDef *huart, uint16_t __Frame_Header)
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
    Frame_Header = __Frame_Header;
}

/**
 * @brief 数据处理过程
 * 
 */
void Class_WIT::Data_Process()
{
    //数据处理过程
    Struct_WIT_UART_Data *tmp_buffer = (Struct_WIT_UART_Data *)UART_Manage_Object->Rx_Buffer;

    //未通过校验
    if(tmp_buffer->Frame_Header != Frame_Header || Math_Sum_8(UART_Manage_Object->Rx_Buffer, 10) != tmp_buffer->Checksum)
    {
        return;
    }

    switch (tmp_buffer->Data_Type)
    {
    case (WIT_Data_Type_TIME):
    {
        memcpy(&Data_Time, tmp_buffer->Data, 8);
    }
    break;
    case (WIT_Data_Type_ACCELERATE):
    {
        Data_Accelerate.Accelerate_X = (int16_t)(tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * 16.0f * 9.8f;

        Data_Accelerate.Accelerate_Y = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * 16.0f * 9.8f;

        Data_Accelerate.Accelerate_Z = (int16_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * 16.0f * 9.8f;

        Data_Other.Temperature = (int16_t)(tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f;
    }
    break;
    case (WIT_Data_Type_OMEGA):
    {
        Data_Omega.Omega_X = (int16_t)(tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * 2000.0f / 180.0f * PI;

        Data_Omega.Omega_Y = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * 2000.0f / 180.0f * PI;
        
        Data_Omega.Omega_Z = (int16_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * 2000.0f / 180.0f * PI;

        Data_Other.Voltage = (int16_t)(tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f;
    }
    break;
    case (WIT_Data_Type_ANGLE):
    {
        Data_Angle.Angle_Roll = (int16_t)(tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f * PI;

        Data_Angle.Angle_Pitch = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f * PI;

        Data_Angle.Angle_Yaw = (int16_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f * PI;

        Data_Other.Voltage = (int16_t)tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8;
    }
    break;
    case (WIT_Data_Type_MAGNETIC):
    {
        Data_Magnetic.Magnetic_X = (int16_t)tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8;

        Data_Magnetic.Magnetic_Y = (int16_t)tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8;

        Data_Magnetic.Magnetic_Z = (int16_t)tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8;

        Data_Other.Temperature = (int16_t)(tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f;
    }
    break;
    case (WIT_Data_Type_PORT):
    {
        //TODO 需适配不同设置情况
    }
    break;
    case (WIT_Data_Type_PRESSURE_ALTITUDE):
    {
        Data_Other.Pressure = (int32_t)tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8 | tmp_buffer->Data[2] << 16 | tmp_buffer->Data[3] << 24;

        Data_Coordinate.Altitude = (int32_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8 | tmp_buffer->Data[6] << 16 | tmp_buffer->Data[7] << 24) / 100.0f;
    }
    break;
    case (WIT_Data_Type_LONGITUDE_LATITUDE):
    {
        int32_t tmp_longitude, tmp_latitude;

        tmp_longitude = (int32_t)tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8 | tmp_buffer->Data[2] << 16 | tmp_buffer->Data[3] << 24;
        tmp_latitude = (int32_t)tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8 | tmp_buffer->Data[6] << 16 | tmp_buffer->Data[7] << 24;

        Data_Coordinate.Longitude = tmp_longitude / 10000000 + tmp_longitude % 10000000 / 100000.0f / 60.0f;

        Data_Coordinate.Latitude = tmp_latitude / 10000000 + tmp_latitude % 10000000 / 100000.0f / 60.0f;;
    }
    break;
    case (WIT_Data_Type_GROUND_SPEED):
    {
        Data_GPS.GPS_Altitude = (int16_t)(tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 10.0f;

        Data_GPS.GPS_Yaw = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 100.0f;

        Data_GPS.GPS_Velocity = (int32_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8 | tmp_buffer->Data[6] << 16 | tmp_buffer->Data[7] << 24) / 1000.0f / 3.6f;
    }
    break;
    case (WIT_Data_Type_QUATERNION):
    {
        Data_Quaternion.Q_0 = (int16_t)(tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8) / 32768.0f;
        
        Data_Quaternion.Q_1 = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 32768.0f;
        
        Data_Quaternion.Q_2 = (int16_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 32768.0f;
        
        Data_Quaternion.Q_3 = (int16_t)(tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 32768.0f;
    }
    break;
    case (WIT_Data_Type_GPS):
    {
        Data_GPS.Satellite_Number = (int16_t)tmp_buffer->Data[0] | tmp_buffer->Data[1] << 8;
        
        Data_GPS.GPS_Locational_Accuracy = (int16_t)(tmp_buffer->Data[2] | tmp_buffer->Data[3] << 8) / 100.0f;
        
        Data_GPS.GPS_Horizontal_Accuracy = (int16_t)(tmp_buffer->Data[4] | tmp_buffer->Data[5] << 8) / 100.0f;
        
        Data_GPS.GPS_Vertical_Accuracy = (int16_t)(tmp_buffer->Data[6] | tmp_buffer->Data[7] << 8) / 100.0f;
    }
    break;
    case (WIT_Data_Type_REGISTER):
    {
        //TODO 需适配不同设置情况
    }
    break;
    }
}

/**
 * @brief UART通信接收回调函数
 * 
 * @param Rx_Data 接收的数据
 */
void Class_WIT::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断陀螺仪是否在线
    Flag += 1;
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测WIT姿态传感器是否存活
 *
 */
void Class_WIT::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过陀螺仪数据
    if (Flag == Pre_Flag)
    {
        //陀螺仪断开连接
        WIT_Status = WIT_Status_DISABLE;
    }
    else
    {
        //陀螺仪保持连接
        WIT_Status = WIT_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
