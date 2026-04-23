/**
 * @file dvc_manifold.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 妙算视觉
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DVC_MANIFOLD_H
#define DVC_MANIFOLD_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 妙算状态
 *
 */
enum Enum_Manifold_Status
{
    Manifold_Status_DISABLE = 0,
    Manifold_Status_ENABLE,
};

/**
 * @brief 妙算源数据
 *
 */
struct Struct_Manifold_UART_Data
{
    uint16_t Frame_Header;
    int16_t Delta_Yaw;
    int16_t Delta_Pitch;
    int16_t Predicted_Delta_Yaw;
    int16_t Predicted_Delta_Pitch;
    int16_t Distance;
    int16_t Predicted_Velocity_Yaw;
    int16_t Predicted_Velocity_Pitch;
    uint16_t Data_Flag;
} __attribute__((packed));

/**
 * @brief 妙算经过处理的数据, 均为国际单位制
 *
 */
struct Struct_Manifold_Data
{
    uint16_t Data_Flag;
    float Delta_Yaw;
    float Delta_Pitch;
    float Predicted_Delta_Yaw;
    float Predicted_Delta_Pitch;
    float Predicted_Velocity_Yaw;
    float Predicted_Velocity_Pitch;
    float Distance;
};

/**
 * @brief Specialized, 单独自瞄接口
 *
 */
class Class_Manifold
{
public:
    void Init(UART_HandleTypeDef *__huart, uint16_t __Frame_Header = 0xabcd);

    inline Enum_Manifold_Status Get_Manifold_Status();
    inline uint16_t Get_Data_Flag();
    inline float Get_Delta_Yaw();
    inline float Get_Delta_Pitch();
    inline float Get_Predicted_Delta_Yaw();
    inline float Get_Predicted_Delta_Pitch();
    inline float Get_Predicted_Velocity_Yaw();
    inline float Get_Predicted_Velocity_Pitch();
    inline float Get_Distance();

    void UART_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    //数据包头标
    uint16_t Frame_Header;

    //常量

    //内部变量

    //当前时刻的妙算接收flag
    uint32_t Flag = 0;
    //前一时刻的妙算接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //视觉状态
    Enum_Manifold_Status Manifold_Status = Manifold_Status_DISABLE;
    //妙算对外接口信息
    Struct_Manifold_Data Data;

    //写变量

    //读写变量

    //内部函数

    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief
 *
 * @return Enum_Manifold_Status
 */
Enum_Manifold_Status Class_Manifold::Get_Manifold_Status()
{
    return (Manifold_Status);
}

/**
 * @brief 获取标志位
 *
 * @return float 距离
 */
uint16_t Class_Manifold::Get_Data_Flag()
{
    return (Data.Data_Flag);
}

/**
 * @brief 获取航向偏差
 *
 * @return float 航向偏差
 */
float Class_Manifold::Get_Delta_Yaw()
{
    return (Data.Delta_Yaw);
}

/**
 * @brief 获取俯仰偏差
 *
 * @return float 俯仰偏差
 */
float Class_Manifold::Get_Delta_Pitch()
{
    return (Data.Delta_Pitch);
}

/**
 * @brief 获取预测航向偏差
 *
 * @return float 预测航向偏差
 */
float Class_Manifold::Get_Predicted_Delta_Yaw()
{
    return (Data.Predicted_Delta_Yaw);
}

/**
 * @brief 获取预测俯仰偏差
 *
 * @return float 预测俯仰偏差
 */
float Class_Manifold::Get_Predicted_Delta_Pitch()
{
    return (Data.Predicted_Delta_Pitch);
}

/**
 * @brief 获取预测航向速度
 *
 * @return float 预测航向速度
 */
float Class_Manifold::Get_Predicted_Velocity_Yaw()
{
    return (Data.Predicted_Velocity_Yaw);
}

/**
 * @brief 获取预测俯仰速度
 *
 * @return float 预测俯仰速度
 */
float Class_Manifold::Get_Predicted_Velocity_Pitch()
{
    return (Data.Predicted_Velocity_Pitch);
}

/**
 * @brief 获取距离
 *
 * @return float 距离
 */
float Class_Manifold::Get_Distance()
{
    return (Data.Distance);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
