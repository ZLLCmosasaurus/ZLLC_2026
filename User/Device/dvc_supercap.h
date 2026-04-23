/**
 * @file dvc_supercap.h
 * @author lez by yssickjgd
 * @brief 超级电容
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DVC_SUPERCAP_H
#define DVC_SUPERCAP_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "drv_uart.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 超级电容状态
 *
 */
enum Enum_Supercap_Status : uint8_t
{
    Disconnected = 0,
    Normal = 1,
    Fault,
    Listen,
    Low_Power_Warning, // 低电压报警，电容电压低于4v
};

/**
 * @brief 超级电容源数据
 *
 */
struct Struct_Supercap_CAN_Data
{
    float Chassis_Power;                  // 底盘功率
    float Buffer_Power;                   // 缓冲功率
    float Cap_Percent;                    // 容量百分比
    Enum_Supercap_Status Supercap_Status; // 超级电容状态
    float Overload_Power;                 // 过载功率
    uint8_t Used_Energy;                  // 已使用能量
} __attribute__((packed));

/**
 * 超级电容工作模式
 */
enum Enum_Supercap_Mode : uint8_t
{
    Supercap_Mode_ENABLE = 0, // 正常启用超级电容
    Supercap_Mode_MONITOR = 1 // 关闭电容补偿，仅监听
};

/**
 * @brief 超级电容发送的数据
 *
 */
struct Struct_Supercap_Tx_Data
{
    float Limit_Power;
    Enum_Supercap_Mode Supercap_Mode;
} __attribute__((packed));

/**
 * @brief Specialized, 超级电容
 *
 */
class Class_Supercap
{
public:
    void Init(CAN_HandleTypeDef *__hcan, float __Limit_Power_Max = 45);
    void Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header = '*', uint8_t __fame_tail = ';', float __Limit_Power_Max = 45.0f);

    inline Enum_Supercap_Status Get_Supercap_Status();
    float Get_Chassis_Power();
    inline float Get_Stored_Energy();
    inline float Get_Now_Voltage();
    inline float Get_Buffer_Power();
    inline void Set_Limit_Power(float __Limit_Power);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Mode);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_UART_Tx_PeriodElapsedCallback();
    void TIM_Supercap_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 切记避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
    uint16_t CAN_ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;

    // 串口模式
    Struct_UART_Manage_Object *UART_Manage_Object;
    uint8_t Fame_Header;
    uint8_t Fame_Tail;
    // 常量

    // 内部变量

    // 当前时刻的超级电容接收flag
    uint32_t Flag = 0;
    // 前一时刻的超级电容接收flag
    uint32_t Pre_Flag = 0;

    // 读变量

    // 超级电容状态
    Enum_Supercap_Status Supercap_Status = Disconnected;
    // 超级电容对外接口信息
    Struct_Supercap_CAN_Data Supercap_Data;

    // 写变量
    Struct_Supercap_Tx_Data Supercap_Tx_Data;

    // 写变量
    float actual_power = 0.0f;
    // 限制的功率
    float Limit_Power = 0.0f;

    // 读写变量

    // 内部函数

    void Data_Process();
    void Output();

    void Data_Process_UART();
    void Output_UART();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 *
 */
inline void Class_Supercap::Set_Supercap_Mode(Enum_Supercap_Mode __Mode)
{
    Supercap_Tx_Data.Supercap_Mode = __Mode;
}
inline float Class_Supercap::Get_Buffer_Power()
{
    return (Supercap_Data.Buffer_Power);
}
/**
 * @brief 获取超级电容状态
 *
 * @return Enum_Supercap_Status 超级电容状态
 */
inline Enum_Supercap_Status
Class_Supercap::Get_Supercap_Status()
{
    return (Supercap_Status);
}

/**
 * @brief 获取输出的功率
 *
 * @return float 输出的功率
 */
inline float Class_Supercap::Get_Chassis_Power()
{
    return (Supercap_Data.Chassis_Power);
}

/**
 * @brief 获取当前的电压
 *
 * @return float 当前的电压百分比
 */
inline float Class_Supercap::Get_Now_Voltage()
{
    return (Supercap_Data.Cap_Percent);
}

/**
 * @brief 设定绝对最大限制功率
 *
 * @param __Limit_Power 绝对最大限制功率
 */
inline void Class_Supercap::Set_Limit_Power(float __Limit_Power)
{
    Supercap_Tx_Data.Limit_Power = __Limit_Power;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
