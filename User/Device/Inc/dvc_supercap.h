/**
 * @file dvc_supercap.h
 * @author lez by yssickjgd
 * @brief 超级电容
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_SUPERCAP_H
#define DVC_SUPERCAP_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "dvc_minipc.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
extern float power;
/**
 * @brief 超级电容状态
 *
 */
enum Enum_Supercap_Status
{
    Supercap_Status_DISABLE = 0,
    Supercap_Status_ENABLE,
};

/**
 * @brief 控制集状态
 *
 */
enum Enum_Control_Status : uint8_t
{
    Control_Status_NORMAL = 1,
    Control_Status_STANDBY,
    Control_Status_LISTEN,
};

/**
 * @brief 报警状态
 *
 */
enum Enum_Warning_Status : uint8_t
{
    Warning_Status_OFF = 0,
    Warning_Status_ON,
};
/**
 * @brief 超电工作状态
 *
 */
enum Enum_Working_Status : uint8_t
{
    Working_Status_ON = 0,
    Working_Status_OFF,
};

/**
 * @brief 超级电容源数据
 *
 */
struct Struct_Supercap_CAN_Data
{
    float Supercap_Voltage;
    float Stored_Energy;
} __attribute__((packed));

/**
 * @brief 超级电容发送的数据
 *
 */
struct Struct_Supercap_Tx_Data
{
    float Limit_Power;
    Enum_Working_Status Working_Status;
}__attribute__((packed));

struct Supercap_Rx_Data_A
{
    int16_t Chassis_Power;
    uint16_t Buffer_Power;
    uint8_t Cap_Proportion;
    Enum_Control_Status Control_Status;
    uint16_t Consuming_Power_Now;
}__attribute__((packed));

struct Supercap_Rx_Data_B
{
    Enum_Warning_Status Warning_Status;
    float Overload_Power;
}__attribute__((packed));

/**
 * @brief Specialized, 超级电容
 * 
 */
class Class_Supercap
{
public:
    void Init(FDCAN_HandleTypeDef *__hcan, float __Limit_Power_Max = 45);
    void Init_UART(UART_HandleTypeDef *__huart, uint8_t __fame_header = '*', uint8_t __fame_tail = ';', float __Limit_Power_Max = 45.0f);

    inline Enum_Supercap_Status Get_Supercap_Status();
    inline float Get_Stored_Energy();
    inline float Get_Now_Voltage();
    inline float Get_Chassis_Power();
    inline Enum_Control_Status Get_Control_Status();
    inline Enum_Warning_Status Get_Warning_Status();
    inline Enum_Supercap_Mode Get_Supercap_Mode();
    inline uint16_t Get_Buffer_Power();
    inline uint8_t Get_Supercap_Proportion();
    inline uint16_t Get_Consuming_Power_Now();
    inline float Get_Consuming_Power();
    

    inline void Set_Limit_Power(float __Limit_Power);
    inline void Set_Now_Power(float __Now_Power);
    inline void Set_Working_Status(Enum_Working_Status __Working_Status);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void UART_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_UART_Tx_PeriodElapsedCallback();
    void TIM_Supercap_PeriodElapsedCallback();

    //裁判系统
    Class_Referee *Referee;

protected:
    //初始化相关常量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 切记避开0x201~0x20b, 默认收包CAN1的0x210, 滤波器最后一个, 发包CAN1的0x220
    uint16_t CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;

    //串口模式
    Struct_UART_Manage_Object *UART_Manage_Object;
    uint8_t Fame_Header;
    uint8_t Fame_Tail;
    //常量

    //内部变量
    float Consuming_Power = 20000.f;
    float Chassis_Power;
    //当前时刻的超级电容接收flag
    uint32_t Flag = 0;
    //前一时刻的超级电容接收flag
    uint32_t Pre_Flag = 0;

    //读变量
    Supercap_Rx_Data_A CAN_Supercap_Rx_Data_Normal;
    Supercap_Rx_Data_B CAN_Supercap_Rx_Data_Error;
    //超级电容状态
    Enum_Supercap_Status Supercap_Status = Supercap_Status_DISABLE;
    Enum_Supercap_Mode Supercap_Mode = Supercap_DISABLE;
    //超级电容对外接口信息
    Struct_Supercap_CAN_Data Supercap_Data;

    //写变量
    Struct_Supercap_Tx_Data Supercap_Tx_Data;

    //写变量

    //限制的功率
    float Limit_Power = 0.0f;

    //读写变量

    //内部函数

    void Data_Process();
    void Output();

    void Data_Process_UART();
    void Output_UART();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取超级电容状态
 * 
 * @return Enum_Supercap_Status 超级电容状态
 */
Enum_Supercap_Status Class_Supercap::Get_Supercap_Status()
{
    return(Supercap_Status);
}
/**
 * @brief 获取控制级状态
 * 
 * @return Enum_Supercap_Status 控制级状态
 */
Enum_Control_Status Class_Supercap::Get_Control_Status()
{
    return(CAN_Supercap_Rx_Data_Normal.Control_Status);
}
/**
 * @brief 获取报错状态
 * 
 * @return Enum_Supercap_Status 报错状态
 */
Enum_Warning_Status Class_Supercap::Get_Warning_Status()
{
    return(CAN_Supercap_Rx_Data_Error.Warning_Status);
}

/**
 * @brief 获取存储的能量
 *
 * @return float 存储的能量
 */
float Class_Supercap::Get_Stored_Energy()
{
    return (Supercap_Data.Stored_Energy);
}

float Class_Supercap::Get_Chassis_Power()
{
    return (Chassis_Power);
}

// /**
//  * @brief 获取输出的功率
//  *
//  * @return float 输出的功率
//  */
// float Class_Supercap::Get_Now_Power()
// {
//     return (Supercap_Data.Now_Power);
// }

/**
 * @brief 获取当前的电压
 *
 * @return float 当前的电压
 */
float Class_Supercap::Get_Now_Voltage()
{
    return (Supercap_Data.Supercap_Voltage);
}
float Class_Supercap::Get_Consuming_Power()
{
    return (Consuming_Power);
}
Enum_Supercap_Mode Class_Supercap::Get_Supercap_Mode()
{
    return(Supercap_Mode);
}
uint16_t Class_Supercap::Get_Buffer_Power()
{
    return(CAN_Supercap_Rx_Data_Normal.Buffer_Power);
}
uint8_t Class_Supercap::Get_Supercap_Proportion()
{
    return(CAN_Supercap_Rx_Data_Normal.Cap_Proportion);
}
uint16_t Class_Supercap::Get_Consuming_Power_Now()
{
    return(CAN_Supercap_Rx_Data_Normal.Consuming_Power_Now);
}
/**
 * @brief 设定绝对最大限制功率
 *
 * @param __Limit_Power 绝对最大限制功率
 */
void Class_Supercap::Set_Limit_Power(float __Limit_Power)
{
    Supercap_Tx_Data.Limit_Power = __Limit_Power;
}
void Class_Supercap::Set_Working_Status(Enum_Working_Status __Working_Status)
{
    Supercap_Tx_Data.Working_Status = __Working_Status;
}
void Class_Supercap::Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode)
{
    Supercap_Mode = __Supercap_Mode;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
