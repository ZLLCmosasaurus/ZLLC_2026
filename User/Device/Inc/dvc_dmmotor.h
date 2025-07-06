/**
 * @file dvc_dmmotor.h
 * @author cjw by yssickjgd
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
enum Enum_DM_Motor_Status
{
    DM_Motor_Status_DISABLE = 0,
    DM_Motor_Status_ENABLE,
};

/**
 * @brief 达妙电机的ID枚举类型
 *
 */
enum Enum_DM_Motor_ID : uint8_t
{
    DM_Motor_ID_UNDEFINED = 0,
    DM_Motor_ID_0xA1,
    DM_Motor_ID_0xA2,
    DM_Motor_ID_0xA3,
    DM_Motor_ID_0xA4,
    DM_Motor_ID_0xA5,
    DM_Motor_ID_0xA6,
    DM_Motor_ID_0xA7,
    DM_Motor_ID_0xA8,
};

/**
 * @brief 达妙电机控制状态
 *
 */
enum Enum_DM_Motor_Control_Status
{
    DM_Motor_Control_Status_DISABLE = 0,
    DM_Motor_Control_Status_ENABLE,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_DM_Motor_Control_Method
{
    DM_Motor_Control_Method_MIT_POSITION = 0,
    DM_Motor_Control_Method_MIT_OMEGA,
    DM_Motor_Control_Method_MIT_TORQUE,
    DM_Motor_Control_Method_POSITION_OMEGA,
    DM_Motor_Control_Method_OMEGA,
};

/**
 * @brief 达妙电机源数据
 *
 */
struct Struct_DM_Motor_CAN_Rx_Data
{
    Enum_DM_Motor_ID CAN_ID;
    uint16_t Position_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 扭矩非国际单位制
 *
 */
struct Struct_DM_Motor_Rx_Data
{
    Enum_DM_Motor_ID CAN_ID;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint16_t Pre_Position;
    int32_t Total_Position;
    int32_t Total_Round;
};

/**
 * @brief J4310无刷电机, 单片机控制输出控制帧
 * DM_Motor_Control_Method_POSITION_OMEGA模式下, 需调参助手辅助设置位置环PI参数, 空载250与0
 * 
 * PMAX值需在调参助手设置为3.141593, 即PI, 此时可在MIT模式下当舵机使用
 *
 */
class Class_DM_Motor_J4310
{
public:

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method = DM_Motor_Control_Method_MIT_POSITION, int32_t __Position_Offset = 0, float __Omega_Max = 20.94359f, float __Torque_Max = 10.0f);

    inline Enum_DM_Motor_Control_Status Get_DM_Motor_Control_Status();
    inline Enum_DM_Motor_Status Get_DM_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_MOS_Temperature();
    inline float Get_Now_Rotor_Temperature();
    inline Enum_DM_Motor_Control_Method Get_Control_Method();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    
    inline void Set_DM_Control_Status(Enum_DM_Motor_Control_Status __DM_Motor_Control_Status);
    inline void Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, 控制帧是0xxa1~0xxaf
    Enum_DM_Motor_ID CAN_ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //位置反馈偏移
    uint32_t Position_Offset;
    //最大速度, 调参助手设置, 推荐20.94359, 也就是最大转速200rpm
    float Omega_Max;
    //最大扭矩, 调参助手设置, 推荐7, 也就是最大输出7NM
    float Torque_Max;

    //常量
    
    //一圈位置刻度
    uint32_t Position_Max = 65536;

    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //电机状态
    Enum_DM_Motor_Status DM_Motor_Status = DM_Motor_Status_DISABLE;
    //电机对外接口信息
    Struct_DM_Motor_Rx_Data Data;

    //写变量

    //读写变量

    //电机控制状态
    Enum_DM_Motor_Control_Status DM_Motor_Control_Status = DM_Motor_Control_Status_DISABLE;
    //电机控制方式
    Enum_DM_Motor_Control_Method DM_Motor_Control_Method = DM_Motor_Control_Method_MIT_POSITION;
    //MIT的Kp值, 0~500, 空载6, 位置控制需要
    float MIT_K_P = 0.0f;
    //MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
    float MIT_K_D = 0.0f;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的扭矩
    float Target_Torque = 0.0f;

    //内部函数

    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_DM_Motor_Status 电机状态
 */
Enum_DM_Motor_Status Class_DM_Motor_J4310::Get_DM_Motor_Status()
{
    return (DM_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DM_Motor_J4310::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DM_Motor_J4310::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return float 当前的扭矩, 直接采用反馈值
 */
float Class_DM_Motor_J4310::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前MOS管的温度, 开氏度
 *
 * @return float 当前MOS管的温度, 开氏度
 */
float Class_DM_Motor_J4310::Get_Now_MOS_Temperature()
{
    return (Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前绕组的温度, 开氏度
 *
 * @return float 当前绕组的温度, 开氏度
 */
float Class_DM_Motor_J4310::Get_Now_Rotor_Temperature()
{
    return (Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DM_Motor_Control_Method 电机控制方式
 */
Enum_DM_Motor_Control_Method Class_DM_Motor_J4310::Get_Control_Method()
{
    return (DM_Motor_Control_Method);
}

/**
 * @brief 获取MIT的Kp值, 0~500
 *
 * @return float MIT的Kp值, 0~500
 */
float Class_DM_Motor_J4310::Get_MIT_K_P()
{
    return (MIT_K_P);
}

/**
 * @brief 获取MIT的Kd值, 0~5
 *
 * @return float MIT的Kd值, 0~5
 */
float Class_DM_Motor_J4310::Get_MIT_K_D()
{
    return (MIT_K_D);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DM_Motor_J4310::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DM_Motor_J4310::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩
 *
 * @return float 目标的扭矩
 */
float Class_DM_Motor_J4310::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 设定电机控制状态
 *
 * @param __DM_Motor_Control_Status 电机控制状态
 */
void Class_DM_Motor_J4310::Set_DM_Control_Status(Enum_DM_Motor_Control_Status __DM_Motor_Control_Status)
{
    DM_Motor_Control_Status = __DM_Motor_Control_Status;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
void Class_DM_Motor_J4310::Set_DM_Motor_Control_Method(Enum_DM_Motor_Control_Method __Control_Method)
{
    DM_Motor_Control_Method = __Control_Method;
}

/**
 * @brief 设定MIT的Kp值, 0~500, 空载6, 位置控制需要
 *
 * @param __MIT_K_P MIT的Kp值, 0~500, 空载6, 位置控制需要
 */
void Class_DM_Motor_J4310::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

/**
 * @brief 设定MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 *
 * @param __MIT_K_D MIT的Kd值, 0~5, 空载0.2, 位置和速度控制需要
 */
void Class_DM_Motor_J4310::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DM_Motor_J4310::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DM_Motor_J4310::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩
 *
 * @param __Target_Torque 目标的扭矩
 */
void Class_DM_Motor_J4310::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
