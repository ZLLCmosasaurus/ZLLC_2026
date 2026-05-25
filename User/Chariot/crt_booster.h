/**
 * @file crt_booster.h
 * @author lez by wanghongxi
 * @brief 发射机构
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/**
 * @brief 摩擦轮编号
 * 1 2
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Class_Booster;

/**
 * @brief 发射机构控制类型
 *
 */
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_CEASEFIRE,
    Booster_Control_Type_SINGLE,
    Booster_Control_Type_REPEATED,
    Booster_Control_Type_MULTI, // 连发
};

enum Enum_Booster_User_Control_Type
{
    Booster_User_Control_Type_DISABLE = 0,
    Booster_User_Control_Type_SINGLE,
    Booster_User_Control_Type_DUAL, // 连发
    Booster_User_Control_Type_REPEATED,
};

enum Enum_Friction_Stage
{
    Friction_Stage_UNREADY = 0, // 摩擦轮转速未达发射速度的稳态
    Friction_Stage_READY,       // 摩擦轮转速达到发射速度的稳态
};

enum Enum_Fire_Control_Type
{
    Fire_Control_Type_MANUL = 0,
    Fire_Control_Type_AUTO,
};


/**
 * @brief 摩擦轮控制类型
 *
 */
enum Enum_Friction_Control_Type
{
    Friction_Control_Type_DISABLE = 0,
    Friction_Control_Type_ENABLE,
};
/**
 * @brief 拨盘预置初始化有限自动机
 */
class Class_FSM_Driver_Init : public Class_FSM
{
public:
    Class_Booster *Booster;
    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 热量检测有限自动机
 *
 */
class Class_FSM_Heat_Detect : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Heat;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 卡弹策略有限自动机
 *
 */
class Class_FSM_Antijamming : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief 摩擦轮状态有限状态机
 */
class Class_FSM_Friction : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

enum Enum_Driver_Reset_Status
{
    Driver_Reset_Status_UNREADY = 0,
    Driver_Reset_Status_READTY,
};

class Class_Booster_Driver : public Class_DJI_Motor_C610
{
public:
    void TIM_PID_PeriodElapsedCallback();
    inline Enum_Driver_Reset_Status Get_Driver_Reset_Status();
    inline void Set_Driver_Reset_Status(Enum_Driver_Reset_Status __Driver_Reset_Status);

protected:
    Enum_Driver_Reset_Status Driver_Reset_Status = Driver_Reset_Status_UNREADY;
};

Enum_Driver_Reset_Status Class_Booster_Driver::Get_Driver_Reset_Status()
{
    return (Driver_Reset_Status);
}
void Class_Booster_Driver::Set_Driver_Reset_Status(Enum_Driver_Reset_Status __Driver_Reset_Status)
{
    Driver_Reset_Status = __Driver_Reset_Status;
}
/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:
    int FiredCounter = 0;
    int JammedCounter = 0;
    // 热量检测有限自动机
    Class_FSM_Heat_Detect FSM_Heat_Detect;
    friend class Class_FSM_Heat_Detect;

    // 卡弹策略有限自动机
    Class_FSM_Antijamming FSM_Antijamming;
    friend class Class_FSM_Antijamming;

    // 摩擦轮状态有限自动机
    Class_FSM_Friction FSM_Friction;
    friend class Class_FSM_Friction;

    // 拨盘预置初始化有限自动机
    Class_FSM_Driver_Init FSM_Driver_Init;
    friend class Class_FSM_Driver_Init;

    // 裁判系统
    Class_Referee *Referee;

    // 拨弹盘电机
    // Class_DJI_Motor_C610 Motor_Driver;
    Class_Booster_Driver Motor_Driver;
    // 摩擦轮电机左
    Class_DJI_Motor_C610 Motor_Friction_Left;
    // 摩擦轮电机右
    Class_DJI_Motor_C610 Motor_Friction_Right;
    // 摩擦轮电机下
    Class_DJI_Motor_C610 Motor_Friction_Down;

    void Init();

    inline float Get_Default_Driver_Omega();
    inline float Get_Friction_Omega();
    inline float Get_Friction_Omega_Threshold();
    inline float Get_Average_Torque();
    inline float Get_Average_Omega_Radian();

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();
    inline Enum_Friction_Control_Type Get_Friction_Control_Type();
    inline Enum_Friction_Stage Get_Friction_Stage();
    inline Enum_Fire_Control_Type Get_Fire_Control_Type();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Friction_Control_Type(Enum_Friction_Control_Type __Friction_Control_Type);
    inline void Set_Friction_Omega(float __Friction_Omega);
    inline void Set_Driver_Omega(float __Driver_Omega);
    inline void Set_Friction_Stage(Enum_Friction_Stage __Friction_Stage);
    inline void Set_Fire_Control_Type(Enum_Fire_Control_Type __Fire_Control_Type);

    void TIM_Calculate_PeriodElapsedCallback();
    void Output();

    Enum_Booster_User_Control_Type Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;

    float Heat_Local = 0.0f; // 本地累加热量
    float Heat_Max = 400.0f;
    float Cooling_Value = 10.0f; // 裁判冷却值

    // 收缩参数（可调）
    float Tau0 = 0.835f;         // 提前收缩时间
    float Tau1 = 0.14f;          // 收缩陡度
    float Recover_Ratio = 0.85f; // 恢复比例

    bool Overheat_Flag = false;

    // 射速相关
    float Base_Frequency = 15.0f; // f0
    float Balance_Frequency = 0.0f;

    float smax = 170.f;
    float cools = 7.0f;

    float tau = 0.0f;

protected:
    // 初始化相关常量

    // 常量

    // 拨弹盘堵转扭矩阈值, 超出被认为卡弹
    uint16_t Driver_Torque_Threshold = 10000;
    // 摩擦轮单次判定发弹阈值, 超出被认为发射子弹
    uint16_t Friction_Torque_Threshold = 5500;
    // 摩擦轮速度判定发弹阈值, 超出则说明已经开机
    float Friction_Omega_Threshold = 600;

    // 内部变量

    // 读变量

    // 拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
    float Default_Driver_Omega = -2.0f * PI;

    // 写变量

    // 发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    Enum_Friction_Control_Type Friction_Control_Type = Friction_Control_Type_DISABLE;
    Enum_Friction_Stage Friction_Stage = Friction_Stage_UNREADY;
    Enum_Fire_Control_Type Fire_Control_Type = Fire_Control_Type_AUTO;
    // 摩擦轮角速度
    float Friction_Omega = 1100.0f;

    // 拨弹盘实际的目标速度, 一圈九发子弹
    // float Driver_Omega = -2.0f * PI * 20.0f / 9.0f;
    float Driver_Omega = -2.0f * PI * 2.5f * 20.0f / 9.0f;
    // 拨弹轮目标绝对角度 加圈数
    float Driver_Angle = 0.0f;
    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
 *
 * @return float 拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
 */
float Class_Booster::Get_Default_Driver_Omega()
{
    return (Default_Driver_Omega);
}

/**
 * @brief 获取摩擦轮默认速度,
 *
 * @return float 获取摩擦轮默认速度
 */
float Class_Booster::Get_Friction_Omega()
{
    return (Friction_Omega);
}

/**
 * @brief 获取摩擦轮默认速度,
 *
 * @return float 获取摩擦轮默认速度
 */
float Class_Booster::Get_Friction_Omega_Threshold()
{
    return (Friction_Omega_Threshold);
}
/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
void Class_Booster::Set_Friction_Control_Type(Enum_Friction_Control_Type __Friction_Control_Type)
{
    Friction_Control_Type = __Friction_Control_Type;
}

/**
 * @brief 获得发射机构状态
 *
 * @return Enum_Booster_Control_Type 发射机构状态
 */
Enum_Booster_Control_Type Class_Booster::Get_Booster_Control_Type()
{
    return (Booster_Control_Type);
}

/**
 * @brief 获得发射机构状态
 *
 * @return Enum_Booster_Control_Type 发射机构状态
 */
Enum_Friction_Control_Type Class_Booster::Get_Friction_Control_Type()
{
    return (Friction_Control_Type);
}

/**
 * @brief 设定摩擦轮角速度
 *
 * @param __Friction_Omega 摩擦轮角速度
 */
void Class_Booster::Set_Friction_Omega(float __Friction_Omega)
{
    Friction_Omega = __Friction_Omega;
}

/**
 * @brief 设定拨弹盘实际的目标速度, 一圈八发子弹
 *
 * @param __Driver_Omega 拨弹盘实际的目标速度, 一圈八发子弹
 */
void Class_Booster::Set_Driver_Omega(float __Driver_Omega)
{
    Driver_Omega = __Driver_Omega;
}

/**
 * @brief 设定摩擦轮状态
 */
void Class_Booster::Set_Friction_Stage(Enum_Friction_Stage __Friction_Stage)
{
    Friction_Stage = __Friction_Stage;
}

/**
 * @brief 设定火控模式
 */
void Class_Booster::Set_Fire_Control_Type(Enum_Fire_Control_Type __Fire_Control_Type)
{
    Fire_Control_Type = __Fire_Control_Type;
}


/**
 * @brief 获取三个摩擦轮的平均反馈力矩
 */
float Class_Booster::Get_Average_Torque()
{
    return (Motor_Friction_Down.Get_Now_Torque() + Motor_Friction_Left.Get_Now_Torque() + Motor_Friction_Right.Get_Now_Torque()) / 3.0f;
}

/**
 * @brief 获取三个摩擦轮的平均实时转速
 */
float Class_Booster::Get_Average_Omega_Radian()
{
    return ((Motor_Friction_Down.Get_Now_Omega_Radian() + Motor_Friction_Left.Get_Now_Omega_Radian() + Motor_Friction_Right.Get_Now_Omega_Radian()) / 3.0f);
}

/**
 * @brief 获取摩擦轮状态
 */
Enum_Friction_Stage Class_Booster::Get_Friction_Stage()
{
    return (Friction_Stage);
}
/**
 * @brief 获取火控类型
 */
Enum_Fire_Control_Type Class_Booster::Get_Fire_Control_Type()
{
    return (Fire_Control_Type);
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
