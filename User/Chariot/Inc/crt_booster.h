/**
 * @file crt_booster.h
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
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
#include "dvc_minipc.h"

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
    Booster_Control_Type_MULTI,  //连发
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

enum Enum_Booster_User_Control_Type
{
    Booster_User_Control_Type_SINGLE=0,
    Booster_User_Control_Type_MULTI, // 连发
};


/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:
    //热量检测有限自动机
    Class_FSM_Heat_Detect FSM_Heat_Detect;
    friend class Class_FSM_Heat_Detect;

    //卡弹策略有限自动机
    Class_FSM_Antijamming FSM_Antijamming;
    friend class Class_FSM_Antijamming;

    //裁判系统
    Class_Referee *Referee;
    //上位机
    Class_MiniPC *MiniPC;

    //拨弹盘电机
    Class_DJI_Motor_C610 Motor_Driver;

    //摩擦轮电机左
    Class_DJI_Motor_C620 Motor_Friction_Left;
    //摩擦轮电机右
    Class_DJI_Motor_C620 Motor_Friction_Right;

    void Init();

    inline float Get_Default_Driver_Omega();
    inline float Get_Friction_Omega();
    inline float Get_Friction_Omega_Threshold();
    inline uint16_t Get_Heat();
    

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();
    inline Enum_Friction_Control_Type Get_Friction_Control_Type();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Friction_Control_Type(Enum_Friction_Control_Type __Friction_Control_Type);
    inline void Set_Friction_Omega(float __Friction_Omega);
    inline void Set_Driver_Omega(float __Driver_Omega);
    inline void Set_Booster_Type(Enum_Booster_Type __Booster_Type);
    inline void Set_Heat(uint16_t __Heat);
    inline void Set_Cooling_Value(uint16_t __Cooling_Value);

    void TIM_Calculate_PeriodElapsedCallback();
	void Output();
		
protected:
    //初始化相关常量

    //常量
    uint16_t Heat_Max = 400;
    uint16_t Cooling_Value = 80;
    float Heat_Consumption = 10.f;
    //拨弹盘堵转扭矩阈值, 超出被认为卡弹
    uint16_t Driver_Torque_Threshold = 8500;
    //摩擦轮单次判定发弹阈值, 超出被认为发射子弹
    uint16_t Friction_Torque_Threshold = 2000;
    //摩擦轮速度判定发弹阈值, 超出则说明已经开机
    float Friction_Omega_Threshold = 600;

    //内部变量
    uint16_t Heat;
    float shoot_time = 0.f;
    float ShootTime = 0.f;
    float shoot_speed = 0.f;
    float Now_Angle = 0.f;
    //读变量

    //拨弹盘默认速度, 一圈八发子弹, 此速度下与冷却均衡
    float Default_Driver_Omega = -2.0f * PI;

    //写变量

    //发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    Enum_Friction_Control_Type Friction_Control_Type = Friction_Control_Type_DISABLE;
    Enum_Booster_Type Booster_Type;
    //摩擦轮角速度
    float Friction_Omega = 650.0f;
    //拨弹盘实际的目标速度, 一圈八发子弹
    float Driver_Omega = -2.0f * PI * 2;
    //拨弹轮目标绝对角度 加圈数
    float Driver_Angle = 0.0f;
    //读写变量

    //内部函数

    
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

uint16_t Class_Booster::Get_Heat()
{
    return (Heat);
}
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
void Class_Booster::Set_Heat(uint16_t __Heat)
{
    Heat = __Heat;
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


void Class_Booster::Set_Booster_Type(Enum_Booster_Type __Booster_Type)
{
    Booster_Type = __Booster_Type;
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
void Class_Booster::Set_Cooling_Value(uint16_t __Cooling_Value)
{
    Cooling_Value = __Cooling_Value;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
