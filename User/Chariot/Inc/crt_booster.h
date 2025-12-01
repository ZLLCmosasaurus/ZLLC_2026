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
};

/**
 * @brief Specialized, 发射策略有限自动机
 *
 */
class Class_FSM_Shooting : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 发射策略有限自动机
 *
 */
class Class_FSM_Push_Calibration  : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold = 7000.0f;
    float Angle_Forward_L = 0.0f;
    float Angle_Backward_L = 0.0f;
    float Angle_Forward_R = 0.0f;
    float Angle_Backward_R = 0.0f;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:

    //发射有限自动机
    Class_FSM_Shooting FSM_Shooting;
    friend class Class_FSM_Shooting;

    //皮筋电机校准
    Class_FSM_Push_Calibration FSM_Push_Calibration;
    friend class Class_FSM_Push_Calibration;

    //裁判系统
    Class_Referee *Referee;
    //上位机
    Class_MiniPC *MiniPC;

    //发射电机
    Class_DJI_Motor_C610 Motor_Pull;

    Class_DJI_Motor_C620 Motor_Push_L;
    Class_DJI_Motor_C620 Motor_Push_R;

    void Init();


    inline int Get_Measured_Tension();
    inline Enum_Booster_Control_Type Get_Booster_Control_Type();

    inline void Set_Measured_Tension(int __Measured_Tension);
    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);

    void TIM_Calculate_PeriodElapsedCallback();
	void Output();
		
protected:
    //初始化相关常量

    //常量
    int Target_Tension = 0;

    //内部变量

    //读变量

    int Measured_Tension = 0;

    //写变量

    //发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_DISABLE;

    //读写变量

    //内部函数

    
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

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
 * @brief 获取拉力,
 *
 * @return int 获取拉力
 */
int Class_Booster::Get_Measured_Tension()
{
    return (Measured_Tension);
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
 * @brief 设定测量拉力
 *
 * @param __Measured_Tension 测量拉力
 */
void Class_Booster::Set_Measured_Tension(int __Measured_Tension)
{
    Measured_Tension = __Measured_Tension;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
