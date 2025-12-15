/**
 * @file crt_gimbal.h
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"
#include "alg_fsm.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Class_Gimbal;

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
    Gimbal_Control_Type_YAW_CALIBRATION,
    Gimbal_Control_Type_PITCH_CALIBRATION,
};

/**
 * @brief Specialized, Yaw轴校准有限自动机
 *
 */
class Class_FSM_Yaw_Calibration : public Class_FSM
{
public:
    Class_Gimbal *Gimbal;

    float Torque_Threshold = 600.0f;
    float speed = 0.5f;

    float Angle_Left = 0.0f;
    float Angle_Right = 0.0f;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, Yaw轴校准有限自动机
 *
 */
class Class_FSM_Pitch_Calibration : public Class_FSM
{
public:
    Class_Gimbal *Gimbal;

    float Torque_Threshold = 600.0f;
    float speed = 0.5f;

    float Angle_Upside_L = 0.0f;
    float Angle_Downside_L = 0.0f;
    float Angle_Upside_R = 0.0f;
    float Angle_Downside_R = 0.0f;

    int Up_Flag_L = 0;
    int Down_Flag_L = 0;
    int Up_Flag_R = 0;
    int Down_Flag_R = 0;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, 云台类
 *
 */
class Class_Gimbal
{
public:

    //imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    Class_FSM_Yaw_Calibration FSM_Yaw_Calibration;
    Class_FSM_Pitch_Calibration FSM_Pitch_Calibration;

    friend class Class_FSM_Yaw_Calibration;
    friend class Class_FSM_Pitch_Calibration;

    Class_DJI_Motor_C610 Motor_Pitch_L;
    Class_DJI_Motor_C610 Motor_Pitch_R;
    Class_DJI_Motor_C610 Motor_Yaw;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();
    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);


    void TIM_Calculate_PeriodElapsedCallback();
    float Calculate_Linear(float now_enc, float up_enc, float down_enc);

protected:
    //初始化相关常量
    float Gimbal_Head_Angle;
    //常量
    float CRUISE_SPEED_YAW = 100.f;
    float CRUISE_SPEED_PITCH = 70.f;
    // yaw轴最小值
    float Min_Yaw_Angle = - 10.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 10.0f;

    //yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = -25.0f;
    // pitch轴最大值
    float Max_Pitch_Angle = -10.0f ; //多10°

    //内部变量 


    //读变量

    //写变量

    //云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE ;

    //读写变量

    // yaw轴角度
    float Target_Yaw_Angle = 0.0f;

    // pitch轴角度
    float Target_Pitch_Angle = 0.0f;


    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}
/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}

/**
 * @brief 获取云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 获取云台控制类型
 */
Enum_Gimbal_Control_Type Class_Gimbal::Get_Gimbal_Control_Type()
{
    return (Gimbal_Control_Type);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}
/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}
/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
