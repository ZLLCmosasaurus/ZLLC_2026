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

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

enum Enum_Motor_Yaw_Type :uint8_t
{
    Yaw_A = 0,
    Yaw_B,
};

struct IMU_Data
{
    float Pitch;
    float Roll;
    float Yaw;
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief Specialized, yaw轴电机类
 *
 */
class Class_Gimbal_Yaw_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    //陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量

    //内部变量
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Yaw_Motor_GM6020::Get_Trer_Rad_Yaw()
{
    return (True_Rad_Yaw);
} 

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Gyro_Yaw()
{
    return (True_Gyro_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Angle_Yaw()
{
    return (True_Angle_Yaw);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    //陀螺仪获取云台角速度
    Class_IMU* IMU;


    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //常量
    

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    //内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}


/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_LK6010 : public Class_LK_Motor
{
public:
    //陀螺仪获取云台角速度
    Class_IMU* IMU;
    
    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    //初始化相关变量

    //常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    //内部变量 
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    //读变量

    //写变量

    //读写变量

    //内部函数
};

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);

}

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

    /*后期yaw pitch这两个类要换成其父类，大疆电机类*/

    // yaw轴电机
    Class_Gimbal_Yaw_Motor_GM6020 Motor_Yaw;

    // pitch轴电机
    Class_Gimbal_Pitch_Motor_GM6020 Motor_Pitch;


    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();
    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);


    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量
    float Gimbal_Head_Angle;
    //常量
    float CRUISE_SPEED_YAW = 100.f;
    float CRUISE_SPEED_PITCH = 70.f;
    // yaw轴最小值
    float Min_Yaw_Angle = - 180.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 180.0f;

    //yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = -15.0f;
    // pitch轴最大值
    float Max_Pitch_Angle = 25.0f ; //多10°

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
