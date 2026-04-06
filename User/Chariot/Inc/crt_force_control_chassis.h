#ifndef CRT_FORCE_CONTROL_CHASSIS_H
#define CRT_FORCE_CONTROL_CHASSIS_H
#include "alg_pid.h"
#include "dvc_motor_dji.h"
#include "alg_new_power_limit.h"
#include "dvc_supercap.h"
#include "dvc_imu.h"
#include "dvc_dwt.h"
#include "kalman_filter.h"
#include "alg_filter.h"
#include "alg_slope.h"
/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type__
{
    Chassis_Control_Type_DISABLE__ = 0,
    Chassis_Control_Type_NORMAL__,
};

struct Struct_Chassis_INS_Data
{
    
    float IMU_X_Velocity;
    float IMU_Y_Velocity;
    float IMU_Omega;

    float Mix_Velocity_X;
    float Mix_Velocity_Y;
    float Mix_Omega;
};

class Class_Chassis
{
public:
    // 底盘速度值PID
    Class_PID PID_Velocity_X;

    // 底盘速度方向PID
    Class_PID PID_Velocity_Y;

    // 底盘角度PID, radian
    Class_PID PID_Radian;

    // 底盘角速度PID
    Class_PID PID_Omega;

    // 轮向电机
    Class_Motor_DJI_C620 Motor_Wheel[4];
    
    //功率管理
    Class_New_Power_Limit Power_Limit;

    Struct_Power_Management Power_Management;

    //IMU
    Class_IMU Boardc_BMI;

    //超电
    Class_Supercap Supercap;

    //低通滤波器
    Class_Filter_Fourier PID_Velocity_Filter[2];
    Class_Filter_Fourier PID_Omega_Filter;

    // Spike滤波，滤跳变
    SpikeFilter Velocity_X_Spike;
    SpikeFilter Velocity_Y_Spike;
    SpikeFilter Omega_Spike;

    float Now_Velocity_X_Spike = 0.0f;
    float Now_Velocity_Y_Spike = 0.0f;
    float Now_Omega_Spike = 0.0f;

    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    inline float Get_Now_Motor_Power();

    inline float Get_Now_Wheel_Motor_Power();

    inline float Get_Wheel_Factor();

    inline float Get_Now_Velocity_X();

    inline float Get_Now_Velocity_Y();

    inline float Get_Now_Omega();

    inline float Get_Now_AHRS_Omega();

    inline float Get_Angle_Pitch();

    inline float Get_Angle_Roll();

    inline float Get_Slope_Direction_X();

    inline float Get_Slope_Direction_Y();

    inline float Get_Slope_Direction_Z();

    inline Enum_Chassis_Control_Type__ Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline float Get_Target_Radian();

    inline void Set_Power_Limit_Max(float __Power_Limit_Max);

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type__ __Chassis_Control_Type);

    inline void Set_Target_Velocity_X(float __Target_Velocity_X);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Radian(float __Target_Radian);

    void Init();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();
protected:
    const float Wheel_Radius = 0.141f/2.0f; // 轮子半径
    const float half_w = 0.159f;            // m
    const float half_l = 0.152f;           // m
    const float Motor_Omega_Contration = 20.0f / 16384.0f;  // 轮子速度环P的收缩系数，主要是方便调参
    // 内部变量
    //观测车体的速度
    Struct_Chassis_INS_Data INS_Data;
    // 轮向电机线速度目标值
    float Target_Wheel_Velocity[4];
    // 轮向电机角速度目标值
    float Target_Wheel_Omega[4];
    // 论相惦记目标力
    float Target_Wheel_Force[4];
    // 轮向电机电流目标值
    float Target_Wheel_Current[4];
    // 轮向电机静摩擦阻力电流值
    float Static_Resistance_Wheel_Current[4] = {0.0f,
                                                0.0f,
                                                0.0f,
                                                0.0f};
    // 轮向电机动摩擦阻力电流值
    float Dynamic_Resistance_Wheel_Current[4] = {0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f};
    // 轮向电机摩擦阻力连续化的角速度阈值
    float Wheel_Resistance_Omega_Threshold = 1.0f;
    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor[4] = {600.0f, 600.0f,600.0f, 600.0f}; 

    // 读变量

    // 功率
    float Now_Motor_Power = 0.0f;
    // 轮向电机功率
    float Now_Wheel_Motor_Power = 0.0f;

    // 轮向电机功率因数
    float Wheel_Factor = 1.0f;

    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 底盘相对Odom角度
    float Angle_Pitch = 0.0f;
    float Angle_Roll = 0.0f;

    // 斜坡法向量在底盘方向向量
    float Slope_Direction_X = 0.0f;
    float Slope_Direction_Y = 0.0f;
    float Slope_Direction_Z = 1.0f;

    // 写变量

    // 功率限制上限
    float Power_Limit_Max = 60.0f;

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_Type__ Chassis_Control_Type = Chassis_Control_Type_DISABLE__;

    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 角速度环目标角速度
    float Target_Omega = 0.0f;
    // 目标角度
    float Target_Radian = 0.0f;

    // 内部函数
    void Self_Resolution();
    
    void PID_Radian_Output();

    void Kinematics_Inverse_Resolution();

    void _Steer_Motor_Kinematics_Nearest_Transposition();

    void Output_To_Dynamics();

    void Dynamics_Inverse_Resolution();

    void Output_To_Motor();

    void _Power_Limit_Control();

    float __Steer_Power_Limit_Control(float steer_available_power, float steer_consume_power);

    float __Wheel_Power_Limit_Control(float wheel_available_power, float wheel_consume_power);
};
/**
 * @brief 获取功率
 *
 * @return float 功率
 */
inline float Class_Chassis::Get_Now_Motor_Power()
{
    return (Now_Motor_Power);
}

/**
 * @brief 获取轮向电机功率
 *
 * @return float 轮向电机功率
 */
inline float Class_Chassis::Get_Now_Wheel_Motor_Power()
{
    return (Now_Wheel_Motor_Power);
}

/**
 * @brief 获取轮向电机功率因数
 *
 * @return float 轮向电机功率因数
 */
inline float Class_Chassis::Get_Wheel_Factor()
{
    return (Wheel_Factor);
}

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取当前角速度, 优先使用陀螺仪数据, rad/s
 *
 * @return float 当前角速度, 优先使用陀螺仪数据, rad/s
 */
// inline float Class_Chassis::Get_Now_AHRS_Omega()
// {
//     if (AHRS_Chassis->Get_Status() == AHRS_WHEELTEC_Status_ENABLE)
//     {
//         return (-AHRS_Chassis->Get_Omega_Z());
//     }
//     else
//     {
//         return (Now_Omega);
//     }
// }

/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 */
inline float Class_Chassis::Get_Angle_Pitch()
{
    return (Angle_Pitch);
}

/**
 * @brief 获取底盘相对Odom角度
 *
 * @return float 底盘相对Odom角度
 */
inline float Class_Chassis::Get_Angle_Roll()
{
    return (Angle_Roll);
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_X()
{
    return (Slope_Direction_X);
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_Y()
{
    return (Slope_Direction_Y);
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_Z()
{
    return (Slope_Direction_Z);
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type__ Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标速度方向
 *
 * @return float 目标速度方向
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_Chassis::Get_Target_Radian()
{
    return (Target_Radian);
}

/**
 * @brief 设定功率控制上限
 *
 * @return __Power_Limit_Max 功率控制上限
 */
inline void Class_Chassis::Set_Power_Limit_Max(float __Power_Limit_Max)
{
    Power_Limit_Max = __Power_Limit_Max;
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type__ __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Chassis::Set_Target_Radian(float __Target_Radian)
{
    Target_Radian = __Target_Radian;
}

#endif
