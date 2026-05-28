/**
 * @file crt_chassis.h
 * @author cjw by wanghongxi
 * @brief 底盘
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 建系原则
 *  ↑y
 *  │
 *  └————→x
 */

/**
 * @brief 轮组编号
 * 1 2
 * 4 3
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_lkmotor.h"
//#include "alg_power_limit.h"
#include "alg_new_power_limit.h"
#include "dvc_supercap.h"
#include "config.h"
#include "dvc_minipc.h"
#include "drv_math.h"
#include "kalman_filter.h"

/* Exported macros -----------------------------------------------------------*/
#define wheel_diameter 0.12f   // 驱动轮直径，m
#define half_length 0.178f           // 轮距的一半，m

#define WHEEL_RADIUS (wheel_diameter / 2)   // 驱动轮半径，m
#define R_DIST (half_length * 1.414f) // 旋转中心与四个舵轮的距离


#define PI 3.141593f
#define PI2 (2 * PI)
#define RPM_TO_RAD (PI2 / 60)                // 将转速(RPM)转换为角速度(rad/s)  1 rpm = 2pi/60 rad/s
#define RPM_TO_VEL (PI * wheel_diameter / 60)  // 将转速(RPM)转换为轮子线速度(cm/s)  vel = rpm*pi*D/60  m/s
#define VEL_TO_RPM (1 / RPM_TO_VEL)            // 将轮子线速度(m/s)转换为转速(RPM)
#define M2006_REDUCTION_RATIO 36.000000f     // 定义M2006电机的减速比
#define M3508_REDUCTION_RATIO 15.76f     // 定义M3508电机的减速比
#define MF7025_ENCODER_ANGLE 4096.0f         // 定义MF7025电机编码器每圈脉冲数

#define RAD_TO_4096 (4096.0f / PI / 2.0f)      // 将弧度值转换为编码器计数值

/* Exported types ------------------------------------------------------------*/
extern float wwx,wwy;
extern float H7_Offset_X, H7_Offset_Y;

/**
 * @brief 底盘冲刺状态枚举
 *
 */
enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0, 
    Sprint_Status_ENABLE,
};


/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type :uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_Drive,  //底盘直驱
    //Chassis_Control_Type_SPIN_NePositive  // 反小陀螺
};

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
//舵轮
class Class_Steering_Wheel_Chassis
{
public:

    Class_IMU *IMU;
    Class_LK_Motor *Motor_Yaw;
    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;
    //对yaw反馈角度滤波，具体7025需不需要另说
    Class_Filter_Fourier Filter_Omega;
    //超电类
    Class_Supercap Supercap;    
    //功率限制
    Class_Power_Limit Power_Limit;
    Struct_Power_Management Power_Management;
    
    //裁判系统
    Class_Referee *Referee;

    //下方转动电机
    Class_DJI_Motor_C620 Motor_Wheel[4];
    Class_DJI_Motor_C620_Steer Motor_Steer[4];

    //随动环
    //Class_PID Chassis_Follow_PID_Angle;

    void Init(float __Velocity_X_Max = 8.0f, float __Velocity_Y_Max = 8.0f, float __Omega_Max = 20.0f, float __Steer_Power_Ratio = 0.5);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Steer_Power();
    inline float Get_Target_Steer_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Spin_Omega();
    inline float Get_Relative_Angle();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Drive_Omega(float __Target_Drive_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Relative_Angle(float __Relative_Angle);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);

protected:
    //初始化相关常量

    //速度X限制
    float Velocity_X_Max=20.0f;
    //速度Y限制
    float Velocity_Y_Max=20.0f;
    //角速度限制
    float Omega_Max = 4.0f;
    //舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    //底盘小陀螺模式角速度
    float Spin_Omega = 8.0f;
    //常量


    //电机理论上最大输出
    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    //内部变量
    float Relative_Angle = 0.0f;

    //舵向电机目标值
    float Target_Steer_Angle[4];
    //驱动电机目标值
    float Target_Wheel_Omega[4];
    //驱动电机扭矩
    float Target_Wheel_Torque[4];

    //读变量

    //当前总功率
    float Now_Power = 0.0f;
    //当前舵向电机功率
    float Now_Steer_Power = 0.0f;
    //可使用的舵向电机功率
    float Target_Steer_Power = 0.0f;
    //当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    //可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    //写变量

    //读写变量

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;
    //直驱下的目标角速度
    float Target_Drive_Omega = 0.0f;

    //内部函数
    void Speed_Resolution();

    void Set_Chassis_Kalman_Measure(float value1, float value2, float value3, float value4, float value5, float value6);
    void Chassis_Speed_Estimate();
    void Stree_Angle_Resolution();
    void Force_Speed_Resolution();

    Class_PID PID_Omega;
    Class_PID PID_Velocity_X;
    Class_PID PID_Velocity_Y;

        // 轮向电机动摩擦阻力电流值(起转阻力)
    float Dynamic_Resistance_Wheel_Current[4] = {0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f};
    // 轮向电机摩擦阻力连续化的角速度阈值
    float Wheel_Resistance_Omega_Threshold = 1.0f;
    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.0f;

    const float Wheel_Azimuth[4] = {3.0f * PI / 4.0f,
                                    - 3.0f * PI / 4.0f,
                                    - PI / 4.0f,
                                    PI / 4.0f};

    KalmanFilter_t Chassis_Speed_Kalman;
};

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Steering_Wheel_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Steering_Wheel_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Steering_Wheel_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Steering_Wheel_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Steering_Wheel_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Steering_Wheel_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Steering_Wheel_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}


/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Steering_Wheel_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Steering_Wheel_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Steering_Wheel_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Steering_Wheel_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Steering_Wheel_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Steering_Wheel_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

float Class_Steering_Wheel_Chassis::Get_Relative_Angle()
{
    return (Relative_Angle);
}
/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Steering_Wheel_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Steering_Wheel_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Steering_Wheel_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Steering_Wheel_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标直驱角速度
 *
 * @param __Target_Drive_Omega 目标直驱角速度
 */
void Class_Steering_Wheel_Chassis::Set_Target_Drive_Omega(float __Target_Drive_Omega)
{
    Target_Drive_Omega = __Target_Drive_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Steering_Wheel_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Steering_Wheel_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Steering_Wheel_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Steering_Wheel_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Steering_Wheel_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

void Class_Steering_Wheel_Chassis::Set_Relative_Angle(float __Relative_Angle)
{
    Relative_Angle = __Relative_Angle;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
