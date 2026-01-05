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
 * @brief 轮组编号
 * 3 2
 *  1
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"
#include "alg_power_limit.h"
#include "dvc_supercap.h"
#include "config.h"
#include "dvc_minipc.h"
#include "alg_fsm.h"
/* Exported macros -----------------------------------------------------------*/
#define wheel_diameter 0.14100000f   // 轮子直径，m
#define half_width 0.159f            // m
#define half_length 0.152f           // m
#define ROTATION_CENTER_OFFSET 0.0f // 旋转中心位置偏移量，现在只有y方向上的偏移，且向y负方向偏移，这个偏移量为绝对值

#define THETA_A atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_B atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_C atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_D atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）

#define R_A half_width / cos(THETA_A) // 旋转中心与A舵轮的距离
#define R_B half_width / cos(THETA_B) // 旋转中心与B舵轮的距离
#define R_C half_width / cos(THETA_C) // 旋转中心与C舵轮的距离
#define R_D half_width / cos(THETA_D) // 旋转中心与D舵轮的距离

#define PI 3.141593f
#define PI2 2 * PI
#define RPM2RAD 0.104720f                // 		1 rpm = 2pi/60 rad/s
#define RPM2VEL 0.523599f                // 		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f                //
#define M2006_REDUCTION_RATIO 36.000000f //
#define M3508_REDUCTION_RATIO 19.000000f //
#define GM6020_ENCODER_ANGLE 8192.0f

#define RAD_TO_8191 8191.0f / PI / 2.0f

#define RAD2MM (280.0f / 26.5f)
/* Exported types ------------------------------------------------------------*/

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
enum Enum_Chassis_Control_Type : uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_NORMAL,
    Chassis_Control_Type_SLOPE,
};

/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
// omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;

    // 功率限制
    Class_Power_Limit Power_Limit;
    // Struct_Power_Management Power_Management;

    // 裁判系统
    Class_Referee *Referee;

    // 下方转动电机
    Class_DJI_Motor_C620 Motor_Wheel[4];
    Class_DJI_Motor_C620_Steer Motor_Steer[4];

    // 随动环
    Class_PID Chassis_Follow_PID_Angle;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f, float __Steer_Power_Ratio = 0.5);

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
    inline Enum_Supercap_Mode Get_Supercap_Mode();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Relative_Angle(float __Relative_Angle);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);
    void Power_Limit_Update();

protected:
    // 初始化相关常量

    // 速度X限制
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;
    // 舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    // 底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    // 常量

    // 电机理论上最大输出
    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    // 内部变量
    float Relative_Angle = 0.0f;

    // 舵向电机目标值
    float Target_Steer_Angle[3];
    // 转动电机目标值
    float Target_Wheel_Omega[4];

    // 读变量

    // 当前总功率
    float Now_Power = 0.0f;
    // 当前舵向电机功率
    float Now_Steer_Power = 0.0f;
    // 可使用的舵向电机功率
    float Target_Steer_Power = 0.0f;
    // 当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    // 可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Supercap_Mode Supercap_Mode = Supercap_DISABLE;
    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 内部函数
    void Speed_Resolution();
};

/* Exported variables --------------------------------------------------------*/
#ifdef ENGINEER
//工程底盘参数
// 轮子直径 单位m
const float WHELL_DIAMETER = 0.141f; // m
// 线速度转角速度 rad/s
const float VEL2RAD = 1.0f / (WHELL_DIAMETER / 2.0f); // v = omega * r, omega = v / r， v与r单位保持一致，使用m
#else
// 三轮车底盘参数

// 轮组半径
const float WHEEL_RADIUS = 0.0520f;

// 轮距中心长度
const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

// 前心距中心长度
const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

// 前后轮距
const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

// 前轮距前心
const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

// 轮组方位角
const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

// 轮子直径 单位m
const float WHELL_DIAMETER = 0.13f;

// 底盘半宽 单位m
const float HALF_WIDTH = 0.281f;

// 底盘半长 单位m
const float HALF_LENGTH = 0.281f;

// 底盘中心到每个轮子轴心投影距离
const float CHASSIS_RADIUS = sqrt(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH);

// 线速度转角速度 rad/s
const float VEL2RAD = 1.0f / (WHELL_DIAMETER / 2.0f);
#endif
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Tricycle_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Tricycle_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Tricycle_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

float Class_Tricycle_Chassis::Get_Relative_Angle()
{
    return (Relative_Angle);
}
Enum_Supercap_Mode Class_Tricycle_Chassis::Get_Supercap_Mode()
{
    return (Supercap_Mode);
}
/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Tricycle_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Tricycle_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

void Class_Tricycle_Chassis::Set_Relative_Angle(float __Relative_Angle)
{
    Relative_Angle = __Relative_Angle;
}
void Class_Tricycle_Chassis::Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode)
{
    Supercap_Mode = __Supercap_Mode;
}

class Class_Mecanum_Chassis;
class Class_FSM_Calibration_Chassis : public Class_FSM
{
public:
    Class_Mecanum_Chassis *Chassis;
    void Reload_TIM_Status_PeriodElapsedCallback();

    /*电机校准执行函数*/
    bool Motor_Calibration(Class_DJI_Motor_C620 *Motor, uint8_t index, float locked_torque, uint16_t &locked_cnt);
    
    inline bool Get_Uplift_cali_status(uint8_t index);

protected:
    /*抬升校准状态机相关变量*/
    float uplift_offset[4] = {0.0f};
    float uplift_cali_torque = 10000.0f;       //待测
    bool uplift_cali_status[4] = {false};

    uint16_t uplift_locked_cnt[4] = {0};// 抬升堵转时间计数
};

class Class_Mecanum_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;

    // 功率限制
    Class_Power_Limit Power_Limit;

    // 裁判系统
    Class_Referee *Referee;

    // 下方转动电机，顺时针方向编号
    Class_DJI_Motor_C620 Mecanum_Wheels[4];
    // 主动轮电机 - 2325 速度环不需要校准
    Class_DM_Motor_J4310 Track_Motor[2];
    // 抬升机构电机
    Class_DJI_Motor_C620 Uplift_Motor[4];

    // 抬升机构校准状态机
    Class_FSM_Calibration_Chassis Calibration_FSM;
    friend class Class_FSM_Calibration_Chassis;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Target_Track_Omega();
    inline float Get_Target_Uplift_Radian(uint8_t index);
    inline Enum_Supercap_Mode Get_Supercap_Mode();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Target_Track_Omega(float __Target_Track_Omega);
    inline void Set_Target_Uplift_Radian(uint8_t index, float __Target_Uplift_Radian);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);

    // 着地时的角度，相对于Min_Radian
    float Uplift_Touch_Radian[4] = {20.5f, 20.5f, 17.0f, 17.0f};
    
protected:
    // 初始化相关常量

    // 速度X限制，m/s
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;
    // 底盘电机最大转速
    float Wheels_Omega_Max = 31.416f; // 300rpm
    // 抬升机构最大高度 mm
    float Uplift_Height_Max = 280; 
    // 抬升电机校准后的最大角度，以电机返回的角度作为抬升高度的上限
    float Uplift_Max_Radian[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 抬升电机基于校准后最大角度而言的最低角度，差值为26.5rad
    float Uplift_Min_Radian[4] = {
                                  Uplift_Max_Radian[0] - 28.5f,
                                  Uplift_Max_Radian[1] - 28.5f,
                                  Uplift_Max_Radian[2] - 22.5f,
                                  Uplift_Max_Radian[3] - 22.5f
                                 };

    // 常量

    // 电机理论上最大输出，发给电机的电流值
    float Wheel_Max_Output = 16384.0f;

    // 麦轮轮组电机目标线速度与角速度
    float Target_Motor_Velocity[4];
    float Target_Motor_Omega[4];
    // 履带电机目标角速度，rad/s，两边保持一致
    float Target_Track_Omega;
    // 传给抬升电机的实际目标角度
    float Target_Uplift_Motor_Radian[4];
    // 用于和Uplift_Min_Radian相加得到电机目标角度的值，也就是加offset之前的目标角度，主要用于遥控器逻辑，初始在最高点，设为最小值和最大值之间的差值，此时赋给电机的角度应为0.0f
    float Target_Uplift_Radian[4] = {28.5f, 28.5f, 22.5f, 22.5f};

    // 读变量

    // 当前总功率
    float Now_Power = 0.0f;
    // 当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    // 可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Supercap_Mode Supercap_Mode = Supercap_DISABLE;
    // 目标速度X，m/s
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标车体角速度
    float Target_Omega = 0.0f;
    // 目标转向速度（与X，Y速度一样应为m/s）：(L*tan a + l) * Omega
    float Target_Velocity_Steer = Target_Omega * (half_length + half_width);
    // 抬升机构目标高度, m
    float Target_Uplift_Height[4];

    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 内部函数
    void Speed_Resolution();
    void Output();
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Mecanum_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Mecanum_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Mecanum_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Mecanum_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Mecanum_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Mecanum_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Mecanum_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标履带角速度
 *
 * @return float 目标履带角速度
 */
float Class_Mecanum_Chassis::Get_Target_Track_Omega()
{
    return (Target_Track_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Mecanum_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Mecanum_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Mecanum_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 获取目标抬升角度
 * @param index 目标抬升机构的索引，从0开始
 * @return float 目标抬升角度，为
 */
float Class_Mecanum_Chassis::Get_Target_Uplift_Radian(uint8_t index)
{
    if(index >= 4) return 13.0f;
    
    return (Target_Uplift_Radian[index]);
}

Enum_Supercap_Mode Class_Mecanum_Chassis::Get_Supercap_Mode()
{
    return (Supercap_Mode);
}
/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Mecanum_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Mecanum_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Mecanum_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Mecanum_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标履带角速度
 *
 * @param __Target_Track_Omega 目标履带角速度
 */
void Class_Mecanum_Chassis::Set_Target_Track_Omega(float __Target_Track_Omega)
{
    Target_Track_Omega = __Target_Track_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Mecanum_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Mecanum_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Mecanum_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Mecanum_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Mecanum_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

/**
 * @brief 设定抬升机构目标角度，rad，会自动加上offset赋给用于发给电机的目标角度
 *
 * @param index 目标抬升机构的索引，从0开始
 * @param __Target_Radian 目标抬升角度，rad
 */
void Class_Mecanum_Chassis::Set_Target_Uplift_Radian(uint8_t index, float __Target_Radian)
{
    if(index >= 4) return;

    Target_Uplift_Radian[index] = __Target_Radian;
    Target_Uplift_Motor_Radian[index] = Uplift_Min_Radian[index] + Target_Uplift_Radian[index];

    Math_Constrain(Target_Uplift_Radian + index, 0.0f, Uplift_Max_Radian[index] - Uplift_Min_Radian[index]);
    Math_Constrain(Target_Uplift_Motor_Radian + index, Uplift_Min_Radian[index], Uplift_Max_Radian[index]);
}

void Class_Mecanum_Chassis::Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode)
{
    Supercap_Mode = __Supercap_Mode;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/