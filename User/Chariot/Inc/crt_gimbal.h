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

#define GEAR_RATIO 2
#define DM2325_GEAR_RATIO 50

#ifndef PI
#define PI 3.1415926535f
#endif

#define PITCH_RATIO 2.0588f // pitch轴减速比17:35

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"
#include "dvc_dmmotor.h"
#include "dvc_jd_motor.h"
#include "alg_fsm.h"
#include "arm_model.h"
#include "crt_joint_planner.h"
#include "dvc_dwt.h"
#include "buzzer.h"
#include "trajectories.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type : uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

enum Enum_Motor_DM_J0_Yaw_Type : uint8_t
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
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
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
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
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
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
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

class Class_Gimbal;
class Class_FSM_Calibration : public Class_FSM
{
public:
    Class_Gimbal *Gimbal;
    void Reload_TIM_Status_PeriodElapsedCallback();

    /*电机校准执行函数*/
    bool Motor_Calibration(Class_DM_Motor_J4310 *Motor, float &Cali_Offset, float Cali_Max_Radian, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);
    bool Motor_Calibration(Class_DJI_Motor_C610 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);

    inline bool Get_Roll_cali_status();
    inline bool Get_Pitch_cali_status();
    inline bool Get_Gripper_cali_status();

protected:
    /*roll轴校准相关变量*/
    bool roll_cali_status = false;   // 校准状态，初始为false
    float roll_locked_torque = 2.5f; // 堵转力矩
    float roll_offset = 0.0f;        // 存储校准后的偏差, rad
    float roll_range = 261.8f;

    /*Pitch轴校准相关变量*/
    bool pitch_cali_status = false;
    float pitch_locked_torque = 2.5f;
    float pitch_offset = 0.0f;
    float pitch_range = 174.0f;

    float gripper_offset = 0.0f;           // 夹爪校准后的偏差角度，rad
    float gripper_locked_torque = 1600.0f; // 堵转力矩
    bool gripper_cali_status = false;

    uint16_t roll_locked_cnt = 0;    // Roll轴堵转时间计数
    uint16_t pitch_locked_cnt = 0;   // Pitch轴堵转时间计数
    uint16_t gripper_locked_cnt = 0; // 夹爪堵转时间计数
};

/**
 * @brief Specialized, 云台类，在工程机器人上直接把机械臂当做云台
 *
 */
class Class_Gimbal
{
public:
    // imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    /*SCARA臂*/
    Class_DM_Motor_J4310 J0_Pitch_4340;
    Class_DM_Motor_J4310 J1_Yaw_8009P;
    Class_DM_Motor_J4310 J2_Yaw_4340P;
    Class_DM_Motor_J4310 J3_Roll_2325;
    Class_DM_Motor_J4310 J4_Pitch_2325;
    Class_Gimbal_Joint_Planner J1_Yaw_Planner;
    Class_Gimbal_Joint_Planner J2_Yaw_Planner;
    Class_Jodell_Motor Jodell_ERG150T; // 钧舵ERG150T夹爪电机，兼具Roll和夹爪功能

    Class_FSM_Calibration Calibration_FSM; // 校准状态机类
    friend class Class_FSM_Calibration;

    /*机械臂DH建模*/
    Class_Trajectory_Tracer Trajectory_Tracer;
    /*机械整臂初始化标志位*/
    bool arm_init = false;

    void Init();

    inline float Get_Target_J0_Pitch_Radian();
    inline float Get_Target_J1_Yaw_Radian();
    inline float Get_Target_J2_Yaw_Radian();
    inline float Get_Target_J3_Roll_Radian();
    inline float Get_Target_J4_Pitch_Radian();
    inline float Get_Target_J5_Roll_Radian();

    inline uint8_t Get_Target_Gripper_Position();

    inline float Get_Target_Gripper_Angle();
    inline float Get_Target_Gripper_Radian();
    inline float Get_Gripper_Min_Radian();

    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);

    inline void Set_Target_J0_Pitch_Radian(float __Target_J0_Pitch_Radian);
    inline void Set_Target_J1_Yaw_Radian(float __Target_J1_Yaw_Radian);
    inline void Set_Target_J2_Yaw_Radian(float __Target_J2_Yaw_Radian);
    inline void Set_Target_J3_Roll_Radian(float __Target_J3_Roll_Radian);
    inline void Set_Target_J4_Pitch_Radian(float __Target_J4_Pitch_Radian);
    inline void Set_Target_J5_Roll_Radian(float __Target_J5_Yaw_Radian);

    inline void Set_Target_Gripper_Position(uint8_t __Target_Gripper_Position);

    inline void Set_Target_Gripper_Angle(float __Target_Gripper_Angle);
    inline void Set_Target_Gripper_Radian(float __Target_Gripper_Radian);

    // 2325特殊类型函数
    inline float Get_Target_J3_Roll_Radian_In_PI();
    inline float Get_Target_J4_Pitch_Radian_In_PI();

    /*VT03相关函数*/
    inline float Get_Target_VT03_Pitch_Angle();
    inline float Get_Target_VT03_Yaw_Angle();
    inline void Set_Target_VT03_Pitch_Angle(float __Target_VT03_Pitch_Angle);
    inline void Set_VT03_Yaw_PWM(uint16_t __PWM);
    inline void Set_Target_VT03_Yaw_Angle(float __Target_VT03_Yaw_Angle);
    inline void Set_Planner_Enable(Enum_Gimbal_Joint joint, bool enable);
    inline void Set_Planner_Mode(Enum_Gimbal_Joint joint, Enum_Joint_Planner_Mode mode);
    inline void Reset_Planner(Enum_Gimbal_Joint joint);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 电机CAN通信优先级变量
    static inline uint32_t can_priority_cnt = 0; // 电机CAN通信优先级计数器，前面写inline是为了能保持变量是类内部静态变量的同时可以自动初始化

    // gripper校准角度，默认为0
    float gripper_cali_offset = 0.0f;
    float Min_gripper_Radian = gripper_cali_offset;
    float Max_gripper_Radian = gripper_cali_offset + 0.95f; // 最大角度，完全闭合时为0.95f

    /*SCARA臂*/
    float J0_Pitch_Min_Radian = -1.431f;
    float J0_Pitch_Max_Radian = 0.698f;

    float J1_Yaw_Min_Radian = -0.872f;
    float J1_Yaw_Max_Radian = 0.872f;
    float J1_Yaw_MIT_KP = 100.0f;
    float J1_Yaw_MIT_KD = 2.0f;

    float J2_Yaw_Min_Radian = -2.443f;
    float J2_Yaw_Max_Radian = 2.443f;
    float J2_Yaw_MIT_KP = 120.0f;
    float J2_Yaw_MIT_KD = 2.0f;

    float J3_Roll_Cali_Offset;
    float J3_Roll_Min_Radian = -(150.0f / 180.0f) * PI * DM2325_GEAR_RATIO;
    float J3_Roll_Max_Radian = (150.0f / 180.0f) * PI * DM2325_GEAR_RATIO;
    // Roll轴零点，定义为中间位置
    float J3_Roll_Zero_Position_Radian = (J3_Roll_Min_Radian + J3_Roll_Max_Radian) / 2.0f;

    float J4_Pitch_Cali_Offset;
    float J4_Pitch_Min_Radian = -(90.0f / 180.0f) * PI * DM2325_GEAR_RATIO;
    float J4_Pitch_Max_Radian = (90.0f / 180.0f) * PI * DM2325_GEAR_RATIO;
    // Pitch轴零点，定义为中间位置
    float J4_Pitch_Zero_Position_Radian = (J4_Pitch_Min_Radian + J4_Pitch_Max_Radian) / 2.0f;

    // 图传舵机控制变量
    uint16_t VT03_Pitch_Servo_PWM = 1300;
    uint16_t VT03_Yaw_Servo_PWM = 1800; // 默认与底盘同向（正前方）
    // 内部变量

    // 读变量

    // 写变量

    // 云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    // 读写变量

    // Set函数中会自己乘减速比
    float Target_J0_Pitch_Radian = 0.0f;
    float Target_J0_Pitch_Omega = 0.75f * PI * PITCH_RATIO;

    float Target_J1_Yaw_Radian = 0.0f;
    float Target_J1_Yaw_Omega = 0.65f * PI;

    float Target_J2_Yaw_Radian = 0.0f;
    float Target_J2_Yaw_Omega = 0.75f * PI;

    // Set函数中会自己乘以减速比
    float Target_J3_Roll_Radian;
    float Target_J3_Roll_Omega = 1.25f * PI;

    float Target_J4_Pitch_Radian;
    float Target_J4_Pitch_Omega = 1.25f * PI;

    float Target_J5_Roll_Radian = 0.0f;
    float Target_J5_Roll_Omega = 1.25f * PI;

    // 钧舵夹爪行程
    uint8_t Target_Gripper_Position;

    // 夹爪角度，degree
    float Target_Gripper_Angle = 0.0f;
    float Target_Gripper_Radian = 0.0f;
    float Target_Gripper_Omega = 0.75f;

    /*图传相关角度*/
    float Target_VT03_Pitch_Angle = 0;
    float Max_VT03_Yaw_Angle = 180.0f;
    float Min_VT03_Yaw_Angle = -90.0f;
    float Target_VT03_Yaw_Angle = 0.0f;

    // 建模解算测试用
    float model_angle[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.5f, 0.0f};
    float model_degree[6];
    float control_angle[6] = {0};
    float xyz[3] = {0}; // 正解算结果
    float rpy[3] = {0};

#ifdef MY_DEBUG
    float debug_radian[6] = {0.0f}; // 测试电机控制角度映射专用
#endif
    float delta_time = 0.0f; // 用DWT测得的时间间隔，用这个看解析解的计算速度

    // 内部函数
    void Dispatch_J1_With_Planner();
    void Dispatch_J2_With_Planner();
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

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

void Class_Gimbal::Set_Planner_Enable(Enum_Gimbal_Joint joint, bool enable)
{
    switch (joint)
    {
    case (Gimbal_Joint_J1_Yaw):
    {
        J1_Yaw_Planner.Set_Enable(enable);
        break;
    }
    case (Gimbal_Joint_J2_Yaw):
    {
        J2_Yaw_Planner.Set_Enable(enable);
        break;
    }
    default:
    {
        break;
    }
    }
}

void Class_Gimbal::Set_Planner_Mode(Enum_Gimbal_Joint joint, Enum_Joint_Planner_Mode mode)
{
    switch (joint)
    {
    case (Gimbal_Joint_J1_Yaw):
    {
        J1_Yaw_Planner.Set_Mode(mode);
        break;
    }
    case (Gimbal_Joint_J2_Yaw):
    {
        J2_Yaw_Planner.Set_Mode(mode);
        break;
    }
    default:
    {
        break;
    }
    }
}

void Class_Gimbal::Reset_Planner(Enum_Gimbal_Joint joint)
{
    switch (joint)
    {
    case (Gimbal_Joint_J1_Yaw):
    {
        J1_Yaw_Planner.Reset(J1_Yaw_8009P.Get_Now_Angle_Rad());
        break;
    }
    case (Gimbal_Joint_J2_Yaw):
    {
        J2_Yaw_Planner.Reset(J2_Yaw_4340P.Get_Now_Angle_Rad());
        break;
    }
    default:
    {
        break;
    }
    }
}

#ifdef PUMA
/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}
float Class_Gimbal::Get_Target_Yaw_Radian()
{
    return (Target_Yaw_Radian);
}

/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}
float Class_Gimbal::Get_Target_Pitch_Radian()
{
    return (Target_Pitch_Radian);
}

/**
 * @brief 获取pitch_2轴角度
 *
 * @return float pitch_2轴角度
 */
float Class_Gimbal::Get_Target_Pitch_2_Angle()
{
    return (Target_Pitch_2_Angle);
}
float Class_Gimbal::Get_Target_Pitch_2_Radian()
{
    return (Target_Pitch_2_Radian);
}

/**
 * @brief 获取pitch_3轴角度
 *
 * @return float pitch_3轴角度
 */
float Class_Gimbal::Get_Target_Pitch_3_Angle()
{
    return (Target_Pitch_3_Angle);
}
float Class_Gimbal::Get_Target_Pitch_3_Radian()
{
    return (Target_Pitch_3_Radian);
}

/**
 * @brief 获取roll轴角度
 *
 * @return float roll轴角度
 */
float Class_Gimbal::Get_Target_Roll_Angle()
{
    return (Target_Roll_Angle);
}
float Class_Gimbal::Get_Target_Roll_Radian()
{
    return (Target_Roll_Radian);
}

/**
 * @brief 获取roll轴Min_Radian，可以用于在Roll轴校准后的零点上作增量计算
 *
 * @return float roll轴校准后的零点
 */
float Class_Gimbal::Get_Roll_Min_Radian()
{
    return (Min_Roll_Radian);
}

float Class_Gimbal::Get_Roll_Cali_Offset()
{
    return (roll_cali_offset);
}

/**
 * @brief 获取roll_2轴角度
 *
 * @return float roll_2轴角度
 */
float Class_Gimbal::Get_Target_Roll_2_Angle()
{
    return (Target_Roll_2_Angle);
}
float Class_Gimbal::Get_Target_Roll_2_Radian()
{
    return (Target_Roll_2_Radian);
}
float Class_Gimbal::Get_Target_Roll_2_Radian_Single()
{
    float single_radian = fmod(Target_Roll_2_Radian, 2.0f * PI);
    if (single_radian < 0.0f)
    {
        single_radian += 2.0f * PI;
    }
    return single_radian;
}

/**
 * @brief 获取Gripper角度
 *
 * @return float 夹爪角度
 */
float Class_Gimbal::Get_Target_Gripper_Angle()
{
    return (Target_Gripper_Angle);
}
float Class_Gimbal::Get_Target_Gripper_Radian()
{
    return (Target_Gripper_Radian);
}

/**
 * @brief 获取夹爪的Min_Radian，可以用于在夹爪校准后的零点上作增量计算（比较规范的写法和用法）
 *
 * @return float 夹爪校准后的零点
 */
float Class_Gimbal::Get_Gripper_Min_Radian()
{
    return (Min_gripper_Radian);
}

/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
    Set_Target_Yaw_Radian(Target_Yaw_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Yaw_Radian(float __Target_Yaw_Radian)
{
    Target_Yaw_Radian = __Target_Yaw_Radian;
    Math_Constrain(&Target_Yaw_Radian, Min_Yaw_Radian, Max_Yaw_Radian);
}

/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
    Set_Target_Pitch_Radian(Target_Pitch_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_Radian(float __Target_Pitch_Radian)
{
    Target_Pitch_Radian = __Target_Pitch_Radian;
    Math_Constrain(&Target_Pitch_Radian, Min_Pitch_Radian, Max_Pitch_Radian);
}

/**
 * @brief 设定pitch_2轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_2_Angle(float __Target_Pitch_2_Angle)
{
    Target_Pitch_2_Angle = __Target_Pitch_2_Angle;
    Set_Target_Pitch_2_Radian(Target_Pitch_2_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_2_Radian(float __Target_Pitch_2_Radian)
{
    Target_Pitch_2_Radian = __Target_Pitch_2_Radian;
    Math_Constrain(&Target_Pitch_2_Radian, Min_Pitch_2_Radian, Max_Pitch_2_Radian);
}

/**
 * @brief 设定pitch_3轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_3_Angle(float __Target_Pitch_3_Angle)
{
    Target_Pitch_3_Angle = __Target_Pitch_3_Angle;
    Set_Target_Pitch_3_Radian(Target_Pitch_3_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_3_Radian(float __Target_Pitch_3_Radian)
{
    Target_Pitch_3_Radian = __Target_Pitch_3_Radian;
    Math_Constrain(&Target_Pitch_3_Radian, Min_Pitch_3_Radian, Max_Pitch_3_Radian);
}

/**
 * @brief 设定roll轴角度
 *
 */
void Class_Gimbal::Set_Target_Roll_Angle(float __Target_Roll_Angle)
// 设置Degree制的**关节**角度，希望关节转动45°，就直接赋45.0f
{
    Target_Roll_Angle = __Target_Roll_Angle;
    // 关节角度(deg) -> 关节角度(rad) -> 电机角度(rad) -> 调用Set_Target_Roll_Radian自动加上Offset和限位
    float joint_rad = Target_Roll_Angle * PI / 180.0f;
    float motor_rad = joint_rad * 100.0f;
    Set_Target_Roll_Radian(motor_rad);
}
void Class_Gimbal::Set_Target_Roll_Radian(float __Target_Roll_Radian)
// 设置2325的目标角度，使用Target_Radian赋值给电机，控制的是**转子端**的角度，这里传入函数的Target_Roll_Radian不用手动加偏移，在函数里会自己加
{
    Target_Roll_Radian = __Target_Roll_Radian + Min_Roll_Radian;
    Math_Constrain(&Target_Roll_Radian, Min_Roll_Radian, Max_Roll_Radian);
}

/**
 * @brief 设定roll_2总角度
 *
 */
void Class_Gimbal::Set_Target_Roll_2_Angle(float __Target_Roll_2_Angle)
{
    Target_Roll_2_Angle = __Target_Roll_2_Angle;
    Set_Target_Roll_2_Radian(Target_Roll_2_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Roll_2_Radian(float __Target_Roll_2_Radian)
{
    Target_Roll_2_Radian = __Target_Roll_2_Radian;
}
/**
 * @brief 设定roll_2单圈角度，会自动计算出正转还是倒转，传入的角度必须要是0.0f - 2PI单圈值
 *
 */
void Class_Gimbal::Set_Target_Roll_2_Radian_Single(float Target_Roll_2_Radian_Single)
{
    // 电机对象传回的总角度，可能为负数
    float current_total_radian = Motor_6020_J5_Roll_2.Get_Now_Radian();
    // 先对2PI取模
    float current_single_radian = fmod(current_total_radian, 2.0f * PI);

    if (current_single_radian < 0.0f)
    // 如果是负数的话转成正数，这样就转成了单圈角度
    {
        current_single_radian += 2.0f * PI;
    }

    // 偏差值，加到总圈数上进行设置
    float delta = Target_Roll_2_Radian_Single - current_single_radian;

    if (delta > PI)
    {
        delta -= 2.0f * PI;
    }
    else if (delta < -PI)
    {
        delta += 2.0f * PI;
    }

    float __Target_Roll_2_Radian = current_total_radian + delta;

    Set_Target_Roll_2_Radian(__Target_Roll_2_Radian);
}

bool Class_FSM_Calibration::Get_roll_cali_status()
{
    return roll_cali_status;
}

#endif
bool Class_FSM_Calibration::Get_Roll_cali_status()
{
    return (roll_cali_status);
}

bool Class_FSM_Calibration::Get_Pitch_cali_status()
{
    return (pitch_cali_status);
}

bool Class_FSM_Calibration::Get_Gripper_cali_status()
{
    return (gripper_cali_status);
}

float Class_Gimbal::Get_Target_J0_Pitch_Radian()
{
    return (Target_J0_Pitch_Radian);
}

float Class_Gimbal::Get_Target_J1_Yaw_Radian()
{
    return (Target_J1_Yaw_Radian);
}

float Class_Gimbal::Get_Target_J2_Yaw_Radian()
{
    return (Target_J2_Yaw_Radian);
}

float Class_Gimbal::Get_Target_J3_Roll_Radian()
{
    return (Target_J3_Roll_Radian - J3_Roll_Zero_Position_Radian);
}

float Class_Gimbal::Get_Target_J4_Pitch_Radian()
{
    return (Target_J4_Pitch_Radian - J4_Pitch_Zero_Position_Radian);
}

float Class_Gimbal::Get_Target_J5_Roll_Radian()
{
    return (Target_J5_Roll_Radian);
}

uint8_t Class_Gimbal::Get_Target_Gripper_Position()
{
    return (Target_Gripper_Position);
}

void Class_Gimbal::Set_Target_J0_Pitch_Radian(float __Target_J0_Pitch_Radian)
{
    Target_J0_Pitch_Radian = __Target_J0_Pitch_Radian;
    Math_Constrain(&Target_J0_Pitch_Radian, J0_Pitch_Min_Radian, J0_Pitch_Max_Radian);
}

void Class_Gimbal::Set_Target_J1_Yaw_Radian(float __Target_J1_Yaw_Radian)
{
    Target_J1_Yaw_Radian = __Target_J1_Yaw_Radian;
    Math_Constrain(&Target_J1_Yaw_Radian, J1_Yaw_Min_Radian, J1_Yaw_Max_Radian);
}

void Class_Gimbal::Set_Target_J2_Yaw_Radian(float __Target_J2_Yaw_Radian)
{
    Target_J2_Yaw_Radian = __Target_J2_Yaw_Radian;
    Math_Constrain(&Target_J2_Yaw_Radian, J2_Yaw_Min_Radian, J2_Yaw_Max_Radian);
}

void Class_Gimbal::Set_Target_J3_Roll_Radian(float __Target_J3_Roll_Radian)
// 设置2325的目标角度，使用Target_Radian赋值给电机，控制的是**转子端**的角度，
// 这里函数声明里的Target_Roll_Radian不用手动加偏移，函数会自己加
{
    Target_J3_Roll_Radian = J3_Roll_Zero_Position_Radian + __Target_J3_Roll_Radian * DM2325_GEAR_RATIO;
    Math_Constrain(&Target_J3_Roll_Radian, J3_Roll_Min_Radian, J3_Roll_Max_Radian);
}

void Class_Gimbal::Set_Target_J4_Pitch_Radian(float __Target_J4_Pitch_Radian)
{
    Target_J4_Pitch_Radian = J4_Pitch_Zero_Position_Radian + __Target_J4_Pitch_Radian * DM2325_GEAR_RATIO;
    Math_Constrain(&Target_J4_Pitch_Radian, J4_Pitch_Min_Radian, J4_Pitch_Max_Radian);
}

void Class_Gimbal::Set_Target_J5_Roll_Radian(float __Target_J5_Roll_Radian)
{
    // 最后的Roll轴无限转
    Target_J5_Roll_Radian = __Target_J5_Roll_Radian;
}

void Class_Gimbal::Set_Target_Gripper_Position(uint8_t __Target_Gripper_Posiiton)
{
    Target_Gripper_Position = __Target_Gripper_Posiiton;
}

/**
 * @brief 设定Gripper角度
 *
 */
void Class_Gimbal::Set_Target_Gripper_Angle(float __Target_Gripper_Angle)
{
    Target_Gripper_Angle = __Target_Gripper_Angle;
    Math_Constrain(&Target_Gripper_Angle, 0.0f, 54.45f);
    Set_Target_Gripper_Radian(Target_Gripper_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Gripper_Radian(float __Target_Gripper_Radian)
{
    Target_Gripper_Radian = gripper_cali_offset + __Target_Gripper_Radian;
    Math_Constrain(&Target_Gripper_Radian, Min_gripper_Radian, Max_gripper_Radian);
}

float Class_Gimbal::Get_Target_J3_Roll_Radian_In_PI()
{
    return (Target_J3_Roll_Radian - J3_Roll_Zero_Position_Radian) / DM2325_GEAR_RATIO;
}

float Class_Gimbal::Get_Target_J4_Pitch_Radian_In_PI()
{
    return (Target_J4_Pitch_Radian - J4_Pitch_Zero_Position_Radian) / DM2325_GEAR_RATIO;
}

float Class_Gimbal::Get_Target_VT03_Pitch_Angle()
{
    return Target_VT03_Pitch_Angle;
}

float Class_Gimbal::Get_Target_VT03_Yaw_Angle()
{
    return Target_VT03_Yaw_Angle;
}

void Class_Gimbal::Set_VT03_Yaw_PWM(uint16_t __PWM)
{
    VT03_Yaw_Servo_PWM = __PWM;
}

void Class_Gimbal::Set_Target_VT03_Pitch_Angle(float __Target_VT03_Pitch_Angle)
{
    Target_VT03_Pitch_Angle = __Target_VT03_Pitch_Angle;
}

void Class_Gimbal::Set_Target_VT03_Yaw_Angle(float __Target_VT03_Yaw_Angle)
{
    Target_VT03_Yaw_Angle = __Target_VT03_Yaw_Angle;
    Math_Constrain(&Target_VT03_Yaw_Angle, Min_VT03_Yaw_Angle, Max_VT03_Yaw_Angle);

    // 映射关系
    uint16_t Target_VT03_Yaw_PWM;
    if (Target_VT03_Yaw_Angle <= -90.0f) Target_VT03_Yaw_PWM = 2300;
    if (Target_VT03_Yaw_Angle >= 180.0f) Target_VT03_Yaw_PWM = 800;

    Target_VT03_Yaw_PWM = (uint16_t)(1800.0f - (50.0f / 9.0f) * Target_VT03_Yaw_Angle + 0.5f);
    Set_VT03_Yaw_PWM(Target_VT03_Yaw_PWM);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
