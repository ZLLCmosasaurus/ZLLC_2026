/**
 * @file alg_power_limit.h
 * @author qyx
 * @brief 自适应功率限制算法
 * @version 1.2
 * @date
 *
 * @copyright ZLLC 2025
 *
 */

#ifndef ALG_NEW_POWER_LIMIT_H
#define ALG_NEW_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"
#include "RLS.hpp"

/* Exported macros -----------------------------------------------------------*/
#define RAD_TO_RPM 9.5493f
#define PI 3.14159265354f

/*3508参数*/
#define M3508_REDUATION (3591.f / 187.f)                                                      // 3508标准减速比
#define M3508_TORQUE_CONSTANT 0.3                                                             // 3508带标准减速箱的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f)                                  // Icmd映射到Itorque
#define M3508_Kt (M3508_TORQUE_CONSTANT / M3508_REDUATION)                                    // 3508转子的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)          // 发送的电流控制值（16384）映射到转子扭矩
#define M3508_TORQUE_TO_CMD_CURRENT (1.0f / (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)) // 转子扭矩映射到电流控制值（16384）
/*------------------------------------------------------------------------*/

/*6020参数*/
// 需哨兵组成员补充，最终需要GM6020_CMD_CURRENT_TO_TORQUE，GM6020_TORQUE_TO_CMD_CURRENT
/*----------------------------------------------------------------------------*/

#ifdef AGV
#define MOTORS_NUM 8
#ifdef INFANTRY || HERO
// 转向电机参数选择
#define DIR_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define DIR_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

// 动力电机参数选择
#define MOT_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define MOT_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

#endif

#ifdef SENTRY
// 转向电机参数选择
#define DIR_CMD_CURRENT_TO_TORQUE GM6020_CMD_CURRENT_TO_TORQUE
#define DIR_TORQUE_TO_CMD_CURRENT GM6020_TORQUE_TO_CMD_CURRENT

// 动力电机参数选择
#define MOT_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define MOT_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

#endif

// 根据电机索引选择对应的转换系数
#define GET_CMD_CURRENT_TO_TORQUE(motor_index) ((motor_index % 2 == 0) ? DIR_CMD_CURRENT_TO_TORQUE : MOT_CMD_CURRENT_TO_TORQUE)
#define GET_TORQUE_TO_CMD_CURRENT(motor_index) ((motor_index % 2 == 0) ? DIR_TORQUE_TO_CMD_CURRENT : MOT_TORQUE_TO_CMD_CURRENT)
#else
#define MOTORS_NUM 4
// 根据电机索引选择对应的转换系数
#define GET_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define GET_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT
#endif

/*---------------------------------------------------------------------------------*/

typedef struct
{
    __fp16 feedback_omega;  // 反馈的转子转速,rpm
    __fp16 feedback_torque; // 反馈的转子转矩,Nm

    __fp16 torque;           // pid输出的转子转矩,Nm
    float theoretical_power; // 理论功率
    float scaled_power;      // 功率（收缩后）
    float omega_error;       // 转子转速误差(绝对值)
    int16_t pid_output;      // pid输出的扭矩电流控制值（16384）
    int16_t output;          // 最终输出扭矩电流控制值（16384）
} Struct_Power_Motor_Data;

typedef struct
{
    uint16_t Max_Power;                             // 最大功率限制
    float Scale_Conffient;                          // 功率收缩系数
    float Theoretical_Total_Power;                  // 理论总功率
    float Scaled_Total_Power;                       // 收缩后总功率
    float Actual_Power;                             // 实际总功率
    float Total_error;                              // 转子转速总误差（绝对值）
    Struct_Power_Motor_Data Motor_Data[MOTORS_NUM]; // 舵轮底盘八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机

} Struct_Power_Management;

class Class_Power_Limit
{
public:
    float Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index);
    float Calculate_Toque(float omega, float power, float torque, uint8_t motor_index);
    void Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data);
    void Power_Task(Struct_Power_Management &power_management);
    void Init(float __E_lower, float __E_upper);
    void Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient);
#ifdef AGV
    // AGV模式下的getter/setter
    inline float Get_K1_Mot() const { return k1_mot; }
    inline float Get_K2_Mot() const { return k2_mot; }
    inline float Get_K1_Dir() const { return k1_dir; }
    inline float Get_K2_Dir() const { return k2_dir; }
    inline float Get_K3_Mot() const { return k3_mot; }
    inline float Get_K3_Dir() const { return k3_dir; }

    inline void Set_K1_Mot(float _k1) { k1_mot = _k1; }
    inline void Set_K2_Mot(float _k2) { k2_mot = _k2; }
    inline void Set_K1_Dir(float _k1) { k1_dir = _k1; }
    inline void Set_K2_Dir(float _k2) { k2_dir = _k2; }
    inline void Set_K3_Mot(float _k3) { k3_mot = _k3; }
    inline void Set_K3_Dir(float _k3) { k3_dir = _k3; }
#else
    // 普通模式下的getter/setter
    inline float Get_K1() const { return k1; }
    inline float Get_K2() const { return k2; }
    inline float Get_K3() const { return k3; }

    inline void Set_K1(float _k1) { k1 = _k1; }
    inline void Set_K2(float _k2) { k2 = _k2; }
    inline void Set_K3(float _k3) { k3 = _k3; }
#endif

protected:
#ifdef AGV
    // 参数
    float k1_mot = 0.000249867036; // 动力电机k1
    float k2_mot = 489.124512;     // 动力电机k2
    float k3_mot = 9.28f / 8.0f;   // 动力电机k3

    float k1_dir = 0.000249867036; // 转向电机k1
    float k2_dir = 489.124512;     // 转向电机k2
    float k3_dir = 9.28f / 8.0f;   // 转向电机k3

    RLS<2> rls_mot{1e-5f, 0.92f}; // 动力电机RLS
    RLS<2> rls_dir{1e-5f, 0.92f}; // 转向电机RLS
#else
    // 普通四电机底盘参数
    float k1 = 9.74917129e-05;
    float k2 = 489.123322;
    float k3 = 2.52f / 4.0f;
    RLS<2> rls{1e-5f, 0.9999f};
#endif
    float ErrorLow = 0.0f;
    float ErrorUp = 0.0f;
};

#endif