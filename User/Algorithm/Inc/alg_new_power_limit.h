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
#define M3508_REDUATION (3591.f / 187.f)                                                       // 3508标准减速比
#define M3508_TORQUE_CONSTANT 0.3f                                                             // 3508带标准减速箱的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f)                                  // Icmd映射到Itorque
#define M3508_Kt (M3508_TORQUE_CONSTANT / M3508_REDUATION)                                    // 3508转子的转矩常数
#define M3508_CMD_CURRENT_TO_TORQUE (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)          // 发送的电流控制值（16384）映射到转子扭矩
#define M3508_TORQUE_TO_CMD_CURRENT (1.0f / (M3508_CMD_CURRENT_TO_TORQUE_CURRENT * M3508_Kt)) // 转子扭矩映射到电流控制值（16384）
/*------------------------------------------------------------------------*/

/*6020参数*/
// 需哨兵组成员补充，最终需要GM6020_CMD_CURRENT_TO_TORQUE，GM6020_TORQUE_TO_CMD_CURRENT
/*----------------------------------------------------------------------------*/

//映射到转子后的一些转换参数
#define GET_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define GET_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

/*---------------------------------------------------------------------------------*/

typedef struct
{
    float feedback_omega;  // 反馈的转子转速,rpm
    float feedback_torque; // 反馈的转子转矩,Nm

    float torque;           // pid输出的转子转矩,Nm
    float theoretical_power; // 理论功率
    float scaled_power;      // 功率（收缩后）
    float Target_error;      //目标值与当前值的误差绝对值

    int16_t pid_output; // pid输出的扭矩电流控制值（16384）
    int16_t output;     // 最终输出扭矩电流控制值（16384）
} Struct_Power_Motor_Data;

typedef struct
{
    uint16_t Max_Power;            // 最大功率限制
    float Min_Buffer = 40.f;       // 最小预留缓冲能量
    float Buffer_K = 8.0f;         // 缓冲环Kp
    float Buffer_Power;            // 缓冲环可用能量
    float Scale_Conffient;         // 功率收缩系数
    float Theoretical_Total_Power; // 理论总功率  考虑负功率
    float Needed_Scaled_Theoretical_Total_Power;    //需要再分配的理论总功率
    float Scaled_Total_Power;      // 收缩后总功率
    float Actual_Power;            // 实际总功率
    float Total_error;              

    #ifdef AGV
    Struct_Power_Motor_Data Motor_Data[8]; // 舵轮底盘八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机
    #else
    Struct_Power_Motor_Data Motor_Data[4]; // 舵轮底盘八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机
    #endif

} Struct_Power_Management;

class Class_Power_Limit
{
public:

    float Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index);
    float Calculate_Toque(float omega, float power, float torque, uint8_t motor_index);
    void Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data);
    void Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient);
    void Power_Task(Struct_Power_Management &power_management);
    void Init(float __ErrorLow = 5, float __ErrorUp = 100);

    inline float Get_K1() const { return k1; }
    inline float Get_K2() const { return k2; }
    inline float Get_K3() const { return k3; }

    inline void Set_K1(float _k1) { k1 = _k1; }
    inline void Set_K2(float _k2) { k2 = _k2; }
    inline void Set_K3(float _k3) { k3 = _k3; }

protected:

    // 普通四电机底盘参数
    uint8_t Max_flag = 0;
    float ErrorLow;               //低于Error会进行等比分配
    float ErrorUp;                //高于error，直接按error分配功率，防止功率乱吃，分配错误

    float k1 = 0.0200000001f;
    float k2 = 485.00f;
    float k3 = 4.90f / 8.00f;
    RLS<2> rls{1e-5f, 0.9999f};

};

#endif