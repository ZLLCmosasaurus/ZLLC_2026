#ifndef _ALG_NEW_POWER_LIMIT_H_
#define _ALG_NEW_POWER_LIMIT_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
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
// 转向电机参数选择
#define DIR_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define DIR_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

// 动力电机参数选择
#define MOT_CMD_CURRENT_TO_TORQUE M3508_CMD_CURRENT_TO_TORQUE
#define MOT_TORQUE_TO_CMD_CURRENT M3508_TORQUE_TO_CMD_CURRENT

/*---------------------------------------------------------------------------------*/
// 根据电机索引选择对应的转换系数
#define GET_CMD_CURRENT_TO_TORQUE(motor_index) ((motor_index % 2 == 0) ? DIR_CMD_CURRENT_TO_TORQUE : MOT_CMD_CURRENT_TO_TORQUE)
#define GET_TORQUE_TO_CMD_CURRENT(motor_index) ((motor_index % 2 == 0) ? DIR_TORQUE_TO_CMD_CURRENT : MOT_TORQUE_TO_CMD_CURRENT)
typedef struct
{
    int16_t feedback_omega;  // 反馈的转子转速,rpm
    float feedback_torque; // 反馈的转子转矩,Nm

    float torque;           // pid输出的转子转矩,Nm
    float theoretical_power; // 理论功率
    float scaled_power;      // 功率（收缩后）

    int16_t pid_output; // pid输出的扭矩电流控制值（16384）
    int16_t output;     // 最终输出扭矩电流控制值（16384）
} Struct_Power_Motor_Data;

typedef struct
{
    float Max_Power;            // 最大功率限制
    float Scale_Conffient;         // 功率收缩系数//0：转向电机 1：动力电机
    float Theoretical_Total_Power; // 理论总功率
    float Needed_Scaled_Theoretical_Total_Power;    //需要再分配的理论总功率
    float Scaled_Total_Power;      // 收缩后总功率
    float Actual_Power;            // 实际总功率

    // 麦轮轮组只有四个电机
    Struct_Power_Motor_Data Motor_Data[4]; // 舵轮底盘八个电机，分为四组，默认偶数索引值的电机为转向电机，奇数索引值的电机为动力电机

} Struct_Power_Management;

class Class_New_Power_Limit
{
public:
    float Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index);
    float Calculate_Toque(float omega, float power, float torque, uint8_t motor_index);
    void Power_Task(Struct_Power_Management &power_management);
    void Power_Allocate(float Power_Limit, float rate,float dir_predict_power,float mot_predict_power,float *dir_power_allocate,float *mot_power_allocate);
    void Power_Allocate(float Power_Limit, float mot_predict_power, float *mot_power_allocate);
    void Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient);
    // 普通模式下的getter/setter
    inline float Get_K1() const { return k1; }
    inline float Get_K2() const { return k2; }
    inline float Get_K3() const { return k3; }
    
    inline void Set_K1(float _k1) { k1 = _k1; }
    inline void Set_K2(float _k2) { k2 = _k2; }
    inline void Set_K3(float _k3) { k3 = _k3; }
    inline void Set_Control_Status(uint8_t status) { Control_Status = status; }

protected:

    float k1 = 0.01;
    float k2 = 550.0f;
    float k3 = 3.50;

    uint8_t Control_Status; // 控制状态
};
#endif
