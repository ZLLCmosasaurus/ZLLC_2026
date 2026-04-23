/**
 * @file alg_power_limit.h
 * @author lez 
 * @brief 功率限制算法
 * @version 1.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "arm_math.h"
#include "dvc_djimotor.h"
#include "config.h"

/* Exported macros -----------------------------------------------------------*/

// #define RAD_TO_RPM              9.5493f
// #define CMD_CURRENT_TO_TORQUE   ((20.f/16384.f)*0.3f)   //计算出3508直驱输出轴转子和cmd电流的关系
// #define REDUATION               (3591.f/187.f)         //减速比
 
// class Class_DJI_Motor_C620;

// class Class_Power_Limit
// {
//     public:

//     inline void Set_Power_Limit(float __total_power_limit);
//     inline void Set_Chassis_Buffer(float __buffer);
//     inline void Set_Supercap_Enegry(float __energy);
//     inline void Set_Supercap_Voltage(float __voltage);
//     inline void Set_Supercap_Print_Flag(uint8_t __flag);

//     float Get_Torque_Current(uint8_t num);

//     void Set_Motor(Class_DJI_Motor_C620 (&Motor)[4]);
//     //输出功率限制之后的电流到电机缓冲区
//     void Output(Class_DJI_Motor_C620 (&Motor)[4]);

//     void TIM_Adjust_PeriodElapsedCallback(Class_DJI_Motor_C620 (&Motor)[4]);
    
//     protected:

//     float Limit_K = 1.0f;
//     float Chassis_Buffer;
//     const float Min_Buffer = 10.0f; 
//     const float Protected_Buffer = 30.0f;

//     //转矩系数 rad转rpm系数
// 	float Toque_Coefficient = 1.99688994e-6f * (3591/187) / 13.93f;  // (20/16384)*(0.3)*(187/3591)/9.55

//     //电机模型参数
// 	float k1 = 1.3;		// k1 
// 	float k2 = 0.015;		// k2 
// 	float Alpha = 0.0f;
//     float Tansfer_Coefficient = 9.55f;  //转化系数 w*t/Tansfer_Coefficient

//     //超电电量
//     float Supercap_Energy;
//     //超电电压
//     float Supercap_Voltage;
//     //超电冲刺标志
//     uint8_t Supercap_Print_Flag = 0;

//     //过程变量
//     float equation_a;
//     float equation_b;
//     float equation_c;

//     //四电机输入目标力矩电流
//     float Input_Torque_Current[4];  
//     //四电机输出目标力矩电流
//     float Output_Torque_Current[4];  
//     //四电机当前电流值
//     float Torque_Torque_Current_Now[4];  
//     //四电机当前角速度
//     float Omega[4];	 
//     //底盘总功率限制
//     float Total_Power_Limit;  
//     //底盘总预测功率
//     float Total_Predict_Power = 0;  
//     //预测功率
//     float Predict_Power[4]; 
//     //功率伸缩系数
// 	float Power_Scale;  
//     //伸缩之后的功率限制
// 	float Scaled_Give_Power[4];  
// };

// /**
//  * @brief 设定总功率限制
//  *
//  */
// void Class_Power_Limit::Set_Power_Limit(float __total_power_limit)
// {
//     Total_Power_Limit = __total_power_limit;
// }

// /**
//  * @brief 设定底盘当前剩余缓冲能量
//  *
//  */
// void Class_Power_Limit::Set_Chassis_Buffer(float __buffer)
// {
//     Chassis_Buffer = __buffer;
// }

// /**
//  * @brief 设定超级电容当前剩余能量
//  *
//  */
// void Class_Power_Limit::Set_Supercap_Enegry(float __energy)
// {
//    Supercap_Energy = __energy;
// }

// /**
//  * @brief 设定超级电容当前电压
//  *
//  */
// void Class_Power_Limit::Set_Supercap_Voltage(float __voltage)
// {
//     Supercap_Voltage = __voltage;
// }

// /**
//  * @brief 设定超级电容冲刺标志
//  *
//  */
// void Class_Power_Limit::Set_Supercap_Print_Flag(uint8_t __flag)
// {
//     Supercap_Print_Flag = __flag;
// }

/* Exported types ------------------------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
