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

#include "alg_new_power_limit.h"
//#include "alg_power_limit.h"
#include "math.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f;}           //omega 转换成 rpm
static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }

/**
 * @brief 
 * @param __ErrorLow 低于Error会进行等比分配
 * @param __ErrorUp 高于error，直接按error分配功率，防止功率乱吃，分配错误
 */
void Class_New_Power_Limit::Init(float __ErrorLow, float __ErrorUp)
{
#ifdef AGV
    ErrorLow = __ErrorLow;
    ErrorUp = __ErrorUp;
#else
    ErrorLow = __ErrorLow;
    ErrorUp = __ErrorUp;
    float initParams[2] = {k1, k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
#endif
}

/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机，舵轮需要传此参数
 * @return float 理论功率值
 */
float Class_New_Power_Limit::Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
{

    float cmdPower = rpm2av(omega) * torque +
                     fabs(rpm2av(omega)) * k1 +
                     torque * torque * k2 +
                     k3;

    return cmdPower;
}

/**
 * @brief 计算功率系数
 *
 * @param actual_power 实际功率
 * @param motor_data 电机数据数组
 */
void Class_New_Power_Limit::Calculate_Power_Coefficient(float actual_power, const Struct_Power_Motor_Data *motor_data)
{
#ifdef AGV
    // static Matrixf<2, 1> samples_mot, samples_dir;
    // static Matrixf<2, 1> params_mot, params_dir;
    // float effectivePower_mot = 0, effectivePower_dir = 0;

    // samples_mot[0][0] = samples_mot[1][0] = 0;
    // samples_dir[0][0] = samples_dir[1][0] = 0;

    // if (actual_power > 5)
    // {
    //     // 分别处理动力电机(奇数索引)和转向电机(偶数索引)
    //     for (int i = 0; i < 8; i++)
    //     {
    //         if (i % 2 == 0) // 转向电机
    //         {
    //             if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
    //             {
    //                 effectivePower_dir += motor_data[i].feedback_torque *
    //                                       rpm2av(motor_data[i].feedback_omega);
    //             }
    //             samples_dir[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
    //             samples_dir[1][0] += motor_data[i].feedback_torque *
    //                                  motor_data[i].feedback_torque;
    //         }
    //         else // 动力电机
    //         {
    //             if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
    //             {
    //                 effectivePower_mot += motor_data[i].feedback_torque *
    //                                       rpm2av(motor_data[i].feedback_omega);
    //             }
    //             samples_mot[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
    //             samples_mot[1][0] += motor_data[i].feedback_torque *
    //                                  motor_data[i].feedback_torque;
    //         }
    //     }

    //     // 更新RLS参数
    //     float power_ratio = 0.8f; // 动力电机功率占比
    //     params_mot = rls_mot.update(samples_mot,
    //                                 power_ratio * actual_power - effectivePower_mot - 4 * k3_mot);
    //     params_dir = rls_dir.update(samples_dir,
    //                                 (1 - power_ratio) * actual_power - effectivePower_dir - 4 * k3_dir);

    //     // 更新系数
    //     k1_mot = my_fmax(params_mot[0][0], 1e-5f);
    //     k2_mot = my_fmax(params_mot[1][0], 1e-5f);
    //     k1_dir = my_fmax(params_dir[0][0], 1e-5f);
    //     k2_dir = my_fmax(params_dir[1][0], 1e-5f);
    // }
#else
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    float effectivePower = 0;

    samples[0][0] = samples[1][0] = 0;

    if (actual_power > 5)
    {
        for (int i = 0; i < 4; i++)
        {
            // if (motor_data[i].feedback_torque * rpm2av(motor_data[i].feedback_omega) > 0)
            // {
                
            // }
            effectivePower += motor_data[i].feedback_torque *
                                  rpm2av(motor_data[i].feedback_omega);
            samples[0][0] += fabsf(rpm2av(motor_data[i].feedback_omega));
            samples[1][0] += motor_data[i].feedback_torque *
                             motor_data[i].feedback_torque;
        }

        params = rls.update(samples, actual_power - effectivePower - 4 * k3);
        k1 = my_fmax(params[0][0], 1e-7f);
        k2 = my_fmax(params[1][0], 1e-7f);
    }
#endif
}

/**
 * @brief 大P计算，计算每个电机分配的功率
 * @param Motor_Data 电机结构体
 * @param __Total_error 目标值与实际值绝对值误差加和
 * @param Max_Power 限制的最大功率
 * @param __Scale_Conffient 功率收缩因子
 */
void Class_New_Power_Limit::Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient)
{
    Motor_Data.scaled_power = Motor_Data.theoretical_power *
                                    __Scale_Conffient;
    //按照误差分配
    // if(__Total_error <= ErrorLow ){
    //     Motor_Data.scaled_power = Motor_Data.theoretical_power *
    //                                 __Scale_Conffient;
    // }
    // else if(__Total_error >= ErrorUp){
    //     Motor_Data.scaled_power = Max_Power * 
    //                                 (Motor_Data.Target_error/__Total_error);
    // }
    // else{           //处于中间线性变化分配策略
    //     float Kp = 0.0f;
    //     Kp = (__Total_error-ErrorLow) / (ErrorUp - ErrorLow);

    //     Motor_Data.scaled_power =
    //         (1-Kp) * Motor_Data.theoretical_power *
    //         __Scale_Conffient   +    Kp * Max_Power * (Motor_Data.Target_error/__Total_error);
    // }
}

/**
 * @brief 计算限制后的扭矩
 *
 * @param omega 转子转速，单位为rpm
 * @param power 限制功率值
 * @param torque 原始扭矩值
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机
 * @return float 限制后的扭矩值
 */

uint8_t test_flag=0;
float solution1;
float solution2;
float test_cal;
float Class_New_Power_Limit::Calculate_Toque(float omega, float power, float torque, uint8_t motor_index)
{
    omega = rpm2av(omega);
    float newTorqueCurrent = 0.0f;

    float delta = omega * omega - 4 * (k1 * fabs(omega) + k3 - power) * k2;

    if(delta < 0.0f)
    {
        newTorqueCurrent = 0.0f; //-omega / (2.0f * k2);
    }
    else 
    {
        float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
        float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);
        if ((solution1 > 0.0f && solution2 < 0.0f) || (solution1 < 0.0f && solution2 > 0.0f))
        {
            if ((torque > 0.0f && solution1 > 0.0f) || (torque < 0.0f && solution1 < 0.0f))
            {
                newTorqueCurrent = solution1;
            }
            else
            {
                newTorqueCurrent = solution2;
            }
        }
        else
        {
            if (Math_Abs(solution1) < Math_Abs(solution2))
            {
                newTorqueCurrent = solution1;
            }
            else
            {
                newTorqueCurrent = solution2;
            }
        }

        // newTorqueCurrent = (torque > 0) ? solution1 : solution2;
    }

    return newTorqueCurrent;
}

/**
 * @brief 功率限制主任务
 *
 * @param power_management 功率管理结构体
 */
void Class_New_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
#ifdef AGV
    //需要分配的理论转向舵，行进舵功率，和预测的真实功率
    float theoretical_sum_mot = 0, theoretical_sum_dir = 0, theoretical_sum = 0;
    float scaled_sum_mot = 0, scaled_sum_dir = 0;

    // 分别计算动力和转向电机的理论功率
    for (uint8_t i = 0; i < 8; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);
        theoretical_sum += power_management.Motor_Data[i].theoretical_power;
        //只计入大于0的功率，小于0的不参与分配
        if (power_management.Motor_Data[i].theoretical_power > 0)
        {
            if (i >= 4) // 转向电机
            {
                theoretical_sum_dir += power_management.Motor_Data[i].theoretical_power;
            }
            else // 动力电机
            {
                theoretical_sum_mot += power_management.Motor_Data[i].theoretical_power;
            }
        }
    }

    // 新的功率分配逻辑
    float scale_mot = 1.0f, scale_dir = 1.0f;
    float dir_power_limit = power_management.Max_Power * 0.8f; // 转向电机功率上限
    float mot_power_limit = 0.0f;                                     // 动力电机功率上限，动态计算

    // 首先分配转向电机功率
    if (theoretical_sum_dir > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        scale_dir = dir_power_limit / theoretical_sum_dir;
        mot_power_limit = power_management.Max_Power - dir_power_limit;
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        scale_dir = 1.0f;
        // 剩余功率全部分配给动力电机
        mot_power_limit = power_management.Max_Power - theoretical_sum_dir;
    }

    // 然后分配动力电机功率
    if (theoretical_sum_mot > mot_power_limit)
    {
        scale_mot = mot_power_limit / theoretical_sum_mot;
    }
    else
    {
        scale_mot = 1.0f;
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 8; i++)
    {
        float scale = (i >= 4) ? scale_dir : scale_mot;

        if(power_management.Motor_Data[i].theoretical_power < 0){
            power_management.Motor_Data[i].scaled_power = power_management.Motor_Data[i].theoretical_power;
            power_management.Motor_Data[i].output       = power_management.Motor_Data[i].pid_output;
            continue;
        }
        else{
            power_management.Motor_Data[i].scaled_power =
                power_management.Motor_Data[i].theoretical_power * scale;
        }

        if (i >= 4)
        {
            scaled_sum_dir += power_management.Motor_Data[i].scaled_power;
        }
        else
        {
            scaled_sum_mot += power_management.Motor_Data[i].scaled_power;
        }

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            GET_TORQUE_TO_CMD_CURRENT;

        // 限幅处理
        power_management.Motor_Data[i].output =
            (power_management.Motor_Data[i].output > 16384) ? 16384: (power_management.Motor_Data[i].output < -16384) ? -16384
                                                                                                                       : power_management.Motor_Data[i].output;
    }

    power_management.Theoretical_Total_Power = theoretical_sum;
    power_management.Scaled_Total_Power = scaled_sum_mot + scaled_sum_dir;

    //Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
#else
    float theoretical_sum = 0.0f;                                       //预测的总功率
    float NeedScaled_theoretical_sum = 0.0f;                            //需要分配的总功率
    float scaled_sum = 0.0f;                                            

    // 计算理论功率
    for (uint8_t i = 0; i < 4; i++)
    {
        power_management.Motor_Data[i].theoretical_power =
            Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega,
                                        power_management.Motor_Data[i].torque,
                                        i);
        theoretical_sum += power_management.Motor_Data[i].theoretical_power;
        if (power_management.Motor_Data[i].theoretical_power > 0)
        {
            NeedScaled_theoretical_sum += power_management.Motor_Data[i].theoretical_power;
            power_management.Total_error += power_management.Motor_Data[i].Target_error;                //只有消耗功率的电机才计入误差做功率再分配
        }
        else
        {
            NeedScaled_theoretical_sum += 0.0;
        }
    }
    power_management.Theoretical_Total_Power = theoretical_sum;
    power_management.Needed_Scaled_Theoretical_Total_Power = NeedScaled_theoretical_sum;

    //判断是否需要功率再分配
    if(power_management.Max_Power >= power_management.Theoretical_Total_Power){
        for (uint8_t i = 0; i < 4; i++){
            power_management.Motor_Data[i].output = power_management.Motor_Data[i].pid_output;
        }
        return;         //函数直接结束即可
    }
    else{
        // 计算收缩系数
        power_management.Scale_Conffient =
                power_management.Max_Power / (power_management.Needed_Scaled_Theoretical_Total_Power);
    }

    // 应用收缩系数并更新输出
    for (uint8_t i = 0; i < 4; i++)
    {
        //反向电动势的功率作为缓冲使用，而不是直接利用
        //当前电机是反向放电的话不参与功率再分配
        if(power_management.Motor_Data[i].theoretical_power < 0){
            power_management.Motor_Data[i].output = power_management.Motor_Data[i].pid_output;
            continue;
        }

        //当前电机需要进行功率再分配
        Calulate_Power_Allocate(power_management.Motor_Data[i], power_management.Total_error, 
                                power_management.Max_Power, power_management.Scale_Conffient);

        scaled_sum += power_management.Motor_Data[i].scaled_power;

        power_management.Motor_Data[i].output =
            Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                            power_management.Motor_Data[i].scaled_power,
                            power_management.Motor_Data[i].torque,
                            i) *
            GET_TORQUE_TO_CMD_CURRENT;

        if ((power_management.Motor_Data[i].output) >= 16384)
        {
            power_management.Motor_Data[i].output = 16384;
        }
		if ((power_management.Motor_Data[i].output) <= -16384)
        {
            power_management.Motor_Data[i].output = -16384;
        }
    }

    power_management.Scaled_Total_Power = scaled_sum;           //没啥用的变量，实际上等于需要再分配的理论总功率

    //RLS更新参数
    //Calculate_Power_Coefficient(power_management.Actual_Power, power_management.Motor_Data);
    // if((power_management.Motor_Data[0].feedback_torque > 0.01f && power_management.Motor_Data[0].feedback_omega < 200)||
    //    (power_management.Motor_Data[0].feedback_torque > 0.01f && power_management.Motor_Data[0].feedback_omega < 200)||
    //    (power_management.Motor_Data[0].feedback_torque > 0.01f && power_management.Motor_Data[0].feedback_omega < 200)||
    //    (power_management.Motor_Data[0].feedback_torque > 0.01f && power_management.Motor_Data[0].feedback_omega < 200))
    // {
    //     Set_K2(2000.f);
    // }
    // else
    // {
    //     Set_K2(530.f);
    // }
#endif
}