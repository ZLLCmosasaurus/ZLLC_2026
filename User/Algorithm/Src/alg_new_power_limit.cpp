/**
 * @file alg_power_limit.h
 * @author zxf
 * @brief 西交功率限制算法
 * @version 1.0
 * @date
 * 以舵轮功率限制为例，请结合飞书电控知识库-电控进阶-功率限制文档理解。
 * 这里强调一些注意事项：我们的限制对象是3508电机，轮电机一般具备减速比，
 * 为了统一计算，因此我们采用对转子进行功率限制，那么pidout与omega都要进行减速比换算与量纲转换
 * 1.预测功率
 * void Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
 * 换算公式如下：
 * Omega为：除以减速比的速度，单位为rpm
 * torque = pidout*M3508_CMD_CURRENT_TO_TORQUE
 * 2.功率分配
 * 3.限制功率
 *
 * @copyright ZLLC 2026
 *
 */
#include "alg_new_power_limit.h"
#include "math.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }
static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }

/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机，舵轮需要传此参数
 * @return float 理论功率值
 */
// 西交功率模型
float Class_New_Power_Limit::Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
{

    float cmdPower = rpm2av(omega) * torque +
                     fabs(rpm2av(omega)) * k1 +
                     torque * torque * k2 +
                     k3;

    return cmdPower;
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
float Class_New_Power_Limit::Calculate_Toque(float omega, float power, float torque, uint8_t motor_index)
{

    omega = rpm2av(omega);
    float newTorqueCurrent = 0.0f;

    float delta = omega * omega - 4 * (k1 * fabs(omega) + k3 - power) * k2;

    if (floatEqual(delta, 0.0f))
    {
        newTorqueCurrent = -omega / (2.0f * k2);
    }
    else if (delta > 0.0f)
    {
        float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
        float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);

        newTorqueCurrent = (torque > 0) ? solution1 : solution2;
    }
    else // delta < 0
    {
        newTorqueCurrent = -omega / (2.0f * k2);
    }

    return newTorqueCurrent;
}
/**
 * @brief 计算每个电机分配的功率
 * @param Motor_Data 电机结构体
 * @param __Total_error 目标值与实际值绝对值误差加和
 * @param Max_Power 限制的最大功率
 * @param __Scale_Conffient 功率收缩因子
 */
void Class_New_Power_Limit::Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient)
{
    Motor_Data.scaled_power = Motor_Data.theoretical_power *
                              __Scale_Conffient;
}
/**
 * @brief 功率限制主任务
 *
 * @param power_management 功率管理结构体
 */
float motmotor_predic_power = 0.f;
float mot_needScaled_power = 0.f;
float mot_max_power = 0.0f;
float power_pid_out = 0.0f, pre_pid_out = 0.0f;
void Class_New_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
    float tmp_motmotor_predic_power = 0.0f;
    float tmp_mot_needScaled_power = 0.f;

    for (int i = 0; i < 4; i++)
    {
        power_management.Motor_Data[i].theoretical_power = Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega, power_management.Motor_Data[i].torque, i);
        tmp_motmotor_predic_power += power_management.Motor_Data[i].theoretical_power;

        if (power_management.Motor_Data[i].theoretical_power > 0.0f)
        {
            tmp_mot_needScaled_power += power_management.Motor_Data[i].theoretical_power;
        }
        else
        {
            tmp_mot_needScaled_power += 0.0f;
        }
    }
#ifdef Enable_Power_Buffer_Loop // 有回传实际功率的功率限制
    // 只有在启用功率缓冲时才执行
    Control_Status = 1;
#else // 无回传实际功率的功率限制
    // 禁用时的替代方案
    Control_Status = 0;
#endif
    if (Control_Status == 1)
    {
        power_pid_out = (power_management.Actual_Power - power_management.Max_Power + 1.f) * 5.0f;
        power_pid_out = (pre_pid_out + power_pid_out) / 2.0f;
        if (power_pid_out < 0.0f)
            power_pid_out = 0.0f;
        if (power_pid_out > 60.0f)
            power_pid_out = 60.0f;
        pre_pid_out = power_pid_out;
    }
    else
    {
        power_pid_out = 0;
    }
    motmotor_predic_power = tmp_motmotor_predic_power + power_pid_out;
    mot_needScaled_power = tmp_mot_needScaled_power + power_pid_out;
    // 计算理论总功率
    power_management.Theoretical_Total_Power = motmotor_predic_power;
    // 计算需要收缩的理论总功率
    power_management.Needed_Scaled_Theoretical_Total_Power = mot_needScaled_power;

    // 上层功率分配
    Power_Allocate(power_management.Max_Power, motmotor_predic_power, &mot_max_power);
    // 下层功率限制

    if (mot_max_power >= motmotor_predic_power) // 计算行进收缩系数
    {
        for (int j = 0; j < 4; j++)
        {
            power_management.Motor_Data[j].output = power_management.Motor_Data[j].pid_output;
        }
    }
    else
    {
        power_management.Scale_Conffient = mot_max_power / mot_needScaled_power;
        for (int i = 0; i < 4; i++)
        {
            if (power_management.Motor_Data[i].theoretical_power < 0.0f)
            {
                power_management.Motor_Data[i].output = power_management.Motor_Data[i].pid_output;
                continue;
            }

            Calulate_Power_Allocate(power_management.Motor_Data[i], 0,
                                    power_management.Max_Power, power_management.Scale_Conffient);

            power_management.Motor_Data[i].output =
                Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                                power_management.Motor_Data[i].scaled_power,
                                power_management.Motor_Data[i].torque,
                                i) *
                MOT_TORQUE_TO_CMD_CURRENT;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if ((power_management.Motor_Data[i].output) >= 16384)
        {
            power_management.Motor_Data[i].output = 16384;
        }

        if ((power_management.Motor_Data[i].output) <= -16384)
        {
            power_management.Motor_Data[i].output = -16384;
        }
    }
}
void Class_New_Power_Limit::Power_Allocate(float Power_Limit, float rate, float dir_predict_power, float mot_predict_power, float *dir_power_allocate, float *mot_power_allocate)
{
    // 新的功率分配逻辑
    float dir_power_limit = Power_Limit * rate;         // 转向电机功率上限
    float mot_power_limit = Power_Limit * (1.f - rate); // 动力电机功率上限，动态计算

    // 实际分配到的功率
    float dir_allocate, mot_allocate;

    // 首先分配转向电机功率
    if (dir_predict_power > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        dir_allocate = dir_power_limit;
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        dir_allocate = dir_predict_power;
    }
    // 优先分配转向舵
    mot_power_limit = Power_Limit - dir_allocate;
    // 然后分配动力电机功率
    if (mot_predict_power > mot_power_limit)
    {
        mot_allocate = mot_power_limit;
    }
    else
    {
        mot_allocate = mot_predict_power;
    }
    // 赋值给成员变量
    *dir_power_allocate = dir_allocate;
    *mot_power_allocate = mot_allocate;
}

void Class_New_Power_Limit::Power_Allocate(float Power_Limit, float mot_predict_power, float *mot_power_allocate)
{
    // 新的功率分配逻辑
    float mot_power_limit = Power_Limit;

    // 实际分配到的功率
    float mot_allocate;

    // 分配动力电机功率
    if (mot_predict_power > mot_power_limit)
    {
        mot_allocate = mot_power_limit;
    }
    else
    {
        mot_allocate = mot_predict_power;
    }
    // 赋值给成员变量
    *mot_power_allocate = mot_allocate;
}