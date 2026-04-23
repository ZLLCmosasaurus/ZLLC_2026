/**
 * @file alg_slope.h
 * @author lez by yssickjgd
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

#define STATUS_MAX (10)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
class Class_Slope
{
public:
    void Init(float __Increase_Value, float __Decrease_Value);

    inline float Get_Out();
    inline float Get_Decrease_Value();
    inline float Get_Increase_Value();
    inline float Get_Default_Increase_Value();
    inline float Get_Default_Decrease_Value();

    inline void Set_Increase_Value(float __Increase_Value);
    inline void Set_Decrease_Value(float __Decrease_Value);
    inline void Set_Target(float __Target);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绝对值增量, 一次计算周期改变值  1ms为一个周期
    float Increase_Value;
   //绝对值减量, 一次计算周期改变值  1ms为一个周期
    float Decrease_Value;

    //常量

    //内部变量
    //初始化的加减速值
    float Default_Increase_Value;  //1ms为一个周期
    float Default_Decrease_Value;   //1ms为一个周期

    //当前值
    float Now = 0.0f;

    //读变量

    //输出值
    float Out = 0.0f;

    //写变量

    //目标值
    float Target = 0.0f;

    //读写变量

    //内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 */
float Class_Slope::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
void Class_Slope::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief 设置增量值
 *
 * @param __Increase_Value 增量值
 */
void Class_Slope::Set_Increase_Value(float __Increase_Value)
{
    Increase_Value = __Increase_Value;
}

/**
 * @brief 设置减量值
 *
 * @param __Decrease_Value 减量值
 */
void Class_Slope::Set_Decrease_Value(float __Decrease_Value)
{
    Decrease_Value = __Decrease_Value;
}

/**
 * @brief 获取增量值
 *
 * @return float 增量值
 */
float Class_Slope::Get_Increase_Value()
{
    return Increase_Value;
}

/**
 * @brief 获取减量值
 *
 * @return float 减量值
 */
float Class_Slope::Get_Decrease_Value()
{
    return Decrease_Value;
}


/**
 * @brief 获取默认增量值
 *
 * @return float 默认增量值
 */
float Class_Slope::Get_Default_Increase_Value()
{
    return Default_Increase_Value;
}


/**
 * @brief 获取默认减量值
 *
 * @return float 默认减量值
 */
float Class_Slope::Get_Default_Decrease_Value()
{
    return Default_Decrease_Value;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
