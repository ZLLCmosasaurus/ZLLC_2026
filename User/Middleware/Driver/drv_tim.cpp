/**
 * @file drv_tim.cpp
 * @author lez by yssickjgd
 * @brief TIM定时器初始化与配置流程, 仅有中断回调函数参9
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_tim.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_TIM_Manage_Object TIM1_Manage_Object;
Struct_TIM_Manage_Object TIM2_Manage_Object;
Struct_TIM_Manage_Object TIM3_Manage_Object;
Struct_TIM_Manage_Object TIM4_Manage_Object;
Struct_TIM_Manage_Object TIM5_Manage_Object;
Struct_TIM_Manage_Object TIM6_Manage_Object;
Struct_TIM_Manage_Object TIM7_Manage_Object;
Struct_TIM_Manage_Object TIM8_Manage_Object;
Struct_TIM_Manage_Object TIM9_Manage_Object;
Struct_TIM_Manage_Object TIM10_Manage_Object;
Struct_TIM_Manage_Object TIM11_Manage_Object;
Struct_TIM_Manage_Object TIM12_Manage_Object;
Struct_TIM_Manage_Object TIM13_Manage_Object;
Struct_TIM_Manage_Object TIM14_Manage_Object;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化TIM定时器
 *
 * @param htim 定时器编号
 * @param Callback_Function 处理回调函数
 */
void TIM_Init(TIM_HandleTypeDef *htim, TIM_Call_Back Callback_Function)
{
    if (htim->Instance == TIM1)
    {
        TIM1_Manage_Object.TIM_Handler = htim;
        TIM1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM2)
    {
        TIM2_Manage_Object.TIM_Handler = htim;
        TIM2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Manage_Object.TIM_Handler = htim;
        TIM3_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM4)
    {
        TIM4_Manage_Object.TIM_Handler = htim;
        TIM4_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM5)
    {
        TIM5_Manage_Object.TIM_Handler = htim;
        TIM5_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM6)
    {
        TIM6_Manage_Object.TIM_Handler = htim;
        TIM6_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM7)
    {
        TIM7_Manage_Object.TIM_Handler = htim;
        TIM7_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM8)
    {
        TIM8_Manage_Object.TIM_Handler = htim;
        TIM8_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM9)
    {
        TIM9_Manage_Object.TIM_Handler = htim;
        TIM9_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM10)
    {
        TIM10_Manage_Object.TIM_Handler = htim;
        TIM10_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM11)
    {
        TIM11_Manage_Object.TIM_Handler = htim;
        TIM11_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM12)
    {
        TIM12_Manage_Object.TIM_Handler = htim;
        TIM12_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM13)
    {
        TIM13_Manage_Object.TIM_Handler = htim;
        TIM13_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (htim->Instance == TIM14)
    {
        TIM14_Manage_Object.TIM_Handler = htim;
        TIM14_Manage_Object.Callback_Function = Callback_Function;
    }
}

/**
 * @brief HAL库TIM定时器中断
 *
 * @param htim TIM编号
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // //判断程序初始化完成
    // if(init_finished == 0)
    // {
    //     return;
    // }

    //选择回调函数
    if (htim->Instance == TIM1)
    {
        TIM1_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM2)
    {
        TIM2_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM4)
    {
        TIM4_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM5)
    {
        TIM5_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM6)
    {
        TIM6_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM7)
    {
        TIM7_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM8)
    {
        TIM8_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM9)
    {
        TIM9_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM10)
    {
        TIM10_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM11)
    {
        TIM11_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM12)
    {
        TIM12_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM13)
    {
        TIM13_Manage_Object.Callback_Function();
    }
    else if (htim->Instance == TIM14)
    {
        TIM14_Manage_Object.Callback_Function();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
