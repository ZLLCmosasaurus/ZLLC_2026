/**
 * @file dvc_sampler.cpp
 * @author lez by yssickjgd
 * @brief ADC采样器
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_sampler.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 采样器初始化
 * 
 * @param hadc 指定的ADC
 * @param __Sampler_Serial 采样点序列号, 从0开始
 * @param __Sampler_Filter 采用的滤波器
 */
//void Class_Sampler::Init(ADC_HandleTypeDef *hadc, uint16_t __Sampler_Serial, Enum_Sampler_Filter __Sampler_Filter)
//{
//    if (hadc->Instance == ADC1)
//    {
//        ADC_Manage_Object = &ADC1_Manage_Object;
//    }
//    else if (hadc->Instance == ADC2)
//    {
//        ADC_Manage_Object = &ADC2_Manage_Object;
//    }
//    else if (hadc->Instance == ADC3)
//    {
//        ADC_Manage_Object = &ADC3_Manage_Object;
//    }
//    Sampler_Serial = __Sampler_Serial;
//    Sampler_Filter = __Sampler_Filter;

//    ADC_Value = &ADC_Manage_Object->ADC_Data[Sampler_Serial];
//}

/**
 * @brief TIM定时器中断计算滤波器回调函数
 *
 */
void Class_Sampler::TIM_Sampler_PeriodElapsedCallback()
{
    //选择滤波器, 若没有则直接输出
    switch (Sampler_Filter)
    {
    case (Sampler_Filter_NULL):
    {
        Value = *ADC_Value / 4096.0f;
    }
    break;
    case (Sampler_Filter_KALMAN):
    {
        Filter_Kalman.Set_Now(*ADC_Value / 4096.0f);
        Filter_Kalman.Recv_Adjust_PeriodElapsedCallback();
        Value = Filter_Kalman.Get_Out();
    }
    break;
    case (Sampler_Filter_FOURIER):
    {
        Filter_Fourier.Set_Now(*ADC_Value / 4096.0f);
        Filter_Fourier.TIM_Adjust_PeriodElapsedCallback();
        Value = Filter_Fourier.Get_Out();
    }
    break;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
