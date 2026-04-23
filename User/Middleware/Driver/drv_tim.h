/**
 * @file drv_tim.h
 * @author lez by yssickjgd
 * @brief TIM定时器初始化与配置流程, 仅有中断回调函数参9
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DRV_TIM_H
#define DRV_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*TIM_Call_Back)();

/**
 * @brief TIM定时器处理结构体
 *
 */
struct Struct_TIM_Manage_Object
{
    TIM_HandleTypeDef *TIM_Handler;
    TIM_Call_Back Callback_Function;
};

/* Exported variables --------------------------------------------------------*/



extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim12;

extern Struct_TIM_Manage_Object TIM1_Manage_Object;
extern Struct_TIM_Manage_Object TIM2_Manage_Object;
extern Struct_TIM_Manage_Object TIM3_Manage_Object;
extern Struct_TIM_Manage_Object TIM4_Manage_Object;
extern Struct_TIM_Manage_Object TIM5_Manage_Object;
extern Struct_TIM_Manage_Object TIM6_Manage_Object;
extern Struct_TIM_Manage_Object TIM7_Manage_Object;
extern Struct_TIM_Manage_Object TIM8_Manage_Object;
extern Struct_TIM_Manage_Object TIM9_Manage_Object;
extern Struct_TIM_Manage_Object TIM10_Manage_Object;
extern Struct_TIM_Manage_Object TIM11_Manage_Object;
extern Struct_TIM_Manage_Object TIM12_Manage_Object;
extern Struct_TIM_Manage_Object TIM13_Manage_Object;
extern Struct_TIM_Manage_Object TIM14_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void TIM_Init(TIM_HandleTypeDef *htim, TIM_Call_Back Callback_Function);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
