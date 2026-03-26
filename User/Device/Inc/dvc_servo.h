/**
 * @file dvc_servo.h
 * @author lez by yssickjgd
 * @brief PWM舵机配置与操作
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/**
 * @brief 舵机配置与操作
 * 0.5ms~2.5ms对应arr为50~250, 中点是150
 * 本文件部分单位是角度制
 * 
 */

#ifndef DVC_SERVO_H
#define DVC_SERVO_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 舵机
 * 
 */
class Class_Servo
{
public:
    void Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel, float __Max_Angle = PI);
    inline void Set_Target_Angle(float __Target_Angle);

protected:
    //初始化相关常量

    //舵机驱动定时器编号
    TIM_HandleTypeDef *Driver_PWM_TIM;
    //定时器通道
    uint8_t Driver_PWM_TIM_Channel;
    //舵机可动范围, 默认180角度舵机
    float Max_Angle;

    //常量

    //内部变量

    //读变量

    //写变量

    //舵机角度目标值
    float Target_Angle = 0.0f;

    //读写变量

    //内部函数
    
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设定舵机角度目标值
 * 
 * @param __Target_Angle 舵机角度目标值
 */
void Class_Servo::Set_Target_Angle(float __Target_Angle)
{
    Math_Constrain(&__Target_Angle, 0.0f, Max_Angle);
    Target_Angle = __Target_Angle;
    Output();
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
