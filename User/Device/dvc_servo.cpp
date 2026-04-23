// /**
//  * @file dvc_servo.cpp
//  * @author lez by yssickjgd
//  * @brief PWM舵机配置与操作
//  * @version 0.1
//  * @date 2024-07-1 0.1 24赛季定稿
//  *
//  * @copyright ZLLC 2024
//  *
//  */

// /**
//  * @brief 注意定时器配置需要保证arr为1000, psc根据整体频率为100Hz进行调整
//  * 即一份时间片0.01ms, 对应270角度舵机是1.35角度
//  * 0.5ms~2.5ms对应arr为50~250, 中点是150
//  * 虽然本文件部分单位是角度制, 但为统一, 所有对外接口采用国际单位制的弧度制
//  * 
//  */

// /* Includes ------------------------------------------------------------------*/

 #include "dvc_servo.h"

// /* Private macros ------------------------------------------------------------*/

// /* Private types -------------------------------------------------------------*/

// /* Private variables ---------------------------------------------------------*/

// /* Private function declarations ---------------------------------------------*/

// /* Function prototypes -------------------------------------------------------*/

// /**
//  * @brief 舵机初始化
//  * 
//  * @param __Driver_PWM_TIM 舵机驱动定时器编号
//  * @param __Driver_PWM_TIM_Channel_x 定时器通道
//  * @param __Angle_Offset 舵机角度偏移量, 默认0
//  * @param __Max_Angle 舵机双边可动范围, 默认270角度舵机, 记得除以二, 也就是对应两侧可动范围都是3/4 PI
//  */
// void Class_Servo::Init(TIM_HandleTypeDef *__Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel, float __Max_Angle)
// {
//     Driver_PWM_TIM = __Driver_PWM_TIM;
//     Driver_PWM_TIM_Channel = __Driver_PWM_TIM_Channel;
//     Max_Angle = __Max_Angle;

//     HAL_TIM_PWM_Start(__Driver_PWM_TIM, __Driver_PWM_TIM_Channel);
// }

// /**
//  * @brief 角度输出
//  * 
//  */
// void Class_Servo::Output()
// {
//     uint16_t out;

//     out = 150.0f + (Target_Angle - Max_Angle / 2.0f) / (Max_Angle / 2.0f) * 100.0f;

//     __HAL_TIM_SetCompare(Driver_PWM_TIM, Driver_PWM_TIM_Channel, out);
// }

// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
