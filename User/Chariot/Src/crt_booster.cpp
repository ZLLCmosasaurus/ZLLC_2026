/**
 * @file crt_booster.cpp
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
void Class_FSM_Shooting::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向左堵转
        {

        }
        break;
        case (1)://左侧检测
        {

        }
        break;
        case (2)://向右堵转
        {

        }
        break;
        case (3)://右侧检测
        {

        }
        break;
        case (4)://正常控制流程
        {

        }
        break;
    }
}

/**
 * @brief 发射机构初始化
 *
 */

void Class_Booster::Init()
{
    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(9, 0);

    FSM_Push_Calibration.Booster = this;
    FSM_Push_Calibration.Init(4, 0);

    //拉力电机
    Motor_Pull.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pull.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pull.Get_Output_Max(), Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Push_L.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Push_L.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Push_L.Get_Output_Max(), Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

    Motor_Push_R.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Push_R.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Push_R.Get_Output_Max(), Motor_Push_R.Get_Output_Max());
    Motor_Push_R.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA);

}

/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    //控制拨弹轮
    switch (Booster_Control_Type)
    {
        case (Booster_Control_Type_DISABLE):
        {
            Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

            Motor_Pull.Set_Target_Torque(0.f);
            Motor_Push_L.Set_Target_Torque(0.f);
            Motor_Push_R.Set_Target_Torque(0.f);

            Motor_Pull.Set_Out(0.f);
            Motor_Push_L.Set_Out(0.f);
            Motor_Push_R.Set_Out(0.f);
        }
        break;
    }

}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{     
    //PID输出
    Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
