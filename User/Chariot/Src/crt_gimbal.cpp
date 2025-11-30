/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
void Class_FSM_Yaw_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向左堵转
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(2.0f);
            if(abs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://左侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Left = Gimbal->Motor_Yaw.Get_Now_Angle();
                Set_Status(2);
            }
            else{
                Set_Status(0);
            }
        }
        break;
        case (2)://向右堵转
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(-2.0f);
            if(abs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }
        }
        break;
        case (3)://右侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Right = Gimbal->Motor_Yaw.Get_Now_Angle();
                Set_Status(4);
            }
            else{
                Set_Status(2);
            }
        }
        break;
        case (4)://正常控制流程
        {
            Gimbal->Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }
        break;
    }
}
/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    FSM_Yaw_Calibration.Init(5,0);

    Motor_Pitch_L.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_L.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_L.Get_Output_Max(), Motor_Pitch_L.Get_Output_Max());
    Motor_Pitch_L.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Pitch_R.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_R.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_R.Get_Output_Max(), Motor_Pitch_R.Get_Output_Max());
    Motor_Pitch_R.Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

    Motor_Yaw.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Yaw.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA);
}


/**
 * @brief 输出到电机
 *
 */

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch_L.Set_Target_Torque(0.0f);
        Motor_Pitch_R.Set_Target_Torque(0.0f);

        Motor_Yaw.Set_Out(0.f);
        Motor_Pitch_L.Set_Out(0.f);
        Motor_Pitch_R.Set_Out(0.f);

    }
    else if(Gimbal_Control_Type == Gimbal_Control_Type_YAW_CALIBRATION)// 非失能模式
    {
        
    }
    else if(Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
    {
        
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{

    FSM_Yaw_Calibration.Reload_TIM_Status_PeriodElapsedCallback();

    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_L.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
