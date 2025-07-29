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

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    Motor_Yaw.PID_Angle.Init(30.f, 0.0f, 0.0f, 0.0f, 500, 500);
    Motor_Yaw.PID_Omega.Init(60.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_Yaw.Get_Output_Max(), 10.f, 50.f);
    Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);
    
    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(22.f, 0.0f, 0.001f, 0.0f, 2.f, 650.f);
    Motor_Pitch.PID_Omega.Init(90.0f, 20.0f, 0.0f, 0.0f, 6000, Motor_Pitch.Get_Output_Max(),0.f,0.f,40.f);
    Motor_Pitch.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
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
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);


        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);

        Motor_Yaw.Set_Out(0.0f);
        Motor_Pitch.Set_Out(0.0f);

    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());
            Motor_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
           
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {

            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

        }
    }
}
void Class_Gimbal_Pitch_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环

        //			if(True_Angle_Pitch>=15){
        //			Target_Omega_Angle=-test_omega;
        //			}
        //			if(True_Angle_Pitch<=-15){
        //				Target_Omega_Angle=test_omega;
        //			}

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        // PID_Angle.Set_Target(-m_angle);
        PID_Angle.Set_Target(-Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 57.3);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();
        Set_Out(-PID_Omega.Get_Out() + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();	
}
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环

        //			if(True_Angle_Pitch>=15){
        //			Target_Omega_Angle=-test_omega;
        //			}
        //			if(True_Angle_Pitch<=-15){
        //				Target_Omega_Angle=test_omega;
        //			}

        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        // PID_Angle.Set_Target(-m_angle);
        PID_Angle.Set_Target(-Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw * 57.3);
        }
        else
        {
            // 角度环
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = -PID_Omega.Get_Out();
        Set_Out(-PID_Omega.Get_Out());
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();	
}
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
