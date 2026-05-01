/**
 * @file crt_gimbal.cpp
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
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
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_angle = 0;
float Test_Target_Omega = 0;
float last_angle = 0;

float Get_Friction_Compensation(float omega, float omega_th)
{
    if (Math_Abs(omega) >= omega_th)
        return (omega > 0 ? 1 : -1);
    else
    {
        return omega / omega_th;
    }
}

void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环速度控制
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
        PID_Omega.Set_Target(Target_Omega_Angle);
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
        // PID_Angle.Set_Target(Target_Angle);
        //  Target_Angle=test_angle;
        if (last_angle != Target_Angle)
        {
            PID_Angle.Set_Target(Target_Angle);
        }
        last_angle = Target_Angle;
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw * 180.f / PI);
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

        Target_Torque = PID_Omega.Get_Out();
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

void Class_Gimbal_Yaw_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float test_omega = 1.0f;
float m_angle = 0.0f;
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

void Class_Gimbal_Pitch_Motor_GM6020::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_GM6020::Transform_Angle()
{
    True_Rad_Pitch = IMU->Get_Rad_Roll();
    True_Gyro_Pitch = IMU->Get_Gyro_Roll();
    True_Angle_Pitch = IMU->Get_Angle_Roll();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Test_Out = 0;
void Class_Gimbal_Pitch_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch); // 在R超前的情况下
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        //				if (True_Rad_Pitch < -30.0f)
        //				{
        //					Target_Torque *= 0.5f;
        //				}

        // Set_Out(Target_Torque);
        Target_Torque += Gravity_Compensate * cosf(True_Rad_Pitch - 27.85f * PI / 180.0f) + Friction_Compensate * Get_Friction_Compensation(Get_Now_Omega(), 10.0f);
        Set_Out(-(Target_Torque)); // 补偿重力，pitch水平时，重心与pitch转轴连线与水平面夹角27.85度
    }
    break;

    case (DM_Motor_Control_Method_ONE_TO_FOUR):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(-True_Angle_Pitch); // 在R超前的情况下
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(-True_Gyro_Pitch * 57.3);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        Set_Out(Target_Torque); // 补偿重力
    }
    break;
    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out = Out;
    }
    default:
    {
        Set_Out(0.0);
    }
    break;
    }
    Output(); // 进入父类中进行输出
}

void Class_Gimbal_Pitch_Motor_J4310::Disable()
{
    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_J4310::Transform_Angle()
{
    True_Rad_Pitch = IMU->Get_Rad_Roll();
    True_Gyro_Pitch = IMU->Get_Gyro_Roll();
    True_Angle_Pitch = IMU->Get_Angle_Roll();

    // 一阶低通滤波
    // Low_Pass_Filter();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw); // 在R超前的情况下

            if (Target_Angle - True_Angle_Yaw > 180.0f)
            {
                PID_Angle.Set_Target(Target_Angle - 360.0f);
            }
            else if (Target_Angle - True_Angle_Yaw < -180.0f)
            {
                PID_Angle.Set_Target(Target_Angle + 360.0f);
            }
            else
            {
                // 优化-180到180
            }
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw * 57.3);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        Set_Out(Target_Torque);
    }
    break;

    case (DM_Motor_Control_Method_ONE_TO_FOUR):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw); // 在R超前的情况下
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw * 57.3);
        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        Set_Out(Target_Torque);
    }
    break;

    case (DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out = Out;
    }
    default:
    {
        Set_Out(0.0);
    }
    break;
    }
    Output(); // 进入父类中进行输出
}

void Class_Gimbal_Yaw_Motor_J4310::Disable()
{
    //    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    //    Set_Out(0.0f);
    //    Output();
    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_J4310::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw();
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case (LK_Motor_Control_Method_TORQUE):
    {
        Out = Target_Torque * Torque_Current / Current_Max * Current_Max_Cmd;
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status() == IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status() != IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch * 180.f / PI);
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

        Out = PID_Omega.Get_Out() + Gravity_Compensate;
        Set_Out(Out);
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
 * @brief 一阶低通滤波
 *
 */
void Class_Gimbal_Pitch_Motor_J4310::Low_Pass_Filter()
{
    Filter_True_Gyro_Pitch = Filter_Low_Pass_Proportion * True_Gyro_Pitch + (1 - Filter_Low_Pass_Proportion) * Filter_True_Gyro_Pitch;
}
/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_LK6010::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Pitch();
    True_Angle_Pitch = 1 * IMU->Get_Angle_Pitch();
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();
    // yaw轴6020电机
    Motor_Yaw.PID_Angle.Init(55.0f, 0.01f, 0.95f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.PID_Omega.Init(40.0f, 0.075f, 0.005f, 0.0f, 2000.0f, 20000.0f, 0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE);

    // pitch轴4310电机
    //  Motor_Pitch_J4310.PID_Angle.Init(18.0f,1.0f,0.0f,0.0f,2000,4090,0.0f,0.0f,0,0.001f,0.0f,PID_D_First_ENABLE);
    //  Motor_Pitch_J4310.PID_Omega.Init(37.0f,0.0f,0.0f,0.0f,2000,4090, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    Motor_Pitch_J4310.PID_Angle.Init(50.0f, 2.0f, 0.9f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0, 0.001f, 0.0f, PID_D_First_ENABLE);
    Motor_Pitch_J4310.PID_Omega.Init(4.0f, 14.0f, 0.002f, 0.0f, 1000, 2048, 0.0f, 0.0f, 0.0f, 0.001f, 0.5f);
    Motor_Pitch_J4310.IMU = &Boardc_BMI;
    Motor_Pitch_J4310.Init(&hcan1, (Enum_DM_Motor_ID)0x71, DM_Motor_Control_Method_MIT_IMU_Angle, 0, 20.94f, 5.0f);
    Motor_Pitch_J4310.PID_Angle.Set_I_Separate_Threshold(5.0f);
}

/**
 * @brief 输出到电机
 *
 */
float temp_err = 0.0f;
float temp_target_angle = 0.0f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Pitch_J4310.Disable();
        Motor_Yaw.Disable();

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch_J4310.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_J4310.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_J4310.PID_Torque.Set_Integral_Error(0.0f);
    }
    else // 非失能模式
    {
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        Motor_Pitch_J4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);

        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle();
            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle();
        }
        // 限制角度范围 处理yaw轴180度问题
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
        {
            Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
        {
            Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
        // pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
        // 设置目标角度
        Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
        Motor_Pitch_J4310.Set_Target_Angle(Target_Pitch_Angle);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    // 根据不同c板的放置方式来修改这几个函数
    Motor_Pitch_J4310.Transform_Angle();
    Motor_Yaw.Transform_Angle();

    Motor_Pitch_J4310.TIM_PID_PeriodElapsedCallback();
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
