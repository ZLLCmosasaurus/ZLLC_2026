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
float YAW_Reference_Angle;
float YAW_Chassis_Angle;
float DebugMo = 135.0f;
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch(DM_Motor_Control_Method)
    {
	    case(DM_Motor_Control_Method_MIT_IMU_Angle):
		{
            PID_Angle.Set_Target(Target_Angle);
            if(IMU->Get_DM_IMU_Status()!=DM_IMU_Status_DISABLE)
            {
                //角度环
                PID_Angle.Set_Now(True_Angle_Pitch);
                PID_Angle.TIM_Adjust_PeriodElapsedCallback();

                Target_Omega = PID_Angle.Get_Out();
							
                //速度环
                PID_Omega.Set_Target(Target_Omega);
                //PID_Omega.Set_Now(True_Gyro_Pitch * RAD_TO_DEG);//因为大Pitch不稳内环实际值先用电机自身
                PID_Omega.Set_Now(Data.Now_Omega);
            }
            else
            {
                PID_Angle.Set_Now(Data.Now_Angle);
                PID_Angle.TIM_Adjust_PeriodElapsedCallback();

                Target_Omega = PID_Angle.Get_Out();

                //速度环
                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.Set_Now(Data.Now_Omega * RAD_TO_DEG);
            }

            PID_Omega.TIM_Adjust_PeriodElapsedCallback();
            Target_Torque=PID_Omega.Get_Out();		
            Set_Out(Target_Torque + abs(cosf(True_Rad_Pitch) * DebugMo));//补偿重力
		}
        break;
        case(DM_Motor_Control_Method_MIT_OPENLOOP):
        {
            Out=Out;
        }
        default:
        {
            Set_Out(0.0);
        }
        break;
    }
    Output();//进入父类中进行输出	
}

void Class_Gimbal_Pitch_Motor_DM4310::Disable()
{
    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数d:\.RMUC_2026\zllc-2026-M64\ZLLC_2026\User\Driver\Src\drv_math.cpp
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::Transform_Angle()
{
    True_Rad_Pitch = -1 * IMU->Get_DMIMU_Pitch() * DEG_TO_RAD;
    True_Gyro_Pitch = -1 * IMU->Get_DMIMU_Gyro_Pitch();
    True_Angle_Pitch = -1 * IMU->Get_DMIMU_Pitch();
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_LK7025::TIM_PID_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
    case (LK_Motor_Control_Method_TORQUE):
    {
        Out = Target_Torque * Torque_Current / Current_Max * Current_Max_Cmd;
        Set_Out(Out);
    }
    case (LK_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Angle = PID_Angle.Get_Out();
        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = (int16_t)(PID_Omega.Get_Out());
    }
    break;
    case (LK_Motor_Control_Method_IMU_OMEGA):
    {
        // 角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_DM_IMU_Status() == DM_IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_DM_IMU_Status() != DM_IMU_Status_DISABLE)
        {
            // 角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            // 速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw);
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

        Out = PID_Omega.Get_Out();
        Set_Out(Out);
    }
    break;
    case (LK_Motor_Control_Method_ANGLE_LOCK):
    {
        PID_Angle.Set_Target(YAW_Reference_Angle + 30.8f);
        PID_Angle.Set_Now(YAW_Chassis_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();
        Target_Omega_Angle = PID_Angle.Get_Out();
        // 速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
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

void Class_Gimbal_Yaw_Motor_LK7025::Disable()
{
    Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OpenLoop);
    Set_Out(0.0f);
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_LK7025::Transform_Angle()
{
    True_Rad_Yaw = 1 * IMU->Get_DMIMU_Yaw() * DEG_TO_RAD;
    True_Gyro_Yaw = 1 * IMU->Get_DMIMU_Gyro_Yaw();
    True_Angle_Yaw = 1 * IMU->Get_DMIMU_Yaw();
}
/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{

    // yaw轴电机
//   Motor_Yaw.PID_Angle.Init(0.8f, 0.0f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
//   Motor_Yaw.PID_Omega.Init(13.f, 0.f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
    Motor_Yaw.PID_Angle.Init(0.5f, 0.0f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
    Motor_Yaw.PID_Omega.Init(90.f, 50.0f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
//    Motor_Yaw.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
//    Motor_Yaw.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 200.0f, 4000.0f,0.0f,0.0f,0.0f,0.001f,0.0f);
    //Motor_Yaw.PID_Angle.Init(30.f, 0.0f, 0.0f, 0.0f, 500, 500);
    //Motor_Yaw.PID_Omega.Init(60.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_Yaw.Get_Output_Max(), 10.f, 50.f);
    //Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.IMU = &DM_IMU;
    Motor_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, 220.f, 0, 33.0f, LK_Motor_Control_Method_IMU_ANGLE, LK_Motor_Control_Torque);
    
    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(0.7f, 0.0f, 0.0f, 0.0f, 200.0f, 4090.0f);
    Motor_Pitch.PID_Omega.Init(180.0f, 1.0f, 0.0f, 0.0f, 2000.0f, 4090.0f);
    Motor_Pitch.IMU = &DM_IMU;
    Motor_Pitch.Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_OPENLOOP);

    Motor_Pitch_2.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, PI);
    Motor_Pitch_2.Set_Target_Omega(0.5f);
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
        Motor_Yaw.Disable();
        Motor_Pitch.Disable();


        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);


        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);

        Motor_Yaw.Set_Out(0.0f);
        Motor_Pitch.Set_Out(0.0f);
			
		Motor_Pitch_2.Set_Target_Omega(0.0f);

    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            //Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
            Motor_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Pitch_2_Angle, Min_Pitch_2_Angle, Max_Pitch_2_Angle);
            //Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            Motor_Pitch_2.Set_Target_Omega(1.0f);

            // 设置目标角度
            // if(Motor_Pitch_2.Get_Now_Angle() > LOCK_PITCH - 0.3f)
            // {
            //     Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            // }
            // else
            // {
            //     Motor_Yaw.Disable();
            //     Motor_Yaw.Set_Out(0.0f);
            // }
            // 限制角度范围 处理yaw轴180度问题
            while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
            {
                Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
            }
            while ((Target_Yaw_Angle - Motor_Yaw.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
            {
                Target_Yaw_Angle += (2 * Max_Yaw_Angle);
            }
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Pitch_2.Set_Target_Angle(Target_Pitch_2_Angle);
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            //控制方式
            Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
            Motor_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());
            Motor_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
           
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            //控制方式
            Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
            Motor_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制模式
    Output();

    // 根据不同c板的放置方式来修改这几个函数
	Motor_Pitch.Transform_Angle();
    Motor_Yaw.Transform_Angle();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_2.TIM_Process_PeriodElapsedCallback();
}
    

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
