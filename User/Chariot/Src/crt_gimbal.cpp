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
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch(DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle_DEG);
        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {
            //角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            if(Target_Angle_DEG-True_Angle_Yaw>180.0f)
			{
				PID_Angle.Set_Target(Target_Angle_DEG-360.0f);
			}
			else if(Target_Angle_DEG - True_Angle_Yaw <-180.0f)
			{
				PID_Angle.Set_Target(Target_Angle_DEG+360.0f);
			}
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();
            //速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(True_Gyro_Yaw );

        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG=PID_Angle.Get_Out();

        //     //速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(Data.Now_Omega_Radian *(180.0 / PI)); // 电机反馈的角速度是弧度每秒，转为角度每秒

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque);
    }
    break;
    case(DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out=Out;
    }
    break;
    default:
    {
        Set_Out(0.0);
    }
    break;
    
    }
    Output();//进入父类中进行输出
}
void Class_Gimbal_Yaw_Motor_DM4310::Disable()
{

    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_OPENLOOP);
    Set_Out(0.0f);
    Output();
			
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_DM4310::Transform_Angle()
{
    //粗略修正YAW轴漂移
    // Service_time = DWT_GetTimeline_us();
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw() * 57.29;
    True_Angle_Yaw =  IMU->Get_Angle_Yaw();
    // True_Gyro_Yaw = IMU->Get_DMIMU_Gyro_Yaw() * 57.29; 
    // True_Angle_Yaw = IMU->Get_DMIMU_Yaw() - Service_time * K;

    kalman_update(&Kf_Gyro_Yaw, True_Gyro_Yaw);

    
}
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_IMU_Angle):
    {
        PID_Angle.Set_Target(Target_Angle_DEG);

        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {

            //角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(True_Gyro_Pitch );

        }
        else
        {
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_DEG=PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_DEG);
            PID_Omega.Set_Now(Data.Now_Omega_Radian *(180.0 / PI)); // 电机反馈的角速度是弧度每秒，转为角度每秒

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
    
        Target_Torque = PID_Omega.Get_Out();
        Set_Out(Target_Torque + Gravity_Compensate);
    }
    break;
    case(DM_Motor_Control_Method_MIT_OPENLOOP):
    {
        Out=Out;
    }
    break;
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
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_DM4310::Transform_Angle()
{ 
    // True_Angle_Pitch = -1 * IMU ->Get_DMIMU_Pitch();// 角度
    // True_Gyro_Pitch = -1 * IMU->Get_DMIMU_Gyro_Pitch() * 57.29; // 角速度
    True_Rad_Pitch = 1 * IMU->Get_Rad_Pitch();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Pitch() * 57.29;
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
    

    //yaw轴电机
    Motor_Yaw_DM4310.PID_Angle.Init(18.0f, 1.0f, 0.0f, 0.0f, 2000.0f, 4090.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    Motor_Yaw_DM4310.PID_Omega.Init(85.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 4090.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
	//Motor_Yaw_DM4310.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 4090.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    //Motor_Yaw_DM4310.PID_Omega.Init(95.0f, 10.0f, 0.0f, 0.0f, 2000.0f, 4090.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0f);
    // Motor_Yaw_DM4310.IMU = &dmIMU;
    Motor_Yaw_DM4310.IMU = &Boardc_BMI;
    Motor_Yaw_DM4310.Init(&hfdcan3, DM_Motor_ID_0xA3, DM_Motor_Control_Method_MIT_IMU_Angle, 0, 20.94f, 2.0f);
    //CAN_Send_Data(&hfdcan1, DM_Motor_ID_0xA1+0xf0, DM_Motor_CAN_Message_Save_Zero, 8);

    //yaw轴电机
    Motor_Yaw.Init(&hfdcan3, DM_Motor_ID_0xA3, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.94f, 2.0f);
    
    // pitch轴电机
    Motor_Pitch_DM4310.PID_Angle.Init(20.0f,0.0f,0.0f,0.0f,2000,4090.0f);
	Motor_Pitch_DM4310.PID_Omega.Init(150.0f,5.0f,0.0f,0.0f,2000,4090.0f);
	//Motor_Pitch_DM4310.PID_Angle.Init(10.0f,0.0f,0.0f,0.0f,2000,4090.0f);
    //Motor_Pitch_DM4310.PID_Omega.Init(80.0f,5.0f,0.0f,0.0f,2000,4090.0f);
    // Motor_Pitch_DM4310.IMU = &dmIMU;
    Motor_Pitch_DM4310.IMU = &Boardc_BMI;
    Motor_Pitch_DM4310.Init(&hfdcan2, DM_Motor_ID_0xA4, DM_Motor_Control_Method_MIT_IMU_Angle, 0, 20.94f, 2.0f);

    // pitch轴电机
    Motor_Pitch.Init(&hfdcan2, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.94f, 2.0f);

    //相机控制电机初始化

    kalman_init(&Motor_Yaw_DM4310.Kf_Gyro_Yaw, 0);
}

/**
 * @brief 输出到电机
 *
 */
float Tmp_Target_Pitch_Angle = 0.0f,Tmp_Target_Yaw_Angle = 0.0f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE) // 云台失能
    {
        // 云台失能
        Motor_Pitch_DM4310.Disable();
        Motor_Yaw_DM4310.Disable();
        
        //PID积分清零
        Motor_Yaw_DM4310.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw_DM4310.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_DM4310.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_DM4310.PID_Omega.Set_Integral_Error(0.0f);
        //设定输出力矩清零
        Motor_Pitch_DM4310.Set_Target_Torque(0.0f);
        Motor_Yaw_DM4310.Set_Target_Torque(0.0f);

        Motor_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Pitch.Set_Target_Angle(0);
        Motor_Yaw.Set_Target_Angle(0);
        Motor_Pitch.Set_Target_Omega(0);
        Motor_Yaw.Set_Target_Omega(0);

    }
    else // 非失能模式
    {
        Motor_Yaw_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);
        Motor_Pitch_DM4310.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_IMU_Angle);
        
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            Motor_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_Pitch.Set_Target_Angle(SupposedDeg_Pitch);
            Motor_Pitch.Set_Target_Omega(SupposedSpe_Pitch);
            Motor_Yaw.Set_Target_Angle(SupposedDeg_Yaw);
            Motor_Yaw.Set_Target_Omega(SupposedSpe_Yaw);

            // PITCH限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 限制角度范围 处理yaw轴180度问题
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
            {
                Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
            }
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
            {
                Target_Yaw_Angle += (2 * Max_Yaw_Angle);
            }


            // 设置目标角度
            Motor_Yaw_DM4310.Set_Target_Angle_DEG(Target_Yaw_Angle);
            Motor_Pitch_DM4310.Set_Target_Angle_DEG(Target_Pitch_Angle);
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 限制角度范围 处理yaw轴180度问题
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
            {
                Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
            }
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
            {
                Target_Yaw_Angle += (2 * Max_Yaw_Angle);
            }

            Target_Yaw_Angle = MiniPC->Get_Rx_Yaw_Angle() * 180/PI;
            Target_Pitch_Angle = MiniPC->Get_Rx_Pitch_Angle() * 180/PI;

            // 设置目标角度
            Motor_Yaw_DM4310.Set_Target_Angle_DEG(Target_Yaw_Angle);
            Motor_Pitch_DM4310.Set_Target_Angle_DEG(Target_Pitch_Angle);
           
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {


            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 限制角度范围 处理yaw轴180度问题
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) > Max_Yaw_Angle)
            {
                Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
            }
            while ((Target_Yaw_Angle - Motor_Yaw_DM4310.Get_True_Angle_Yaw()) < -Max_Yaw_Angle)
            {
                Target_Yaw_Angle += (2 * Max_Yaw_Angle);
            }

            // 设置目标角度
            Motor_Yaw_DM4310.Set_Target_Angle_DEG(Target_Yaw_Angle);
            Motor_Pitch_DM4310.Set_Target_Angle_DEG(Target_Pitch_Angle);

        }
    }
    //设定达妙电机始终在线
    Motor_Pitch_DM4310.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Motor_Yaw_DM4310.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
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
	Motor_Pitch_DM4310.Transform_Angle();
    Motor_Yaw_DM4310.Transform_Angle();

    //PID输出
    Motor_Yaw_DM4310.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_DM4310.TIM_PID_PeriodElapsedCallback();

    // Motor_Yaw.TIM_Process_PeriodElapsedCallback();
    // Motor_Pitch.TIM_Process_PeriodElapsedCallback();

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
