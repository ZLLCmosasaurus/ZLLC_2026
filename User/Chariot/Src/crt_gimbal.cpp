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

    External_IMU.Init(0.0);

    // main电机
    Motor_Main_Yaw.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 3, 15);
    Motor_Main_Yaw.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 400.0f, 2048.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());            //这样一直输出的都是0，失能
    Motor_Main_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE, MAIN_YAW_ENCODER_OFFSET);

    // yaw轴电机
    Motor_Yaw.PID_Angle.Init(0.f, 0.0f, 0.0f, 0.0f, 500, 500);
    Motor_Yaw.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 6000, Motor_Yaw.Get_Output_Max(), 10.f, 50.f);
    Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, YAW_ENCODER_OFFSET);
    
    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(0.f, 0.0f, 0.0f, 0.0f, 50.f, 100.f);
    Motor_Pitch.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 6000, Motor_Pitch.Get_Output_Max(),0.f,0.f,40.f);
    Motor_Pitch.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
}


/**
 * @brief 输出到电机
 *
 */

void Class_Gimbal::Output()
{
    float Main_Diff_Angle = 0.0f;               //大Yaw角度优化处理的相差角度
    static float pre_yaw_angle = 0.0f, pre_pitch_angle = 0.0f, pre_main_yaw_angle = 0.0f;

    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);
        Motor_Main_Yaw.Set_Target_Torque(0.0f);
    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);

            //对于大Yaw控制的突变点与优劣弧处理       0--2*PI
            Main_Diff_Angle = Normalize_Angle_Radian_PI_to_PI(Target_Yaw_Angle - Boardc_BMI.Get_Angle_Yaw());

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 设置目标角度    Motor_Yaw的角度是以偏置零点为原点，改Encoder_offset实现校准
            Motor_Yaw.Set_Target_Angle(0.0f);                       //可能可以加前馈
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Main_Yaw.Set_Target_Angle(Target_Yaw_Angle + Main_Diff_Angle);

            pre_yaw_angle      = 0.0f;
            pre_pitch_angle    = Motor_Pitch.Get_Now_Angle();
            pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            if(MiniPC->Get_Auto_aim_Status() == Auto_aim_Status_DISABLE){               //巡航模式
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

                Motor_Yaw.Set_Target_Omega_Angle(CRUISE_YAW_SPEED);
                Motor_Pitch.Set_Target_Omega_Angle(CRUISE_PITCH_SPEED);

                if(Motor_Yaw.Get_Now_Angle() < -50.0f){
                    Motor_Yaw.Set_Target_Omega_Angle(CRUISE_YAW_SPEED);
                }
                else if(Motor_Yaw.Get_Now_Angle() > 50.0f){
                    Motor_Yaw.Set_Target_Omega_Angle(-CRUISE_YAW_SPEED);
                }

                if(Motor_Pitch.Get_Now_Angle() < -20.0f){
                    Motor_Pitch.Set_Target_Omega_Angle(CRUISE_PITCH_SPEED);
                }
                else if(Motor_Pitch.Get_Now_Angle() > 20.0f){
                    Motor_Pitch.Set_Target_Omega_Angle(-CRUISE_PITCH_SPEED);
                }

                //更新历史值
                pre_yaw_angle = Motor_Yaw.Get_Now_Angle();
                pre_pitch_angle = Motor_Pitch.Get_Now_Angle();
            }
            else{
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                //这是处于间接识别到的阶段（识别不稳定或者装甲板在闪烁）
                //也就是只要识别到一次目标，就会切换到自瞄模式，至少持续0.5s才会退出，这0.5s是为识别不稳定留下的空间
                if(MiniPC->Get_Rx_Yaw_Angle() == 0.0f && MiniPC->Get_Rx_Pitch_Angle() == 0.0f)
                {
                    //这里并不是上一个目标点而是丢失目标后的最后当前点
                    Motor_Yaw.Set_Target_Angle(pre_yaw_angle);
                    Motor_Pitch.Set_Target_Angle(pre_pitch_angle);
                }
                else
                {
                    float MiniPC_Target_Yaw    = MiniPC->Get_Rx_Yaw_Angle();
                    float MiniPC_Target_Pitch  = MiniPC->Get_Rx_Pitch_Angle();

                    //怕超出限位到死区
                    Math_Constrain(&MiniPC_Target_Yaw, -LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);
                    Math_Constrain(&MiniPC_Target_Pitch, Min_Pitch_Angle, Max_Pitch_Angle);

                    Motor_Yaw.Set_Target_Angle(MiniPC_Target_Yaw);
                    Motor_Pitch.Set_Target_Angle(MiniPC_Target_Pitch);

                    pre_yaw_angle = Motor_Yaw.Get_Now_Angle();
                    pre_pitch_angle = Motor_Pitch.Get_Now_Angle();
                }
            }

            // 大yaw控制逻辑   由上位机控制是否转动
            Target_Yaw_Angle = Boardc_BMI.Get_Angle_Yaw();                  //角度一直更新防止切回手动控制Target还是上一次的数据
            if(fabs(MiniPC->Get_Rx_Target_Omega_Yaw_Main()) < 0.01f){
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                Main_Diff_Angle = Normalize_Angle_Radian_PI_to_PI(pre_main_yaw_angle - Boardc_BMI.Get_Angle_Yaw());
                Motor_Main_Yaw.Set_Target_Angle(pre_main_yaw_angle + Main_Diff_Angle);
                Motor_Main_Yaw.Set_Target_Omega_Angle(0.0f);
                
            }
            else{
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OMEGA);
                Motor_Main_Yaw.Set_Target_Omega_Angle(MiniPC->Get_Rx_Target_Omega_Yaw_Main());     //实际上单位是rad
                pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();                                   //不能像其他变量一样一直更新，不然目标角度会一直变化，会有问题         
            }

        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            Target_Yaw_Angle = Boardc_BMI.Get_Angle_Yaw();

            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);

            Main_Diff_Angle = Normalize_Angle_Radian_PI_to_PI(pre_main_yaw_angle - Boardc_BMI.Get_Angle_Yaw());

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            //Math_Constrain(&Target_Yaw_Angle,-LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);            //有问题！！！！！

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(pre_yaw_angle);
            Motor_Pitch.Set_Target_Angle(pre_pitch_angle);
            Motor_Main_Yaw.Set_Target_Angle(pre_main_yaw_angle + Main_Diff_Angle);

        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制更新
    Output();

    //数据传输更新        记得对方向
    Motor_Yaw.Set_Transform_Omega(External_IMU.Get_Gyro_Yaw());
    Motor_Yaw.Set_Transform_Angle(Motor_Yaw.Get_Now_Angle());

    Motor_Main_Yaw.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Yaw());
    Motor_Main_Yaw.Set_Transform_Angle(Boardc_BMI.Get_Angle_Yaw());

    Motor_Pitch.Set_Transform_Omega(External_IMU.Get_Gyro_Pitch());
    Motor_Pitch.Set_Transform_Angle(External_IMU.Get_Angle_Pitch());

    //可能得写死区严重时的强制保护

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Main_Yaw.TIM_Process_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
