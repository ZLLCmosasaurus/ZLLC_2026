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
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(speed);
            if(fabs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
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
            else if(fabs(Gimbal->Motor_Yaw.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
            }
        }
        break;
        case (2)://向右堵转
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(-speed);
            if(fabs(Gimbal->Motor_Yaw.Get_Now_Torque()) > Torque_Threshold){
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
            else if (fabs(Gimbal->Motor_Yaw.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
            }
        }
        break;
        case (4)://正常控制流程
        {
            //Gimbal->Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Set_Status(5);
        }
        break;
        case (5)://检测是否重新校准
        {

            if(Gimbal->Get_Gimbal_Control_Type() == Gimbal_Control_Type_YAW_CALIBRATION && Status[Now_Status_Serial].Time > 2000){
                Set_Status(0);
            }
            float yaw = Gimbal->Calculate_Linear(Gimbal->Max_Yaw_Angle,
                                                 Gimbal->Min_Yaw_Angle,
                                                 Gimbal->Motor_Pitch_L.Get_Now_Angle(), 
                                                 Angle_Left, 
                                                 Angle_Right);
            Gimbal->Motor_Yaw.Set_Transform_Angle(yaw);
        }
        break;
    }
}

void Class_FSM_Pitch_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向上堵转
        {
            Gimbal->Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Pitch_L.Set_Target_Omega_Radian(speed);
            Gimbal->Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Pitch_R.Set_Target_Omega_Radian(-speed);
            if(fabs(Gimbal->Motor_Pitch_L.Get_Now_Torque()) > Torque_Threshold && //都堵转
                fabs(Gimbal->Motor_Pitch_R.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://记录角度
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Upside_L = Gimbal->Motor_Pitch_L.Get_Now_Angle();
                Gimbal->Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Gimbal->Motor_Pitch_L.Set_Target_Torque(0.0f);
                Gimbal->Motor_Pitch_L.Set_Out(0.f);
                Up_Flag_L = 1;
            }
            else if(fabs(Gimbal->Motor_Pitch_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
                Up_Flag_L = 0;
                Up_Flag_R = 0;
            }
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Upside_R = Gimbal->Motor_Pitch_R.Get_Now_Angle();
                Gimbal->Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Gimbal->Motor_Pitch_R.Set_Target_Torque(0.0f);
                Gimbal->Motor_Pitch_R.Set_Out(0.f);
                Up_Flag_R = 1;
            }
            else if(fabs(Gimbal->Motor_Pitch_R.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
                Up_Flag_L = 0;
                Up_Flag_R = 0;
            }
            if(Up_Flag_L == 1 && Up_Flag_R == 1){
                Set_Status(2);
            }
        }
        break;
        case (2)://向下堵转
        {
            Gimbal->Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Pitch_L.Set_Target_Omega_Radian(-speed);
            Gimbal->Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Pitch_R.Set_Target_Omega_Radian(speed);
            if(fabs(Gimbal->Motor_Pitch_L.Get_Now_Torque()) > Torque_Threshold || 
                fabs(Gimbal->Motor_Pitch_R.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }
        }
        break;
        case (3)://下面检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Downside_L = Gimbal->Motor_Pitch_L.Get_Now_Angle();
                Gimbal->Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Gimbal->Motor_Pitch_L.Set_Target_Torque(0.0f);
                Gimbal->Motor_Pitch_L.Set_Out(0.f);
                Down_Flag_L = 1;
            }
            else if(fabs(Gimbal->Motor_Pitch_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
                Down_Flag_L = 0;
                Down_Flag_R = 0;
            }
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Downside_R = Gimbal->Motor_Pitch_R.Get_Now_Angle();
                Gimbal->Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Gimbal->Motor_Pitch_R.Set_Target_Torque(0.0f);
                Gimbal->Motor_Pitch_R.Set_Out(0.f);
                Down_Flag_R = 1;
            }
            else if(fabs(Gimbal->Motor_Pitch_R.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
                Down_Flag_L = 0;
                Down_Flag_R = 0;
            }
            if(Down_Flag_L ==1 && Down_Flag_R == 1){
                Set_Status(4);
            }
        }
        break;
        case (4)://正常控制流程
        {
            Gimbal->Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Set_Status(5);
        }
        break;
        case (5)://检测是否重新校准
        {

            if(Gimbal->Get_Gimbal_Control_Type() == Gimbal_Control_Type_PITCH_CALIBRATION && Status[Now_Status_Serial].Time > 2000){
                Set_Status(0);
                Up_Flag_L = 0;
                Up_Flag_R = 0;
                Down_Flag_L = 0;
                Down_Flag_R = 0;
            }
            float pitch_l = Gimbal->Calculate_Linear(Gimbal->Max_Pitch_Angle,
                                                     Gimbal->Min_Pitch_Angle,
                                                     Gimbal->Motor_Pitch_L.Get_Now_Angle(), 
                                                     Angle_Upside_L, 
                                                     Angle_Downside_L);
            float pitch_r = Gimbal->Calculate_Linear(Gimbal->Max_Pitch_Angle,
                                                     Gimbal->Min_Pitch_Angle,
                                                     Gimbal->Motor_Pitch_R.Get_Now_Angle(),
                                                     Angle_Upside_R, 
                                                     Angle_Downside_R);
            float now_pitch = (pitch_l + pitch_r) / 2.0f;

            Gimbal->Motor_Pitch_L.Set_Transform_Angle(-now_pitch * PI / 180.0f);
            Gimbal->Motor_Pitch_R.Set_Transform_Angle(now_pitch * PI / 180.0f);//由于反装这里左电机角度环目标取负 与右边电机相对 左边正转向上走
            //角度误差重新校准
            // if(fabs(pitch_l - pitch_r) > 0.1f){
            //     Set_Status(0);
            //     Up_Flag_L = 0;
            //     Up_Flag_R = 0;
            //     Down_Flag_L = 0;
            //     Down_Flag_R = 0;
            // }
        }
        break;
    }
}

/**
 * @brief 单电机映射计算函数
 * @param now_enc  当前电机编码器值
 * @param up_enc   校准记录的上边界编码器值
 * @param down_enc 校准记录的下边界编码器值
 * @return float   映射后的Pitch角度
 */
float Class_Gimbal::Calculate_Linear(float max,float min,float now_enc, float up_enc, float down_enc)
{
    // 安全保护：防止未校准或数据异常导致除0
    if (abs(down_enc - up_enc) < 0.001f) {
        return 0.0f; 
    }

    // 线性插值公式：Y = Y_up + (X - X_up) * Slope
    // Slope = (Y_down - Y_up) / (X_down - X_up)
    
    float slope = (max - min) / (down_enc - up_enc);
    float angle = min + (now_enc - up_enc) * slope;

    return angle;
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    FSM_Yaw_Calibration.Gimbal = this;
    FSM_Pitch_Calibration.Gimbal = this;

    FSM_Yaw_Calibration.Init(9,0);
    FSM_Pitch_Calibration.Init(9,0);

    Motor_Pitch_L.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_L.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, 2000, Motor_Pitch_L.Get_Output_Max());
    Motor_Pitch_L.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Pitch_R.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_R.PID_Omega.Init(2000.0f, 20.0f, 0.001f, 0.0f, 2000, Motor_Pitch_R.Get_Output_Max());
    Motor_Pitch_R.Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

    Motor_Yaw.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Yaw.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA);
}


/**
 * @brief 输出到电机
 *
 */
float test_a = -15.f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // // 云台失能
        // Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        // Motor_Yaw.Set_Target_Torque(0.0f);
        // Motor_Pitch_L.Set_Target_Torque(0.0f);
        // Motor_Pitch_R.Set_Target_Torque(0.0f);

        // Motor_Yaw.Set_Out(0.f);
        // Motor_Pitch_L.Set_Out(0.f);
        // Motor_Pitch_R.Set_Out(0.f);

    }
    else if(Gimbal_Control_Type == Gimbal_Control_Type_YAW_CALIBRATION)// 非失能模式
    {
        
    }
    else if(Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
    {
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        Motor_Yaw.Set_Target_Radian(Target_Yaw_Angle);
        Motor_Pitch_L.Set_Target_Radian(-Target_Pitch_Angle * PI / 180.0f);
        Motor_Pitch_R.Set_Target_Radian(Target_Pitch_Angle * PI / 180.0f);//由于反装这里左电机角度环目标取负 与右边电机相对 左边正转向上走
        //调试用
        Motor_Pitch_L.Set_Target_Radian((-test_a) * PI / 180.0f);
        Motor_Pitch_R.Set_Target_Radian((test_a) * PI / 180.0f);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{

    //FSM_Yaw_Calibration.Reload_TIM_Status_PeriodElapsedCallback();
    FSM_Pitch_Calibration.Reload_TIM_Status_PeriodElapsedCallback();



    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_L.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
