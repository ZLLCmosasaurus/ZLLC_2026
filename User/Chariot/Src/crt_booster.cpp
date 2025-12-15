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
        case (0)://向前堵转
        {

        }
        break;
        case (1)://前侧检测
        {

        }
        break;
        case (2)://向后堵转
        {
    
        }
        break;
        case (3)://后侧检测
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
 * @brief  将当前角度线性映射到目标行程
 * @param  curr_angle   当前电机角度
 * @param  angle_start  起始点角度（通常是 Backward 角度）
 * @param  angle_end    结束点角度（通常是 Forward 角度）
 * @param  max_length   物理最大行程（例如 1.0 表示百分比，或者 200.0 表示 mm）
 * @return 映射后的位置值
 */
float Class_FSM_Push_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f) {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

float Class_FSM_Pull_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f) {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

void Class_FSM_Push_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向前堵转
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Booster->Motor_Push_L.Set_Target_Omega_Radian(2.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(2.0f);
            
            if(fabs(Booster->Motor_Push_L.Get_Now_Torque()) > Torque_Threshold &&
                fabs(Booster->Motor_Push_R.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://前侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward_L = Booster->Motor_Push_L.Get_Now_Angle();
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_L.Set_Target_Torque(0.f);
                Booster->Motor_Push_L.Set_Out(0.f);
                forward_flag_L = 1;
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
                forward_flag_L = 0;
                forward_flag_R = 0;
            }
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward_R = Booster->Motor_Push_R.Get_Now_Angle();
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_R.Set_Target_Torque(0.f);
                Booster->Motor_Push_R.Set_Out(0.f);
                forward_flag_R = 1;
            }
            else if (fabs(Booster->Motor_Push_R.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
                forward_flag_L = 0;
                forward_flag_R = 0;
            }
            if(forward_flag_L == 1 && forward_flag_R == 1){
                Set_Status(2);
            }
        }
        break;
        case (2)://向后堵转
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Booster->Motor_Push_L.Set_Target_Omega_Radian(-2.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(-2.0f);

            if(fabs(Booster->Motor_Push_L.Get_Now_Torque()) > Torque_Threshold && 
                fabs(Booster->Motor_Push_R.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }
        }
        break;
        case (3)://后侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward_L = Booster->Motor_Push_L.Get_Now_Angle();
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_L.Set_Target_Torque(0.f);
                Booster->Motor_Push_L.Set_Out(0.f);
                backward_flag_L = 1;
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
                backward_flag_L = 0;
                backward_flag_R = 0;
            }
            
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward_R = Booster->Motor_Push_R.Get_Now_Angle();
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_R.Set_Target_Torque(0.f);
                Booster->Motor_Push_R.Set_Out(0.f);
                backward_flag_R = 1;
            }
            else if (fabs(Booster->Motor_Push_R.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
                backward_flag_L = 0;
                backward_flag_R = 0;
            }
            if(backward_flag_L == 1 && backward_flag_R == 1){
                Set_Status(4);
            }
        }
        break;
        case (4)://正常控制流程
        {

            Set_Status(5);
            
        }
        break;
        case (5)://校准检测
        {
            float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Forward_L, Angle_Backward_L,1.0f);
            float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Forward_R, Angle_Backward_R,1.0f);
            float now_position = (now_position_l + now_position_r) / 2.0f;
            Booster->Motor_Push_L.Set_Transform_Angle(now_position);
            Booster->Motor_Push_R.Set_Transform_Angle(now_position);
        }
    }
}
void Class_FSM_Pull_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向前堵转
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(2.0f);
            
            if(fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://前侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward = Booster->Motor_Pull.Get_Now_Angle();
                Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Pull.Set_Target_Torque(0.f);
                Booster->Motor_Pull.Set_Out(0.f);
                Set_Status(2);
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
            }
        }
        break;
        case (2)://向后堵转
        {

            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(-2.0f);

            if(fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }

        }
        break;
        case (3)://后侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward = Booster->Motor_Pull.Get_Now_Angle();
                Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Pull.Set_Target_Torque(0.f);
                Booster->Motor_Pull.Set_Out(0.f);
                Set_Status(4);
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
            }
            
        }
        break;
        case (4)://正常控制流程
        {

            Set_Status(5);
            
        }
        break;
        case (5)://校准检测
        {
            float now_position = Linear_Map_Position(Booster->Motor_Pull.Get_Now_Angle(), Angle_Forward, Angle_Backward,1.0f);
            Booster->Motor_Pull.Set_Transform_Angle(now_position);
        }
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
    FSM_Push_Calibration.Init(6, 0);

    FSM_Pull_Calibration.Booster = this;
    FSM_Pull_Calibration.Init(6, 0);

    //拉力电机
    Motor_Pull.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pull.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, 2000, Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Push_L.PID_Angle.Init(0.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Push_L.PID_Omega.Init(0.0f, 0.0f, 0.00f, 0.0f, Motor_Push_L.Get_Output_Max(), Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

    Motor_Push_R.PID_Angle.Init(0.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Push_R.PID_Omega.Init(0.0f, 0.0f, 0.00f, 0.0f, Motor_Push_R.Get_Output_Max(), Motor_Push_R.Get_Output_Max());
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
            // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

            // Motor_Pull.Set_Target_Torque(0.f);
            // Motor_Push_L.Set_Target_Torque(0.f);
            // Motor_Push_R.Set_Target_Torque(0.f);

            // Motor_Pull.Set_Out(0.f);
            // Motor_Push_L.Set_Out(0.f);
            // Motor_Push_R.Set_Out(0.f);
        }
        break;
        case (Booster_Control_Type_NORMAL):
        {

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
    //皮筋校准
    FSM_Push_Calibration.Reload_TIM_Status_PeriodElapsedCallback();

    //FSM_Pull_Calibration.Reload_TIM_Status_PeriodElapsedCallback();
    //发射状态机
    //FSM_Shooting.Reload_TIM_Status_PeriodElapsedCallback();

    Output();

    //PID输出
    Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
