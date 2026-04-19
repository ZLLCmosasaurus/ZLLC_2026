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

#ifdef MOTOR_TEST
bool set_roll_output_enable = false;
bool set_roll_cali_enable = false;
bool gripper_output_flag = false; // 测试状态机，输出之前先看数值对不对
float cali_radian = -300.0f;

/*6020测试用*/
uint8_t debug_6020_mode = 0;

float debug_6020_omega_kp = 0.0f;
float debug_6020_omega_ki = 0.0f;
float debug_6020_omega_kd = 0.0f;

float debug_6020_angle_kp = 0.0f;
float debug_6020_angle_ki = 0.0f;
float debug_6020_angle_kd = 0.0f;

/*C610测试用*/
uint8_t debug_c610_mode = 0;

float debug_c610_omega_kp = 0.0f;
float debug_c610_omega_ki = 0.0f;
float debug_c610_omega_kd = 0.0f;

float debug_c610_angle_kp = 0.0f;
float debug_c610_angle_ki = 0.0f;
float debug_c610_angle_kd = 0.0f;
#endif

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

    J0_Pitch_4340.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 20.0f);
    J1_Yaw_8009P.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);
    J2_Yaw_4340P.Init(&hfdcan1, DM_Motor_ID_0xA3, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);
    J3_Roll_2325.Init(&hfdcan2, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 200.0f, 10.0f);
    J4_Pitch_2325.Init(&hfdcan2, DM_Motor_ID_0xA5, DM_Motor_Control_Method_POSITION_OMEGA, 0, 200.0f, 10.0f);
    Jodell_ERG150T.Init(&huart2, 9);

    /*初始化状态机，不进行初始化的话状态机没法访问云台对象中的电机*/
    Calibration_FSM.Gimbal = this;
    /*初始化轨迹追踪器*/
    Trajectory_Tracer.Gimbal = this;
}

/**
 * @brief 输出到电机
 *
 */

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
#ifdef PUMA
        // 云台失能
        Motor_DM_J0_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J1_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J2_Pitch_2.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J4_Pitch_3.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

        Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_6020_J5_Roll_2.PID_Angle.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.PID_Omega.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.PID_Torque.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.Set_Target_Torque(0.0f);
        Motor_6020_J5_Roll_2.Set_Out(0.0f);
#endif
        J0_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J1_Yaw_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J2_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J3_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J4_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Jodell_ERG150T.Set_Motor_Control_Status(Jodell_Motor_Control_DISABLE);

        // 测试完成后去掉注释
        arm_init = false;
        Calibration_FSM.Set_Status(0);
    }
    else // 非失能模式
    {
        if (arm_init)
        {
            if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
            {
                J0_Pitch_4340.Set_Target_Omega(Target_J0_Pitch_Omega);
                J0_Pitch_4340.Set_Target_Angle(Target_J0_Pitch_Radian);

                J1_Yaw_8009P.Set_Target_Omega(Target_J1_Yaw_Omega);
                J1_Yaw_8009P.Set_Target_Angle(Target_J1_Yaw_Radian);

                J2_Yaw_4340P.Set_Target_Omega(Target_J2_Yaw_Omega);
                J2_Yaw_4340P.Set_Target_Angle(Target_J2_Yaw_Radian);

                if (Calibration_FSM.Get_Roll_cali_status())
                {
                    J3_Roll_2325.Set_Target_Omega(Target_J3_Roll_Omega);
                    J3_Roll_2325.Set_Target_Angle(Target_J3_Roll_Radian);
                }

                if (Calibration_FSM.Get_Pitch_cali_status())
                {
                    J4_Pitch_2325.Set_Target_Omega(Target_J4_Pitch_Omega);
                    J4_Pitch_2325.Set_Target_Angle(Target_J4_Pitch_Radian);
                }

                Jodell_ERG150T.Set_Target_Omega(Target_J5_Roll_Omega);
                Jodell_ERG150T.Set_Target_Roll(Target_J5_Roll_Radian);

                Jodell_ERG150T.Set_Gripper_Position(Target_Gripper_Position);
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
            {
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
            {
            }
        }
        else
        /*将机械臂调整到初始姿态，只有在整车上电和整臂断电重连（机器人复活）时才会触发，2325的放在校准状态机*/
        {
            Calibration_FSM.Reload_TIM_Status_PeriodElapsedCallback();
        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    // 控制模式，用于设置电机的转动模式，转动的目标速度和角度
    Output();

    // 电机优先级计数器
    can_priority_cnt++;

    // // 单编码器电机校准状态机回调函数
    // if (arm_init)
    // {
    //     Calibration_FSM.Reload_TIM_Status_PeriodElapsedCallback();
    // }

    switch (can_priority_cnt % 5)
    {
    case (1):
    {
        J0_Pitch_4340.TIM_Process_PeriodElapsedCallback();
        J4_Pitch_2325.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (2):
    {
        J1_Yaw_8009P.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (3):
    {
        J2_Yaw_4340P.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (4):
    {
        J3_Roll_2325.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (0):
    {
        can_priority_cnt = 0;
        break;
    }
    }

    // 用于更新当前机械臂位置
    // Trajectory_Tracer.arm_pos_rpy_update();
}

/**
 * @brief 单编码器电机校准状态机
 *
 */
// void Class_FSM_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
// {
//     Status[Now_Status_Serial].Time++;
//     switch (Now_Status_Serial)
//     {
//     case (0):
//         /*校准状态*/
//         {
//             if (Gimbal->J3_Roll_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !roll_cali_status)
//             {
//                 roll_cali_status = Motor_Calibration(&Gimbal->J3_Roll_2325, roll_offset, -310.0f, 2.5f, roll_locked_torque, roll_locked_cnt);
//             }

//             // if (Gimbal->J4_Pitch_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !pitch_cali_status)
//             // {
//             //     pitch_cali_status = Motor_Calibration(&Gimbal->J4_Pitch_2325, pitch_offset, -160.0f, 2.5f, pitch_locked_torque, pitch_locked_cnt);
//             // }

//             if (roll_cali_status)
//             {
//                 Gimbal->J3_Roll_Cali_Offset = roll_offset;
//                 Gimbal->J3_Roll_Min_Radian = Gimbal->J3_Roll_Cali_Offset * DM2325_GEAR_RATIO;
//                 Gimbal->J3_Roll_Max_Radian = Gimbal->J3_Roll_Min_Radian + roll_range;
//                 Gimbal->J3_Roll_Zero_Position_Radian = (Gimbal->J3_Roll_Min_Radian + Gimbal->J3_Roll_Max_Radian) / 2.0f;
//             }

//             // if (pitch_cali_status)
//             // {
//             //     Gimbal->J4_Pitch_Cali_Offset = pitch_offset;
//             //     Gimbal->J4_Pitch_Min_Radian = Gimbal->J4_Pitch_Cali_Offset * DM2325_GEAR_RATIO;
//             //     Gimbal->J4_Pitch_Max_Radian = Gimbal->J4_Pitch_Cali_Offset + pitch_range;
//             //     Gimbal->J4_Pitch_Zero_Position_Radian = (Gimbal->J4_Pitch_Min_Radian + Gimbal->J4_Pitch_Max_Radian) / 2.0f;
//             // }

//             bool cali_flag = (roll_cali_status) &&
//                             //  (pitch_cali_status) &&
//                              (true);
//             if (cali_flag)
//             {
//                 Set_Status(1);
//             }

//             break;
//         }
//     case (1):
//         /*校准完成状态*/
//         {
//             if (Gimbal->J3_Roll_2325.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
//             {
//                 roll_cali_status = false;
//                 Set_Status(0);
//             }
//             // if (Gimbal->J4_Pitch_2325.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
//             // {
//             //     pitch_cali_status = false;
//             //     Set_Status(0);
//             // }

//             break;
//         }
//     }
// }

void Class_FSM_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0):
        /*前三轴转到初始位置*/
        {
            Gimbal->J0_Pitch_4340.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Gimbal->J1_Yaw_8009P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Gimbal->J2_Yaw_4340P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Gimbal->J3_Roll_2325.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Gimbal->J4_Pitch_2325.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

            Gimbal->J0_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Gimbal->J1_Yaw_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Gimbal->J2_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Gimbal->Jodell_ERG150T.Set_Motor_Control_Status(Jodell_Motor_Control_ENABLE);

            Gimbal->J0_Pitch_4340.Set_Target_Omega(1.0f);
            Gimbal->J0_Pitch_4340.Set_Target_Angle(0.0f); // Radian 0

            Gimbal->J1_Yaw_8009P.Set_Target_Omega(0.5f);
            Gimbal->J1_Yaw_8009P.Set_Target_Angle(0.0f);

            Gimbal->J2_Yaw_4340P.Set_Target_Omega(0.5f);
            Gimbal->J2_Yaw_4340P.Set_Target_Angle(0.0f);

            Gimbal->Jodell_ERG150T.Set_Target_Omega(2.0f * PI);
            Gimbal->Jodell_ERG150T.Set_Target_Roll(0.0f);

            bool init_flag =
                (fabs(Gimbal->J0_Pitch_4340.Get_Target_Angle() + PI - Gimbal->J0_Pitch_4340.Get_Now_Angle()) < 0.05f) &&
                (fabs(Gimbal->J1_Yaw_8009P.Get_Target_Angle() + PI - Gimbal->J1_Yaw_8009P.Get_Now_Angle()) < 0.05f) &&
                (fabs(Gimbal->J2_Yaw_4340P.Get_Target_Angle() + PI - Gimbal->J2_Yaw_4340P.Get_Now_Angle()) < 0.05f) &&
                (Gimbal->Jodell_ERG150T.Get_Motor_Working_Status() == Jodell_Motor_Working_ENABLE) &&
                (Gimbal->Jodell_ERG150T.Get_Now_Omega() <= 0.1f) &&
                (true);

            if (init_flag)
            {
                Set_Status(1);
            }

            break;
        }

    case (1):
        /*Roll和Pitch校准状态*/
        {
            if (Gimbal->J3_Roll_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !roll_cali_status)
            {
                roll_cali_status = Motor_Calibration(&Gimbal->J3_Roll_2325, roll_offset, -310.0f, 5.0f * PI, roll_locked_torque, roll_locked_cnt);
            }

            if (Gimbal->J4_Pitch_2325.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !pitch_cali_status)
            {
                pitch_cali_status = Motor_Calibration(&Gimbal->J4_Pitch_2325, pitch_offset, -160.0f, 5.0f, pitch_locked_torque, pitch_locked_cnt);
            }

            if (roll_cali_status)
            {
                Gimbal->J3_Roll_Cali_Offset = roll_offset;
                Gimbal->J3_Roll_Min_Radian = (Gimbal->J3_Roll_Cali_Offset / PI) * 310.0f;
                Gimbal->J3_Roll_Max_Radian = Gimbal->J3_Roll_Min_Radian + roll_range;
                Gimbal->J3_Roll_Zero_Position_Radian = (Gimbal->J3_Roll_Min_Radian + Gimbal->J3_Roll_Max_Radian) / 2.0f;
                Gimbal->Set_Target_J3_Roll_Radian(0.0f);

                if (Gimbal->J3_Roll_2325.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
                {
                    Gimbal->J3_Roll_2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
                }

                Gimbal->J3_Roll_2325.Set_Target_Omega(0.5f * PI * DM2325_GEAR_RATIO);
                Gimbal->J3_Roll_2325.Set_Target_Angle(Gimbal->Target_J3_Roll_Radian);
            }

            if (pitch_cali_status)
            {
                Gimbal->J4_Pitch_Cali_Offset = pitch_offset;
                Gimbal->J4_Pitch_Min_Radian = (Gimbal->J4_Pitch_Cali_Offset / PI) * pitch_range;
                Gimbal->J4_Pitch_Max_Radian = Gimbal->J4_Pitch_Min_Radian + pitch_range;
                Gimbal->J4_Pitch_Zero_Position_Radian = (Gimbal->J4_Pitch_Min_Radian + Gimbal->J4_Pitch_Max_Radian) / 2.0f;
                Gimbal->Set_Target_J4_Pitch_Radian(0.0f);

                if (Gimbal->J4_Pitch_2325.Get_DM_Motor_Control_Status() == DM_Motor_Control_Status_DISABLE)
                {
                    Gimbal->J4_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
                }

                Gimbal->J4_Pitch_2325.Set_Target_Omega(0.5f * PI * DM2325_GEAR_RATIO);
                Gimbal->J4_Pitch_2325.Set_Target_Angle(Gimbal->Target_J4_Pitch_Radian);
            }

            bool cali_flag = (roll_cali_status) &&
                             (pitch_cali_status) &&
                             (true);
            if (cali_flag)
            {
                Set_Status(2);
            }

            break;
        }

    case (2):
        /*Roll和Pitch校准完后转到0点位置*/
        {

            // Gimbal->J4_Pitch_2325.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

            bool finish_flag =
                (fabs((Gimbal->J3_Roll_2325.Get_Target_Angle()) / DM2325_GEAR_RATIO + PI - Gimbal->J3_Roll_2325.Get_Now_Angle()) < 0.1f) &&
                (fabs((Gimbal->J4_Pitch_2325.Get_Target_Angle()) / DM2325_GEAR_RATIO + PI - Gimbal->J4_Pitch_2325.Get_Now_Angle()) < 0.1f) &&
                (true);

            if (finish_flag)
            {
                Gimbal->arm_init = true;
                Set_Status(3);
            }

            break;
        }

    case (3):
    {

        break;
    }
    }
}

/**
 * @brief 校准执行函数 2325
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DM_Motor_J4310 *Motor, float &Cali_Offset, float Cali_Max_Radian, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
{
#ifdef MOTOR_TEST
    /*测试用*/
    if (set_roll_cali_enable)
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    }
    else
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
#endif

    Motor->Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

    Motor->Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
    Motor->Set_Target_Omega(Cali_Omega);
    /*往逆时针方向校准*/
    Motor->Set_Target_Angle(Cali_Max_Radian);

    float Now_Torque = Motor->Get_Now_Torque();
    float Now_Omega = Motor->Get_Now_Omega();

    if ((fabs(Now_Torque) >= locked_torque) && (fabs(Now_Omega) <= 0.5f))
    {
        locked_cnt++;

        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            Cali_Offset = Motor->Get_Now_Angle() - PI; // 协议里上电后默认角度映射为PI，但此时对应电机发送角度的映射为0.0rad，所以要减去PI

            Motor->Set_Target_Angle((Cali_Offset + 0.05f) * DM2325_GEAR_RATIO); // 校准好后松开一点，乘以一个减速比

            return true;
        }
    }
    else
    {
        if (locked_cnt > 0)
        {
            locked_cnt -= 1;
        }
        else
        {
            locked_cnt = 0;
        }
    }
    return false;
}

/**
 * @brief 校准执行函数 C610 - 2006
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DJI_Motor_C610 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
{
    Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    Motor->Set_Target_Omega_Radian(Cali_Omega);
    Motor->Set_Target_Radian(-3.14f);

    if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (Motor->Get_Now_Omega_Radian() <= 0.01f))
    {
        locked_cnt++;
        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            gripper_offset = Motor->Get_Now_Radian();

            Motor->Set_Target_Radian(gripper_offset + 0.015f); // 校准完成后稍微松开一点，避免一直堵转，校准是往张开的方向动的，所以这里往加紧的方向动一下

            return true;
        }
    }
    else
    {
        locked_cnt = 0;
    }
    return false;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
