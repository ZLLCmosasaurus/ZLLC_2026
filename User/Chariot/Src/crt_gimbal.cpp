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
/*6020测试用，在夹爪装上去调完参数之前不用删*/
uint8_t debug_6020_mode = 0;

float debug_6020_omega = 0.0f;
float debug_6020_omega_kp = 0.0f;
float debug_6020_omega_ki = 0.0f;
float debug_6020_omega_kd = 0.0f;

float debug_6020_radian = 0.0f;
float debug_6020_angle_kp = 0.0f;
float debug_6020_angle_ki = 0.0f;
float debug_6020_angle_kd = 0.0f;

uint32_t dwt_cnt = 0;
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
    J3_Yaw_4340P.Init(&hfdcan2, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);
    J4_Pitch_4340P.Init(&hfdcan2, DM_Motor_ID_0xA5, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 20.0f);
    J5_Yaw_4340P.Init(&hfdcan2, DM_Motor_ID_0xA6, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);

    Motor_C610_Gripper.PID_Angle.Init(42.5f, 5.0f, 0.0f, 0.0f, 500, 500, 500);
    Motor_C610_Gripper.PID_Omega.Init(1800.0f, 0.0f, 0.0f, 0.0f, 2000, 4000, 10.f, 50.f); // 尝试把速度环的Ki禁用，用于夹爪夹紧
    Motor_C610_Gripper.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE);
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
        J3_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J4_Pitch_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        J5_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_C610_Gripper.PID_Angle.Set_Integral_Error(0.0f);
        Motor_C610_Gripper.PID_Omega.Set_Integral_Error(0.0f);
        Motor_C610_Gripper.Set_Target_Omega_Angle(0.0f);
        Motor_C610_Gripper.Set_Out(0.0f);
        arm_init = false;
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

                J3_Yaw_4340P.Set_Target_Omega(Target_J3_Yaw_Omega);
                J3_Yaw_4340P.Set_Target_Angle(Target_J3_Yaw_Radian);

                J4_Pitch_4340P.Set_Target_Omega(Target_J4_Pitch_Omega);
                J4_Pitch_4340P.Set_Target_Angle(Target_J4_Pitch_Radian);

                J5_Yaw_4340P.Set_Target_Omega(Target_J5_Yaw_Omega);
                J5_Yaw_4340P.Set_Target_Angle(Target_J5_Yaw_Radian);

                if (Calibration_FSM.Get_Gripper_cali_status())
                {
                    Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE); // 用Motor_Test调试时删这一行，因为上面Motor_Test的代码块里写了标志位用于使能和失能
                    Motor_C610_Gripper.Set_Target_Radian(Target_Gripper_Radian);
                }
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
            J0_Pitch_4340.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            J1_Yaw_8009P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            J2_Yaw_4340P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            J3_Yaw_4340P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            J4_Pitch_4340P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            J5_Yaw_4340P.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

            J0_Pitch_4340.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J1_Yaw_8009P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J2_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J3_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J4_Pitch_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            J5_Yaw_4340P.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

            J0_Pitch_4340.Set_Target_Omega(1.0f);
            J0_Pitch_4340.Set_Target_Angle(0.0f); // Radian 0

            J1_Yaw_8009P.Set_Target_Omega(0.5f);
            J1_Yaw_8009P.Set_Target_Angle(0.0f);

            J2_Yaw_4340P.Set_Target_Omega(0.5f);
            J2_Yaw_4340P.Set_Target_Angle(0.0f);

            J3_Yaw_4340P.Set_Target_Omega(0.5f);
            J3_Yaw_4340P.Set_Target_Angle(0.0f); // Radian 0

            J4_Pitch_4340P.Set_Target_Omega(0.5f);
            J4_Pitch_4340P.Set_Target_Angle(0.0f); // Radian 0

            J5_Yaw_4340P.Set_Target_Omega(0.5f);
            J5_Yaw_4340P.Set_Target_Angle(0.0f); // Radian 0

            bool init_flag = 
                (fabs(J0_Pitch_4340.Get_Target_Angle() + PI - J0_Pitch_4340.Get_Now_Angle()) < 0.05f) &&
                (fabs(J1_Yaw_8009P.Get_Target_Angle() + PI - J1_Yaw_8009P.Get_Now_Angle()) < 0.05f) &&
                (fabs(J2_Yaw_4340P.Get_Target_Angle() + PI - J2_Yaw_4340P.Get_Now_Angle()) < 0.05f) &&
                (fabs(J3_Yaw_4340P.Get_Target_Angle() + PI - J3_Yaw_4340P.Get_Now_Angle()) < 0.05f) &&
                (fabs(J4_Pitch_4340P.Get_Target_Angle() + PI - J4_Pitch_4340P.Get_Now_Angle()) < 0.05f) &&
                (fabs(J5_Yaw_4340P.Get_Target_Angle() + PI - J5_Yaw_4340P.Get_Now_Angle()) < 0.05f);

            arm_init = init_flag;
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

    // 单编码器电机校准状态机回调函数
    if (arm_init)
    {
        Calibration_FSM.Reload_TIM_Status_PeriodElapsedCallback();
    }

    switch (can_priority_cnt % 5)
    {
    case (1):
    {
        J0_Pitch_4340.TIM_Process_PeriodElapsedCallback();
        J4_Pitch_4340P.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (2):
    {
        J1_Yaw_8009P.TIM_Process_PeriodElapsedCallback();
        J5_Yaw_4340P.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (3):
    {
        J2_Yaw_4340P.TIM_Process_PeriodElapsedCallback();
        Motor_C610_Gripper.TIM_PID_PeriodElapsedCallback();
        break;
    }
    case (4):
    {
        J3_Yaw_4340P.TIM_Process_PeriodElapsedCallback();
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
void Class_FSM_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0):
        /*校准状态*/
        {
            /*夹爪2006的校准状态机*/
            if (Gimbal->Motor_C610_Gripper.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && !gripper_cali_status)
            {
                gripper_cali_status = Motor_Calibration(&Gimbal->Motor_C610_Gripper, 0.75f, gripper_locked_torque, gripper_locked_cnt);
            }

            if (gripper_cali_status)
            {
                Gimbal->gripper_cali_offset = gripper_offset;
                Gimbal->Min_gripper_Radian = Gimbal->gripper_cali_offset;
                Gimbal->Max_gripper_Radian = Gimbal->gripper_cali_offset + 0.95f; // 夹爪张开最大时为0.95f
            }

            if (gripper_cali_status)
            {
                Set_Status(1);
            }

            break;
        }
    case (1):
        /*校准完成状态*/
        {
            if (Gimbal->Motor_C610_Gripper.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
            {
                gripper_cali_status = false;
                Set_Status(0);
            }
            break;
        }
    }
}

/**
 * @brief 校准执行函数 2325
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DM_Motor_J4310 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
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
    Motor->Set_Target_Angle(-310.0f);

    if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (fabs(Motor->Get_Now_Omega()) <= 0.01f))
    {
        locked_cnt++;

        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            Cali_Offset = Motor->Get_Now_Angle() - PI; // 协议里上电后默认角度为PI，但是发送角度时这个PI不计入，所以要减去PI

            Motor->Set_Target_Angle((Cali_Offset + 0.05f) * 100.0f); // 校准好后松开一点

#ifdef MOTOR_TEST
            Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE); // 测试用
#endif

            return true;
        }
    }
    else
    {
        locked_cnt = 0;
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
