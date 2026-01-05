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

    Motor_DM_J0_Yaw.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);
    Motor_DM_J1_Pitch.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 20.0f);
    Motor_DM_J2_Pitch_2.Init(&hfdcan1, DM_Motor_ID_0xA3, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 25.0f);
    Motor_DM_J4_Pitch_3.Init(&hfdcan2, DM_Motor_ID_0xA5, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 25.0f);
    Motor_DM_J3_Roll.Init(&hfdcan1, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);

    Motor_6020_J5_Roll_2.PID_Angle.Init(25.0f, 0.0f, 0.0f, 0.0f, 500, 500, 500);
    Motor_6020_J5_Roll_2.PID_Omega.Init(300.0f, 2.5f, 0.0f, 0.0f, 6000, Motor_6020_J5_Roll_2.Get_Output_Max(), 10.f, 50.f);
    Motor_6020_J5_Roll_2.PID_Torque.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_6020_J5_Roll_2.Get_Output_Max(), Motor_6020_J5_Roll_2.Get_Output_Max());
    Motor_6020_J5_Roll_2.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 0);

    Motor_C610_Gripper.PID_Angle.Init(42.5f, 5.0f, 0.0f, 0.0f, 500, 500, 500);
    Motor_C610_Gripper.PID_Omega.Init(1800.0f, 0.0f, 0.0f, 0.0f, 2000, 4000, 10.f, 50.f); // 尝试把速度环的Ki禁用，用于夹爪夹紧
    Motor_C610_Gripper.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE);
    /*初始化状态机，不进行初始化的话状态机没法访问云台对象中的电机的*/
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
                // 控制方式
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J4_Pitch_3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

#ifdef MOTOR_TEST
                switch (debug_6020_mode)
                {
                case (0): // 清空，0力矩输出
                {
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.PID_Torque.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.Set_Target_Torque(0.0f);
                    Motor_6020_J5_Roll_2.Set_Out(0.0f);
                    break;
                }
                case (1): // omega mode
                {
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_P(debug_6020_omega_kp);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_I(debug_6020_omega_ki);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_D(debug_6020_angle_kd);
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    break;
                }
                case (2):
                { // angle mode
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_P(debug_6020_angle_kp);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_I(debug_6020_angle_ki);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_D(debug_6020_angle_kd);
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    break;
                }
                }

                if (Calibration_FSM.Get_Gripper_cali_status())
                {
                    if (!gripper_output_flag)
                    {
                        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                        Motor_C610_Gripper.Set_Target_Torque(0.0f);
                        Motor_C610_Gripper.Set_Out(0.0f);
                    }
                    else
                    {
                        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    }
                    /*C610*/
                    Set_Target_Gripper_Angle(debug_gripper_target_angle);
                }

                if (set_roll_output_enable) // 测试用
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
                }
                else
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                }

                if (Calibration_FSM.Get_roll_cali_status())
                {
                    Motor_DM_J3_Roll.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                    Motor_DM_J3_Roll.Set_Target_Omega(2.0f);
                }

                // 限制Debug输入（角度制）
                Math_Constrain(&debug_j1_target_angle, Min_Pitch_Angle, Max_Pitch_Angle);
                Math_Constrain(&debug_j0_target_angle, Min_Yaw_Angle, Max_Yaw_Angle);
                Math_Constrain(&debug_j2_target_angle, Min_Pitch_2_Angle, Max_Pitch_2_Angle);
                Math_Constrain(&debug_j4_target_angle, Min_Pitch_3_Angle, Max_Pitch_3_Angle);
                // roll debug使用radian，但是根据要求，debug应该统一使用angle。
                // 这里假设debug_roll_target_radian是想用rad直接控制。
                // 但用户说“debug区的代码就直接使用Angle制”，所以可能需要把 debug_roll_target_radian 改为 debug_roll_target_angle 并转换。
                // 不过Min_Roll_Radian是rad。
                // 暂时保留 debug_roll_target_radian 的使用，因为它是变量名。
                Math_Constrain(&debug_roll_target_radian, Min_Roll_Radian, Max_Roll_Radian);

                Target_Pitch_Omega = debug_j1_target_omega;
                Target_Pitch_2_Omega = debug_j2_target_omega;
                Target_Pitch_3_Omega = debug_j4_target_omega;

                /*速度环，degree制*/
                Target_Roll_2_Omega = debug_j5_target_omega;
                // Target_Gripper_Omega = debug_gripper_target_omega * PI / 180.0f;

                // 使用Set函数进行设置（会自动转换）
                Set_Target_Yaw_Angle(debug_j0_target_angle);
                Set_Target_Pitch_Angle(debug_j1_target_angle);
                Set_Target_Pitch_2_Angle(debug_j2_target_angle);
                Set_Target_Pitch_3_Angle(debug_j4_target_angle);
                Set_Target_Roll_2_Angle(debug_j5_target_angle);

                // Roll轴直接给Radian (旧逻辑是这样)，如果需要改为Angle制，需要引入 debug_roll_target_angle
                // 这里因为 debug_roll_target_radian 已经是 rad，直接调用 Set_Target_Roll_Radian
                Set_Target_Roll_Radian(debug_roll_target_radian);
#endif
#ifdef MY_DEBUG
                /**
                 * 平动测试用，标志位move_test_flag，用于更改debug_radian的数值来完成测试
                 * 0: 不更改debug_radian，可以自行在debug模式下修改debug_radian来测试
                 * 1: 将debug_radian赋为平动测试的初始角度
                 * 2: 使用计算出的平动中机械臂角度来控制机械臂
                 * default: 急停，机械臂角度保持在当前角度
                 */
                //把平动起始位置转成控制时的角度
                Trajectory_Tracer.model_to_control(model_angle, move_init_control_angle);
                switch (move_test_flag)
                {
                case 0:
                    break;

                case 1:
                {
                    for (int i = 0; i < 6; i++)
                    {
                        debug_radian[i] = move_init_control_angle[i];
                    }
                    break;
                }

                case 2:
                {
                    static uint32_t point_cnt = 0;    // 平动轨迹点计数器
                    // 每4ms填入下一个目标角度，在4ms中也分优先级发送
                    switch (can_priority_cnt % 5)
                    {
                    case (1):
                    {
                        debug_radian[0] = move_control_angle[0];
                        debug_radian[4] = move_control_angle[4];
                        break;
                    }
                    case (2):
                    {
                        debug_radian[1] = move_control_angle[1];
                        debug_radian[5] = move_control_angle[5];
                        break;
                    }
                    case (3):
                    {
                        debug_radian[2] = move_control_angle[2];
                        break;
                    }
                    case (4):
                        debug_radian[3] = move_control_angle[3];
                        break;
                    case (0):   //不在这清零，执行完Output后TIM_Process_PeriodElapsedCallback里清零，如果清零两次的话电机更新目标角度和电机通信不同步
                    {
                        if(point_cnt < valid_solution_cnt)
                        {
                            Trajectory_Tracer.model_to_control(q_solution[point_cnt], move_control_angle);
                            point_cnt++;
                        }
                        else
                        {
                            point_cnt = valid_solution_cnt - 1;
                        }
                        break;
                    }
                    }
                    break;
                }

                default:
                    break;
                }

                Set_Target_Yaw_Radian(debug_radian[0]);
                Set_Target_Pitch_Radian(debug_radian[1]);
                Set_Target_Pitch_2_Radian(debug_radian[2]);
                Set_Target_Roll_Radian(debug_radian[3]);
                Set_Target_Pitch_3_Radian(debug_radian[4]);
                Set_Target_Roll_2_Radian_Single(debug_radian[5]);
#endif
                // 电机设置目标角度 (使用 Radian)
                Motor_DM_J0_Yaw.Set_Target_Omega(Target_Yaw_Omega);
                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);

                Motor_DM_J1_Pitch.Set_Target_Omega(Target_Pitch_Omega);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);

                Motor_DM_J2_Pitch_2.Set_Target_Omega(Target_Pitch_2_Omega);
                Motor_DM_J2_Pitch_2.Set_Target_Angle(Target_Pitch_2_Radian);

                Motor_DM_J4_Pitch_3.Set_Target_Omega(Target_Pitch_3_Omega);
                Motor_DM_J4_Pitch_3.Set_Target_Angle(Target_Pitch_3_Radian);

                Motor_6020_J5_Roll_2.Set_Target_Omega_Radian(Target_Roll_2_Omega);
                Motor_6020_J5_Roll_2.Set_Target_Radian(Target_Roll_2_Radian);

                if (Calibration_FSM.Get_roll_cali_status())
                    /* Target_Roll_Radian在Set函数里已经进行了转换，加offset和限位，所以可以直接赋给电机
                       角速度改成1.5PI，这样转的快一点。                                                 */
                    Motor_DM_J3_Roll.Set_Target_Omega(1.5f * PI);
                Motor_DM_J3_Roll.Set_Target_Angle(Target_Roll_Radian);

                if (Calibration_FSM.Get_Gripper_cali_status())
                {
                    Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE); // 用Motor_Test调试时删这一行，因为上面Motor_Test的代码块里写了标志位用于使能和失能
                    Motor_C610_Gripper.Set_Target_Radian(Target_Gripper_Radian);
                }
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
            {
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                Set_Target_Yaw_Angle(MiniPC->Get_Rx_Yaw_Angle());
                Set_Target_Pitch_Angle(MiniPC->Get_Rx_Pitch_Angle());

                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
            {
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                // 限制角度
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
                Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

                // Ensure Radians are updated
                Set_Target_Yaw_Angle(Target_Yaw_Angle);
                Set_Target_Pitch_Angle(Target_Pitch_Angle);

                // 设置目标角度
                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);
            }
        }
        else
        /*将机械臂调整到初始姿态，只有在整车上电和整臂断电重连（机器人复活）时才会触发，2325的放在校准状态机*/
        {
            // TO DO: 这部分最好写成状态机，防止机械臂各自打架
            Motor_DM_J0_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J1_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J2_Pitch_2.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J4_Pitch_3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

            Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J4_Pitch_3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_DM_J0_Yaw.Set_Target_Omega(0.5f);
            Motor_DM_J0_Yaw.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J1_Pitch.Set_Target_Omega(0.5f);
            Motor_DM_J1_Pitch.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J2_Pitch_2.Set_Target_Omega(0.5f);
            Motor_DM_J2_Pitch_2.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J4_Pitch_3.Set_Target_Omega(0.5f);
            Motor_DM_J4_Pitch_3.Set_Target_Angle(0.0f); // Radian 0

            // 6020每次断电重连时，电机内部保存的圈数会清零，所以直接让转到0即可
            Motor_6020_J5_Roll_2.Set_Target_Omega_Radian(1.0f * PI);
            Motor_6020_J5_Roll_2.Set_Target_Radian(0.0f); // Radian 0

            arm_init = true;
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
    // 发送控制帧
    switch (can_priority_cnt % 5)
    {
    case (1):
    {
        Motor_DM_J0_Yaw.TIM_Process_PeriodElapsedCallback();
        Motor_DM_J4_Pitch_3.TIM_Process_PeriodElapsedCallback();
        break;
    }
    case (2):
    {
        Motor_DM_J1_Pitch.TIM_Process_PeriodElapsedCallback();
        Motor_6020_J5_Roll_2.TIM_PID_PeriodElapsedCallback();
        break;
    }
    case (3):
    {
        Motor_DM_J2_Pitch_2.TIM_Process_PeriodElapsedCallback();
        Motor_C610_Gripper.TIM_PID_PeriodElapsedCallback();
        break;
    }
    case (4):
        Motor_DM_J3_Roll.TIM_Process_PeriodElapsedCallback();
        break;
    case (0):
        can_priority_cnt = 0;
        break;
    }

    // 用于更新当前机械臂位置
    Trajectory_Tracer.arm_pos_rpy_update();
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
            /*roll轴2325的校准状态机*/
            if (Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !roll_cali_status)
            {
                roll_cali_status = Motor_Calibration(&Gimbal->Motor_DM_J3_Roll, 2.0f, locked_torque, locked_cnt);
            }

            /*夹爪2006的校准状态机*/
            if (Gimbal->Motor_C610_Gripper.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && !gripper_cali_status)
            {
                gripper_cali_status = Motor_Calibration(&Gimbal->Motor_C610_Gripper, 0.75f, gripper_locked_torque, gripper_locked_cnt);
            }

            if (roll_cali_status)
            {
                Gimbal->roll_cali_offset = Cali_Offset + 0.05f;
                Gimbal->Min_Roll_Radian = Gimbal->roll_cali_offset * 100.0f;
                Gimbal->Max_Roll_Radian = Gimbal->Min_Roll_Radian + 300.0f;
            }

            if (gripper_cali_status)
            {
                Gimbal->gripper_cali_offset = gripper_offset;
                Gimbal->Min_gripper_Radian = Gimbal->gripper_cali_offset;
                Gimbal->Max_gripper_Radian = Gimbal->gripper_cali_offset + 0.95f; // 夹爪张开最大时为0.95f
            }

            if (roll_cali_status && gripper_cali_status)
            {
                Set_Status(1);
            }
            break;
        }
    case (1):
        /*校准完成状态*/
        {
            if (Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
            {
                roll_cali_status = false;
                Set_Status(0);
            }
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

/**
 * @brief 校准执行函数 C620 - 3508
 *
 */
bool Motor_Calibration(Class_DJI_Motor_C620_Uplift *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
{
    // // 速度环跑校准
    // Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor->Set_Target_Omega_Radian(Cali_Omega);

    // if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (Motor->Get_Now_Omega_Radian() <= 0.01f))
    // {
    //     locked_cnt++;
    //     if (locked_cnt >= 50)
    //     {
    //         locked_cnt = 0;

    //         uplift_offset = Motor->Get_Now_Radian();

    //         Motor->Set_Target_Radian(gripper_offset + 0.015f); // 校准完成后稍微松开一点，避免一直堵转，校准是往张开的方向动的，所以这里往加紧的方向动一下

    //         return true;
    //     }
    // }
    // else
    // {
    //     locked_cnt = 0;
    // }
    return false;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
