#include "crt_force_control_chassis.h"

void Class_Chassis::Init()
{
    // PID初始化

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(60.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(70.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.002f);

    // 轮向电机ID初始化
    // 电机初始化
    Motor_Wheel[0].Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[0].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[1].Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[1].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[2].Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[2].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[3].Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[3].PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);

    // 超级电容初始化
    Supercap.Init_UART(&huart1);

    // imu初始化
    Boardc_BMI.Init();

    // 低通滤波初始化
    // 实际速度滤波
    //  PID_Velocity_Filter[0].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    //  PID_Velocity_Filter[1].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    //  PID_Omega_Filter.Init(-20.0f,20.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,15);

    // Spike滤波初始化
    init_filter(&Velocity_X_Spike, 5);
    init_filter(&Velocity_Y_Spike, 5);
    init_filter(&Omega_Spike, 5);

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f, 0.01f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f, 0.01f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);
}

// #define Control_Type_Oemga
#define Control_Type_Current

void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
        // Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}
/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();

    // PID_Velocity_Filter[0].Set_Now(Now_Velocity_X);
    // PID_Velocity_Filter[0].TIM_Adjust_PeriodElapsedCallback();
    // PID_Velocity_Filter[1].Set_Now(Now_Velocity_Y);
    // PID_Velocity_Filter[1].TIM_Adjust_PeriodElapsedCallback();
    // PID_Omega_Filter.Set_Now(Now_Omega);
    // PID_Omega_Filter.TIM_Adjust_PeriodElapsedCallback();

    // Spike滤波处理函数
    Now_Velocity_X_Spike = process_sample(&Velocity_X_Spike, Now_Velocity_X);
    Now_Velocity_Y_Spike = process_sample(&Velocity_Y_Spike, Now_Velocity_Y);
    Now_Omega_Spike = process_sample(&Omega_Spike, Now_Omega);

    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    // 卡尔曼滤波器更新车体x方向速度
    // xvEstimateKF_Update(&vaEstimateKF,Acceleration_X_Filter.Get_Out(),Now_Velocity_X);

    // INS_Data.Mix_Velocity_X = vel_acc[0];
}

/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    Kinematics_Inverse_Resolution();

#ifdef Control_Type_Current

    Output_To_Dynamics();

    Dynamics_Inverse_Resolution();

#endif

    Output_To_Motor();
}

/**
 * @brief 运动学逆解算，车身速度结算到电机角速度，用于电机速度环
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    Target_Wheel_Velocity[0] = Slope_Velocity_X.Get_Out() - Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * (half_l + half_w);
    Target_Wheel_Velocity[1] = Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * (half_l + half_w);
    Target_Wheel_Velocity[2] = Slope_Velocity_X.Get_Out() - Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * (half_l + half_w);
    Target_Wheel_Velocity[3] = Slope_Velocity_X.Get_Out() + Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * (half_l + half_w);

    Target_Wheel_Omega[0] = Target_Wheel_Velocity[0] / Wheel_Radius;
    Target_Wheel_Omega[1] = Target_Wheel_Velocity[1] / Wheel_Radius;
    Target_Wheel_Omega[2] = Target_Wheel_Velocity[2] / Wheel_Radius;
    Target_Wheel_Omega[3] = Target_Wheel_Velocity[3] / Wheel_Radius;

    Target_Wheel_Omega[1] = -Target_Wheel_Omega[1];
    Target_Wheel_Omega[2] = -Target_Wheel_Omega[2];
}
/**
 * @brief 输出到动力学状态
 *
 */
void Class_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE__):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {
            PID_Velocity_X.Set_Integral_Error(0.0f);
            PID_Velocity_Y.Set_Integral_Error(0.0f);
            PID_Omega.Set_Integral_Error(0.0f);
        }

        break;
    }
    case (Chassis_Control_Type_NORMAL__):
    {
        PID_Velocity_X.Set_Target(Slope_Velocity_X.Get_Out());
        PID_Velocity_X.Set_Now(Now_Velocity_X_Spike);
        PID_Velocity_X.TIM_Adjust_PeriodElapsedCallback();

        PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
        PID_Velocity_Y.Set_Now(Now_Velocity_Y_Spike);
        PID_Velocity_Y.TIM_Adjust_PeriodElapsedCallback();

        PID_Omega.Set_Target(Slope_Omega.Get_Out());
        PID_Omega.Set_Now(Now_Omega_Spike);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        break;
    }
    }
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega;

    force_x = PID_Velocity_X.Get_Out();
    force_y = PID_Velocity_Y.Get_Out();
    torque_omega = PID_Omega.Get_Out();

    Target_Wheel_Force[0] = force_x - force_y - torque_omega / (half_l + half_w);
    Target_Wheel_Force[1] = force_x + force_y + torque_omega / (half_l + half_w);
    Target_Wheel_Force[2] = force_x - force_y + torque_omega / (half_l + half_w);
    Target_Wheel_Force[3] = force_x + force_y - torque_omega / (half_l + half_w);

    Target_Wheel_Force[1] = -Target_Wheel_Force[1];
    Target_Wheel_Force[2] = -Target_Wheel_Force[2];

    for (int i = 0; i < 4; i++)
    {
        // #define force (20.0f/(0.154f/2.0f)*7.0f*0.01562f) 预测最大的摩擦力为28.4N左右
        //  摩擦力转换至扭矩
        Target_Wheel_Current[i] = (Target_Wheel_Force[i] * Wheel_Radius) + Wheel_Speed_Limit_Factor[i] * Motor_Omega_Contration * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega()); //(tmp_force[i] * Wheel_Radius) / (13.933f * 0.5f) / M3508_Kt;//Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega());
        // 动摩擦阻力前馈
        if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] += Dynamic_Resistance_Wheel_Current[i];
        }
        else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] -= Dynamic_Resistance_Wheel_Current[i];
        }
        else
        {
            Target_Wheel_Current[i] += Motor_Wheel[i].Get_Now_Omega() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
        }
    }

    // 根据斜坡与压力进行电流限幅防止贴地打滑
    // TODO
}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE__):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {

            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);

            Motor_Wheel[i].Set_Target_Current(0.0f);
        }

        break;
    }
    case (Chassis_Control_Type_NORMAL__):
    {
#ifdef Control_Type_Current
        // 全向模型
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
        }

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Target_Current(Target_Wheel_Current[i]);
        }
#endif
#ifdef Control_Type_Oemga
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
        }
#endif

        break;
    }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }

    // 进行功率限制
    Power_Management.Max_Power = 120.0f;
    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Reset_Set_Out_And_Output(Power_Management.Motor_Data[i].output);
    }
}

/**
 * @brief 自身解算
 *
 */
void Class_Chassis::Self_Resolution()
// 正运动学解算
{
    // static float last_X = 0.0f;
    // static float last_Y = 0.0f;
    // static float last_Omega = 0.0f;
    // 根据电机编码器与陀螺仪计算速度和角度

    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega = 0.0f;

    // 轮线速度的计算方式 v = Ω * r，根据正运动学测得得结果，1和2号的轮子转向反了，这里取负
    float wheel_vel0 = Motor_Wheel[0].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel1 = -Motor_Wheel[1].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel2 = -Motor_Wheel[2].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel3 = Motor_Wheel[3].Get_Now_Omega() * Wheel_Radius;

    // 从逆运动学方程推导正运动学：

    // 建立轮速向量
    float wheel_vel[4] = {wheel_vel0, wheel_vel1, wheel_vel2, wheel_vel3};
    // 因此正运动学公式为：
    for (int i = 0; i < 4; i++)
    {
        Now_Velocity_X += wheel_vel[i];

        if (i % 2 == 0)
        {
            Now_Velocity_Y -= wheel_vel[i];
        }
        else
        {
            Now_Velocity_Y += wheel_vel[i];
        }
    }

    Now_Velocity_X *= 0.25f;
    Now_Velocity_Y *= 0.25f;
    Now_Omega = (-wheel_vel[0] + wheel_vel[1] + wheel_vel[2] - wheel_vel[3]) / (4.0f * (half_l + half_w));

    // bool flag = (fabs(Now_Velocity_X - last_X) >= 1.0f) || (fabs(Now_Velocity_Y - last_Y)) || (fabs(Now_Omega - last_Omega));

    // if(flag)
    // {

    // }

    // last_X = Now_Velocity_X;
    // last_Y = Now_Velocity_Y;
    // last_Omega = Now_Omega;

    for (int i = 0; i < 4; i++) // 数据传递处理
    {
        // 都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega() / RPM_TO_RADPS * M3508_REDUATION;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE; // 与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Target_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;       // 与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Target_Current() * 16384.0f / 20.0f;
    }
}
