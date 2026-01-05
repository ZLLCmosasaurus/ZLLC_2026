/**
 * @file crt_chassis.cpp
 * @author cjw
 * @brief 底盘
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    // Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2, 100.f);

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f, 0.01f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f, 0.01f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    // 电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }
#ifdef AGV
    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan2, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan2, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan2, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan2, DJI_Motor_ID_0x204);

    // 舵向电机PID初始化

    Motor_Steer[0].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[0].Get_Output_Max(), Motor_Steer[0].Get_Output_Max());
    Motor_Steer[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());

    Motor_Steer[1].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[1].Get_Output_Max(), Motor_Steer[1].Get_Output_Max());
    Motor_Steer[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[1].Get_Output_Max());

    Motor_Steer[2].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[2].Get_Output_Max(), Motor_Steer[2].Get_Output_Max());
    Motor_Steer[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[2].Get_Output_Max());

    Motor_Steer[3].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[3].Get_Output_Max(), Motor_Steer[3].Get_Output_Max());
    Motor_Steer[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[3].Get_Output_Max());

    // 舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan2, DJI_Motor_ID_0x205);
    Motor_Steer[1].Init(&hfdcan2, DJI_Motor_ID_0x206);
    Motor_Steer[2].Init(&hfdcan2, DJI_Motor_ID_0x207);
    Motor_Steer[3].Init(&hfdcan2, DJI_Motor_ID_0x208);
    // 舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(1.76999998); // 应该是轮子朝向的正方向
    Motor_Steer[1].Set_Zero_Position(2.38000011);
    Motor_Steer[2].Set_Zero_Position(2.16000009);
    Motor_Steer[3].Set_Zero_Position(2.6400001);
#endif

#ifdef OMNI_WHEEL
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x204);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x202);
#endif

    // 底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}

/**
 * @brief 速度解算
 *
 */
float car_V, car_yaw; // 车体总体朝向与速度
void Class_Tricycle_Chassis::Speed_Resolution()
{
#ifdef AGV
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);

            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Steer[i].Set_Out(0.0f);
        }
        break;
    }
    case (Chassis_Control_Type_FLLOW):
    case (Chassis_Control_Type_SPIN):
    {
        // 轮组自锁，每个小轮坐标系都符合右手系
        static uint32_t Lock_Time = 0;
        static uint8_t Lock_Flag = 0;
        float delta_Angle = 0.0f, Transform_Radian = 0.0f; // 用于优化处理的变量
        if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
        {
            Lock_Time++;
            if (Lock_Time > 100)
                Lock_Flag = 1;
            if (Lock_Flag)
            {
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

                    Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                }

                Motor_Steer[0].Set_Target_Radian(-PI / 4.0f);
                Motor_Steer[1].Set_Target_Radian(PI / 4.0f);
                Motor_Steer[2].Set_Target_Radian(-PI / 4.0f);
                Motor_Steer[3].Set_Target_Radian(PI / 4.0f);
                for (int i = 0; i < 4; i++)
                {
                    Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                    // 优劣弧处理
                    if ((i % 2) == 0)
                    {
                        delta_Angle = -PI / 4.0f - Transform_Radian;
                    }
                    else
                    {
                        delta_Angle = PI / 4.0f - Transform_Radian;
                    }
                    delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);

                    Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);
                    Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                    // Motor_Steer[i].Set_Out(0.0f);
                    // Motor_Wheel[i].Set_Out(0.0f);
                    Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                    Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                }
                break;
            }
        }

        if (Lock_Flag)
        {
            Lock_Flag = 0;
            Lock_Time = 0;
        }

        // 0 1 2 3 左前 右前 右后 左后 逆时针    前X左Y坐标系   基于编码器0度朝前，逆时针为正角度   确保轮子正转的是朝前的速度，不然得单独加负号
        float True_Vx[4], True_Vy[4], True_Target_Angle_Radian[4];

        // 斜坡处理
        True_Vx[0] = True_Vx[3] = Slope_Velocity_X.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() * CHASSIS_RADIUS / 2;
        True_Vx[1] = True_Vx[2] = Slope_Velocity_X.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() * CHASSIS_RADIUS / 2;

        True_Vy[0] = True_Vy[1] = Slope_Velocity_Y.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() * CHASSIS_RADIUS / 2;
        True_Vy[2] = True_Vy[3] = Slope_Velocity_Y.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() * CHASSIS_RADIUS / 2;

        // True_Vx[0] = True_Vx[3] = Target_Velocity_X - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
        // True_Vx[1] = True_Vx[2] = Target_Velocity_X + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

        // True_Vy[0] = True_Vy[1] = Target_Velocity_Y + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
        // True_Vy[2] = True_Vy[3] = Target_Velocity_Y - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

        // 舵轮转动角度的优化处理
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

            // 计算速度
            float temp_Target_Omega = 0.0f;
            arm_sqrt_f32(True_Vx[i] * True_Vx[i] + True_Vy[i] * True_Vy[i], &temp_Target_Omega);
            temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;

            // 计算目标角度
            if (fabs(temp_Target_Omega) < 0.0001)
            { // 避免X =0 ；Y = 0的情况
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
            }
            else
            {
                True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]); //-PI -- PI   会自动处理Vx = 0;
            }

            // 角度优化处理
            delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian(); // -2PI -- 2PI
            delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                              // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
            if (delta_Angle > PI / 2.0f)
            {
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle - PI;
                temp_Target_Omega *= -1.0f;
            }
            else if (delta_Angle < -PI / 2.0f)
            {
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle + PI;
                temp_Target_Omega *= -1.0f;
            }
            else
            {
                // 不需要处理角度
            }

            // 处理-180 - 180的突变问题    同时还有优劣弧处理
            delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();
            True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + Normalize_Angle_Radian_PI_to_PI(delta_Angle);

            Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
            Motor_Wheel[i].Set_Target_Omega_Radian(temp_Target_Omega);
        }

        for (int i = 0; i < 4; i++)
        {
            Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
            Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    }
#endif
#ifdef OMNI_WHEEL
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能 轮组无力
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
    }
    break;
    case (Chassis_Control_Type_FLLOW):
    {
        // 底盘四电机模式配置
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        // 底盘限速
        if (Velocity_X_Max != 0)
        {
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
        }
        if (Velocity_Y_Max != 0)
        {
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
        }
        if (Omega_Max != 0)
        {
            Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
        }

        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);

        // 斜坡函数
        // float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        // float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        // float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        // float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);

        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RPM * RPM2RAD;
        // 角速度*减速比  设定目标
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad * M3508_REDUCTION_RATIO);

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
    }
    break;
    case (Chassis_Control_Type_SPIN):
    {
        // 底盘四电机模式配置
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        // 底盘限速
        if (Velocity_X_Max != 0)
        {
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
        }
        if (Velocity_Y_Max != 0)
        {
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
        }
        if (Omega_Max != 0)
        {
            Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
        }

        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);

        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RPM * RPM2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RPM * RPM2RAD;
        // 角速度*减速比  设定目标
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad * M3508_REDUCTION_RATIO);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad * M3508_REDUCTION_RATIO);

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
    }
    break;
    }
#endif
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    // 速度解算
    Speed_Resolution();
    // 电机PID计算
    /***************************超级电容*********************************/
    Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max());
    Supercap.Set_Supercap_Mode(Get_Supercap_Mode());
    Supercap.TIM_Supercap_PeriodElapsedCallback();

#if POWER_CONTROL == 1
    /*************************功率限制策略*******************************/
    // Power_Limit_Update();
    Power_Limit.Set_Motor(Motor_Wheel); // 添加四个电机的控制电流和当前转速
    Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    if (Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        if (Supercap.Get_Consuming_Power() <= 200.f)
        {
            Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max() / 3.0f);
            Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() / 3.0f);
        }
    }
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);
#endif
}

void Class_Tricycle_Chassis::Power_Limit_Update()
{
}

/** ----------------------------麦克纳姆轮底盘类-------------------------------------- **/
void Class_Mecanum_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    // Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2, 100.f);

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f, 0.01f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f, 0.01f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    // // 电机PID初始化
    // Mecanum_Wheels[0].PID_Omega.Init(850.0f, 0.0f, 0.0f, 0.0f, Mecanum_Wheels[0].Get_Output_Max(), Mecanum_Wheels[0].Get_Output_Max());
    // Mecanum_Wheels[1].PID_Omega.Init(800.0f, 0.0f, 0.0f, 0.0f, Mecanum_Wheels[1].Get_Output_Max(), Mecanum_Wheels[1].Get_Output_Max());
    // Mecanum_Wheels[2].PID_Omega.Init(800.0f, 0.0f, 0.0f, 0.0f, Mecanum_Wheels[2].Get_Output_Max(), Mecanum_Wheels[2].Get_Output_Max());
    // Mecanum_Wheels[3].PID_Omega.Init(800.0f, 0.0f, 0.0f, 0.0f, Mecanum_Wheels[3].Get_Output_Max(), Mecanum_Wheels[3].Get_Output_Max());
    
    // 抬升电机PID初始化
    for(int i = 0; i < 4; i++)
    {
        Uplift_Motor[i].PID_Omega.Init(1200.0f, 0.0f, 0.0f, 0.0f, Uplift_Motor[i].Get_Output_Max(), Uplift_Motor[i].Get_Output_Max());
        Uplift_Motor[i].PID_Angle.Init(4.5f, 0.0f, 0.0f, 0.0f,0.0f, 7.0f * PI);
    }

    // // 麦轮轮组电机ID初始化
    // Mecanum_Wheels[0].Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 3591.0f / 187.0f);
    // Mecanum_Wheels[1].Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 3591.0f / 187.0f);
    // Mecanum_Wheels[2].Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 3591.0f / 187.0f);
    // Mecanum_Wheels[3].Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA, 3591.0f / 187.0f);

    // 抬升电机ID初始化
    Uplift_Motor[0].Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_ANGLE);
    Uplift_Motor[1].Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_ANGLE);
    Uplift_Motor[2].Init(&hfdcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_ANGLE);
    Uplift_Motor[3].Init(&hfdcan2, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_ANGLE);

    // 主动轮电机ID初始化
    Track_Motor[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_OMEGA);
    Track_Motor[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_OMEGA);
    
    // 底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    // 抬升状态机传指针
    Calibration_FSM.Chassis = this;
}

void Class_Mecanum_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Mecanum_Wheels[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Mecanum_Wheels[i].Set_Target_Omega_Radian(0.0f);
            Mecanum_Wheels[i].PID_Omega.Set_Integral_Error(0.0f);
            Mecanum_Wheels[i].Set_Out(0.0f);
        }
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        // 设置电机控制模式
        for (int i = 0; i < 4; i++)
        {
            Mecanum_Wheels[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }

        // 底盘限速
        if (Velocity_X_Max != 0)
        {
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
        }
        if (Velocity_Y_Max != 0)
        {
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
        }
        if (Omega_Max != 0)
        {
            Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            Target_Velocity_Steer = Target_Omega * (half_length + half_width);
        }

        // 整体线速度分配至四个轮子的线速度
        Target_Motor_Velocity[0] = Target_Velocity_X - Target_Velocity_Y - Target_Velocity_Steer;
        Target_Motor_Velocity[1] = Target_Velocity_X + Target_Velocity_Y + Target_Velocity_Steer;
        Target_Motor_Velocity[2] = Target_Velocity_X - Target_Velocity_Y + Target_Velocity_Steer;
        Target_Motor_Velocity[3] = Target_Velocity_X + Target_Velocity_Y - Target_Velocity_Steer;

        // 以抬升为后方，1和2的轮子转向相反，在这里取负
        Target_Motor_Velocity[1] = -Target_Motor_Velocity[1];
        Target_Motor_Velocity[2] = -Target_Motor_Velocity[2];

        // 轮子线速度转角速度，给电机设置目标值
        for (int i = 0; i < 4; i++)
        {
            Target_Motor_Omega[i] = Target_Motor_Velocity[i] * VEL2RAD;
            Math_Constrain(Target_Motor_Omega, -Wheels_Omega_Max, Wheels_Omega_Max);
            Mecanum_Wheels[i].Set_Target_Omega_Radian(Target_Motor_Omega[i]);
        }

        break;
    }
    case (Chassis_Control_Type_SLOPE):
    {
        // 设置电机控制模式
        for (int i = 0; i < 4; i++)
        {
            Mecanum_Wheels[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }

        float Velocity_X_Slope = Slope_Velocity_X.Get_Out();
        float Velocity_Y_Slope = Slope_Velocity_Y.Get_Out();
        float Velocity_Steer_Slope = Slope_Omega.Get_Out() * (half_length + half_width);

        // 整体线速度分配至四个轮子的线速度
        Target_Motor_Velocity[0] = Velocity_X_Slope - Velocity_Y_Slope - Velocity_Steer_Slope;
        Target_Motor_Velocity[1] = Velocity_X_Slope + Velocity_Y_Slope + Velocity_Steer_Slope;
        Target_Motor_Velocity[2] = Velocity_X_Slope - Velocity_Y_Slope + Velocity_Steer_Slope;
        Target_Motor_Velocity[3] = Velocity_X_Slope + Velocity_Y_Slope - Velocity_Steer_Slope;

        // 以抬升为后方，1和2的轮子转向相反，在这里取负
        Target_Motor_Velocity[1] = -Target_Motor_Velocity[1];
        Target_Motor_Velocity[2] = -Target_Motor_Velocity[2];

        // 轮子线速度转角速度，给电机设置目标值
        for (int i = 0; i < 4; i++)
        {
            Target_Motor_Omega[i] = Target_Motor_Velocity[i] * VEL2RAD;
            Math_Constrain(Target_Motor_Omega, -Wheels_Omega_Max, Wheels_Omega_Max);
            Mecanum_Wheels[i].Set_Target_Omega_Radian(Target_Motor_Omega[i]);
        }

        break;
    }
    }
}

void Class_Mecanum_Chassis::Output()
{
    if(Chassis_Control_Type == Chassis_Control_Type_DISABLE)
    {
        // 抬升电机
        for(int i = 0; i < 4; i++){
            Uplift_Motor[i].Set_Out(0.0f);
        }

        // 主动轮电机，直接设置为失能，如果是为了防止打滑，可以换成目标速度设为0
        Track_Motor[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Track_Motor[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
    else
    {
        // 抬升电机
        for(int i = 0; i < 4; i++)
        {
            Uplift_Motor[i].Set_Target_Radian(Target_Uplift_Motor_Radian[i]);
        }

        for(int i = 0; i < 4; i++)
        {
            Uplift_Motor[i].TIM_PID_PeriodElapsedCallback();
        }

        // 主动轮电机
        Track_Motor[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Track_Motor[0].Set_Target_Omega(Target_Track_Omega);

        Track_Motor[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Track_Motor[1].Set_Target_Omega(-Target_Track_Omega);
    }

}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Mecanum_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    // 斜坡函数计算用于速度解算初始值获取
    // Slope_Velocity_X.Set_Target(Target_Velocity_X);
    // Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    // Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    // Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    // Slope_Omega.Set_Target(Target_Omega);
    // Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    // // 速度解算
    // Speed_Resolution();
    // // 电机PID计算
    // for (int i = 0; i < 4; i++)
    // {
    //     Mecanum_Wheels[i].TIM_PID_PeriodElapsedCallback();
    // }

    // 将角度赋值给底盘中的电机，以及PID计算
    Output();

    /***************************超级电容*********************************/
    Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max());
    Supercap.Set_Supercap_Mode(Get_Supercap_Mode());
    Supercap.TIM_Supercap_PeriodElapsedCallback();

#if POWER_CONTROL == 1
    /*************************功率限制策略*******************************/
    // Power_Limit_Update();
    Power_Limit.Set_Motor(Mecanum_Wheels); // 添加四个电机的控制电流和当前转速
    Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    if (Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        if (Supercap.Get_Consuming_Power() <= 200.f)
        {
            Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max() / 3.0f);
            Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() / 3.0f);
        }
    }
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Mecanum_Wheels);
#endif
}

/**
 * @brief 单编码器电机校准状态机
 *
 */
void Class_FSM_Calibration_Chassis::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0):
        /*校准状态*/
        {
            bool all_cali_status = true;
            for(int i = 0; i < 4; i++)
            {
                if(Chassis->Uplift_Motor[i].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && !uplift_cali_status[i])
                {
                    uplift_cali_status[i] = Motor_Calibration(Chassis->Uplift_Motor + i, i, uplift_cali_torque, uplift_locked_cnt[i]);
                }

                if(uplift_cali_status[i])
                {
                    Chassis->Uplift_Max_Radian[i] = uplift_offset[i] - 2.0f;
                    Chassis->Uplift_Min_Radian[i] = Chassis->Uplift_Max_Radian[i] - (i < 2 ? 26.5f : 20.0f);
                }

                all_cali_status = all_cali_status && uplift_cali_status[i];
            }

            if(all_cali_status)
            {
                Set_Status(1);
            }

            break;
        }
    case (1):
        /*校准完成状态*/
        {
            bool online_status = true;
            for(int i = 0; i < 4; i++)
            {
                online_status = online_status && (Chassis->Uplift_Motor[i].Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
            }

            if(!online_status)
            {
                Set_Status(0);
            }

            break;
        }
    }
}

/**
 * @brief 校准执行函数 C610 - 2006
 *
 */
bool Class_FSM_Calibration_Chassis::Motor_Calibration(Class_DJI_Motor_C620 *Motor, uint8_t i, float locked_torque, uint16_t &locked_cnt)
{
    Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    Motor->Set_Target_Radian(30.0f);

    if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (Motor->Get_Now_Omega_Radian() <= 0.01f))
    {
        locked_cnt++;
        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            uplift_offset[i] = Motor->Get_Now_Radian();

            Motor->Set_Target_Radian(uplift_offset[i] - 2.0f); // 校准完成后稍微松开一点，避免一直堵转，校准是往张开的方向动的，所以这里往加紧的方向动一下

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
