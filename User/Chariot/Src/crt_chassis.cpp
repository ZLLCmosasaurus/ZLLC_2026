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
 * @brief 关节电机过热检测状态机
 * 
 */
void Class_FSM_OverHeated_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    Heat = Chassis -> Get_Now_Joint_Heat();
    switch (Now_Status_Serial)
    {
    case (0):
    {
        //正常状态

        if(Heat > 90.0f)//如果电机温度达到90℃，100℃电机过温
        {
            Set_Status(1);
        }

    }
    break;
    case (1):
    {
        Chassis -> Set_Pose_Control_Type(Pose_DISABLE);
        if(Heat < 50.0f && Status[1].Time > 3000)//至少冷却3s
        {
            Set_Status(0);
        }
    }
    break;

    }
}
/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
#ifdef TRACK_LEG
/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
void Class_HybridTrackLeg_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    //Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2,45.f);

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f,0.01f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f,0.01f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    //imu初始化
    // BoardDM_BMI.Init();
    //过热检测状态机初始化
    FSM_OverHeated_Detect.Chassis = this;
    FSM_OverHeated_Detect.Init(2,0);

    //轮向电机PID初始化
    Motor_Wheel[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());
    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    //关节电机PID初始化
    //DM速度位置控制模式Kp、Ki参数需要用上位机调节
    //关节电机ID初始化
    Motor_Joint[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA);
    Motor_Joint[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA);

    //履带驱动电机PID初始化
    Motor_Track[0].PID_Omega.Init(1500.0f, 50.0f, 0.0f, 0.0f, Motor_Track[0].Get_Output_Max(), Motor_Track[0].Get_Output_Max());
    Motor_Track[1].PID_Omega.Init(1250.0f, 50.0f, 0.0f, 0.0f, Motor_Track[1].Get_Output_Max(), Motor_Track[1].Get_Output_Max());//需调参
    //履带电机ID初始化
    Motor_Track[0].Init(&hfdcan2,DJI_Motor_ID_0x201);
    Motor_Track[1].Init(&hfdcan2,DJI_Motor_ID_0x202);

    //底部导轮电机PID初始化
    Motor_Guider[0].PID_Omega.Init(650.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[0].Get_Output_Max(), Motor_Guider[0].Get_Output_Max());
    Motor_Guider[1].PID_Omega.Init(700.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[1].Get_Output_Max(), Motor_Guider[1].Get_Output_Max());//需调参
    //底部导轮电机ID初始化
    Motor_Guider[0].Init(&hfdcan2, DJI_Motor_ID_0x203);
    Motor_Guider[1].Init(&hfdcan2, DJI_Motor_ID_0x204);

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}
#endif


/**
 * @brief 速度解算
 *
 */
float car_V,car_yaw;//车体总体朝向与速度
#ifdef TRACK_LEG
/**
 * @brief 速度解算
 *
 */
void Class_HybridTrackLeg_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case(Chassis_Control_Type_DISABLE):
    {
        for(int i = 0; i < 4;i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
        break;
    }
    case(Chassis_Control_Type_SPIN_Positive):
    case(Chassis_Control_Type_SPIN_Negative):
    case(Chassis_Control_Type_FLLOW):
    {
        //电机模式配置
        //轮向电机
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        
        //底盘限速
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
        }
        #ifdef SPEED_SLOPE
        //速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        #else
        //速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        #endif            
        //线速度 cm/s  转角速度  RAD 
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        //角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(  motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(- motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(- motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(  motor4_temp_rad);

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    }
}
#endif

/**
 * @brief 关节电机控制状态函数
 * 
 */
void Class_HybridTrackLeg_Chassis::Jointleg_Controller()
{
    #ifdef LOCKED_SWITCH
    #endif
    #ifdef AUTO_SWITCH //自动伸缩腿，已注释，不过目前是开环，后期可做成闭环
    static uint16_t mod2s = 0;// 2s重置计数器
    static uint8_t  pose_state = 1; // 位姿控制状态 0-Enable 1-Standby 
    Chassis_Pitch = BoardDM_BMI.Get_Angle_Pitch();
    Error_Pitch = Chassis_Pitch;
    Joint_Heat = Motor_Joint[1].Get_Now_Rotor_Temperature();
    // if (Error_Pitch > 10.0f)
    // {
    //     // 如果误差大于 10°，切换到 ENABLE
    //     if (pose_state != 0)
    //     {
    //         Set_Pose_Control_Type(Pose_ENABLE);
    //         pose_state = 0;
    //         mod2s = 0;  // 重置计时器
    //     }
    // }
    // else
    // {
    //     // 如果误差小于等于 10°，检查是否需要切换回 STANDBY
    //     if (pose_state == 0)
    //     {
    //         // 如果当前在 ENABLE 状态，计时器累加
    //         mod2s++;
    //         if (mod2s >= 2000)  // 2000ms = 2s
    //         {
    //             Set_Pose_Control_Type(Pose_STANDBY);
    //             pose_state = 1;
    //             mod2s = 0;  // 重置计时器
    //         }
    //     }
    // }
    switch (Pose_Control_Type)
    {
        case(Pose_DISABLE)://失能
    {
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
		Motor_Joint[0].Set_Target_Angle(0);
        Motor_Joint[0].Set_Target_Omega(0);
        Motor_Joint[1].Set_Target_Angle(0);
        Motor_Joint[1].Set_Target_Omega(0);
        break;
    }
    case(Pose_STANDBY)://待机
    {
        //启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        //设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        //位置速度模式
        Motor_Joint[0].Set_Target_Angle(Set_Leg_Angle[0]/180.0f*PI);
        Motor_Joint[0].Set_Target_Omega(Set_Leg_Velocity[0]);
        Motor_Joint[1].Set_Target_Angle(-Set_Leg_Angle[0]/180.0f*PI);
        Motor_Joint[1].Set_Target_Omega(-Set_Leg_Velocity[0]);
        break;
    }
    case(Pose_ENABLE)://使能
    {
        //启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        //设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        //位置速度模式
        Motor_Joint[0].Set_Target_Angle(Set_Leg_Angle[1]/180.0f*PI);
        Motor_Joint[0].Set_Target_Omega(Set_Leg_Velocity[1]);
        Motor_Joint[1].Set_Target_Angle(-Set_Leg_Angle[1]/180.0f*PI);
        Motor_Joint[1].Set_Target_Omega(-Set_Leg_Velocity[1]);
        break;
    }
    }
    #endif

    for(int i = 0; i < 2; i++)
    {
        Motor_Joint[i].TIM_Process_PeriodElapsedCallback();
    }
}

/**
 * @brief 履带状态控制函数
 * 
 */
void Class_HybridTrackLeg_Chassis::Track_Controller()
{
    switch (Track_Control_Type)
    {
    case (Track_Off):
    {
        //关闭履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Track[0].Set_Target_Torque(0.0f);
        Motor_Track[1].Set_Target_Torque(0.0f);
    }
    break;
    case (Track_On):
    {
        //开启履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[0].Set_Target_Omega_Radian(-Target_Track_Omega);
        Motor_Track[1].Set_Target_Omega_Radian(Target_Track_Omega);
    }
    break;
    }

    for (int i = 0; i < 2; i++)
        {
            Motor_Track[i].TIM_PID_PeriodElapsedCallback();
        }
}

/**
 * @brief 导轮状态控制函数
 * 
 */
void Class_HybridTrackLeg_Chassis::Guider_Controller()
{
    switch (Track_Control_Type)
    {
    case (Track_Off):
    {
        //关闭底部导轮电机
        Motor_Guider[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Guider[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Guider[0].Set_Target_Torque(0.0f);
        Motor_Guider[1].Set_Target_Torque(0.0f);
    }
    break;
    case (Track_On):
    {
        //开启底部导轮电机
        Motor_Guider[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Guider[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Guider[0].Set_Target_Omega_Radian(Target_Guider_Omega);
        Motor_Guider[1].Set_Target_Omega_Radian(-Target_Guider_Omega);
    }
    break;
    }

    for (int i = 0; i < 2; i++)
        {
            Motor_Guider[i].TIM_PID_PeriodElapsedCallback();
        }
}

#ifdef TRACK_LEG
/**
 * @brief 姿态切换函数
 * 
 */
void Class_HybridTrackLeg_Chassis::Switch_Pose()
{
    Jointleg_Controller(); //关节

    //计算回调函数
    static uint8_t mod5 = 0;
    mod5++;
    if (mod5 >= 5)
    {
        mod5 = 0;
            Track_Controller(); //履带
            Guider_Controller(); //导轮

    }

}
#endif

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
#ifdef TRACK_LEG
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_HybridTrackLeg_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    //debug
    //Chassis_Control_Type = Enum_Chassis_Control_Type(Chassis_Flag);
    #ifdef SPEED_SLOPE
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    #endif
    //过温保护
    FSM_OverHeated_Detect.Reload_TIM_Status_PeriodElapsedCallback();

    //位姿切换
    Switch_Pose();

    #ifdef Speed_Controller
    //速度解算
    Speed_Resolution();
    /*************************功率限制策略*******************************/
    //1.获取电机信息
    uint8_t chassis_status = (uint8_t)(Get_Chassis_Control_Type());
    // 清空不存在的转向电机
    for (uint8_t i = 0; i < 8; i += 2) 
    {
    Power_Management.Motor_Data[i].feedback_omega = 0;
    Power_Management.Motor_Data[i].feedback_torque = 0;
    Power_Management.Motor_Data[i].pid_output = 0;
    Power_Management.Motor_Data[i].torque = 0;
    }
    Power_Management.Motor_Data[1].feedback_omega = Motor_Wheel[0].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[3].feedback_omega = Motor_Wheel[1].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[5].feedback_omega = Motor_Wheel[2].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[7].feedback_omega = Motor_Wheel[3].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[1].feedback_torque = Motor_Wheel[0].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[3].feedback_torque = Motor_Wheel[1].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[5].feedback_torque = Motor_Wheel[2].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[7].feedback_torque = Motor_Wheel[3].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[1].pid_output = Motor_Wheel[0].Get_Out();
    Power_Management.Motor_Data[3].pid_output = Motor_Wheel[1].Get_Out();
    Power_Management.Motor_Data[5].pid_output = Motor_Wheel[2].Get_Out();
    Power_Management.Motor_Data[7].pid_output = Motor_Wheel[3].Get_Out();
    Power_Management.Motor_Data[1].torque = Motor_Wheel[0].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[3].torque = Motor_Wheel[1].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[5].torque = Motor_Wheel[2].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[7].torque = Motor_Wheel[3].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    //2.跑功率控制策略
    Power_Limit.Set_Control_Status(chassis_status);
    Power_Management.Max_Power = 100.0f;

    Power_Limit.Power_Task(Power_Management);
    //3.获取输出
    Motor_Wheel[0].Set_Out(Power_Management.Motor_Data[1].output);
    Motor_Wheel[1].Set_Out(Power_Management.Motor_Data[3].output);
    Motor_Wheel[2].Set_Out(Power_Management.Motor_Data[5].output);
    Motor_Wheel[3].Set_Out(Power_Management.Motor_Data[7].output);

    #endif
}
#endif



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
