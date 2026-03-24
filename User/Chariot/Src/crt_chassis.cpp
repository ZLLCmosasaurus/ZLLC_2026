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
    Power_Limit.Init(0,0);
    Supercap.Init(&hfdcan2,100.f);
    
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

        //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f,0.01f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f,0.01f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    #ifdef AGV
    //舵向电机PID初始化

    Motor_Steer[0].PID_Angle.Init(12.f, 0.0f, 0.0f, 0.0f, Motor_Steer[0].Get_Output_Max(), Motor_Steer[0].Get_Output_Max());
    Motor_Steer[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());
    
    Motor_Steer[1].PID_Angle.Init(12.f, 0.0f, 0.0f, 0.0f, Motor_Steer[1].Get_Output_Max(), Motor_Steer[1].Get_Output_Max());
    Motor_Steer[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[1].Get_Output_Max());

    Motor_Steer[2].PID_Angle.Init(12.f, 0.0f, 0.0f, 0.0f, Motor_Steer[2].Get_Output_Max(), Motor_Steer[2].Get_Output_Max());
    Motor_Steer[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[2].Get_Output_Max());

    Motor_Steer[3].PID_Angle.Init(12.f, 0.0f, 0.0f, 0.0f, Motor_Steer[3].Get_Output_Max(), Motor_Steer[3].Get_Output_Max());
    Motor_Steer[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[3].Get_Output_Max());


    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan1, DJI_Motor_ID_0x205);
    Motor_Steer[1].Init(&hfdcan1, DJI_Motor_ID_0x206);
    Motor_Steer[2].Init(&hfdcan1, DJI_Motor_ID_0x207);
    Motor_Steer[3].Init(&hfdcan1, DJI_Motor_ID_0x208);

    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(0.36000001f);               //应该是轮子朝向的正方向，行进轮超前，并且顺时针转动为正方向的角度
    Motor_Steer[1].Set_Zero_Position(0.47999999f);
    Motor_Steer[2].Set_Zero_Position(1.63);
    Motor_Steer[3].Set_Zero_Position(-2.51999998f);
    #endif

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}


/**
 * @brief 速度解算
 *
 */
float car_V,car_yaw;//车体总体朝向与速度
void Class_Tricycle_Chassis::Speed_Resolution(){ 
    #ifdef AGV 
   switch (Chassis_Control_Type)
    {
        case(Chassis_Control_Type_DISABLE):
        {
            for(int i = 0; i < 4;i++){
                Motor_Wheel[i].Disable();
                Motor_Steer[i].Disable();
            }
            break;
        }
        case(Chassis_Control_Type_FLLOW):
        case(Chassis_Control_Type_SPIN):
        {
            //轮组自锁，每个小轮坐标系都符合右手系
            static uint32_t Lock_Time = 0;
            static uint8_t  Lock_Flag = 0;
            float delta_Angle = 0.0f, Transform_Radian = 0.0f;                  //用于优化处理的变量
            if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
            {
                Lock_Time++;
                if(Lock_Time > 100)  Lock_Flag = 1;
                if (Lock_Flag)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

                        Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                    }

                    Motor_Steer[0].Set_Target_Radian( PI / 4.0f);
                    Motor_Steer[1].Set_Target_Radian(-PI / 4.0f);
                    Motor_Steer[2].Set_Target_Radian( PI / 4.0f);
                    Motor_Steer[3].Set_Target_Radian(-PI / 4.0f);
                    for (int i = 0; i < 4; i++)
                    {
                        Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                        //优劣弧处理
                        if((i % 2) == 0){
                            delta_Angle =  PI / 4.0f - Transform_Radian;
                        }
                        else{
                            delta_Angle = -PI / 4.0f - Transform_Radian;
                        }
                        delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);

                        if (delta_Angle > PI / 2.0f)                        //做的是180度以内的最优角度选择，而不是以前的优劣弧处理，优劣弧不一定路径最短
                        {
                            delta_Angle = delta_Angle - PI;

                        }
                        else if (delta_Angle < -PI / 2.0f)
                        {
                            delta_Angle = delta_Angle + PI;

                        }
                        else
                        {
                            // 不需要处理角度
                        }

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

            if(Lock_Flag){
                Lock_Flag = 0;
                Lock_Time = 0;
            }

            //轮子 0 1 2 3  转向 4 5 6 7 左前 右前 右后 左后 逆时针
            //0 1 2 3 左前 右前 右后 左后 逆时针    前X左Y坐标系   基于编码器0度朝前，逆时针为正角度   确保轮子正转的是朝前的速度，不然得单独加负号
            float True_Vx[4], True_Vy[4], True_Target_Angle_Radian[4];
            
            //斜坡处理
            // True_Vx[0] = True_Vx[3] = Slope_Velocity_X.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            // True_Vx[1] = True_Vx[2] = Slope_Velocity_X.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;

            // True_Vy[0] = True_Vy[1] = Slope_Velocity_Y.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            // True_Vy[2] = True_Vy[3] = Slope_Velocity_Y.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            

            True_Vx[0] = True_Vx[3] = Target_Velocity_X + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
            True_Vx[1] = True_Vx[2] = Target_Velocity_X - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

            True_Vy[0] = True_Vy[1] = Target_Velocity_Y + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
            True_Vy[2] = True_Vy[3] = Target_Velocity_Y - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

            //舵轮转动角度的优化处理
            for(int i = 0;i<4;i++){
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);         //舵轮控制模式

                //计算速度
                float temp_Target_Omega = 0.0f;
                arm_sqrt_f32(True_Vx[i] * True_Vx[i] + True_Vy[i] * True_Vy[i], &temp_Target_Omega);
                temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;

                //计算目标角度
                if(fabs(temp_Target_Omega) < 0.0001){            //避免X =0 ；Y = 0的情况
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                }
                else{
                    True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]);           //-PI -- PI   会自动处理Vx = 0;
                }
                
                //角度优化处理         180度内最短路径选择   不是选优劣弧   已经顺便处理了跳变点
                delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();     // -2PI -- 2PI  
                delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                 // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
                if(delta_Angle > PI/2.0f){
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle - PI;
                    temp_Target_Omega *= -1.0f;
                }
                else if(delta_Angle < -PI/2.0f){
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle + PI;
                    temp_Target_Omega *= -1.0f;
                }
                else{
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle;
                    //不需要处理角度
                }
                
                Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
                Motor_Wheel[i].Set_Target_Omega_Radian(temp_Target_Omega);
            }

            for(int i=0;i<4;i++)
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
            //底盘失能 轮组无力
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                Motor_Wheel[i].Set_Out(0.0f);
            }            
        }
        break;
        case (Chassis_Control_Type_FLLOW) :
        {
            // 底盘四电机模式配置
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

            for(int i=0;i<4;i++)
            {
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            }
        }
        break;
        case (Chassis_Control_Type_SPIN) :
        {
            // 底盘四电机模式配置
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

            for(int i=0;i<4;i++)
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
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback()
{
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    
    //速度解算
    Speed_Resolution();

    #if POWER_CONTROL == 1
    /*************************功率限制策略*******************************/
    #ifdef POWER_LIMIT_1

    Power_Limit.Set_Motor(Motor_Wheel);                         //添加四个电机的控制电流和当前转速
    Power_Limit.Set_Power_Limit(Limit_Power);
    Power_Limit.Set_Chassis_Buffer(Chassis_Energy_Power);
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);

    #elif defined (POWER_LIMIT_2)
    static uint8_t supercap_flag = 0;                   //超电能量低于50J的标志位

    // 计算限制功率
    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE)
    {
        // 缓冲环限制功率
        Power_Management.Buffer_Power = 0.0f;    //Referee->Get_Chassis_Energy_Buffer() - 30.0f; 
        // Power_Management.Buffer_Power = (Referee->Get_Chassis_Energy_Buffer() - 30.0f) * 1.5f;
        Math_Constrain(&Power_Management.Buffer_Power, -50.0f, 30.0f);

        if (Supercap.Get_Supercap_Status() != Supercap_Status_DISABLE)              //&& Sprint_Status == Sprint_Status_ENABLE
        {
            if(supercap_flag == 1){                     //超电低电量了
                Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
                if(Supercap.Get_Supercap_Proportion() > (uint8_t)55){
                    supercap_flag = 0;                  //充满电可以使用
                }
            }
            else{
                if (Supercap.Get_Supercap_Proportion() > (uint8_t)25)
                {
                    Power_Management.Max_Power = Supercap.Get_Buffer_Power() + Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
                }
                else
                {
                    supercap_flag = 1;
                    Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();
                }
            }
        }
        else
        {
            // Power_Management.Max_Power = Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            supercap_flag = 0;
            Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();              //不吃缓冲能量
        }
    }
    else
    {
        // 裁判系统离线限制功率
        supercap_flag = 0;
        Power_Management.Max_Power = 100.0f;
        Power_Management.Buffer_Power = 0.0f;
    }

    Power_Management.Actual_Power = Supercap.Get_Chassis_Power();
    Power_Management.Total_error = 0.0f;

    #ifdef AGV

    for (int i = 0; i < 4; i++) // 数据传递处理
    {
        // 都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE; // 与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                 // 与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

        Power_Management.Motor_Data[i + 4].feedback_omega = Motor_Steer[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Steer[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i + 4].feedback_torque = Motor_Steer[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].torque = Motor_Steer[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].pid_output = Motor_Steer[i].Get_Out();
    }

    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
        Motor_Wheel[i].Output();

        Motor_Steer[i].Set_Out(Power_Management.Motor_Data[i + 4].output);
        Motor_Steer[i].Output();
    }
    #else
    for (int i = 0; i < 4; i++) // 数据传递处理
    {
        // 都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE; // 与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                 // 与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

        Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
    }
    Power_Management.Total_error = 0.0;
    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
        Motor_Wheel[i].Output();
    }
    #endif

    Supercap.Set_Supercap_Mode(Supercap_ENABLE);
    Supercap.Set_Limit_Power(Power_Management.Max_Power);               //这样子是优先使用的缓冲功率
    Supercap.Set_Referee_Limit_Power((uint8_t)Referee->Get_Chassis_Power_Max());
    Supercap.Set_Referee_Buffer_Power(Referee->Get_Chassis_Energy_Buffer());
    Supercap.TIM_Supercap_PeriodElapsedCallback();

    #endif

    #endif
}

void Class_Tricycle_Chassis::Power_Limit_Update()
{

}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
