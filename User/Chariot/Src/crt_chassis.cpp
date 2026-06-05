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
float dt_calc;
uint32_t cnt_num;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 关节电机过热检测状态机
 *
 */
void Class_FSM_OverHeated_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    Heat = Chassis->Get_Now_Joint_Heat();
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态

        if (Heat > 90.0f) // 如果电机温度达到90℃，100℃电机过温
        {
            Set_Status(1);
        }
    }
    break;
    case (1):
    {
        Chassis->Set_Pose_Control_Type(Pose_DISABLE);
        if (Heat < 50.0f && Status[1].Time > 3000) // 至少冷却3s
        {
            Set_Status(0);
        }
    }
    break;
    }
}

/**
 * @brief 失能关节电机
 *
 */
void Class_DM_Motor_8009P::Disable()
{
    Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
    Set_Target_Angle(0.0f);
    Set_Target_Omega(0.0f);
    Set_MIT_K_P(0.0f);
    Set_MIT_K_D(0.0f);
    Set_Target_Torque(0.0f);
}

/**
 * @brief 关节电机PID控制周期回调函数,
 *
 * @param None
 */
void Class_DM_Motor_8009P::PID_Calculate(float __Target_Value, float __Now_Value)
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_POSITION):
    {
        PID_Posture.Set_Target(__Target_Value);
        PID_Posture.Set_Now(__Now_Value);
        PID_Posture.TIM_Adjust_PeriodElapsedCallback();
        Error_Compensation = PID_Posture.Get_Out();
    }
    break;
    default:
    {
        Error_Compensation = 0.0f;
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
    // 超级电容初始化

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f, 0.01f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f, 0.01f);
    //  斜坡函数加减速角度
    Slope_Position.Init(0.0014f, 0.0028f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    // imu初始化
    // BoardDM_BMI.Init();
    // 过热检测状态机初始化
    FSM_OverHeated_Detect.Chassis = this;
    FSM_OverHeated_Detect.Init(2, 0);

    // 轮向电机PID初始化
    Motor_Wheel[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());
    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    // 关节电机PID初始化
    // DM速度位置控制模式Kp、Ki参数需要用上位机调节
    Motor_Leg[0].PID_Posture.Init(1.0f, 0.0f, 0.0f, 0.0f);
    Motor_Leg[1].PID_Posture.Init(1.0f, 0.0f, 0.0f, 0.0f);
    // 关节电机ID初始化
    Motor_Joint[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA, 0, PI, 20.94359f, 20.0f);
    Motor_Joint[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, PI, 20.94359f, 20.0f);
    Motor_Leg[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION, 0, PI, 20.94359f, 20.0f);
    Motor_Leg[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_MIT_POSITION, 0, PI, 20.94359f, 20.0f);

    // 履带驱动电机PID初始化
    Motor_Track[0].PID_Omega.Init(1500.0f, 50.0f, 0.0f, 0.0f, Motor_Track[0].Get_Output_Max(), Motor_Track[0].Get_Output_Max());
    Motor_Track[1].PID_Omega.Init(1250.0f, 50.0f, 0.0f, 0.0f, Motor_Track[1].Get_Output_Max(), Motor_Track[1].Get_Output_Max()); // 需调参
    // 履带电机ID初始化
    Motor_Track[0].Init(&hfdcan2, DJI_Motor_ID_0x201);
    Motor_Track[1].Init(&hfdcan2, DJI_Motor_ID_0x202);

    // 底部导轮电机PID初始化
    Motor_Guider[0].PID_Omega.Init(650.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[0].Get_Output_Max(), Motor_Guider[0].Get_Output_Max());
    Motor_Guider[1].PID_Omega.Init(700.0f, 10.0f, 0.0f, 0.0f, Motor_Guider[1].Get_Output_Max(), Motor_Guider[1].Get_Output_Max()); // 需调参
    // 底部导轮电机ID初始化
    Motor_Guider[0].Init(&hfdcan2, DJI_Motor_ID_0x203);
    Motor_Guider[1].Init(&hfdcan2, DJI_Motor_ID_0x204);

    // 底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}
#endif

/**
 * @brief 速度解算
 *
 */
float car_V, car_yaw; // 车体总体朝向与速度
#ifdef TRACK_LEG
/**
 * @brief 速度解算
 *
 */
void Class_HybridTrackLeg_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
        break;
    }
    case (Chassis_Control_Type_SPIN_Positive):
    case (Chassis_Control_Type_SPIN_Negative):
    case (Chassis_Control_Type_FLLOW):
    {
        // 电机模式配置
        // 轮向电机
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
            Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
        }
#ifdef SPEED_SLOPE
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
#else
        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega * (HALF_WIDTH + HALF_LENGTH);
#endif
        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);

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
 * @brief 角度转换为相对角度,根据DM板放置的位置改变,单位为弧度
 *
 */
void Class_HybridTrackLeg_Chassis::Transform_Angle_To_Relative()
{
    // Chassis_Pitch = BoardDM_BMI.Get_Rad_Roll();
}

/**
 * @brief 更新增量式位置累加器，限制单次步长及总行程
 * @param pitch  当前车体俯仰角 (rad)
 */
void Class_HybridTrackLeg_Chassis::Update_LegPositionAccumulator(float pitch)
{
    float d_pitch = pitch - last_pitch_;
    last_pitch_ = pitch;

    float incr = K_INCR * d_pitch;

    // 限制单周期位置增量（位置变化速率）
    if (incr > MAX_ANGLE_INCR)
        incr = MAX_ANGLE_INCR;
    if (incr < -MAX_ANGLE_INCR)
        incr = -MAX_ANGLE_INCR;

    accumulated_angle_ += incr;

    // 总行程限幅
    if (accumulated_angle_ > MAX_ANGLE)
        accumulated_angle_ = MAX_ANGLE;
    if (accumulated_angle_ < -MAX_ANGLE)
        accumulated_angle_ = -MAX_ANGLE;
}

/**
 * @brief 对姿态PID输出的原始速度做斜坡与限幅处理
 * @param raw_velocity  姿态PID输出（期望速度，rad/s）
 * @return 规划后的速度指令 (rad/s)
 */
float Class_HybridTrackLeg_Chassis::Plan_LegVelocity(float raw_velocity)
{
    float vel_error = raw_velocity - last_velocity_target_;
    float max_step = VELOCITY_SLEW * CONTROL_PERIOD;

    if (vel_error > max_step)
        vel_error = max_step;
    if (vel_error < -max_step)
        vel_error = -max_step;

    float velocity_target = last_velocity_target_ + vel_error;
    last_velocity_target_ = velocity_target;

    // 速度幅值限幅
    if (velocity_target > MAX_VELOCITY)
        velocity_target = MAX_VELOCITY;
    if (velocity_target < -MAX_VELOCITY)
        velocity_target = -MAX_VELOCITY;

    return velocity_target;
}

/**
 * @brief 计算带斜坡的前馈力矩
 * @param velocity_target  规划后的目标速度 (rad/s)
 * @param error_pitch      当前角度误差 (rad)
 */
void Class_HybridTrackLeg_Chassis::Calc_LegFeedforwardTorque(float velocity_target, float error_pitch)
{
    // 根据工况设定目标力矩
    bool need_high_torque = (fabsf(velocity_target) > TFF_THRESHOLD_V) &&
                            (fabsf(error_pitch) > 0.02f);

    if (need_high_torque)
    {
        target_tff_ = TFF_AGGRESSIVE_LEVEL; // 需要大力矩时设定目标值
        // 方向与速度一致
        if (velocity_target < 0)
            target_tff_ = -target_tff_;
    }
    else
    {
        // 维持期可以保持一个小力矩或者归零
        target_tff_ = TFF_HOLD_LEVEL;
        if (velocity_target < 0)
            target_tff_ = -target_tff_;
    }

    // 斜坡升降：让实际力矩以固定速率追踪目标
    float max_step = TFF_RAMP_RATE * CONTROL_PERIOD; // 单周期最大变化量
    if (target_tff_ > tff_hold_)
    {
        tff_hold_ += max_step;
        if (tff_hold_ > target_tff_)
            tff_hold_ = target_tff_;
    }
    else if (target_tff_ < tff_hold_)
    {
        tff_hold_ -= max_step;
        if (tff_hold_ < target_tff_)
            tff_hold_ = target_tff_;
    }
}

/**
 * @brief 向左右腿电机发送MIT指令
 * @param velocity_target  规划后的速度指令
 * @param kp_stance        MIT刚度系数
 * @param kd_stance        MIT阻尼系数
 */
void Class_HybridTrackLeg_Chassis::Send_LegMITCommands(float velocity_target, float kp_stance, float kd_stance)
{
    // 左腿
    Motor_Leg[0].Set_Target_Constants(
        accumulated_angle_, // 目标角度
        velocity_target,    // 目标速度
        tff_hold_,          // 前馈力矩
        kp_stance,
        kd_stance);

    // 右腿（取反，按实际机械对称性调整）
    Motor_Leg[1].Set_Target_Constants(
        -accumulated_angle_,
        -velocity_target,
        -tff_hold_,
        kp_stance,
        kd_stance);
}

/**
 * @brief 关节电机控制状态函数
 *
 */
void Class_HybridTrackLeg_Chassis::Jointleg_Controller()
{
#ifdef AUTO_SWITCH
    Motor_Leg[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Motor_Leg[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    switch (Pose_Control_Type)
    {
    case Pose_DISABLE: // 失能关节，通常防止过热
    {
        for (int i = 0; i < 2; i++)
        {
            // 关节电机失能
            Motor_Leg[i].Disable();
            // PID积分清零
            Motor_Leg[i].PID_Posture.Set_Integral_Error(0.0f);
        }
    }
    break;
    case Pose_ENABLE: // 使能，上台阶使用，自动根据IMU反馈调整腿部姿态
    {
        Referance_Angle = 0.0f;
        Error_Pitch = Referance_Angle - Chassis_Pitch;

        // 更新位置累加器（内部处理增量、限幅）
        Update_LegPositionAccumulator(Chassis_Pitch);

        //
        for (int i = 0; i < 2; i++)
        {
            Motor_Leg[i].Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
            Motor_Leg[i].PID_Calculate(Referance_Angle, Chassis_Pitch);
        }
        float raw_velocity = Motor_Leg[0].Error_Compensation;

        // 速度规划（斜坡+限幅）
        float velocity_cmd = Plan_LegVelocity(raw_velocity);

        // 计算前馈力矩
        Calc_LegFeedforwardTorque(velocity_cmd, Error_Pitch);

        // 发送MIT指令
        Send_LegMITCommands(velocity_cmd, 0.0F, 0.0F);
    }
    break;
    case Pose_STANDBY: // 待机，下台阶前使用，保持腿部姿态为小角度定值
    {
        // 设置关节电机控制方式为角度MIT模式
        Motor_Leg[0].Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
        Motor_Leg[1].Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
        Referance_Angle = -PI * (20.0f / 180.0f);
        // Motor_Leg[0].Set_Target_Angle(Motor_Leg[0].Target_Angle_Calc);
        // Motor_Leg[1].Set_Target_Angle(Motor_Leg[1].Target_Angle_Calc);
        Motor_Leg[1].Set_Target_Angle(0.0f);
    }
    break;
    case Pose_CONTRACT: // 缩起，上台阶后使用，腿部收缩
    {
        // 设置关节电机控制方式为角度MIT模式
        Motor_Leg[0].Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
        Motor_Leg[1].Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
        Motor_Leg[0].Set_Target_Angle(Set_Leg_Angle[0] / 180.0f * PI);
        Motor_Leg[1].Set_Target_Angle(-Set_Leg_Angle[0] / 180.0f * PI);
    }
    break;
    }

#endif
#ifdef LOCKED_SWITCH               // 自动伸缩腿，已注释，不过目前是开环，后期可做成闭环
    static uint16_t mod2s = 0;     // 2s重置计数器
    static uint8_t pose_state = 1; // 位姿控制状态 0-Enable 1-Standby
    // Chassis_Pitch = BoardDM_BMI.Get_Angle_Pitch();
    Error_Pitch = Chassis_Pitch;
    Joint_Heat = Motor_Joint[1].Get_Now_Rotor_Temperature();

    Slope_Position.TIM_Calculate_PeriodElapsedCallback();

    switch (Pose_Control_Type)
    {
    case Pose_DISABLE: // 失能
    {
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Joint[0].Set_Target_Angle(0);
        Motor_Joint[0].Set_Target_Omega(0);
        Motor_Joint[1].Set_Target_Angle(0);
        Motor_Joint[1].Set_Target_Omega(0);

        Slope_Position.Set_Target(Set_Leg_Angle[0]);

        break;
    }
    case Pose_STANDBY: // 待机
    {
        // 启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        // 设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        // 位置速度模式
        Slope_Position.Set_Target(Set_Leg_Angle[0]);

        Motor_Joint[0].Set_Target_Angle(Slope_Position.Get_Out());
        Motor_Joint[0].Set_Target_Omega(Set_Leg_Velocity[0]);
        Motor_Joint[1].Set_Target_Angle(-Slope_Position.Get_Out());
        Motor_Joint[1].Set_Target_Omega(-Set_Leg_Velocity[0]);
        break;
    }
    case Pose_ENABLE: // 使能
    {
        // 启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        // 设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        // 位置速度模式
        Slope_Position.Set_Target(Set_Leg_Angle[1]);

        Motor_Joint[0].Set_Target_Angle(Slope_Position.Get_Out());
        Motor_Joint[0].Set_Target_Omega(Set_Leg_Velocity[1]);
        Motor_Joint[1].Set_Target_Angle(-Slope_Position.Get_Out());
        Motor_Joint[1].Set_Target_Omega(-Set_Leg_Velocity[1]);
        break;
    }
    case Pose_CONTRACT:
    {
        // 启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        // 设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        // 位置速度模式
        Slope_Position.Set_Target(Set_Leg_Angle[2]);

        Motor_Joint[0].Set_Target_Angle(Slope_Position.Get_Out());
        Motor_Joint[0].Set_Target_Omega(Set_Leg_Velocity[1]);
        Motor_Joint[1].Set_Target_Angle(-Slope_Position.Get_Out());
        Motor_Joint[1].Set_Target_Omega(-Set_Leg_Velocity[1]);
        break;
    }
    }
#endif
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
        // 关闭履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Track[0].Set_Target_Torque(0.0f);
        Motor_Track[1].Set_Target_Torque(0.0f);
    }
    break;
    case (Track_On):
    {
        // 开启履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[0].Set_Target_Omega_Radian(-Target_Track_Omega);
        Motor_Track[1].Set_Target_Omega_Radian(Target_Track_Omega);
    }
    break;
    }

    for (int i = 0; i < 2; i++)
    {
        // Motor_Track[i].TIM_PID_PeriodElapsedCallback();
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
        // 关闭底部导轮电机
        Motor_Guider[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Guider[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Guider[0].Set_Target_Torque(0.0f);
        Motor_Guider[1].Set_Target_Torque(0.0f);
    }
    break;
    case (Track_On):
    {
        // 开启底部导轮电机
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
    Transform_Angle_To_Relative(); // 将IMU获取的车体倾角转换为相对角度，作为关节电机控制的输入

    // 计算回调函数

    static uint8_t mod5 = 0;
    mod5++;
    if (Pose_Control_Type != Pose_DISABLE)
    {
        if (mod5 == 1)
            Motor_Joint[0].TIM_Process_PeriodElapsedCallback();
        else if (mod5 == 2)
            Motor_Joint[1].TIM_Process_PeriodElapsedCallback();
    }
    if (mod5 == 5)
    {
        mod5 = 0;
        Jointleg_Controller(); // 腿
    }
}
#endif

#ifdef TRACK_LEG
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_HybridTrackLeg_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
#ifdef SPEED_SLOPE
    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

#endif
    // 过温保护
    // FSM_OverHeated_Detect.Reload_TIM_Status_PeriodElapsedCallback();

    // 位姿切换
    Switch_Pose();

#ifdef POWER_CONTROL
    Power_Management.Max_Power = Supercap.Get_Chassis_Device_LimitPower();

    Power_Limit.Power_Task(Power_Management);

    for (int i = 0, j = 0; i < 4; i += 2, ++j)
    {
        Power_Management.Motor_Data[i].feedback_omega = Motor_Track[j].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Track[j].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i].pid_output = Motor_Track[j].Get_Out();
        Power_Management.Motor_Data[i].torque = Motor_Track[j].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        Motor_Track[j].Reset_Out_And_Output(Power_Management.Motor_Data[i].output);
    }
#endif
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
