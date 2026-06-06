#include "crt_force_control_chassis.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.002f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.002s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {1.0f, 0.0f, 
                           0.0f, 1.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {2000.0f, 0.0f, 
                            0.0f,  2000.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
float vel_acc[2];

// #define Control_Type_Oemga
#define Control_Type_Current

//#define Omni_Wheel
#define Mecanum
/* Private types -------------------------------------------------------------*/

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
	 memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    // Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

// 根据位姿控制类型和电机索引，判断该电机是否应参与运动（并占用功率预算）
static inline bool IsMotorActiveForPose(Enum_Pose_Control_Type__ pose_type, int motor_index)
{
    // 偶数索引（0,2）：履带电机，始终参与运动
    if (motor_index % 2 == 0) return true;

    // 奇数索引：轮毂电机，根据位姿类型决定
    int wheel_idx = (motor_index - 1) / 2;  // 映射：0-左上,1-左下,2-右下,3-右上
    switch (pose_type) {
        case Pose_ENABLE__:   // 只使用履带 + 后面两个轮子（左下、右下）
            return (wheel_idx == 1 || wheel_idx == 2);
        case Pose_STANDBY__:  // 只使用履带 + 前面两个轮子（左上、右上）
            return (wheel_idx == 0 || wheel_idx == 3);
        default:              // Pose_DISABLE__ 或其他情况：所有电机都参与
            return true;
    }
}

void Class_Chassis::Init()
{

    // PID初始化

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(150.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(75.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.002f);

    // 轮向电机ID初始化
    #ifdef Control_Type_Current
    // 电机初始化
    Motor_Wheel[0].Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_DISABLE);
    Motor_Wheel[0].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[1].Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_DISABLE);
    Motor_Wheel[1].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[2].Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_DISABLE);
    Motor_Wheel[2].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[3].Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_CURRENT, 3591.f / 187.f, Motor_DJI_Power_Limit_Status_DISABLE);
    Motor_Wheel[3].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    #endif
    #ifdef Control_Type_Oemga
    // 电机初始化 :默认轮组 左上0 左下1 右下2 右上3
    Motor_Wheel[0].Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[0].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    Motor_Wheel[1].Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[1].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    Motor_Wheel[2].Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[2].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    Motor_Wheel[3].Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);
    Motor_Wheel[3].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    #endif
    // 履带驱动电机初始化
    Motor_Track[0].Init(&hfdcan2, Motor_DJI_ID_0x201);
    Motor_Track[0].PID_Omega.Init(1.5f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 履带驱动电机初始化
    Motor_Track[1].Init(&hfdcan2, Motor_DJI_ID_0x202);
    Motor_Track[1].PID_Omega.Init(1.5f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f); 
   
    
    // 超级电容初始化
    Supercap.Init(&hfdcan2, 100);

    //imu初始化
    // Boardc_BMI.Init();
    //低通滤波初始化
    // //加速度计
    // Acceleration_X_Filter.Init(-29.42f,+29.42f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,30);
    // Acceleration_Y_Filter.Init(-29.42f,+29.42f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,30);
    // Gyro_Z_Filter.Init(-20.0f,20.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,30);
    // //实际速度滤波
    // PID_Velocity_Filter[0].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    // PID_Velocity_Filter[1].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    // PID_Omega_Filter.Init(-20.0f,20.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,15);

    // xvEstimateKF_Init(&vaEstimateKF);
    
   // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.064f, 0.064f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.064f, 0.064f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.1f, 0.1f);

    kalman_Init(&kalman_Now_VelocityX,0.9,0.005,0,1);
    kalman_Init(&kalman_Now_VelocityY,0.9,0.005,0,1);
    kalman_Init(&kalman_Now_VelocityZ,0.9,0.005,0,1);

}



/*
1.运动学逆解算
2.
*/
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
       // Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
    for (int j = 0; j < 2; j++)
    {
        Motor_Track[j].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}
/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{

    Self_Resolution();
    
    // Acceleration_X_Filter.Set_Now(Boardc_BMI.Get_Accel_X());
    // Acceleration_X_Filter.TIM_Adjust_PeriodElapsedCallback();     
    // Acceleration_Y_Filter.Set_Now(Boardc_BMI.Get_Accel_Y());
    // Acceleration_Y_Filter.TIM_Adjust_PeriodElapsedCallback();
    // Gyro_Z_Filter.Set_Now(Boardc_BMI.Get_Gyro_Yaw());
    // Gyro_Z_Filter.TIM_Adjust_PeriodElapsedCallback();
    // PID_Velocity_Filter[0].Set_Now(Now_Velocity_X);
    // PID_Velocity_Filter[0].TIM_Adjust_PeriodElapsedCallback();
    // PID_Velocity_Filter[1].Set_Now(Now_Velocity_Y);
    // PID_Velocity_Filter[1].TIM_Adjust_PeriodElapsedCallback();
    // PID_Omega_Filter.Set_Now(Now_Omega);
    // PID_Omega_Filter.TIM_Adjust_PeriodElapsedCallback();


    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    //卡尔曼滤波器更新车体x方向速度
    //xvEstimateKF_Update(&vaEstimateKF,Acceleration_X_Filter.Get_Out(),Now_Velocity_X);
    
    //INS_Data.Mix_Velocity_X = vel_acc[0];


}

void Class_Chassis::TIM_1ms_Kalmancale_PeriodElapsedCallback()
{
    kalman_set_now(&kalman_Now_VelocityX,Now_Velocity_X);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityX);
    kalman_set_now(&kalman_Now_VelocityY,Now_Velocity_Y);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityY);
    kalman_set_now(&kalman_Now_VelocityZ,Now_Omega);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityZ);
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
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    // for (int i = 0; i < 4; i++)
    // {
    //     float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

    //     // 解算到每个轮组的具体线速度
    //     tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * arm_sin_f32(Wheel_Azimuth[i]);
    //     tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * arm_cos_f32(Wheel_Azimuth[i]);
    //     arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

    //     // 根据线速度决定轮向电机角速度
    //     Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;
    // }
    #ifdef Onim_Wheel
    float motor0_temp_linear_vel = -1.0f * (arm_cos_f32(PI / 4.0f) * Target_Velocity_Y - arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor1_temp_linear_vel = -1.0f * (-arm_cos_f32(PI / 4.0f) * Target_Velocity_Y - arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor2_temp_linear_vel = -1.0f * (-arm_cos_f32(PI / 4.0f) * Target_Velocity_Y + arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor3_temp_linear_vel = -1.0f * (arm_cos_f32(PI / 4.0f) * Target_Velocity_Y + arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    #endif
    #ifdef Mecanum
    float motor0_temp_linear_vel = -1.0f *(Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH__ +HALF_LENGTH__));
    float motor1_temp_linear_vel = -1.0f *(-Target_Velocity_Y - Target_Velocity_X + Target_Omega * (HALF_WIDTH__ +HALF_LENGTH__));
    float motor2_temp_linear_vel = -1.0f *(-Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH__ +HALF_LENGTH__));
    float motor3_temp_linear_vel = -1.0f *(Target_Velocity_Y + Target_Velocity_X + Target_Omega * (HALF_WIDTH__ +HALF_LENGTH__));
    #endif
    Target_Wheel_Omega[0] = motor0_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[1] = motor1_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[2] = motor2_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[3] = motor3_temp_linear_vel / Wheel_Radius;
    
    // _Steer_Motor_Kinematics_Nearest_Transposition();
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
        PID_Velocity_X.Set_Now(kalman_Now_VelocityX.Out);
        PID_Velocity_X.TIM_Adjust_PeriodElapsedCallback();

        PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
        PID_Velocity_Y.Set_Now(kalman_Now_VelocityY.Out);
        PID_Velocity_Y.TIM_Adjust_PeriodElapsedCallback();

        PID_Omega.Set_Target(Slope_Omega.Get_Out());
        PID_Omega.Set_Now(kalman_Now_VelocityZ.Now);
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

    float sqrt2_4 = 0.0f;
    arm_sqrt_f32(1.0f / 8.0f, &sqrt2_4);
    // 每个轮的扭力
    // 解算到每个轮组的具体摩擦力
    float tmp_force[4];
    #ifdef Onim_Wheel
    tmp_force[0] = sqrt2_4 * (-force_x + force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[1] = sqrt2_4 * (-force_x - force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[2] = sqrt2_4 * (force_x - force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[3] = sqrt2_4 * (force_x + force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    #endif
    #ifdef Mecanum
    tmp_force[0] = -force_x + force_y + torque_omega / (4.0f * (HALF_WIDTH__ +HALF_LENGTH__));
    tmp_force[1] = -force_x - force_y + torque_omega / (4.0f * (HALF_WIDTH__ +HALF_LENGTH__));
    tmp_force[2] = force_x - force_y + torque_omega / (4.0f * (HALF_WIDTH__ +HALF_LENGTH__));
    tmp_force[3] = force_x + force_y + torque_omega / (4.0f * (HALF_WIDTH__ +HALF_LENGTH__));
    #endif
    for (int i = 0; i < 4; i++)
    {
        //#define force (20.0f/(0.154f/2.0f)*7.0f*0.01562f) 预测最大的摩擦力为28.4N左右
        // 摩擦力转换至扭矩 
        Target_Wheel_Current[i] = -1.0f * (tmp_force[i] * Wheel_Radius) + Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega()) ;//(tmp_force[i] * Wheel_Radius) / (13.933f * 0.5f) / M3508_Kt;//Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega());
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
    switch(Track_Control_Type)
    {
        case Track_Off__:
        {
            for (int j = 0; j < 2; j++)
            {
                Motor_Track[j].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
                Motor_Track[j].Set_Target_Current(0.0f);
            }
            break;
        }
        case Track_On__:
        {
            Motor_Track[0].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Track[1].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Track[0].Set_Target_Omega(-Target_Track_Omega);
            Motor_Track[1].Set_Target_Omega(Target_Track_Omega);
            break;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }

    for (int j = 0; j < 2; j++)
    {
        Motor_Track[j].TIM_Calculate_PeriodElapsedCallback();
    }

    Power_Management.Max_Power = Supercap.Get_Chassis_Device_LimitPower();
    //Power_Management.Max_Power = 120.0f;
    
    for (int i = 0; i < 8; i++)
    {
        bool active = IsMotorActiveForPose(Pose_Control_Type, i);
        if (!active)
        {
            // 非活跃电机：临时将 pid_output 和 torque 置零，让功率算法认为它们不消耗功率
            Power_Management.Motor_Data[i].pid_output = 0;
            Power_Management.Motor_Data[i].torque = 0;
        }
    }
    
    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 8; i++)
    {
        if (!IsMotorActiveForPose(Pose_Control_Type, i))
            Power_Management.Motor_Data[i].output = 0;
    }

    for (int i = 1, j = 0; i < 8; i+=2, ++j)
    {
        Motor_Wheel[j].Reset_Set_Out_And_Output(Power_Management.Motor_Data[i].output);
    }

    for(int i = 0, j = 0; i < 4; i+=2, ++j)
    {
        // Motor_Track[j].Reset_Set_Out_And_Output(Power_Management.Motor_Data[i].output);
    }
}

/**
 * @brief 自身解算
 *
 */
//#define filter_complementary
uint32_t imu_cnt_1 = 0,imu_cnt_2 = 0;
float alpha_v_adaptive = 0.9f,alpha_omega_adaptive = 0.0f;
void Class_Chassis::Self_Resolution()
{
    // 根据电机编码器与陀螺仪计算速度和角度

    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega = 0.0f;

    // 轮速计的计算方式

    float wheel_vel0 = Motor_Wheel[0].Get_Now_Omega() * Wheel_Radius;// 左上
    float wheel_vel1 = Motor_Wheel[1].Get_Now_Omega() * Wheel_Radius;// 左下
    float wheel_vel2 = Motor_Wheel[2].Get_Now_Omega() * Wheel_Radius;// 右下
    float wheel_vel3 = Motor_Wheel[3].Get_Now_Omega() * Wheel_Radius;// 右上

    // 从逆运动学方程推导正运动学：

    // 建立轮速向量
    float wheel_vel[4] = {wheel_vel0, wheel_vel1, wheel_vel2, wheel_vel3};
    float K = 0.0f;
    #ifdef Onim_Wheel
    arm_sqrt_f32(0.5f, &K);
    Now_Velocity_X = -1.0f * (-wheel_vel0 - wheel_vel1 + wheel_vel2 + wheel_vel3) / (4 * K);
    Now_Velocity_Y = -1.0f * (wheel_vel0 - wheel_vel1 - wheel_vel2 + wheel_vel3) / (4 * K);
    Now_Omega = -1.0f *(wheel_vel0 + wheel_vel1 + wheel_vel2 + wheel_vel3) / (4 * Wheel_To_Core_Distance);
    #endif
    // 因此正运动学公式为：
    #ifdef Mecanum
    Now_Velocity_X = -1.0f *(-wheel_vel0 - wheel_vel1 + wheel_vel2 + wheel_vel3) / 4;
    Now_Velocity_Y = -1.0f *(wheel_vel0 - wheel_vel1 - wheel_vel2 + wheel_vel3) / 4;
    Now_Omega = -1.0f *(wheel_vel0 + wheel_vel1 + wheel_vel2 + wheel_vel3) / (4 * (HALF_WIDTH__ + HALF_LENGTH__));
    #endif

    // 角度解算
    // float pitch = -AHRS_Chassis->Get_Angle_Pitch();
    // float roll = AHRS_Chassis->Get_Angle_Roll();
    // if (isnan(pitch) == true || isnan(roll) == true)
    // {
    //     Angle_Pitch = 0.0f;
    //     Angle_Roll = 0.0f;
    // }
    // else
    // {
    //     Angle_Pitch = pitch;
    //     Angle_Roll = roll;
    // }

    // Slope_Direction_X = arm_sin_f32(pitch) * arm_cos_f32(roll);
    // Slope_Direction_Y = -arm_sin_f32(roll);
    // Slope_Direction_Z = arm_cos_f32(pitch) * arm_cos_f32(roll);
    //转换电机数据到Power_Management的量纲
    for (int i = 1, j = 0; i < 8; i+=2, ++j)         //数据传递处理
    {
        //都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[j].Get_Now_Omega() /  RPM_TO_RADPS *  3591.f / 187.f;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[j].Get_Now_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[j].Get_Target_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[j].Get_Target_Current() * 16384.0f / 20.0f;

        //Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
        
    }
    for(int i = 0, j = 0; i < 4; i+=2, ++j)
    {
        Power_Management.Motor_Data[i].feedback_omega = Motor_Track[j].Get_Now_Omega() * M3508_REDUATION * RAD_TO_RPM;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Track[j].Get_Now_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i].pid_output = Motor_Track[j].Get_Target_Current() * 16384.0f / 20.0f;
        Power_Management.Motor_Data[i].torque = Motor_Track[j].Get_Target_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;
    }
    #ifdef  filter_complementary
    //互补滤波-计算车体速度
    //使用Imu的加速度做积分计算车体xy轴方向速度，同时在零速下清0
    INS_Data.IMU_X_Velocity += Acceleration_X_Filter.Get_Out() * DWT_GetDeltaT(&imu_cnt_1);
    INS_Data.IMU_Y_Velocity += Acceleration_Y_Filter.Get_Out() * DWT_GetDeltaT(&imu_cnt_2);
    //直接用IMU的角速度作为车体角速度
    INS_Data.IMU_Omega = Gyro_Z_Filter.Get_Out();
    //互补滤波-计算车体角度
    INS_Data.Mix_Velocity_X = Now_Velocity_X * alpha_v_adaptive  + INS_Data.IMU_X_Velocity * (1.0f - alpha_v_adaptive);
    INS_Data.Mix_Velocity_Y = Now_Velocity_Y * alpha_v_adaptive  + INS_Data.IMU_Y_Velocity * (1.0f - alpha_v_adaptive);
    INS_Data.Mix_Omega = Now_Omega * alpha_omega_adaptive  + INS_Data.IMU_Omega * (1.0f - alpha_omega_adaptive);
    //反馈校正：用融合后的速度修正积分速度（抑制漂移的关键！）
    //INS_Data.IMU_X_Velocity = INS_Data.Mix_Velocity_X;
    //INS_Data.IMU_Y_Velocity = INS_Data.Mix_Velocity_Y;
    //零速处理-可以添加IMU的角速度和加速度判断
    if(fabs(Now_Velocity_X) < 0.01f && fabs(Now_Velocity_Y) < 0.01f && fabs(Now_Omega) < 0.01f)
    {
        INS_Data.Mix_Velocity_X = 0.0f;
        INS_Data.Mix_Velocity_Y = 0.0f;
        INS_Data.Mix_Omega = 0.0f;
        INS_Data.IMU_X_Velocity = 0.0f;
        INS_Data.IMU_Y_Velocity = 0.0f;
    }
    #endif


}
