/**
 * @file dvc_imu.cpp
 * @author lez by wanghongxi
 * @brief imu
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#include "dvc_imu.h"
#include "buzzer.h"

void Class_IMU::Init()
{
    // 初始化BMI088传感器，计算零漂 并检查初始化是否成功
    IMU_BMI088.init(&hspi1,&BMI088_Raw_Data);
    HAL_Delay(100);

    // 初始化IST8310传感器
    IMU_IST8310.init(&hi2c3);
    HAL_Delay(100);

    // 初始化MahonyAHRS算法，并传入初始四元数
    IMU_MahonyAHRS.init(INS_Quat);
 
    //EKF初始化
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0 ,&QEKF_INS);

    INS.AccelLPF = 0.0085;

    //初始化温控pid参数
    PID_IMU_Tempture.Init(2000, 3000, 0, 0.0, uint32_max, uint32_max);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

}

float tmp_gravity_b[3];
void Class_IMU::TIM_Calculate_PeriodElapsedCallback(void)
{
    static uint8_t Tempture_Cnt_mod50 = 0;
    Tempture_Cnt_mod50++;

    INS_DWT_Dt = DWT_GetDeltaT(&INS_DWT_Count);
    INS_DWT_Dt_Sum += INS_DWT_Dt;

    IMU_BMI088.BMI088_Read(&BMI088_Raw_Data);

    INS.Accel[0] = BMI088_Raw_Data.Accel[0];
    INS.Accel[1] = BMI088_Raw_Data.Accel[1];
    INS.Accel[2] = BMI088_Raw_Data.Accel[2];
    INS.Gyro[0] = BMI088_Raw_Data.Gyro[0];
    INS.Gyro[1] = BMI088_Raw_Data.Gyro[1];
    INS.Gyro[2] = BMI088_Raw_Data.Gyro[2];

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], INS_DWT_Dt ,&QEKF_INS);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(X_b, INS.xn, INS.q);
    BodyFrameToEarthFrame(Y_b, INS.yn, INS.q);
    BodyFrameToEarthFrame(Z_b, INS.zn, INS.q); 

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    
    EarthFrameToBodyFrame(Gravity, tmp_gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - tmp_gravity_b[i]) * INS_DWT_Dt / (INS.AccelLPF + INS_DWT_Dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + INS_DWT_Dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    // 获取最终数据
    Get_Angle();

    if(Tempture_Cnt_mod50 % 50 == 0)
    {
        PID_IMU_Tempture.Set_Now(BMI088_Raw_Data.Temperature);
        PID_IMU_Tempture.Set_Target(44.0f);
        PID_IMU_Tempture.TIM_Adjust_PeriodElapsedCallback();
        int16_t Compare = PID_IMU_Tempture.Get_Out();
        Math_Constrain(&Compare,(int16_t)0,(int16_t)5000);
        TIM_Set_PWM(&htim10, TIM_CHANNEL_1, Compare);
    }

    imu_start_flag = 1;
}

void Class_IMU::TIM1msMod50_Alive_PeriodElapsedCallback(void)
{
    if(imu_start_flag)  //陀螺仪已经开启通讯
    {
        if((Pre_BMI088_Raw_Data.Accel[0] == BMI088_Raw_Data.Accel[0]) 
        && (Pre_BMI088_Raw_Data.Accel[1] == BMI088_Raw_Data.Accel[1]) 
        && (Pre_BMI088_Raw_Data.Accel[2] == BMI088_Raw_Data.Accel[2]))  //判断陀螺仪是否掉线
        {
            IMU_Status = IMU_Status_DISABLE;
            buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
        }
        else IMU_Status = IMU_Status_ENABLE;

        memcpy(&Pre_BMI088_Raw_Data, &BMI088_Raw_Data, sizeof(IMU_Data_t));  //保留上一次的数据
    }
}

void Class_IMU::Get_Angle()
{
    // 获取最终数据
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

    INS_Rad[0] = atan2f(2.0f*(INS_Quat[0]*INS_Quat[3]+INS_Quat[1]*INS_Quat[2]), 2.0f*(INS_Quat[0]*INS_Quat[0]+INS_Quat[1]*INS_Quat[1])-1.0f);
    INS_Rad[1] = asinf(-2.0f*(INS_Quat[1]*INS_Quat[3]-INS_Quat[0]*INS_Quat[2]));
    INS_Rad[2] = atan2f(2.0f*(INS_Quat[0]*INS_Quat[1]+INS_Quat[2]*INS_Quat[3]),2.0f*(INS_Quat[0]*INS_Quat[0]+INS_Quat[3]*INS_Quat[3])-1.0f);
    for(int i=0;i<3;i++){
        INS_Angle[i] = INS_Rad[i] * 180.0f / 3.1415926f;
    }
}

void Class_IMU::TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;
    if(value < 0) value = 0;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}

float Class_IMU::Get_Angle_Roll(void)
{
    return (INS.Roll);
}

float Class_IMU::Get_Angle_Pitch(void)
{
    return (INS.Pitch);
}

float Class_IMU::Get_Angle_Yaw(void)
{
    return (INS.Yaw);
}

float Class_IMU::Get_Accel_X(void)
{
    return (INS.Accel[0]);
}

float Class_IMU::Get_Accel_Y(void)
{
    return (INS.Accel[1]);
}

float Class_IMU::Get_Accel_Z(void)
{
    return (INS.Accel[2]);
}

float Class_IMU::Get_Gyro_Roll(void)
{
    return (INS.Gyro[1]);
}

float Class_IMU::Get_Gyro_Pitch(void)
{
    return (INS.Gyro[0]);
}

float Class_IMU::Get_Gyro_Yaw(void)
{
    return (INS.Gyro[2]);
}

float Class_IMU::Get_Rad_Roll(void)
{
    return (INS.Roll/180.f*PI);
}

float Class_IMU::Get_Rad_Pitch(void)
{
    return (INS.Pitch/180.f*PI);
}

float Class_IMU::Get_Rad_Yaw(void)
{
    return (INS.Yaw/180.f*PI);
}

Enum_IMU_Status Class_IMU::Get_IMU_Status(void)
{
    return IMU_Status;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void Class_IMU::BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void Class_IMU::EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

//====================================================================================================
// END OF CODE
//====================================================================================================

