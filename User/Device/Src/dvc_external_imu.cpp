#include "dvc_external_imu.h"

void Class_External_IMU::TIM1msMod50_Alive_PeriodElapsedCallback(void)
{
    if (Flag == Pre_Flag) // 判断陀螺仪是否掉线
    {
        IMU_Status = IMU_Status_DISABLE;
    }
    else
        IMU_Status = IMU_Status_ENABLE;

    Pre_Flag = Flag;
}

uint32_t imu_cnt = 0;
float IMU_Dt;
void Class_External_IMU::UART_BMI_Data_Process(uint8_t *Buffer){
	
  if(Buffer[0] == 0xA5 && Buffer[13] == 0xB5){
    Flag ++;
    IMU_Dt = DWT_GetDeltaT(&imu_cnt);
    BMI088_Raw_Data.Accel[0] = (float)((int16_t)(Buffer[1] << 8) | (Buffer[2])) / 100.0f;
    BMI088_Raw_Data.Accel[1] = (float)((int16_t)(Buffer[3] << 8) | (Buffer[4])) / 100.0f;
    BMI088_Raw_Data.Accel[2] = (float)((int16_t)(Buffer[5] << 8) | (Buffer[6])) / 100.0f;

    BMI088_Raw_Data.Gyro[0] = (float)((int16_t)(Buffer[7] << 8) | (Buffer[8])) / 100.0f;
    BMI088_Raw_Data.Gyro[1] = (float)((int16_t)(Buffer[9] << 8) | (Buffer[10])) / 100.0f;
    BMI088_Raw_Data.Gyro[2] = (float)((int16_t)(Buffer[11] << 8) | (Buffer[12])) / 100.0f;
  }
}

void Class_External_IMU::Init(float yaw_offset)
{ 
    //EKF初始化
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0, yaw_offset, &QEKF_INS);
}

static float tmp_gravity_b[3];
void Class_External_IMU::TIM_Calculate_PeriodElapsedCallback(void)
{
  static uint8_t Tempture_Cnt_mod50 = 0;
    Tempture_Cnt_mod50++;

    INS_DWT_Dt = DWT_GetDeltaT(&INS_DWT_Count);
    INS_DWT_Dt_Sum += INS_DWT_Dt;

    INS.Accel[0] = BMI088_Raw_Data.Accel[0];
    INS.Accel[1] = BMI088_Raw_Data.Accel[1];
    INS.Accel[2] = BMI088_Raw_Data.Accel[2];
    INS.Gyro[0] = BMI088_Raw_Data.Gyro[0];
    INS.Gyro[1] = BMI088_Raw_Data.Gyro[1];
    INS.Gyro[2] = BMI088_Raw_Data.Gyro[2];

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], INS_DWT_Dt, &QEKF_INS);

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

    imu_start_flag = 1;
}