/**
 * @file dvc_imu.h
 * @author lez by wanghongxi
 * @brief imu
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DVC_IMU_H
#define DVC_IMU_H

#include "main.h"
#include "dvc_dwt.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_boardc_ist8310.h"
#include "alg_MahonyAHRS.h"
#include "alg_pid.h"

//----------------------------------------------------------------------------------------------------

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

enum Enum_IMU_Status 
{
    IMU_Status_DISABLE = 0,
    IMU_Status_ENABLE,
};

// Variable declaration

class Class_IMU
{
    public:

    void Init(void);
    void TIM_Calculate_PeriodElapsedCallback(void);
    void TIM1msMod50_Alive_PeriodElapsedCallback(void);

    float Get_Angle_Roll(void);
    float Get_Angle_Pitch(void);
    float Get_Angle_Yaw(void);

    float Get_Rad_Roll(void);
    float Get_Rad_Pitch(void);
    float Get_Rad_Yaw(void);

    float Get_Accel_X(void);
    float Get_Accel_Y(void);
    float Get_Accel_Z(void);

    float Get_Gyro_Roll(void);
    float Get_Gyro_Pitch(void);
    float Get_Gyro_Yaw(void);

    Enum_IMU_Status Get_IMU_Status(void);

    protected:

    Class_MahonyAHRS IMU_MahonyAHRS;
    Class_BoardC_BMI IMU_BMI088;
    Class_BoardC_IST8310 IMU_IST8310;
    Class_PID PID_IMU_Tempture;
    
    Enum_IMU_Status IMU_Status;

    void Get_Angle(void);
    void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

    INS_t INS;
    QEKF_INS_t QEKF_INS;
    IMU_Data_t BMI088_Raw_Data;
    IMU_Data_t Pre_BMI088_Raw_Data;
    IST8310_Real_Data_t IST8310_Real_Data;

    const float X_b[3] = {1, 0, 0};
    const float Y_b[3] = {0, 1, 0};
    const float Z_b[3] = {0, 0, 1};

    float INS_Quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float INS_Rad[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
    float INS_Angle[3] = {0.0f, 0.0f, 0.0f};

    //重力加速度
	const float Gravity[3] = {0, 0, 9.81f};

    //DWT计数
    float INS_DWT_Dt;
    float INS_DWT_Dt_Sum;
    uint32_t INS_DWT_Count;

    uint8_t gyro_update_flag = 0;
    uint8_t accel_update_flag = 0;
    uint8_t accel_temp_update_flag = 0;
    uint8_t mag_update_flag = 0;
    uint8_t imu_start_flag = 0;    

    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
};

//---------------------------------------------------------------------------------------------------
// Function declarations



#endif
//=====================================================================================================
// End of file
//=====================================================================================================
