/**
 * @file alg_MahonyAHRS.h
 * @author lez by yssickjgd
 * @brief 互补滤波
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef ALG_MAHONYAHRS_h
#define ALG_MAHONYAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

// Definitions

// #define sampleFreq	1000.0f			// sample frequency in Hz
// #define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
// #define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

// volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
// volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
// //volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
// volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

class Class_MahonyAHRS 
{
    public:

    void init(float quat[4]);
    void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
    void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);

    protected:

    float twoKp = 2.0f * 0.5f;  // 2 * proportional gain
    float twoKi = 2.0f * 0.0f;  // 2 * integral gain
    float sampleFreq = 1000.0f;	// sample frequency in Hz
    float integralFBx = 0.0f;
    float integralFBy = 0.0f;
    float integralFBz = 0.0f;	// integral error terms scaled by Ki
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
};


// extern volatile float twoKp;			// 2 * proportional gain (Kp)
// extern volatile float twoKi;			// 2 * integral gain (Ki)
// extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
