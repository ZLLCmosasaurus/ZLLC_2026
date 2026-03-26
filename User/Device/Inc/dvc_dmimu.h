#ifndef __DM_IMU_H
#define __DM_IMU_H

// #include "stm32h7xx_hal.h"
#include "drv_can.h"
#include "dvc_dwt.h"
#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

}imu_t;

//canid （0x01）为发送标识符 mstid(0x11)为接收标识符 ，这两个参数可以在上位机软件设置

class Class_DM_IMU
{
public:
	inline float Get_DMIMU_Pitch(void) { return Data.pitch; }
	inline float Get_DMIMU_Roll(void) { return Data.roll; }
	inline float Get_DMIMU_Yaw(void) { return Data.yaw; }
	inline float Get_DMIMU_Gyro_Pitch(void) { return Data.gyro[1]; }
	inline float Get_DMIMU_Gyro_Yaw(void) { return Data.gyro[2]; }
	inline float Get_DMIMU_Gyro_Roll(void) { return Data.gyro[0]; }

	int float_to_uint(float x_float, float x_min, float x_max, int bits);
	float uint_to_float(int x_int, float x_min, float x_max, int bits);
	void IMU_UpdateAccel(uint8_t *pData);
	void IMU_UpdateGyro(uint8_t *pData);
	void IMU_UpdateEuler(uint8_t *pData);
	void IMU_UpdateQuaternion(uint8_t *pData);
	void IMU_UpdateData(uint8_t *pData);
	void IMU_RequestData(FDCAN_HandleTypeDef *hcan, uint16_t can_id, uint8_t reg);

protected:
	imu_t Data;

};

#endif