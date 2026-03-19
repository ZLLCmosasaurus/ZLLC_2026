#ifndef __ALG_LESO_H
#define __ALG_LESO_H

#include "stdint.h"
#include "drv_math.h"

#define GM6020_TORQUE_CONST 0.741f
#define GM6020_I_TO_OUT (16384.0f / 3.0f)

#define M3508_TORQUE_CONSTANT 0.3f                                                             // 3508带标准减速箱的转矩常数
#define M3508_TORQUE_CURRENT_TO_CMD_CURRENT (16384.0f / 20.0f)                                 // Icmd映射到Itorque
#define M3508_GEARBOX_RATE    19.0f                                                            // 3508自带减速箱的减速比 

enum Enum_LESO_Mode{
  Observe_Motor_Omega,
  Observe_Motor_Angle,
};

enum Enum_Motor_Type{
  Motor_GM6020,
  Motor_GM3508,
  Motor_GM2006,
  Motor_DM4310
};

class Class_LESO{

public:
  void Init(float __J, float __w0, float __Gearbox_Rate, Enum_LESO_Mode __LESO_Mode, Enum_Motor_Type __Motor_Type, float __Dt = 0.001f);
  void Disable();


  inline float Get_Omega_hat();                    //rad
  inline float Get_Angle_hat();                    //rad
  inline float Get_Disturbance_hat();              //
  inline float Get_Compensation_Out();             
  

  inline void Set_Now_Angle(float __Now_Angle);    //rad
  inline void Set_Now_Omega(float __Now_Omega);
  inline void Set_CMD_Torque(float __CMD_Torque);

  void TIM_Adjust_PeriodElapsedCallback();

private:

  uint8_t Init_Flag = 0;

  float Gearbox_Rate;

  float Dt;
  float J;                                //转动惯量
  float Input_Torque;
  float CMD_Torque;

  float w0;
  float beta1;
  float beta2;
  float beta3;

  Enum_LESO_Mode LESO_Mode;
  Enum_Motor_Type Motor_Type;

  float Angle_hat;
  float Omega_hat;
  float Disturbance_hat;
  float Disturbance_Torque;
  float Compensation_Out;

  float Now_Omega;                      //rad
  float Now_Angle;                      //rad
  
};


inline float Class_LESO::Get_Omega_hat()
{
  return Omega_hat;
}

inline float Class_LESO::Get_Angle_hat()
{
  return Angle_hat;
}

inline float Class_LESO::Get_Disturbance_hat()
{
  return Disturbance_hat;
}

inline float Class_LESO::Get_Compensation_Out()
{
  return Compensation_Out;
}

inline void Class_LESO::Set_Now_Angle(float __Now_Angle)
{
  Now_Angle = __Now_Angle;
}

inline void Class_LESO::Set_Now_Omega(float __Now_Omega)
{
  Now_Omega = __Now_Omega;
}

inline void Class_LESO::Set_CMD_Torque(float __CMD_Torque)
{
  CMD_Torque = __CMD_Torque;
}

#endif 