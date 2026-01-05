#ifndef __ALG_SMC_CONTROL_H
#define __ALG_SMC_CONTROL_H

#include "arm_math.h"

#define GM6020_TORQUE_CONST 0.741f
#define GM6020_I_TO_OUT 16384.0f / 3.0f

class Class_SMC{
  public:

  void Init(float __J, float __K, float __c, float __epsilon, float __Torque_Fric = 0.0f, float __T = 0.001);

  inline void Set_Now(float __Now_x1, float __Now_x2);
  inline void Set_Target(float __Target);

  inline float Get_Out();

  void TIM_Adjust_PeriodElapsedCallback();

  private:

  float J;          //转动惯量
  float K;          //指数趋近律系数
  float c;          //收敛到平衡点速度
  float epsilon;
  float Torque_Fric;
  
  float Target = 0.0f;
  float Last_Target = 0.0f;
  float LLast_Target = 0.0f;
  float d_Target = 0.0f;
  float dd_Target = 0.0f;

  float Now_x1 = 0.0f;           //angle 度
  float Now_x2 = 0.0f;           //按模型来说是x1的一阶导
  float Now_dx1 = 0.0f;
  float Now_dx2 = 0.0f; 

  float error;
  float d_error;

  float s;                //滑模面函数
  float ds;

  float T = 0.001;        //1000Hz

  float Out = 0.0f;

  const float s_Delta = 3.0f;

  void TIM_Data_Updata();
  float Sat_Function(float s);    //饱和函数
};

inline void Class_SMC::Set_Now(float __Now_x1, float __Now_x2)
{
  Now_x1 = __Now_x1;
  Now_x2 = __Now_x2;
}

inline void Class_SMC::Set_Target(float __Target)
{
  Target = __Target;
}

inline float Class_SMC::Get_Out()
{
  return Out;
}

#endif