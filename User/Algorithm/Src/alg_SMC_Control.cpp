#include "alg_SMC_Control.h"

#include "dvc_dwt.h"

void Class_SMC::Init(float __J, float __K, float __c, float __epsilon, float __Torque_Fric, float __T){

  J = __J;        //0.04
  K = __K;        //40
  c = __c;        //20
  epsilon = __epsilon;        //30
  Torque_Fric = __Torque_Fric;    //0
  T = __T;        //0.01

}

void Class_SMC::TIM_Data_Updata()
{
  d_Target  = (Target - Last_Target) * 100.0f * PI / 180.0f;
  dd_Target = (Target - 2 * Last_Target + LLast_Target) * 10.0f * PI / 180.0f;
  //dd_Target = 0.0f;

  error   = (Target - Now_x1) * PI / 180.0f;
  d_error = (d_Target - Now_x2) * PI / 180.0f;

  // if(abs(error) < 0.3){
  //   error = 0;
  // }

  s = c * error + d_error;

  Now_dx1 = Now_x2;

  LLast_Target = Last_Target;
  Last_Target  = Target;
}

float Class_SMC::Signal(float __s)
{
  if (__s > 0)
    return 1;
  else if (__s < 0)
    return -1;
  else
    return 0;
}

float Class_SMC::Sat_Function(float __s)
{
  float y;
  y = __s / epsilon;
  if(fabs(y) < s_Delta){
    return y;
  }
  else{
    return Signal(__s);
  }
}

void Class_SMC::TIM_Adjust_PeriodElapsedCallback(){

  TIM_Data_Updata();

  float Sat = Sat_Function(s);
  float Torque_ALL = J  * (epsilon * Sat + K * s + dd_Target + c * d_error); 

  float I = Torque_ALL / GM6020_TORQUE_CONST;
  Out = I * GM6020_I_TO_OUT;

  if(Out > 16384.0f){
    Out = 16384.0f;
  }
  else if(Out < -16384.0f){
    Out = -16384.0f;
  }
}


