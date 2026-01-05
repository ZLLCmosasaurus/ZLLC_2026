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
  d_Target  = (Target - Last_Target) * 500.0f;
  dd_Target = (Target - 2 * Last_Target + LLast_Target) * 10.0f;
  //dd_Target = 0.0f;

  error   = Target - Now_x1;
  d_error = d_Target - Now_x2;

  s = c * error + d_error;

  Now_dx1 = Now_x2;

  LLast_Target = Last_Target;
  Last_Target  = Target;
}

float Class_SMC::Sat_Function(float __s)
{
  if(fabs(__s) > s_Delta){
    return __s > 0 ? 1 : -1;
  }
  else{
    return __s/s_Delta;
  }
}

void Class_SMC::TIM_Adjust_PeriodElapsedCallback(){

  TIM_Data_Updata();

  float Sat = Sat_Function(s);
  float Torque_ALL = J * PI * (epsilon * Sat + K * s + dd_Target + c * d_error) / 180.0f; 

  float I = Torque_ALL / GM6020_TORQUE_CONST;
  Out = I * GM6020_I_TO_OUT;

  if (error > 0.1)
  {
    if (Out > 0)
    {
      Out += 1000;
    }
    else if (Out < 0)
    {
      Out -= 1000;
    }

    if (fabs(error) < 0.08)
    {
      Out = 0.0;
    }
  }

  if(Out > 16384.0f){
    Out = 16384.0f;
  }
  else if(Out < -16384.0f){
    Out = -16384.0f;
  }
}


