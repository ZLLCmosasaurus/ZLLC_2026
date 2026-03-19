#include "alg_LESO.h"

void Class_LESO::Init(float __J, float __w0, float __Gearbox_Rate, Enum_LESO_Mode __LESO_Mode, Enum_Motor_Type __Motor_Type, float __Dt)
{
  J = __J;
  w0 = __w0;
  Gearbox_Rate = __Gearbox_Rate;
  LESO_Mode = __LESO_Mode;
  Motor_Type = __Motor_Type;
  Dt = __Dt;

  if(LESO_Mode == Observe_Motor_Omega){
    beta1 = 2 * w0;
    beta2 = w0 * w0;
  }
  else if(LESO_Mode == Observe_Motor_Angle){
    beta1 = 3 * w0;
    beta2 = 3 * w0 * w0;
    beta3 = w0 * w0 * w0;
  }
}

void Class_LESO::Disable()
{
  Init_Flag = 0;
}

void Class_LESO::TIM_Adjust_PeriodElapsedCallback(){
  if(LESO_Mode == Observe_Motor_Omega){
    beta1 = 2 * w0;
    beta2 = w0 * w0;
  }
  else if(LESO_Mode == Observe_Motor_Angle){
    beta1 = 3 * w0;
    beta2 = 3 * w0 * w0;
    beta3 = w0 * w0 * w0;
  }

  

  if(Motor_Type == Motor_GM6020){
    Input_Torque = CMD_Torque * GM6020_TORQUE_CONST * Gearbox_Rate / GM6020_I_TO_OUT;
  }
  else if(Motor_Type == Motor_GM3508){
    Input_Torque = CMD_Torque * M3508_TORQUE_CONSTANT * Gearbox_Rate / (M3508_TORQUE_CURRENT_TO_CMD_CURRENT * M3508_GEARBOX_RATE);
  }
  else if(Motor_Type == Motor_DM4310){
    Input_Torque = CMD_Torque;
  }

  if(LESO_Mode == Observe_Motor_Omega){
    if(!Init_Flag){
      Omega_hat = Now_Omega;
      Init_Flag = 1;
    }

    //  x_hat(k+1)|k
    Omega_hat        = Omega_hat + Disturbance_hat * Dt + Input_Torque * Dt / J;
    Disturbance_hat  = Disturbance_hat;

    //  x_hat(k+1)|(k+1)
    Omega_hat        = Omega_hat + beta1 * Dt * (Now_Omega - Omega_hat);
    Disturbance_hat  = Disturbance_hat + beta2 * Dt * (Now_Omega - Omega_hat);

  }
  else if(LESO_Mode == Observe_Motor_Angle){
    if(!Init_Flag){
      Angle_hat = Now_Angle;              //初始角度为IMU角度，防止预测开始时的一下发散
      Init_Flag = 1;
    }

    Angle_hat       = Angle_hat + Omega_hat * Dt;
    Omega_hat       = Omega_hat + Disturbance_hat * Dt + Input_Torque * Dt / J;
    Disturbance_hat = Disturbance_hat;

    Angle_hat       = Angle_hat + beta1 * Dt * (Now_Angle - Angle_hat);
    Omega_hat       = Omega_hat + beta2 * Dt * (Now_Angle - Angle_hat);
    Disturbance_hat = Disturbance_hat + beta3 * Dt * (Now_Angle - Angle_hat);

  }

  Math_Constrain(&Disturbance_hat, -10.0f / J, 10.0f / J);

  Disturbance_Torque = J * Disturbance_hat;
  if (Motor_Type == Motor_GM6020){
    Compensation_Out = -(Disturbance_Torque * GM6020_I_TO_OUT) / (GM6020_TORQUE_CONST * Gearbox_Rate);
  }
  else if (Motor_Type == Motor_GM3508){
    Compensation_Out = -(Disturbance_Torque * M3508_TORQUE_CURRENT_TO_CMD_CURRENT * M3508_GEARBOX_RATE) / (M3508_TORQUE_CONSTANT * Gearbox_Rate);
  }
  else if(Motor_Type == Motor_DM4310){
    Compensation_Out = -Disturbance_Torque;
  }

}