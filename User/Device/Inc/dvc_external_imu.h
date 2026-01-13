#ifndef __DVC_EXTERNAL_IMU_H
#define __DVC_EXTERNAL_IMU_H

#include "dvc_imu.h"


class Class_External_IMU : public Class_IMU {

  public:
    void Init(float yaw_offset);
    void TIM_Calculate_PeriodElapsedCallback(void);
    void UART_BMI_Data_Process(uint8_t *Buffer);
    void TIM1msMod50_Alive_PeriodElapsedCallback(void);
  
  private:
    uint32_t Flag;
    uint32_t Pre_Flag;

};


#endif