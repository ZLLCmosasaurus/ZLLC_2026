/**
 * @file alg_power_limit.cpp
 * @author cjw
 * @brief 功率限制算法
 * @version 1.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_power_limit.h"
#include "dvc_djimotor.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief ��ȡ���Ť�ص���
 *
 * @param num ������
 * @return float ���Ť�ص���
 */
float Class_Power_Limit::Get_Torque_Current(uint8_t num)
{
    return Output_Torque[num];
}

float Class_Power_Limit::Calculate_Limit_K(float omega[], float torque[], float power_limit, uint8_t motor_nums)
{
    float limit_k = 1.; // 输出伸缩因子k

    float tmp_predict = 0;

    float torque_square_sum = 0;
    float omega_square_sum = 0;
    float torque_multi_omega_sum = 0;

    float func_a, func_b, func_c;
    float delta;

    for (int i = 0; i < motor_nums; i++)
    {
        torque_square_sum += torque[i] * torque[i];
        omega_square_sum += omega[i] * omega[i];
        torque_multi_omega_sum += fabs(omega[i] * torque[i]);

        tmp_predict += fabs(omega[i] * torque[i]) +
                       k1 * torque[i] * torque[i] +
                       k2 * omega[i] * omega[i] +
                       Alpha;
    }
    Total_Predict_Power = tmp_predict;

    if (tmp_predict > power_limit)
    {
        func_a = k1 * torque_square_sum;
        func_b = torque_multi_omega_sum;
        func_c = k2 * omega_square_sum - power_limit + Alpha * motor_nums;

        delta = func_b * func_b - 4 * func_a * func_c; // b*b-4*a*c

        if (delta >= 0)
        {
            limit_k = (-func_b + sqrtf(delta)) / (2 * func_a); // 求根公式
        }
        else
        {
            limit_k = 1;
        }
    }

    return (limit_k > 1) ? 1 : limit_k;
}


/**
 * @brief ��ʱ�����ڵ���ص�����
 *
 */
float test_scale;
float Power_in;
void Class_Power_Limit::TIM_Adjust_PeriodElapsedCallback(Class_DJI_Motor_C620 (&Motor)[4])
{	
	#ifdef defined (POWER_LIMIT_1)
		//计算缓冲能量
		Buffer_power = (Chassis_Buffer-Min_Buffer) * Buffer_K;
		Math_Constrain(&Buffer_power,-Buffer_power_limit,Buffer_power_limit);

    //跑功率限制
		// float power_limit_sum = fabs(Total_Power_Limit + Buffer_power);
		Total_Power_Limit = fabs(Total_Power_Limit + Buffer_power);
    Limit_K = Calculate_Limit_K(Omega,Input_Torque,Total_Power_Limit,4);
     //设置输出
		Output(Motor);
	#else defined (POWER_LIMIT_2)

	#endif

}

/**
 * @brief �趨����������
 *
 */
void Class_Power_Limit::Output(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
	{
		int16_t out_limit = Motor[i].Get_Out()*Limit_K;
        Motor[i].CAN_Tx_Data[0] = (int16_t)out_limit >> 8;
        Motor[i].CAN_Tx_Data[1] = (int16_t)out_limit;
    }
}

/**
 * @brief �趨�ĸ�����Ŀ��Ƶ����͵�ǰ���ٶ�
 *
 */
void Class_Power_Limit::Set_Motor(Class_DJI_Motor_C620 (&Motor)[4])
{
    for(int i=0;i<4;i++)
    {
      Input_Torque[i] = Motor[i].Get_Out()*current_to_torqure;
      Omega[i] = Motor[i].Get_Now_Omega_Radian();
			Torque_Now[i] = Motor[i].Get_Now_Torque()*current_to_torqure;
    }
}

/* Function prototypes -------------------------------------------------------*/


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
