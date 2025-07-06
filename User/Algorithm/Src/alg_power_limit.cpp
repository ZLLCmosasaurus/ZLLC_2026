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

#define POWER_LIMIT_NEW_CONTROL
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
	#ifdef POWER_LIMIT_BUFFER_LOOP
	//if chassis Energy Buffer is too low, choose the spare way to limit the power
	if(Chassis_Buffer<Protected_Buffer)
	{
		Limit_K = (Chassis_Buffer-Min_Buffer)/Protected_Buffer;
		if(Limit_K<0) Limit_K = 0;
		for(int i=0;i<4;i++)
		{
			Output_Torque_Current[i] = Limit_K * Input_Torque_Current[i]; 
		}
		Output(Motor);		
	}
	// else use motor model to predict and limit the power
	#elif defined (POWER_LIMIT_NEW_CONTROL)
		//计算缓冲能量
		Buffer_power = (Chassis_Buffer-Min_Buffer) * Buffer_K;
		Math_Constrain(&Buffer_power,-Buffer_power_limit,Buffer_power_limit);
		//收集电机参数
		Set_Motor(Motor);
        //跑功率限制
		// float power_limit_sum = fabs(Total_Power_Limit + Buffer_power);
		Total_Power_Limit = fabs(Total_Power_Limit + Buffer_power);
        Limit_K = Calculate_Limit_K(Omega,Input_Torque,Total_Power_Limit,4);
        //设置输出
		Output(Motor);
	#elif defined (POWER_LIMIT_OLD_CONTROL)
	float Power_Limit;
	//max_power=50;//裁判系统读取不到数据时自己赋值,比赛使用时要注释掉
	float Power_out_1=abs((Omega[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0]));
	float Power_out_2=abs((Omega[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1]));
	float Power_out_3=abs((Omega[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2]));
	float Power_out_4=abs((Omega[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3]));
	float Power_out=Power_out_1+Power_out_2+Power_out_3+Power_out_4;
	float Power_out_square_sum=Power_out_1*Power_out_1+Power_out_2*Power_out_2+Power_out_3*Power_out_3+Power_out_4*Power_out_4;
	float Power_in_1=Power_out_1+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])+k2*((Omega[0])*(Omega[0]));
	float Power_in_2=Power_out_2+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])+k2*((Omega[1])*(Omega[1]));
	float Power_in_3=Power_out_3+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])+k2*((Omega[2])*(Omega[2]));
	float Power_in_4=Power_out_4+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])+k2*((Omega[3])*(Omega[3]));
	
	float Target_current_square_sum((CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])+
									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])+
									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])+
									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3]));
	float Actual_speed_square_sum=((Omega[0])*(Omega[0])+
								   (Omega[1])*(Omega[1])+
								   (Omega[2])*(Omega[2])+
								   (Omega[3])*(Omega[3]));
	Power_in=Power_in_1+Power_in_2+Power_in_3+Power_in_4;
	
	//根据电量选择使用电容组
	//根据电容组电压线性改变限制功率
	if(Supercap_Voltage >= 13.0f)
	{
		if(Supercap_Print_Flag == 0)
		{
			Power_Limit= Total_Power_Limit *1.5f + (Supercap_Voltage * 100.0f - 130.0f) * 2.0f;	
		}
		else
		{
			Power_Limit= Total_Power_Limit *1.5f + (Supercap_Voltage * 100.0f - 130.0f) * 4.0f;
		}
	}
	else
		Power_Limit = Total_Power_Limit * 1.2f;

	//功率限制
	if(Power_in>Power_Limit)  
	{
		if((Power_out*Power_out-4.0f*k1*(Target_current_square_sum)*(k2*(Actual_speed_square_sum)-Power_Limit+Alpha))>=0)
		{
			Power_Scale=(-1.0f*Power_out+sqrt(Power_out*Power_out-4.0f*k1*(Target_current_square_sum)*(k2*(Actual_speed_square_sum)-Power_Limit+Alpha)))/(2.0f*k2*Target_current_square_sum);
			Power_Scale/= 100.0f;
			if(Power_Scale>1)Power_Scale=1;
			if(Power_Scale<0)Power_Scale=0;
		}
		else 
			Power_Scale=0;
	}
	else 
	{
		Power_Scale=1.0f;
	}
	Output_Torque_Current[0]=Input_Torque_Current[0]*Power_Scale;
	Output_Torque_Current[1]=Input_Torque_Current[1]*Power_Scale;
	Output_Torque_Current[2]=Input_Torque_Current[2]*Power_Scale;
	Output_Torque_Current[3]=Input_Torque_Current[3]*Power_Scale;
	Output(Motor);
	
//使用的电流值改为实际电流值，用于调节POWER_K1和POWER_K2，重新计算Power_in_actual
//	Power_out_1_actual=abs_f(((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor1.actual_current*20.0f/16384.0f));
//	Power_out_2_actual=abs_f(((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor2.actual_current*20.0f/16384.0f));
//	Power_out_3_actual=abs_f(((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor3.actual_current*20.0f/16384.0f));
//	Power_out_4_actual=abs_f(((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor4.actual_current*20.0f/16384.0f));
//	Power_out_actual=Power_out_1_actual+Power_out_2_actual+Power_out_3_actual+Power_out_4_actual;
//	Power_in_1_actual=Power_out_1_actual+POWER_K1*(K_M*chassis_motor1.actual_current*20.0f/16384.0f)*(K_M*chassis_motor1.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
//	Power_in_2_actual=Power_out_2_actual+POWER_K1*(K_M*chassis_motor2.actual_current*20.0f/16384.0f)*(K_M*chassis_motor2.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
//	Power_in_3_actual=Power_out_3_actual+POWER_K1*(K_M*chassis_motor3.actual_current*20.0f/16384.0f)*(K_M*chassis_motor3.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
//	Power_in_4_actual=Power_out_4_actual+POWER_K1*(K_M*chassis_motor4.actual_current*20.0f/16384.0f)*(K_M*chassis_motor4.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
//	Power_in_actual=Power_in_1_actual+Power_in_2_actual+Power_in_3_actual+Power_in_4_actual;
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
