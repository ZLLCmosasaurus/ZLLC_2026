/**
 * @file alg_power_limit.cpp
 * @author lez
 * @brief 功率限制算法
 * @version 1.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_power_limit.h"
#include "dvc_djimotor.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/


// /**
//  * @brief ��ȡ���Ť�ص���
//  *
//  * @param num ������
//  * @return float ���Ť�ص���
//  */
// float Class_Power_Limit::Get_Torque_Current(uint8_t num)
// {
//     return Output_Torque_Current[num];
// }


// /**
//  * @brief ��ʱ�����ڵ���ص�����
//  *
//  */
// float test_scale;
// float Power_in;
// void Class_Power_Limit::TIM_Adjust_PeriodElapsedCallback(Class_DJI_Motor_C620 (&Motor)[4])
// {	
// 	#ifdef POWER_LIMIT_BUFFER_LOOP
// 	//if chassis Energy Buffer is too low, choose the spare way to limit the power
// 	if(Chassis_Buffer<Protected_Buffer)
// 	{
// 		Limit_K = (Chassis_Buffer-Min_Buffer)/Protected_Buffer;
// 		if(Limit_K<0) Limit_K = 0;
// 		for(int i=0;i<4;i++)
// 		{
// 			Output_Torque_Current[i] = Limit_K * Input_Torque_Current[i]; 
// 		}
// 		Output(Motor);		
// 	}
// 	// else use motor model to predict and limit the power
// 	#elif defined (POWER_LIMIT_NEW_CONTROL)
// 		float temp_current[4];
// 		//predict 4 motor's power and calculate the sum_power of predict and power_scale
// 		float tmp_total_power = 0;
// 		Output_Torque_Current[0] = Output_Torque_Current[1] = Output_Torque_Current[2] = Output_Torque_Current[3] =0.;
// 		for(int i=0;i<4;i++)
// 		{
// 			Predict_Power[i] = fabs(Omega[i] * (Torque_Torque_Current_Now[i]*CMD_CURRENT_TO_TORQUE) / Tansfer_Coefficient) + 
// 							   k1 * (Torque_Torque_Current_Now[i]*CMD_CURRENT_TO_TORQUE) * (Torque_Torque_Current_Now[i]*CMD_CURRENT_TO_TORQUE) +
// 			                   k2 * Omega[i] * Omega[i] + 
// 							   Alpha;
// 			tmp_total_power += Predict_Power[i];
// 		}
// 		Total_Predict_Power = tmp_total_power;

// 		//if the total predict power is bigger than the total power limit, scale the power
// 		if(Total_Predict_Power > Total_Power_Limit)
// 		{
// 			Power_Scale = Total_Power_Limit / Total_Predict_Power;		
// 			//calculate 4 motor's limit_power
// 			for(int i=0;i<4;i++)
// 			{
// 				Scaled_Give_Power[i] = Predict_Power[i] * Power_Scale;		

// 				//according to equation to calculate the torque_current_limit
// 				equation_a = k1 * CMD_CURRENT_TO_TORQUE* CMD_CURRENT_TO_TORQUE;
// 				equation_b = (Omega[i]*RAD_TO_RPM)*CMD_CURRENT_TO_TORQUE / Tansfer_Coefficient;
// 				equation_c = k2 * (Omega[i]*RAD_TO_RPM) * (Omega[i]*RAD_TO_RPM) - Scaled_Give_Power[i];
				
// 				//if the equation has no solution, return
// 				if((equation_b*equation_b - 4*equation_a*equation_c)<0)
// 					return;
// 				//if the equation has solution
// 				else
// 				{
// 					//answer = -b + sqrt(b*b - 4*a*c) / 2*a  or  -b - sqrt(b*b - 4*a*c) / 2*a
// 					if(Input_Torque_Current[i]>0)
// 					{
// 						temp_current[i] = (-equation_b + sqrt(equation_b*equation_b - 4*equation_a*equation_c)) / (2*equation_a);
// 						if(temp_current[i]>16000.0f)
// 						{
// 							Output_Torque_Current[i] = 16000;
// 						}
// 						else if(temp_current[i]<0)
// 						{
// 							Output_Torque_Current[i] = 0;
// 						}
// 						else
// 						{
// 							Output_Torque_Current[i] = temp_current[i];
// 						}
// 					}
// 					else
// 					{
// 						temp_current[i] = (-equation_b - sqrt(equation_b*equation_b - 4*equation_a*equation_c)) / (2*equation_a);
// 						if(temp_current[i]<-16000)
// 						{
// 							Output_Torque_Current[i] = -16000;
// 						}
// 						else if(temp_current[i]>0)
// 						{
// 							Output_Torque_Current[i] = 0;
// 						}
// 						else
// 						{
// 							Output_Torque_Current[i] = temp_current[i];
// 						}
// 					}
// 				}
// 			}
// 			//set limit current_cmd to motor
// 			Output(Motor);	
// 		}
// 		else
// 			Power_Scale = 1.0f;

// 	#elif defined (POWER_LIMIT_OLD_CONTROL)
// 	float Power_Limit;
// 	//max_power=50;//裁判系统读取不到数据时自己赋值,比赛使用时要注释掉
// 	float Power_out_1=abs((Omega[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0]));
// 	float Power_out_2=abs((Omega[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1]));
// 	float Power_out_3=abs((Omega[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2]));
// 	float Power_out_4=abs((Omega[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3]));
// 	float Power_out=Power_out_1+Power_out_2+Power_out_3+Power_out_4;
// 	float Power_out_square_sum=Power_out_1*Power_out_1+Power_out_2*Power_out_2+Power_out_3*Power_out_3+Power_out_4*Power_out_4;
// 	float Power_in_1=Power_out_1+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])+k2*((Omega[0])*(Omega[0]));
// 	float Power_in_2=Power_out_2+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])+k2*((Omega[1])*(Omega[1]));
// 	float Power_in_3=Power_out_3+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])+k2*((Omega[2])*(Omega[2]));
// 	float Power_in_4=Power_out_4+k1*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])+k2*((Omega[3])*(Omega[3]));
	
// 	float Target_current_square_sum((CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[0])+
// 									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[1])+
// 									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[2])+
// 									(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3])*(CMD_CURRENT_TO_TORQUE*Input_Torque_Current[3]));
// 	float Actual_speed_square_sum=((Omega[0])*(Omega[0])+
// 								   (Omega[1])*(Omega[1])+
// 								   (Omega[2])*(Omega[2])+
// 								   (Omega[3])*(Omega[3]));
// 	Power_in=Power_in_1+Power_in_2+Power_in_3+Power_in_4;
	
// 	//根据电量选择使用电容组
// 	//根据电容组电压线性改变限制功率
// 	if(Supercap_Voltage >= 13.0f)
// 	{
// 		if(Supercap_Print_Flag == 0)
// 		{
// 			Power_Limit= Total_Power_Limit *1.5f + (Supercap_Voltage * 100.0f - 130.0f) * 2.0f;	
// 		}
// 		else
// 		{
// 			Power_Limit= Total_Power_Limit *1.5f + (Supercap_Voltage * 100.0f - 130.0f) * 4.0f;
// 		}
// 	}
// 	else
// 		Power_Limit = Total_Power_Limit * 1.2f;

// 	//功率限制
// 	if(Power_in>Power_Limit)  
// 	{
// 		if((Power_out*Power_out-4.0f*k1*(Target_current_square_sum)*(k2*(Actual_speed_square_sum)-Power_Limit+Alpha))>=0)
// 		{
// 			Power_Scale=(-1.0f*Power_out+sqrt(Power_out*Power_out-4.0f*k1*(Target_current_square_sum)*(k2*(Actual_speed_square_sum)-Power_Limit+Alpha)))/(2.0f*k2*Target_current_square_sum);
// 			Power_Scale/= 100.0f;
// 			if(Power_Scale>1)Power_Scale=1;
// 			if(Power_Scale<0)Power_Scale=0;
// 		}
// 		else 
// 			Power_Scale=0;
// 	}
// 	else 
// 	{
// 		Power_Scale=1.0f;
// 	}
// 	Output_Torque_Current[0]=Input_Torque_Current[0]*Power_Scale;
// 	Output_Torque_Current[1]=Input_Torque_Current[1]*Power_Scale;
// 	Output_Torque_Current[2]=Input_Torque_Current[2]*Power_Scale;
// 	Output_Torque_Current[3]=Input_Torque_Current[3]*Power_Scale;
// 	Output(Motor);
	
// //使用的电流值改为实际电流值，用于调节POWER_K1和POWER_K2，重新计算Power_in_actual
// //	Power_out_1_actual=abs_f(((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor1.actual_current*20.0f/16384.0f));
// //	Power_out_2_actual=abs_f(((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor2.actual_current*20.0f/16384.0f));
// //	Power_out_3_actual=abs_f(((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor3.actual_current*20.0f/16384.0f));
// //	Power_out_4_actual=abs_f(((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor4.actual_current*20.0f/16384.0f));
// //	Power_out_actual=Power_out_1_actual+Power_out_2_actual+Power_out_3_actual+Power_out_4_actual;
// //	Power_in_1_actual=Power_out_1_actual+POWER_K1*(K_M*chassis_motor1.actual_current*20.0f/16384.0f)*(K_M*chassis_motor1.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
// //	Power_in_2_actual=Power_out_2_actual+POWER_K1*(K_M*chassis_motor2.actual_current*20.0f/16384.0f)*(K_M*chassis_motor2.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
// //	Power_in_3_actual=Power_out_3_actual+POWER_K1*(K_M*chassis_motor3.actual_current*20.0f/16384.0f)*(K_M*chassis_motor3.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
// //	Power_in_4_actual=Power_out_4_actual+POWER_K1*(K_M*chassis_motor4.actual_current*20.0f/16384.0f)*(K_M*chassis_motor4.actual_current*20.0f/16384.0f)+POWER_K2*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
// //	Power_in_actual=Power_in_1_actual+Power_in_2_actual+Power_in_3_actual+Power_in_4_actual;
// 	#endif
// }

// /**
//  * @brief �趨����������
//  *
//  */
// void Class_Power_Limit::Output(Class_DJI_Motor_C620 (&Motor)[4])
// {
//     for(int i=0;i<4;i++)
// 	{
//         Motor[i].CAN_Tx_Data[0] = (int16_t)Output_Torque_Current[i] >> 8;
//         Motor[i].CAN_Tx_Data[1] = (int16_t)Output_Torque_Current[i];
//     }
// }

// /**
//  * @brief �趨�ĸ�����Ŀ��Ƶ����͵�ǰ���ٶ�
//  *
//  */
// void Class_Power_Limit::Set_Motor(Class_DJI_Motor_C620 (&Motor)[4])
// {
//     for(int i=0;i<4;i++)
//     {
//         Input_Torque_Current[i] = Motor[i].Get_Out();
//         Omega[i] = Motor[i].Get_Now_Omega_Radian();
// 		Torque_Torque_Current_Now[i] = Motor[i].Get_Now_Torque();
//     }
// }

/* Function prototypes -------------------------------------------------------*/


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
