/**
 * @file crt_chassis.cpp
 * @author cjw
 * @brief 底盘
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"
#include "buzzer.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Steering_Wheel_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    //Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2,100.f);
    
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.05f,0.05f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.05f,0.05f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    //电机PID批量初始化
    for (int i = 0; i < 4; i++) 
    {
        Motor_Wheel[i].PID_Omega.Init(800.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }
    #ifdef AGV
    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OPENLOOP, 8.0f);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OPENLOOP, 8.0f);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_OPENLOOP, 8.0f);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x207, DJI_Motor_Control_Method_OPENLOOP, 8.0f);
    
    //舵向电机PID初始化

    Motor_Steer[0].PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, Motor_Steer[0].Get_Output_Max(), Motor_Steer[0].Get_Output_Max());
    Motor_Steer[0].PID_Omega.Init(1200.0f,8.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());
    
    Motor_Steer[1].PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, Motor_Steer[1].Get_Output_Max(), Motor_Steer[1].Get_Output_Max());
    Motor_Steer[1].PID_Omega.Init(1200.0f, 8.0f, 0.0f, 0.0f, 8000, Motor_Steer[1].Get_Output_Max());

    Motor_Steer[2].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[2].Get_Output_Max(), Motor_Steer[2].Get_Output_Max());
    Motor_Steer[2].PID_Omega.Init(1200.0f, 8.0f, 0.0f, 0.0f, 8000, Motor_Steer[2].Get_Output_Max());

    Motor_Steer[3].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[3].Get_Output_Max(), Motor_Steer[3].Get_Output_Max());
    Motor_Steer[3].PID_Omega.Init(1200.0f, 8.0f, 0.0f, 0.0f, 8000, Motor_Steer[3].Get_Output_Max());


    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Steer[1].Init(&hfdcan1, DJI_Motor_ID_0x204);
    Motor_Steer[2].Init(&hfdcan1, DJI_Motor_ID_0x206);
    Motor_Steer[3].Init(&hfdcan1, DJI_Motor_ID_0x208);
    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(-0.07f);
    Motor_Steer[1].Set_Zero_Position(0.88f);
    Motor_Steer[2].Set_Zero_Position(2.42f);
    Motor_Steer[3].Set_Zero_Position(-1.13f);
    #endif

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}

float dsb;
/**
 * @brief 速度解算
 *
 */
float car_V,car_yaw;//车体总体朝向与速度
void Class_Steering_Wheel_Chassis::Speed_Resolution()
{
	if(Motor_Steer[0].Get_MA600_Status()==MA600_Status_DISABLE || Motor_Steer[1].Get_MA600_Status()==MA600_Status_DISABLE ||
       Motor_Steer[2].Get_MA600_Status()==MA600_Status_DISABLE || Motor_Steer[3].Get_MA600_Status()==MA600_Status_DISABLE)
	{
        buzzer_setTask(&buzzer, BUZZER_DJI_STARTUP_PRIORITY);
		for(uint8_t i=0;i<4;i++)
		{
			Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
			Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
			Motor_Wheel[i].Set_Out(0.0f);

			Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
			Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
			Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
			Motor_Steer[i].Set_Out(0.0f);
		}		
		return;		
	}
    // else{
    //     buzzer_setTask(&buzzer, BUZZER_FREE_PRIORITY);
    // }		
    #ifdef AGV 
    switch (Chassis_Control_Type)
    {
        case(Chassis_Control_Type_DISABLE):
        {
            for(int i = 0; i < 4;i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Out(0.0f);

                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
                Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Steer[i].Set_Out(0.0f);
            }
            break;
        }
		//舵轮运动学逆解
        case(Chassis_Control_Type_FLLOW):
        case(Chassis_Control_Type_SPIN_Positive):
        case(Chassis_Control_Type_SPIN_NePositive):
        case(Chassis_Control_Type_Drive):
        {
            //轮组自锁，每个小轮坐标系都符合右手系
            static uint32_t Lock_Time = 0;
            static uint8_t  Lock_Flag = 0;
            float delta_Angle = 0.0f, Transform_Radian = 0.0f;                  //用于优化处理的变量
            if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
            {
                Lock_Time++;
                if(Lock_Time > 500)  Lock_Flag = 1;
                if (Lock_Flag)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

                        Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                    }

                    Motor_Steer[0].Set_Target_Radian( THETA);
                    Motor_Steer[1].Set_Target_Radian(-THETA);
                    Motor_Steer[2].Set_Target_Radian( THETA);
                    Motor_Steer[3].Set_Target_Radian(-THETA);
                    for (int i = 0; i < 4; i++)
                    {
                        Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                        //优劣弧处理
                        if((i % 2) == 0){
                            delta_Angle = THETA - Transform_Radian;
                        }
                        else{
                            delta_Angle = - THETA - Transform_Radian;
                        }
                        
						delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);//将角度限制在-PI到PI之间

                        if(delta_Angle > PI/2.0f)
                        {
                            delta_Angle = delta_Angle - PI;
                        }
                        else if(delta_Angle < -PI/2.0f)
                        {
                            delta_Angle = delta_Angle + PI;
                        }
                        Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);
                        Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                        // Motor_Steer[i].Set_Out(0.0f);
                        // Motor_Wheel[i].Set_Out(0.0f);
                        Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                    }
                    break;
                }
            }
            else
						{
                Lock_Time = 0;
            }

            if(Lock_Flag)
						{
                Lock_Flag = 0;
            }

            //0 1 2 3 左前 右前 右后 左后 顺时针    右x前y坐标系   基于编码器0度朝前，逆时针为正角度   确保轮子正转的是朝前的速度，不然得单独加负号
            float True_Vx[4], True_Vy[4], True_Target_Angle_Radian[4];
            
            //斜坡处理
            dsb = R_DIST;
            True_Vx[0] = True_Vx[3] = Slope_Velocity_X.Get_Out() - sinf((PI/2) - THETA) * Target_Omega *  R_DIST * 4.0f;
            True_Vx[1] = True_Vx[2] = Slope_Velocity_X.Get_Out() + sinf((PI/2) - THETA) * Target_Omega *  R_DIST * 4.0f;

            True_Vy[0] = True_Vy[1] = Slope_Velocity_Y.Get_Out() - cosf((PI/2) - THETA) * Target_Omega *  R_DIST * 4.0f;
            True_Vy[2] = True_Vy[3] = Slope_Velocity_Y.Get_Out() + cosf((PI/2) - THETA) * Target_Omega *  R_DIST * 4.0f;

            //舵轮转动角度的优化处理
            for(int i = 0;i<4;i++){
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);         //舵轮控制模式

                //计算速度
                float temp_Target_Omega = 0.0f;
                arm_sqrt_f32(True_Vx[i] * True_Vx[i] + True_Vy[i] * True_Vy[i], &temp_Target_Omega);
                temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;//从线速度到角速度，速控底盘

                //计算目标角度
                if(fabs(temp_Target_Omega) < 0.0001 && True_Vy[i] == 0.0f && True_Vx[i] == 0.0f)
                {            //避免X =0 ；Y = 0的情况
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                }
                else
                {
                    True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]); //-PI -- PI   会自动处理Vx = 0;
                }
                
                //角度优化处理
                delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();     // -2PI -- 2PI  
                delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                 // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
                if(delta_Angle > PI/2.0f)
                {
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle - PI;
                    temp_Target_Omega *= -1.0f;
                }
                else if(delta_Angle < -PI/2.0f)
                {
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle + PI;
                    temp_Target_Omega *= -1.0f;
                }
                else{
                    //不需要处理角度
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle;
                }
                
                //处理-180 - 180的突变问题    同时还有优劣弧处理
                // delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                // True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + Normalize_Angle_Radian_PI_to_PI(delta_Angle);
                
                Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
								if(i==0||i==3)
								{
                Motor_Wheel[i].Set_Target_Omega_Radian(-temp_Target_Omega);									
								}
								else if(i==1 || i==2)
								{
                Motor_Wheel[i].Set_Target_Omega_Radian(temp_Target_Omega);									
								}
								else
								{
									
								}

            }

            for(int i=0;i<4;i++)
            {   
                Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
            }
            break;
        }
    }
    #endif   
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
float Max_Power_test = 70.0f;
float Chassis_Buffer = 0.0;
float a,b,c;
void Class_Steering_Wheel_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    
    //速度逆解算
    Speed_Resolution();    
    /***************************超级电容*********************************/
    // Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max());
    // Supercap.Set_Supercap_Mode(Supercap_ENABLE);
    // Supercap.TIM_Supercap_PeriodElapsedCallback();

	
	//   if(Chassis_Control_Type == Chassis_Control_Type_DISABLE){
    //     return;         //失能情况下不跑功率限制，防止出错
    // }

    //#ifdef POWER_LIMIT


    #ifdef POWER_LIMIT_JH
    static uint8_t supercap_flag = 0;                   //超电能量低于50J的标志位
    
    //计算限制功率
    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE)
    {
        // 缓冲环限制功率
        Power_Management.Buffer_Power = 0.0f;    //Referee->Get_Chassis_Energy_Buffer() - 30.0f; 
        // Power_Management.Buffer_Power = (Referee->Get_Chassis_Energy_Buffer() - 30.0f) * 1.5f;
        Math_Constrain(&Power_Management.Buffer_Power, -50.0f, 30.0f);

        if (Supercap.Get_Supercap_Status() != Supercap_Status_DISABLE && __Sprint_Status == Sprint_Status_ENABLE)
        {
            Power_Management.Max_Power = 60.f + Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            if(Supercap.Get_Buffer_Power() <= 60.f)
            {
                Power_Management.Max_Power = Referee->Get_Chassis_Power_Max(); 
            }
        }
        else
        {
            // Power_Management.Max_Power = Power_Management.Buffer_Power + Referee->Get_Chassis_Power_Max();
            supercap_flag = 0;
            Power_Management.Max_Power = Referee->Get_Chassis_Power_Max();              //不吃缓冲能量
        }
    }
    else{
        //裁判系统离线限制功率
        Power_Management.Max_Power = 100.0f;
        Chassis_Buffer = 0.0f;
    }

    #ifdef AGV
    for (int i = 0; i < 4; i++)         //数据传递处理
    {
        //都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Wheel[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Out();

        Power_Management.Motor_Data[i + 4].feedback_omega  = Motor_Steer[i].Get_Now_Omega_Radian() * RAD_TO_RPM * Motor_Steer[i].Get_Gearbox_Rate();
        Power_Management.Motor_Data[i + 4].feedback_torque = Motor_Steer[i].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].torque          = Motor_Steer[i].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
        Power_Management.Motor_Data[i + 4].pid_output      = Motor_Steer[i].Get_Out();  
    }

    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Set_Out(Power_Management.Motor_Data[i].output);
        // //Motor_Wheel[i].Output();

        Motor_Steer[i].Set_Out(Power_Management.Motor_Data[i + 4].output);//set_out已经有output输出
      //  Motor_Steer[i].Output();
    }

    // if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
    //     Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max() + Power_Management.Buffer_Power);
    // }
    // else{
    //     Supercap.Set_Limit_Power(90.0f);
    // }

    Supercap.Set_Supercap_Mode(Supercap_ENABLE);
    //Supercap.Set_Limit_Power(Power_Management.Max_Power);               //这样子是优先使用的缓冲功率
    Supercap.Set_Limit_Power((float)Referee->Get_Chassis_Power_Max());
    Supercap.Set_Referee_Limit_Power((uint8_t)Referee->Get_Chassis_Power_Max());
    Supercap.Set_Referee_Buffer_Power(Referee->Get_Chassis_Energy_Buffer());
    Supercap.TIM_Supercap_PeriodElapsedCallback();          //向超电发送信息
    #endif

#endif	
		
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
