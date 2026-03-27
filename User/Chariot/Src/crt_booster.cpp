/**
 * @file crt_booster.cpp
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// int shoot_num;
int test_period = 100;
float dt_output;uint32_t cnt2;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    static Enum_Friction_Control_Type Last_Friction_Control_Type = Friction_Control_Type_DISABLE;
    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态

        if ((abs(Booster->Fric[0].Get_Now_Torque()) >= Booster->Friction_Torque_Threshold) && (abs(Booster->Fric[0].Get_Now_Torque()) < 10000.0f))
        {
            // 大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            // 停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        // 发射嫌疑状态

        if (Status[Now_Status_Serial].Time >= test_period)
        {
            // 长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        if (Last_Friction_Control_Type == Friction_Control_Type_DISABLE && Booster->Friction_Control_Type == Friction_Control_Type_ENABLE)
        {
            //Heat += 100.0f;
            Set_Status(0);
        }
        else if(Last_Friction_Control_Type == Friction_Control_Type_ENABLE && Booster->Friction_Control_Type == Friction_Control_Type_DISABLE)
        {
            //Heat += 100.0f;
            Set_Status(0);
        }
        else
        {
            // 发射完成状态->加上热量进入下一轮检测
            Booster->actual_bullet_num++;
            // shoot_num ++;
            Heat += 100.0f;
            Set_Status(0);
        }
        Last_Friction_Control_Type = Booster->Get_Friction_Control_Type();
    }
    break;
    case (3):
    {
        // 停机状态

        if (abs(Booster->Fric[0].Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            // 开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    // 热量冷却到0
    if (Heat > 0)
    {
        Heat -= Booster->Cooling_Value / 1000.0f;

    }
    else
    {
        Heat = 0;
    }

}

uint8_t Get_Shoot_Cmd(float Heat_Now, float Heat_Max)
{
    if (Heat_Now + 100 <= Heat_Max)
    {
        return 1;   // 可以射击
    }
    else
    {
        return 0;   // 不可以射击
    }
}

/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Antijamming::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0):
        {
            //正常状态
            Booster->Output();

            if (abs(Booster->Motor_Driver.Get_Now_Torque()) >= Booster->Driver_Torque_Threshold)
            {
                //大扭矩->卡弹嫌疑状态
                Set_Status(1);
            }
        }
        break;
        case (1):
        {
            //卡弹嫌疑状态
            Booster->Output();

            if (Status[Now_Status_Serial].Time >= 500)
            {
                //长时间大扭矩->卡弹反应状态
                Set_Status(2);
            }
            else if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
            {
                //短时间大扭矩->正常状态
                Set_Status(0);
            }
        }
        break;
        case (2):
        {
            //卡弹反应状态->准备卡弹处理
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Original_Angle = Booster->Motor_Driver.Get_Now_Radian();
            Booster->Driver_Angle = Original_Angle + (PI / 9.0f);//回退20度
            Booster->Motor_Driver.Set_Target_Radian(Booster->Driver_Angle);
            Set_Status(3);
        }
        break;
        case (3):
        {
            //卡弹处理状态

            if (Status[Now_Status_Serial].Time >= 100)
            {
                Booster->Driver_Angle = Original_Angle;
                Booster->Motor_Driver.Set_Target_Radian(Booster->Driver_Angle);
                Set_Status(4);
            }
        }
        break;
        case (4):
        {
            static uint8_t Torque_tim_cnt1 = 0;
			if((abs(Booster->Motor_Driver.Get_Now_Torque()) < 0.3f*Booster->Driver_Torque_Threshold))
			{
				Torque_tim_cnt1++;
				if(Torque_tim_cnt1 > 50)
				{
					Set_Status(0);
					Torque_tim_cnt1 = 0;
				}
			}
			else
			{
				Torque_tim_cnt1 = 0;
			}
					
			static uint8_t Torque_tim_cnt2 = 0;
            if((abs(Booster->Motor_Driver.Get_Now_Torque()) > Booster->Driver_Torque_Threshold))
			{
			    Torque_tim_cnt2++;
			    if(Torque_tim_cnt2 > 150)
			    {
			    	Set_Status(5);
			    	Torque_tim_cnt2 = 0;
			    }
			}
			else
			{
				Torque_tim_cnt2 = 0;
			}
			// if((abs(Booster->Motor_Driver.Get_Now_Torque()) > Booster->Driver_Torque_Threshold))
			// {
			// Torque_tim_cnt2++;
			// if(Torque_tim_cnt2 > 50)
			// {
			// 	Set_Status(2);
			// 	Torque_tim_cnt2 = 0;
			// }
			// }
			// else
			// {
			// 	Torque_tim_cnt2 = 0;
			// }
        }
        break;
        case (5):
        {
            Booster->Output();
            // 发射机构失能
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Booster->Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.Set_Out(0.0f);
            Booster->Driver_Angle = Booster->Motor_Driver.Get_Now_Radian();
            if(Status[Now_Status_Serial].Time >= 2000)
            {
                if((abs(Booster->Motor_Driver.Get_Now_Torque()) < 0.3f*Booster->Driver_Torque_Threshold))
                {
                    Set_Status(0);
                }
            }
        }
    }
}
void Class_Fric_Motor::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Rpm);
        PID_Omega.Set_Now(Data.Now_Omega_Rpm);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}
/**
 * @brief 发射机构初始化
 *
 */

void Class_Booster::Init()
{
    //正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    FSM_Heat_Detect.Booster = this;
    FSM_Heat_Detect.Init(3, 3);

    //正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态, 卡弹处理反应状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(6, 0);

    //拨弹盘电机
    // Motor_Driver.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f);
    // Motor_Driver.PID_Omega.Init(1200.0f, 20.0f, 0.0f, 0.0f, 16000.0f, 16000.0f);
    // Motor_Driver.Init(&hfdcan3, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA, 50.895f);
    //Motor_Driver.PID_Angle.Init(100.0f, 5.0f, 0.0f, 0.0f, 0.0f,0.0f);
    //Motor_Driver.PID_Omega.Init(3000.0f, 40.0f, 0.0f, 0.0f, 14000.0f,  14000.0f);
		Motor_Driver.PID_Angle.Init(100.0f, 2.0f, 4.0f, 0.0f, 0.0f,0.0f);
    Motor_Driver.PID_Omega.Init(1000.0f, 15.0f, 0.0f, 0.0f, 16000.0f,  16000.0f);
    Motor_Driver.Init(&hfdcan3, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 50.895f);
    kalman_init(&Kf_Omega,0.0f);
    //注意初始化ID 此版本6020为电流环版本 可能会有ID冲突
    #ifdef Single_Friction
    //摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(10.0f, 0.05f, 0.f, 0.0f, 2000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f); 

    //摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(10.0f, 0.05f, 0.f, 0.0f, 2000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);
    #endif
    #ifdef Double_Friction
    //4*摩擦轮初始化
    Fric[0].Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[0].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,13000.0f);

    Fric[1].Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[1].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,13000.0f);

    Fric[2].Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[2].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,13000.0f);

    Fric[3].Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA, 1.0f);
    Fric[3].PID_Omega.Init(10.0f, 0.05f, 0.0f, 0.0f, 2000.0f,13000.0f);
    #endif

}

/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    dt_output = DWT_GetDeltaT(&cnt2);
    Now_Angle = Motor_Driver.Get_Now_Radian();
    //Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    //控制拨弹轮
    switch (Booster_Control_Type)
    {
        #ifdef Single_Friction
        case (Booster_Control_Type_DISABLE):
        {
            // 发射机构失能
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 关闭摩擦轮
            Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Left.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Friction_Right.PID_Angle.Set_Integral_Error(0.0f);

            Motor_Driver.Set_Out(0.0f);
            Motor_Friction_Left.Set_Out(0.0f);
            Motor_Friction_Right.Set_Out(0.0f);

            shoot_time = 0;
        }
        break;
        case (Booster_Control_Type_CEASEFIRE):
        {
            
            // 停火
            if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
            {
               // Motor_Driver.Set_Target_Radian(Motor_Driver.Get_Now_Radian());
            }
            else if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_OMEGA)
            {
                Motor_Driver.Set_Target_Omega_Radian(0.0f);
                Motor_Driver.Set_Out(0.f);               
            }
            else
            {
                Motor_Driver.Set_Out(0.f);
            }
            shoot_time = 0;
            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        break;
        case (Booster_Control_Type_SINGLE):
        {
            // 单发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Driver_Angle = Now_Angle - 2.0f * PI / 8.0f;
            // Driver_Angle -= 2.0f * PI / 8.0f;
            Motor_Driver.Set_Target_Radian(Driver_Angle);

            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
        }
        break;
        case (Booster_Control_Type_MULTI):
        {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Driver_Angle = Now_Angle - 2.0f * PI / 8.0f * 5.0f; //五连发5
            // Driver_Angle -= 2.0f * PI / 8.0f * 5.0f; //五连发
            Motor_Driver.Set_Target_Radian(Driver_Angle);

            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
        }
        break;
        case (Booster_Control_Type_REPEATED):
        {
            float max_speed = 30.f;
            float speed_x = fabs((float)(MiniPC->Get_Chassis_Target_Velocity_X() / 100.f));
            float speed_y = fabs((float)(MiniPC->Get_Chassis_Target_Velocity_Y() / 100.f));
            float max_sum = sqrt(speed_x * speed_x + speed_y * speed_y);

            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
            Default_Driver_Omega = - 80.f / 10.0f / 8.0f * 2.0f * PI;
            Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);
            if(max_sum > 0.f && max_sum <= 0.75f)
            {
                max_speed = 30.f;
            }
            else if(max_sum > 0.75f && max_sum <= 4.f)
            {
                max_speed = 8.f;
            }
            else if(max_sum > 4.f && max_sum <= 9.f)
            {
                max_speed = 4.f;
            }
            else if(max_sum > 9.f && max_sum <= 16.f)
            {
                max_speed = 2.f;
            }
            else if(max_sum > 16.f)
            {
                max_speed = 1.f;
            }
            // 热量控制
            Cooling_Value = CAN3_Chassis_Rx_Data_B.cooling_value;
            if(Heat == 0 && (uint16_t)FSM_Heat_Detect.Heat != 0)
            {
                Heat = (uint16_t)FSM_Heat_Detect.Heat;
            }
            if(shoot_time == 0)
            {
                ShootTime = ((Heat_Max - Heat) + 2 * Cooling_Value) * 10;
                if(Heat_Max - Heat < 100){
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 3 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
                else{
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 5 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
            }               
            else if(0 < shoot_time && shoot_time < ShootTime)
            {
                Driver_Omega = shoot_speed * 2 * PI / 7.f;
                Math_Constrain(&Driver_Omega, 0.0f, max_speed);
                Motor_Driver.Set_Target_Omega_Radian(-Driver_Omega);
            }
            else
            {
                shoot_speed = (Cooling_Value / Heat_Consumption);
                Driver_Omega = shoot_speed * 2 * PI / 7.f;
                Math_Constrain(&Driver_Omega, 0.0f, max_speed);
                Motor_Driver.Set_Target_Omega_Radian(-Driver_Omega);
            }
            if(shoot_time < ShootTime)
            {
                shoot_time++;
            }
            //Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 2.5f);//测试用 平常注释
            //裁判系统正常模式
            if(Heat > 340)
            {
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.4f);
            }
            if(Heat > 370)
            {
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.25f);
            }
            //离线保守模式
            if(Heat > 300)
            {
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.4f);
            }
            if(Heat > 350)
            {
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.25f);
            }
            
            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
        }
        break;
        #endif  
        #ifdef Double_Friction
        case (Booster_Control_Type_DISABLE):
        {
            // 发射机构失能
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Driver.Set_Out(0.0f);

            Driver_Angle = Motor_Driver.Get_Now_Radian();

            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
                Fric[i].Set_Target_Torque(0.0f);
            }

            // 关闭摩擦轮
            Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
        }
        break;
        case (Booster_Control_Type_CEASEFIRE):
        {
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Driver.Set_Target_Radian(Driver_Angle);

            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }
        }
        break;
        case (Booster_Control_Type_SINGLE):
        {
            // 单发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            for (auto i = 0; i < 4; i++)
            {
                Fric[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }

            Driver_Angle -= 2.0f * PI / 6.0f;

            Motor_Driver.Set_Target_Radian(Driver_Angle);

            // 点一发立刻停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        break;
        #endif
    }
    //Friction_Control_Type = Enum_Friction_Control_Type(Shoot_Flag);调试用
    //控制摩擦轮
    #ifdef Single_Friction
    if(Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Motor_Friction_Left.Set_Target_Omega_Rpm(-Fric_High_Rpm);
        Motor_Friction_Right.Set_Target_Omega_Rpm(Fric_High_Rpm);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Rpm(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Rpm(0.0f);
    }
    #endif
    #ifdef Double_Friction
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Fric[0].Set_Target_Omega_Rpm(Fric_High_Rpm + Fric_Transform_Rpm);
        Fric[1].Set_Target_Omega_Rpm(Fric_Low_Rpm + Fric_Transform_Rpm);
        Fric[2].Set_Target_Omega_Rpm(-(Fric_Low_Rpm + Fric_Transform_Rpm));
        Fric[3].Set_Target_Omega_Rpm(-(Fric_High_Rpm + Fric_Transform_Rpm));
    }
    else
    {
        Fric[0].Set_Target_Omega_Rpm(0);
        Fric[1].Set_Target_Omega_Rpm(0);
        Fric[2].Set_Target_Omega_Rpm(0);
        Fric[3].Set_Target_Omega_Rpm(0);
    }
    #endif
}

/**
 * @brief 自动弹速调参
 * 
 */
void Class_Booster::TIM_Adjust_Bullet_Velocity_PeriodElapsedCallback()
{
    //当裁判系统正常回传弹速数据时
        if (Referee_Bullet_Velocity != Pre_Referee_Bullet_Velocity)
        {
            Referee_Bullet_Velocity_Updata_Status = Referee_Bullet_Velocity_Updata_Status_ENABLE;
        }
        else
        {
            Referee_Bullet_Velocity_Updata_Status = Referee_Bullet_Velocity_Updata_Status_DISABLE;
        }
    //只在此状态下进行
        switch (Referee_Bullet_Velocity_Updata_Status)
        {
        case Referee_Bullet_Velocity_Updata_Status_ENABLE:
        {
            if (fabs(Referee_Bullet_Velocity - Pre_Referee_Bullet_Velocity) <= 0.3f)
            {
                if (Referee_Bullet_Velocity >= 12.0f)
                {
                    Fric_Transform_Rpm -= (int16_t)(150.0f * fabs(Referee_Bullet_Velocity - 12.0f));
                }
                else if (Referee_Bullet_Velocity >= 11.85f && Referee_Bullet_Velocity < 12.0f)
                {
                    Fric_Transform_Rpm -= (int16_t)(50.0f * fabs(Referee_Bullet_Velocity - 11.85f));
                }
                else if (Referee_Bullet_Velocity <= 11.60f)
                {
                    Fric_Transform_Rpm += (int16_t)(50.0f * fabs(Referee_Bullet_Velocity - 11.65f));
                }
            }
        }
        break;
        default:
        {
            //不做处理
        }
        break;
        }

        Pre_Referee_Bullet_Velocity = Referee_Bullet_Velocity;
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{     
    
    //无需裁判系统的热量控制计算
    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    //卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
    //弹速调整
    TIM_Adjust_Bullet_Velocity_PeriodElapsedCallback();
	// Output();
     kalman_update(&Kf_Omega,Motor_Driver.Get_Now_Omega_Radian());
    //PID输出
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    #ifdef Single_Friction
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
    #endif
    #ifdef Double_Friction
    for (auto i = 0; i < 4; i++)
    {
        Fric[i].TIM_PID_PeriodElapsedCallback();
    }
    #endif

    if(Referee->Get_Shoot_Speed()>10.0f && Referee->Get_Shoot_Speed()<16.0f)
    {
        Speed = Referee->Get_Shoot_Speed();
    }


    Cmd_if_Fire = Get_Shoot_Cmd(FSM_Heat_Detect.Heat,100);
    

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
