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

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
Enum_Friction_Control_Type Last_Friction_Control_Type = Friction_Control_Type_DISABLE;
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //static Enum_Friction_Control_Type Last_Friction_Control_Type = Friction_Control_Type_DISABLE;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态

			if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold && abs(Booster->Motor_Friction_Right.Get_Now_Torque())<=10000 )
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

        if (Status[Now_Status_Serial].Time >= 30)
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
            Last_Friction_Control_Type = Booster->Get_Friction_Control_Type();
            Set_Status(0);
        }
        else if(Last_Friction_Control_Type == Friction_Control_Type_ENABLE && Booster->Friction_Control_Type == Friction_Control_Type_DISABLE)
        {
            //Heat += 100.0f;
            Last_Friction_Control_Type = Booster->Get_Friction_Control_Type();
            Set_Status(0);
        }
        else
        {
            // 发射完成状态->加上热量进入下一轮检测
            Booster->actual_bullet_num++;
            // shoot_num ++;
            Heat += 10.0f;
            Set_Status(0);
        }
    }
    break;
    case (3):
    {
        // 停机状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
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
        Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    }
    else
    {
        Heat = 0;
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

            if (Status[Now_Status_Serial].Time >= 100)
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
            //Booster->Driver_Angle = Booster->Motor_Driver.Get_Now_Radian() + PI / 12.0f;//原版本
            Booster->Driver_Angle = Booster->Motor_Driver.Get_Now_Radian() - (2 * PI / 8.0f);
            Booster->Motor_Driver.Set_Target_Radian(Booster->Driver_Angle);
            Set_Status(3);
        }
        break;
        case (3):
        {
            static uint16_t tim1_check_cnt = 0,tim2_check_cnt = 0;
            //卡弹处理跳转正常状态
                if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
                {
                    tim1_check_cnt++;
                    tim2_check_cnt=0;
                }
                else
                {
                    //刷新时间重新计时
                    tim1_check_cnt=0;
                    //超阈值计时
                    tim2_check_cnt++;
                }

                if(tim1_check_cnt >= 200)
                {
                    //长时间回拨->正常状态
                    tim1_check_cnt = 0;
                    Set_Status(0);
                }

                //检测卡死状态跳转到失能摩擦轮状态
                if(tim2_check_cnt >= 400)
                {
                    tim2_check_cnt=0;
                    Set_Status(4);
                }
        }
        break;
        case (4):
        {
            Booster->Output();
            // 发射机构失能
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Booster->Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.Set_Out(0.0f);
            static uint16_t tim3_check_cnt = 0;
            tim3_check_cnt++;
            if(tim3_check_cnt > 1000)
            {
                if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
                {
                    Booster->Motor_Driver.Set_Target_Radian(Booster->Motor_Driver.Get_Now_Radian());
                    tim3_check_cnt = 0;
                    Set_Status(1);
                }
            }
        }   
        break;
    }
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

    //正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(4, 0);

    //拨弹盘电机
    Motor_Driver.PID_Angle.Init(58.0f, 0.1f, 0.0f, 0.0f, 2.0f * PI / 9.0f * 20, 2.0f * PI / 9.0f * 20);
    Motor_Driver.PID_Omega.Init(1800.0f, 25.0f, 0.0f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&hfdcan2, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA);

    //摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(80.0f, 0.0f, 0.f, 0.0f, 2000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);
    
    //摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(80.0f, 0.0f, 0.f, 0.0f, 2000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);


}


#ifdef OLD_CODE
/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{

#ifdef BULLET_SPEED_PID
//    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE && Referee->Get_Shoot_Speed() > 15 && (Booster_Control_Type == Booster_Control_Type_SINGLE || Booster_Control_Type == Booster_Control_Type_MULTI))
//    {
//        Bullet_Speed.Set_Now(Referee->Get_Shoot_Speed());
//        Bullet_Speed.Set_Target(Target_Bullet_Speed);
//        Bullet_Speed.TIM_Adjust_PeriodElapsedCallback();
//        //Friction_Omega += Bullet_Speed.Get_Out();
//    }
#endif
    //Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // 控制拨弹轮
    switch (Booster_Control_Type)
    {
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
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);

        shoot_time = 0;
    }
    break;
    case (Booster_Control_Type_CEASEFIRE):
    {
        // 停火
        if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
        {
            // Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
        }
        else if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_OMEGA)
        {
            Motor_Driver.Set_Target_Omega_Radian(0.0f);
        }
        shoot_time = 0;
    }
    break;
    case (Booster_Control_Type_SINGLE):
    {
        // 单发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        if (tmp_heat + 30 <= Shooter_Barrel_Heat_Limit)
        {
            Driver_Angle += 2.5f * 2.0f * PI / 9.0f;
            Motor_Driver.Set_Target_Radian(Driver_Angle);   
        }    
        // if (FSM_Heat_Detect.Heat + 20 < Referee->Get_Booster_17mm_1_Heat_Max())
        // {

        //     Driver_Angle += 2.5f * 2.0f * PI / 9.0f;
        //     Motor_Driver.Set_Target_Radian(Driver_Angle);
        // }

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_MULTI):
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Driver_Angle += 2.5f * 2.0f * PI / 9.0f * 5.0f; // 五连发  一圈的角度/一圈弹丸数*发出去的弹丸数
        Motor_Driver.Set_Target_Radian(Driver_Angle);

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_REPEATED):
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
            Default_Driver_Omega = Cooling_Value / 10.0f / 9.0f * 2.0f * PI * 2.5;
            Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);
            if (shoot_time == 0)                    //说明停火进来的
            {
                ShootTime = ((Heat_Max - Heat) + 2 * Cooling_Value) * 10;
                if (Heat_Max - Heat < 100)              //分级弹频
                {
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 3 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
                else
                {
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 5 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
            }
            else if (0 < shoot_time && shoot_time < ShootTime)
            {
                Driver_Omega = shoot_speed * 2 * PI / 9.f * 2.5;
                Math_Constrain(&Driver_Omega, 0.0f, 18.0f);
                Motor_Driver.Set_Target_Omega_Radian(Driver_Omega);
            }
            else
            {
                shoot_speed = (Cooling_Value / Heat_Consumption);
                Driver_Omega = shoot_speed * 2 * PI / 9.f * 2.5;;
                Math_Constrain(&Driver_Omega, 0.0f, 18.0f);
                Motor_Driver.Set_Target_Omega_Radian(Driver_Omega);
            }
            if (shoot_time < ShootTime)
            {
                //10 * 控制周期(s)
                shoot_time += 1;                           //注意这里应该和运算频率有关
            }
            // Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 2.5f);//测试用 平常注释
            if(Cooling_Value < 20)
            {
                if (Heat > Heat_Max * 0.90f)         //这里和最大热量有关
                {
                    Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.005f);  //爆发优先
                }
            }
            else
            {
                if (Heat > Heat_Max * 0.77f)         //这里和最大热量有关
                {
                    Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.005f);  //冷却优先
                }
            }
            // if (Heat > 340)
            // {
            //     Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.25f);
            // }
            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
    }
    break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {

        Motor_Friction_Left.Set_Target_Omega_Radian(-Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(Friction_Omega);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
    }
	
}
#endif

#define Heat_Detect_ENABLE
uint8_t Swtich_To_Angle_Control_Flag = 0;
void Class_Booster::Output()
{

#ifdef BULLET_SPEED_PID
//    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE && Referee->Get_Shoot_Speed() > 15 && (Booster_Control_Type == Booster_Control_Type_SINGLE || Booster_Control_Type == Booster_Control_Type_MULTI))
//    {
//        Bullet_Speed.Set_Now(Referee->Get_Shoot_Speed());
//        Bullet_Speed.Set_Target(Target_Bullet_Speed);
//        Bullet_Speed.TIM_Adjust_PeriodElapsedCallback();
//        //Friction_Omega += Bullet_Speed.Get_Out();
//    }
#endif

    // 控制拨弹轮
    switch (Booster_Control_Type)
    {
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
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);

    }
    break;
    case (Booster_Control_Type_CEASEFIRE):
    {
        // 停火

        if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
        {
            // Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
        }
        else if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_OMEGA)
        {
            Motor_Driver.Set_Target_Omega_Radian(0.0f);
        }
    }
    break;
    case (Booster_Control_Type_SINGLE):
    {
        // 单发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        //调整目标值与实际值相对应，以便从速度环平滑过渡到角度环
        if (Swtich_To_Angle_Control_Flag == 1)
        {
            Driver_Angle = Motor_Driver.Get_Now_Radian();

            Swtich_To_Angle_Control_Flag = 0;
        }

#ifdef Heat_Detect_ENABLE
        if (FSM_Heat_Detect.Heat + 30 < Referee->Get_Booster_17mm_1_Heat_Max())
        {

            Driver_Angle +=  2.5f * 2.0f * PI / 9.0f;
            Motor_Driver.Set_Target_Radian(Driver_Angle);
        }
        #endif
        
        #ifdef Heat_Detect_DISABLE
        Drvier_Angle += 2.0f * PI / 9.0f;
        Motor_Driver.Set_Target_Radian(Drvier_Angle);
        #endif
        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

    }
    break;
    case (Booster_Control_Type_MULTI):
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Driver_Angle += 2.5f * 2.0f * PI / 9.0f * 5.0f; // 五连发  一圈的角度/一圈弹丸数*发出去的弹丸数
        Motor_Driver.Set_Target_Radian(Driver_Angle);

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_REPEATED):
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        
        #ifdef Heat_Detect_ENABLE
        // float Tmp_Heat = 0.0f;
        // if(Referee->Get_Referee_Status() == Referee_Status_ENABLE)
        // {
        //     Tmp_Heat = Referee->Get_Booster_17mm_1_Heat_CD()
        //     Default_Driver_Omega = Referee->Get_Booster_17mm_1_Heat_CD() / 10.0f / 8.0f * 2.0f * PI;
        // }
        //根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
        Default_Driver_Omega = Referee->Get_Booster_17mm_1_Heat_CD() / 10.0f / 9.0f * 2.0f * PI * 2.5f;
        //热量控制
        if (Driver_Omega <= Default_Driver_Omega)
        {
            Motor_Driver.Set_Target_Omega_Radian(Driver_Omega);
        }
        else
        {
            float tmp_omega;
            tmp_omega = (Default_Driver_Omega - Driver_Omega) / Referee->Get_Booster_17mm_1_Heat_Max() * (FSM_Heat_Detect.Heat + 20.0f) + Driver_Omega;
            Motor_Driver.Set_Target_Omega_Radian(tmp_omega);
        }

        Swtich_To_Angle_Control_Flag = 1;

        #endif
        #ifdef Heat_Detect_DISABLE
        if (Referee->Get_Booster_17mm_1_Heat() + 30 < Referee->Get_Booster_17mm_1_Heat_Max())
        {
            Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);
        }
        else
        {
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        #endif
        }
    break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {

        Motor_Friction_Left.Set_Target_Omega_Radian(-Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(Friction_Omega);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
    }
	
}
/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{                          
    // 冷却时间获取
    // if (Referee->Get_Referee_Status() == Referee_Status_DISABLE)
    // {
    //     // Heat_Max = 88;
    //     // Cooling_Value = 14; // 裁判系统没反馈用默认速度
    //     // //无需裁判系统的热量控制计算
    //     // FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    // }
    // else
    // {
    //     Heat = Referee->Get_Booster_17mm_1_Heat();
    //     Heat_Max = Referee->Get_Booster_17mm_1_Heat_Max();
    //     Cooling_Value = Referee->Get_Booster_17mm_1_Heat_CD();
    // }

    // Heat = Referee->Get_Booster_17mm_1_Heat();
    // Heat_Max = Referee->Get_Booster_17mm_1_Heat_Max();
    // Cooling_Value = Referee->Get_Booster_17mm_1_Heat_CD();

    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    //卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
    //Output();
    //PID输出
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
