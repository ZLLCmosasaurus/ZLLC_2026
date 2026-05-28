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

			if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold && abs(Booster->Motor_Friction_Right.Get_Now_Torque())<=4000 )
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

        if (Status[Now_Status_Serial].Time >= 45)
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

    // 摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(100.0f, 0.0f, 0.1f, 0.0f, 3000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(100.0f, 0.0f, 0.1f, 0.0f, 3000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机下
    Motor_Friction_Down.PID_Omega.Init(100.0f, 0.0f, 0.1f, 0.0f, 3000.0f, Motor_Friction_Down.Get_Output_Max());
    Motor_Friction_Down.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);


}

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
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        // 关闭摩擦轮
        Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Left.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Friction_Right.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Friction_Down.PID_Angle.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Out(0.0f);
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Down.Set_Target_Omega_Radian(0.0f);

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
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
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
        Driver_Angle += 2.0f * PI / 9.0f;
        Motor_Driver.Set_Target_Radian(Driver_Angle);
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
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        //调整目标值与实际值相对应，以便从速度环平滑过渡到角度环
        if (Swtich_To_Angle_Control_Flag == 1)
        {
            Driver_Angle = Motor_Driver.Get_Now_Radian();

            Swtich_To_Angle_Control_Flag = 0;
        }

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
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

#ifdef Heat_Detect_ENABLE
        // ---------- 热量预测 ----------
        float net_heat_rate = 10.0f * Base_Frequency - Cooling_Value;//冷却值记得读裁判系统，还有最大热量

        if (Overheat_Flag)
        {
            τ = 0.0f;
        }
        else if (net_heat_rate > 0 && Heat_Local < Heat_Max)
        {
            τ = (Heat_Max - Heat_Local) / net_heat_rate;
        }
        else
        {
            τ = INFINITY;
        }

        // ---------- 收缩因子 ----------
        float S = 1.0f / (1.0f + exp((τ - Tau0) / Tau1));

        // ---------- 平衡频率 ----------
        Balance_Frequency = Cooling_Value / 10.0f;

        // ---------- 目标角速度 ----------
        float omega_balance = Balance_Frequency * (2.5f * 2.0f * PI / 9.0f);
        float omega_max = Base_Frequency * (2.5f * 2.0f * PI / 9.0f);

        float target_omega = omega_max - (omega_max - omega_balance) * S;
        Motor_Driver.Set_Target_Omega_Radian(target_omega);

        // ---------- 过热迟滞 ----------
        if (Heat_Local >= Heat_Max)
        {
            Overheat_Flag = true;
        }
        else if (Heat_Local <= Recover_Ratio * Heat_Max)
        {
            Overheat_Flag = false;
        }

#endif
#ifdef Heat_Detect_DISABLE
        Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);
#endif
        Swtich_To_Angle_Control_Flag = 1;
    }
    break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Down.Set_Target_Omega_Radian(Friction_Omega);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Down.Set_Target_Omega_Radian(0.0f);
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

    Heat_Max = Referee->Get_Booster_17mm_1_Heat_Max();
    Heat_Local = Referee->Get_Booster_17mm_1_Heat();
    Cooling_Value = Referee->Get_Booster_17mm_1_Heat_CD();
    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    //卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
    //Output();
    //PID输出
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Down.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
