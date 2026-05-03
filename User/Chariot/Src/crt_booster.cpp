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
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        //正常状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold)
        {
            //大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            //停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        //发射嫌疑状态

        if (Status[Now_Status_Serial].Time >= 15)
        {
            //长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        //发射完成状态->加上热量进入下一轮检测

        Heat += 10.0f;
        Set_Status(0);
    }
    break;
    case (3):
    {
        //停机状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            //开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    //热量冷却到0
    if (Heat > 0)
    {
        Heat -= 30.f / 1000.0f;//哨兵默认30
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

            if (Status[Now_Status_Serial].Time >= 300)
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
            Booster->shoot_time = 0;
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            //Booster->Driver_Angle = Booster->Motor_Driver.Get_Now_Radian() + PI / 12.0f;//原版本
            Booster->Driver_Angle = Booster->Motor_Driver.Get_Now_Radian() - (2 * PI / 27.0f);      //1/3个弹丸
            Booster->Motor_Driver.Set_Target_Radian(Booster->Driver_Angle);
            Set_Status(3);
        }
        break;
        case (3):
        {
            //卡弹处理状态

            if (Status[Now_Status_Serial].Time >= 300)
            {
                //长时间回拨->正常状态
                Set_Status(0);
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
    Motor_Driver.PID_Angle.Init(28.0f, 0.0f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Driver.PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 36.0f * 2.5f);

    //摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(80.0f, 0.0f, 0.f, 0.0f, 2000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);
    
    //摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(80.0f, 0.0f, 0.f, 0.0f, 2000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);
}

/**
 * @brief 输出到电机
 *
 */
uint32_t  booster_t=0;
float bt = 0.0f;
extern Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
void Class_Booster::Output()
{
    Now_Angle = Motor_Driver.Get_Now_Radian();

    //控制拨弹轮
    switch (Booster_Control_Type)
    {
        case (Booster_Control_Type_DISABLE):
        {
            // 发射机构失能
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            // Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            // Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 关闭摩擦轮
            Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Left.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Friction_Right.PID_Angle.Set_Integral_Error(0.0f);

            Motor_Driver.Set_Target_Torque(0.0f);
            Motor_Friction_Left.Set_Target_Torque(0.0f);
            Motor_Friction_Right.Set_Target_Torque(0.0f);

            shoot_time = 0;
        }
        break;
        case (Booster_Control_Type_CEASEFIRE):
        {
            // 停火
            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
            {
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

            if(Referee->Get_Referee_Status() == Referee_Status_ENABLE){
                if(Referee->Get_Booster_17mm_1_Heat_Max() - Referee->Get_Booster_17mm_1_Heat() < 30){
                    Driver_Angle = Now_Angle;
                }
                else{
                    Driver_Angle = Now_Angle + 2.0f * PI / 9.0f;
                }
            }
            else{
                Driver_Angle = Now_Angle + 2.0f * PI / 9.0f;
            }

            // Driver_Angle -= 2.0f * PI / 8.0f;
            Motor_Driver.Set_Target_Radian(Driver_Angle);

            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            
            //执行一次就停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        break;
        case (Booster_Control_Type_MULTI):
        {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Driver_Angle = Now_Angle + 2.0f * PI / 9.0f * 5.0f; //五连发5
            // Driver_Angle -= 2.0f * PI / 8.0f * 5.0f; //五连发
            Motor_Driver.Set_Target_Radian(Driver_Angle);

            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);

            //执行一次就停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        }
        break;
        case (Booster_Control_Type_REPEATED):
        {
            bt = DWT_GetDeltaT(&booster_t);
            // 连发模式
            
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            // 根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
            Default_Driver_Omega = Cooling_Value / Heat_Consumption / 9.0f * 2.0f * PI;
           // Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);

            if (Heat< Heat_Max * 0.95f) {        //这里和最大热量有关
            
            if (shoot_time == 0)                    //说明停火进来的
            {
                ShootTime = ((Heat_Max - Heat) + 2 * Cooling_Value) * 10;
                if (Heat_Max - Heat < 100)              //分级弹频
                {
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 2 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
                else
                {
                    shoot_speed = (10 * (Heat_Max - Heat) - Cooling_Value - 5 * Heat_Consumption) / (Heat_Consumption * (ShootTime / 100.f)) + Cooling_Value / Heat_Consumption;
                }
            }
            else if (0 < shoot_time && shoot_time < ShootTime)
            {
                Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Driver_Omega = shoot_speed * 2 * PI / 9.f;
                Math_Constrain(&Driver_Omega, 0.0f, 18.0f);
                Motor_Driver.Set_Target_Omega_Radian(Driver_Omega);
            }
           else {
            
            // 低射速用角度环控制（模拟匀速运动）
            
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            static uint8_t first = 1;
            if (first) {
                Driver_Angle = Now_Angle;   // 对齐当前实际角度
                first = 0;
            }
            if(bt>0.0021f){
                bt=0.002f;
            }
            // 计算平衡角速度 (rad/s)
            float omega_balance = ((Cooling_Value / Heat_Consumption) * 2.0f * PI / 9.0f)*bt;
            Driver_Angle += omega_balance; // 累加平衡角速度到目标角度
            Motor_Driver.Set_Target_Radian(Driver_Angle);
        }

            if (shoot_time < ShootTime)
            {
                //10 * 控制周期(s)
                shoot_time += 2;                           //注意这里应该和运算频率有关
            }
}
            // Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 2.5f);//测试用 平常注释
           else        //这里和最大热量有关
            {
                Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.4f);
            }
            // if (Heat > 340)
            // {
            //     Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega * 0.25f);
            // }

            Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
        }
        break;  
    }

    //控制摩擦轮
    if(Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(-Friction_Omega);
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
    if (Referee->Get_Referee_Status() == Referee_Status_DISABLE)
    {
        Heat_Max = 260;
        Cooling_Value = 30; // 裁判系统没反馈用默认速度
        //无需裁判系统的热量控制计算
        FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    }
    else
    {
        Heat = Referee->Get_Booster_17mm_1_Heat();
        Heat_Max = Referee->Get_Booster_17mm_1_Heat_Max();
        Cooling_Value = Referee->Get_Booster_17mm_1_Heat_CD();
    }

    //卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();
    //PID输出
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
