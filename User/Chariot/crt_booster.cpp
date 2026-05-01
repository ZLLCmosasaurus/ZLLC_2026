/**
 * @file crt_booster.cpp
 * @author lez by wanghongxi
 * @brief 发射机构
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

#include "drv_math.h"

/* Private macros ------------------------------------------------------------*/
#define Heat_Detect_ENABLE
// #define Heat_Detect_DISABLE
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 根据拨弹盘角度变化计算发弹量与热量
 *
 */
Enum_Friction_Control_Type Last_Friction_Control_Type =
    Friction_Control_Type_DISABLE;
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    // 每发弹丸对应拨弹盘的运动位移角度
    constexpr float ANGLE_PER_BULLET = 40.0f;

    // 当拨弹盘走过位移角度的多少时，便判定为发弹
    constexpr float SHOOT_JUDGE_RATIO = 0.9f;

    // 是否启用依靠裁判系统数据的同步机制，以消除多算或少算发弹的误差
    constexpr bool IS_SHADOW_SYNCHRONIZATION_ENABLE = true;

    // 计算得到的热量比裁判系统低时，本地热量追赶的速度系数
    constexpr float HEAT_CHASE_FACTOR = 0.05f;

    // 计算得到的热量比裁判系统高时，本地热量等待的速度系数
    constexpr float HEAT_WAIT_FACTOR = 0.005f;

    // 拨弹盘初始角度偏移量
    static float ANGLE_OFFSET = 0.0f;

    // 初始化判断标志位
    static bool is_initialized = false;

    if (!is_initialized) {
        // 初始化拨弹盘角度偏移量
        ANGLE_OFFSET   = Booster->Motor_Driver.Get_Now_Angle();
        is_initialized = true;
    }

    // 当前拨弹盘总位移角度
    float total_displacement = Booster->Motor_Driver.Get_Now_Angle() - ANGLE_OFFSET;

    // 当下发弹量对应的拨弹盘位移
    float expected_displacement = Booster->actual_bullet_num * ANGLE_PER_BULLET;

    // 发弹判断
    float delta = total_displacement - expected_displacement;
    if (delta > ANGLE_PER_BULLET * SHOOT_JUDGE_RATIO) {
        Booster->actual_bullet_num++;
        Heat += 10.0f;
    }

    uint16_t referee_heat = Booster->Referee->Get_Booster_17mm_1_Heat();

    if (IS_SHADOW_SYNCHRONIZATION_ENABLE) {
        // 与裁判系统值比对，修正误差
        float error = referee_heat - Heat;
        if (error > 0) {
            // 补偿少算
            Heat += HEAT_CHASE_FACTOR * error;
        } else if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_DISABLE ||
                   Booster->Get_Booster_Control_Type() == Booster_Control_Type_CEASEFIRE) {
            // 当停止发射时才补偿多算
            Heat += HEAT_WAIT_FACTOR * error;
        }
    }

    // 热量冷却到0
    if (Heat > 0) {
        Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    } else {
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

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial) {
        case (0): {
            // 正常状态
            Booster->Output();

            if (abs(Booster->Motor_Driver.Get_Now_Torque()) >=
                Booster->Driver_Torque_Threshold) {
                // 大扭矩->卡弹嫌疑状态
                Set_Status(1);
            }
        } break;
        case (1): {
            // 卡弹嫌疑状态
            Booster->Output();

            if (Status[Now_Status_Serial].Time >= 100) {
                // 长时间大扭矩->卡弹反应状态
                Set_Status(2);
            } else if (abs(Booster->Motor_Driver.Get_Now_Torque()) <
                       Booster->Driver_Torque_Threshold) {
                // 短时间大扭矩->正常状态
                Set_Status(0);
            }
        } break;
        case (2): {
            // 卡弹反应状态->准备卡弹处理
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_ANGLE);
            Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() - PI / 12.0f;
            Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
            Set_Status(3);
        } break;
        case (3): {
            // 卡弹处理状态

            // if (Status[Now_Status_Serial].Time >= 300)
            // {
            //     // 长时间回拨->正常状态
            //     Set_Status(0);
            // }

            static uint16_t cnt1 = 0, cnt2 = 0;
            if (abs(Booster->Motor_Driver.Get_Now_Torque()) <
                Booster->Driver_Torque_Threshold) {
                cnt1++;
                cnt2 = 0;
                if (cnt1 >= 200) {
                    // 长时间回拨->正常状态
                    cnt1 = 0;
                    Set_Status(0);
                }
            } else {
                cnt1 = 0;
                cnt2++;
                if (cnt2 >= 200) {
                    cnt2 = 0;
                    Set_Status(4);
                }
            }

        } break;
        case (4): {
            Booster->Output();

            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OPENLOOP);

            Booster->Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Booster->Motor_Driver.Set_Out(0.0f);
            Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian();

            static uint32_t last_time_alive = 0;
            uint32_t time_alive             = HAL_GetTick();

            if (Status[Now_Status_Serial].Time == 1) {
                last_time_alive = time_alive;
            }

            if (time_alive - last_time_alive > 2000) {
                last_time_alive = time_alive;
                Set_Status(0);
            }
        } break;
    }
}

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    // 正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    FSM_Heat_Detect.Booster = this;
    FSM_Heat_Detect.Init(3, 3);

    // 正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(5, 0);

    // 拨弹盘电机
    Motor_Driver.PID_Angle.Init(40.0f, 0.1f, 0.0f, 0.0f, Default_Driver_Omega,
                                Default_Driver_Omega);
    Motor_Driver.PID_Omega.Init(6000.0f, 40.0f, 0.0f, 0.0f,
                                Motor_Driver.Get_Output_Max(),
                                Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&hcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA,
                      90);

    // 摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(150.0f, 4.0f, 0.2f, 0.0f, 2000.0f,
                                       Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.Init(&hcan1, DJI_Motor_ID_0x202,
                             DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(150.0f, 4.0f, 0.2f, 0.0f, 2000.0f,
                                        Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.Init(&hcan1, DJI_Motor_ID_0x201,
                              DJI_Motor_Control_Method_OMEGA, 1.0f);

#ifdef BULLET_SPEED_PID
    Bullet_Speed.Init(3.0, 0, 0, 0, 10, 200, 0, 0, 0, 0.001, 0.3);
#endif
}

/**
 * @brief 输出到电机
 *
 */
uint8_t Swtich_To_Angle_Control_Flag = 0;
void Class_Booster::Output()
{

#ifdef BULLET_SPEED_PID
//    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE &&
//    Referee->Get_Shoot_Speed() > 15 && (Booster_Control_Type ==
//    Booster_Control_Type_SINGLE || Booster_Control_Type ==
//    Booster_Control_Type_MULTI))
//    {
//        Bullet_Speed.Set_Now(Referee->Get_Shoot_Speed());
//        Bullet_Speed.Set_Target(Target_Bullet_Speed);
//        Bullet_Speed.TIM_Adjust_PeriodElapsedCallback();
//        //Friction_Omega += Bullet_Speed.Get_Out();
//    }
#endif

    // 控制拨弹轮
    switch (Booster_Control_Type) {
        case (Booster_Control_Type_DISABLE): {
            // 发射机构失能
            Motor_Driver.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OPENLOOP);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);

            // 关闭摩擦轮
            Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Left.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Friction_Right.PID_Angle.Set_Integral_Error(0.0f);

            Motor_Driver.Set_Out(0.0f);
            Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
            Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);

        } break;
        case (Booster_Control_Type_CEASEFIRE): {
            // 停火

            if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE) {
                // Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
            } else if (Motor_Driver.Get_Control_Method() ==
                       DJI_Motor_Control_Method_OMEGA) {
                Motor_Driver.Set_Target_Omega_Radian(0.0f);
                Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
                Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
                Motor_Driver.Set_Out(0.0f);
            }
        } break;
        case (Booster_Control_Type_SINGLE): {
            // 单发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);

            // 调整目标值与实际值相对应，以便从速度环平滑过渡到角度环
            if (Swtich_To_Angle_Control_Flag == 1) {
                Drvier_Angle = Motor_Driver.Get_Now_Radian();

                Swtich_To_Angle_Control_Flag = 0;
            }

#ifdef Heat_Detect_ENABLE
            if (FSM_Heat_Detect.Heat + 20 < Referee->Get_Booster_17mm_1_Heat_Max()) {

                Drvier_Angle += 2.0f * PI / 9.0f;
                Motor_Driver.Set_Target_Radian(Drvier_Angle);
            }
#endif

#ifdef Heat_Detect_DISABLE
            Drvier_Angle += 2.0f * PI / 9.0f;
            Motor_Driver.Set_Target_Radian(Drvier_Angle);
#endif
            // 点一发立刻停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

        } break;
        case (Booster_Control_Type_MULTI): {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);

            Drvier_Angle +=
                2.0f * PI / 9.0f * 5.0f; // 五连发  一圈的角度/一圈弹丸数*发出去的弹丸数
            Motor_Driver.Set_Target_Radian(Drvier_Angle);

            // 点一发立刻停火
            Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
        } break;
        case (Booster_Control_Type_REPEATED): {
            // 连发模式
            Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Left.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);
            Motor_Friction_Right.Set_DJI_Motor_Control_Method(
                DJI_Motor_Control_Method_OMEGA);

#ifdef Heat_Detect_ENABLE
            // 根据冷却计算拨弹盘默认速度, 此速度下与冷却均衡
            Default_Driver_Omega =
                Referee->Get_Booster_17mm_1_Heat_CD() / 10.0f / 9.0f * 2.0f * PI;
            // 热量控制
            // 机器人冷却(单位：点/s)
            float a = static_cast<float>(Referee->Get_Booster_17mm_1_Heat_CD());
            // 枪口还能利用的热量上限(单位：点)
            float m = static_cast<float>(Referee->Get_Booster_17mm_1_Heat_Max() - FSM_Heat_Detect.Heat);
            // float m = static_cast<float>(Referee->Get_Booster_17mm_1_Heat_Max() - Referee->Get_Booster_17mm_1_Heat());
            // 每射击一发消耗热量(单位：点)
            constexpr float d = 10.0f;
            // 射速(单位：发/s)
            static float shoot_speed = 0.0f;

            // 本周期射击的总持续时间(单位：ms)
            static uint16_t ShootTime = 0;
            // 本周期射击的实际时间(单位：ms)
            static uint16_t shoot_time      = 0;
            static uint16_t last_shoot_time = 0;

            // 最终得出的拨弹盘转速
            float target_omega = 0.0f;
            // 射速与拨弹盘转速的转换关系
            constexpr float rad_per_bullet = 2.0f * PI / 9.0f; // 假设一圈9发

            if (m >= 100) {
                // 热量充裕时
                target_omega = Driver_Omega;
                shoot_time   = 0;
            } else if (m >= 30 && m <= 100) {
                if (shoot_time == 0) {
                    /**
                     * @brief 决定本次射击周期要持续多久
                     *
                     * @param (m + k * a) 要经过多少个裁判系统解算周期 按照同济大学的解释如下：
                     * > 根据热量上限和冷却决定射击策略，计算得当射击时间为m（热量上限）+1*a（冷却速率）时基本可以抹除冷却优先和爆发优先的差距，即两者各级对应射速相近
                     * > 当k增大时，差距射击频率差距主要体现在低等级（爆发高，冷却低），等级越高影响越小。爆发模式下各等级射频更加均匀且持续时间更长，
                     * > 冷却模式正好相反，低等级射频低，高等级射频高且持续时间短，可灵活选择m+k*a
                     * 详见此链接：https://bbs.robomaster.com/article/630409
                     *
                     * 最终乘以100是为了将单位统一为ms，因为此函数的执行周期为1ms，而裁判系统的结算频率为10Hz。
                     */
                    ShootTime = (m + 2 * a) * 100;
                    Math_Constrain<uint16_t>(&ShootTime, 1000, 5600);
                    // 分级射速
                    if (m < 50) {
                        constexpr float kNormalFactor = 3.0f;

                        shoot_speed = (d * m - a - kNormalFactor * d) /
                                          (d * ShootTime / 1000.0f) +
                                      a / d;
                    } else {
                        constexpr float kBurstFactor = 7.0f;

                        shoot_speed = (d * m - a - kBurstFactor * d) /
                                          (d * ShootTime / 1000.0f) +
                                      a / d;
                    }
                } else if (0 < shoot_time && shoot_time < ShootTime) {
                    // 在射击周期内，按原计划射弹
                    target_omega = shoot_speed * rad_per_bullet;
                    Math_Constrain<float>(&target_omega, 0.0f, 18.0f);
                } else {
                    // 超出了射击周期，贴近冷却回复速度射弹
                    target_omega = rad_per_bullet * a / d;
                    if (target_omega < 1.0f * rad_per_bullet) {
                        target_omega = 0.0f;
                    }
                    Math_Constrain<float>(&target_omega, 0.0f, 18.0f);
                }
                if (shoot_time < ShootTime) {
                    shoot_time++;
                }
                last_shoot_time = ShootTime;
                if (m >= 40) {
                    // 射击周期结束，热量还有余裕，重新进入下一个射击周期
                    if (shoot_time >= ShootTime) {
                        shoot_time = 0;
                    }
                } else if (m <= 32) {
                    // 热量不足，强制进入下一个射击周期
                    shoot_time = last_shoot_time;
                }
            } else if (m <= 30) {
                // 热量不足，强制停机
                target_omega = 0.0f;
                shoot_time   = 0;
            }

            Motor_Driver.Set_Target_Omega_Radian(target_omega);
            Swtich_To_Angle_Control_Flag = 1;

#endif
#ifdef Heat_Detect_DISABLE
            if (Referee->Get_Booster_17mm_1_Heat() + 30 <
                Referee->Get_Booster_17mm_1_Heat_Max()) {
                Motor_Driver.Set_Target_Omega_Radian(Default_Driver_Omega);
            } else {
                Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
            }
#endif
        } break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE) {

        Motor_Friction_Left.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(-Friction_Omega);
    } else {
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

    // 无需裁判系统的热量控制计算
    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    // 卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();

    // Output();

    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER
 * **************************/
