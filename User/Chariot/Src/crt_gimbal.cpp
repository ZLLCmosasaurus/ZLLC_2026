/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief 对于电机角度控制时的突变点处理
 * @param Target_Angle 
 * @param Now_Angle 
 */
void Angle_Continuity_Process(float* Target_Angle, float Now_Angle){
    float Diff_Angle = *Target_Angle - Now_Angle;
    while (Diff_Angle > 180.0f)
    {
        *Target_Angle -= (2 * 180.0f);
        Diff_Angle = *Target_Angle - Now_Angle;
    }
    while (Diff_Angle < -180.0f)
    {
        *Target_Angle += (2 * 180.0f);
        Diff_Angle = *Target_Angle - Now_Angle;
    }
}

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    External_IMU.Init(7.9999998e-05);

    // main电机
    Motor_Main_Yaw.PID_Angle.Init(0.28f, 0.0f, 0.0004f, 0.0f, 3, 15);
    Motor_Main_Yaw.PID_Omega.Init(2200.0f, 0.0f, 0.0f, 0.0f, 400.0f, 2048.0f);
    Motor_Main_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Main_Yaw.Get_Output_Max(), Motor_Main_Yaw.Get_Output_Max());            //这样一直输出的都是0，失能
    Motor_Main_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, LK_Motor_Control_Method_ANGLE, MAIN_YAW_ENCODER_OFFSET);

    // yaw轴电机  0.6876f
    Motor_Yaw.PID_Angle.Init(0.65f, 0.0f, 0.0f, 0.012f, 10, 10);
    //Kp给大容易因为大小Yaw联动的噪声出问题，达不到理想的想要，用大Ki补偿误差，还有Ki对抖动不敏感（积分，相位延迟）强制补偿掉，也可以尝试LESO，但他可能对噪声敏感一些（重在抗扰动）
    //Ki太大对阶跃信号抖动滞后，不用了
    Motor_Yaw.PID_Omega.Init(6000.0f, 7000.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());

    Motor_Yaw.SMC_Control.Init(0.005, 85.0, 85.0, 5.0);

    Motor_Yaw.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, YAW_ENCODER_OFFSET);
    
    // pitch轴电机
    // Motor_Pitch.PID_Angle.Init(0.70f, 0.0f, 0.00f, 0.0f, 10.0f, 10.0f);
    // Motor_Pitch.PID_Omega.Init(120.0f, 0.0f, 0.0f, 0.0f, 2048.0f, 2048.0f);
    Motor_Pitch.PID_Angle.Init(0.46f, 0.0f, 0.0003f, 0.0f, 7.0f, 7.0f);
    Motor_Pitch.PID_Omega.Init(9000.0f, 3200.0f, 0.0f, 0.0f, 16384.0f, 16384.0f);
    // Motor_Pitch.PID_Angle.Init(0.55f, 0.0f, 0.0017, 0.0f, 7.0f, 7.0f);
    // Motor_Pitch.PID_Omega.Init(7000.0f, 5000.0f, 0.0f, 0.0f, 16384.0f, 16384.0f);
    // Motor_Pitch.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    // Motor_Pitch.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 2048.0f, 2048.0f);
    // Motor_Pitch.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_TORQUE, 0, 20.94359f, 3.0f);
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE);
    Motor_Pitch_LESO.Init(0.08, 20.0, 1.0, Observe_Motor_Omega, Motor_GM6020, 0.002f);

    External_IMU_Gyro_Yaw.Init(0.99,0.07);
    External_IMU_Gyro_Pitch.Init(0.99,0.07);
    Motor_Yaw_Angle_Filter.Init(0.34, 0.34);
}


/**
 * @brief 输出到电机
 *
 */
float tmp_Target_Angle = 0.0f, tmp_Target_Pitch_Angle = 0.0f, test_c = -1800;
extern float Sin_Single;
void Class_Gimbal::Output()
{
    static uint32_t camera_switch_clpt_time = 0;
    static uint32_t camera_switch_time = 0;
    static uint8_t camera_switch_flag = 0, camera_switch_clpt_flag = 0;
    static uint8_t Curise_Flag = 0;                   //当前模式是否是巡航，是否处于全向感知标志位
    static float pre_yaw_angle = 0.0f, pre_pitch_angle = 0.0f, pre_main_yaw_angle = 0.0f;

    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);
        Motor_Main_Yaw.Set_Target_Torque(0.0f);

        Curise_Flag = 0;
        camera_switch_flag = 0;
        camera_switch_time = 0;
        camera_switch_clpt_flag = 0;
        camera_switch_clpt_time = 0;
    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Target_Yaw_Angle = tmp_Target_Angle;
            // Target_Pitch_Angle = tmp_Target_Pitch_Angle;//会和dr16的遥控器输入冲突
            //对于大Yaw控制的突变点与优劣弧处理       0--2*PI
            Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());
            Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 设置目标角度    Motor_Yaw的角度是以偏置零点为原点，改Encoder_offset实现校准
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);                       //可能可以加前馈
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

            Curise_Flag        = 0;
            camera_switch_flag = 0;
            camera_switch_time = 0;
            camera_switch_clpt_flag = 0;
            camera_switch_clpt_time = 0;
            pre_yaw_angle      = 0.0f;
            pre_pitch_angle    = Motor_Pitch.Get_Transform_Angle();
            pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {//0是海康,4是每瞄到,当前不处于切换大小yaw并且间隔超过500 * 控制频率ms
            if(MiniPC->Get_Camera_Id() != 0 && MiniPC->Get_Camera_Id() != 4 && camera_switch_flag == 0 && camera_switch_time > 500){
                Curise_Flag        = 0;                     //保证从巡航切进来后能正确进入开启巡航
                camera_switch_time = 0;
                camera_switch_flag = 1;
                camera_switch_clpt_time = 0;
                camera_switch_clpt_flag = 0;

                Target_Yaw_Angle      = 0.0f;
                Target_Pitch_Angle    = 0.0f;
                Target_Main_Yaw_Angle = Boardc_BMI.Get_Angle_Yaw() + MiniPC->Get_Camera_Id() * 90.0f;

                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            }

            if(camera_switch_flag){
                Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());
                Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());

                Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);                       //可能可以加前馈
                Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
                Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

                if(fabs(Motor_Main_Yaw.Get_Transform_Angle() - Target_Main_Yaw_Angle) < 1.0f){
                    camera_switch_flag = 0;             //清空状态
                    camera_switch_clpt_time = 0;                    //初始进入置零
                    camera_switch_clpt_flag = 1;                    //标志刚完成全向感知
                }

                pre_yaw_angle      = 0.0f;
                pre_pitch_angle    = 0.0f;
                pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();

                return;
            }

            camera_switch_time ++;              //记录距离上一次全向感知执行完成的时间间隔

            if(MiniPC->Get_Auto_aim_Status() == Auto_aim_Status_DISABLE){               //巡航模式
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                
                if(!Curise_Flag){
                    Motor_Yaw.Set_Target_Omega_Angle(CRUISE_YAW_SPEED);
                    Motor_Pitch.Set_Target_Omega_Angle(CRUISE_PITCH_SPEED);
                    Curise_Flag = 1;
                }

                if(Motor_Yaw.Get_Zero_Offset_Angle() < -LIMIT_YAW_ANGLE){
                    Motor_Yaw.Set_Target_Omega_Angle(CRUISE_YAW_SPEED);
                }
                else if(Motor_Yaw.Get_Zero_Offset_Angle() > LIMIT_YAW_ANGLE){
                    Motor_Yaw.Set_Target_Omega_Angle(-CRUISE_YAW_SPEED);
                }

                if(Motor_Pitch.Get_Transform_Angle() < -5.0f){
                    Motor_Pitch.Set_Target_Omega_Angle(CRUISE_PITCH_SPEED);
                }
                else if(Motor_Pitch.Get_Transform_Angle() > 20.0f){
                    Motor_Pitch.Set_Target_Omega_Angle(-CRUISE_PITCH_SPEED);
                }

                // Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                // Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                // Target_Yaw_Angle = 0.0f;
                // Target_Pitch_Angle = 0.0f;//会和dr16的遥控器输入冲突
                //对于Yaw控制的突变点与优劣弧处理       0--2*PI
                // Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

                // // 限制角度
                // Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

                // Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);                       //可能可以加前馈
                // Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

                //更新历史值
                pre_yaw_angle = Motor_Yaw.Get_Zero_Offset_Angle();
                pre_pitch_angle = Motor_Pitch.Get_Transform_Angle();
            }
            else{
                Curise_Flag = 0;

                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                //这是处于间接识别到的阶段（识别不稳定或者装甲板在闪烁）
                //也就是只要识别到一次目标，就会切换到自瞄模式，至少持续0.5s才会退出，这0.5s是为识别不稳定留下的空间
                if(MiniPC->Get_Rx_Yaw_Angle() == 0.0f && MiniPC->Get_Rx_Pitch_Angle() == 0.0f)
                {
                    //这里并不是上一个目标点而是丢失目标后的最后当前点
                    Target_Yaw_Angle   = pre_yaw_angle;
                    Target_Pitch_Angle = pre_pitch_angle;

                    Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

                    Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
                    Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
                }
                else
                {
                    float MiniPC_Target_Yaw    = MiniPC->Get_Rx_Yaw_Angle();
                    float MiniPC_Target_Pitch  = MiniPC->Get_Rx_Pitch_Angle();

                    Target_Yaw_Angle   = MiniPC_Target_Yaw - Motor_Main_Yaw.Get_Transform_Angle();
                    Target_Pitch_Angle = MiniPC_Target_Pitch;

                    //怕超出限位到死区
                    Math_Constrain(&Target_Yaw_Angle, -LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);
                    Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

                    Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

                    Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
                    Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

                    pre_yaw_angle = Motor_Yaw.Get_Zero_Offset_Angle();
                    pre_pitch_angle = Motor_Pitch.Get_Transform_Angle();
                }
            }

            if(camera_switch_clpt_flag){
                camera_switch_clpt_time ++;
                if(camera_switch_clpt_time > 1500){
                    camera_switch_clpt_flag = 0;
                }
                
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                Target_Main_Yaw_Angle = pre_main_yaw_angle;
                Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());

                Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);
                return;
            }
            else{
                camera_switch_clpt_time = 0;
            }

            // 大yaw控制逻辑   由上位机控制是否转动
            if(fabs(MiniPC->Get_Rx_Target_Omega_Yaw_Main()) < 0.01f){
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);

                Target_Main_Yaw_Angle = pre_main_yaw_angle;
                Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());

                Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);
                Motor_Main_Yaw.Set_Target_Omega_Angle(0.0f);
                
            }
            else{
                Target_Main_Yaw_Angle = Boardc_BMI.Get_Angle_Yaw();                  //角度一直更新防止切回手动控制Target还是上一次的数据
                Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_OMEGA);
                Motor_Main_Yaw.Set_Target_Omega_Angle(MiniPC->Get_Rx_Target_Omega_Yaw_Main());     //实际上单位是rad
                pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();                                   //不能像其他变量一样一直更新，不然目标角度会一直变化，会有问题         
            }

        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            Curise_Flag = 0;
            camera_switch_flag = 0;
            camera_switch_time = 0;
            camera_switch_clpt_flag = 0;
            camera_switch_clpt_time = 0;

            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);

            Target_Yaw_Angle      = pre_yaw_angle;
            Target_Pitch_Angle    = pre_pitch_angle;
            Target_Main_Yaw_Angle = pre_main_yaw_angle;
            Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());
            Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

            // 限制角度
            Math_Constrain(&Target_Yaw_Angle,-LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);            
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

        }
    }
}

void Class_Gimbal::Output_Test()
{
    static float pre_yaw_angle = 0.0f, pre_pitch_angle = 0.0f, pre_main_yaw_angle = 0.0f;

    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_TORQUE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);

        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Main_Yaw.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);
        Motor_Main_Yaw.Set_Target_Torque(0.0f);
    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Target_Yaw_Angle = tmp_Target_Angle;
            // Target_Pitch_Angle = tmp_Target_Pitch_Angle;//会和dr16的遥控器输入冲突
            //对于大Yaw控制的突变点与优劣弧处理       0--2*PI
            Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());
            Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 设置目标角度    Motor_Yaw的角度是以偏置零点为原点，改Encoder_offset实现校准
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);                       //可能可以加前馈
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

            Motor_Yaw.Set_Transform_Target_Vel(0.0f);
            Motor_Yaw.Set_Transform_Target_Acc(0.0f);
            Motor_Pitch.Set_Transform_Target_Vel(0.0f);
            Motor_Pitch.Set_Transform_Target_Acc(0.0f);

            pre_yaw_angle      = 0.0f;
            pre_pitch_angle    = Motor_Pitch.Get_Transform_Angle();
            pre_main_yaw_angle = Boardc_BMI.Get_Angle_Yaw();
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            if(MiniPC->Get_mode() == 1 || MiniPC->Get_mode() == 2){
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                float MiniPC_Target_Yaw    = MiniPC->Get_Rx_Yaw_Angle();
                float MiniPC_Target_Pitch  = MiniPC->Get_Rx_Pitch_Angle();

                Target_Yaw_Angle   = MiniPC_Target_Yaw - Motor_Main_Yaw.Get_Transform_Angle();
                Target_Pitch_Angle = MiniPC_Target_Pitch;

                //怕超出限位到死区
                Math_Constrain(&Target_Yaw_Angle, -LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

                Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

                Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
                Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

                Motor_Yaw.Set_Transform_Target_Vel(MiniPC->Data_NUC_To_MCU_Test.yaw_vel);
                Motor_Yaw.Set_Transform_Target_Acc(MiniPC->Data_NUC_To_MCU_Test.yaw_acc);
                Motor_Pitch.Set_Transform_Target_Vel(MiniPC->Data_NUC_To_MCU_Test.pitch_vel);
                Motor_Pitch.Set_Transform_Target_Acc(MiniPC->Data_NUC_To_MCU_Test.pitch_acc);

                pre_yaw_angle = Motor_Yaw.Get_Zero_Offset_Angle();
                pre_pitch_angle = Motor_Pitch.Get_Transform_Angle();
            }
            else{
                Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

                Target_Yaw_Angle = 0.0f;
                Target_Pitch_Angle = 0.0f;//会和dr16的遥控器输入冲突
                // 对于Yaw控制的突变点与优劣弧处理       0--2*PI
                Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

                // 限制角度
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

                Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);                       //可能可以加前馈
                Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

                Motor_Yaw.Set_Transform_Target_Vel(0.0f);
                Motor_Yaw.Set_Transform_Target_Acc(0.0f);
                Motor_Pitch.Set_Transform_Target_Vel(0.0f);
                Motor_Pitch.Set_Transform_Target_Acc(0.0f);

                //更新历史值
                pre_yaw_angle = Motor_Yaw.Get_Zero_Offset_Angle();
                pre_pitch_angle = Motor_Pitch.Get_Transform_Angle();
            }

            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
            Target_Main_Yaw_Angle = pre_main_yaw_angle;
            Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());
            Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Main_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);

            Target_Yaw_Angle      = pre_yaw_angle;
            Target_Pitch_Angle    = pre_pitch_angle;
            Target_Main_Yaw_Angle = pre_main_yaw_angle;
            Angle_Continuity_Process(&Target_Main_Yaw_Angle, Boardc_BMI.Get_Angle_Yaw());
            Angle_Continuity_Process(&Target_Yaw_Angle, Motor_Yaw.Get_Zero_Offset_Angle());

            // 限制角度
            Math_Constrain(&Target_Yaw_Angle,-LIMIT_YAW_ANGLE, LIMIT_YAW_ANGLE);            
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Main_Yaw.Set_Target_Angle(Target_Main_Yaw_Angle);

            Motor_Yaw.Set_Transform_Target_Vel(0.0f);
            Motor_Yaw.Set_Transform_Target_Acc(0.0f);
            Motor_Pitch.Set_Transform_Target_Vel(0.0f);
            Motor_Pitch.Set_Transform_Target_Acc(0.0f);

        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    External_IMU_Gyro_Yaw.Set_Now(External_IMU.Get_Gyro_Yaw());
    External_IMU_Gyro_Yaw.Recv_Adjust_PeriodElapsedCallback();              //滤除由于大Yaw转动带来的联动噪声
    External_IMU_Gyro_Pitch.Set_Now(External_IMU.Get_Gyro_Pitch());
    External_IMU_Gyro_Pitch.Recv_Adjust_PeriodElapsedCallback();              //滤除由于大Yaw转动带来的联动噪声
    // Motor_Yaw_Angle_Filter.Set_Now(Motor_Yaw.Get_Zero_Offset_Angle());
    // Motor_Yaw_Angle_Filter.Recv_Adjust_PeriodElapsedCallback();

    //数据传输更新        记得对方向
    Motor_Yaw.Set_Transform_Omega(External_IMU_Gyro_Yaw.Get_Out());
    Motor_Yaw.Set_Transform_Angle(Motor_Yaw.Get_Zero_Offset_Angle());

    Motor_Main_Yaw.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Yaw());
    Motor_Main_Yaw.Set_Transform_Angle(Boardc_BMI.Get_Angle_Yaw());

    Motor_Pitch.Set_Transform_Omega(External_IMU_Gyro_Pitch.Get_Out());
    Motor_Pitch.Set_Transform_Angle(External_IMU.Get_Angle_Pitch());

    //控制更新
    // Output();
    Output_Test();

    //可能得写死区严重时的强制保护

    // Motor_Pitch_LESO.Set_CMD_Torque(Motor_Pitch.Get_Out());               //上一时刻的输出力矩
    // Motor_Pitch_LESO.Set_Now_Omega(Motor_Pitch.Get_Transform_Omega());
    // Motor_Pitch_LESO.TIM_Adjust_PeriodElapsedCallback();

    // if(Motor_Yaw.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE){
    //     Motor_Yaw.TIM_SMC_PeriodElapsedCallback();
    // }
    // else{
    //     Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    // }

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Main_Yaw.TIM_Process_PeriodElapsedCallback();

    if(Get_Gimbal_Control_Type() != Gimbal_Control_Type_DISABLE){
        Pitch_Compensite_Output = test_c * cosf(Motor_Pitch.Get_Transform_Angle() / 57.3f);
        // Pitch_Compensite_Output = Motor_Pitch_LESO.Get_Compensation_Out();
        Motor_Pitch.Compensite_Output(Pitch_Compensite_Output);
    }
    else{
        Pitch_Compensite_Output = 0;
        Motor_Pitch.Compensite_Output(Pitch_Compensite_Output);
    }

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
