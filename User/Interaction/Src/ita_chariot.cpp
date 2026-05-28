/**
 * @file ita_chariot.cpp
 * @author cjw by yssickjgd
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
#include "dvc_dwt.h"
#include "dvc_GraphicsSendTask.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

uint16_t gimbal_lock = 2;
uint16_t run_time = 1;
bool Reset_Pitch_2 = false;
/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    #ifdef CHASSIS
    
        Boardc_BMI.Init();

        //裁判系统
        Referee.Init(&huart10);

        //底盘
        Chassis.IMU = &Boardc_BMI;
        Chassis.Referee = &Referee;
        Chassis.Init();

        // 底盘随动PID环初始化
        PID_Chassis_Follow.Init(-0.15f, 0.0f, -0.0008f, 0.0f, 2.5f, 5.5f);
        //PID_Chassis_Follow.Init(0.f, 0.0f, 0.0f, 0.0f, 2.5f, 5.5f);

        
		Motor_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, 200.f, 0, 33.0f, LK_Motor_Control_Method_IMU_ANGLE, LK_Motor_Control_Torque);
        Chassis.Motor_Yaw = &Motor_Yaw;
        
        //超电
        Chassis.Supercap.Referee = &Referee;

        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);

        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);
        FSM_Alive_Control_VT13.Chariot = this;
        FSM_Alive_Control_VT13.Init(5, 0);

        //遥控器
        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;   

        // 初始化活动控制器为无
        Active_Controller = Controller_NONE;

        //云台
        Gimbal.Init();
        Gimbal.MiniPC = &MiniPC;

        //发射机构
        Booster.Referee = &Referee;
        Booster.Init();
        //Booster.MiniPC = &MiniPC;
				
        //上位机
        MiniPC.Init(&hfdcan1);
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;

    #endif

    HAL_TIM_PWM_Init(&htim12);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
    buzzer_init_example();
}


#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Cool;
    Shooter_Barrel_Heat_Limit = Referee.Get_Booster_17mm_1_Heat_Max();
    Shooter_Barrel_Heat = Referee.Get_Booster_17mm_1_Heat();
    Shooter_Cool = Referee.Get_Booster_17mm_1_Heat_CD();
    // 发送数据给云台
    CAN2_Chassis_Tx_Gimbal_Data[0] = Referee.Get_ID();
    CAN2_Chassis_Tx_Gimbal_Data[1] = Referee.Get_Game_Stage();
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Shooter_Cool, sizeof(uint16_t));
    //memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Shooter_Speed, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Shooter_Barrel_Heat, sizeof(uint16_t));
}
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback_1()
{
    uint16_t Shooter_Speed; 
    Shooter_Speed = uint16_t(Referee.Get_Shoot_Speed() * 10);
    memcpy(CAN2_Chassis_Tx_Gimbal_Data_1, &Shooter_Speed, sizeof(uint16_t));
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS    
//控制类型字节
uint8_t control_type;
//目标角速度
float chassis_omega;
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{   
    Gimbal_Alive_Flag++;
    //云台坐标系的目标速度
    float gimbal_velocity_y,gimbal_velocity_x;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    
    //超电控制类型
    //Enum_Supercap_Mode supercap_mode;
    //float映射到int16之后的速度
    int16_t tmp_velocity_x, tmp_velocity_y, tmp_gimbal_pitch;
	int16_t tmp_omega;

	
    memcpy(&tmp_velocity_x, &Rx_Data[0], sizeof(int16_t));
    memcpy(&tmp_velocity_y, &Rx_Data[2], sizeof(int16_t));
    memcpy(&tmp_omega, &Rx_Data[4], sizeof(int16_t));
    memcpy(&tmp_gimbal_pitch, &Rx_Data[6], sizeof(uint16_t));
            

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, -450 , 450, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, -450 , 450, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.0f, 4.0f);      // Chassis_Radius;//映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度

    Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -40.0f, 40.0f);

    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    // 角速度前馈，保证小陀螺时走直线
    float Feedback_Angle =  0.0f;
    if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
    {
        //Feedback_Angle = -0.057f * Chassis.Get_Spin_Omega();
    }
    else
    {
        Feedback_Angle = 0.0f;
    }

    float derta_angle;
    Chassis_Angle = Motor_Yaw.Get_Now_Radian();
    derta_angle = (Reference_Radian - Chassis_Angle) + Offset_Angle + Feedback_Angle;
    derta_angle = derta_angle < 0 ? (derta_angle + 2 * PI) : derta_angle;

    // 云台坐标系的目标速度转为底盘坐标系的目标速度 正常情况下不会更正，只有有偏移未矫正或者小陀螺或反小陀螺会更正
    //无论云台如何动，dt7传的数据可以直接对应底盘车体运动行进，而不是完全以头为正方向
	//从笛卡尔系到右手系
    chassis_velocity_x = 1.0f * ((float)(gimbal_velocity_y * cos(derta_angle) - gimbal_velocity_x * sin(derta_angle)));
    chassis_velocity_y = -1.0f * ((float)(gimbal_velocity_y * sin(derta_angle) + gimbal_velocity_x * cos(derta_angle)));

    if(chassis_omega < 0.1f && chassis_omega > -0.1f)chassis_omega = 0;//死区处理
            
    //设定底盘目标速度
    // Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    // Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Velocity_X(gimbal_velocity_y);
    Chassis.Set_Target_Velocity_Y(-gimbal_velocity_x);
    if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_Drive)
    {
        Chassis.Set_Target_Drive_Omega(-chassis_omega);
    }

}
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback_1()
{
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //云台控制类型
    Enum_Gimbal_Control_Type gimbal_control_type;

    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint8_t));

    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Sprint_Status = (Enum_Sprint_Status)(control_type >> 2 & 0x01);
    gimbal_control_type = (Enum_Gimbal_Control_Type)((control_type >> 3) & 0x03);
    Fric_Status = (Enum_Fric_Status)(control_type >> 5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type >> 6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type >> 7 & 0x01);
				
    // 更新JudgeReceiveData中的控制类型
    JudgeReceiveData.Gimbal_Control_Type = gimbal_control_type;
    JudgeReceiveData.Fric_Status = Fric_Status;
    JudgeReceiveData.Chassis_Control_Type = chassis_control_type;
    //设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);  

    //Chassis.Set_Supercap_Mode(supercap_mode);
    //Chassis.Set_Supercap_Mode(Supercap_ENABLE);
}
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback_2()
{
    memcpy(&H7_Offset_X, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(float));
    memcpy(&H7_Offset_Y, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(float));
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
uint16_t Shooter_Barrel_Heat_Limit; 
uint16_t Shooter_Barrel_Cooling_Value;
uint16_t tmp_heat;
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
	
    Enum_Referee_Data_Robots_ID robo_id;
    Enum_Referee_Game_Status_Stage game_stage;
    uint16_t Shooter_Barrel_Heat;
    
    robo_id = (Enum_Referee_Data_Robots_ID)CAN_Manage_Object->Rx_Buffer.Data[0];
    game_stage = (Enum_Referee_Game_Status_Stage)CAN_Manage_Object->Rx_Buffer.Data[1];
    memcpy(&Shooter_Barrel_Heat_Limit, CAN_Manage_Object->Rx_Buffer.Data + 2, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Cooling_Value, CAN_Manage_Object->Rx_Buffer.Data + 4, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Heat, CAN_Manage_Object->Rx_Buffer.Data + 6, sizeof(uint16_t));
    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Booster_17mm_1_Heat_Max(Shooter_Barrel_Heat_Limit);
    Referee.Set_Game_Stage(game_stage);
    Referee.Set_Booster_17mm_1_Heat(Shooter_Barrel_Heat);
    Referee.Set_Booster_17mm_1_Heat_CD(Shooter_Barrel_Cooling_Value);
}
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback_1()
{
    uint16_t tmp_shooter_speed;
    float Shooter_Speed;
    memcpy(&tmp_shooter_speed, CAN_Manage_Object->Rx_Buffer.Data, sizeof(uint16_t));
    Shooter_Speed = tmp_shooter_speed / 10.0f;
    Referee.Set_Booster_Speed(Shooter_Speed);

}
#endif


/**
 * @brief can回调函数给底盘发送数据
 *
 */
#ifdef GIMBAL
float chassis_omega1 = 0;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, gimbal_pitch; 
    //映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0;
    uint16_t tmp_gimbal_pitch = 0;
    float chassis_omega = 0;
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
    //Supercap_Mode = MiniPC.Get_Supercap_Mode();
    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x,-4.f , 4.f ,-450,450);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y,-4.f , 4.f ,-450,450);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));
    
    tmp_chassis_omega = -Math_Float_To_Int(chassis_omega,-4.f ,4.f ,-200,200);//随动环 逆时针为正所以加负号
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(int16_t));

    tmp_gimbal_pitch = Math_Float_To_Int(gimbal_pitch, -20.0f, 25.0f, 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 6, &tmp_gimbal_pitch, sizeof(uint16_t));

    chassis_omega1 = chassis_omega;

}
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback_1()
{
    //控制类型字节
    uint8_t control_type;
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type = Chassis.Get_Chassis_Control_Type();
    //云台控制类型
    Enum_Gimbal_Control_Type gimbal_control_type = Gimbal.Get_Gimbal_Control_Type();
    //发射机构控制状态
    uint8_t booster_fire_type = Booster.Get_Friction_Control_Type();
    control_type = (uint8_t)(Referee_UI_Refresh_Status << 7 | MiniPC_Status << 6 | booster_fire_type << 5 | gimbal_control_type << 3 | Sprint_Status << 2 | chassis_control_type);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data_1, &control_type ,sizeof(uint8_t));
}
float H7_X = 0.072f;
float H7_Y = 0.158f;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback_2()
{
    memcpy(&H7_X, &CAN2_Gimbal_Tx_Chassis_Data_2, sizeof(float));
    memcpy(&H7_Y, &CAN2_Gimbal_Tx_Chassis_Data_2 + 4, sizeof(float));
}
#endif

/**
 * @brief 底盘控制逻辑
 *
 */
float chassis_omega_t;		
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x = 0, dr16_l_y = 0, dr16_r_x = 0;
    float vt13_l_x = 0, vt13_l_y = 0;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    static float chassis_omega = 0;

    //双击检测标志位
    static bool z_key_flag = false;
    //双击检测时间戳
    static float z_key_last_time = 0;

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
        dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();
            }
            else
            {
                //有大PITCH的情况下用
                if (Gimbal.Motor_Pitch_2.Get_Now_Angle() > LOCK_PITCH + 0.13f && Gimbal.Motor_Pitch_2.Get_Now_Angle() < 3.13f)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                    Chassis.Set_Target_Omega(0.0f);
                }
                else
                {
                    // 底盘随动
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                }
            }
        }
        else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = -Chassis.Get_Spin_Omega();
            // if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            // {
            //     chassis_omega = Chassis.Get_Spin_Omega();
            // }
        }

    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_l_x = (Math_Abs(VT13.Get_Left_X()) > DR16_Dead_Zone) ? VT13.Get_Left_X() : 0;
        vt13_l_y = (Math_Abs(VT13.Get_Left_Y()) > DR16_Dead_Zone) ? VT13.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = vt13_l_x * sqrt(1.0f - vt13_l_y * vt13_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = vt13_l_y * sqrt(1.0f - vt13_l_x * vt13_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (VT13.Get_Switch() == VT13_Switch_Status_Left)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = -Chassis.Get_Spin_Omega();
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Middle)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = Chassis.Get_Spin_Omega();
        }

        if (VT13.Get_Trigger() == VT13_Trigger_PRESSED)
        {
            Sprint_Status = Sprint_Status_ENABLE;
        }
        else
        {
            Sprint_Status = Sprint_Status_DISABLE;
        }
    }
        /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {
            if (DR16.Get_Keyboard_Key_Shift() == DR16_Key_Status_PRESSED) // 按住shift加速
            {
                DR16_Mouse_Chassis_Shift = 1.0f;
                Sprint_Status = Sprint_Status_ENABLE;
            }
            else
            {
                DR16_Mouse_Chassis_Shift = 2.0f;
                Sprint_Status = Sprint_Status_DISABLE;
            }

            if (DR16.Get_Keyboard_Key_A() == DR16_Key_Status_PRESSED) // x轴
            {
                chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_D() == DR16_Key_Status_PRESSED)
            {
                chassis_velocity_x = Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_W() == DR16_Key_Status_PRESSED) // y轴
            {
                chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (DR16.Get_Keyboard_Key_S() == DR16_Key_Status_PRESSED)
            {
                chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }

            if (DR16.Get_Keyboard_Key_E() == DR16_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                    chassis_omega = Chassis.Get_Spin_Omega();
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (DR16.Get_Keyboard_Key_R() == DR16_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
            }
            else
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
            }
        }
        else if (Active_Controller == Controller_VT13)
        {
            if (VT13.Get_Keyboard_Key_Shift() == VT13_Key_Status_PRESSED) // 按住shift加速
            {
                DR16_Mouse_Chassis_Shift = 1.0f;
                Sprint_Status = Sprint_Status_ENABLE;
            }
            else
            {
                DR16_Mouse_Chassis_Shift = 2.0f;
                Sprint_Status = Sprint_Status_DISABLE;
            }
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }
                        // 双击Z键切换底盘控制模式
            // if (VT13.Get_Keyboard_Key_Z() == VT13_Key_Status_TRIG_FREE_PRESSED)
            // {
            //     float tmp_time = DWT_GetTimeline_ms();
            //     if (tmp_time - z_key_last_time < 500) // 双击检测
            //     {
            //         z_key_flag = true;
            //     }
            //     else
            //     {
            //         z_key_flag = false;
            //     }
            //     if(z_key_flag)
            //     {
            //         if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_Drive)
            //         {
            //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            //         }
            //         else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
            //         {
            //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
            //         }
            //     }
            //     z_key_last_time = tmp_time;

            // }
            if (VT13.Get_Keyboard_Key_A() == VT13_Key_Status_PRESSED) // x轴
            {
                chassis_velocity_x = -Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_D() == VT13_Key_Status_PRESSED)
            {
                chassis_velocity_x = Chassis.Get_Velocity_X_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_W() == VT13_Key_Status_PRESSED) // y轴
            {
                chassis_velocity_y = Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }
            if (VT13.Get_Keyboard_Key_S() == VT13_Key_Status_PRESSED)
            {
                chassis_velocity_y = -Chassis.Get_Velocity_Y_Max() / DR16_Mouse_Chassis_Shift;
            }

            if (VT13.Get_Keyboard_Key_E() == VT13_Key_Status_TRIG_FREE_PRESSED) // E键切换小陀螺与随动
            {
                if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
                    chassis_omega = Chassis.Get_Spin_Omega();
                }
                else if (Chassis.Get_Chassis_Control_Type() != Chassis_Control_Type_Drive)
                {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                }
            }

            if (VT13.Get_Keyboard_Key_R() == VT13_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
            }
            else
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
            }

            // if(z_key_flag == true)
            // {
            //     if (Chassis.Get_Chassis_Control_Type() != Chassis_Control_Type_Drive)
            //     {
            //         if (Gimbal.Motor_Pitch_2.Get_Now_Angle() > LOCK_PITCH + 0.35f && Gimbal.Motor_Pitch_2.Get_Now_Angle() < 3.13f)
            //         {
            //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
            //             Chassis.Set_Target_Omega(0.0f);
            //         }
            //         else
            //         {
            //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            //             z_key_flag = false;
            //         }
            //     }
            //     else
            //     {
            //         z_key_flag = false;
            //     }
            // }
        }
    }

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x正，左y正
    chassis_omega_t = chassis_omega;
    Chassis.Set_Target_Omega(chassis_omega);
}
#endif

/**
 * @brief 鼠标数据转换
 *
 */
#ifdef GIMBAL
void Class_Chariot::Transform_Mouse_Axis()
{
    // 根据当前活动的控制器选择鼠标数据
    if (Active_Controller == Controller_DR16)
    {
        True_Mouse_X = -DR16.Get_Mouse_X();
        True_Mouse_Y = -DR16.Get_Mouse_Y();
        True_Mouse_Z = DR16.Get_Mouse_Z();
    }
    else if (Active_Controller == Controller_VT13)
    {
        True_Mouse_X = -VT13.Get_Mouse_X();
        True_Mouse_Y = -VT13.Get_Mouse_Y();
        True_Mouse_Z = VT13.Get_Mouse_Z();
    }
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{
    // 角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch, tmp_gimbal_pitch_2;
    // 遥控器摇杆值
    float dr16_y = 0, dr16_r_y = 0;
    float vt13_y = 0, vt13_r_y = 0;
    // 获取当前角度值
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
    tmp_gimbal_pitch_2 = Gimbal.Get_Target_Pitch_Angle_2();

    //双击检测标志位
    static bool z_key_flag = false;
    //双击检测时间戳
    static float z_key_last_time = 0;

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下自瞄
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
        }
        else // 非自瞄模式
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // 遥控器操作逻辑
            if(DR16.Get_Right_Switch() != DR16_Switch_Status_DOWN){
                tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;
                Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
                //Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE);
                //有大PITCH的情况下用
                if(Gimbal.Motor_Pitch_2.Get_Now_Angle() > LOCK_PITCH + 0.07f && Gimbal.Motor_Pitch_2.Get_Now_Angle() < 3.13f)
                {
                    //Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_type_FOLD);
                    Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE_LOCK);
                    Gimbal.Motor_Pitch_2.Set_Target_Angle(LOCK_PITCH);   
                }
                tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Angle_Resolution;
            }
            else if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN && DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE){
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_type_FOLD);
                Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE_LOCK);
                tmp_gimbal_pitch_2 += dr16_r_y * DR16_Pitch_Angle_Resolution * 0.05f;
                tmp_gimbal_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
            }
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > DR16_Dead_Zone) ? VT13.Get_Right_Y() : 0;

        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
        }
        if (VT13.Get_Button_Right() == VT13_Button_TRIG_FREE_PRESSED) //
        {
            if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL)
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            else if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC)
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

            // 两次开启自瞄分别切换四点五点
            // if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Nomal)
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill); // 五点
            // else
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
        }

        // 遥控器操作逻辑
        tmp_gimbal_yaw -= vt13_y * DR16_Yaw_Angle_Resolution;
        tmp_gimbal_pitch += vt13_r_y * DR16_Pitch_Angle_Resolution;
    }		
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {
            if (DR16.Get_Keyboard_Key_Q() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_pitch = 0;
            }

            // 长按右键  开启自瞄
            if (DR16.Get_Mouse_Right_Key() == DR16_Key_Status_PRESSED)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
            // C键按下 一键调头
            if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
            }
            // V键按下 自瞄模式中切换四点和五点模式
            if (DR16.Get_Keyboard_Key_V() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {

            }
            // Z键按下 切换反小陀螺开关
            // if (DR16.Get_Keyboard_Key_Z() == DR16_Key_Status_TRIG_FREE_PRESSED)
            // {
            //     if (Gimbal.MiniPC->Get_Antispin_Type() == Antispin_On)
            //     {
            //         Gimbal.MiniPC->Set_Antispin_Type(Antispin_Off);
            //     }
            //     else
            //     {
            //         Gimbal.MiniPC->Set_Antispin_Type(Antispin_On);
            //     }
            // }
            // G键按下重新使能大pitch电机
            if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                Reset_Pitch_2 = true;
            }
            // R键切换云台折叠方式
            // if (DR16.Get_Keyboard_Key_R() == DR16_Key_Status_TRIG_FREE_PRESSED)
            // {
            //     if ( Gimbal.Get_Gimbal_Control_Type() != Gimbal_Control_type_FOLD)
            //         Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_type_FOLD);
            //     else
            //         Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // }
        }
        else if (Active_Controller == Controller_VT13)
        {
            if (VT13.Get_Keyboard_Key_Q() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_pitch = 0;
            }

            // 长按右键  开启自瞄
            if (VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED)
            {
                if (Gimbal.Get_Gimbal_Control_Type() != Gimbal_Control_type_FOLD)
                {
                    Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
                    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
                    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
                }
                else
                {
                    //Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_type_FOLD);
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                }
            }
            else
            {
                if (Gimbal.Get_Gimbal_Control_Type() != Gimbal_Control_type_FOLD)
                {
                    if (Gimbal.Motor_Pitch_2.Get_Now_Angle() < LOCK_PITCH + 0.07f || Gimbal.Motor_Pitch_2.Get_Now_Angle() > 3.10f)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
                        Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
                    }
                    else
                    {
                        Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE_LOCK);
                    }
                }
            }
            tmp_gimbal_yaw -= VT13.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch -= VT13.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;

            // C键按下 一键调头
            if (VT13.Get_Keyboard_Key_C() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
            }

            // 双击Z键切换底盘控制模式
            if (VT13.Get_Keyboard_Key_Z() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                float tmp_time = DWT_GetTimeline_ms();
                if (tmp_time - z_key_last_time < 500) // 双击检测
                {
                    z_key_flag = true;
                }
                else
                {
                    z_key_flag = false;
                }
                if(z_key_flag)
                {
                    if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_type_FOLD)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
                        Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE_LOCK);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                    }
                    else
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_type_FOLD);
                        // 强制同步：云台 FOLD 时，底盘设为 Drive
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                    }
                }
                z_key_last_time = tmp_time;

            }
            // V键按下 自瞄模式中切换四点和五点模式
            if (VT13.Get_Keyboard_Key_V() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {

            }
            if (VT13.Get_Keyboard_Key_G() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                Reset_Pitch_2 = true;
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
                z_key_flag == true;
            }
            if(z_key_flag == true)
            {
                if (Gimbal.Get_Gimbal_Control_Type() != Gimbal_Control_type_FOLD)
                {
                    if (Gimbal.Motor_Pitch_2.Get_Now_Angle() > LOCK_PITCH + 0.07f && Gimbal.Motor_Pitch_2.Get_Now_Angle() < 3.13f)
                    {
                        Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_ANGLE_LOCK);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_Drive);
                    }
                    else
                    {
                        Gimbal.Motor_Yaw.Set_LK_Motor_Control_Method(LK_Motor_Control_Method_IMU_ANGLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                        z_key_flag = false;
                    }
                }
                else
                {
                    z_key_flag = false;
                }
            }
            if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_Drive)
            {
                Chassis.Set_Target_Omega(-VT13.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution);
            }
        }
    }  
		// 设定目标角度
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
    //Gimbal.Set_Target_Pitch_Angle_2(tmp_gimbal_pitch_2);

}
#endif

/**
 * @brief 发射机构控制逻辑
 *
 */
#ifdef GIMBAL
#define DISABLE_ZD_SHOOT
void Class_Chariot::Control_Booster()
{
    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 左上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            //Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            Fric_Status = Fric_Status_OPEN;

            if(DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
            {         //自瞄模式火控 上位机控制打弹ee
                if (MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_ENABLE)
                {
                    // Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    if (DR16.Get_Yaw() > 0.8f) // 五连发
                    {
                        if (MiniPC.Get_Fire_Status())
                        {
                            Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                        }
                        //Shoot_Flag = 1;
                    }
                }
            }
            else
            {
                if (DR16.Get_Yaw() > -0.2f && DR16.Get_Yaw() < 0.2f)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    Shoot_Flag = 0;
                }
                if (DR16.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    Shoot_Flag = 1;
                }
                if (DR16.Get_Yaw() > 0.8f) // 五连发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    //Shoot_Flag = 1;
                }
                // if (DR16.Get_Yaw() > 0.8f) // 五连发
                // {
                //     Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                // }
            }
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            Fric_Status = Fric_Status_CLOSE;
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 开启摩擦轮和发射机构
        if (VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED)
        {
            if (Fric_Status == Fric_Status_OPEN)
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                Fric_Status = Fric_Status_CLOSE;
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                Fric_Status = Fric_Status_OPEN;
            }
        }
        if (Fric_Status == Fric_Status_OPEN)
        {
            static uint16_t limi = 0;
            if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC)
            {
                if(MiniPC.Get_Fire_Status() == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_ENABLE){
                if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    Shoot_Flag = 0;
                }
                if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    Shoot_Flag = 1;
                }
                if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                }
                }
            }
            else
            {
                if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    Shoot_Flag = 0;
                }
                if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    Shoot_Flag = 1;
                }
                if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                }
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        // 分别处理DR16和VT13遥控器
        if (Active_Controller == Controller_DR16)
        {

            if (DR16.Get_Keyboard_Key_B() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
                }
                else
                {
                    Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
                }
            }
            // 按下ctrl键 开启摩擦轮
            if (DR16.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Fric_Status == Fric_Status_CLOSE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                    Fric_Status = Fric_Status_OPEN;
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                    Fric_Status = Fric_Status_CLOSE;
                }
            }

            // 按下鼠标左键 单发
            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_TRIG_FREE_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    }
                }
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                {
                    if (DR16.Get_Mouse_Left_Key() == DR16_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    }
                    else
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                }
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }
        else if (Active_Controller == Controller_VT13)
        {
            // if (VT13.Get_Keyboard_Key_B() == VT13_Key_Status_TRIG_FREE_PRESSED)
            // {
            //     if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
            //     {
            //         Booster.Booster_User_Control_Type = Booster_User_Control_Type_MULTI;
            //     }
            //     else
            //     {
            //         Booster.Booster_User_Control_Type = Booster_User_Control_Type_SINGLE;
            //     }
            // }
            if (VT13.Get_Keyboard_Key_Ctrl() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Fric_Status == Fric_Status_CLOSE)
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                    Fric_Status = Fric_Status_OPEN;
                }
                else
                {
                    Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                    Fric_Status = Fric_Status_CLOSE;
                }
            }

            if (Booster.Get_Friction_Control_Type() == Friction_Control_Type_ENABLE)
            {
                if (VT13.Get_Mouse_Right_Key() == VT13_Key_Status_PRESSED)
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    }
                    else
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    }
                }
                else
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_REPEATED);
                    }
                    else
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                    }
                }
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }

            // if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_type_FOLD)
            // {
            //     Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            //     Fric_Status = Fric_Status_CLOSE;
            // }
        }
    }
}
#endif

uint8_t change_time = 0;
uint32_t Last_Cnt_Omega;
float Dt_Omega;
float reference_angle = Reference_Angle;
/**
 * @brief 计算回调函数
 *
 */
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS

	if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
	{
		// 随动环
		Chassis_Angle = Motor_Yaw.Get_Now_Angle();

        // // 将角度规范化到 [-π, π] 范围内
        // if (Chassis_Angle > 180.0f)
        //     Chassis_Angle = Chassis_Angle - 360.0f;
        // else if (Chassis_Angle < -180.0f)
        //     Chassis_Angle = Chassis_Angle + 360.0f;

        // if (Reference_Angle > 180.0f)
        //     Reference_Angle = Reference_Angle - 360.0f;
        // else if (Reference_Angle < -180.0f)
        //     Reference_Angle = Reference_Angle + 360.0f;

        // float diff = Reference_Angle - Chassis_Angle;
        // if(diff > 360.0f)
        // {
        //     Reference_Angle -= 360.0f;
        // }
        // else if(diff < -360.0f)
        // {
        //     Reference_Angle += 360.0f;
        // }
        
		// PID_Chassis_Follow.Set_Target(Reference_Angle);
		// PID_Chassis_Follow.Set_Now(Chassis_Angle);
        // //处理优劣弧
		// if(Reference_Angle - Chassis_Angle > 180.0f)
		// {
		// 	PID_Chassis_Follow.Set_Target(Reference_Angle - 360.0f);
		// }
		// else if(Reference_Angle - Chassis_Angle < -180.0f)
		// {
		// 	PID_Chassis_Follow.Set_Target(Reference_Angle + 360.0f);
		// }
		// else
		// {
					
		// }
        float Chassis_Radian = Motor_Yaw.Get_Now_Radian();
        float Delta_Radian   = Reference_Radian - Chassis_Radian;

        Delta_Radian = Normalize_Angle_Radian_PI_to_PI(Delta_Radian);

        PID_Chassis_Follow.Set_Target((Chassis_Radian + Delta_Radian) * 57.3f);
        PID_Chassis_Follow.Set_Now(Chassis_Radian*57.3f);

		PID_Chassis_Follow.TIM_Adjust_PeriodElapsedCallback();
		 Chassis.Set_Target_Omega(PID_Chassis_Follow.Get_Out());
        //Chassis.Set_Target_Omega(0.0f);
        change_time = 0;
	}
		
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
	{
		//Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
        //补充力控底盘
        // Dt_Omega += DWT_GetDeltaT(&Last_Cnt_Omega);

        // float Control_Omega = (4.0f + 1.0f * sinf(2.0 * PI * Dt_Omega)) * PI;

        // Chassis.Set_Target_Omega(Control_Omega);
        Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
	}
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_Drive)
    {
        //在遥控器控制策略中实现
        if(change_time < 100)
        {
            Chassis.Set_Target_Omega(0.0f);
            change_time++;
        }
    }
	else
	{
		Chassis.Set_Target_Omega(0.0f);
	}

	// Chassis.Set_Sprint_Status(Sprint_Status);	
     // 超电控制
    if (Sprint_Status == Sprint_Status_ENABLE)
    {
        Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Supercap_BufferPower);
    }
    else
    {
        Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Referee_BufferPower);
    }
    Chassis.Supercap.TIM_Supercap_PeriodElapsedCallback();
    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);//还有飞坡前馈没写
				
#elif defined(GIMBAL)
    Chassis_Angle = Gimbal.Motor_Yaw.Get_Now_Angle();
    float diff = reference_angle - Chassis_Angle;
    if(diff > 360.0f)
    {
        reference_angle -= 360.0f;
    }
    else if(diff < -360.0f)
    {
        reference_angle += 360.0f;
    }
    YAW_Reference_Angle = reference_angle;
    YAW_Chassis_Angle = Chassis_Angle;
    //各个模块的分别解算
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    //传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();
#endif				
}


#ifdef GIMBAL
/**
 * @brief 判断DR16控制数据来源
 *
 */
void Class_Chariot::Judge_DR16_Control_Type()
{
    if (DR16.Get_Left_X() != 0 ||
        DR16.Get_Left_Y() != 0 ||
        DR16.Get_Right_X() != 0 ||
        DR16.Get_Right_Y() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_REMOTE;
    }
    else if (DR16.Get_Mouse_X() != 0 ||
             DR16.Get_Mouse_Y() != 0 ||
             DR16.Get_Mouse_Z() != 0 ||
             DR16.Get_Keyboard_Key_A() != 0 ||
             DR16.Get_Keyboard_Key_D() != 0 ||
             DR16.Get_Keyboard_Key_W() != 0 ||
             DR16.Get_Keyboard_Key_S() != 0 ||
             DR16.Get_Keyboard_Key_Shift() != 0 ||
             DR16.Get_Keyboard_Key_Ctrl() != 0 ||
             DR16.Get_Keyboard_Key_Q() != 0 ||
             DR16.Get_Keyboard_Key_E() != 0 ||
             DR16.Get_Keyboard_Key_R() != 0 ||
             DR16.Get_Keyboard_Key_F() != 0 ||
             DR16.Get_Keyboard_Key_G() != 0 ||
             DR16.Get_Keyboard_Key_Z() != 0 ||
             DR16.Get_Keyboard_Key_C() != 0 ||
             DR16.Get_Keyboard_Key_V() != 0 ||
             DR16.Get_Keyboard_Key_B() != 0)
    {
        DR16_Control_Type = DR16_Control_Type_KEYBOARD;
    }
    else
    {
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            DR16_Control_Type = DR16_Control_Type_NONE;
    }
}

/**
 * @brief 判断VT13控制数据来源
 *
 */
void Class_Chariot::Judge_VT13_Control_Type()
{
    if (VT13.Get_Left_X() != 0 ||
        VT13.Get_Left_Y() != 0 ||
        VT13.Get_Right_X() != 0 ||
        VT13.Get_Right_Y() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_REMOTE;
    }
    else if (VT13.Get_Mouse_X() != 0 ||
             VT13.Get_Mouse_Y() != 0 ||
             VT13.Get_Mouse_Z() != 0 ||
             VT13.Get_Keyboard_Key_A() != 0 ||
             VT13.Get_Keyboard_Key_D() != 0 ||
             VT13.Get_Keyboard_Key_W() != 0 ||
             VT13.Get_Keyboard_Key_S() != 0 ||
             VT13.Get_Keyboard_Key_Shift() != 0 ||
             VT13.Get_Keyboard_Key_Ctrl() != 0 ||
             VT13.Get_Keyboard_Key_Q() != 0 ||
             VT13.Get_Keyboard_Key_E() != 0 ||
             VT13.Get_Keyboard_Key_R() != 0 ||
             VT13.Get_Keyboard_Key_F() != 0 ||
             VT13.Get_Keyboard_Key_G() != 0 ||
             VT13.Get_Keyboard_Key_Z() != 0 ||
             VT13.Get_Keyboard_Key_C() != 0 ||
             VT13.Get_Keyboard_Key_V() != 0 ||
             VT13.Get_Keyboard_Key_B() != 0)
    {
        VT13_Control_Type = VT13_Control_Type_KEYBOARD;
    }
    else
    {
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            VT13_Control_Type = VT13_Control_Type_NONE;
    }
}

/**
 * @brief 控制云台折叠
 *
 */
int clock_1 = 0;
void Class_Chariot::Contorl_Fold_Pitch()
{
    if(run_time == 0)
    {        
        clock_1++;
        if(gimbal_lock == 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        }
        if(clock_1 >= 500)
        {
            Gimbal.Set_Target_Pitch_Angle_2(LOCK_PITCH - 1.7f);
            clock_1 = 0; 
        }
        if((Gimbal.Motor_Pitch_2.Get_Now_Angle() - PI) < (LOCK_PITCH - 1.4f))
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            gimbal_lock = 2;
		    clock_1 = 0; 
            run_time++;
        }
    }
}

/**
 * @brief 控制云台伸展
 *
 */
int clock_2 = 0;
void Class_Chariot::Contorl_Stretch_Pitch()
{
if(run_time == 0){
    clock_2++;
    if(gimbal_lock == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    }
    if(clock_2 >= 500)
    {
        Gimbal.Set_Target_Pitch_Angle_2(LOCK_PITCH);
        clock_2 = 0; 
    }
    if((Gimbal.Motor_Pitch_2.Get_Now_Angle() - PI) > (LOCK_PITCH - 0.1f))
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        gimbal_lock = 2;
		clock_2 = 0; 
        run_time++;
    }
}
}

/**
 * @brief 判断当前活动的控制器
 *
 */
void Class_Chariot::Judge_Active_Controller()
{
    // 检查DR16是否有输入
    Judge_DR16_Control_Type();

    // 检查VT13是否有输入
    Judge_VT13_Control_Type();

    // 判断当前活动的控制器
    if (VT13_Control_Type != VT13_Control_Type_NONE)
    {
        Active_Controller = Controller_VT13;
    }
    else if (DR16_Control_Type != DR16_Control_Type_NONE)
    {
        Active_Controller = Controller_DR16;
    }
    else
    {
        Active_Controller = Controller_NONE;
    }
}

/**
 * @brief 获取当前活动的控制器类型
 *
 * @return Enum_Active_Controller 当前活动的控制器类型
 */
Enum_Active_Controller Class_Chariot::Get_Active_Controller()
{
    return Active_Controller;
}
/**
 * @brief 获取DR16控制类型
 *
 */
Enum_DR16_Control_Type Class_Chariot::Get_DR16_Control_Type()
{
    if (Active_Controller == Controller_DR16)
    {
        return DR16_Control_Type;
    }
    else
    {
        return DR16_Control_Type_NONE;
    }
}

/**
 * @brief 获取VT13控制类型
 *
 */
Enum_VT13_Control_Type Class_Chariot::Get_VT13_Control_Type()
{
    if (Active_Controller == Controller_VT13)
    {
        return VT13_Control_Type;
    }
    else
    {
        return VT13_Control_Type_NONE;
    }
}

#endif

/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    // 判断DR16控制数据来源
    Judge_DR16_Control_Type();
    Judge_VT13_Control_Type();
    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif

/**
 * @brief 在线判断回调函数
 *
 */
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    static uint8_t mod50_mod7 = 0;
    static uint8_t mod50_mod20 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        mod50_mod7++;
        mod50_mod20++;
#ifdef CHASSIS
        for (auto &wheel : Chassis.Motor_Wheel)
        {
            wheel.TIM_Alive_PeriodElapsedCallback();
        }
        for (auto &steer : Chassis.Motor_Steer)
        {
            steer.TIM_Alive_PeriodElapsedCallback_MA600();
        }
        if (mod50_mod3 % 3 == 0)
        {
            // Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
        if (mod50_mod7 % 7 == 0)
        {
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            mod50_mod7 = 0;
        }
        if (mod50_mod20 % 20 == 0)
        {
            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod20 = 0;
        }
        // 云台，随动掉线保护
        // if (Motor_Yaw.Get_LK_Motor_Status() == LK_Motor_Status_DISABLE || Gimbal_Status == Gimbal_Status_DISABLE)
        // {
        //     buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
        //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        // }
        if (Motor_Yaw.Get_LK_Motor_Status() == LK_Motor_Status_DISABLE)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        if (Gimbal_Status == Gimbal_Status_DISABLE)
        {
            //buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        if (referee_dma_busy)
        {
            uint32_t update_time_value = DWT_GetTimeline_ms();
            if (update_time_value - last_update_time_value > 50000)
            {
                last_update_time_value = update_time_value;
//                referee_dma_busy = 0;
//                referee_dma_count = 0;
            }
        }
        else
        {
            last_update_time_value = DWT_GetTimeline_ms();
        }
#elif defined(GIMBAL)

        if (mod50_mod3 % 3 == 0)
        {
            // 判断底盘通讯在线状态
            TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
            VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
        if (mod50_mod7 % 7 == 0)
        {
            MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod7 = 0;
        }

        Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Pitch_2.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();


#endif

        mod50 = 0;
    }
}


/**
 * @brief 离线保护函数
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    // 云台离线保护
    #ifdef GIMBAL

        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE && VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            // 记录离线前一状态
            Pre_Gimbal_Control_Type = Gimbal.Get_Gimbal_Control_Type();
            Pre_Chassis_Control_Type = Chassis.Get_Chassis_Control_Type();
            // 控制模块禁用
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            // 遥控器中途断联导致错误，重启 DMA
            if (huart5.ErrorCode)
            {
                HAL_UART_DMAStop(&huart5); // 停止以重启
                // HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
            }
        }
        else
        {
            // Gimbal.Set_Gimbal_Control_Type(Pre_Gimbal_Control_Type);
            // Chassis.Set_Chassis_Control_Type(Pre_Chassis_Control_Type);
        }

    #endif

    //底盘离线保护
    #ifdef CHASSIS
    if(Get_Gimbal_Status() == Gimbal_Status_DISABLE)
    {
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
    }
        
    #endif

}

/**
 * @brief 底盘通讯在线判断回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback()
{
    if (Chassis_Alive_Flag == Pre_Chassis_Alive_Flag)
    {
        Chassis_Status = Chassis_Status_DISABLE;
        buzzer_setTask(&buzzer,BUZZER_CALIBRATED_PRIORITY);
        //Referee.Referee_Status = Referee_Status_DISABLE;
    }
    else
    {
        Chassis_Status = Chassis_Status_ENABLE;
        //Referee.Referee_Status = Referee_Status_ENABLE;
    }
    Pre_Chassis_Alive_Flag = Chassis_Alive_Flag;   
}
#endif

#ifdef CHASSIS
void Class_Chariot::TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback()
{
    if (Gimbal_Alive_Flag == Pre_Gimbal_Alive_Flag)
    {
        Gimbal_Status = Gimbal_Status_DISABLE;
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;  
}
#endif

#ifdef GIMBAL
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    // 离线检测状态
    case (0):
    {
        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart5.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }

        // 转移为 在线状态
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
        if (Status[Now_Status_Serial].Time > 1000)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(1);
        }
    }
    break;
    // 遥控器关闭状态
    case (1):
    {
        // 离线保护
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
            Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart5.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }
    }
    break;
    // 遥控器在线状态
    case (2):
    {
        // 转移为 刚离线状态
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(3);
        }
    }
    break;
    // 刚离线状态
    case (3):
    {
        // 记录离线检测前控制模式
        Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
        Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

        // 无条件转移到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    // 遥控器串口错误状态
    case (4):
    {
        HAL_UART_DMAStop(&huart5); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}

void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            // 离线保护
            if (Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }

            if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart1.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart1); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
