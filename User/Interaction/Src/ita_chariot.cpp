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
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

uint16_t gimbal_lock = 2;
uint16_t run_time = 1;
/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart10);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();

        // 底盘随动PID环初始化
        PID_Chassis_Follow.Init(-0.02f, 0.0f, -0.0002f, 0.0f, 0.2f, 0.2f);

        
		Motor_Yaw.Init(&hfdcan2, LK_Motor_ID_0x141, 200.f, 0, 33.0f, LK_Motor_Control_Method_IMU_ANGLE, LK_Motor_Control_Torque);
        
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
        Booster.Init();
        Booster.MiniPC = &MiniPC;
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object,&UART8_Manage_Object,&CAN3_Manage_Object);
        MiniPC.IMU = &Gimbal.DM_IMU;
        MiniPC.Referee = &Referee;

    #endif
}


#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Speed;
    Shooter_Barrel_Heat_Limit = Referee.Get_Booster_17mm_1_Heat_Max();
    Shooter_Barrel_Heat = Referee.Get_Booster_17mm_1_Heat();
    Shooter_Speed = uint16_t(Referee.Get_Shoot_Speed() * 10);
    // 发送数据给云台
    CAN2_Chassis_Tx_Gimbal_Data[0] = Referee.Get_ID();
    CAN2_Chassis_Tx_Gimbal_Data[1] = Referee.Get_Game_Stage();
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Shooter_Barrel_Heat, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Shooter_Speed, sizeof(uint16_t));
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
Struct_CAN_Referee_Rx_Data_t CAN_Referee_Rx_Data;
#ifdef CHASSIS    
//控制类型字节
uint8_t control_type;

void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{   
    Gimbal_Alive_Flag++;
    //底盘坐标系的目标速度
    float gimbal_velocity_y,gimbal_velocity_x;
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //超电控制类型
    Enum_Supercap_Mode supercap_mode;
    //float映射到int16之后的速度
    int16_t tmp_velocity_x, tmp_velocity_y;
	int16_t tmp_omega;

	
    memcpy(&tmp_velocity_x, &Rx_Data[0], sizeof(int16_t));
    memcpy(&tmp_velocity_y, &Rx_Data[2], sizeof(int16_t));
    memcpy(&tmp_omega, &Rx_Data[4], sizeof(int16_t));
    memcpy(&control_type, &Rx_Data[7], sizeof(uint8_t));
            

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, -450 , 450, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, -450 , 450, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.0f, 4.0f);      // Chassis_Radius;//映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
    chassis_control_type = (Enum_Chassis_Control_Type)control_type;

    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    // 角速度前馈，保证小陀螺时走直线
    float Feedback_Angle =  0.0f;
    if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
    {
        Feedback_Angle = -0.025f * Chassis.Get_Spin_Omega();
    }
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
    {
        Feedback_Angle = 0.025f * Chassis.Get_Spin_Omega();
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
    chassis_velocity_y = 1.0f * ((float)(gimbal_velocity_y * cos(derta_angle) - gimbal_velocity_x * sin(derta_angle)));
    chassis_velocity_x = 1.0f * ((float)(gimbal_velocity_y * sin(derta_angle) + gimbal_velocity_x * cos(derta_angle)));

    if(chassis_omega < 0.5f && chassis_omega > -0.5f)chassis_omega = 0;//限幅
            
    //设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
						
    //设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);            
    //Chassis.Set_Supercap_Mode(supercap_mode);
    Chassis.Set_Supercap_Mode(Supercap_ENABLE);

}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;

    Enum_Referee_Data_Robots_ID robo_id;
    Enum_Referee_Game_Status_Stage game_stage;
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t tmp_shooter_speed;
    float Shooter_Speed;
    robo_id = (Enum_Referee_Data_Robots_ID)CAN_Manage_Object->Rx_Buffer.Data[0];
    game_stage = (Enum_Referee_Game_Status_Stage)CAN_Manage_Object->Rx_Buffer.Data[1];
    memcpy(&Shooter_Barrel_Heat_Limit, CAN_Manage_Object->Rx_Buffer.Data + 2, sizeof(uint16_t));
    memcpy(&Shooter_Barrel_Heat, CAN_Manage_Object->Rx_Buffer.Data + 4, sizeof(uint16_t));
    memcpy(&tmp_shooter_speed, CAN_Manage_Object->Rx_Buffer.Data + 6, sizeof(uint16_t));
    Shooter_Speed = tmp_shooter_speed / 10.0f;
    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Booster_17mm_1_Heat(Shooter_Barrel_Heat);
    Referee.Set_Booster_17mm_1_Heat_Max(Shooter_Barrel_Heat_Limit);
    Referee.Set_Game_Stage(game_stage);
    //Referee.Set_Booster_Speed(Shooter_Speed);
}
#endif


/**
 * @brief can回调函数给底盘发送数据
 *
 */
#ifdef GIMBAL
//控制类型字节
uint8_t control_type;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0; 
    //映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0;
    float chassis_omega = 0;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //超电控制类型
    uint8_t Supercap_Mode;
    //控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    Supercap_Mode = MiniPC.Get_Supercap_Mode();
    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x,-4.f , 4.f ,-450,450);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y,-4.f , 4.f ,-450,450);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));
    
    tmp_chassis_omega = -Math_Float_To_Int(chassis_omega,-4.f ,4.f ,-200,200);//随动环 逆时针为正所以加负号
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(int16_t));

    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 6,&Supercap_Mode ,sizeof(uint8_t));//超电

    control_type =  (uint8_t)chassis_control_type;
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 7,&control_type ,sizeof(uint8_t));

}
#endif

/**
 * @brief 底盘控制逻辑
 *
 */  		
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x = 0, dr16_l_y = 0;
    float vt13_l_x = 0, vt13_l_y = 0;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    static float chassis_omega = 0;

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = -Chassis.Get_Spin_Omega();
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                chassis_omega = Chassis.Get_Spin_Omega();
            }
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
    }
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x正，左y正
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

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

        //if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下自瞄
        //{
            // Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

            // // 两次开启自瞄分别切换四点五点
            // if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Nomal)
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill); // 五点
            // else
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
        //}
        //else // 非自瞄模式
        //{
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // 遥控器操作逻辑
            tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;
            if(DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE){
                if(DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE){
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                }
                else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP){
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                }
                tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Angle_Resolution;
            }
            else if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN){
                if(DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE){
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                }
                else if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP){
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                }
                tmp_gimbal_pitch_2 += dr16_r_y * DR16_Pitch_Angle_Resolution * 0.05f;
            }
            else if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
            {
                if(DR16.Get_Left_Switch() == DR16_Switch_Status_UP){
                    gimbal_lock = 0;
                }
                else if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
                {
                    gimbal_lock = 1;
                }
                else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE)
                {
                    gimbal_lock = 2;
                    run_time = 0;
                }
            }

        //}
        if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW &&
            DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN) // 随动才能开舵机 右拨中-下 打开舵机
        {
            Compare = 1700;
            Bulletcap_Status = Bulletcap_Status_OPEN;
        }
        else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW &&
                 DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE) // 随动才能开舵机 右拨下-中 关闭舵机
        {
            Compare = 400;
            Bulletcap_Status = Bulletcap_Status_CLOSE;
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > DR16_Dead_Zone) ? VT13.Get_Right_Y() : 0;

        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_DISABLE)
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
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
    //todo：键鼠   
		// 设定目标角度
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
    Gimbal.Set_Target_Pitch_Angle_2(tmp_gimbal_pitch_2);    
}
#endif

/**
 * @brief 发射机构控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 左上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            Fric_Status = Fric_Status_OPEN;

            if(DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
            {         //自瞄模式火控 上位机控制打弹
                // if(MiniPC.Get_Fire_Status() == 1 && MiniPC.Get_MiniPC_Status() == MiniPC_Data_Status_ENABLE){
                //      Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                //     //Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                // }
            }
            else
            {
                if (DR16.Get_Yaw() > -0.2f && DR16.Get_Yaw() < 0.2f)
                {
                    Shoot_Flag = 0;
                }
                if (DR16.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    Shoot_Flag = 1;
                }
                if (DR16.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                    Shoot_Flag = 1;
                }
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
            if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }
    }
    /************************************键鼠控制逻辑*********************************************/
    /*。。。*/

}
#endif

/**
 * @brief 计算回调函数
 *
 */
float PID_Ch;
void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS
    // 底盘给云台发消息
    CAN_Chassis_Tx_Gimbal_Callback();

	if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
	{
		// 随动环
		Chassis_Angle = Motor_Yaw.Get_Now_Angle();

		PID_Chassis_Follow.Set_Target(Reference_Angle);
		PID_Chassis_Follow.Set_Now(Chassis_Angle);
        //处理优劣弧
		if(Reference_Angle - Chassis_Angle > 180.0f)
		{
			PID_Chassis_Follow.Set_Target(Reference_Angle - 360.0f);
		}
		else if(Reference_Angle - Chassis_Angle < -180.0f)
		{
			PID_Chassis_Follow.Set_Target(Reference_Angle + 360.0f);
		}
		else
		{
					
		}
		PID_Chassis_Follow.TIM_Adjust_PeriodElapsedCallback();
        PID_Ch = PID_Chassis_Follow.Get_Out();
		Chassis.Set_Target_Omega(PID_Chassis_Follow.Get_Out());
	}
		
    else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
	{
		Chassis.Set_Target_Omega(-Chassis.Get_Spin_Omega());
	}
		
	else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
	{
		Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
	}
	else
	{
		Chassis.Set_Target_Omega(0.0f);
	}

	// Chassis.Set_Sprint_Status(Sprint_Status);		
    Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);//还有飞坡前馈没写
				
#elif defined(GIMBAL)

    //各个模块的分别解算
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    //传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();
    //给下板发送数据
    CAN_Gimbal_Tx_Chassis_Callback();
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
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        mod50_mod7++;
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
            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
        if (mod50_mod7 % 7 == 0)
        {
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            mod50_mod7 = 0;
        }
        // 云台，随动掉线保护
        if (Motor_Yaw.Get_LK_Motor_Status() == LK_Motor_Status_DISABLE || Gimbal_Status == Gimbal_Status_DISABLE)
        {
            //buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#elif defined(GIMBAL)

        if (mod50_mod3 % 3 == 0)
        {
            // 判断底盘通讯在线状态
            TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
//            VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }

        Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Pitch_2.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
        Gimbal.DM_IMU.TIM1msMod50_Alive_PeriodElapsedCallback();

        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();

        MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

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
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
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

void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback(){
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart9.ErrorCode)
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
            //离线保护
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart9.ErrorCode)
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
            HAL_UART_DMAStop(&huart9); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
