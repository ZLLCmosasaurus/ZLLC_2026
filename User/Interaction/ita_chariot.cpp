/**
 * @file ita_chariot.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
#include "dvc_GraphicsSendTask.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
#ifdef CHASSIS

    // 裁判系统
    Referee.Init(&huart6);

    // 底盘
    Chassis.Referee = &Referee;
    Chassis.Init();

    // 底盘随动PID环初始化
    PID_Chassis_Fllow.Init(4.0f, 0.0f, 0.1f, 0.0f, 10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.01f);

    // yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x206);

    huart6.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), 115200);

#elif defined(GIMBAL)

    Chassis.Set_Velocity_X_Max(4.0f);
    Chassis.Set_Velocity_Y_Max(4.0f);

    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);
    FSM_Alive_Control_VT13.Chariot = this;
    FSM_Alive_Control_VT13.Init(5, 0);

    // 遥控器
    DR16.Init(&huart3, &huart1);
    DR16_Dead_Zone = __DR16_Dead_Zone;

    // 初始化活动控制器为无
    Active_Controller = Controller_NONE;

    // 云台
    Gimbal.Init();
    Gimbal.MiniPC = &MiniPC;
    // 发射机构
    Booster.Referee = &Referee;
    Booster.Init();

    // 上位机
    //MiniPC.Init(&MiniPC_USB_Manage_Object);
    MiniPC.Init(&hcan1);
    MiniPC.IMU = &Gimbal.Boardc_BMI;
    MiniPC.Referee = &Referee;

#endif
    buzzer_init_example();
    // Buzzer.Buzzer_Init(&htim4, TIM_CHANNEL_3);
}

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS
// 控制类型字节
uint8_t control_type;
float offset_k = -0.041f;
// 底盘和云台夹角（弧度制）
float derta_angle;
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 目标角速度
    float chassis_omega;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    Enum_Gimbal_Control_Type gimbal_control_type;
    //    //底盘和云台夹角（弧度制）
    //    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y, tmp_gimbal_pitch;
    uint8_t tmp_omega;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&tmp_omega, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch, &CAN_Manage_Object->Rx_Buffer.Data[5], sizeof(uint16_t));
    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -30.0f, 30.0f);

    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Sprint_Status = (Enum_Sprint_Status)(control_type >> 2 & 0x01);
    // 将原来的Fric_Status解析改为云台控制类型解析
    gimbal_control_type = (Enum_Gimbal_Control_Type)((control_type >> 3) & 0x03);
    // 更新JudgeReceiveData中的云台控制类型
    JudgeReceiveData.Gimbal_Control_Type = gimbal_control_type;
    Booster_User_Control_Type = (Enum_Booster_User_Control_Type)(control_type >> 5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type >> 6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type >> 7 & 0x01);

    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Motor_Yaw.Get_Now_Radian();
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        Offset_Angle = offset_k * Motor_Yaw.Get_Now_Omega_Radian();
    }
    else
    {
        Offset_Angle = 0.0f;
    }

    derta_angle = Reference_Angle - Chassis_Angle + Offset_Angle;
    derta_angle = derta_angle < 0 ? (derta_angle + 2 * PI) : derta_angle;

    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));

    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);

    // 底盘控制方案
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        chassis_omega = Math_Int_To_Float(tmp_omega, 0, 0xFF, -1 * 8.0f, 8.0f);
        Chassis.Set_Spin_Omega(chassis_omega);
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
        chassis_velocity_x = 0;
        chassis_velocity_y = 0;
    }

    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
}
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback_1()
{
    uint16_t before_game_bullet_num = 0;
    MiniPC_Type = Enum_MiniPC_Type(CAN_Manage_Object->Rx_Buffer.Data[0]);
    Antispin_Type = Enum_Antispin_Type(CAN_Manage_Object->Rx_Buffer.Data[1]);
    memcpy(&Booster_fric_omega_left, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&Booster_fric_omega_right, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(uint16_t));
    memcpy(&Booster_bullet_num, &CAN_Manage_Object->Rx_Buffer.Data[6], sizeof(uint16_t));
    if (Referee.Get_Game_Stage() == Referee_Game_Status_Stage_NOT_STARTED)
    {
        Booster_bullet_num_before = before_game_bullet_num;
    }
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
    Referee.Set_Booster_Speed(Shooter_Speed);
}
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback_1()
{
    uint16_t tmp_heat;
    memcpy(&tmp_heat, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    // Booster.set_heat(tmp_heat);
}
#endif

/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
// 控制类型字节
uint8_t control_type;
int32_t test_omega1;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, gimbal_pitch;
    // 映射之后的目标速度 int16_t
    uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0;
    uint8_t tmp_chassis_omega = 0;
    uint16_t tmp_gimbal_pitch = 0;
    float chassis_omega = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;

    // 控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
    chassis_control_type = (Enum_Chassis_Control_Type)0; // Chassis.Get_Chassis_Control_Type();
    Enum_Gimbal_Control_Type gimbal_control_type = Gimbal.Get_Gimbal_Control_Type();
    uint8_t booster_user_control = Booster.Booster_User_Control_Type;
    // 修改control_type，将Fric_Status替换为gimbal_control_type
    //    control_type = (uint8_t)(Referee_UI_Refresh_Status << 7 | MiniPC_Status << 6 | booster_user_control << 5 | gimbal_control_type << 3 | Sprint_Status << 2 | chassis_control_type);
    control_type = (uint8_t)((uint8_t)Sprint_Status << 2 | ((uint8_t)chassis_control_type));
    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max(), 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(uint16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max(), 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(uint16_t));

    test_omega1 = Math_Float_To_Int(chassis_omega, -1 * 4.0f, 4.0f, 0, 256);
    tmp_chassis_omega = test_omega1;
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(uint8_t));

    tmp_gimbal_pitch = Math_Float_To_Int(gimbal_pitch, -30.0f, 30.0f, 0, 0x7FFF);
    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 5, &tmp_gimbal_pitch, sizeof(uint16_t));

    memcpy(CAN2_Gimbal_Tx_Chassis_Data + 7, &control_type, sizeof(uint8_t));
}
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback_1()
{
    uint16_t tmp_fric_omega_left = 0;
    uint16_t tmp_fric_omega_right = 0;
    uint16_t tmp_actual_bullet_num = 0;
    tmp_fric_omega_left = (uint16_t)abs(Booster.Motor_Friction_Left.Get_Now_Omega_Radian());
    tmp_fric_omega_right = (uint16_t)abs(Booster.Motor_Friction_Right.Get_Now_Omega_Radian());
    // tmp_actual_bullet_num = Booster.actual_bullet_num;
    //CAN2_Gimbal_Tx_Chassis_Data_1[0] = MiniPC.Get_MiniPC_Type();
    // CAN2_Gimbal_Tx_Chassis_Data_1[1] = MiniPC.Get_Antispin_Type();
    memcpy(CAN2_Gimbal_Tx_Chassis_Data_1 + 2, &tmp_fric_omega_left, sizeof(uint16_t));
    memcpy(CAN2_Gimbal_Tx_Chassis_Data_1 + 4, &tmp_fric_omega_right, sizeof(uint16_t));
    memcpy(CAN2_Gimbal_Tx_Chassis_Data_1 + 6, &tmp_actual_bullet_num, sizeof(uint16_t));
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
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            chassis_omega = -Chassis.Get_Spin_Omega();
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ANTI_SPIN);
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
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            chassis_omega = -Chassis.Get_Spin_Omega();
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Middle)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            chassis_omega = Chassis.Get_Spin_Omega();
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
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
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
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
                    chassis_omega = Chassis.Get_Spin_Omega();
                }
                else
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            }

            if (VT13.Get_Keyboard_Key_R() == VT13_Key_Status_PRESSED) // 按下R键刷新UI
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_ENABLE;
            }
            else
            {
                Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
            }
        }
    }

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
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
float minipc_yaw_offset = -5.0f;
float tmp_gimbal_yaw, tmp_gimbal_pitch;
float _tmp_gimbal_yaw, _tmp_gimbal_pitch;
float __tmp_gimbal_yaw, __tmp_gimbal_pitch;
float world_gimbal_pitch, world_gimbal_yaw;
float true_pitch = 0.0f, true_yaw = 0.0f;
float dr16_pitch, dr16_yaw;
bool init_finish_flag = true;
bool position_init_flag = true;
float test_gimbal_pitch = -45.0f;

void Class_Chariot::Control_Gimbal()
{
    // 角度目标值
    // float tmp_gimbal_yaw, tmp_gimbal_pitch;
    // 遥控器摇杆值

    float dr16_y = 0, dr16_r_y = 0;
    float vt13_y = 0, vt13_r_y = 0;
    // 获取当前角度值
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
    true_pitch = Gimbal.Motor_Pitch_J4310.Get_True_Angle_Pitch();
    true_yaw = Gimbal.Motor_Yaw.Get_True_Angle_Yaw();

    // IMU:相对于地面系的姿态角
    // 将无人机系下的姿态角解算到地面系
    // if (init_finish_flag)
    // {
    //     float roll = Gimbal.Boardc_BMI.Get_Rad_Roll();
    //     float yaw = Gimbal.Get_Target_Yaw_Angle() * PI / 180;
    //     float pitch = Gimbal.Get_Target_Pitch_Angle() * PI / 180;

    //     float x_w = cosf(pitch) * cosf(yaw);
    //     float y_w = sinf(pitch);
    //     float z_w = cosf(pitch) * sinf(yaw);

    //     float x_d = x_w;
    //     float y_d = y_w * cosf(-roll) + z_w * sinf(-roll);
    //     float z_d = -y_w * sinf(-roll) + z_w * cosf(-roll);

    //     tmp_gimbal_yaw = atan2f(z_d, x_d) * 180 / PI;
    //     tmp_gimbal_pitch = atan2f(y_d, sqrtf(x_d * x_d + z_d * z_d)) * 180 / PI;
    //     world_gimbal_pitch = tmp_gimbal_pitch;
    //     world_gimbal_yaw = tmp_gimbal_yaw;
    // }

    // 先判断当前活动的控制器
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_pitch = DR16.Get_Right_X();
        dr16_yaw = DR16.Get_Right_Y();
        dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
        dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下自瞄
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

            // 两次开启自瞄分别切换四点五点
            // if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Nomal)
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill); // 五点
            // else
            //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
        }
        else // 非自瞄模式
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            // 遥控器操作逻辑
            tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution * Get_Nonlinear_Angle_Resolution(dr16_y);
            tmp_gimbal_pitch += dr16_r_y * DR16_Pitch_Angle_Resolution * Get_Nonlinear_Angle_Resolution(dr16_r_y);
        }

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

                if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
                    {
                        tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                        tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    }
                }
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
            //

            // F键按下 一键开关弹舱
            if (DR16.Get_Keyboard_Key_F() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Compare == 1700)
                {
                    Bulletcap_Status = Bulletcap_Status_CLOSE;
                    Compare = 400;
                }
                else
                {
                    Bulletcap_Status = Bulletcap_Status_OPEN;
                    Compare = 1700;
                }
            }
            // C键按下 一键调头
            if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
            }
            // V键按下 自瞄模式中切换四点和五点模式
            if (DR16.Get_Keyboard_Key_V() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                // if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
                // {
                //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
                // }
                // else
                // {
                //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
                // }
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
            // G键按下切换Pitch锁定模式和free模式
            if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Pitch_Control_Status == Pitch_Status_Control_Free)
                    Pitch_Control_Status = Pitch_Status_Control_Lock;
                else
                    Pitch_Control_Status = Pitch_Status_Control_Free;
            }
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
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);

                if (MiniPC.Get_MiniPC_Status() == MiniPC_Status_ENABLE)
                {
                    tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                    tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
                    {
                        tmp_gimbal_yaw = MiniPC.Get_Rx_Yaw_Angle();
                        tmp_gimbal_pitch = MiniPC.Get_Rx_Pitch_Angle();
                    }
                }
            }
            else
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            tmp_gimbal_yaw -= VT13.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
            tmp_gimbal_pitch += VT13.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;

            // F键按下 一键开关弹舱
            if (VT13.Get_Keyboard_Key_F() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Compare == 1700)
                {
                    Bulletcap_Status = Bulletcap_Status_CLOSE;
                    Compare = 400;
                }
                else
                {
                    Bulletcap_Status = Bulletcap_Status_OPEN;
                    Compare = 1700;
                }
            }
            // C键按下 一键调头
            if (VT13.Get_Keyboard_Key_C() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_gimbal_yaw += 180;
            }

            // Z键按下 切换反小陀螺开关
            // if (VT13.Get_Keyboard_Key_Z() == VT13_Key_Status_TRIG_FREE_PRESSED)
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
            // V键按下 自瞄模式中切换四点和五点模式
            if (VT13.Get_Keyboard_Key_V() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                // if (Gimbal.MiniPC->Get_MiniPC_Type() == MiniPC_Type_Windmill)
                // {
                //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Nomal);
                // }
                // else
                // {
                //     Gimbal.MiniPC->Set_MiniPC_Type(MiniPC_Type_Windmill);
                // }
            }
            // G键按下切换Pitch锁定模式和free模式
            if (VT13.Get_Keyboard_Key_G() == VT13_Key_Status_TRIG_FREE_PRESSED)
            {
                if (Pitch_Control_Status == Pitch_Status_Control_Free)
                    Pitch_Control_Status = Pitch_Status_Control_Lock;
                else
                    Pitch_Control_Status = Pitch_Status_Control_Free;
            }
        }
    }

    // 如果pitch为锁定状态

    // 设定角度

    // if (init_finish_flag)
    // {
    //     if (Gimbal.Get_Gimbal_Roll_Control_Type() == Gimbal_Roll_Control_Type_ROLL_LOCKED)
    //     {
    //         // 将地面系的姿态角解算为无人机系
    //         float roll = Gimbal.Boardc_BMI.Get_Rad_Roll();
    //         float yaw = tmp_gimbal_yaw * PI / 180;
    //         float pitch = tmp_gimbal_pitch * PI / 180;

    //         float x_w = cosf(pitch) * cosf(yaw);
    //         float y_w = sinf(pitch);
    //         float z_w = cosf(pitch) * sinf(yaw);

    //         float x_d = x_w;
    //         float y_d = y_w * cosf(roll) + z_w * sinf(roll);
    //         float z_d = -y_w * sinf(roll) + z_w * cosf(roll);

    //         __tmp_gimbal_yaw = atan2f(z_d, x_d) * 180 / PI;
    //         __tmp_gimbal_pitch = atan2f(y_d, sqrtf(x_d * x_d + z_d * z_d)) * 180 / PI;
    //     }
    //     else
    //     {
    //         __tmp_gimbal_yaw = tmp_gimbal_yaw;
    //         __tmp_gimbal_pitch = tmp_gimbal_pitch;
    //     }
    // }

    if (Pitch_Control_Status == Pitch_Status_Control_Lock)
        tmp_gimbal_pitch = 0;
    Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
}
#endif

/**
 * @brief 发射机构控制逻辑
 *
 */
#ifdef GIMBAL
Enum_Booster_Control_Type Test_Booster_Control_Type;
bool Booster_Init_Flag = 0;
void Class_Chariot::Control_Booster()
{
    // 先判断当前活动的控制器
    Test_Booster_Control_Type = Booster.Get_Booster_Control_Type();
    Judge_Active_Controller();

    /************************************遥控器控制逻辑*********************************************/
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 右上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            // if (Booster_Init_Flag = 0)
            // {
            //     Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            //     Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            //     Fric_Status = Fric_Status_OPEN;
            //     Booster_Init_Flag = 1;
            // }
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            Fric_Status = Fric_Status_OPEN;

            if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
            {                               // 自瞄模式火控 上位机控制打弹
                if (DR16.Get_Yaw() < -0.8f) // 连发
                {
                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                }
            }
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
                Shoot_Flag = 1;
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
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
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
            if (VT13.Get_Keyboard_Key_B() == VT13_Key_Status_TRIG_FREE_PRESSED)
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
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_SINGLE)
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_TRIG_FREE_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                    }
                }
                if (Booster.Booster_User_Control_Type == Booster_User_Control_Type_MULTI)
                {
                    if (VT13.Get_Mouse_Left_Key() == VT13_Key_Status_PRESSED)
                    {
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
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
    }
}
#endif

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
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback_1()
{
    uint16_t current_heat;
    current_heat = Referee.Get_Booster_17mm_1_Heat();
    memcpy(&CAN2_Chassis_Tx_Gimbal_Data_1[2], &current_heat, sizeof(uint16_t));
}
#endif
/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS
    // 计算云台与底盘的夹角（弧度制）
    float chassis_gimbal_diff = (Reference_Angle - Chassis_Angle);
    if (chassis_gimbal_diff <= 0)
    {
        chassis_gimbal_diff += 2 * PI;
    }

    // 将夹角信息存入JudgeReceiveData
    JudgeReceiveData.Chassis_Gimbal_Diff = chassis_gimbal_diff;

    // 小陀螺 随动计算角速度
    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN)
    {
        Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
    }
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {
        // 随动yaw角度优化
        float temp_yaw, temp_reference;
        temp_yaw = Chassis_Angle;
        temp_reference = Reference_Angle;

        // 将角度规范化到 [-π, π] 范围内
        if (temp_yaw > PI)
            temp_yaw = temp_yaw - 2 * PI;
        else if (temp_yaw < -PI)
            temp_yaw = temp_yaw + 2 * PI;

        if (temp_reference > PI)
            temp_reference = temp_reference - 2 * PI;
        else if (temp_reference < -PI)
            temp_reference = temp_reference + 2 * PI;

        // 计算角度差，选择最短路径（优弧）
        float angle_diff = temp_reference - temp_yaw;

        // 优弧劣弧判断与优化
        if (angle_diff > PI)
        {
            angle_diff -= 2 * PI;
        }
        else if (angle_diff < -PI)
        {
            angle_diff += 2 * PI;
        }

        // 使用优化后的角度差进行PID控制
        PID_Chassis_Fllow.Set_Target(temp_yaw + angle_diff);
        PID_Chassis_Fllow.Set_Now(temp_yaw);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        Chassis.Set_Target_Omega(PID_Chassis_Fllow.Get_Out());
    }

    static uint8_t mod2 = 0;
    // 各个模块的分别解算
    mod2++;
    if (mod2 == 2)
    {
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        mod2 = 0;
    }

    // 底盘解算任务

#elif defined(GIMBAL)

    // 各个模块的分别解算
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    // 传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();

#endif
}

/**
 * @brief 判断DR16控制数据来源
 *
 */
#ifdef GIMBAL
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
// Enum_DR16_Control_Type Class_Chariot::Get_DR16_Control_Type()
// {
//     if (Active_Controller == Controller_DR16)
//     {
//         return DR16_Control_Type;
//     }
//     else
//     {
//         return DR16_Control_Type_NONE;
//     }
// }

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
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
#ifdef CHASSIS

        for (auto &wheel : Chassis.Motor_Wheel)
        {
            wheel.TIM_Alive_PeriodElapsedCallback();
        }
        if (mod50_mod3 % 3 == 0)
        {
            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
        // 云台，随动掉线保护
        if (Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE || Gimbal_Status == Gimbal_Status_DISABLE)
        {
            buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#elif defined(GIMBAL)

        if (mod50_mod3 % 3 == 0)
        {
            // 判断底盘通讯在线状态
            TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
            VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
            MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }

        Gimbal.Motor_Pitch_J4310.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();

        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        // Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Down.TIM_Alive_PeriodElapsedCallback();
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
        if (huart3.ErrorCode)
        {
            HAL_UART_DMAStop(&huart3); // 停止以重启
            // HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
        }
    }
    else
    {
        // Gimbal.Set_Gimbal_Control_Type(Pre_Gimbal_Control_Type);
        // Chassis.Set_Chassis_Control_Type(Pre_Chassis_Control_Type);
    }

#endif

// 底盘离线保护
#ifdef CHASSIS

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
        Referee.Referee_Status = Referee_Status_DISABLE;
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        Referee.Referee_Status = Referee_Status_ENABLE;
        Chassis_Status = Chassis_Status_ENABLE;
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
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;
}
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    // 离线检测状态
    case (0):
    {
        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
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
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE && Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }

        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE || Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
        {
            Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
            Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
            Chariot->Booster.Set_Booster_Control_Type(Chariot->Get_Pre_Booster_Control_Type());
            // 这里可以直接设置成Normal模式
            Status[Now_Status_Serial].Time = 0;
            Set_Status(2);
        }

        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
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
        HAL_UART_DMAStop(&huart3); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

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
        if (huart6.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }

        // 转移为 在线状态
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
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
        if (huart6.ErrorCode)
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
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
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
        HAL_UART_DMAStop(&huart6); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
