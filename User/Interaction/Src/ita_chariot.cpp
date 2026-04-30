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
float last_gripper_value = 0.0f;
// 测试单圈设置函数用，测完删
float single_radian = 0.0f;

static uint32_t cal_cnt = 0;
float calculate_s;
/* Private function declarations ---------------------------------------------*/

#ifdef GIMBAL
static Enum_Controller_Key_Status To_Controller_Key_Status(Enum_DR16_Key_Status status)
{
    switch (status)
    {
    case DR16_Key_Status_PRESSED:
        return Controller_Key_Status_PRESSED;
    case DR16_Key_Status_TRIG_FREE_PRESSED:
        return Controller_Key_Status_TRIG_FREE_PRESSED;
    case DR16_Key_Status_TRIG_PRESSED_FREE:
        return Controller_Key_Status_TRIG_PRESSED_FREE;
    case DR16_Key_Status_FREE:
    default:
        return Controller_Key_Status_FREE;
    }
}

static Enum_Controller_Key_Status To_Controller_Key_Status(Enum_VT13_Key_Status status)
{
    switch (status)
    {
    case VT13_Key_Status_PRESSED:
        return Controller_Key_Status_PRESSED;
    case VT13_Key_Status_TRIG_FREE_PRESSED:
        return Controller_Key_Status_TRIG_FREE_PRESSED;
    case VT13_Key_Status_TRIG_PRESSED_FREE:
        return Controller_Key_Status_TRIG_PRESSED_FREE;
    case VT13_Key_Status_FREE:
    default:
        return Controller_Key_Status_FREE;
    }
}

static bool Key_Is_Pressed(Enum_Controller_Key_Status status)
{
    return status == Controller_Key_Status_PRESSED;
}

static bool Key_Is_Trig_Pressed_Free(Enum_Controller_Key_Status status)
{
    return status == Controller_Key_Status_TRIG_PRESSED_FREE;
}

static bool Key_Is_Shift_Active(Enum_Controller_Key_Status status)
{
    return status == Controller_Key_Status_PRESSED ||
           status == Controller_Key_Status_TRIG_FREE_PRESSED ||
           status == Controller_Key_Status_TRIG_PRESSED_FREE;
}

static float Apply_Dead_Zone(float value, float dead_zone)
{
    return (Math_Abs(value) > dead_zone) ? value : 0.0f;
}

static float Build_Mouse_Right_X(float mouse_x, float resolution)
{
    float right_x = fabs(mouse_x * resolution);

    if (right_x >= 0.2f)
    {
        right_x = 1.0f;
    }

    if (mouse_x < 0.0f)
    {
        right_x = -right_x;
    }

    return right_x;
}

static void Load_Custom_Controller_Targets(const Struct_Custom_Controller_Data &data,
                                           float &j0_pitch,
                                           float &j1_yaw,
                                           float &j2_yaw,
                                           float &j3_roll,
                                           float &j4_pitch,
                                           float &j5_roll,
                                           uint8_t &gripper_position)
{
    j0_pitch = data.Angle[0];
    j1_yaw = data.Angle[1];
    j2_yaw = data.Angle[2];
    j3_roll = data.Angle[3];
    j4_pitch = data.Angle[4];
    j5_roll = data.Angle[5];
    gripper_position = data.gripper_status ? 255 : 0;
}
#endif

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
#ifdef CHASSIS

    // 裁判系统
    Referee.Init(&huart10);

    // 底盘
    Chassis.Referee = &Referee;
    // 限速，暂时给到2m/s ， 1.75m/s和 4 rad/s
    Chassis.Init(2.0f, 2.0f, 4.0f);
    // 力控底盘
    Force_Chassis.Init();

    // 超电
    Chassis.Supercap.Referee = &Referee;

#elif defined(GIMBAL)

    Chassis.Set_Velocity_X_Max(4.0f);
    Chassis.Set_Velocity_Y_Max(4.0f);

    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);

// 遥控器
#ifdef USE_DR16
    DR16.Init(&huart5, &huart1);
    DR16_Dead_Zone = __DR16_Dead_Zone;
#endif

#ifdef USE_VT13
    FSM_Alive_Control_VT13.Chariot = this;
    FSM_Alive_Control_VT13.Init(5, 0);
#endif

    Active_Controller = Controller_NONE;

    // 云台
    Gimbal.Init();
    Gimbal.MiniPC = &MiniPC;

    // 发射机构
    Booster.Init();
    Booster.MiniPC = &MiniPC;

    // 上位机
    MiniPC.Init(&MiniPC_USB_Manage_Object, &UART8_Manage_Object, &CAN3_Manage_Object);
    MiniPC.IMU = &Gimbal.Boardc_BMI;
    MiniPC.Referee = &Referee;

    // 存取矿状态机
    FSM_Save_Load.Gimbal = &this->Gimbal;

#endif

#ifdef CHASSIS_TEST
    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);
    DR16.Init(&huart5, &huart1);
    DR16_Dead_Zone = __DR16_Dead_Zone;
#endif

    // 蜂鸣器
    buzzer_init_example();
}

#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    // 检查到裁判系统存活时通过CAN发送自定义控制器角度数据
    if (Referee.Get_Referee_Status() == Referee_Status_ENABLE)
    {
        memcpy(&CAN3_Controller_Tx_Data_A, &Referee.Interaction_Custom_Controller.Data[0], 8);
        memcpy(&CAN3_Controller_Tx_Data_B, &Referee.Interaction_Custom_Controller.Data[8], 8);
    }
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
#ifdef CHASSIS
// Struct_CAN_Referee_Rx_Data_t CAN_Referee_Rx_Data;
// 控制类型字节
uint8_t control_type;

void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{
    Gimbal_Alive_Flag++;

    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 目标角速度
    float chassis_delta_radian;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    // 抬升控制相关
    bool lift_select[4] = {false};
    Enum_Lift_Direction lift_drc[4];
    // 上台阶状态机方向
    Enum_Uplift_FSM_Direction uplift_fsm_drc;
    // 小轮子从动开关
    Enum_Wheel_Slave_Status wheels_slave_status;
    // 150mm后门开关
    bool backdoor_status;
    // 下台阶初始化开关
    bool downlift_status;

    switch (CAN_Manage_Object->Rx_Buffer.Header.Identifier)
    {
    // 底盘控制数据回传
    case (0x77):
    {
        memcpy(&Rx_Frame, Rx_Data, sizeof(Rx_Frame));

#ifdef AGV
        chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x, -450, 450, -4, 4);
        chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y, -450, 450, -4, 4);
        chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.f, 4.f) / Chassis_Radius; // 映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
#else
        chassis_velocity_x = -Math_Int_To_Float(Rx_Frame.x_velocity, -450, 450, -4.f, 4.f);
        chassis_velocity_y = -Math_Int_To_Float(Rx_Frame.y_velocity, -450, 450, -4.f, 4.f);
        chassis_delta_radian = -Math_Int_To_Float(Rx_Frame.yaw_data, -200, 200, -1.f, 1.f);
#endif

        for (uint8_t i = 0; i < 4; ++i)
        {
            lift_select[i] = (Rx_Frame.lift.lift_select >> i) & 0x01;
            if (lift_select[i])
            {
                lift_drc[i] = ((Rx_Frame.lift.lift_direction >> i) & 0x01)
                                  ? Lift_Direction_UP
                                  : Lift_Direction_DOWN;
            }
            else
            {
                lift_drc[i] = Lift_Direction_HOLD;
            }
        }
        chassis_control_type = (Enum_Chassis_Control_Type)Rx_Frame.status.chassis_contorl_mode;
        uplift_fsm_drc = (Enum_Uplift_FSM_Direction)Rx_Frame.status.uplift_fsm_direction;
        wheels_slave_status = (Enum_Wheel_Slave_Status)Rx_Frame.status.wheel_slave_ctrl;
        backdoor_status = (bool)Rx_Frame.status.backdoor_jump;
        downlift_status = (bool)Rx_Frame.status.downlift_init;

        // 设定底盘控制类型
        Chassis.Set_Chassis_Control_Type(chassis_control_type);
        // 设定底盘目标速度
        Chassis.Set_Target_Velocity_X(chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
#ifdef OMNI_WHEEL
        Chassis.Set_Target_Velocity_X(-chassis_velocity_x);
#endif
        Chassis.Set_Delta_Radian(chassis_delta_radian); // 目标角度增量
        Chassis.Set_Uplift_FSM_Direction(uplift_fsm_drc);
        Chassis.Set_Lift_Select(lift_select[0], lift_select[1], lift_select[2], lift_select[3]);
        for (uint8_t i = 0; i < 4; i++)
        {
            Chassis.Set_Lift_Direction(i, lift_drc[i]);
        }
        Chassis.Set_Wheel_Slave_Status(wheels_slave_status);
        Chassis.Set_Backdoor_Jump(backdoor_status);
        Chassis.Set_Downlift_Init(downlift_status);

        Control_Chassis();

        break;
    }
    case (0x95):
        // 裁判系统数据回传
        {
            // memcpy(&CAN_Referee_Rx_Data, &CAN_Manage_Object->Rx_Buffer.Data, sizeof(Struct_CAN_Referee_Rx_Data_t));
            break;
        }
    }
}
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
Referee_Rx_A_t PRE_CAN3_Chassis_Rx_Data_A;
Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
Referee_Rx_D_t CAN3_Chassis_Rx_Data_D;
Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
Referee_Rx_F_t CAN3_Chassis_Rx_Data_F;
Referee_Rx_G_t CAN3_Chassis_Rx_Data_G;
float speed_a, speed_b;
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
    switch (CAN_Manage_Object->Rx_Buffer.Header.Identifier)
    {
    // 自定义控制器A包
    case (0x81):
    {
        memcpy(&Controller_Buffer, &CAN_Manage_Object->Rx_Buffer.Data, 8);
        break;
    }
    // 自定义控制器B包
    case (0x82):
    {
        memcpy(&Controller_Buffer, &CAN_Manage_Object->Rx_Buffer.Data, 7);
        break;
    }
    }
}
#endif

/**
 * @brief can回调函数给底盘发送数据
 * Data[0], Data[1]：X轴速度
 * Data[2]，Data[3]：Y轴速度
 * Data[4]，Data[5]：Yaw速度/角度
 * Data[6]: 0-3位控制四个抬升，4-7位控制升高/降低
 * Data[7]：0-1位：底盘控制模式
 *          第2位：上台阶状态机前进/保持
 *          第3位：150mm台阶后门跳转开关
 *          第4位：下台阶初始抬升高度跳转
 *          第5位：小轮子从动开关
 *          第6位：保留
 *          第7位：保留
 */
#ifdef GIMBAL
// 控制类型字节
uint8_t control_type;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    // 发送的数据帧
    Gimbal_Tx_Chassis_Frame Tx_Frame;
    // 映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_radian = 0;
    // 抬升控制字节
    uint8_t select_bits = 0;
    uint8_t direction_bits = 0;

    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    // 底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, chassis_radian = 0;
    // 抬升控制选择
    bool uplift_select[4];
    // 抬升方向设置
    Enum_Lift_Direction uplift_direction[4];

    // 控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_radian = Chassis.Get_Target_Omega();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    for (uint8_t i = 0; i < 4; i++)
    {
        uplift_select[i] = Chassis.Get_Lift_Select(i);
        uplift_direction[i] = Chassis.Get_Lift_Direction(i);
    }

    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -4.f, 4.f, -450, 450);
    Tx_Frame.x_velocity = tmp_chassis_velocity_x;

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -4.f, 4.f, -450, 450);
    Tx_Frame.y_velocity = tmp_chassis_velocity_y;

    tmp_chassis_radian = -Math_Float_To_Int(chassis_radian, -1.f, 1.f, -200, 200);
    Tx_Frame.yaw_data = tmp_chassis_radian;

    // 抬升控制打包
    for (uint8_t i = 0; i < 4; i++)
    {
        if (uplift_select[i])
            select_bits |= (1 << i);
        if (uplift_direction[i] == Lift_Direction_UP)
            direction_bits |= (1 << i);
    }
    Tx_Frame.lift.lift_select = select_bits;
    Tx_Frame.lift.lift_direction = direction_bits;

    // 其他状态数据打包
    Tx_Frame.status.backdoor_jump = (Chassis.Get_Backdoor_Jump() ? 1 : 0);     // 1/8
    Tx_Frame.status.downlift_init = (Chassis.Get_Downlift_Init() ? 1 : 0);     // 2/8
    Tx_Frame.status.chassis_contorl_mode = Chassis.Get_Chassis_Control_Type(); // 4/8
    Tx_Frame.status.uplift_fsm_direction = Chassis.Get_Uplift_FSM_Direction(); // 5/8
    Tx_Frame.status.wheel_slave_ctrl = Chassis.Get_Wheel_Slave_Status();       // 6/8

    // 装入CAN发送缓冲区
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &Tx_Frame, sizeof(Tx_Frame));
}

#endif
/**
 * @brief 底盘控制逻辑
 *
 */
float Offset_K = 0.175f;
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    float chassis_velocity_x = 0.0f;
    float chassis_velocity_y = 0.0f;
    float chassis_delta_radian = 0.0f;
    bool lift_select[4] = {false, false, false, false};
    Enum_Lift_Direction lift_drc[4] = {Lift_Direction_HOLD, Lift_Direction_HOLD, Lift_Direction_HOLD, Lift_Direction_HOLD};
    Enum_Wheel_Slave_Status wheel_slave_status = Chassis.Get_Wheel_Slave_Status();
    Enum_Uplift_FSM_Direction uplift_fsm_dir = Uplift_FSM_HOLD;
    bool backdoor_status = false;
    bool downlift_init = false;
    float move_left_x = left_x;
    float move_left_y = left_y;
    float move_right_x = right_x;

    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        switch (controller_dr16_left_switch)
        {
        case DR16_Switch_Status_UP:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            break;
        case DR16_Switch_Status_MIDDLE:
            Chassis.Set_Chassis_Control_Type(Gimbal.arm_init ? Chassis_Control_Type_NORMAL : Chassis_Control_Type_DISABLE);
            if (controller_dr16_right_switch == DR16_Switch_Status_UP)
            {
                lift_select[0] = true;
                lift_select[1] = true;
                lift_drc[0] = Lift_Direction_UP;
                lift_drc[1] = Lift_Direction_UP;
            }
            else if (controller_dr16_right_switch == DR16_Switch_Status_DOWN)
            {
                lift_select[0] = true;
                lift_select[1] = true;
                lift_drc[0] = Lift_Direction_DOWN;
                lift_drc[1] = Lift_Direction_DOWN;
            }
            break;
        case DR16_Switch_Status_DOWN:
            Chassis.Set_Chassis_Control_Type(Gimbal.arm_init ? Chassis_Control_Type_SLOPE : Chassis_Control_Type_DISABLE);
            break;
        default:
            break;
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        switch (controller_vt13_switch)
        {
        case VT13_Switch_Status_Left:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            break;
        case VT13_Switch_Status_Middle:
            Chassis.Set_Chassis_Control_Type(Gimbal.arm_init ? Chassis_Control_Type_NORMAL : Chassis_Control_Type_DISABLE);
            break;
        case VT13_Switch_Status_Right:
            Chassis.Set_Chassis_Control_Type(Gimbal.arm_init ? Chassis_Control_Type_SLOPE : Chassis_Control_Type_DISABLE);
            break;
        default:
            break;
        }
    }
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        move_left_x = 0.0f;
        move_left_y = 0.0f;
        move_right_x = 0.0f;

        if (Key_Is_Pressed(controller_key_w))
        {
            move_left_y = 1.0f;
        }
        else if (Key_Is_Pressed(controller_key_s))
        {
            move_left_y = -1.0f;
        }

        if (Key_Is_Pressed(controller_key_a))
        {
            move_left_x = -1.0f;
        }
        else if (Key_Is_Pressed(controller_key_d))
        {
            move_left_x = 1.0f;
        }

        if (controller_mouse_right_key == Controller_Key_Status_FREE)
        {
            move_right_x = Build_Mouse_Right_X(controller_mouse_x, Mouse_Resolution);
        }

        switch (Keyboard_Control_Type)
        {
        case Keyboard_Control_Type_DISABLE:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            move_left_x = 0.0f;
            move_left_y = 0.0f;
            move_right_x = 0.0f;
            break;
        case Keyboard_Control_Type_WORKING:
        case Keyboard_Control_Type_SAVE_LOAD:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
            if (!Key_Is_Pressed(controller_key_shift))
            {
                move_left_x *= 0.5f;
                move_left_y *= 0.5f;
                move_right_x *= 0.5f;
            }
            if (Key_Is_Pressed(controller_key_q))
            {
                lift_select[0] = true;
                lift_drc[0] = Lift_Direction_UP;
            }
            else if (Key_Is_Pressed(controller_key_e))
            {
                lift_select[0] = true;
                lift_drc[0] = Lift_Direction_DOWN;
            }
            break;
        case Keyboard_Control_Type_MOVING:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
            if (Key_Is_Pressed(controller_key_shift))
            {
                move_left_x *= 0.5f;
                move_left_y *= 0.5f;
                move_right_x *= 0.5f;
            }
            break;
        case Keyboard_Control_Type_UPLIFT:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SLOPE);
            if (Key_Is_Trig_Pressed_Free(controller_key_ctrl))
            {
                wheel_slave_status = (wheel_slave_status == Wheel_Slave_OFF) ? Wheel_Slave_ON : Wheel_Slave_OFF;
            }
            if (Key_Is_Pressed(controller_key_r))
            {
                backdoor_status = true;
            }
            if (Key_Is_Pressed(controller_key_shift) && Key_Is_Pressed(controller_key_c))
            {
                uplift_fsm_dir = Uplift_FSM_FORWARD;
            }
            break;
        case Keyboard_Control_Type_DOWNLIFT:
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
            if (!Key_Is_Pressed(controller_key_shift))
            {
                move_left_x *= 0.5f;
                move_left_y *= 0.5f;
                move_right_x *= 0.5f;
            }
            if (Key_Is_Pressed(controller_key_q))
            {
                lift_select[0] = true;
                lift_select[1] = true;
                lift_drc[0] = Lift_Direction_UP;
                lift_drc[1] = Lift_Direction_UP;
            }
            else if (Key_Is_Pressed(controller_key_e))
            {
                lift_select[0] = true;
                lift_select[1] = true;
                lift_drc[0] = Lift_Direction_DOWN;
                lift_drc[1] = Lift_Direction_DOWN;
            }
            if (Key_Is_Pressed(controller_key_f))
            {
                lift_select[2] = true;
                lift_select[3] = true;
                lift_drc[2] = Lift_Direction_UP;
                lift_drc[3] = Lift_Direction_UP;
            }
            else if (Key_Is_Pressed(controller_key_g))
            {
                lift_select[2] = true;
                lift_select[3] = true;
                lift_drc[2] = Lift_Direction_DOWN;
                lift_drc[3] = Lift_Direction_DOWN;
            }
            if (Key_Is_Trig_Pressed_Free(controller_key_ctrl))
            {
                wheel_slave_status = (wheel_slave_status == Wheel_Slave_OFF) ? Wheel_Slave_ON : Wheel_Slave_OFF;
            }
            break;
        }
    }
    else
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }

    left_x = move_left_x;
    left_y = move_left_y;
    right_x = move_right_x;

    switch (Chassis.Get_Chassis_Control_Type())
    {
    case Chassis_Control_Type_DISABLE:
        chassis_velocity_x = 0.0f;
        chassis_velocity_y = 0.0f;
        chassis_delta_radian = 0.0f;
        break;
    case Chassis_Control_Type_NORMAL:
    case Chassis_Control_Type_SLOPE:
        chassis_velocity_x = left_y * sqrt(1.0f - left_y * left_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = -left_x * sqrt(1.0f - left_x * left_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_delta_radian = -right_x * sqrt(1.0f - right_x * right_x / 2.0f) * 0.01f;
        break;
    }

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
    Chassis.Set_Target_Omega(chassis_delta_radian);
    Chassis.Set_Lift_Select(lift_select[0], lift_select[1], lift_select[2], lift_select[3]);
    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.Set_Lift_Direction(i, lift_drc[i]);
    }
    Chassis.Set_Backdoor_Jump(backdoor_status);
    Chassis.Set_Downlift_Init(downlift_init);
    Chassis.Set_Uplift_FSM_Direction(uplift_fsm_dir);
    Chassis.Set_Wheel_Slave_Status(wheel_slave_status);
}
#elifdef CHASSIS_TEST
void Class_Chariot::Chassis_Test_Control()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_r_x, dr16_r_y;
    // 底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float target_uplift_rad[4] = {0.0f};
    float track_omega = 0.0f;

    float chassis_radian = Force_Chassis.Get_Target_Radian();

    // 获取当前的抬升机构高度用于做增量
    for (int i = 0; i < 4; i++)
    {
        target_uplift_rad[i] = Chassis.Get_Target_Uplift_Radian(i);
    }

    // 排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
    dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

    dr16_l_x = -dr16_l_x;
    dr16_l_y = -dr16_l_y;
    dr16_r_x = dr16_r_x;
    dr16_r_y = -dr16_r_y;

    // 遥控器操作逻辑
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    volatile int DR16_Right_Switch_Status = DR16.Get_Right_Switch();
    switch (DR16_Left_Switch_Status)
    {
    case (DR16_Switch_Status_MIDDLE): // 左中 底盘正常控制模式，最大速度为底盘初始化设置的速度
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
        Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);
        break;
    }
    case (DR16_Switch_Status_DOWN): // 左下 上台阶状态机 底盘轮组最大速度设置成1.0f
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SLOPE);
        Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);
        break;
    }
    default:
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
        break;
    }
    }

    // 力控底盘控制逻辑
    volatile int Force_Chassis_control_type = Force_Chassis.Get_Chassis_Control_Type();
    switch (Force_Chassis_control_type)
    {
    case (Chassis_Control_Type_DISABLE__):
    { // 失能
        chassis_velocity_x = 0;
        chassis_velocity_y = 0;
        chassis_radian = chassis_radian;
        track_omega = 0.0f;
        break;
    }
    case (Chassis_Control_Type_NORMAL__):
    {
        if (DR16_Left_Switch_Status == DR16_Switch_Status_DOWN)
        // 左下低速模式
        {
            chassis_velocity_y = -dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * 1.0f;
            chassis_velocity_x = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * 1.0f;
            chassis_radian += -dr16_r_x * sqrt(1.0f - dr16_r_x * dr16_r_x / 2.0f) * 0.005f;

            track_omega = dr16_r_y * sqrt(1.0f - dr16_r_y * dr16_r_y / 2.0f) * 25.0f;
        }
        else if (DR16_Left_Switch_Status == DR16_Switch_Status_MIDDLE)
        // 左中正常速度
        {
            // 设定矩形到圆形映射进行控制，velocity_x为前，velocity_y为左
            chassis_velocity_y = -dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
            chassis_velocity_x = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
            chassis_radian += -dr16_r_x * sqrt(1.0f - dr16_r_x * dr16_r_x / 2.0f) * 0.01f;

            track_omega = 0.0f;
        }
        else
        // 其他跳变状态
        {
            chassis_velocity_x = 0.0f;
            chassis_velocity_y = 0.0f;
            chassis_radian = chassis_radian;
            track_omega = 0.0f;
        }
        break;
    }
    }

    if (chassis_radian > PI)
        chassis_radian -= 2 * PI;
    if (chassis_radian < -PI)
        chassis_radian += 2 * PI;

    Force_Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Force_Chassis.Set_Target_Velocity_Y(chassis_velocity_y); // 前x左y正
    Force_Chassis.Set_Target_Radian(chassis_radian);

    Chassis.Set_Target_Track_Omega(track_omega);

    // 原底盘类中履带和抬升的控制逻辑
    volatile int Chassis_control_type = Chassis.Get_Chassis_Control_Type();
    switch (Chassis_control_type)
    {
    case (Chassis_Control_Type_DISABLE):
    { // 失能

        break;
    }
    case (Chassis_Control_Type_SLOPE):
    {
        // 上台阶状态机，右摇杆不控制抬升机构，将DR16_Right的状态传给状态机变量并运行状态机
        Chassis.Ledder_FSM.DR16_Right = (Enum_DR16_Switch_Status)DR16_Right_Switch_Status;
        Chassis.Ledder_FSM.Yaw = DR16.Get_Yaw();
        Chassis.Ledder_FSM.Reload_TIM_Status_PeriodElapsedCallback();
        Chassis.Ledder_FSM.DR16_Pre_Right = (Enum_DR16_Switch_Status)DR16_Right_Switch_Status;
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        // 上台阶状态机状态清零
        Chassis.Ledder_FSM.Set_Status(0);

        // 底盘在线状态下遥控器控制抬升机构

        // 抬升机构控制逻辑
        switch (DR16_Right_Switch_Status)
        {
        case (DR16_Switch_Status_UP):
            // 前左抬升高度控制
            {
                target_uplift_rad[0] += PI * 0.003f * dr16_right_y;
                break;
            }

        case (DR16_Switch_Status_DOWN):
            // 后两个抬升高度控制
            {
                target_uplift_rad[2] += PI * 0.003f * dr16_right_y;
                target_uplift_rad[3] += PI * 0.003f * dr16_right_y;
                break;
            }

        case (DR16_Switch_Status_MIDDLE):
        {
            target_uplift_rad[1] += PI * 0.003f * dr16_right_y;
            break;
        }

        default:
        {
        }
        }

        for (int i = 0; i < 4; i++)
        {
            Chassis.Set_Target_Uplift_Radian(i, target_uplift_rad[i]);
        }
    }
    }
}
#endif

#ifdef CHASSIS
bool low_height_flag = false;
static uint8_t flag_count = 0;
void Class_Chariot::Control_Chassis()
{
    // 底盘坐标系速度目标值 float
    float chassis_velocity_x = 0.0f, chassis_velocity_y = 0.0f;
    float chassis_radian = Force_Chassis.Get_Target_Radian();
    float track_omega = 0.0f;
    float target_uplift_rad[4] = {0.0f};

    // 获取当前的抬升机构高度用于做增量
    for (int i = 0; i < 4; i++)
    {
        target_uplift_rad[i] = Chassis.Get_Target_Uplift_Radian(i);
    }

    volatile int Chassis_Control_Type = Chassis.Get_Chassis_Control_Type();
    volatile int DR16_Right_Uplift_Status = Chassis.Get_DR16_Right_Uplift_Status();
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
        break;
    }

    case (Chassis_Control_Type_NORMAL):
    case (Chassis_Control_Type_SLOPE):
    {
        Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);

        if (Chassis.Get_Wheel_Slave_Status() == Wheel_Slave_ON)
        {
            // 机械安装方向相反，因此乘以一个负号
            track_omega = -1.0f * Force_Chassis.Get_Target_Velocity_X();
        }
        else
        {
            track_omega = 0.0f;
        }

        break;
    }
    }

    // 力控底盘控制逻辑
    volatile int Force_Chassis_control_type = Force_Chassis.Get_Chassis_Control_Type();
    switch (Force_Chassis_control_type)
    {
    case (Chassis_Control_Type_DISABLE__):
    { // 失能
        chassis_velocity_x = 0;
        chassis_velocity_y = 0;
#ifdef RADIAN_CONTROL
        chassis_radian = chassis_radian;
#elifdef OMEGA_CONTROL
        chassis_radian = 0;
#endif

        break;
    }
    case (Chassis_Control_Type_NORMAL__):
    {
        chassis_velocity_x = Chassis.Get_Target_Velocity_X();
        chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
#ifdef RADIAN_CONTROL
        chassis_radian += Chassis.Get_Delta_Radian();
#elifdef OMEGA_CONTROL
        chassis_radian = Chassis.Get_Delta_Radian() * 300.0f;
#endif

        break;
    }
    }
#ifdef RADIAN_CONTROL
    if (chassis_radian > PI)
        chassis_radian -= 2 * PI;
    if (chassis_radian < -PI)
        chassis_radian += 2 * PI;
#elifdef OMEGA_CONTROL
    if (chassis_radian > 4.0f)
        chassis_radian = 4.0f;
    if (chassis_radian < -4.0f)
        chassis_radian = -4.0f;
#endif

    Force_Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Force_Chassis.Set_Target_Velocity_Y(chassis_velocity_y); // 前x左y正
#ifdef RADIAN_CONTROL
    Force_Chassis.Set_Target_Radian(chassis_radian);
#elifdef OMEGA_CONTROL
    Force_Chassis.Set_Target_Omega(chassis_radian);
#endif

    Chassis.Set_Target_Track_Omega(track_omega);

    // 原底盘类中履带和抬升的控制逻辑
    volatile int Chassis_control_type = Chassis.Get_Chassis_Control_Type();
    switch (Chassis_control_type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 失能
        // 下台阶状态机状态归零
        Chassis.Ledder_FSM.Set_Status(0);
        break;
    }
    case (Chassis_Control_Type_SLOPE):
    {
        // 上台阶状态机，右摇杆不控制抬升机构，将DR16_Right的状态传给状态机变量并运行状态机
        // 150mm台阶后门
        if (Chassis.Get_Backdoor_Jump())
        {
            Chassis.Ledder_FSM.Set_Status(3);
            Chassis.Ledder_FSM.TRIGGER_CNT = 3;
        }
        // 台阶状态机执行函数
        Chassis.Ledder_FSM.Reload_TIM_Status_PeriodElapsedCallback();
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        // 下台阶状态机状态归零
        Chassis.Ledder_FSM.Set_Status(0);
        for (int i = 0; i < 4; i++)
        {
            // 如果按下下台阶初始化状态，则把下台阶的角度赋值给目标角度
            if (Chassis.Get_Downlift_Init())
            {
                target_uplift_rad[i] = Chassis.Downlift_Touch_Radian[i];
            }

            // 如果使用按键控制特定抬升的方向，则作为增量加给目标角度
            if (Chassis.Get_Lift_Select(i))
            {
                switch (Chassis.Get_Lift_Direction(i))
                {
                case (Lift_Direction_HOLD):
                {
                    break;
                }
                case (Lift_Direction_UP):
                {
                    target_uplift_rad[i] += PI * 0.03f;
                    break;
                }
                case (Lift_Direction_DOWN):
                {
                    target_uplift_rad[i] -= PI * 0.03f;
                    break;
                }
                }
            }

            // 赋值给目标角度
            Chassis.Set_Target_Uplift_Radian(i, target_uplift_rad[i]);
        }
    }
    }
}
#endif

/**
 * @brief 鼠标数据转换
 *
 */
#ifdef GIMBAL
void Class_Chariot::Transform_Mouse_Axis()
{
    True_Mouse_X = -DR16.Get_Mouse_X();
    True_Mouse_Y = DR16.Get_Mouse_Y();
    True_Mouse_Z = DR16.Get_Mouse_Z();
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL

void Class_Chariot::Control_Gimbal()
{
    float tmp_vt03_pitch = Gimbal.Get_Target_VT03_Pitch_Angle();
    float tmp_vt03_yaw = Gimbal.Get_Target_VT03_Yaw_Angle();
    tmp_j0_pitch_radian = Gimbal.Get_Target_J0_Pitch_Radian();
    tmp_j1_yaw_radian = Gimbal.Get_Target_J1_Yaw_Radian();
    tmp_j2_yaw_radian = Gimbal.Get_Target_J2_Yaw_Radian();
    tmp_j3_roll_radian = Gimbal.Get_Target_J3_Roll_Radian_In_PI();
    tmp_j4_pitch_radian = Gimbal.Get_Target_J4_Pitch_Radian_In_PI();
    tmp_j5_roll_radian = Gimbal.Get_Target_J5_Roll_Radian();
    tmp_gripper_position = Gimbal.Get_Target_Gripper_Position();

    Judge_Gimbal_Control_Type();

    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        if (controller_dr16_left_switch == DR16_Switch_Status_MIDDLE)
        {
            if (yaw >= 0.80f)
            {
                tmp_gripper_position = 0;
            }
            else if (yaw <= -0.80f)
            {
                tmp_gripper_position = 255;
            }
        }
        else if (controller_dr16_left_switch == DR16_Switch_Status_DOWN &&
                 controller_dr16_right_switch == DR16_Switch_Status_MIDDLE)
        {
            Load_Custom_Controller_Targets(Offline_Custom_Controller.Custom_Controller_Data,
                                           tmp_j0_pitch_radian,
                                           tmp_j1_yaw_radian,
                                           tmp_j2_yaw_radian,
                                           tmp_j3_roll_radian,
                                           tmp_j4_pitch_radian,
                                           tmp_j5_roll_radian,
                                           tmp_gripper_position);
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        if (controller_vt13_switch == VT13_Switch_Status_Middle)
        {
            if (yaw >= 0.80f)
            {
                tmp_gripper_position = 0;
            }
            else if (yaw <= -0.80f)
            {
                tmp_gripper_position = 255;
            }
        }
        else if (controller_vt13_switch == VT13_Switch_Status_Right)
        {
            Load_Custom_Controller_Targets(VT13.Custom_Controller.Custom_Controller_Data,
                                           tmp_j0_pitch_radian,
                                           tmp_j1_yaw_radian,
                                           tmp_j2_yaw_radian,
                                           tmp_j3_roll_radian,
                                           tmp_j4_pitch_radian,
                                           tmp_j5_roll_radian,
                                           tmp_gripper_position);
        }
    }
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        switch (Keyboard_Control_Type)
        {

        case Keyboard_Control_Type_WORKING:
            if (Active_Controller == Controller_DR16)
            {
                Load_Custom_Controller_Targets(Offline_Custom_Controller.Custom_Controller_Data,
                                               tmp_j0_pitch_radian,
                                               tmp_j1_yaw_radian,
                                               tmp_j2_yaw_radian,
                                               tmp_j3_roll_radian,
                                               tmp_j4_pitch_radian,
                                               tmp_j5_roll_radian,
                                               tmp_gripper_position);
            }
            else
            {
                Load_Custom_Controller_Targets(VT13.Custom_Controller.Custom_Controller_Data,
                                               tmp_j0_pitch_radian,
                                               tmp_j1_yaw_radian,
                                               tmp_j2_yaw_radian,
                                               tmp_j3_roll_radian,
                                               tmp_j4_pitch_radian,
                                               tmp_j5_roll_radian,
                                               tmp_gripper_position);
            }
            break;

        case Keyboard_Control_Type_SAVE_LOAD:
        {
            bool step = Key_Is_Trig_Pressed_Free(controller_mouse_right_key);
            FSM_Save_Load.Reload_TIM_Status_PeriodElapsedCallback(Save_Load_Unit, step);

            if (Key_Is_Trig_Pressed_Free(controller_mouse_left_key))
            {
                tmp_gripper_position = (tmp_gripper_position > 0) ? 0 : 255;
            }
            break;
        }

        case Keyboard_Control_Type_MOVING:
        case Keyboard_Control_Type_UPLIFT:
        case Keyboard_Control_Type_DOWNLIFT:
        case Keyboard_Control_Type_DISABLE:
        default:
            break;
        }

        if (Key_Is_Pressed(controller_mouse_right_key))
        {
            tmp_vt03_yaw += controller_mouse_x;
            tmp_vt03_pitch += controller_mouse_y;
        }
    }

    if (Keyboard_Control_Type != Keyboard_Control_Type_SAVE_LOAD)
    {
        FSM_Save_Load.Set_Status(0);
        Gimbal.Set_Target_J0_Pitch_Radian(tmp_j0_pitch_radian);
        Gimbal.Set_Target_J1_Yaw_Radian(tmp_j1_yaw_radian);
        Gimbal.Set_Target_J2_Yaw_Radian(tmp_j2_yaw_radian);
        Gimbal.Set_Target_J3_Roll_Radian(tmp_j3_roll_radian);
        Gimbal.Set_Target_J4_Pitch_Radian(tmp_j4_pitch_radian);
        Gimbal.Set_Target_J5_Roll_Radian(tmp_j5_roll_radian);
    }

    Gimbal.Set_Target_Gripper_Position(tmp_gripper_position);
    Gimbal.Set_Target_VT03_Pitch_Angle(tmp_vt03_pitch);
    Gimbal.Set_Target_VT03_Yaw_Angle(tmp_vt03_yaw);
}
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
int Booster_Sign = 0;
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    static uint8_t booster_sign = 0;
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    switch (DR16_Left_Switch_Status)
    {
    case (DR16_Switch_Status_MIDDLE): // 左中 失能
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
        break;
    }
    case (DR16_Switch_Status_DOWN): // 左下 上位机
    {
    }
    }
}
#endif

/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS

#ifdef CHASSIS_TEST // 底盘测试用
    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    // 遥控器关闭，所有电机输出置0
    {
        for (int i = 0; i < 4; i++)
        {
            // Chassis.Mecanum_Wheels[i].Set_Out(0.0f);
            Force_Chassis.Motor_Wheel[i].Set_Target_Current(0.0f);
        }

        Chassis.Uplift_Motor[0].Set_Out(0.0f);
        Chassis.Uplift_Motor[1].Set_Out(0.0f);
        Chassis.Uplift_Motor[2].Set_Out(0.0f);

        Chassis.Track_Motor[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Chassis.Track_Motor[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
    else
    {
        // 原底盘的定时器回调，主要用于获取当前功率以及抬升机构和履带类的PID计算
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);

        static uint8_t ms_cnt = 0;
        ms_cnt++;
        if (ms_cnt % 2 == 0)
        // 力控底盘定时器回调，轮组电机的PID
        {
            Force_Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
            Force_Chassis.TIM_2ms_Control_PeriodElapsedCallback();
            ms_cnt = 0;
        }
    }
#else // 底盘给云台发消息
    CAN_Chassis_Tx_Gimbal_Callback();

    // 云台，随动掉线保护
    if (Get_Gimbal_Status() == DR16_Status_ENABLE || Referee.Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);

        static uint8_t ms_cnt = 0;
        ms_cnt++;
        if (ms_cnt % 2 == 0)
        // 力控底盘定时器回调，轮组电机的PID
        {
            Force_Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
            Force_Chassis.TIM_2ms_Control_PeriodElapsedCallback();
            ms_cnt = 0;
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            // Chassis.Mecanum_Wheels[i].Set_Out(0.0f);
            Force_Chassis.Motor_Wheel[i].Set_Target_Current(0.0f);
        }

        Chassis.Uplift_Motor[0].Set_Out(0.0f);
        Chassis.Uplift_Motor[1].Set_Out(0.0f);
        Chassis.Uplift_Motor[2].Set_Out(0.0f);
        Chassis.Uplift_Motor[3].Set_Out(0.0f);

        Chassis.Track_Motor[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Chassis.Track_Motor[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
// DWT_SysTimeUpdate();
#endif

#elif defined(GIMBAL)

    // 各个模块的分别解算
    Gimbal.TIM_Calculate_PeriodElapsedCallback();
    Booster.TIM_Calculate_PeriodElapsedCallback();
    // 传输数据给上位机
    MiniPC.TIM_Write_PeriodElapsedCallback();
    // 给下板发送数据
    CAN_Gimbal_Tx_Chassis_Callback();
#endif

#ifdef MOTOR_TEST_CHASSIS
    Output_Motor_Test_Chassis();
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
        {
            DR16_Control_Type = DR16_Control_Type_NONE;
        }
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
        {
            VT13_Control_Type = VT13_Control_Type_NONE;
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
 * @brief 获取DR16控制数据来源
 *
 * @return Enum_DR16_Control_Type DR16控制数据来源
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
 * @brief 获取VT13控制数据来源
 *
 * @return VT13_Control_Type
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

void Class_Chariot::Judge_Keyboard_Mode()
{
    const bool shift_active = Key_Is_Shift_Active(controller_key_shift);

    if (Key_Is_Trig_Pressed_Free(controller_key_z) && !shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_WORKING;
    }
    else if (Key_Is_Trig_Pressed_Free(controller_key_x) && !shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_MOVING;
    }
    else if (Key_Is_Trig_Pressed_Free(controller_key_c) && !shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_UPLIFT;
    }
    else if (Key_Is_Trig_Pressed_Free(controller_key_v) && !shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_DOWNLIFT;
    }
    else if (Key_Is_Trig_Pressed_Free(controller_key_z) && shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_SAVE_LOAD;
        Save_Load_Unit = SAVE_LOAD_UNIT_1;
    }
    else if (Key_Is_Trig_Pressed_Free(controller_key_x) && shift_active)
    {
        Keyboard_Control_Type = Keyboard_Control_Type_SAVE_LOAD;
        Save_Load_Unit = SAVE_LOAD_UNIT_2;
    }
}

void Class_Chariot::Create_Controller_Snapshot()
{
    controller_right_y = 0.0f;
    controller_mouse_x = 0.0f;
    controller_mouse_y = 0.0f;
    controller_mouse_z = 0.0f;
    controller_mouse_left_key = Controller_Key_Status_FREE;
    controller_mouse_right_key = Controller_Key_Status_FREE;
    controller_key_w = Controller_Key_Status_FREE;
    controller_key_s = Controller_Key_Status_FREE;
    controller_key_a = Controller_Key_Status_FREE;
    controller_key_d = Controller_Key_Status_FREE;
    controller_key_shift = Controller_Key_Status_FREE;
    controller_key_ctrl = Controller_Key_Status_FREE;
    controller_key_q = Controller_Key_Status_FREE;
    controller_key_e = Controller_Key_Status_FREE;
    controller_key_r = Controller_Key_Status_FREE;
    controller_key_f = Controller_Key_Status_FREE;
    controller_key_g = Controller_Key_Status_FREE;
    controller_key_z = Controller_Key_Status_FREE;
    controller_key_x = Controller_Key_Status_FREE;
    controller_key_c = Controller_Key_Status_FREE;
    controller_key_v = Controller_Key_Status_FREE;
    controller_key_b = Controller_Key_Status_FREE;
    controller_dr16_left_switch = DR16_Switch_Status_UP;
    controller_dr16_right_switch = DR16_Switch_Status_UP;
    controller_vt13_switch = VT13_Switch_Status_Left;
    left_x = 0.0f;
    left_y = 0.0f;
    right_x = 0.0f;
    yaw = 0.0f;
    dr16_left_x = 0.0f;
    dr16_left_y = 0.0f;
    dr16_right_x = 0.0f;
    dr16_right_y = 0.0f;
    dr16_yaw = 0.0f;
    vt13_left_x = 0.0f;
    vt13_left_y = 0.0f;
    vt13_right_x = 0.0f;
    vt13_right_y = 0.0f;
    vt13_yaw = 0.0f;

    if (Active_Controller == Controller_DR16)
    {
        left_x = Apply_Dead_Zone(DR16.Get_Left_X(), DR16_Dead_Zone);
        left_y = Apply_Dead_Zone(DR16.Get_Left_Y(), DR16_Dead_Zone);
        right_x = Apply_Dead_Zone(DR16.Get_Right_X(), DR16_Dead_Zone);
        controller_right_y = Apply_Dead_Zone(DR16.Get_Right_Y(), DR16_Dead_Zone);
        yaw = Apply_Dead_Zone(DR16.Get_Yaw(), DR16_Dead_Zone);
        controller_mouse_x = DR16.Get_Mouse_X();
        controller_mouse_y = DR16.Get_Mouse_Y();
        controller_mouse_z = DR16.Get_Mouse_Z();
        controller_mouse_left_key = To_Controller_Key_Status(DR16.Get_Mouse_Left_Key());
        controller_mouse_right_key = To_Controller_Key_Status(DR16.Get_Mouse_Right_Key());
        controller_key_w = To_Controller_Key_Status(DR16.Get_Keyboard_Key_W());
        controller_key_s = To_Controller_Key_Status(DR16.Get_Keyboard_Key_S());
        controller_key_a = To_Controller_Key_Status(DR16.Get_Keyboard_Key_A());
        controller_key_d = To_Controller_Key_Status(DR16.Get_Keyboard_Key_D());
        controller_key_shift = To_Controller_Key_Status(DR16.Get_Keyboard_Key_Shift());
        controller_key_ctrl = To_Controller_Key_Status(DR16.Get_Keyboard_Key_Ctrl());
        controller_key_q = To_Controller_Key_Status(DR16.Get_Keyboard_Key_Q());
        controller_key_e = To_Controller_Key_Status(DR16.Get_Keyboard_Key_E());
        controller_key_r = To_Controller_Key_Status(DR16.Get_Keyboard_Key_R());
        controller_key_f = To_Controller_Key_Status(DR16.Get_Keyboard_Key_F());
        controller_key_g = To_Controller_Key_Status(DR16.Get_Keyboard_Key_G());
        controller_key_z = To_Controller_Key_Status(DR16.Get_Keyboard_Key_Z());
        controller_key_x = To_Controller_Key_Status(DR16.Get_Keyboard_Key_X());
        controller_key_c = To_Controller_Key_Status(DR16.Get_Keyboard_Key_C());
        controller_key_v = To_Controller_Key_Status(DR16.Get_Keyboard_Key_V());
        controller_key_b = To_Controller_Key_Status(DR16.Get_Keyboard_Key_B());
        controller_dr16_left_switch = DR16.Get_Left_Switch();
        controller_dr16_right_switch = DR16.Get_Right_Switch();

        dr16_left_x = left_x;
        dr16_left_y = left_y;
        dr16_right_x = right_x;
        dr16_right_y = controller_right_y;
        dr16_yaw = yaw;
    }
    else if (Active_Controller == Controller_VT13)
    {
        left_x = Apply_Dead_Zone(VT13.Get_Left_X(), DR16_Dead_Zone);
        left_y = Apply_Dead_Zone(VT13.Get_Left_Y(), DR16_Dead_Zone);
        right_x = Apply_Dead_Zone(VT13.Get_Right_X(), DR16_Dead_Zone);
        controller_right_y = Apply_Dead_Zone(VT13.Get_Right_Y(), DR16_Dead_Zone);
        yaw = Apply_Dead_Zone(VT13.Get_Yaw(), DR16_Dead_Zone);
        controller_mouse_x = VT13.Get_Mouse_X();
        controller_mouse_y = VT13.Get_Mouse_Y();
        controller_mouse_z = VT13.Get_Mouse_Z();
        controller_mouse_left_key = To_Controller_Key_Status(VT13.Get_Mouse_Left_Key());
        controller_mouse_right_key = To_Controller_Key_Status(VT13.Get_Mouse_Right_Key());
        controller_key_w = To_Controller_Key_Status(VT13.Get_Keyboard_Key_W());
        controller_key_s = To_Controller_Key_Status(VT13.Get_Keyboard_Key_S());
        controller_key_a = To_Controller_Key_Status(VT13.Get_Keyboard_Key_A());
        controller_key_d = To_Controller_Key_Status(VT13.Get_Keyboard_Key_D());
        controller_key_shift = To_Controller_Key_Status(VT13.Get_Keyboard_Key_Shift());
        controller_key_ctrl = To_Controller_Key_Status(VT13.Get_Keyboard_Key_Ctrl());
        controller_key_q = To_Controller_Key_Status(VT13.Get_Keyboard_Key_Q());
        controller_key_e = To_Controller_Key_Status(VT13.Get_Keyboard_Key_E());
        controller_key_r = To_Controller_Key_Status(VT13.Get_Keyboard_Key_R());
        controller_key_f = To_Controller_Key_Status(VT13.Get_Keyboard_Key_F());
        controller_key_g = To_Controller_Key_Status(VT13.Get_Keyboard_Key_G());
        controller_key_z = To_Controller_Key_Status(VT13.Get_Keyboard_Key_Z());
        controller_key_x = To_Controller_Key_Status(VT13.Get_Keyboard_Key_X());
        controller_key_c = To_Controller_Key_Status(VT13.Get_Keyboard_Key_C());
        controller_key_v = To_Controller_Key_Status(VT13.Get_Keyboard_Key_V());
        controller_key_b = To_Controller_Key_Status(VT13.Get_Keyboard_Key_B());
        controller_vt13_switch = VT13.Get_Switch();

        vt13_left_x = left_x;
        vt13_left_y = left_y;
        vt13_right_x = right_x;
        vt13_right_y = controller_right_y;
        vt13_yaw = yaw;
    }
}

void Class_Chariot::Controller_Data_Update()
{
    Judge_Active_Controller();
    Create_Controller_Snapshot();

    if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
        (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        Judge_Keyboard_Mode();
    }
}

void Class_Chariot::Judge_Gimbal_Control_Type()
{
    if (Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        if (controller_dr16_left_switch == DR16_Switch_Status_UP)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Keyboard_Control_Type = Keyboard_Control_Type_DISABLE;
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }
    }
    else if (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_REMOTE)
    {
        if (controller_vt13_switch == VT13_Switch_Status_Left)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Keyboard_Control_Type = Keyboard_Control_Type_DISABLE;
        }
        else
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        }
    }
    else if ((Active_Controller == Controller_DR16 && DR16_Control_Type == DR16_Control_Type_KEYBOARD) ||
             (Active_Controller == Controller_VT13 && VT13_Control_Type == VT13_Control_Type_KEYBOARD))
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
    }
    else
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Keyboard_Control_Type = Keyboard_Control_Type_DISABLE;
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
    // 判断遥控器数据来源
    Controller_Data_Update();
    // 底盘，云台，发射机构控制逻辑
    Control_Gimbal();
    Control_Chassis();
}
#endif
/**
 * @brief 在线判断回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
// TIM_Unline_Protect_PeriodElapsedCallback();
#ifdef CHASSIS
#ifndef CHASSIS_TEST

        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();

        // IMU
        Force_Chassis.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        // 力控底盘中轮组电机的存活检测以及底盘类中抬升电机的存活检测
        for (int i = 0; i < 4; i++)
        {
            // Chassis.Mecanum_Wheels[i].TIM_Alive_PeriodElapsedCallback();
            Force_Chassis.Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
        }

        Chassis.Uplift_Motor[0].TIM_Alive_PeriodElapsedCallback();
        Chassis.Uplift_Motor[1].TIM_Alive_PeriodElapsedCallback();
        Chassis.Uplift_Motor[2].TIM_Alive_PeriodElapsedCallback();
        Chassis.Uplift_Motor[3].TIM_Alive_PeriodElapsedCallback();

        // 主动轮电机存活检测
        Chassis.Track_Motor[0].TIM_Alive_PeriodElapsedCallback();
        Chassis.Track_Motor[1].TIM_Alive_PeriodElapsedCallback();

        if (mod50_mod3 % 3 == 0)
        {
            TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
            Chassis.TOFSense.TIM1msMod150_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
        if (Get_Gimbal_Status() == Gimbal_Status_DISABLE)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            Chassis.Set_Target_Velocity_X(0);
            Chassis.Set_Target_Velocity_Y(0);
            Chassis.Set_Target_Omega(0);
        }
#else
        Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();

        // 力控底盘中的超电存活检测函数
        Force_Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();

        // IMU
        Force_Chassis.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        // 力控底盘中轮组电机的存活检测以及底盘类中抬升电机的存活检测
        for (int i = 0; i < 4; i++)
        {
            // Chassis.Mecanum_Wheels[i].TIM_Alive_PeriodElapsedCallback();
            Force_Chassis.Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
        }

        Chassis.Uplift_Motor[0].TIM_Alive_PeriodElapsedCallback();
        Chassis.Uplift_Motor[1].TIM_Alive_PeriodElapsedCallback();
        Chassis.Uplift_Motor[2].TIM_Alive_PeriodElapsedCallback();

        // 主动轮电机存活检测
        Chassis.Track_Motor[0].TIM_Alive_PeriodElapsedCallback();
        Chassis.Track_Motor[1].TIM_Alive_PeriodElapsedCallback();

        if (mod50_mod3 % 3 == 0)
        {
            TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }

        // if (Get_Gimbal_Status() == Gimbal_Status_DISABLE)
        // {
        //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        //     Chassis.Set_Target_Velocity_X(0);
        //     Chassis.Set_Target_Velocity_Y(0);
        //     Chassis.Set_Target_Omega(0);
        // }
#endif
#elif defined(GIMBAL)

        if (mod50_mod3 % 3 == 0)
        {
            // 判断底盘通讯在线状态
            TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
            VT13.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
#ifdef defined(USE_DR16)
#ifdef DEBUG
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#else
        if (CAN3_Chassis_Rx_Data_A.game_process != 4)
        {
            if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }
        }
#endif
#elif defined(USE_VT13)
#ifdef DEBUG
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#else
        if (CAN3_Chassis_Rx_Data_A.game_process != 4)
        {
            if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }
        }
#endif

#endif
        Gimbal.J0_Pitch_4340.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J1_Yaw_8009P.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J2_Yaw_4340P.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J3_Roll_2325.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J4_Pitch_2325.TIM_Alive_PeriodElapsedCallback();
        if (mod50_mod3 == 1)
        {
            Gimbal.Jodell_ERG150T.TIM1msMod50_Alive_PeriodElapsedCallback();
        }

        is_arm_online = (Gimbal.J0_Pitch_4340.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J1_Yaw_8009P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J2_Yaw_4340P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J3_Roll_2325.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J4_Pitch_2325.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.Jodell_ERG150T.Get_Motor_Working_Status() == Jodell_Motor_Working_ENABLE);

        if (!is_arm_online)
            Gimbal.arm_init = false; // 如果机械臂掉线，arm_init设为false

        Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

        Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
        Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();

        MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

#endif

#ifdef MOTOR_TEST_CHASSIS
        Test_Motor.TIM_Alive_PeriodElapsedCallback();
#endif

#ifdef CHASSIS_TEST
        if (mod50_mod3 % 3 == 0)
        {
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
            mod50_mod3 = 0;
        }
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
#ifdef defined(USE_DR16)
#ifdef DEBUG
    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }
#else
    if (CAN3_Chassis_Rx_Data_A.game_process != 4)
    {
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
    }
#endif
#elif defined(USE_VT13)
#ifdef DEBUG
    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    }
#else
    if (CAN3_Chassis_Rx_Data_A.game_process != 4)
    {
        if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
    }
#endif

#endif

#endif

// 底盘离线保护
#ifdef CHASSIS
    if (Get_Gimbal_Status() == Gimbal_Status_DISABLE)
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
    }
    else
    {
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
        if (huart5.ErrorCode)
        {
            Status[Now_Status_Serial].Time = 0;
            Set_Status(4);
        }

        // 转移为 在线状态
        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            buzzer_setTask(&buzzer, BUZZER_DJI_STARTUP_PRIORITY);
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
#endif

#ifdef CHASSIS_TEST
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
        Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        Chariot->Force_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);

        if (Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
            Chariot->Force_Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type__());
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
        Chariot->Set_Pre_Chassis_Control_Type__(Chariot->Force_Chassis.Get_Chassis_Control_Type());

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
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
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

        // 转移为 在线状态
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
        {
            buzzer_setTask(&buzzer, BUZZER_DJI_STARTUP_PRIORITY);
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
        // 转移为 刚离线状态
        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            buzzer_setTask(&buzzer, BUZZER_FORCE_STOP_PRIORITY);
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
        HAL_UART_DMAStop(&huart1); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}

// 存取矿状态机定时器回调函数
void Class_FSM_Save_Load::Reload_TIM_Status_PeriodElapsedCallback(Enum_Save_Load_Type Enum_Save_Load_Type, bool step)
{
    Status[Now_Status_Serial].Time++;

    // 选取数据源
    Struct_Enery_Unit_Position &active_unit = ((Enum_Save_Load_Type == SAVE_LOAD_UNIT_1) ? Enery_Unit_1 : Enery_Unit_2);

    // 状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 准备阶段
        if (step)
        {
            Set_Status(1);
        }

        break;
    }
    case (1):
    {
        // 移动到初始位置
        Gimbal->Set_Target_J0_Pitch_Radian(active_unit.init_pos[0]);
        Gimbal->Set_Target_J1_Yaw_Radian(active_unit.init_pos[1]);
        Gimbal->Set_Target_J2_Yaw_Radian(active_unit.init_pos[2]);
        Gimbal->Set_Target_J3_Roll_Radian(active_unit.init_pos[3]);
        Gimbal->Set_Target_J4_Pitch_Radian(active_unit.init_pos[4]);
        Gimbal->Set_Target_J5_Roll_Radian(active_unit.init_pos[5]);

        bool finish_flag = false;
        test_flag[0] = (fabs((Gimbal->J0_Pitch_4340.Get_Now_Angle() - PI) - Gimbal->Get_Target_J0_Pitch_Radian()) <= 0.1f);
        test_flag[1] = (fabs((Gimbal->J1_Yaw_8009P.Get_Now_Angle() - PI) - Gimbal->Get_Target_J1_Yaw_Radian()) <= 0.1f);
        test_flag[2] = (fabs((Gimbal->J2_Yaw_4340P.Get_Now_Angle() - PI) - Gimbal->Get_Target_J2_Yaw_Radian()) <= 0.1f);
        test_flag[3] = (fabs((Gimbal->J3_Roll_2325.Get_Now_Angle() - PI) - Gimbal->Get_Target_J3_Roll_Radian_In_PI()) <= 0.1f);
        test_flag[4] = (fabs((Gimbal->J4_Pitch_2325.Get_Now_Angle() - PI + 0.52741) - Gimbal->Get_Target_J4_Pitch_Radian_In_PI()) <= 0.1f);
        test_flag[5] = (fabs(Gimbal->Jodell_ERG150T.Get_Now_Roll() - Gimbal->Get_Target_J5_Roll_Radian()) <= 0.1f);
        finish_flag = test_flag[0] &&
                      test_flag[1] &&
                      test_flag[2] &&
                      test_flag[3] &&
                      test_flag[4] &&
                      test_flag[5] &&
                      (true);

        if (finish_flag)
        {
            Set_Status(2);
        }

        break;
    }
    case (2):
    {
        // J2先移动
        Gimbal->Set_Target_J2_Yaw_Radian(active_unit.finish_pos[2]);

        bool finish_flag = (fabs((Gimbal->J2_Yaw_4340P.Get_Now_Angle() - PI) * 4.0f - Gimbal->Get_Target_J2_Yaw_Radian()) <= 0.1f);
        if (finish_flag)
        {
            Set_Status(3);
        }

        break;
    }
    case (3):
    {
        // J1开始移动到存矿位置
        Gimbal->Set_Target_J1_Yaw_Radian(active_unit.finish_pos[1]);

        bool finish_flag = (fabs((Gimbal->J1_Yaw_8009P.Get_Now_Angle() - PI) * 4.0f - Gimbal->Get_Target_J1_Yaw_Radian()) <= 0.1f);
        if (finish_flag)
        {
            Set_Status(4);
        }

        break;
    }
    case (4):
    {
        // 等待操作手操作
        if (step)
        {
            Set_Status(5);
        }

        break;
    }
    case (5):
    {
        // 移动到辅助位置，防止存矿后归位时卡矿
        Gimbal->Set_Target_J1_Yaw_Radian(active_unit.auxiliary_pos[1]);
        Gimbal->Set_Target_J2_Yaw_Radian(active_unit.auxiliary_pos[2]);

        bool finish_flag = false;
        finish_flag = (fabs((Gimbal->J1_Yaw_8009P.Get_Now_Angle() - PI) * 4.0f - Gimbal->Get_Target_J1_Yaw_Radian()) <= 0.1f) &&
                      (fabs((Gimbal->J2_Yaw_4340P.Get_Now_Angle() - PI) * 4.0f - Gimbal->Get_Target_J2_Yaw_Radian()) <= 0.1f) &&
                      (true);

        if (finish_flag)
        {
            Set_Status(6);
        }

        break;
    }
    case (6):
        // 归位到初始位置
        {
            Gimbal->Set_Target_J0_Pitch_Radian(active_unit.init_pos[0]);
            Gimbal->Set_Target_J1_Yaw_Radian(active_unit.init_pos[1]);
            Gimbal->Set_Target_J2_Yaw_Radian(active_unit.init_pos[2]);
            Gimbal->Set_Target_J3_Roll_Radian(active_unit.init_pos[3]);
            Gimbal->Set_Target_J4_Pitch_Radian(active_unit.init_pos[4]);
            Gimbal->Set_Target_J5_Roll_Radian(active_unit.init_pos[5]);

            break;
        }
    }
}
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
