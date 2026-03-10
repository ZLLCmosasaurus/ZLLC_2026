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

#endif

#ifdef CHASSIS_TEST
    // 遥控器离线控制 状态机
    FSM_Alive_Control.Chariot = this;
    FSM_Alive_Control.Init(5, 0);
    DR16.Init(&huart5, &huart1);
    DR16_Dead_Zone = __DR16_Dead_Zone;
#endif
}

#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Heat;
    uint16_t Cooling_Value;
    uint16_t Self_HP, Self_Outpost_HP, Oppo_Outpost_HP, Self_Base_HP, Ammo_number;
    uint8_t color, remaining_energy, supercap_proportion, radar_info, dart_target;
    uint16_t Pre_HP[6] = {0};
    uint16_t HP[6] = {0};
    uint8_t Flag[6] = {0};
    float Pre_Count[6] = {0};
    uint16_t Position[8] = {0};
    int16_t Bullet_Speed = 0.f;
    int16_t Self_Position_X, Self_Position_Y;
    int16_t Target_Position_X, Target_Position_Y;
    // 数据更新
    if (Referee.Get_ID() == Referee_Data_Robots_ID_RED_SENTRY_7)
    {
        color = 1;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_BASE_10);

        for (int i = 0; i < 6; i++)
        {
            Pre_HP[i] = HP[i];
        }

        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_SENTRY_7);
    }
    else if (Referee.Get_ID() == Referee_Data_Robots_ID_BLUE_SENTRY_7)
    {
        color = 0;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_BASE_10);

        for (int i = 0; i < 6; i++)
        {
            Pre_HP[i] = HP[i];
        }

        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_RED_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_RED_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_RED_SENTRY_7);
    }
    Shooter_Heat = Referee.Get_Booster_17mm_1_Heat();
    if (Referee.Get_Shoot_Booster_Type() == Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1)
    {
        Bullet_Speed = (int16_t)(Referee.Get_Shoot_Speed() * 100.f);
    }
    Self_HP = Referee.Get_HP();
    Ammo_number = Referee.Get_17mm_Remaining();
    Cooling_Value = Referee.Get_Booster_17mm_Heat_CD();
    remaining_energy = Referee.Get_Remaining_Energy();
    supercap_proportion = Chassis.Supercap.Get_Supercap_Proportion();
    Self_Position_X = (int16_t)(Referee.Get_Location_X() * 100.f);
    Self_Position_Y = (int16_t)(Referee.Get_Location_Y() * 100.f);
    Target_Position_X = (int16_t)(Referee.Get_Radar_Send_Coordinate_X() * 100.f);
    Target_Position_Y = (int16_t)(Referee.Get_Radar_Send_Coordinate_Y() * 100.f);
    radar_info = Referee.Get_Radar_Info();
    dart_target = Referee.Get_Dart_Command_Target() | (0x01 & Referee.Get_Sentry_Info_1() >> 19) << 2;

    for (int i = 0; i < 6; i++) // 无敌状态辨认
    {
        if (HP[i] > 0 && Pre_HP[i] == 0)
        {
            Flag[i] = 1;
            Pre_Count[i] = DWT_GetTimeline_s();
        }
        if ((DWT_GetTimeline_s() - Pre_Count[i]) > 7.f && Flag[i] == 1)
        {
            Flag[i] = 0;
            Pre_Count[i] = 0;
        }
    }

    Position[0] = Referee.Get_Hero_Position_X();
    Position[1] = Referee.Get_Hero_Position_Y();
    Position[2] = Referee.Get_Sentry_Position_X();
    Position[3] = Referee.Get_Sentry_Position_Y();
    Position[4] = Referee.Get_Infantry_3_Position_X();
    Position[5] = Referee.Get_Infantry_3_Position_Y();
    Position[6] = Referee.Get_Infantry_4_Position_X();
    Position[7] = Referee.Get_Infantry_4_Position_Y();

    // 发送数据给云台
    // A包
    CAN3_Chassis_Tx_Data_A[0] = Referee.Get_Game_Stage();
    CAN3_Chassis_Tx_Data_A[1] = Referee.Get_Remaining_Time() >> 8;
    CAN3_Chassis_Tx_Data_A[2] = Referee.Get_Remaining_Time();
    CAN3_Chassis_Tx_Data_A[3] = Referee.Get_HP() >> 8;
    CAN3_Chassis_Tx_Data_A[4] = Referee.Get_HP();
    CAN3_Chassis_Tx_Data_A[5] = Self_Outpost_HP >> 8;
    CAN3_Chassis_Tx_Data_A[6] = Self_Outpost_HP;
    CAN3_Chassis_Tx_Data_A[7] = color << 7 | Flag[5] << 5 | Flag[4] << 4 | Flag[3] << 3 | Flag[2] << 2 | Flag[1] << 1 | Flag[0] << 0;

    // B包
    memcpy(CAN3_Chassis_Tx_Data_B + 0, &Self_Base_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 2, &Oppo_Outpost_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 4, &Ammo_number, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 6, &Cooling_Value, sizeof(uint16_t));

    // C包
    memcpy(CAN3_Chassis_Tx_Data_C + 0, &Shooter_Heat, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 4, &remaining_energy, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 5, &supercap_proportion, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 6, &radar_info, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 7, &dart_target, sizeof(uint8_t));

    // D包
    memcpy(CAN3_Chassis_Tx_Data_D + 0, &Position[0], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 2, &Position[1], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 4, &Position[2], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 6, &Position[3], sizeof(uint16_t));

    // E包
    memcpy(CAN3_Chassis_Tx_Data_E + 0, &Self_Position_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_E + 2, &Self_Position_Y, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_E + 4, &Bullet_Speed, sizeof(int16_t));

    // F包
    memcpy(CAN3_Chassis_Tx_Data_F + 0, &Position[4], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 2, &Position[5], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 4, &Position[6], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 6, &Position[7], sizeof(uint16_t));

    // G包
    memcpy(CAN3_Chassis_Tx_Data_G + 0, &Target_Position_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_G + 2, &Target_Position_Y, sizeof(int16_t));
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
Struct_CAN_Referee_Rx_Data_t CAN_Referee_Rx_Data;
#ifdef CHASSIS
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
    // 抬升机构状态
    Enum_Chassis_DR16_Right_Uplift_Status dr16_uplift_status;
    // float映射到int16之后的速度
    int16_t tmp_velocity_x, tmp_velocity_y, tmp_delta_radian;
    uint8_t tmp_dr16_uplift_status;

    switch (CAN_Manage_Object->Rx_Buffer.Header.Identifier)
    {
    // 底盘控制数据回传
    case (0x77):
    {
        memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(int16_t));
        memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(int16_t));
        memcpy(&tmp_delta_radian, &CAN_Manage_Object->Rx_Buffer.Data[4], sizeof(int16_t));
        memcpy(&tmp_dr16_uplift_status, &CAN_Manage_Object->Rx_Buffer.Data[6], sizeof(uint8_t));
        memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

#ifdef AGV
        chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x, -450, 450, -4, 4);
        chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y, -450, 450, -4, 4);
        chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.f, 4.f) / Chassis_Radius; // 映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
#else
        chassis_velocity_x = -Math_Int_To_Float(tmp_velocity_x, -450, 450, -4.f, 4.f);
        chassis_velocity_y = -Math_Int_To_Float(tmp_velocity_y, -450, 450, -4.f, 4.f);
        chassis_delta_radian = -Math_Int_To_Float(tmp_delta_radian, -200, 200, -1.f, 1.f);
#endif
        chassis_control_type = (Enum_Chassis_Control_Type)control_type;
        dr16_uplift_status = (Enum_Chassis_DR16_Right_Uplift_Status)tmp_dr16_uplift_status;
        // 设定底盘控制类型
        Chassis.Set_Chassis_Control_Type(chassis_control_type);
        // if (chassis_omega < 0.5f && chassis_omega > -0.5f)
        //     chassis_omega = 0;
        // 设定底盘目标速度
        Chassis.Set_Target_Velocity_X(chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
#ifdef OMNI_WHEEL
        Chassis.Set_Target_Velocity_X(-chassis_velocity_x);
#endif
        Chassis.Set_Delta_Radian(chassis_delta_radian); // 目标角度增量
        Chassis.Set_DR16_Right_Uplift_Status(dr16_uplift_status);

        Control_Chassis();

        break;
    }
    case (0x95):
        // 裁判系统数据回传
        {
            memcpy(&CAN_Referee_Rx_Data, &CAN_Manage_Object->Rx_Buffer.Data, sizeof(Struct_CAN_Referee_Rx_Data_t));
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
    case (0x188):
    {
        memcpy(&PRE_CAN3_Chassis_Rx_Data_A, &CAN3_Chassis_Rx_Data_A, sizeof(Referee_Rx_A_t));
        CAN3_Chassis_Rx_Data_A.game_process = CAN_Manage_Object->Rx_Buffer.Data[0];
        CAN3_Chassis_Rx_Data_A.remaining_time = CAN_Manage_Object->Rx_Buffer.Data[1] << 8 | CAN_Manage_Object->Rx_Buffer.Data[2];
        CAN3_Chassis_Rx_Data_A.self_blood = CAN_Manage_Object->Rx_Buffer.Data[3] << 8 | CAN_Manage_Object->Rx_Buffer.Data[4];
        CAN3_Chassis_Rx_Data_A.self_outpost_HP = CAN_Manage_Object->Rx_Buffer.Data[5] << 8 | CAN_Manage_Object->Rx_Buffer.Data[6];
        CAN3_Chassis_Rx_Data_A.color_invincible_state = CAN_Manage_Object->Rx_Buffer.Data[7];
        break;
    }
    case (0x199):
    {
        memcpy(&CAN3_Chassis_Rx_Data_B, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_B_t));
        break;
    }
    case (0x178):
    {
        memcpy(&CAN3_Chassis_Rx_Data_C, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_C_t));
        Booster.Set_Heat(CAN3_Chassis_Rx_Data_C.Booster_Heat);
        break;
    }
    case (0x198):
    {
        memcpy(&CAN3_Chassis_Rx_Data_D, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_D_t));
        break;
    }
    case (0x197):
    {
        memcpy(&CAN3_Chassis_Rx_Data_E, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_E_t));
        speed_a = (float)(CAN3_Chassis_Rx_Data_E.Bullet_Speed / 100.f);
        break;
    }
    case (0x196):
    {
        memcpy(&CAN3_Chassis_Rx_Data_F, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_F_t));
        break;
    }
    case (0x191):
    {
        memcpy(&CAN3_Chassis_Rx_Data_G, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_G_t));
        break;
    }
    }
}
#endif

/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
// 控制类型字节
uint8_t control_type;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    // 底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, chassis_radian = 0;
    // 映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_radian = 0;
    uint8_t dr16_uplift_status = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    Enum_Chassis_DR16_Right_Uplift_Status dr16_right_switch_status;
    // 控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_radian = Chassis.Get_Target_Omega();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    dr16_right_switch_status = Chassis.Get_DR16_Right_Uplift_Status();
    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -4.f, 4.f, -450, 450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -4.f, 4.f, -450, 450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));

    tmp_chassis_radian = -Math_Float_To_Int(chassis_radian, -1.f, 1.f, -200, 200); // 随动环 逆时针为正所以加负号
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_radian, sizeof(int16_t));

    dr16_uplift_status = (uint8_t)dr16_right_switch_status;
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 6, &dr16_uplift_status, sizeof(uint8_t)); // 右拨杆状态

    control_type = (uint8_t)chassis_control_type; // 底盘控制模式
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 7, &control_type, sizeof(uint8_t));
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
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_r_x, dr16_yaw;
    // 底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_delta_radian = 0;

    // 排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
    dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_yaw = (Math_Abs(DR16.Get_Yaw()) > 0.5f) ? DR16.Get_Yaw() : 0;

    // 遥控器操作逻辑
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    volatile int DR16_Right_Switch_Status = DR16.Get_Right_Switch();
    switch (DR16_Left_Switch_Status)
    {
    case (DR16_Switch_Status_UP):
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        break;
    }
    case (DR16_Switch_Status_MIDDLE): // 左中 正常模式控制底盘，只有当机械臂移动到初始位姿后才将底盘设置为正常模式
    {
        if (Gimbal.arm_init == true)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL);
        }
        else
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        break;
    }
    case (DR16_Switch_Status_DOWN):
    {
        if (Gimbal.arm_init == true)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SLOPE);
        }
        else
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
        break;
    }
    default:
    {
        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        break;
    }
    }

    // 云台到底盘坐标系转换
    volatile int Chassis_control_type = Chassis.Get_Chassis_Control_Type();
    switch (Chassis_control_type)
    {
    case (Chassis_Control_Type_DISABLE):
    { // 失能
        chassis_velocity_x = 0;
        chassis_velocity_y = 0;
        chassis_delta_radian = 0;
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        // 设定矩形到圆形映射进行控制
        chassis_velocity_y = -dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_x = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();
        chassis_delta_radian = -dr16_r_x * sqrt(1.0f - dr16_r_x * dr16_r_x / 2.0f) * 0.01f;

        // 传递拨杆状态
        switch (DR16_Right_Switch_Status)
        {
        case (DR16_Switch_Status_MIDDLE):
        {
            Chassis.Set_DR16_Right_Uplift_Status(DR16_Right_Switch_MIDDLE);
            break;
        }

        case (DR16_Switch_Status_UP):
        {
            if (dr16_yaw >= 0.9f)
            {
                Chassis.Set_DR16_Right_Uplift_Status(Chassis_Uplift_FLAG_ADD);
            }
            else
            {
                Chassis.Set_DR16_Right_Uplift_Status(DR16_Right_Switch_UP);
            }
            break;
        }

        case (DR16_Switch_Status_DOWN):
        {
            Chassis.Set_DR16_Right_Uplift_Status(DR16_Right_Switch_DOWN);
            break;
        }
        }

        break;
    }
    case (Chassis_Control_Type_SLOPE):
    {
        chassis_velocity_y = -dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * 1.0f;
        chassis_velocity_x = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * 1.0f;
        chassis_delta_radian += -dr16_r_x * sqrt(1.0f - dr16_r_x * dr16_r_x / 2.0f) * 0.005f;

        // 传递拨杆状态，拨杆上+Yaw = 状态机前进，拨杆下+Yaw = 开启小轮子
        switch (DR16_Right_Switch_Status)
        {
        case (DR16_Switch_Status_MIDDLE):
        {
            Chassis.Set_DR16_Right_Uplift_Status(DR16_Right_Switch_MIDDLE);
            break;
        }

        case (DR16_Switch_Status_UP):
        {
            Chassis.Set_DR16_Right_Uplift_Status(
                dr16_yaw >= 0.8f ? Chassis_Uplift_FLAG_ADD : DR16_Right_Switch_UP);
            break;
        }

        case (DR16_Switch_Status_DOWN):
        {
            Chassis.Set_DR16_Right_Uplift_Status(
                dr16_yaw >= 0.8f ? Chassis_Uplift_WHEEL_ON : DR16_Right_Switch_DOWN);
            break;
        }
        }
    }
    }

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y); // 前x左y正
    Chassis.Set_Target_Omega(chassis_delta_radian);
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

        if (DR16_Right_Uplift_Status == Chassis_Uplift_WHEEL_ON)
        {
            track_omega = 25.0f;
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
        chassis_radian = chassis_radian;
        break;
    }
    case (Chassis_Control_Type_NORMAL__):
    {
        chassis_velocity_x = Chassis.Get_Target_Velocity_X();
        chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
        chassis_radian += Chassis.Get_Delta_Radian();
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
        Enum_DR16_Switch_Status DR16_Right_Switch_Status;
        float Yaw;

        if (DR16_Right_Uplift_Status == Chassis_Uplift_FLAG_ADD)
        {
            Yaw = 1.0f;
            DR16_Right_Switch_Status = DR16_Switch_Status_UP;
        }
        else
        {
            Yaw = 0.0f;
            DR16_Right_Switch_Status = DR16_Switch_Status_MIDDLE;
        }

        Chassis.Ledder_FSM.DR16_Right = DR16_Right_Switch_Status;
        Chassis.Ledder_FSM.Yaw = Yaw;
        Chassis.Ledder_FSM.Reload_TIM_Status_PeriodElapsedCallback();
        Chassis.Ledder_FSM.DR16_Pre_Right = (Enum_DR16_Switch_Status)DR16_Right_Switch_Status;
        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
        // // 上台阶状态机状态清零
        // Chassis.Ledder_FSM.Set_Status(0);

        // // 底盘在线状态下遥控器控制抬升机构

        // // 抬升机构控制逻辑
        // switch (DR16_Right_Uplift_Status)
        // {
        // case (DR16_Right_Switch_UP):
        // {
        //     for (int i = 0; i < 4; i++)
        //     {
        //         target_uplift_rad[i] += PI * 0.03f;
        //     }
        //     break;
        // }

        // case (DR16_Right_Switch_DOWN):
        // {
        //     for (int i = 0; i < 4; i++)
        //     {
        //         target_uplift_rad[i] -= PI * 0.03f;
        //     }
        //     break;
        // }

        // default:
        // {
        // }
        // }

        switch (DR16_Right_Uplift_Status)
        {
        case (DR16_Right_Switch_UP):
        {
            for (int i = 0; i < 4; i++)
            {
                target_uplift_rad[i] += PI * 0.03f;
            }
            break;
        }

        case (DR16_Right_Switch_DOWN):
        {
            for (int i = 0; i < 4; i++)
            {
                target_uplift_rad[i] -= PI * 0.03f;
            }

            Math_Constrain(target_uplift_rad + 0, Chassis.Uplift_Touch_Radian[0], 28.5f);
            Math_Constrain(target_uplift_rad + 1, Chassis.Uplift_Touch_Radian[1], 28.5f);
            break;
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
    // 排除遥控器死区
    dr16_right_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_right_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
    dr16_left_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
    dr16_left_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    // dr16左上角的滑杆，用来控制夹爪，不要被名字迷惑了
    dr16_yaw = (Math_Abs(DR16.Get_Yaw()) > DR16_Dead_Zone) ? DR16.Get_Yaw() : 0;

#ifdef PUMA
    /*获取当前各个关节的角度值，用来给使用遥控器控制关节时计算目标角度*/
    tmp_arm_yaw = Gimbal.Get_Target_Yaw_Radian();
    /*以下除了roll以外的关节改之前为调用电机对象的Get_Target_Angle函数，但是我认为使用云台类中各个关节的目标角度应该也一样
      在加入平动模式时，解算出的角度也需要使用Gimbal对象中各个关节的Set函数进行设置，这样可以保证两个模式下数据的同步*/
    tmp_arm_pitch1 = Gimbal.Get_Target_Pitch_Radian();
    tmp_arm_pitch2 = Gimbal.Get_Target_Pitch_2_Radian();
    tmp_arm_pitch3 = Gimbal.Get_Target_Pitch_3_Radian();
    tmp_arm_roll = Gimbal.Get_Target_Roll_Radian() - Gimbal.Get_Roll_Min_Radian(); // 安全规范的写法，可以避免模式切换时数据不同步导致的关节转动
    tmp_arm_roll_2 = Gimbal.Get_Target_Roll_2_Radian();                            // 统一使用弧度制
#endif

    /* 改之前为获取gripper的target_angle，由于夹爪的Set函数自己会加Offset，所以用Target_Angle来赋值会导致Offset叠加，
    这里给一个新变量来存。由于夹爪的控制是通过遥控器/鼠标来进行控制的，不会有别的地方不使用last_gripper_value来对夹爪
    电机赋值，换句话说，在比赛中使用键鼠/遥控器操作时也是在last_gripper_value上进行增减，所以这里就先继续沿用这种方案。 */
    tmp_gripper_radian = last_gripper_value;
    tmp_j0_pitch_radian = Gimbal.Get_Target_J0_Pitch_Radian();
    tmp_j1_yaw_radian = Gimbal.Get_Target_J1_Yaw_Radian();
    tmp_j2_yaw_radian = Gimbal.Get_Target_J2_Yaw_Radian();
    tmp_j3_yaw_radian = Gimbal.Get_Target_J3_Yaw_Radian();
    tmp_j4_pitch_radian = Gimbal.Get_Target_J4_Pitch_Radian();
    tmp_j5_yaw_radian = Gimbal.Get_Target_J5_Yaw_Radian();

    if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 失能
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    }
    // 其余位置都是遥控器控制
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中，摇杆控制机械臂
    {

        tmp_j0_pitch_radian = 0.0f;
        tmp_j1_yaw_radian = 0.0f;
        tmp_j2_yaw_radian = 0.0f;
        tmp_j3_yaw_radian = 0.0f;
        tmp_j4_pitch_radian = 0.0f;
        tmp_j5_yaw_radian = 0.0f;

        Gimbal.Set_Target_J0_Pitch_Radian(tmp_j0_pitch_radian);
        Gimbal.Set_Target_J1_Yaw_Radian(tmp_j1_yaw_radian);
        Gimbal.Set_Target_J2_Yaw_Radian(tmp_j2_yaw_radian);
        Gimbal.Set_Target_J3_Yaw_Radian(tmp_j3_yaw_radian);
        Gimbal.Set_Target_J4_Pitch_Radian(tmp_j4_pitch_radian);
        Gimbal.Set_Target_J5_Yaw_Radian(tmp_j5_yaw_radian);
    }
    else if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    {
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
        {
            if (dr16_right_y >= 0.8f)
            // 右摇杆往上拨切换到蜷缩状态，这个状态两个大臂收起，防止冲下台阶时重心侧翻和撞地
            {
                // j1 - 0.637
                // j2 - -2.094
                // j3 - -1.0832
                tmp_j0_pitch_radian = 0.0f;
                tmp_j1_yaw_radian = 0.637f;
                tmp_j2_yaw_radian = -2.094f;
                tmp_j3_yaw_radian = -1.0832f;
                tmp_j4_pitch_radian = 0.0f;
                tmp_j5_yaw_radian = 0.0f;
            }
            else if (dr16_right_y <= -0.8f)
            {
                // 右摇杆往下拨切换到初始位姿，方便底盘通过台阶
                tmp_j0_pitch_radian = 0.0f;
                tmp_j1_yaw_radian = 0.0f;
                tmp_j2_yaw_radian = 0.0f;
                tmp_j3_yaw_radian = 0.0f;
                tmp_j4_pitch_radian = 0.0f;
                tmp_j5_yaw_radian = 0.0f;
            }
        }
        else if (DR16.Get_Right_Switch() == DR16_Switch_Status_MIDDLE)
        {
            tmp_j0_pitch_radian = Offline_Controller_Data.Angle[0];
            tmp_j1_yaw_radian = Offline_Controller_Data.Angle[1];
            tmp_j2_yaw_radian = Offline_Controller_Data.Angle[2];
            tmp_j3_yaw_radian = Offline_Controller_Data.Angle[3];
            tmp_j4_pitch_radian = Offline_Controller_Data.Angle[4];
            tmp_j5_yaw_radian = Offline_Controller_Data.Angle[5];
            // 夹爪控制
            tmp_gripper_radian += dr16_right_y * DR16_Gripper_Resolution;
        }
        Gimbal.Set_Target_J0_Pitch_Radian(tmp_j0_pitch_radian);
        Gimbal.Set_Target_J1_Yaw_Radian(tmp_j1_yaw_radian);
        Gimbal.Set_Target_J2_Yaw_Radian(tmp_j2_yaw_radian);
        Gimbal.Set_Target_J3_Yaw_Radian(tmp_j3_yaw_radian);
        Gimbal.Set_Target_J4_Pitch_Radian(tmp_j4_pitch_radian);
        Gimbal.Set_Target_J5_Yaw_Radian(tmp_j5_yaw_radian);
    }
    else // 左下，摇杆控制底盘，机械臂保持原来的姿态
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
    }

    Math_Constrain(&tmp_gripper_radian, 0.0f, 0.90f);
    last_gripper_value = tmp_gripper_radian;
    // 云台对象中夹爪赋值
    Gimbal.Set_Target_Gripper_Radian(tmp_gripper_radian);
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
    DR16_Control_Type = DR16_Control_Type_REMOTE;
}
#endif
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
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
        Gimbal.J3_Yaw_4340P.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J4_Pitch_4340P.TIM_Alive_PeriodElapsedCallback();
        Gimbal.J5_Yaw_4340P.TIM_Alive_PeriodElapsedCallback();
        Gimbal.Motor_C610_Gripper.TIM_Alive_PeriodElapsedCallback();

        is_arm_online = (Gimbal.J0_Pitch_4340.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J1_Yaw_8009P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J2_Yaw_4340P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J3_Yaw_4340P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J4_Pitch_4340P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE ||
                         Gimbal.J5_Yaw_4340P.Get_DM_Motor_Status() == DJI_Motor_Status_ENABLE);

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
        Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

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
        if (huart9.ErrorCode)
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
        Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

        if (Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
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
        HAL_UART_DMAStop(&huart9); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        Status[Now_Status_Serial].Time = 0;
        Set_Status(0);
    }
    break;
    }
}
#endif

#ifdef MOTOR_TEST_CHASSIS
void Class_Chariot::Init_Motor_Test_Chassis()
{
    // 电机PID初始化
    Test_Motor.PID_Omega.Init(1200.0f, 0.0f, 0.0f, 0.0f, Test_Motor.Get_Output_Max(), Test_Motor.Get_Output_Max());
    Test_Motor.PID_Angle.Init(4.5f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f * PI);
    // 电机ID初始化
    Test_Motor.Init(&hfdcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);
}

void Class_Chariot::Output_Motor_Test_Chassis()
{
    // static uint8_t ms_cnt = 0;
    // ms_cnt++;

    Test_Motor.Set_DJI_Motor_Control_Method(Test_Method);
    Test_Motor.Set_Target_Radian(target_angle);
    Test_Motor.Set_Target_Omega_Radian(target_omega);

    // 模拟实际中的电机通信，500hz，每2ms与一个电机通信一次
    // if(ms_cnt % 2 == 0)
    // {
    calculate_s = DWT_GetDeltaT(&cal_cnt);
    Test_Motor.TIM_PID_PeriodElapsedCallback();
    // ms_cnt = 0;
    //}
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
