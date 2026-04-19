/**
 * @file ita_chariot.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef TSK_INTERACTION_H
#define TSK_INTERACTION_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"
#include "dvc_VT13.h"
#include "crt_gimbal.h"
#include "crt_booster.h"
#include "dvc_imu.h"
#include "tsk_config_and_callback.h"
#include "dvc_supercap.h"
#include "crt_chassis.h"
#include "crt_force_control_chassis.h"
#include "config.h"
#include "alg_filter.h"
#include "arm_model.h"

/* Exported macros -----------------------------------------------------------*/
class Class_Chariot;
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘通讯状态
 *
 */
enum Enum_Chassis_Status
{
    Chassis_Status_DISABLE = 0,
    Chassis_Status_ENABLE,
};

/**
 * @brief 云台通讯状态
 *
 */
enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_ENABLE,
};

/**
 * @brief DR16控制数据来源
 *
 */
enum Enum_DR16_Control_Type
{
    DR16_Control_Type_REMOTE = 0,
    DR16_Control_Type_KEYBOARD,
    DR16_Control_Type_NONE,
};

/**
 * @brief VT13控制数据来源
 *
 */
enum Enum_VT13_Control_Type
{
    VT13_Control_Type_REMOTE = 0,
    VT13_Control_Type_KEYBOARD,
    VT13_Control_Type_NONE,
};

/**
 * @brief 活动遥控器类型
 *
 */
enum Enum_Active_Controller
{
    Controller_NONE = 0,
    Controller_DR16,
    Controller_VT13
};

/**
 * @brief 键鼠模式下机器人工况模式
 *
 */
enum Enum_Keyboard_Control_Type
{
    Keyboard_Control_Type_MOVING,   // 平地移动
    Keyboard_Control_Type_WORKING,  // 取兑矿模式
    Keyboard_Control_Type_UPLIFT,   // 上台阶模式
    Keyboard_Control_Type_DOWNLIFT  // 下台阶模式
};

/**
 * @brief 机器人是否离线 控制模式有限自动机
 *
 */
class Class_FSM_Alive_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

class Class_FSM_Alive_Control_VT13 : public Class_FSM
{
public:
    uint8_t Start_Flag = 0; // 记录第一次上电开机，以初始化状态
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

// 自定义控制器数据格式
struct Struct_Offline_Controller_Data
{
    float Angle[6];
    bool gripper_status;
} __attribute__((packed));

// 云台发送底盘相关通信帧格式
// Data[6] 的位域定义
struct LiftControl {
    uint8_t lift_select : 4; // 0-3位：控制四个抬升，0为不选中，1为选中
    uint8_t lift_direction   : 4; // 4-7位：控制升高/降低，0为降低，1为抬高
} __attribute__((packed));

// Data[7] 的位域定义
struct OtherStatus {
    uint8_t chassis_contorl_mode     : 2; // 0-1位：底盘控制模式，00:失能，01：使能，10：上台阶状态机
    uint8_t uplift_fsm_direction     : 1; // 2位：上台阶状态机前进/保持
    uint8_t backdoor_jump            : 1; // 3位：150mm台阶后门跳转开关
    uint8_t downlift_init            : 1; // 4位：下台阶初始抬升高度跳转
    uint8_t wheel_slave_ctrl         : 1; // 5位：小轮子从动开关
    uint8_t reserved                 : 2; // 6-7位：保留
} __attribute__((packed));

// 完整的协议数据结构
struct Gimbal_Tx_Chassis_Frame {
    int16_t x_velocity;      // Data[0,1] X轴速度
    int16_t y_velocity;      // Data[2,3] Y轴速度
    int16_t yaw_data;        // Data[4,5] Yaw速度/角度
    LiftControl lift;        // Data[6]   抬升控制
    OtherStatus status;     // Data[7]   其他状态
}__attribute__((packed));

/**
 * @brief 控制对象
 *
 */
class Class_Chariot
{
public:
    // 裁判系统
    Class_Referee Referee;
    // 底盘
    Class_Mecanum_Chassis Chassis;
    // 力控底盘
    Class_Chassis Force_Chassis;

#ifdef GIMBAL
    // 遥控器
    Class_DR16 DR16;
    Class_VT13 VT13;
    // 上位机
    Class_MiniPC MiniPC;
    // 云台
    Class_Gimbal Gimbal;
    // 发射机构
    Class_Booster Booster;

    // 离线状态下自定义控制器数据
    Struct_Offline_Controller_Data Offline_Controller_Data;

    uint8_t UART1_Buffer[14];

    // 遥控器离线保护控制状态机
    Class_FSM_Alive_Control FSM_Alive_Control;
    friend class Class_FSM_Alive_Control;

    Class_FSM_Alive_Control_VT13 FSM_Alive_Control_VT13;
    friend class Class_FSM_Alive_Control_VT13;

#elifdef CHASSIS_TEST
    Class_DR16 DR16;
    float DR16_Dead_Zone = 0.3f;
    void Chassis_Test_Control();
    // 遥控器离线保护控制状态机
    Class_FSM_Alive_Control FSM_Alive_Control;
    friend class Class_FSM_Alive_Control;

    Enum_Chassis_Control_Type Pre_Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Chassis_Control_Type__ Pre_Chassis_Control_Type__ = Chassis_Control_Type_DISABLE__;
    inline Enum_Chassis_Control_Type Get_Pre_Chassis_Control_Type();
    inline Enum_Chassis_Control_Type__ Get_Pre_Chassis_Control_Type__();
    inline void Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Pre_Chassis_Control_Type__(Enum_Chassis_Control_Type__ __Chassis_Control_Type);

#endif

#ifdef MOTOR_TEST_CHASSIS
    Class_DJI_Motor_C620 Test_Motor;
    void Init_Motor_Test_Chassis();
    void Output_Motor_Test_Chassis();
    float target_omega = 0.0f; // rad
    float target_angle = 0.0f; // rad
    Enum_DJI_Motor_Control_Method Test_Method = DJI_Motor_Control_Method_OMEGA;
#endif

    void Init(float __DR16_Dead_Zone = 0);

#ifdef CHASSIS

    void CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data);
    void CAN_Chassis_Tx_Gimbal_Callback();
    void TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
    inline void Set_Gimbal_Status(Enum_Gimbal_Status __Gimbal_Status);
    inline Enum_Gimbal_Status Get_Gimbal_Status();

#elif defined(GIMBAL)

    inline void DR16_Offline_Cnt_Plus();

    inline uint16_t Get_DR16_Offline_Cnt();
    inline void Clear_DR16_Offline_Cnt();

    inline Enum_Chassis_Control_Type Get_Pre_Chassis_Control_Type();
    inline Enum_Gimbal_Control_Type Get_Pre_Gimbal_Control_Type();
    inline Enum_Booster_Control_Type Get_Pre_Booster_Control_Type();

    inline void Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);

    inline Enum_Chassis_Status Get_Chassis_Status();
    // inline Enum_DR16_Control_Type Get_DR16_Control_Type();
    // inline Enum_VT13_Control_Type Get_VT13_Control_Type();

    void CAN_Gimbal_Rx_Chassis_Callback();
    void CAN_Gimbal_Tx_Chassis_Callback();

    void TIM_Control_Callback();

    void TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();
#endif

    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_Unline_Protect_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

    // 底盘云台通讯变量
    // 冲刺
    Enum_Sprint_Status Sprint_Status = Sprint_Status_DISABLE;
    // 迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    // 裁判系统UI刷新状态
    Enum_Referee_UI_Refresh_Status Referee_UI_Refresh_Status = Referee_UI_Refresh_Status_DISABLE;
    // 数据帧结构体
    Gimbal_Tx_Chassis_Frame Rx_Frame;
    
    void Judge_DR16_Control_Type();
    void Judge_VT13_Control_Type();

    void Control_Chassis();

    // 角度目标值
    float tmp_j0_pitch_radian, tmp_j1_yaw_radian, tmp_j2_yaw_radian, tmp_j3_roll_radian, tmp_j4_pitch_radian, tmp_j5_roll_radian;
    uint8_t tmp_gripper_position;
    float tmp_gripper_radian;

    // 遥控器摇杆值
    float dr16_right_x, dr16_right_y, dr16_left_x, dr16_left_y, dr16_yaw;
    float vt13_right_x, vt13_right_y, vt13_left_x, vt13_left_y, vt13_yaw;

protected:
    // 初始化相关常量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = &CAN3_Manage_Object;

#ifdef CHASSIS
    // 写变量
    uint32_t Gimbal_Alive_Flag = 0;
    uint32_t Pre_Gimbal_Alive_Flag = 0;

    Enum_Gimbal_Status Gimbal_Status = Gimbal_Status_DISABLE;
#endif

#ifdef GIMBAL
    // 机械臂在线状态，当所有电机全掉线时视为机械臂掉线，需要重新arm_init
    bool is_arm_online = true;

    // 遥控器相关变量
    // 遥控器拨动的死区, 0~1
    float DR16_Dead_Zone = 0.3f;
    // 常量
    // 键鼠模式按住shift 最大速度缩放系数
    float DR16_Mouse_Chassis_Shift = 2.0f;

    // DR16调试机械臂相关常量
    float DR16_J0_Pitch_Resolution = 0.001f * PI;
    float DR16_J1_Yaw_Resolution = 0.00075f * PI;
    float DR16_J2_Yaw_Resolution = 0.00075f * PI;
    float DR16_J3_Roll_Resolution = 0.0015f * PI;
    float DR16_J4_Pitch_Resolution = 0.0015f * PI;
    float DR16_J5_Roll_Resolution = 0.0015f * PI;
    // DR16控制夹爪的速度，因为夹爪可移动范围是0.0 rad - 0.95 rad，所以现在给到0.05rad/s，
    float DR16_Gripper_Resolution = 0.005f * PI;

    // 键鼠控制相关常量

    // DR16鼠标底盘yaw角度控制灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Chassis_Yaw_Angle_Resolution = 57.8f * 10.0f;
    // DR16鼠标底盘yaw角速度控制灵敏度系数, 不同鼠标不同参数
    float DR16_Mouse_Chassis_Yaw_Omega_Resolution = 57.8f * 10.0f;

    // 鼠标控制VT03图传Yaw角度灵敏度系数
    float DR16_Mouse_VT03_Yaw_Angle_Resolution = 57.8f * 10.0f;
    // 鼠标控制VT03图传Pitch角度灵敏度系数
    float DR16_Mouse_VT03_Pitch_Angle_Resolution = 57.8f * 10.0f;
    // 键鼠控制模式下机器人工况模式
    Enum_Keyboard_Control_Type Keyboard_Control_Type = Keyboard_Control_Type_MOVING;

    // 内部变量
    // 遥控器离线计数
    uint16_t DR16_Offline_Cnt = 0;
    // 拨盘发射标志位
    uint16_t Shoot_Cnt = 0;
    // 读变量
    float True_Mouse_X;
    float True_Mouse_Y;
    float True_Mouse_Z;
    // 写变量
    uint32_t Chassis_Alive_Flag = 0;
    uint32_t Pre_Chassis_Alive_Flag = 0;
    // 读写变量
    Enum_Chassis_Status Chassis_Status = Chassis_Status_DISABLE;

    // 底盘 云台 发射机构 前一帧控制类型
    Enum_Chassis_Control_Type Pre_Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Gimbal_Control_Type Pre_Gimbal_Control_Type = Gimbal_Control_Type_NORMAL;
    Enum_Booster_Control_Type Pre_Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

    // 单发连发标志位
    uint8_t Shoot_Flag = 0;
    // DR16控制数据来源
    Enum_DR16_Control_Type DR16_Control_Type = DR16_Control_Type_REMOTE;
    Enum_VT13_Control_Type VT13_Control_Type = VT13_Control_Type_NONE;
    // 当前使用的遥控器
    Enum_Active_Controller Active_Controller = Controller_NONE;
    // 内部函数

    // 判断当前使用的遥控器
    void Judge_Active_Controller();
    // 获取当前使用的遥控器
    Enum_Active_Controller Get_Active_Controller();
    // 获取DR16控制类型
    Enum_DR16_Control_Type Get_DR16_Control_Type();
    // 获取VT13控制类型
    Enum_VT13_Control_Type Get_VT13_Control_Type();

    // void Judge_DR16_Control_Type();

    // void Control_Chassis();
    void Control_Gimbal();
    void Control_Booster();

    void Transform_Mouse_Axis();
#endif
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#ifdef GIMBAL
/**
 * @brief 获取底盘通讯状态
 *
 * @return Enum_Chassis_Status 底盘通讯状态
 */
Enum_Chassis_Status Class_Chariot::Get_Chassis_Status()
{
    return (Chassis_Status);
}

/**
 * @brief 获取前一帧底盘控制类型
 *
 * @return Enum_Chassis_Control_Type 前一帧底盘控制类型
 */

Enum_Chassis_Control_Type Class_Chariot::Get_Pre_Chassis_Control_Type()
{
    return (Pre_Chassis_Control_Type);
}

/**
 * @brief 获取前一帧云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 前一帧云台控制类型
 */

Enum_Gimbal_Control_Type Class_Chariot::Get_Pre_Gimbal_Control_Type()
{
    return (Pre_Gimbal_Control_Type);
}

/**
 * @brief 获取前一帧发射机构控制类型
 *
 * @return Enum_Booster_Control_Type 前一帧发射机构控制类型
 */
Enum_Booster_Control_Type Class_Chariot::Get_Pre_Booster_Control_Type()
{
    return (Pre_Booster_Control_Type);
}

/**
 * @brief 设置前一帧底盘控制类型
 *
 * @param __Chassis_Control_Type 前一帧底盘控制类型
 */
void Class_Chariot::Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Pre_Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设置前一帧云台控制类型
 *
 * @param __Gimbal_Control_Type 前一帧云台控制类型
 */
void Class_Chariot::Set_Pre_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Pre_Gimbal_Control_Type = __Gimbal_Control_Type;
}

/**
 * @brief 设置前一帧发射机构控制类型
 *
 * @param __Booster_Control_Type 前一帧发射机构控制类型
 */
void Class_Chariot::Set_Pre_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Pre_Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief DR16离线计数加一
 */
void Class_Chariot::DR16_Offline_Cnt_Plus()
{
    DR16_Offline_Cnt++;
}

/**
 * @brief 获取DR16离线计数
 *
 * @return uint16_t DR16离线计数
 */
uint16_t Class_Chariot::Get_DR16_Offline_Cnt()
{
    return (DR16_Offline_Cnt);
}

/**
 * @brief DR16离线计数置0
 *
 */
void Class_Chariot::Clear_DR16_Offline_Cnt()
{
    DR16_Offline_Cnt = 0;
}

#endif

#ifdef CHASSIS_TEST
/**
 * @brief 获取前一帧底盘控制类型
 *
 * @return Enum_Chassis_Control_Type 前一帧底盘控制类型
 */

Enum_Chassis_Control_Type Class_Chariot::Get_Pre_Chassis_Control_Type()
{
    return (Pre_Chassis_Control_Type);
}

/**
 * @brief 设置前一帧底盘控制类型
 *
 * @param __Chassis_Control_Type 前一帧底盘控制类型
 */
void Class_Chariot::Set_Pre_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Pre_Chassis_Control_Type = __Chassis_Control_Type;
}

Enum_Chassis_Control_Type__ Class_Chariot::Get_Pre_Chassis_Control_Type__()
{
    return (Pre_Chassis_Control_Type__);
}

void Class_Chariot::Set_Pre_Chassis_Control_Type__(Enum_Chassis_Control_Type__ __Chassis_Control_Type)
{
    Pre_Chassis_Control_Type__ = __Chassis_Control_Type;
}
#endif

#ifdef CHASSIS
void Class_Chariot::Set_Gimbal_Status(Enum_Gimbal_Status __Gimbal_Status)
{
    Gimbal_Status = __Gimbal_Status;
}

Enum_Gimbal_Status Class_Chariot::Get_Gimbal_Status()
{
    return Gimbal_Status;
}

#endif

#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
