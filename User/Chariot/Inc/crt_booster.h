/**
 * @file crt_booster.h
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 摩擦轮编号
 * 1 2
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_servo.h"
#include "dvc_TensionMeter.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Class_Booster;

/**
 * @brief 发射机构控制类型
 *
 */
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_NORMAL,
    Booster_Control_Type_Push_CALIBRATION,
    Booster_Control_Type_Pull_CALIBRATION,
    // Booster_Control_Type_Shooting,-- IGNORE --
};

/**
 * @brief 发射过程控制类型
 *
 */
enum Enum_Shooting_Control_Type
{
    Shooting_Control_Type_WAITING= 0,
    Shooting_Control_Type_INIT,
    Shooting_Control_Type_READY_PRE,
    Shooting_Control_Type_READY_PRE2,
    Shooting_Control_Type_READY,
    Shooting_Control_Type_PULLRING,
    Shooting_Control_Type_SHOOTING,
    Shooting_Control_Type_SHOOTING_FINISHED,
};

/*
 * @brief 换弹机构所处状态
 *
 */
enum Enum_Reload_Status
{
    Reload_Status_DISABLE = 0, // 失能
    Reload_Status_RELOADING,   // 换弹中状态
    Reload_Status_FINISHED,    // 换弹完成状态
};

/**
 * @brief 换弹过程控制类型
 *
 */
enum Enum_Reload_Control_Type
{
    Reload_Control_Type_UNCALIBRATED = 0, // 没校准完
    Reload_Control_Type_INIT,             // 校准完的初始状态
    Reload_Control_Type_WAITING,          //等待上膛滑块到位
    Reload_Control_Type_PUSHING,          // 上弹推进过程
    Reload_Control_Type_RETRACTING,       // 换弹机构回退过程（给发射机构让路）
    Reload_Control_Type_HOLD,             // 保持当前角度不动状态
    Reload_Control_Type_Test,             // 测试用状态
};

/**
 * @brief Specialized, 有限自动机->发射过程状态机
 *
 */
class Class_FSM_Shooting : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Shooting_TIM_Status_PeriodElapsedCallback();
    Enum_Shooting_Control_Type Shooting_Control_Type = Shooting_Control_Type_INIT;

    float push_speed =  60.0f;
};

/*
 * @brief Specialized, 有限自动机->换弹机构状态机
 *
 */
class Class_FSM_Reload : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();

    Enum_Reload_Control_Type Reload_Control_Type = Reload_Control_Type_UNCALIBRATED;
};

/**
 * @brief Specialized, 有限自动机->皮筋电机校准状态机
 *
 */
class Class_FSM_Push_Calibration : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold_up = 3000.0f;
    float Torque_Threshold_down = 9000.0f;
    float speed = 60.0f;

    float Angle_Forward_L = 0.0f;
    float Angle_Backward_L = 0.0f;
    float Angle_Forward_R = 0.0f;
    float Angle_Backward_R = 0.0f;

    int forward_flag_L = 0;
    int forward_flag_R = 0;
    int backward_flag_L = 0;
    int backward_flag_R = 0;

    void Push_Calibration_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

/**
 * @brief Specialized, 有限自动机->拉力电机校准状态机
 *
 */
class Class_FSM_Pull_Calibration : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold = 2300.0f;//之前是1500 2000 
    float speed = 40.0f;

    float Angle_Forward = 0.0f;
    float Angle_Backward = 0.0f;

    void Pull_Calibration_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

/**
 * @brief Specialized, 有限自动机->换弹机构->直线电机校准状态机
 *
 */
class Class_FSM_Reload_Linear_Calibration : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold = 1050.0f;
    float speed = 20.0f;

    float Angle_Forward = 0.0f;
    float Angle_Backward = 0.0f;

    void Linear_Calibration_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:
    // 发射有限自动机
    Class_FSM_Shooting FSM_Shooting;
    friend class Class_FSM_Shooting;

    // 换弹机构有限自动机
    Class_FSM_Reload FSM_Reload;
    friend class Class_FSM_Reload;

    // 皮筋电机校准
    Class_FSM_Push_Calibration FSM_Push_Calibration;
    friend class Class_FSM_Push_Calibration;

    // 拉力电机校准
    Class_FSM_Pull_Calibration FSM_Pull_Calibration;
    friend class Class_FSM_Pull_Calibration;

    // 换弹机构直线电机校准
    Class_FSM_Reload_Linear_Calibration FSM_Reload_Linear_Calibration;

    // 裁判系统
    Class_Referee *Referee;
    // 上位机
    Class_MiniPC *MiniPC;

    // 270°舵机->撒放器
    Class_Servo Servo_Trigger;
    Class_Servo Servo_Reload;

    // 拉力机
    Class_TensionMeter TensionMeter = Class_TensionMeter(0x01);

    // 发射电机
    Class_DJI_Motor_C620 Motor_Pull;

    Class_DJI_Motor_C620 Motor_Push_L;
    Class_DJI_Motor_C620 Motor_Push_R;

    // 换弹电机
    Class_DJI_Motor_GM6020 Motor_Reload_Angle;
    Class_DJI_Motor_C610 Motor_Reload_Linear;

    void Pull_Tension_Control(bool is_first_run);

    void Init();

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();
    inline Enum_Shooting_Control_Type Get_Shooting_Control_Type();
    inline Enum_Reload_Control_Type Get_Reload_Control_Type();
    inline Enum_Reload_Status Get_Reload_Status();

    inline int Get_Target_PushMotor_Angle();
    inline int Get_Target_PullMotor_Angle();
    inline int Get_Measured_Tension();
    inline int Get_Target_Tension();
    inline float Get_Target_position_push();
    inline float Get_Target_position_pull();
    inline float Get_Now_position_push();
    inline float Get_Now_position_pull();
    inline float Get_Now_position_reload_linear();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Shooting_Control_Type(Enum_Shooting_Control_Type __Shooting_Control_Type);
    inline void Set_Reload_Status(Enum_Reload_Status __Reload_Status);
    inline void Set_Target_PushMotor_Angle(float __Target_PushMotor_Angle);
    inline void Set_Target_PullMotor_Angle(float __Target_PullMotor_Angle);
    inline void Set_Measured_Tension(int __Measured_Tension);
    inline void Set_Target_Tension(int __Target_Tension);
    inline void Set_Target_position_push(float __target_position_push);
    inline void Set_Target_position_pull(float __target_position_pull);
    inline void Set_Now_position_push(float __now_position_push);
    inline void Set_Now_position_pull(float __now_position_pull);
    inline void Set_Now_position_reload_linear(float __now_position_reload_linear);

    void TIM_Calculate_PeriodElapsedCallback();
    void Output();

protected:
    // 初始化相关常量

    /*----------------------------push与pull----------------------------------*/
    // 校准完成标志位
    // bool Push_Calibration_Finished = false;
    // bool Pull_Calibration_Finished = false;

    float target_position_push = 0.95f; // 校准完成后push电机目标位置
    float target_position_pull = 0.5f;  // 校准完成后pull电机目标位置

    float now_position_push = 0.0f; // 当前push电机位置
    float now_position_pull = 0.0f; // 当前pull电机位置

    /*----------------------------reload----------------------------------*/

    // 对于6020而言 由于是弧度制 所以要写成 多少多少度 // 180*pi
    float init_position_reload_angle = 79.0f * PI / 180.0f;  // 校准完成后Angle电机初始位置
    float init_position_reload_linear = 0.92f; // 校准完成后Linear电机初始位置

    // 在初始化的时候直接先把init的值赋给target得了 方便循环赋值 上面的init不用了----------------

    float target_position_reload_angle = 0.f;  // 换弹机构角度电机目标位置
    float target_position_reload_linear = 0.85f; // 换弹机构线性电机目标位置

    float now_position_reload_angle = 0.0f;  // 当前angle电机位置
    float now_position_reload_linear = 0.0f; // 当前linear电机位置

    /*----------------------------servo----------------------------------*/
    float tirrger_fire_angle = 260.0f; // 舵机发射角度
    float tirrger_reset_angle = 120.0f; // 舵机复位角度

    float reload_lift_angle = 220.0f; // 舵机换弹抬起角度
    float reload_drop_angle = 18.0f;  // 舵机换弹放下角度

    /*----------------------------tension----------------------------------*/
    // 拉力相关变量
    float Measured_Tension = 0;     // 测量的拉力值
    float Target_Tension = 35000.0f; // 目标的拉力值，单位g

    // 拉力环相关变量
    float now_tension_value = 0.0f;                            // 当前测得的拉力值
    float target_tension_value = 0.0f;                         // 目标拉力值
    float target_tension_position_pull = target_position_pull; // 拉力环下 Pull 电机目标位置，初始由 target_position_pull 提供

    // 发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_DISABLE;
    // 换弹机构状态
    Enum_Reload_Status Reload_Status = Reload_Status_DISABLE;

    // 读写变量-> 无用还没删除
    float Target_PushMotor_Angle = 0.0f;
    float Target_PullMotor_Angle = 0.0f;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获得发射机构状态
 *
 * @return Enum_Booster_Control_Type 发射机构状态
 */
Enum_Booster_Control_Type Class_Booster::Get_Booster_Control_Type()
{
    return (Booster_Control_Type);
}

inline Enum_Shooting_Control_Type Class_Booster::Get_Shooting_Control_Type()
{
    return (FSM_Shooting.Shooting_Control_Type);
}

/**
 * @brief 获得换弹机构状态
 *
 * @return  Enum_Reload_Status 换弹机构状态
 */
Enum_Reload_Status Class_Booster::Get_Reload_Status()
{
    return Reload_Status;
}

inline Enum_Reload_Control_Type Class_Booster::Get_Reload_Control_Type()
{
    return (FSM_Reload.Reload_Control_Type);
}

int Class_Booster::Get_Target_PushMotor_Angle()
{
    return Target_PushMotor_Angle;
}

int Class_Booster::Get_Target_PullMotor_Angle()
{
    return Target_PullMotor_Angle;
}

/**
 * @brief 获取当前拉力,
 *
 * @return int 获取当前拉力
 */
inline int Class_Booster::Get_Measured_Tension()
{
    return (Measured_Tension);
}

inline int Class_Booster::Get_Target_Tension()
{
    return (Target_Tension);
}

inline float Class_Booster::Get_Target_position_push()
{
    return (target_position_push);
}

inline float Class_Booster::Get_Target_position_pull()
{
    return (target_position_pull);
}

inline float Class_Booster::Get_Now_position_push()
{
    return (now_position_push);
}

inline float Class_Booster::Get_Now_position_pull()
{
    return (now_position_pull);
}

inline float Class_Booster::Get_Now_position_reload_linear()
{
    return (now_position_reload_linear);
}

/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief 设定发射过程控制状态
 *
 * @param __Shooting_Control_Type 发射过程控制状态
 */
inline void Class_Booster::Set_Shooting_Control_Type(Enum_Shooting_Control_Type __Shooting_Control_Type)
{
    FSM_Shooting.Shooting_Control_Type = __Shooting_Control_Type;
}

inline void Class_Booster::Set_Reload_Status(Enum_Reload_Status __Reload_Status)
{
    Reload_Status = __Reload_Status;
}

inline void Class_Booster::Set_Target_PushMotor_Angle(float __Target_PushMotor_Angle)
{
    Target_PushMotor_Angle = __Target_PushMotor_Angle;
}

inline void Class_Booster::Set_Target_PullMotor_Angle(float __Target_PullMotor_Angle)
{
    Target_PullMotor_Angle = __Target_PullMotor_Angle;
}

/**
 * @brief 设定测量拉力
 *
 * @param __Measured_Tension 测量拉力
 */
void Class_Booster::Set_Measured_Tension(int __Measured_Tension)
{
    Measured_Tension = __Measured_Tension;
}

/**
 * @brief 设置目标拉力,
 *
 * @return int 设置目标拉力
 */
void Class_Booster::Set_Target_Tension(int __Target_Tension)
{
    Target_Tension = __Target_Tension;
}

inline void Class_Booster::Set_Target_position_push(float __target_position_push)
{
    target_position_push = __target_position_push;
}

inline void Class_Booster::Set_Target_position_pull(float __target_position_pull)
{
    target_position_pull = __target_position_pull;
}

inline void Class_Booster::Set_Now_position_push(float __now_position_push)
{
    now_position_push = __now_position_push;
}

inline void Class_Booster::Set_Now_position_pull(float __now_position_pull)
{
    now_position_pull = __now_position_pull;
}

inline void Class_Booster::Set_Now_position_reload_linear(float __now_position_reload_linear)
{
    now_position_reload_linear = __now_position_reload_linear;
}

#endif /* CRT_BOOSTER_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
