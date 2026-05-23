/**
 * @file crt_chassis.h
 * @author cjw by wanghongxi
 * @brief 底盘
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"
#include "dvc_TOFSense.h"
#include "alg_power_limit.h"
#include "dvc_supercap.h"
#include "config.h"
#include "dvc_minipc.h"
#include "alg_fsm.h"
#include "dvc_dr16.h"
#include "dvc_dwt.h"
/* Exported macros -----------------------------------------------------------*/
#define wheel_diameter 0.14100000f  // 轮子直径，m
#define half_width 0.159f           // m
#define half_length 0.152f          // m
#define ROTATION_CENTER_OFFSET 0.0f // 旋转中心位置偏移量，现在只有y方向上的偏移，且向y负方向偏移，这个偏移量为绝对值

#define THETA_A atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_B atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_C atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_D atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）

#define R_A half_width / cos(THETA_A) // 旋转中心与A舵轮的距离
#define R_B half_width / cos(THETA_B) // 旋转中心与B舵轮的距离
#define R_C half_width / cos(THETA_C) // 旋转中心与C舵轮的距离
#define R_D half_width / cos(THETA_D) // 旋转中心与D舵轮的距离

#define PI 3.141593f
#define PI2 2 * PI
#define RPM2RAD 0.104720f                // 		1 rpm = 2pi/60 rad/s
#define RPM2VEL 0.523599f                // 		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f                //
#define M2006_REDUCTION_RATIO 36.000000f //
#define M3508_REDUCTION_RATIO 19.000000f //
#define GM6020_ENCODER_ANGLE 8192.0f

#define RAD_TO_8191 8191.0f / PI / 2.0f

#define RAD2MM (280.0f / 26.5f)
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘右拨杆和抬升状态枚举
 *
 */
enum Enum_Chassis_DR16_Right_Uplift_Status{
    DR16_Right_Switch_MIDDLE = 0,
    DR16_Right_Switch_UP,
    DR16_Right_Switch_DOWN,
    Chassis_Uplift_FLAG_ADD,
    Chassis_Uplift_WHEEL_ON,
};

/**
 * @brief 抬升电机控制枚举
 *
 */
enum Enum_Lift_Direction : uint8_t
{
    Lift_Direction_DOWN = 0,
    Lift_Direction_UP,
    Lift_Direction_HOLD
};

/**
 * @brief 上台阶状态机运行方向枚举
 *
 */
enum Enum_Uplift_FSM_Direction : uint8_t
{
    Uplift_FSM_HOLD = 0,
    Uplift_FSM_FORWARD
};

/**
 * @brief 小轮子从动状态枚举
 *
 */
enum Enum_Wheel_Slave_Status : uint8_t
{
    Wheel_Slave_OFF = 0,
    Wheel_Slave_ON
};

/**
 * @brief 底盘冲刺状态枚举
 *
 */
enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0,
    Sprint_Status_ENABLE,
};

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type : uint8_t
{
    Chassis_Control_Type_DISABLE = 0x00,
    Chassis_Control_Type_NORMAL = 0x01,
    Chassis_Control_Type_SLOPE = 0x02,
};

/**
 * @brief Specialized, 三轮舵轮底盘类
 *
 */
// omnidirectional 全向轮
class Class_Tricycle_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;

    // 功率限制
    Class_Power_Limit Power_Limit;
    // Struct_Power_Management Power_Management;

    // 裁判系统
    Class_Referee *Referee;

    // 下方转动电机
    Class_DJI_Motor_C620 Motor_Wheel[4];
    Class_DJI_Motor_C620_Steer Motor_Steer[4];

    // 随动环
    Class_PID Chassis_Follow_PID_Angle;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f, float __Steer_Power_Ratio = 0.5);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Steer_Power();
    inline float Get_Target_Steer_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Spin_Omega();
    inline float Get_Relative_Angle();
    inline Enum_Supercap_Mode Get_Supercap_Mode();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Relative_Angle(float __Relative_Angle);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);
    void Power_Limit_Update();

protected:
    // 初始化相关常量

    // 速度X限制
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;
    // 舵向电机功率上限比率
    float Steer_Power_Ratio = 0.5f;
    // 底盘小陀螺模式角速度
    float Spin_Omega = 4.0f;
    // 常量

    // 电机理论上最大输出
    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    // 内部变量
    float Relative_Angle = 0.0f;

    // 舵向电机目标值
    float Target_Steer_Angle[3];
    // 转动电机目标值
    float Target_Wheel_Omega[4];

    // 读变量

    // 当前总功率
    float Now_Power = 0.0f;
    // 当前舵向电机功率
    float Now_Steer_Power = 0.0f;
    // 可使用的舵向电机功率
    float Target_Steer_Power = 0.0f;
    // 当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    // 可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Supercap_Mode Supercap_Mode = Supercap_DISABLE;
    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标角速度
    float Target_Omega = 0.0f;
    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 内部函数
    void Speed_Resolution();
};

/* Exported variables --------------------------------------------------------*/
#ifdef ENGINEER
// 工程底盘参数
//  轮子直径 单位m
const float WHELL_DIAMETER = 0.141f; // m
// 线速度转角速度 rad/s
const float VEL2RAD = 1.0f / (WHELL_DIAMETER / 2.0f); // v = omega * r, omega = v / r， v与r单位保持一致，使用m
#else
// 三轮车底盘参数

// 轮组半径
const float WHEEL_RADIUS = 0.0520f;

// 轮距中心长度
const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

// 前心距中心长度
const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

// 前后轮距
const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

// 前轮距前心
const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

// 轮组方位角
const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

// 轮子直径 单位m
const float WHELL_DIAMETER = 0.13f;

// 底盘半宽 单位m
const float HALF_WIDTH = 0.281f;

// 底盘半长 单位m
const float HALF_LENGTH = 0.281f;

// 底盘中心到每个轮子轴心投影距离
const float CHASSIS_RADIUS = sqrt(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH);

// 线速度转角速度 rad/s
const float VEL2RAD = 1.0f / (WHELL_DIAMETER / 2.0f);
#endif
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Tricycle_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Tricycle_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Tricycle_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_Tricycle_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

/**
 * @brief 获取可使用的舵向电机功率
 *
 * @return float 当前舵向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Tricycle_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

float Class_Tricycle_Chassis::Get_Relative_Angle()
{
    return (Relative_Angle);
}
Enum_Supercap_Mode Class_Tricycle_Chassis::Get_Supercap_Mode()
{
    return (Supercap_Mode);
}
/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Tricycle_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Tricycle_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Tricycle_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Tricycle_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Tricycle_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

void Class_Tricycle_Chassis::Set_Relative_Angle(float __Relative_Angle)
{
    Relative_Angle = __Relative_Angle;
}
void Class_Tricycle_Chassis::Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode)
{
    Supercap_Mode = __Supercap_Mode;
}

class Class_Mecanum_Chassis;
class Class_Chassis;
class Class_FSM_Calibration_Chassis : public Class_FSM
{
public:
    Class_Mecanum_Chassis *Chassis;
    void Reload_TIM_Status_PeriodElapsedCallback();
    void Request_Recalibration();

    /*电机校准执行函数*/
    bool Motor_Calibration(Class_DJI_Motor_C620 *Motor, uint8_t index, float locked_torque, uint16_t &locked_cnt);

    inline bool Get_Uplift_cali_status(uint8_t index);

    bool uplift_cali = false;

protected:
    /*抬升校准状态机相关变量*/
    float uplift_offset[4] = {0.0f};
    float uplift_cali_torque = 10000.0f;
    bool uplift_cali_status[4] = {false};
    bool uplift_online_status[4] = {false};

    uint16_t uplift_locked_cnt[4] = {0}; // 抬升堵转时间计数
};

class Class_FSM_Ledder : public Class_FSM
{
public:
    Class_Mecanum_Chassis *Chassis;
    Class_Chassis *Force_Chassis = nullptr;
    
    Enum_Uplift_FSM_Direction FSM_Direction = Uplift_FSM_HOLD;

    float Yaw = 0.0f;
    float Yaw_Delta_s = 0.0f;

    uint32_t Delta_s = 0.0f;

    uint8_t TRIGGER_CNT = 0;
    uint16_t FORWARD_CNT = 0;

    void Init_Attitude_Control();
    void Reload_TIM_Status_PeriodElapsedCallback();

protected:
    void Reset_Attitude_Control();
    bool Get_Attitude_Feedback(float *pitch, float *roll);
    void Apply_Attitude_Control(const float *base_target,
                                const float *lower_limit,
                                const float *upper_limit,
                                bool unilateral_pitch_limit,
                                float pitch_target_rad,
                                float window_limit_rad = 0.0f);
    /*上台阶相关角度*/
    float ledder_prepare[4] = {22.0f, 22.0f, 20.0f, 20.0f};

    float ledder_1_touch[4] = {18.0f, 18.0f, 15.0f, 15.0f};
    float ledder_1_uplift[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float ledder_1_over[4] = {22.0f, 22.0f, 20.0f, 20.0f};
    float ledder_2_touch[4] = {11.0f, 11.0f, 17.0f, 17.0f};
    float ledder_2_uplift[4] = {0.0f, 0.0f, 2.5f, 2.5f};
    float ledder_2_over[4] = {22.0f, 22.0f, 20.0f, 20.0f};
    float Base_Uplift_Target[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 抬升机构触地状态判断相关变量
    float Touched_Torque;

    Class_PID Pitch_PID;
    Class_PID Roll_PID;

    // Index map:
    // 0 - right front, 1 - left front, 2 - left rear, 3 - right rear
    // Sign convention:
    // Pitch > 0: front side goes down.
    // Roll > 0: right side goes up.
    // TODO: replace with the final commissioning value if a different pitch is required.
    float Pitch_Offset = 0.0f;
    float Init_Pitch_Target_Base_Rad = -(5.0f / 180.0f) * PI;
    float Init_Pitch_Target_Rad = -(5.0f / 180.0f) * PI;
    float Init_Attitude_Window_Rad = 0.8f;
    float Init_Attitude_Out_Max = 0.8f;
    float Lift_Attitude_Out_Max = 1.0f;
    float Attitude_Dead_Zone_Rad = 0.0087f;

    float Pitch_Mix[4] = {1.0f, 1.0f, -1.0f, -1.0f};
    float Roll_Mix[4] = {-1.0f, 1.0f, 1.0f, -1.0f};

    Enum_DR16_Switch_Status Judge_DR16_Switch_Status(Enum_DR16_Switch_Status Now_Status, Enum_DR16_Switch_Status Pre_Status);
};

class Class_FSM_Off_Ledder : public Class_FSM_Ledder
{
public:
    void Reload_TIM_Status_PeriodElapsedCallback();
protected:
    // ToF测距模块回传的底盘离地高度
    float Height_Meter;
    // 抬升触地时的底盘阈值高度
    float Trigger_Height;
    // 底盘高度判断误差量，数值越小判断越严格，不可超过台阶高度的1/2
    float Trigger_Offset;

    // 下到第一个台阶时，抬升机构全部接触台阶的高度
    float ledder_prepare[4];
    // 底盘前端/后端腾空时，对应测抬升机构触地的高度
    float ledder_touch[4];
    // 底盘完全腾空后抬升机构收起
    float ledder_finish[4];
};

class Class_Mecanum_Chassis
{
public:
    // 斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    // 斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    // 斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    Class_Supercap Supercap;

    // 功率限制
    Class_Power_Limit Power_Limit;

    // 裁判系统
    Class_Referee *Referee;

    // 下方转动电机，顺时针方向编号
    Class_DJI_Motor_C620 Mecanum_Wheels[4];
    // 主动轮电机 - 2325 速度环不需要校准
    Class_DM_Motor_J4310 Track_Motor[2];
    // 抬升机构电机
    Class_DJI_Motor_C620 Uplift_Motor[4];

    // 激光测距模块
    Class_TOFSense TOFSense;

    // 抬升机构校准状态机
    Class_FSM_Calibration_Chassis Calibration_FSM;
    friend class Class_FSM_Calibration_Chassis;

    // 上台阶状态机
    Class_FSM_Ledder Ledder_FSM;
    friend class Class_FSM_Ledder;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Delta_Radian();
    inline float Get_Target_Track_Omega();
    inline float Get_Track_Request_Power();
    inline float Get_Track_Allocated_Power();
    inline float Get_Track_Power_Scale();
    inline float Get_Slope_Track_Power_Limit();
    inline float Get_Target_Uplift_Radian(uint8_t index);

    inline Enum_Supercap_Mode Get_Supercap_Mode();
    inline Enum_Chassis_DR16_Right_Uplift_Status Get_DR16_Right_Uplift_Status();
    inline Enum_Lift_Direction Get_Lift_Direction(uint8_t index);
    inline bool Get_Lift_Select(uint8_t index);
    inline Enum_Uplift_FSM_Direction Get_Uplift_FSM_Direction();
    inline bool Get_Backdoor_Jump();
    inline bool Get_Downlift_Init();
    inline Enum_Wheel_Slave_Status Get_Wheel_Slave_Status();
    inline bool Get_Lift_Calibrate_Status();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Delta_Radian(float __Delta_Radian);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);
    inline void Set_Target_Track_Omega(float __Target_Track_Omega);
    inline void Set_Track_Allocated_Power(float __Track_Allocated_Power);
    inline void Set_Track_Power_Scale(float __Track_Power_Scale);
    inline void Set_Slope_Track_Power_Limit(float __Slope_Track_Power_Limit);
    inline void Set_Target_Uplift_Radian(uint8_t index, float __Target_Uplift_Radian);
    inline void Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode);
    inline void Set_DR16_Right_Uplift_Status(Enum_Chassis_DR16_Right_Uplift_Status __DR16_Right_Uplift_Status);
    inline void Set_Lift_Calibrate_Status(bool flag);
    inline void Set_Lift_Direction(uint8_t index, Enum_Lift_Direction direction);
    inline void Set_Lift_Select(bool select_1, bool select_2, bool select_3, bool select_4);
    inline void Set_Wheel_Slave_Status(Enum_Wheel_Slave_Status __Wheel_Slave_Status);
    inline void Set_Uplift_FSM_Direction(Enum_Uplift_FSM_Direction __Uplift_FSM_Direction);
    inline void Set_Backdoor_Jump(bool __Backdoor_Jump);
    inline void Set_Downlift_Init(bool __Downlift_Init);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    void Request_Lift_Recalibration();
    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);
    float Calculate_Track_Request_Power();
    void Apply_Track_Power_Limit();

    // 着地时的角度，相对于Min_Radian
    float Downlift_Touch_Radian[4] = {9.0f, 9.0f, 19.5f, 19.5f};

protected:
    // 初始化相关常量

    // 速度X限制，m/s
    float Velocity_X_Max;
    // 速度Y限制
    float Velocity_Y_Max;
    // 角速度限制
    float Omega_Max;
    // 底盘目标角度增量(从DR16接收)
    float Delta_Radian;
    // 底盘电机最大转速
    float Wheels_Omega_Max = 31.416f; // 300rpm
    // 抬升机构最大高度 mm
    float Uplift_Height_Max = 280;
    // 抬升电机校准后的最大角度，以电机返回的角度作为抬升高度的上限
    float Uplift_Max_Radian[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 抬升电机基于校准后最大角度而言的最低角度，差值为26.5rad
    float Uplift_Min_Radian[4] = {
        Uplift_Max_Radian[0] - 28.5f,
        Uplift_Max_Radian[1] - 28.5f,
        Uplift_Max_Radian[2] - 21.15f,
        Uplift_Max_Radian[3] - 21.15f,};

    // 常量

    // 电机理论上最大输出，发给电机的电流值
    float Wheel_Max_Output = 16384.0f;

    // 麦轮轮组电机目标线速度与角速度
    float Target_Motor_Velocity[4];
    float Target_Motor_Omega[4];
    // 履带电机目标角速度，rad/s，两边保持一致
    float Target_Track_Omega;
    float Track_Request_Power = 0.0f;
    float Track_Allocated_Power = 0.0f;
    float Track_Power_Scale = 1.0f;
    float Slope_Track_Power_Limit = 60.0f;
    // 传给抬升电机的实际目标角度
    float Target_Uplift_Motor_Radian[4];
    // 用于和Uplift_Min_Radian相加得到电机目标角度的值，也就是加offset之前的目标角度，主要用于遥控器逻辑，初始在最高点，设为最小值和最大值之间的差值，此时赋给电机的角度应为0.0f
    float Target_Uplift_Radian[4] = {28.5f, 28.5f, 21.15f, 21.15f};

    // 读变量

    // 当前总功率
    float Now_Power = 0.0f;
    // 当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    // 可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    // 写变量

    // 读写变量

    // 底盘控制方式
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Supercap_Mode Supercap_Mode = Supercap_DISABLE;
    Enum_Chassis_DR16_Right_Uplift_Status DR16_Right_Uplift_Status = DR16_Right_Switch_MIDDLE;
    
    // 抬升状态机枚举
    Enum_Uplift_FSM_Direction Uplift_FSM_Direction = Uplift_FSM_HOLD;
    // 小轮子从动状态枚举
    Enum_Wheel_Slave_Status Wheel_Slave_Status = Wheel_Slave_OFF;
    // 抬升控制相关
    bool Lift_Calibrate = false;
    // 抬升选择
    bool Lift_Select[4] = {false};
    // 抬升方向
    Enum_Lift_Direction Lift_Direction[4] = {Lift_Direction_HOLD};
    // 下台阶初始位置
    bool Downlift_Init = false;
    // 150mm后门跳转
    bool Backdoor_Jump = false;

    // 目标速度X，m/s
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标车体角速度
    float Target_Omega = 0.0f;
    // 目标转向速度（与X，Y速度一样应为m/s）：(L*tan a + l) * Omega
    float Target_Velocity_Steer = Target_Omega * (half_length + half_width);
    // 抬升机构目标高度, m
    float Target_Uplift_Height[4];

    // 当前速度X
    float Now_Velocity_X = 0.0f;
    // 当前速度Y
    float Now_Velocity_Y = 0.0f;
    // 当前角速度
    float Now_Omega = 0.0f;

    // 内部函数
    void Speed_Resolution();
    void Output();
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_Mecanum_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_Mecanum_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_Mecanum_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_Mecanum_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_Mecanum_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_Mecanum_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_Mecanum_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

float Class_Mecanum_Chassis::Get_Delta_Radian()
{
    return (Delta_Radian);
}

/**
 * @brief 获取目标履带角速度
 *
 * @return float 目标履带角速度
 */
float Class_Mecanum_Chassis::Get_Target_Track_Omega()
{
    return (Target_Track_Omega);
}

float Class_Mecanum_Chassis::Get_Track_Request_Power()
{
    return (Track_Request_Power);
}

float Class_Mecanum_Chassis::Get_Track_Allocated_Power()
{
    return (Track_Allocated_Power);
}

float Class_Mecanum_Chassis::Get_Track_Power_Scale()
{
    return (Track_Power_Scale);
}

float Class_Mecanum_Chassis::Get_Slope_Track_Power_Limit()
{
    return (Slope_Track_Power_Limit);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_Mecanum_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_Mecanum_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_Mecanum_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 获取目标抬升角度
 * @param index 目标抬升机构的索引，从0开始
 * @return float 目标抬升角度，为
 */
float Class_Mecanum_Chassis::Get_Target_Uplift_Radian(uint8_t index)
{
    if (index >= 4)
        return 22.5f;

    return (Target_Uplift_Radian[index]);
}

Enum_Supercap_Mode Class_Mecanum_Chassis::Get_Supercap_Mode()
{
    return (Supercap_Mode);
}

Enum_Chassis_DR16_Right_Uplift_Status Class_Mecanum_Chassis::Get_DR16_Right_Uplift_Status()
{
    return (DR16_Right_Uplift_Status);
}

/**
 * @brief 获取指定抬升电机的控制方向
 */
inline Enum_Lift_Direction Class_Mecanum_Chassis::Get_Lift_Direction(uint8_t index)
{
    if (index >= 4) return Lift_Direction_HOLD;
    return Lift_Direction[index];
}

/**
 * @brief 获取指定抬升机构是否被选中
 */
inline bool Class_Mecanum_Chassis::Get_Lift_Select(uint8_t index)
{
    if (index >= 4) return false;
    return Lift_Select[index];
}

/**
 * @brief 获取抬升状态机运行方向
 */
inline Enum_Uplift_FSM_Direction Class_Mecanum_Chassis::Get_Uplift_FSM_Direction()
{
    return Uplift_FSM_Direction;
}

/**
 * @brief 获取后门跳转标志位
 */
inline bool Class_Mecanum_Chassis::Get_Backdoor_Jump()
{
    return Backdoor_Jump;
}

/**
 * @brief 获取下台阶初始化状态
 */
inline bool Class_Mecanum_Chassis::Get_Downlift_Init()
{
    return Downlift_Init;
}

/**
 * @brief 获取小轮子从动状态
 */
inline Enum_Wheel_Slave_Status Class_Mecanum_Chassis::Get_Wheel_Slave_Status()
{
    return Wheel_Slave_Status;
}

inline bool Class_Mecanum_Chassis::Get_Lift_Calibrate_Status()
{
    return Lift_Calibrate;
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_Mecanum_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_Mecanum_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_Mecanum_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_Mecanum_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

void Class_Mecanum_Chassis::Set_Delta_Radian(float __Delta_Radian)
{
    Delta_Radian = __Delta_Radian;
}

/**
 * @brief 设定目标履带角速度
 *
 * @param __Target_Track_Omega 目标履带角速度
 */
void Class_Mecanum_Chassis::Set_Target_Track_Omega(float __Target_Track_Omega)
{
    Target_Track_Omega = __Target_Track_Omega;
}

void Class_Mecanum_Chassis::Set_Track_Allocated_Power(float __Track_Allocated_Power)
{
    Track_Allocated_Power = __Track_Allocated_Power;
}

void Class_Mecanum_Chassis::Set_Track_Power_Scale(float __Track_Power_Scale)
{
    Track_Power_Scale = __Track_Power_Scale;
}

inline void Class_Mecanum_Chassis::Set_Lift_Calibrate_Status(bool flag)
{
    Lift_Calibrate = flag;
}

void Class_Mecanum_Chassis::Set_Slope_Track_Power_Limit(float __Slope_Track_Power_Limit)
{
    Slope_Track_Power_Limit = __Slope_Track_Power_Limit;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_Mecanum_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_Mecanum_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_Mecanum_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_Mecanum_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_Mecanum_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

/**
 * @brief 设定抬升机构目标角度，rad，会自动加上offset赋给用于发给电机的目标角度
 *
 * @param index 目标抬升机构的索引，从0开始
 * @param __Target_Radian 目标抬升角度，rad
 */
void Class_Mecanum_Chassis::Set_Target_Uplift_Radian(uint8_t index, float __Target_Radian)
{
    if (index >= 4)
        return;

    Target_Uplift_Radian[index] = __Target_Radian;
    Math_Constrain(Target_Uplift_Radian + index, 0.0f, Uplift_Max_Radian[index] - Uplift_Min_Radian[index]);

    Target_Uplift_Motor_Radian[index] = Uplift_Min_Radian[index] + Target_Uplift_Radian[index];
    Math_Constrain(Target_Uplift_Motor_Radian + index, Uplift_Min_Radian[index], Uplift_Max_Radian[index]);
}

void Class_Mecanum_Chassis::Set_Supercap_Mode(Enum_Supercap_Mode __Supercap_Mode)
{
    Supercap_Mode = __Supercap_Mode;
}

void Class_Mecanum_Chassis::Set_DR16_Right_Uplift_Status(Enum_Chassis_DR16_Right_Uplift_Status __DR16_Right_Uplift_Status)
{
    DR16_Right_Uplift_Status = __DR16_Right_Uplift_Status;
}


/**
 * @brief 设定指定抬升机构的运动方向
 */
inline void Class_Mecanum_Chassis::Set_Lift_Direction(uint8_t index, Enum_Lift_Direction direction)
{
    if (index >= 4) return;
    Lift_Direction[index] = direction;
}

/**
 * @brief 同时设定四个抬升机构的选中状态
 */
inline void Class_Mecanum_Chassis::Set_Lift_Select(bool select_1, bool select_2, bool select_3, bool select_4)
{
    Lift_Select[0] = select_1;
    Lift_Select[1] = select_2;
    Lift_Select[2] = select_3;
    Lift_Select[3] = select_4;
}

/**
 * @brief 设定小轮子（从动轮）状态
 */
inline void Class_Mecanum_Chassis::Set_Wheel_Slave_Status(Enum_Wheel_Slave_Status __Wheel_Slave_Status)
{
    Wheel_Slave_Status = __Wheel_Slave_Status;
}

/**
 * @brief 设定抬升状态机方向
 */
inline void Class_Mecanum_Chassis::Set_Uplift_FSM_Direction(Enum_Uplift_FSM_Direction __Uplift_FSM_Direction)
{
    Uplift_FSM_Direction = __Uplift_FSM_Direction;
}

/**
 * @brief 设定后门跳转标志
 */
inline void Class_Mecanum_Chassis::Set_Backdoor_Jump(bool __Backdoor_Jump)
{
    Backdoor_Jump = __Backdoor_Jump;
}

/**
 * @brief 设定下台阶初始化标志
 */
inline void Class_Mecanum_Chassis::Set_Downlift_Init(bool __Downlift_Init)
{
    Downlift_Init = __Downlift_Init;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
