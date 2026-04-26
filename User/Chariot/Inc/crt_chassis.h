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
#include "alg_new_power_limit.h"
#include "dvc_supercap.h"
#include "config.h"
#include "dvc_minipc.h"
#include "drv_math.h"
#include "alg_fsm.h"
#include "dvc_dwt.h"
#include <cmath>
#include <algorithm>
/* Exported macros -----------------------------------------------------------*/
#ifdef AGV
#define MAX_MOTOR_SPEED 796.06f    //当速度为500rpm时，对应每个舵轮速度的最大值为3.14m/s左右 当速度为636.62rpm时，对应每个舵轮速度的最大值为4.09m/s左右 当速度为796.06rpm时，对应每个舵轮速度的最大值为5m/s左右
#define wheel_diameter 0.12f   // m
#define half_width   0.203f   //m     		//两组相邻舵轮中心间距的一半
#define half_length 0.203f     //m
#define ROTATION_CENTER_OFFSET 0.0f // 旋转中心位置偏移量，现在只有y方向上的偏移，且向y负方向偏移，这个偏移量为绝对值

#define THETA_A atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_B atan((half_length + ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_C atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）
#define THETA_D atan((half_length - ROTATION_CENTER_OFFSET) / half_width) // 转向轮在坐标系下与y轴的夹角（锐角）

#define THETA (PI/4.0f)

#define R_A half_width / cos(THETA) // 旋转中心与A舵轮的距离
#define R_B half_width / cos(THETA) // 旋转中心与B舵轮的距离
#define R_C half_width / cos(THETA) // 旋转中心与C舵轮的距离
#define R_D half_width / cos(THETA) // 旋转中心与D舵轮的距离


#define PI 3.141593f
#define PI2 2 * PI
#define RPM2RAD 0.104720f                // 		1 rpm = 2pi/60 rad/s
#define RPM2VEL 0.523599f                // 		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f                // 
#define M2006_REDUCTION_RATIO 36.000000f // 
#define M3508_REDUCTION_RATIO 19.000000f // 
#define GM6020_ENCODER_ANGLE 8192.0f

#define RAD_TO_8191 8191.0f / PI / 2.0f
#define VEL2RPM 60.0f/PI/wheel_diameter // //rpm = vel*60/π/D    vel(m/s)->rpm

#define Chassis_Spin_Omega 6.0f    //rad/s
#endif

#ifdef TRACK_LEG
#define Chassis_Spin_Omega 6.0f    //rad/s

#define PI 3.141593f
#define PI2 2 * PI
#define RPM2RAD 0.104720f                // 		1 rpm = 2pi/60 rad/s
#define RPM2VEL 0.523599f                // 		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f                // 
#define M2006_REDUCTION_RATIO 36.000000f // 
#define M3508_REDUCTION_RATIO 19.000000f // 
#define GM6020_ENCODER_ANGLE 8192.0f

#define RAD_TO_8191 8191.0f / PI / 2.0f
#endif
/* Exported types ------------------------------------------------------------*/
class Class_HybridTrackLeg_Chassis;
/**
 * @brief 底盘冲刺状态枚举
 *
 */
enum Enum_Sprint_Status : uint8_t
{
    Sprint_Status_DISABLE = 0, 
    Sprint_Status_ENABLE,
};
enum Enum_Yaw_Encoder_Control_Status : uint8_t
{
    Yaw_Encoder_Control_Status_Disable = 0,
    Yaw_Encoder_Control_Status_Enable,
};

/**
 * @brief 底盘逻辑方向枚举
 * 
 */
enum Enum_Chassis_Logics_Direction : uint8_t
{
    Chassis_Logic_Direction_Positive = 0, // 履带侧
    Chassis_Logic_Direction_Negative, // 腿侧
};

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type :uint8_t
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_FLLOW,
    Chassis_Control_Type_SPIN_Positive,
    Chassis_Control_Type_SPIN_Negative,
	Chassis_Control_Type_SPIN,
};
/**
 * @brief 位姿控制类型
 * 
 */
enum Enum_Pose_Control_Type : uint8_t
{
    Pose_DISABLE = 0,
    Pose_STANDBY,
    Pose_ENABLE,
    Pose_CONTRACT,
};

enum Enum_Track_Control_Type: uint8_t
{
    Track_Off = 0,
    Track_On,
};

/**
 * @brief Specialized, 过热检测有限自动机
 *
 */
class Class_FSM_OverHeated_Detect : public Class_FSM
{
public:
    Class_HybridTrackLeg_Chassis *Chassis;

    float Heat;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

/**
 * @brief Specialized, DM8009P电机类，继承自DM4310
 * 
 */
class Class_DM_Motor_8009P : public Class_DM_Motor_J4310
{
public:
    Class_IMU *IMU;

    void Disable();
    void TIM_PID_PeriodElapsedCallback();

    float Target_Angle_Calc; // 目标角度，单位为弧度
    float K_P;
    float K_D;
};

/**
 * @brief Specialized,前履带后轮腿底盘
 * 
 */
class Class_HybridTrackLeg_Chassis
{
public:
    //debug测试用
    uint8_t Chassis_Flag = 0;
    //过温检测状态机
    Class_FSM_OverHeated_Detect FSM_OverHeated_Detect;
    friend class Class_FSM_OverHeated_Detect;
    //斜坡函数加减速速度X
    Class_Slope Slope_Velocity_X;
    //斜坡函数加减速速度Y
    Class_Slope Slope_Velocity_Y;
    //斜坡函数加减速角速度
    Class_Slope Slope_Omega;

    //IMU
    Class_IMU BoardDM_BMI;

    Class_Supercap Supercap;
      
    //功率限制
    Class_New_Power_Limit Power_Limit;
    Struct_Power_Management Power_Management;
    
    //裁判系统
    Class_Referee *Referee;

    //轮向电机
    Class_DJI_Motor_C620 Motor_Wheel[4];

    //关节电机
    Class_DM_Motor_J4310 Motor_Joint[2];
    Class_DM_Motor_8009P Motor_Leg[2];
    //斜坡函数加减速角度
    Class_Slope Slope_Position;

    //履带驱动电机
    Class_DJI_Motor_C620 Motor_Track[2];

    //底部导轮电机
    Class_DJI_Motor_C620 Motor_Guider[2];

    //随动环
    Class_PID Chassis_Follow_PID_Angle;

    void Init(float __Velocity_X_Max = 4.0f, float __Velocity_Y_Max = 4.0f, float __Omega_Max = 8.0f);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline Enum_Pose_Control_Type Get_Pose_Control_Type();
    inline Enum_Track_Control_Type Get_Track_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Now_Joint_Power();
    inline float Get_Now_Joint_Heat();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Joint_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();
    inline float Get_Spin_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Pose_Control_Type(Enum_Pose_Control_Type __Pose_Control_Type);
    inline void Set_Track_Control_Type(Enum_Track_Control_Type __Track_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

    inline void Set_Velocity_Y_Max(float __Velocity_Y_Max);
    inline void Set_Velocity_X_Max(float __Velocity_X_Max);

    float calc_AE_from_alpha(float alpha);
    float residual_AE_omega(float AE, float delta_h, float omega);
    float calc_alpha_from_omega_dh(float omega, float delta_h);
    float solve_AE_from_omega(float omega, float delta_h,float low, float high, float tol);
    float residual_alpha_AE(float alpha, float AE_target);
    float solve_alpha_from_AE(float AE_target,float low, float high, float tol);

    void TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status);

protected:
    //初始化相关常量

    //速度X限制
    float Velocity_X_Max;
    //速度Y限制
    float Velocity_Y_Max;
    //角速度限制
    float Omega_Max;
    //底盘小陀螺模式角速度
    float Spin_Omega = 8.0f;
    //常量

    //履带的角速度
    float Target_Track_Omega = 10.0f;
    //导轮的角速度
    float Target_Guider_Omega = 10.0f;
    //电机理论上最大输出
    float Wheel_Max_Output = 16384.0f;

    //内部变量

    //AE的长度
    float Leg_AE = 0.0f; 

    // 机械误差导致的α偏移量，单位为弧度，正值表示实际α比理论α大
    float alpha_offset = 0.0f; 

    // 参考角度，单位为弧度
    float Referance_Angle = 0.0f; 


    //读变量

    //车体倾斜角度
    float Chassis_Pitch = 0.0f;
    //距离车体水平的差值角度
    float Error_Pitch = 0.0f;

    //当前关节电机温度
    float Joint_Heat = 0.0f;

    //当前总功率
    float Now_Power = 0.0f;
    //当前轮向电机功率
    float Now_Wheel_Power = 0.0f;
    //可使用的轮向电机功率
    float Target_Wheel_Power = 0.0f;

    //写变量

    //读写变量


    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_FLLOW;
    Enum_Pose_Control_Type Pose_Control_Type = Pose_DISABLE;
    Enum_Track_Control_Type Track_Control_Type = Track_Off;
    //目标速度X
    float Target_Velocity_X = 0.0f;
    //目标速度Y
    float Target_Velocity_Y = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
    //当前速度X
    float Now_Velocity_X = 0.0f;
    //当前速度Y
    float Now_Velocity_Y = 0.0f;
    //当前角速度
    float Now_Omega = 0.0f;
    //缩伸腿角度
    float Set_Leg_Angle[2] = {0.0f, -0.73f};
    //缩伸腿速度
    float Set_Leg_Velocity[2] = {2.5f, -1.0f};

    //内部函数
    void Transform_Angle_To_Relative();
    void Speed_Resolution();
    void Switch_Pose();
    void Jointleg_Controller();
    void Track_Controller();
    void Guider_Controller();
    float Leg_Kinematic_Computation(float __delta_h, float __omega);
};
/* Exported variables --------------------------------------------------------*/

//三轮车底盘参数

//轮组半径
const float WHEEL_RADIUS = 0.0520f;

//轮距中心长度
const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

//前心距中心长度
const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

//前后轮距
const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

//前轮距前心
const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

//轮组方位角
const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

//轮子直径 单位m
const float WHELL_DIAMETER = 0.12f;

//底盘半宽 单位m
const float HALF_WIDTH = 0.1595f;

//底盘半长 单位m
const float HALF_LENGTH = 0.190f;

//底盘中心到每个轮子轴心投影距离
const float CHASSIS_RADIUS = sqrt(HALF_LENGTH * HALF_LENGTH + HALF_WIDTH * HALF_WIDTH);

//线速度转角速度 rad/s
const float VEL2RAD = 1.0f/(WHELL_DIAMETER/2.0f);

//杆长常量
const float L1 = 235 * 0.8f;   // DE
const float L2 = 238 * 0.8f;   // AD
const float L3 = 244 * 0.8f;  // BC
const float L4 = 108.89 * 0.8f;  // AB
const float L23 = 57 * 0.8f;  // CD
const float PARAM_L = 491.0f; // 车身长度

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
Enum_Chassis_Control_Type Class_HybridTrackLeg_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取位姿控制方法
 * 
 * @return Enum_Pose_Control_Type 位姿控制方法
 */
Enum_Pose_Control_Type Class_HybridTrackLeg_Chassis::Get_Pose_Control_Type()
{
    return (Pose_Control_Type);
}

/**
 * @brief 获取履带控制模式
 * 
 * @return Enum_Track_Control_Type 履带控制模式
 */
Enum_Track_Control_Type Class_HybridTrackLeg_Chassis::Get_Track_Control_Type()
{
    return (Track_Control_Type);
}

/**
 * @brief 获取速度X限制
 *
 * @return float 速度X限制
 */
float Class_HybridTrackLeg_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

/**
 * @brief 获取速度Y限制
 *
 * @return float 速度Y限制
 */
float Class_HybridTrackLeg_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

/**
 * @brief 获取角速度限制
 *
 * @return float 角速度限制
 */
float Class_HybridTrackLeg_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
float Class_HybridTrackLeg_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
float Class_HybridTrackLeg_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度
 */
float Class_HybridTrackLeg_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取小陀螺角速度
 *
 * @return float 小陀螺角速度
 */
float Class_HybridTrackLeg_Chassis::Get_Spin_Omega()
{
    return (Spin_Omega);
}

/**
 * @brief 获取当前电机功率
 *
 * @return float 当前电机功率
 */
float Class_HybridTrackLeg_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

/**
 * @brief 获取当前轮向电机功率
 *
 * @return float 当前轮向电机功率
 */
float Class_HybridTrackLeg_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

/**
 * @brief 获取可使用的轮向电机功率
 *
 * @return float 可使用的轮向电机功率
 */
float Class_HybridTrackLeg_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
void Class_HybridTrackLeg_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定位姿控制方法
 * 
 * @param __Pose_Control_Type 位姿控制方法
 */
void Class_HybridTrackLeg_Chassis::Set_Pose_Control_Type(Enum_Pose_Control_Type __Pose_Control_Type)
{
    Pose_Control_Type = __Pose_Control_Type;
}

/**
 * @brief 设定履带控制模式
 * 
 * @param __Track_Control_Type 
 */
void Class_HybridTrackLeg_Chassis::Set_Track_Control_Type(Enum_Track_Control_Type __Track_Control_Type)
{
    Track_Control_Type = __Track_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
void Class_HybridTrackLeg_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
void Class_HybridTrackLeg_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
void Class_HybridTrackLeg_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定当前速度X
 *
 * @param __Now_Velocity_X 当前速度X
 */
void Class_HybridTrackLeg_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

/**
 * @brief 设定当前速度Y
 *
 * @param __Now_Velocity_Y 当前速度Y
 */
void Class_HybridTrackLeg_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

/**
 * @brief 设定当前角速度
 *
 * @param __Now_Omega 当前角速度
 */
void Class_HybridTrackLeg_Chassis::Set_Now_Omega(float __Velocity_Y_Max)
{
    Now_Omega = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大Y速度
 *
 * @param __Velocity_Y_Max 输入
 */
void Class_HybridTrackLeg_Chassis::Set_Velocity_Y_Max(float __Velocity_Y_Max)
{
    Velocity_Y_Max = __Velocity_Y_Max;
}

/**
 * @brief 设定当前最大X速度
 *
 * @param __Velocity_X_Max 输入
 */
void Class_HybridTrackLeg_Chassis::Set_Velocity_X_Max(float __Velocity_X_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
}

float Class_HybridTrackLeg_Chassis::Get_Now_Joint_Heat()
{
    return (Joint_Heat);
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
