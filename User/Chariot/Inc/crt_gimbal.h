/**
 * @file crt_gimbal.h
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

#define GEAR_RATIO 2 // roll轴减速比18:36

#ifndef PI
#define PI 3.1415926535f
#endif

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"
#include "dvc_dmmotor.h"
#include "alg_fsm.h"
#include "arm_model.h"
#include "dvc_dwt.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type : uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

enum Enum_Motor_DM_J0_Yaw_Type : uint8_t
{
    Yaw_A = 0,
    Yaw_B,
};

struct IMU_Data
{
    float Pitch;
    float Roll;
    float Yaw;
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief Specialized, yaw轴电机类
 *
 */
class Class_Gimbal_Yaw_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Yaw_Motor_GM6020::Get_Trer_Rad_Yaw()
{
    return (True_Rad_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Gyro_Yaw()
{
    return (True_Gyro_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Angle_Yaw()
{
    return (True_Angle_Yaw);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_LK6010 : public Class_LK_Motor
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}

class Class_Gimbal;
class Class_FSM_Calibration : public Class_FSM
{
public:
    Class_Gimbal *Gimbal;
    void Reload_TIM_Status_PeriodElapsedCallback();

    /*电机校准执行函数*/
    bool Motor_Calibration(Class_DM_Motor_J4310 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);
    bool Motor_Calibration(Class_DJI_Motor_C610 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);
    bool Motor_Calibration(Class_DJI_Motor_C620_Uplift *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);
    
    inline bool Get_roll_cali_status();
    inline bool Get_Gripper_cali_status();
    inline bool Get_Uplift_cali_status(uint8_t index);

protected:
    /*roll轴校准相关变量*/
    float Cali_Offset = 0.0f;      // 存储校准后的偏差, rad
    float locked_torque = 4.5f;    // 堵转力矩
    bool roll_cali_status = false; // 校准状态，初始为false

    float gripper_offset = 0.0f;    //夹爪校准后的偏差角度，rad
    float gripper_locked_torque = 1600.0f; //暂定为8.0f，待测
    bool gripper_cali_status = false;

    float uplift_offset[2] = {0.0f};
    float uplift_cali_torque;       //待测
    bool uplift_cali_status[2] = {false};

    uint16_t locked_cnt = 0;       // 堵转时间计数
    uint16_t gripper_locked_cnt = 0;// 夹爪堵转时间计数
    uint16_t uplift_locked_cnt[2] = {0};// 抬升堵转时间计数
};

/**
 * @brief Specialized, 云台类，在工程机器人上直接把机械臂当做云台
 *
 */
class Class_Gimbal
{
public:
    // imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    Class_DM_Motor_J4310 Motor_DM_J0_Yaw;     // J0 - DM4310
    Class_DM_Motor_J4310 Motor_DM_J1_Pitch;   // J1 - DM8009P
    Class_DM_Motor_J4310 Motor_DM_J2_Pitch_2; // J2 - DM4340
    Class_DM_Motor_J4310 Motor_DM_J3_Roll;    // J3 - DM2325
    Class_DM_Motor_J4310 Motor_DM_J4_Pitch_3; // J4 - DM4340
    Class_DJI_Motor_GM6020 Motor_6020_J5_Roll_2;//J5 - DJI-G6020
    Class_DJI_Motor_C610 Motor_C610_Gripper;    //夹爪 - C610

    Class_FSM_Calibration Calibration_FSM;      //校准状态机类
    friend class Class_FSM_Calibration;

    /*机械臂DH建模*/
    Class_Trajectory_Tracer Trajectory_Tracer;
    /*机械臂初始化标志位*/
    bool arm_init = false;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Yaw_Radian();

    inline float Get_Target_Pitch_Angle();
    inline float Get_Target_Pitch_Radian();

    inline float Get_Target_Pitch_2_Angle();
    inline float Get_Target_Pitch_2_Radian();

    inline float Get_Target_Pitch_3_Angle();
    inline float Get_Target_Pitch_3_Radian();

    inline float Get_Target_Roll_Angle();
    inline float Get_Target_Roll_Radian();
    inline float Get_Roll_Min_Radian();
    inline float Get_Roll_Cali_Offset();

    inline float Get_Target_Roll_2_Angle();
    inline float Get_Target_Roll_2_Radian();
    inline float Get_Target_Roll_2_Radian_Single();

    inline float Get_Target_Gripper_Angle();
    inline float Get_Target_Gripper_Radian();
    inline float Get_Gripper_Min_Radian();

    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);

    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Yaw_Radian(float __Target_Yaw_Radian);

    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);
    inline void Set_Target_Pitch_Radian(float __Target_Pitch_Radian);

    inline void Set_Target_Pitch_2_Angle(float __Target_Pitch_2_Angle);
    inline void Set_Target_Pitch_2_Radian(float __Target_Pitch_2_Radian);

    inline void Set_Target_Pitch_3_Angle(float __Target_Pitch_3_Angle);
    inline void Set_Target_Pitch_3_Radian(float __Target_Pitch_3_Radian);

    inline void Set_Target_Roll_Angle(float __Target_Roll_Angle);
    inline void Set_Target_Roll_Radian(float __Target_Roll_Radian);

    inline void Set_Target_Roll_2_Angle(float __Target_Roll_2_Angle);
    inline void Set_Target_Roll_2_Radian(float __Target_Roll_2_Radian);
    inline void Set_Target_Roll_2_Radian_Single(float Target_Roll_2_Radian_Single);

    inline void Set_Target_Gripper_Angle(float __Target_Gripper_Angle);
    inline void Set_Target_Gripper_Radian(float __Target_Gripper_Radian);

    // 由于减速比和多圈的问题存在，所以要写一个专门用于输出轴的角度设定函数 //不用了，这个函数纯属没用
    inline void Set_Target_Roll_Output_Angle(float Target_Angle);

    void TIM_Calculate_PeriodElapsedCallback();

    #ifdef MY_DEBUG
    /*dh建模和解算相关变量，后期移到arm_model类中*/
    float target_pos[3] = {5.2006f, 2.9150f, 77.7629f};      //目标xyz
    float target_rpy[3] = {1.5704f, 0.6867f, 0.1338f};      //目标欧拉角
    Matrixf<6, 1> solutions[8];     //解析法求解的8组解
    uint8_t valid_IK_cnt = 0;       //合法的逆解个数
    bool valid_solution[8] = {0};   //逆解合法性数组
    uint8_t solution_index = 0;      //当前采用的解的索引
    float control_result[6] = {0};  //转换得到的用于电机控制的角度，调用Set_Target_Radian实现
    float model_result[6] = {0};    //正解算结果，用于和目标位置对比
    float pos_move_result[3] = {0.0f};  //平移后的目标xyz
    uint8_t axis = 0;               //平移的轴选择，0-x,1-y,2-z 
    float s = 0.0f;                 //平移距离，单位mm

    uint8_t move_test_flag = 0; //用于测试平移功能的标志位
    float q_solution[600][6];        // 逆解结果数组，测试用
    float move_start_q[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.5f, 0.0f};   //平移测试起始角度，模型角度
    float move_init_control_angle[6]; //用于测试平移功能的初始角度，通过将模型角度转换得到，是用来发给电机的角度
    float move_control_angle[6];      //用于测试平移功能的目标角度，通过将模型角度转换得到，是用来发给电机的角度
    uint32_t valid_solution_cnt = 0; // 有效解的数量
    bool is_low_speed[6] = {false};
    float now_pos[3];
    #endif

#ifdef MOTOR_TEST

    float debug_j0_target_angle = 0.0f; // J0目标角度（角度）
    float debug_j0_target_radian = debug_j0_target_angle * 3.14 / 180;
    float debug_j0_target_omega = 1.0f; // J0目标速度（rad/s）

    float debug_j1_target_angle = 0.0f; // J1目标角度（角度）
    float debug_j1_target_radian = debug_j1_target_angle * 3.14 / 180;
    float debug_j1_target_omega = 1.0f; // J1目标速度（rad/s）

    float debug_j2_target_angle = 0.0f; // J2目标角度（角度）
    float debug_j2_target_radian = debug_j2_target_angle * 3.14 / 180;
    float debug_j2_target_omega = 1.0f; // J2目标速度（rad/s）

    float debug_j4_target_angle = 0.0f; // J4目标角度（角度）
    float debug_j4_target_radian = debug_j4_target_angle * 3.14 / 180;
    float debug_j4_target_omega = 1.0f; // J4目标速度（rad/s）

    /*6020直接发角度*/
    float debug_j5_target_angle = 0.0f;
    float debug_j5_target_omega = 0.0f;     //角度制

    float debug_gripper_target_omega = 0.0f;
    float debug_gripper_target_angle = 0.0f;

    float debug_kp = 1.8f;
    float debug_kd = 1.0f; // 空载下设置为0.2f，否则可能导致电机持续加速
    float debug_roll_target_torque = 0.0f;
    float debug_roll_target_omega = 1.5f;
    float debug_roll_target_radian = 0.0f; // roll目标位置，弧度制，用于在校准后角度的基础上进行增量，顺时针方向为正，电机校准的方向是逆时针，所以需要加角度
#endif

protected:
    // 电机CAN通信优先级变量
    static inline uint32_t can_priority_cnt = 0;     // 电机CAN通信优先级计数器，前面写inline是为了能保持变量是类内部静态变量的同时可以自动初始化

    // 初始化相关常量
    float Gimbal_Head_Angle;
    // 常量
    float CRUISE_SPEED_YAW = 100.f;
    float CRUISE_SPEED_PITCH = 70.f;

    //Yaw轴，同步带减速比为2
    // yaw轴最小值，deg
    float Min_Yaw_Angle = -360.0f;
    // yaw轴最大值，deg
    float Max_Yaw_Angle = 360.0f;
    // rad制
    float Min_Yaw_Radian = -2 * PI;
    float Max_Yaw_Radian = 2 * PI;

    // pitch轴最小值
    float Min_Pitch_Angle = 0.0f; // 角度，非弧度
    // pitch轴最大值
    float Max_Pitch_Angle = 109.0f; // 角度，非弧度
    // rad
    float Min_Pitch_Radian = 0.0f;
    float Max_Pitch_Radian = 1.90f;

    //pitch2，同步带减速比45:30
    // pitch2轴最小值与最大值，degree
    float Min_Pitch_2_Angle = 0.0f;
    float Max_Pitch_2_Angle = 110.0f;
    // rad
    float Min_Pitch_2_Radian = 0.0f;
    float Max_Pitch_2_Radian = 2.87f;

    // pitch3轴最小值与最大值，degree
    float Min_Pitch_3_Angle = -146.75f;
    float Max_Pitch_3_Angle = 0.0f;
    // rad
    float Min_Pitch_3_Radian = -2.561f;
    float Max_Pitch_3_Radian = 0.0f;

    //roll 电机减速比25:1，同步带减速比2:1，总减速比50:1
    // roll校准角度，默认为0
    float roll_cali_offset = 0.0f;
    // roll轴最小值与最大值
    float Min_Roll_Radian = roll_cali_offset * 50.0f;
    float Max_Roll_Radian = Min_Roll_Radian + 300.0f;

    //gripper校准角度，默认为0
    float gripper_cali_offset = 0.0f;
    float Min_gripper_Radian = gripper_cali_offset;
    float Max_gripper_Radian = gripper_cali_offset + 0.95f; //最大角度，完全闭合时为0.95f

    // 内部变量

    // 读变量

    // 写变量

    // 云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    // 读写变量

    // yaw轴角度 degree & yaw轴角速度 rad/s
    float Target_Yaw_Angle = 0.0f;
    float Target_Yaw_Radian = 0.0f;
    float Target_Yaw_Omega = 0.5f * PI; //作为和基座相连接的yaw，你还是动得慢一点比较好
    // pitch轴角度 degree
    float Target_Pitch_Angle = 0.0f;
    float Target_Pitch_Radian = 0.0f;
    float Target_Pitch_Omega = 0.5f * PI;

    // pitch_2角度 degree
    float Target_Pitch_2_Angle = 0.0f;
    float Target_Pitch_2_Radian = 0.0f;
    float Target_Pitch_2_Omega = 0.5f * PI;

    // pitch_3角度 degree
    float Target_Pitch_3_Angle = 0.0f;
    float Target_Pitch_3_Radian = 0.0f;
    float Target_Pitch_3_Omega = 0.5f * PI;

    // roll角度 degree & radian
    float Target_Roll_Angle = 0.0f;
    float Target_Roll_Radian = 0.0f;
    float Target_Roll_Omega = 0.5f * PI;

    //roll_2角度 degree
    float Target_Roll_2_Angle = 0.0f;
    float Target_Roll_2_Radian = 0.0f;
    float Target_Roll_2_Omega = 1.5f * PI;

    //夹爪角度，degree
    float Target_Gripper_Angle = 0.0f;
    float Target_Gripper_Radian = 0.0f;
    float Target_Gripper_Omega = 0.75f;

    //建模解算测试用
    float model_angle[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.5f, 0.0f};
    float model_degree[6];
    float control_angle[6] = {0};
    float xyz[3] = {0};      //正解算结果
    float rpy[3] = {0};

    #ifdef MY_DEBUG
    float debug_radian[6] = {0.0f};    //测试电机控制角度映射专用
    #endif
    float delta_time = 0.0f; // 用DWT测得的时间间隔，用这个看解析解的计算速度

    // 内部函数
    void Output();
    
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}
float Class_Gimbal::Get_Target_Yaw_Radian()
{
    return (Target_Yaw_Radian);
}

/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}
float Class_Gimbal::Get_Target_Pitch_Radian()
{
    return (Target_Pitch_Radian);
}

/**
 * @brief 获取pitch_2轴角度
 *
 * @return float pitch_2轴角度
 */
float Class_Gimbal::Get_Target_Pitch_2_Angle()
{
    return (Target_Pitch_2_Angle);
}
float Class_Gimbal::Get_Target_Pitch_2_Radian()
{
    return (Target_Pitch_2_Radian);
}

/**
 * @brief 获取pitch_3轴角度
 *
 * @return float pitch_3轴角度
 */
float Class_Gimbal::Get_Target_Pitch_3_Angle()
{
    return (Target_Pitch_3_Angle);
}
float Class_Gimbal::Get_Target_Pitch_3_Radian()
{
    return (Target_Pitch_3_Radian);
}

/**
 * @brief 获取roll轴角度
 *
 * @return float roll轴角度
 */
float Class_Gimbal::Get_Target_Roll_Angle()
{
    return (Target_Roll_Angle);
}
float Class_Gimbal::Get_Target_Roll_Radian()
{
    return (Target_Roll_Radian);
}

/**
 * @brief 获取roll轴Min_Radian，可以用于在Roll轴校准后的零点上作增量计算
 *
 * @return float roll轴校准后的零点
 */
float Class_Gimbal::Get_Roll_Min_Radian()
{
    return (Min_Roll_Radian);
}

float Class_Gimbal::Get_Roll_Cali_Offset()
{
    return (roll_cali_offset);
}

/**
 * @brief 获取roll_2轴角度
 *
 * @return float roll_2轴角度
 */
float Class_Gimbal::Get_Target_Roll_2_Angle()
{
    return (Target_Roll_2_Angle);
}
float Class_Gimbal::Get_Target_Roll_2_Radian()
{
    return (Target_Roll_2_Radian);
}
float Class_Gimbal::Get_Target_Roll_2_Radian_Single()
{
    float single_radian = fmod(Target_Roll_2_Radian, 2.0f * PI);
    if(single_radian < 0.0f)
    {
        single_radian += 2.0f * PI;
    }
    return single_radian;
}

/**
 * @brief 获取Gripper角度
 *
 * @return float 夹爪角度
 */
float Class_Gimbal::Get_Target_Gripper_Angle()
{
    return (Target_Gripper_Angle);
}
float Class_Gimbal::Get_Target_Gripper_Radian()
{
    return (Target_Gripper_Radian);
}

/**
 * @brief 获取夹爪的Min_Radian，可以用于在夹爪校准后的零点上作增量计算（比较规范的写法和用法）
 *
 * @return float 夹爪校准后的零点
 */
float Class_Gimbal::Get_Gripper_Min_Radian()
{
    return (Min_gripper_Radian);
}

/**
 * @brief 获取云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 获取云台控制类型
 */
Enum_Gimbal_Control_Type Class_Gimbal::Get_Gimbal_Control_Type()
{
    return (Gimbal_Control_Type);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}

/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
    Set_Target_Yaw_Radian(Target_Yaw_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Yaw_Radian(float __Target_Yaw_Radian)
{
    Target_Yaw_Radian = __Target_Yaw_Radian;
    Math_Constrain(&Target_Yaw_Radian, Min_Yaw_Radian, Max_Yaw_Radian);
}

/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
    Set_Target_Pitch_Radian(Target_Pitch_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_Radian(float __Target_Pitch_Radian)
{
    Target_Pitch_Radian = __Target_Pitch_Radian;
    Math_Constrain(&Target_Pitch_Radian, Min_Pitch_Radian, Max_Pitch_Radian);
}

/**
 * @brief 设定pitch_2轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_2_Angle(float __Target_Pitch_2_Angle)
{
    Target_Pitch_2_Angle = __Target_Pitch_2_Angle;
    Set_Target_Pitch_2_Radian(Target_Pitch_2_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_2_Radian(float __Target_Pitch_2_Radian)
{
    Target_Pitch_2_Radian = __Target_Pitch_2_Radian;
    Math_Constrain(&Target_Pitch_2_Radian, Min_Pitch_2_Radian, Max_Pitch_2_Radian);
}

/**
 * @brief 设定pitch_3轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_3_Angle(float __Target_Pitch_3_Angle)
{
    Target_Pitch_3_Angle = __Target_Pitch_3_Angle;
    Set_Target_Pitch_3_Radian(Target_Pitch_3_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Pitch_3_Radian(float __Target_Pitch_3_Radian)
{
    Target_Pitch_3_Radian = __Target_Pitch_3_Radian;
    Math_Constrain(&Target_Pitch_3_Radian, Min_Pitch_3_Radian, Max_Pitch_3_Radian);
}

/**
 * @brief 设定roll轴角度
 *
 */
void Class_Gimbal::Set_Target_Roll_Angle(float __Target_Roll_Angle)
//设置Degree制的**关节**角度，希望关节转动45°，就直接赋45.0f
{
    Target_Roll_Angle = __Target_Roll_Angle;
    // 关节角度(deg) -> 关节角度(rad) -> 电机角度(rad) -> 调用Set_Target_Roll_Radian自动加上Offset和限位
    float joint_rad = Target_Roll_Angle * PI / 180.0f;
    float motor_rad = joint_rad * 100.0f;
    Set_Target_Roll_Radian(motor_rad);
}
void Class_Gimbal::Set_Target_Roll_Radian(float __Target_Roll_Radian)
//设置2325的目标角度，使用Target_Radian赋值给电机，控制的是**转子端**的角度，这里传入函数的Target_Roll_Radian不用手动加偏移，在函数里会自己加
{
    Target_Roll_Radian = __Target_Roll_Radian + Min_Roll_Radian;
    Math_Constrain(&Target_Roll_Radian, Min_Roll_Radian, Max_Roll_Radian);
}

/**
 * @brief 设定roll_2总角度
 *
 */
void Class_Gimbal::Set_Target_Roll_2_Angle(float __Target_Roll_2_Angle)
{
    Target_Roll_2_Angle = __Target_Roll_2_Angle;
    Set_Target_Roll_2_Radian(Target_Roll_2_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Roll_2_Radian(float __Target_Roll_2_Radian)
{
    Target_Roll_2_Radian = __Target_Roll_2_Radian;
}
/**
 * @brief 设定roll_2单圈角度，会自动计算出正转还是倒转，传入的角度必须要是0.0f - 2PI单圈值
 *
 */
void Class_Gimbal::Set_Target_Roll_2_Radian_Single(float Target_Roll_2_Radian_Single)
{
    //电机对象传回的总角度，可能为负数
    float current_total_radian = Motor_6020_J5_Roll_2.Get_Now_Radian();
    //先对2PI取模
    float current_single_radian = fmod(current_total_radian, 2.0f * PI);

    if(current_single_radian < 0.0f)
    //如果是负数的话转成正数，这样就转成了单圈角度
    {
        current_single_radian += 2.0f * PI;
    }

    //偏差值，加到总圈数上进行设置
    float delta = Target_Roll_2_Radian_Single - current_single_radian;

    if(delta > PI)
    {
        delta -= 2.0f * PI;
    }
    else if(delta < -PI)
    {
        delta += 2.0f * PI;
    }

    float __Target_Roll_2_Radian = current_total_radian + delta;

    Set_Target_Roll_2_Radian(__Target_Roll_2_Radian);
}

bool Class_FSM_Calibration::Get_roll_cali_status()
{
    return roll_cali_status;
}

bool Class_FSM_Calibration::Get_Gripper_cali_status()
{
    return gripper_cali_status;
}

/**
 * @brief 设定Gripper角度
 *
 */
void Class_Gimbal::Set_Target_Gripper_Angle(float __Target_Gripper_Angle)
{
    Target_Gripper_Angle = __Target_Gripper_Angle;
    Math_Constrain(&Target_Gripper_Angle, 0.0f, 54.45f);
    Set_Target_Gripper_Radian(Target_Gripper_Angle * PI / 180.0f);
}
void Class_Gimbal::Set_Target_Gripper_Radian(float __Target_Gripper_Radian)
{
    Target_Gripper_Radian = gripper_cali_offset + __Target_Gripper_Radian;
    Math_Constrain(&Target_Gripper_Radian, Min_gripper_Radian, Max_gripper_Radian);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
