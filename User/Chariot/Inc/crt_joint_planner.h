#ifndef CRT_JOINT_PLANNER_H
#define CRT_JOINT_PLANNER_H

#include "dvc_dmmotor.h"

enum Enum_Gimbal_Joint : uint8_t
{
    Gimbal_Joint_J1_Yaw = 0,
    Gimbal_Joint_J2_Yaw,
};

enum Enum_Joint_Planner_Mode : uint8_t
{
    Joint_Planner_Mode_DISABLED = 0,
    Joint_Planner_Mode_CONSTANT_ACCEL,
    Joint_Planner_Mode_TRAPEZOID_VELOCITY,
};

class Class_Gimbal_Joint_Planner
{
public:
    void Init(Class_DM_Motor_J4310 *motor, const float *raw_target_ptr, float v_max, float a_max, float dt_s);

    inline void Set_Enable(bool enable);
    inline void Set_Mode(Enum_Joint_Planner_Mode mode);
    inline bool Get_Enable();
    inline void Reset(float now_pos);

    void Update();
    void Dispatch();

protected:
    Class_DM_Motor_J4310 *Motor = nullptr;
    const float *Raw_Target_Ptr = nullptr;

    bool Enable = false;
    Enum_Joint_Planner_Mode Mode = Joint_Planner_Mode_DISABLED;

    float V_Max = 0.0f;
    float A_Max = 0.0f;
    float Dt_S = 0.001f;

    float Planned_Position = 0.0f;
    float Planned_Velocity = 0.0f;
};

inline void Class_Gimbal_Joint_Planner::Set_Enable(bool enable)
{
    Enable = enable;
}

inline void Class_Gimbal_Joint_Planner::Set_Mode(Enum_Joint_Planner_Mode mode)
{
    Mode = mode;
}

inline bool Class_Gimbal_Joint_Planner::Get_Enable()
{
    return (Enable);
}

inline void Class_Gimbal_Joint_Planner::Reset(float now_pos)
{
    Planned_Position = now_pos;
    Planned_Velocity = 0.0f;
}

#endif
