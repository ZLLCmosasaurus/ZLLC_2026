#include "crt_joint_planner.h"

void Class_Gimbal_Joint_Planner::Init(Class_DM_Motor_J4310 *motor, const float *raw_target_ptr, float v_max, float a_max, float dt_s)
{
    Motor = motor;
    Raw_Target_Ptr = raw_target_ptr;
    V_Max = Math_Abs(v_max);
    A_Max = Math_Abs(a_max);
    Dt_S = dt_s;

    if (Motor != nullptr)
    {
        Reset(Motor->Get_Now_Angle_Rad());
    }
}

void Class_Gimbal_Joint_Planner::Update()
{
    if (Enable == false || Motor == nullptr || Raw_Target_Ptr == nullptr)
    {
        return;
    }

    const float target = *Raw_Target_Ptr;

    if (Mode == Joint_Planner_Mode_DISABLED)
    {
        Planned_Position = target;
        Planned_Velocity = 0.0f;
        return;
    }

    const float delta = target - Planned_Position;
    if (Math_Abs(delta) <= 1e-4f)
    {
        Planned_Position = target;
        Planned_Velocity = 0.0f;
        return;
    }

    const float direction = (delta > 0.0f) ? 1.0f : -1.0f;
    const float accel_step = A_Max * Dt_S;

    switch (Mode)
    {
    case Joint_Planner_Mode_CONSTANT_ACCEL:
    {
        const float braking_distance = (Planned_Velocity * Planned_Velocity) / (2.0f * A_Max + 1e-6f);
        if (Math_Abs(delta) <= braking_distance)
        {
            Planned_Velocity -= direction * accel_step;
        }
        else
        {
            Planned_Velocity += direction * accel_step;
        }

        Math_Constrain(&Planned_Velocity, -V_Max, V_Max);

        if (Planned_Velocity * direction < 0.0f)
        {
            Planned_Velocity = 0.0f;
        }
        break;
    }
    case Joint_Planner_Mode_TRAPEZOID_VELOCITY:
    {
        const float braking_distance = (Planned_Velocity * Planned_Velocity) / (2.0f * A_Max + 1e-6f);
        if (Math_Abs(delta) <= braking_distance)
        {
            Planned_Velocity -= direction * accel_step;
        }
        else
        {
            Planned_Velocity += direction * accel_step;
        }

        Math_Constrain(&Planned_Velocity, -V_Max, V_Max);

        if (Planned_Velocity * direction < 0.0f)
        {
            Planned_Velocity = 0.0f;
        }
        break;
    }
    default:
    {
        Planned_Position = target;
        Planned_Velocity = 0.0f;
        return;
    }
    }

    const float next_position = Planned_Position + Planned_Velocity * Dt_S;
    if (((target - Planned_Position) > 0.0f && next_position >= target) ||
        ((target - Planned_Position) < 0.0f && next_position <= target))
    {
        Planned_Position = target;
        Planned_Velocity = 0.0f;
    }
    else
    {
        Planned_Position = next_position;
    }
}

void Class_Gimbal_Joint_Planner::Dispatch()
{
    if (Motor == nullptr)
    {
        return;
    }

    if (Enable == false || Mode == Joint_Planner_Mode_DISABLED)
    {
        Motor->TIM_Process_PeriodElapsedCallback();
        return;
    }

    Motor->Set_Target_Angle(Planned_Position);
    Motor->TIM_Process_PeriodElapsedCallback();
}
