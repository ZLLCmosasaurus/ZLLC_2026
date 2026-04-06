/**
 * @file arm_model.cpp
 * @author hsl
 * @brief 机械臂机器人学模型，用于正逆运动学计算以及轨迹规划
 * @version 0.1
 * @date 2025-12-03
 *
 * @copyright ZLLC 2026
 *
 */
#include "robotics.h"
#include "matrix.h"
#include "utils.h"
#include "arm_model.h"
#include "crt_gimbal.h"
#ifndef PI
#define PI 3.1415926535f
#endif

using namespace robotics;
using namespace matrixf;

/*测试用函数*/
// 测试角度映射辅助函数，将roll_2的多圈转成单圈(0~2PI)
float multi_to_single(float radian)
{
    float single_radian = fmod(radian, 2.0f * PI);
    if (single_radian < 0)
    {
        single_radian += 2.0f * PI;
    }

    return single_radian;
}

Class_Trajectory_Tracer::Class_Trajectory_Tracer(Class_DH_model _dh_model, Class_Gimbal *_Gimbal, float _cali_offset) : dh_model(_dh_model)
{
    Gimbal = _Gimbal;
    cali_offset = _cali_offset;
};

Class_Trajectory_Tracer::Class_Trajectory_Tracer()
{
    dh_model = Class_DH_model();

    #ifdef PUMA
    cali_offset = Gimbal->Get_Roll_Cali_Offset() + 1.514f;
    #endif
};

bool Class_DH_model::Fkine(float joint_angles[6], float pos[3], float rpy[3])
{
    Matrixf<6, 1> q;
    for (int i = 0; i < 6; i++)
    {
        if (joint_angles[i] < qmin[i] || joint_angles[i] > qmax[i])
        {
            return false; // 关节角超出范围，返回false
        }

        q[i][0] = joint_angles[i];
    }

    Matrixf<4, 4> T = arm_model.fkine(q); // 正运动学计算得到齐次变换矩阵

    // 提取位置
    pos[0] = t2p(T)[0][0];
    pos[1] = t2p(T)[1][0];
    pos[2] = t2p(T)[2][0];

    // 提取RPY角
    rpy[0] = t2rpy(T)[0][0]; // Yaw
    rpy[1] = t2rpy(T)[1][0]; // Pitch
    rpy[2] = t2rpy(T)[2][0]; // Roll

    return true;
}

/**
 * @brief 运动学正解算函数，返回是否成功，若成功则说明关节角在范围内，反之则不在
 *
 * @param joint_angles 关节角数组，单位为弧度
 * @param T 输出正解算的齐次变换矩阵
 * @return bool 是否成功
 */
bool Class_DH_model::Fkine(float joint_angles[6], float T[4][4])
{
    Matrixf<6, 1> q;
    for (int i = 0; i < 6; i++)
    {
        if (joint_angles[i] < qmin[i] || joint_angles[i] > qmax[i])
        {
            return false; // 关节角超出范围，返回false
        }

        q[i][0] = joint_angles[i];
    }

    Matrixf<4, 4> T_mat = arm_model.fkine(q); // 写入旋转矩阵
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            T[i][j] = T_mat[i][j];
        }
    }

    return true;
}

uint8_t Class_DH_model::Ikine_Pieper_calculator(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8])
{
    Matrixf<3, 1> rpy;
    rpy[0][0] = rpy_target[0];
    rpy[1][0] = rpy_target[1];
    rpy[2][0] = rpy_target[2];

    Matrixf<4, 4> T_rot = rpy2t(rpy);

    // 设置位置 p2t
    Matrixf<3, 1> pos;
    pos[0][0] = pos_target[0];
    pos[1][0] = pos_target[1];
    pos[2][0] = pos_target[2];

    // 组合 Td = [R P; 0 1]
    // 库中 rp2t 可以直接从 R 和 p 构建 T
    Matrixf<4, 4> T_target = rp2t(t2r(T_rot), pos);

    int sol_count = 0;

    // DH建模参数
    float d1 = 8.0f;
    float a2 = 35.0f;
    float d4 = 27.8f;
    float d6 = 24.5f;
    float offset2 = PI / 2.0f;

    float Px = T_target[0][3];
    float Py = T_target[1][3];
    float Pz = T_target[2][3];

    // Index [2], [6], [10] 对应矩阵的第3列 (0,2), (1,2), (2,2)
    float Ax = T_target[0][2];
    float Ay = T_target[1][2];
    float Az = T_target[2][2];

    // 计算 Pc
    float Pcx = Px - d6 * Ax;
    float Pcy = Py - d6 * Ay;
    float Pcz = Pz - d6 * Az;

    // 第一阶段：位置求解 q1, q2, q3

    // q1 的两个候选解 (正向/背向)
    float q1_candidates[2];
    q1_candidates[0] = atan2f(Pcy, Pcx);
    q1_candidates[1] = atan2f(-Pcy, -Pcx);

    for (int i = 0; i < 2; i++)
    {
        float q1 = normalize_angle(q1_candidates[i]);

        // 将手腕中心投影到臂平面
        float r = sqrtf(Pcx * Pcx + Pcy * Pcy);
        if (i == 1)
            r = -r; // 背向解，r 取负

        float s = Pcz - d1;

        // --- 几何参数计算 ---
        float D2 = r * r + s * s;
        float D = sqrtf(D2);

        // 余弦定理求内角 phi (a2 与 d4 的夹角)
        float cos_phi = (a2 * a2 + d4 * d4 - D2) / (2.0f * a2 * d4);

        // 检查是否超出工作空间
        if (fabsf(cos_phi) > 1.0001f)
        {
            continue;
        }
        // 数值误差修正
        if (cos_phi > 1.0f)
            cos_phi = 1.0f;
        if (cos_phi < -1.0f)
            cos_phi = -1.0f;

        float phi = acosf(cos_phi);

        // 余弦定理求仰角偏差 beta (a2 与 D 的夹角)
        float cos_beta = (a2 * a2 + D2 - d4 * d4) / (2.0f * a2 * D);

        if (cos_beta > 1.0f)
            cos_beta = 1.0f;
        if (cos_beta < -1.0f)
            cos_beta = -1.0f;

        float beta = acosf(cos_beta);
        float psi = atan2f(s, r);

        // 构造两组 q2/q3 解 (Elbow Up / Elbow Down)
        float q3_opts[2];
        float q2_geom_opts[2];

        // 解A: 肘部向上
        q3_opts[0] = PI - phi;
        q2_geom_opts[0] = psi - beta;

        // 解B: 肘部向下
        q3_opts[1] = phi - PI;
        q2_geom_opts[1] = psi + beta;

        for (int k = 0; k < 2; k++)
        {
            // 计算实际 q2, q3
            float q3 = normalize_angle(q3_opts[k]);
            float q2 = normalize_angle(q2_geom_opts[k] - offset2);

            // 姿态求解 q4, q5, q6

            // 计算前三轴产生的旋转矩阵 R03
            // R03 = R01 * R12 * R23

            // 预计算三角函数
            float c1 = cosf(q1), s1 = sinf(q1);
            float th2 = q2 + offset2; // R12 用的 theta
            float c2 = cosf(th2), s2 = sinf(th2);
            float th3 = q3; // R23 用的 theta
            float offset3 = -PI / 2.0f;
            float th3_mat = q3 + offset3;
            float c3 = cosf(th3_mat), s3 = sinf(th3_mat);

            // 手动构建矩阵 (比通用矩阵乘法快)
            // T01 (alpha=90): [c1 0 s1; s1 0 -c1; 0 1 0]
            Matrixf<3, 3> R01;
            R01[0][0] = c1;
            R01[0][1] = 0;
            R01[0][2] = s1;
            R01[1][0] = s1;
            R01[1][1] = 0;
            R01[1][2] = -c1;
            R01[2][0] = 0;
            R01[2][1] = 1;
            R01[2][2] = 0;

            // T12 (alpha=0): [c2 -s2 0; s2 c2 0; 0 0 1]
            Matrixf<3, 3> R12;
            R12[0][0] = c2;
            R12[0][1] = -s2;
            R12[0][2] = 0;
            R12[1][0] = s2;
            R12[1][1] = c2;
            R12[1][2] = 0;
            R12[2][0] = 0;
            R12[2][1] = 0;
            R12[2][2] = 1;

            // T23 (alpha=-90): [c3 0 -s3; s3 0 c3; 0 -1 0]
            Matrixf<3, 3> R23;
            R23[0][0] = c3;
            R23[0][1] = 0;
            R23[0][2] = -s3;
            R23[1][0] = s3;
            R23[1][1] = 0;
            R23[1][2] = c3;
            R23[2][0] = 0;
            R23[2][1] = -1;
            R23[2][2] = 0;

            Matrixf<3, 3> R03 = R01 * R12 * R23;

            // 计算 R36 = R03' * R_target
            Matrixf<3, 3> R_target = t2r(T_target);
            Matrixf<3, 3> R36 = R03.trans() * R_target;

            // === 欧拉角提取 (针对 alpha4=90, alpha5=-90) ===
            float r13 = R36[0][2];
            float r23 = R36[1][2];
            float r31 = R36[2][0];
            float r32 = R36[2][1];
            float r33 = R36[2][2];

            // q5 的两个解
            float q5_candidates[2];
            // atan2(sqrt(x^2+y^2), z)
            q5_candidates[0] = atan2f(sqrtf(r13 * r13 + r23 * r23), r33);
            q5_candidates[1] = atan2f(-sqrtf(r13 * r13 + r23 * r23), r33);

            for (int m = 0; m < 2; m++)
            {
                float q5 = normalize_angle(q5_candidates[m]);
                float q4, q6;
                float s5 = sinf(q5);

                if (fabsf(s5) < 1e-4f)
                { // 奇异点
                    q4 = 0.0f;
                    // R11 = c4c6 - s4s6 = c(4+6) -> 这种情况下
                    // 使用 MATLAB 逻辑: q6 = atan2(-r12, r11)
                    q6 = atan2f(-R36[0][1], R36[0][0]);
                }
                else
                {
                    // q4 = atan2(-r23, -r13)
                    q4 = atan2f(-r23 / s5, -r13 / s5);
                    // q6 = atan2(-r32, r31)
                    q6 = atan2f(-r32 / s5, r31 / s5);
                }

                q4 = normalize_angle(q4);
                q6 = normalize_angle(q6);

                // 保存这一组解
                if (sol_count < 8)
                {
                    float sol_data[6] = {q1, q2, q3, q4, q5, q6};
                    solutions[sol_count] = Matrixf<6, 1>(sol_data);
                    sol_count++;
                }
            }
        }
    }
    return sol_count;
}

bool Class_DH_model::check_valid(Matrixf<6, 1> solution)
{
    for (int j = 0; j < 6; j++)
    {
        float angle = solution[j][0];
        // 检查每个关节角度是否在范围内
        if (angle < qmin[j] || angle > qmax[j])
        {
            return false;
        }
    }

    return true;
}

bool Class_DH_model::check_valid(float q[6])
{
    for (int j = 0; j < 6; j++)
    {
        // 检查每个关节角度是否在范围内
        if (q[j] < qmin[j] || q[j] > qmax[j])
        {
            return false;
        }
    }

    return true;
}

uint8_t Class_DH_model::valid_count(Matrixf<6, 1> solution[8], bool valid[8])
{
    uint8_t valid_count = 0;
    for (int i = 0; i < 8; i++)
    {
        valid[i] = check_valid(solution[i]);
        valid_count += valid[i] == true ? 1 : 0;
    }

    return valid_count;
}

/**
 * @brief 运动学逆解算函数，返回有效解的个数
 *
 * @param pos_target 目标位置 [x, y, z]
 * @param rpy_target 目标姿态 [yaw, pitch, roll]
 * @param q_solution 输出逆解结果，8组解，每组6个关节角
 * @param valid 输出每组解的有效性标志
 * @return uint8_t 实际求解出的有效解个数
 */
uint8_t Class_DH_model::Ikine_Pieper(float pos_target[3], float rpy_target[3], float q_solution[8][6], bool valid[8])
{
    Matrixf<6, 1> q_calc[8];

    Ikine_Pieper_calculator(pos_target, rpy_target, q_calc);
    uint8_t valid_cnt = valid_count(q_calc, valid);

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            q_solution[i][j] = q_calc[i][j][0];
        }
    }

    return valid_cnt;
}

/**
 * @brief 运动学逆解算函数，自动筛选欧氏距离最短的解，并返回欧式距离
 *
 * @param pos_target 目标位置 [x, y, z]
 * @param rpy_target 目标姿态 [yaw, pitch, roll]
 * @param q_solution 输出逆解结果，8组解，每组6个关节角
 * @param q_start 起始点的各关节角度
 * @return float 筛选出的优解的欧式距离
 */
float Class_DH_model::Ikine_Pieper_Best(float pos_target[3], float rpy_target[3], float q_start[6], float q_solution[6])
{
    float solutions[8][6];
    bool valid[8];
    float distance;

    uint8_t valid_cnt = Ikine_Pieper(pos_target, rpy_target, solutions, valid);

    if (valid_cnt == 0)
    {
        return -1.0f; // 无有效解
    }
    else
    {
        float d[8] = {0.0f};    // 欧式距离，等于各关节角差值平方之和再求平方根(这里略去求根这一步)
        uint8_t best_index = 0; // 求最值经典起手式
        for (int i = 0; i < 8; i++)
        {
            if (!valid[i])
            {
                d[i] = 114514.1919f;
            }

            d[i] = Euclidean_Distance(q_start, solutions[i]);

            if (d[i] < d[best_index])
                best_index = i;
        }

        distance = d[best_index];
        memcpy(q_solution, solutions[best_index], 6 * sizeof(float));
    }

    return distance;
}

/**
 * @brief 欧式空间距离计算函数，返回两组关节角之间的欧式空间距离
 *
 * @param q_1 关节角组1 q_2关节角组2
 * @return 两组关节角之间的欧式空间距离（正算术根）
 */
float Class_DH_model::Euclidean_Distance(float q1[6], float q2[6])
{
    float d = 0.0f;
    for (int i = 0; i < 6; i++)
    {
        float err = q1[i] - q2[i];
        d += err * err;
    }

    return sqrtf(d);
}

float Class_DH_model::Euclidean_Distance(Matrixf<6, 1> q1, Matrixf<6, 1> q2)
{
    float d = 0.0f;
    for (int i = 0; i < 6; i++)
    {
        float err = q1[i][0] - q2[i][0];
        d += err * err;
    }

    return sqrtf(d);
}

float normalize_angle(float angle)
{
    // 使用 user_lib.h 中的宏
    return rad_format(angle);
}

/**
 * @brief 速度规划函数s(t)，默认的梯形速度规划
 * length = 10.0cm, v_max = 4.0cm/s, a_max = 8.0cm/s^2
 * t_acc = 0.5s, t_flat = 2.0s, t_dec = 0.5s, t_total = 3.0s
 *
 * @param t_ms   当前时间 (毫秒)
 */
float Class_Trajectory_Tracer::get_s_at(uint32_t t_ms)
{
    // ms转成s
    float t_s = t_ms * 0.001f;
    float s = 0.0f, l = 10.0f, a = 8.0f, v = 4.0f;
    // 加速度 a = 8, 匀速运动速度 v = 4
    if (t_s < 0.5f)
    {
        s += 0.5f * a * t_s * t_s; // 加速阶段 s = 0.5 * a * t^2
    }
    else if (t_s < 2.5f)
    {
        s += 0.5f * a * 0.5f * 0.5f; // 加速段位移
        s += v * (t_s - 0.5f);       // 匀速段位移 s = s_acc + v * (t - t_acc)
    }
    else if (t_s <= 3.0f)
    {
        float t_remain = 3.0f - t_s;
        s = l - 0.5f * a * t_remain * t_remain;
    }
    else
    {
        s = l;
    }

    return s;
}

/**
 * @brief 取点函数p(s)，根据关节角度q_start，指定轴axis和平移长度s，计算平移后的末端xyz坐标
 *
 * @param q_start 关节角数组，单位为弧度
 * @param axis 指定的轴，0 - X轴，1 - Y轴，2 - Z轴
 * @param s 平移长度，单位为cm
 * @param pos 输出的末端xyz坐标数组
 */
void Class_Trajectory_Tracer::get_pos_at(float q_start[6], uint8_t axis, float s, float pos[3])
{
    float xyz[3]; // 平移前的xyz
    /*计算指定方向的方向向量*/
    float axis_vector[3] = {0.0f};
    // 0 - X-axis / 1 - Y-axis / 2 - Z-axis
    Matrixf<3, 3> R = dh_model.arm_model.fkine(Matrixf<6, 1>(q_start)).block<3, 3>(0, 0);
    axis_vector[0] = R[0][axis];
    axis_vector[1] = R[1][axis];
    axis_vector[2] = R[2][axis];

    // 计算当前xyz
    xyz[0] = dh_model.arm_model.fkine(Matrixf<6, 1>(q_start))[0][3];
    xyz[1] = dh_model.arm_model.fkine(Matrixf<6, 1>(q_start))[1][3];
    xyz[2] = dh_model.arm_model.fkine(Matrixf<6, 1>(q_start))[2][3];

    /*计算平移后的坐标*/
    pos[0] = xyz[0] + s * axis_vector[0];
    pos[1] = xyz[1] + s * axis_vector[1];
    pos[2] = xyz[2] + s * axis_vector[2];
}

void Class_Trajectory_Tracer::model_to_control(float model_angles[6], float control_angles[6])
{
    // 由于电机零点设置与建模时不同，此函数用于将模型中逆运动学得到的关节角度转换为电机控制角度
    control_angles[0] = model_angles[0] * 2.0f;              // J0-Yaw 减速比为2，零点和运动方向与控制层匹配
    control_angles[1] = model_angles[1] + 0.9268f;           // J1-Pitch1 电机直连，
    control_angles[2] = model_angles[2] * -1.5f + 3.43f;     // J2-Pitch2 方向相反，减速比1.5，这里把2.1345f看作零点，求当前角度在此基础上的增量，再取相反数乘减速比
    control_angles[3] = (model_angles[3] - 3.028f) * -50.0f; // J3-Roll 减速比为50，零点偏移173.5°，这里算出的是相对Roll_Min_Radian的增量，调用Gimbal中的Set_Target_Roll_Radian时会自动加上Roll_Min_Radian
    control_angles[4] = model_angles[4] - 1.277f;            // J4-Pitch3 直连
    // roll2的零点关系，暂时写为相等
    control_angles[5] = model_angles[5];
}

void Class_Trajectory_Tracer::motor_to_model(float motor_angles[6], float model_angles[6])
{
    // 由于电机零点设置与建模时不同，此函数用于将电机反馈的实际关节角度转换为模型中的关节角度进行正运动学求解
    model_angles[0] = (motor_angles[0] - PI);                       // J0-Yaw
    model_angles[1] = (motor_angles[1] - PI) * 4.0f - 0.9268f;      // J1-Pitch1
    model_angles[2] = -(motor_angles[2] - PI) / 1.5f + 2.1345f;     // J2-Pitch2 方向相反，减速比1.5
    model_angles[3] = -2.0f * (motor_angles[3] - PI - cali_offset); // J3-Roll 减速比为50，零点偏移PI（这个50的减速比真的需要除吗，存疑）
    model_angles[4] = motor_angles[4] - 0.5f * PI - 0.225f;         // J4-Pitch3
    // roll2的零点关系，暂时写为相等
    model_angles[5] = motor_angles[5];
}

uint32_t Class_Trajectory_Tracer::Trajectory_Generator(float q_start[6], uint8_t axis, float trajectory_xyz[600][3])
{
    static uint32_t cal_cnt = 0;
    float s = get_s_at(cal_cnt * 5);
    get_pos_at(q_start, axis, s, trajectory_xyz[cal_cnt]);
    if (cal_cnt == 600)
        cal_cnt = 0; // 计算完成后清零，防止溢出
    cal_cnt++;

    return cal_cnt;
}

uint32_t Class_Trajectory_Tracer::Trajectory_Ikine(float q_start[6], float rpy_target[3], float trajectory_xyz[600][3], float q_solution[600][6], bool low_speed_flag[6])
{
    static uint32_t cal_cnt = 0;    // 计算的次数，完成第一次计算则为1，一次都没完成则为0，可以看作是函数的调用次数
    static uint32_t result_cnt = 0; // 有效结果的个数0-600
    static float max_distance = 0.0f;

    float distance = dh_model.Ikine_Pieper_Best(trajectory_xyz[cal_cnt], rpy_target, q_start, q_solution[cal_cnt]);
    cal_cnt++;

    if (distance >= 0.0f)
    {
        result_cnt++;
        if (distance > max_distance)
        {
            max_distance = distance;
        }
    }

    if (cal_cnt == 300)
    // 匀速段时计算各个轴的变化量，判断是否为低速运动，这里暂时只写8009的
    {
        low_speed_flag[1] = fabsf(q_solution[298][1] - q_solution[299][1]) < 4*1e-4;
    }

    return result_cnt;
}

void Class_Trajectory_Tracer::motor_angles_update()
{
    #ifdef PUMA
    now_motor_angles[0] = Gimbal->Motor_DM_J0_Yaw.Get_Now_Angle();
    now_motor_angles[1] = Gimbal->Motor_DM_J1_Pitch.Get_Now_Angle();
    now_motor_angles[2] = Gimbal->Motor_DM_J2_Pitch_2.Get_Now_Angle();
    now_motor_angles[3] = Gimbal->Motor_DM_J3_Roll.Get_Now_Angle();
    now_motor_angles[4] = Gimbal->Motor_DM_J4_Pitch_3.Get_Now_Angle();
    now_motor_angles[5] = multi_to_single(Gimbal->Motor_6020_J5_Roll_2.Get_Now_Angle());

    for (int i = 0; i < 6; i++)
    {
        motor_to_model(now_motor_angles, now_model_angles);
    }
    #endif
}

void Class_Trajectory_Tracer::arm_pos_rpy_update()
{
    motor_angles_update();
    dh_model.Fkine(now_model_angles, now_pos, now_rpy);
}