#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "robotics.h"
#include "matrix.h"
#include "user_lib.h"

using namespace robotics;
using namespace matrixf;
/* Exported function declarations --------------------------------------------*/

float multi_to_single(float radian);
float normalize_angle(float angle);
/**
 * @brief DH建模类，主要用于模型的正逆解算，内置几个辅助函数
 *
 */
class Class_DH_model
{
public:
    Serial_Link<6> arm_model;
    float qmin[6];
    float qmax[6];

    Class_DH_model(Link _links[6], float _qmin[6], float _qmax[6]) : arm_model(_links)
    {
        for (int i = 0; i < 6; i++)
        {
            qmin[i] = _qmin[i];
            qmax[i] = _qmax[i];
        }
    };

    Class_DH_model() : arm_model(create_default_links())
    {
        float qmin_init[6] = {-2.883f, -0.926f, 0.15f, -3.028f, -1.336f, -PI};
        float qmax_init[6] = {2.883f, 0.926f, 2.133f, 3.028f, 1.336f, PI};
        memcpy(qmin, qmin_init, 6 * sizeof(float));
        memcpy(qmax, qmax_init, 6 * sizeof(float));
    };

    bool Fkine(float joint_angles[6], float pos[3], float rpy[3]);
    bool Fkine(float joint_angles[6], float T[4][4]);

    uint8_t Ikine_Pieper(float pos_target[3], float rpy_target[3], float q_solution[8][6], bool valid[8]);
    float Ikine_Pieper_Best(float pos_target[3], float rpy_target[3], float q_start[6], float q_solution[6]);

private:
    uint8_t Ikine_Pieper_calculator(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8]);

    bool check_valid(Matrixf<6, 1> solution);
    bool check_valid(float q[6]);
    uint8_t valid_count(Matrixf<6, 1> solution[8], bool valid[8]);

    float Euclidean_Distance(float q1[6], float q2[6]);
    float Euclidean_Distance(Matrixf<6, 1> q1, Matrixf<6, 1> q2);

    static Serial_Link<6> create_default_links() {
        Link links[6];
        links[0] = Link(0, 8.0f, 0, PI / 2, R, 0, -2.883f, 2.883f);
        links[1] = Link(0, 0, 35.0f, 0, R, PI / 2, -0.926f + PI / 2, 0.926f + PI / 2);
        links[2] = Link(0, 0, 0, -PI / 2, R, -PI / 2, 0.15f - PI / 2, 2.133f - PI / 2);
        links[3] = Link(0, 27.8f, 0, PI / 2, R, 0, -3.028f, 3.028f);
        links[4] = Link(0, 0, 0, -PI / 2, R, 0, -1.336f + PI / 2 + PI / 4 , (1.336f + PI / 4) + PI);
        links[5] = Link(0, 24.5f, 0, 0, R, 0, 0, 0);
        return Serial_Link<6>(links);
    }
};

class Class_Gimbal;

class Class_Trajectory_Tracer
{
    public:
    Class_DH_model dh_model;
    Class_Gimbal* Gimbal;
    
    /*构造函数*/
    Class_Trajectory_Tracer(Class_DH_model _dh_model, Class_Gimbal* _Gimbal, float _cali_offset);
    Class_Trajectory_Tracer();
    
    /*轨迹规划算法*/
    float get_s_at(uint32_t t_ms);
    float test_get_s_at();
    void get_pos_at(float q_start[6], uint8_t axis, float s, float pos[3]);

    /*轨迹规划器与逆解算*/
    uint32_t Trajectory_Generator(float q_start[6], uint8_t axis, float trajectory_xyz[600][3]);
    uint32_t Trajectory_Ikine(float q_start[6], float rpy_target[3], float trajectory_xyz[600][3], float q_solution[600][6], bool low_speed_flag[6]);

    void motor_angles_update();
    void arm_pos_rpy_update();
    void model_to_control(float model_angles[6], float control_angles[6]);
    void motor_to_model(float motor_angles[6], float model_angles[6]);

    private:

    float cali_offset;  // roll_1轴校准误差

    float now_motor_angles[6];
    float now_model_angles[6];

    float now_pos[3];
    float now_rpy[3];

    float trajectory_xyz_buffer[600][3];
    float q_solution_buffer[600][6];
};
#endif // ARM_MODEL_H