/**
 * @file crt_booster.cpp
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

int a,b,c,d;
volatile int e,f,g,h;

int time_test_pushing = 0;

float GM6020_angle_RELOAD[4] = {117.0f * PI / 180.0f, 241.0f * PI / 180.0f, 360.0f * PI / 180.0f, 479.0f * PI / 180.0f};
float GM6020_angle_ELUDE[4] = {198.0f * PI / 180.0f, 316.0f * PI / 180.0f, 435.0f * PI / 180.0f, 559.0f * PI / 180.0f};


// float GM6020_angle_1_ELUDE = 0.0f;
// float GM6020_angle_2_ELUDE = 0.0f;
// float GM6020_angle_3_ELUDE = 0.0f;
// float GM6020_angle_4_ELUDE = 0.0f;

// float GM6020_angle_1_RELOAD = 0.0f;
// float GM6020_angle_2_RELOAD = 0.0f;
// float GM6020_angle_3_RELOAD = 0.0f;
// float GM6020_angle_4_RELOAD = 0.0f;


int test123 = 0;
int PB3_GPIO = 0;
int PD7_GPIO = 0;
int PE15_GPIO = 0;

// PB3 中断锁存：按下一次即记住，直到状态机消费
static volatile bool pb3_press_event_latched = false;
volatile uint32_t pb3_exti_irq_count = 0;
volatile uint32_t pb3_event_consumed_count = 0;

// PD7 中断锁存：按下一次即记住，直到状态机消费（Push 前端检测）
static volatile bool pd7_press_event_latched = false;
volatile uint32_t pd7_exti_irq_count = 0;
volatile uint32_t pd7_event_consumed_count = 0;

bool push_ready_or_switch = 0;

//换弹次数
int reload_count = 0;
// 换弹完成后发给 Shooting 的一次性放行 token（仅可消费一次）
static uint32_t reload_done_token = 0;
static uint32_t last_consumed_reload_done_token = 0;

float test_Motor_Reload_Linear_Target = 0.5f; // 换弹直线电机测试目标位置
float test_reload_servo_angle = 220.0f; // 舵机测试目标角度

//push电机target能够容忍的误差
float push_target_tolerance = 0.006f;

// 校准是否完成相关标志位
bool Push_Calibration_Finished = false;
bool Pull_Calibration_Finished = false;
bool Reload_Linear_Calibration_Finished = false;

// 是否允许发射相关标志位
// bool Loading_Slider_Ready; // 上膛滑块机构就位
bool Referee_Allow_Shoot = false; // 裁判系统允许发射
int test_allow_fire = 0; // 测试用，允许发射标志位

// 已发镖数量
// static int dart_fired_count = 0;
// static int last_dart_fired_count = 0;

int dart_fired_count = 0;
int last_dart_fired_count = 0;

// READY_PRE：push到位事件时间戳（-1 表示尚未到位）
// static int ready_pre_push_reached_time = -1;
int ready_pre_push_reached_time = -1;

// 拉力误差连续满足阈值的累计时间（单位：ms）
uint16_t tension_in_range_time_ms = 0;

// int servo_test;
int servo_test_flag = 0;
float test_0_1_push = 0.03f;
float test_0_1_pull = 0.9f;
float test_0_1_reload_linear = 0.0f;

// Pull 校准完成后在校准状态机内部锁位
float pull_hold_after_calib_pos = 0.8f;

// 在换弹机构中舵机延时相关变量（临时） 后续可能会换成总线舵机
int reload_servo_flag_drop = 0;
int reload_servo_flag_lift = 0;
int reload_servo_drop_time = 0;
int reload_servo_lift_time = 0;

/*解决裁判系统允许发射信号边沿检测问题 换弹 HOLD*/
static bool referee_allow_prev = false;
static uint32_t referee_allow_rise_cnt = 0;

//使能发射机构的enable_booster_flag
uint8_t enable_booster_flag = 0;

void Update_Referee_Allow_Edge()
{
    if (Referee_Allow_Shoot && !referee_allow_prev) {
        referee_allow_rise_cnt++;
    }
    referee_allow_prev = Referee_Allow_Shoot;
}

extern "C" void Booster_On_PB3_Exti(void)
{
    pb3_exti_irq_count++;
    pb3_press_event_latched = true;
}

extern "C" void Booster_On_PD7_Exti(void)
{
    pd7_exti_irq_count++;
    pd7_press_event_latched = true;
}

bool Consume_PB3_Press_Event()
{
    __disable_irq();
    bool has_event = pb3_press_event_latched;
    pb3_press_event_latched = false;
    __enable_irq();
    if (has_event)
    {
        pb3_event_consumed_count++;
    }
    return has_event;
}

bool Consume_PD7_Press_Event()
{
    __disable_irq();
    bool has_event = pd7_press_event_latched;
    pd7_press_event_latched = false;
    __enable_irq();
    if (has_event)
    {
        pd7_event_consumed_count++;
    }
    return has_event;
}
/*-----------------------------------------------*/

/* Private types -------------------------------------------------------------*/
#define TENSION_DEADZONE 0.2f // 拉力死区
#define SCREW_LEAD 0.004f // 4mm 螺距（每转一圈线性移动4mm）
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  将当前角度线性映射到目标行程
 * @param  curr_angle   当前电机角度
 * @param  angle_start  起始点角度（通常是 Backward 角度）0
 * @param  angle_end    结束点角度（通常是 Forward 角度）1
 * @param  max_length   物理最大行程（例如 1.0 表示百分比，或者 200.0 表示 mm）
 * @return 映射后的位置值
 */
float Class_FSM_Push_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

float Class_FSM_Pull_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

float Class_FSM_Reload_Linear_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

// 已经通过串口读取到一拉力值
// 使用全局变量保存，单位为kg
// 拉力环比例系数
float K_tension = 0.0000001f;
/**
 * @brief 拉力外环控制（将拉力误差映射为 Pull 电机的目标位置）
 *
 */
void Class_Booster::Pull_Tension_Control(bool is_first_run)
{
    // 1. [关键] 首运行初始化：防止电机突然跳变
    if (is_first_run)
    {
        // 假设 Get_Now_position_pull() 返回的是当前归一化位置(0~1)
        target_tension_position_pull = Get_Now_position_pull();
    }

    {
        // 读取测量值与目标值
        now_tension_value = Get_Measured_Tension();
        target_tension_value = Get_Target_Tension();

        float tension_error = target_tension_value - now_tension_value;

        //  [关键] 增加死区防止抖动
        if (fabs(tension_error) < TENSION_DEADZONE)
        {
            tension_error = 0.0f;
        }

        target_tension_position_pull -= K_tension * tension_error;

        // 限幅
        if (target_tension_position_pull > 1.0f)
        {
            target_tension_position_pull = 1.0f;
        }
        if (target_tension_position_pull < 0.0f)
        {
            target_tension_position_pull = 0.0f;
        }

        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pull.Set_Target_Radian(target_tension_position_pull);
    }
}

void Class_FSM_Push_Calibration::Push_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0): // 向前堵转 目前用微动开关
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Booster->Motor_Push_L.Set_Target_Omega_Radian(speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(speed);

        // 进入该状态第一帧清除旧事件，避免跨状态误触发
        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
        }
        
        if(Consume_PD7_Press_Event())//前左侧微动开关触发
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Forward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Forward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Set_Status(1);
        }
    }
    break;
    case (1): // 向后堵转 目前用微动开关
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Booster->Motor_Push_L.Set_Target_Omega_Radian(-speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(-speed);

        // 进入该状态第一帧清除旧事件，避免跨状态误触发
        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PB3_Press_Event();
        }
        
        if(Consume_PB3_Press_Event())//后端左侧微动开关触发
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Backward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Backward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Push_Calibration_Finished = true;
            Set_Status(2);
        }
    }
    break;
    case (2): 
    {
        // 定义上面是1.0f最大行程 下面是0.0f最小行程
        // 所以在函数映射的时候颠倒了位置
        float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L, 1.0f);
        float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R, 1.0f);
        float now_position = (now_position_l + now_position_r) / 2.0f;
        Booster->Set_Now_position_push(now_position); // 更新当前push电机位置
        // 更新PID输入值
        Booster->Motor_Push_L.Set_Transform_Angle(now_position);
        Booster->Motor_Push_R.Set_Transform_Angle(now_position);

        if (Push_Calibration_Finished && Pull_Calibration_Finished && Reload_Linear_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
        }
    }
    }
}

void Class_FSM_Pull_Calibration::Pull_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0): // 向前堵转
    {
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(speed);

        if (fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(1);
        }
    }
    break;
    case (1): // 前侧检测
    {
        if (Status[Now_Status_Serial].Time > 100)
        {
            Angle_Forward = Booster->Motor_Pull.Get_Now_Angle();
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(0.0f);
            Set_Status(2);
        }
        else if (fabs(Booster->Motor_Pull.Get_Now_Torque()) < Torque_Threshold)
        {
            Set_Status(0);
        }
    }
    break;
    case (2): // 向后堵转
    {

        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(-speed);

        if (fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(3);
        }
    }
    break;
    case (3): // 后侧检测
    {
        if (Status[Now_Status_Serial].Time > 100)
        {
            Angle_Backward = Booster->Motor_Pull.Get_Now_Angle();
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(0.0f);
            Set_Status(4);
        }
        else if (fabs(Booster->Motor_Pull.Get_Now_Torque()) < Torque_Threshold)
        {
            Set_Status(2);
        }
    }
    break;
    case (4): // 正常控制流程
    {
        Pull_Calibration_Finished = true;
        Set_Status(5);
    }
    break;
    case (5): // 校准检测
    {
        // 进入校准保持态首帧时，切到角度环并锁定到固定位置，避免“放空”漂移
        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);
        }

        // 定义上面是1.0f最大行程 下面是0.0f最小行程
        float now_position = Linear_Map_Position(Booster->Motor_Pull.Get_Now_Angle(), Angle_Backward, Angle_Forward, 1.0f); // 注意这里颠倒了
        Booster->Set_Now_position_pull(now_position);                                                                       // 更新当前pull电机位置
        // 更新PID输入值
        Booster->Motor_Pull.Set_Transform_Angle(now_position);


        if (Push_Calibration_Finished && Pull_Calibration_Finished && Reload_Linear_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
        }
    }
    }
}

void Class_FSM_Reload_Linear_Calibration::Linear_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0): // 向前堵转
    {
        Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Reload_Linear.Set_Target_Omega_Radian(-speed);

        if (fabs(Booster->Motor_Reload_Linear.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(1);
        }
    }
    break;
    case (1): // 前侧检测
    {
        if (Status[Now_Status_Serial].Time > 100)
        {
            Angle_Forward = Booster->Motor_Reload_Linear.Get_Now_Angle();
            Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Reload_Linear.Set_Target_Omega_Radian(0.0f);
            Set_Status(2);
        }
        else if (fabs(Booster->Motor_Reload_Linear.Get_Now_Torque()) < Torque_Threshold)
        {
            Set_Status(0);
        }
    }
    break;
    case (2): // 向后堵转
    {

        Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Reload_Linear.Set_Target_Omega_Radian(speed);

        if (fabs(Booster->Motor_Reload_Linear.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(3);
        }
    }
    break;
    case (3): // 后侧检测
    {
        if (Status[Now_Status_Serial].Time > 100)
        {
            Angle_Backward = Booster->Motor_Reload_Linear.Get_Now_Angle();
            Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Reload_Linear.Set_Target_Omega_Radian(0.0f);
            Set_Status(4);
        }
        else if (fabs(Booster->Motor_Reload_Linear.Get_Now_Torque()) < Torque_Threshold)
        {
            Set_Status(2);
        }
    }
    break;
    case (4): // 正常控制流程
    {
        Reload_Linear_Calibration_Finished = true;
        Set_Status(5);
    }
    break;
    case (5): // 校准检测
    {
        //Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // 定义上面是1.0f最大行程 下面是0.0f最小行程
        float now_position = Linear_Map_Position(Booster->Motor_Reload_Linear.Get_Now_Angle(), Angle_Backward, Angle_Forward, 1.0f); // 注意这里颠倒了
        Booster->Set_Now_position_reload_linear(now_position);                                                                       // 更新当前linear电机位置
        // 更新PID输入值
        Booster->Motor_Reload_Linear.Set_Transform_Angle(-now_position); // 这里注意：linear电机的正反转和位置定义相反，所以要取负值？？
        if (Push_Calibration_Finished && Pull_Calibration_Finished && Reload_Linear_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
        }
    }
    break;
    }
}

void Class_FSM_Shooting::Shooting_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (Shooting_Control_Type_WAITING):
    {
        // 转到 READY_PRE 状态的条件：大状态是Booster_Control_Type_NORMAL(校准完成)
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL)
        {
            Set_Status(Shooting_Control_Type_INIT);
        }
    }
    break;
    case (Shooting_Control_Type_INIT):
    {
        // disable状态下，保持电机关闭
        // other code------

        bool is_reloading = (Booster->Get_Reload_Status() == Reload_Status_RELOADING);
        static int init_push_reached_time = -1;

        if (Status[Now_Status_Serial].Time == 1)
        {
            init_push_reached_time = -1;
            (void)Consume_PD7_Press_Event(); // 清旧事件，防止跨状态误触发
        }

        // 保持电机在初始位置
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        if (!is_reloading)
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        }

        //push 和 pull 电机都在默认位置先不动 等待指令
        Booster->Motor_Pull.Set_Target_Radian(0.6f);
        if (!is_reloading)
        {
            Booster->Motor_Push_L.Set_Target_Radian(0.95f);
            Booster->Motor_Push_R.Set_Target_Radian(0.95f);

            bool init_push_ready_or_switch =
                (fabs(Booster->Get_Now_position_push() - 0.95f) < push_target_tolerance) ||
                Consume_PD7_Press_Event();

            // 到位或触发前端微动开关后立即锁位，防止继续顶压
            if (init_push_ready_or_switch && init_push_reached_time < 0)
            {
                // const float hold_push_pos_init = Booster->Get_Now_position_push();
                // Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                // Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                // Booster->Motor_Push_L.Set_Target_Radian(hold_push_pos_init-0.08f);// 这里可以微调一下位置，确保更稳妥地触碰到微动开关
                // Booster->Motor_Push_R.Set_Target_Radian(hold_push_pos_init-0.08f);
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);// 这里可以微调一下位置，确保更稳妥地触碰到微动开关
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
                init_push_reached_time = 1;//手动设置为正数1
            }
        }

        // 转到 READY_PRE 状态的条件：大状态是Booster_Control_Type_NORMAL(校准完成)
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL 
        && Referee_Allow_Shoot
        && (is_reloading || init_push_reached_time > 0)
        ) 
        {
            Set_Status(Shooting_Control_Type_READY_PRE);
        }
        // else
        // {
        //     Set_Status(Shooting_Control_Type_WAITING);
        // }
    }
    break;
    case (Shooting_Control_Type_READY_PRE): // 预准备状态：停留一会儿，让舵机撒放器闭合扣住，上膛滑块就位
    {
       bool is_reloading = (Booster->Get_Reload_Status() == Reload_Status_RELOADING);

        // 只在进入该状态的第一次循环执行
        if (Status[Now_Status_Serial].Time == 1)
        {
            ready_pre_push_reached_time = -1;//复位
            (void)Consume_PB3_Press_Event();   // 清旧事件，防止跨状态误触发

            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(Booster->target_position_pull);
            if (!is_reloading)
            {
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_L.Set_Target_Omega_Radian(-240.f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(-240.f);
            }
        }

        //跑到最低点
        push_ready_or_switch = Consume_PB3_Press_Event();

        // 条件是触碰到微动开关
        if (push_ready_or_switch && ready_pre_push_reached_time < 0)
        {
            // 撒放器闭合已经在跑校准过程中完成，但是由于循环跑状态机，所以要再设置一次
            // 保持角度环并锁定当前位姿，防止过冲
            // const float hold_push_pos1 = Booster->Get_Now_position_push();
            // Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Booster->Motor_Push_L.Set_Target_Radian(hold_push_pos1 + 0.003f);// 这里可以微调一下位置，确保更稳妥地触碰到微动开关
            // Booster->Motor_Push_R.Set_Target_Radian(hold_push_pos1 + 0.003f);

            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle); // 舵机扣住
            ready_pre_push_reached_time = Status[Now_Status_Serial].Time;// 记录上膛滑块到位的时间戳
        }

       // 只有“已记录到底时间”后才允许进入 下一阶段
        bool ready_pre_done =
        (ready_pre_push_reached_time > 0) &&
        ((Status[Now_Status_Serial].Time - ready_pre_push_reached_time) > 1200); 

        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL
        && ready_pre_done
        /*&& Booster->Get_Now_position_push() > 0.9f*/
        && Booster->Get_Reload_Status() == Reload_Status_FINISHED
        && Referee_Allow_Shoot) 
        {
        Set_Status(Shooting_Control_Type_READY_PRE2);
        }
        else
        {
            // 否则不动 卡在这里
        }
    }
    break;
    case (Shooting_Control_Type_READY_PRE2):
    {
        // 这个状态是中继状态
        // 上一阶段的push在下面卡着 本阶段把push移到最上面
        bool is_reloading = (Booster->Get_Reload_Status() == Reload_Status_RELOADING);
        static int ready_pre2_push_reached_time = -1;
        if (Status[Now_Status_Serial].Time == 1)
        {
            ready_pre2_push_reached_time = -1;
            (void)Consume_PD7_Press_Event(); // 清旧事件，防止跨状态误触发

            // Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(Booster->target_position_pull);
            // Booster->Motor_Push_L.Set_Target_Radian(0.95f);
            // Booster->Motor_Push_R.Set_Target_Radian(0.95f);
            
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_L.Set_Target_Omega_Radian(240.f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(240.f);

        }

        bool push_top_ready_or_switch = 
        // (fabs(Booster->Get_Now_position_push() - 0.95f) < push_target_tolerance) ||
        Consume_PD7_Press_Event();
        // 条件是到达位置或者触碰到前端微动开关，触发后立即锁位，防止继续顶压
        if (push_top_ready_or_switch && ready_pre2_push_reached_time < 0)
        {
            // const float hold_push_pos2 = Booster->Get_Now_position_push();
            // Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Booster->Motor_Push_L.Set_Target_Radian(hold_push_pos2-0.08);
            // Booster->Motor_Push_R.Set_Target_Radian(hold_push_pos2-0.08);

            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.f);

            ready_pre2_push_reached_time = Status[Now_Status_Serial].Time;
        }
        

        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL 
        && Referee_Allow_Shoot
        && ready_pre2_push_reached_time > 0 // 到位或触发微动开关后才允许进入下一状态
        && fabs(Booster->Get_Now_position_pull() - Booster->target_position_pull) < push_target_tolerance // 拉力位置到位的条件，可以微调
        && Booster->Get_Reload_Status() == Reload_Status_FINISHED)
        {
            Set_Status(Shooting_Control_Type_READY);

        }
    }
    break;
    case (Shooting_Control_Type_READY): // 正式准备状态
    {
        //记录上端触发微动开关后的目前角度
        static uint8_t swtich_mode = 1;
        static float offset_position = 0.0f;
        if(swtich_mode)
        {
            offset_position = (Booster->Motor_Push_L.Get_Now_Radian() + Booster->Motor_Push_R.Get_Now_Radian()) / 2.0f / (2.0f * PI) * SCREW_LEAD;
            swtich_mode = 0;
        }
        float now_position = (Booster->Motor_Push_L.Get_Now_Radian() + Booster->Motor_Push_R.Get_Now_Radian()) / 2.0f / (2.0f * PI) * SCREW_LEAD;
        Booster->Motor_Push_L.Set_Target_Omega_Radian(-8.0f);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(-8.0f);
        if(now_position < offset_position - 0.006f)//向下12mm的行程距离
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            swtich_mode = 1;
            Set_Status(Shooting_Control_Type_PULLRING); 
        }
    }
    break;
    case(Shooting_Control_Type_PULLRING): // 拉环状态，保持一段时间后进入发射状态
    {
        //bool is_reloading = (Booster->Get_Reload_Status() == Reload_Status_RELOADING);
        // static int ready_push_reached_time = -1;

        // // Pull电机跑拉力环
        // 如果是刚进入该状态的第一帧
        // bool first_run = (Status[Now_Status_Serial].Time == 1);

        // Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Pull_Tension_Control(first_run);

        // if (fabs(Booster->now_tension_value - Booster->target_tension_value) < 100.0f) //单位g
        // {
        //     tension_in_range_time_ms += 1; // 每次调用增加1ms
        // }
        // else
        // {
        //     tension_in_range_time_ms = 0; // 不满足条件，重置计时
        // }

        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(0.04f);

        // 发射条件：上膛滑块就位，整体booster处于Normal状态
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL 
        /*&& (is_reloading || ready_push_reached_time > 0)*/
        /*&& fabs(Booster->now_tension_value - Booster->target_tension_value) < 100.0f //单位g*/
        /*&& tension_in_range_time_ms >= 200 // 拉力稳定满足条件至少100ms*/
        && fabs(Booster->Get_Now_position_pull() - 0.04f) < push_target_tolerance // 拉力位置到位的条件，可以微调
        && Booster->Get_Reload_Status() == Reload_Status_FINISHED // 换弹完成状态
        && Referee_Allow_Shoot 
        /*&& test_allow_fire == 1*/)
        {
            // 发射动作：舵机转到发射角度
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
            Set_Status(Shooting_Control_Type_SHOOTING); // 进入发射状态
        }
        else
        {
            // 否则不动 卡在这里
        }
    }
    break;
    case (Shooting_Control_Type_SHOOTING):
    {
        if (test_allow_fire == 1)
        {
            
        }
        // 发射后，等待一段时间让球飞出
        if (Status[Now_Status_Serial].Time > 500) // 等待500ms
        {
            // 发射完成后，恢复舵机位置
            // 此处继续保持打开状态，等待上膛
            if (test_allow_fire == 1)
            {
                Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle); // 尤其注意！！！
            }
            // 发射是很快的，所以不能立刻回归初始位置，否则会炸膛

            // 记录发射飞镖数量
            dart_fired_count += 1;

            Referee_Allow_Shoot = false; // 发射一次后，禁止继续发射，等待裁判系统允许

            Set_Status(Shooting_Control_Type_SHOOTING_FINISHED);
        }
    }
    break;
    case (Shooting_Control_Type_SHOOTING_FINISHED):
    {
        // 发射完成后的状态处理

        // 先保持电机的位置不动，等待上膛完成和裁判系统允许发射的信号
        // Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Push_L.Set_Target_Radian(0.97f);
        // Booster->Motor_Push_R.Set_Target_Radian(0.97f);

        // Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Pull.Set_Target_Radian(0.90f);
    // Reload_Control_Type_WAITING,          //等待上膛滑块到位
    // Reload_Control_Type_PUSHING,          // 上弹推进过程
    // Reload_Control_Type_RETRACTING,       // 换弹机构回退过程（给发射机构让路）
    // Reload_Control_Type_HOLD,             // 保持当前角度不动状态

        // 在此状态卡住，等待裁判系统允许 + 新换弹完成 token（一次性）
        if (Referee_Allow_Shoot
        && Booster->Get_Reload_Status() == Reload_Status_FINISHED
        && reload_done_token > last_consumed_reload_done_token)
        {
            // 消费 token，避免同一次换弹被重复放行
            last_consumed_reload_done_token = reload_done_token;
            // 重置状态机，回到 INIT 状态，准备下一次发射
            Set_Status(Shooting_Control_Type_INIT);
        }

    }
    }
    Shooting_Control_Type = static_cast<Enum_Shooting_Control_Type>(Now_Status_Serial);
}

void Class_FSM_Reload::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (Reload_Control_Type_UNCALIBRATED):
    {
        // linear电机没校准完，不管
        // angle电机不用校准，一开始必须要保持在初始位置
        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(Booster->init_position_reload_angle);

        Booster->target_position_reload_angle = Booster->Motor_Reload_Angle.Get_Target_Radian();
        // 舵机在一定角度   
        Booster->Servo_Reload.Set_Target_Angle(Booster->reload_lift_angle);

        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL) // 校准完成
        {
            Set_Status(Reload_Control_Type_INIT);
        }
    }
    break;
    case (Reload_Control_Type_INIT):    
    {
        // 一开始的Enum_Reload_Status在初始化的时候就设置为FINISHED状态
        // 保持电机在初始位置
        Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Linear.Set_Target_Radian(-Booster->init_position_reload_linear);// 注意：这个电机的正反转和位置定义相反，所以要取负值
        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(Booster->init_position_reload_angle);

        Booster->target_position_reload_angle = Booster->Motor_Reload_Angle.Get_Target_Radian();
        // 舵机在一定角度
        Booster->Servo_Reload.Set_Target_Angle(Booster->reload_lift_angle);

        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL     
        /*&& Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_READY*/
        && (dart_fired_count - last_dart_fired_count > 0) 
        && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED 
        && Referee_Allow_Shoot) // 有发射动作发生
        {
            // 换弹状态设置为：换弹中...
            Booster->Set_Reload_Status(Reload_Status_RELOADING);
            // 进入下一状态
            Set_Status(Reload_Control_Type_WAITING);
        }
    }
    break;
    case (Reload_Control_Type_WAITING):
    {
        // 等待状态：只做状态切换，具体动作放到 PUSHING 中按顺序执行
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL
        && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED
        && Referee_Allow_Shoot
        )
        {
            Set_Status(Reload_Control_Type_PUSHING);
        }
    }
    break;
    case (Reload_Control_Type_PUSHING):
    {
        static int pushing_stage = 0;
        static int pushing_servo_drop_time = 0;

        time_test_pushing++;
        // 只在进入该状态的第一次循环执行
        if (Status[Now_Status_Serial].Time == 1)
        {
            pushing_stage = 0;
            pushing_servo_drop_time = 0;
            (void)Consume_PB3_Press_Event(); // 清旧事件，避免跨状态误触发

            // // 换弹角度电机转动40度
            // Booster->target_position_reload_angle += 40.0f * PI / 180.0f;
            Booster->target_position_reload_angle = GM6020_angle_RELOAD[dart_fired_count] -0.5f * PI / 180.0f; // 这里预设了每发射一次，换弹角度电机增加40度，可以根据实际情况调整
        }
        // Stage 0: Push先下压到位（位置 / PB3电平 / PB3边沿 任一满足）
        if (pushing_stage == 0)
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_L.Set_Target_Omega_Radian(-240.f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(-240.f);

            bool push_bottom_ready_or_switch =
                (PB3_GPIO == 1) ||
                Consume_PB3_Press_Event();

            if (push_bottom_ready_or_switch)
            {
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
                pushing_stage = 1;
            }
        }

        static int time2_test_pushing = 0;
        // Stage 1: Push到位后，6020再转到下一角度
        if (pushing_stage >= 1)
        {
            Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);

            if (pushing_stage == 1
                && fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - Booster->target_position_reload_angle) < 0.003f)
            {
                time2_test_pushing++;
                // Stage 2: 6020到位后舵机动作
                Booster->Servo_Reload.Set_Target_Angle(Booster->reload_drop_angle);
                pushing_servo_drop_time = Status[Now_Status_Serial].Time;
                pushing_stage = 2;
            }
            // else
            // {
            //     time2_test_pushing = 0;
            // }

            // if(time2_test_pushing > 200){
            //     pushing_stage = 2;
            //     time2_test_pushing = 0;
            // }
        }

        // Stage 3: 舵机动作后延时，再让 linear 前进
        if (pushing_stage == 2
            && (Status[Now_Status_Serial].Time - pushing_servo_drop_time) > 750)
        {
            pushing_stage = 3;
        }

        if (pushing_stage >= 3)
        {
            Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Reload_Linear.Set_Target_Radian(-0.08f); // 直线电机前进往下压
        }

        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL  
        && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED 
        && Referee_Allow_Shoot
        && pushing_stage >= 3
        && fabs(Booster->Get_Now_position_reload_linear() - 0.08f) < 0.05f /*达到直线电机前进位置*/ ) 
        {
            // 离开前复位标志位，供下次使用
            reload_servo_flag_drop = 0;//好像没用？  别删
            reload_servo_drop_time = 0;
            Set_Status(Reload_Control_Type_RETRACTING);
        }
    }
    break;
    case (Reload_Control_Type_RETRACTING):
    {
        /*----------------------------------------------*/
        // 直线电机后退返回初始位置
        Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Linear.Set_Target_Radian(-Booster->init_position_reload_linear); // 注意：这个电机的正反转和位置定义相反，所以要取负值

        // 1. 触发舵机动作
        if (fabs(Booster->Get_Now_position_reload_linear() - Booster->init_position_reload_linear) < 0.01f && reload_servo_flag_lift == 0) // 达到直线电机初始位置
        {
            reload_servo_flag_lift = 1;
            reload_servo_lift_time = Status[Now_Status_Serial].Time;
            Booster->Servo_Reload.Set_Target_Angle(Booster->reload_lift_angle); // 舵机抬起
        }
        /*----------------发现这里不延时也可以，只要直线电机到位即可-------------------*/
        // 2. 延时结束 -> 执行一次加法 -> 标记为状态2
        if (Status[Now_Status_Serial].Time > reload_servo_lift_time + 400  && reload_servo_flag_lift == 1) // 这里暂时使用延时 如果用总线舵机可以用串口接收数据回传 用fab比较误差值来判断
        {
            // 这里的代码只会在 flag 从 1 变 2 的瞬间执行一次

            Booster->Servo_Reload.Set_Target_Angle(Booster->reload_lift_angle); // 舵机抬起

            // 换弹角度电机再次转动80度
            // Booster->target_position_reload_angle += 80.0f * PI / 180.0f;
            Booster->target_position_reload_angle = GM6020_angle_ELUDE[dart_fired_count] -0.5f * PI / 180.0f; // 这里预设了每发射一次，换弹角度电机增加40度，可以根据实际情况调整
            // 切换到第2阶段：防止重复加，并开始电机控制
            reload_servo_flag_lift = 2;
        }
        // 3. 持续控制电机 (处于状态2时)
        if (reload_servo_flag_lift == 2)
        {
            Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);
        }

        // 以上的逻辑是：先让直线电机回到初始位置，再让舵机抬起，等待舵机抬起完成后再让角度电机转回初始位置

        // 4. 判断到位退出
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL 
        && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED 
        /*&& Referee_Allow_Shoot*/ 
        && reload_servo_flag_lift == 2 // 确保已经更新过目标了 
        && fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - Booster->target_position_reload_angle) < 0.0025f/* 已经到达了位置 */)
        {
            // // 离开前复位标志位，供下次使用
            reload_servo_flag_lift = 0;
            reload_servo_lift_time = 0;
            Set_Status(Reload_Control_Type_HOLD); // 进下一阶段
        }
        /*----------------------------------------------*/
        // // 直线电机后退返回初始位置
        // Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Reload_Linear.Set_Target_Radian(-Booster->init_position_reload_linear); // 注意：这个电机的正反转和位置定义相反，所以要取负值

        // // 1. 触发舵机动作
        // if (fabs(Booster->Get_Now_position_reload_linear() - (-Booster->init_position_reload_linear)) < 0.01f && reload_servo_flag_lift == 0) // 达到直线电机初始位置
        // {
        //     reload_servo_flag_lift = 1;
        //     reload_servo_lift_time = Status[Now_Status_Serial].Time;
        //     Booster->Servo_Reload.Set_Target_Angle(Booster->reload_lift_angle); // 舵机抬起
        // }
        // /*----------------发现这里不延时也可以，只要直线电机到位即可-------------------*/
        // // 2. 延时结束 -> 执行一次加法 -> 标记为状态2
        // if (Status[Now_Status_Serial].Time > reload_servo_lift_time + 300 && reload_servo_flag_lift == 1) // 这里暂时使用延时 如果用总线舵机可以用串口接收数据回传 用fab比较误差值来判断
        // {
        //     // 这里的代码只会在 flag 从 1 变 2 的瞬间执行一次

        //     // 换弹角度电机再次转动80度
        //     Booster->target_position_reload_angle += 80.0f * PI / 180.0f;
        //     // 切换到第2阶段：防止重复加，并开始电机控制
        //     reload_servo_flag_lift = 2;
        // }
        // // 3. 持续控制电机 (处于状态2时)
        // if (reload_servo_flag_lift == 2)
        // {
        //     Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        //     Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);
        // }

        // // 以上的逻辑是：先让直线电机回到初始位置，再让舵机抬起，等待舵机抬起完成后再让角度电机转回初始位置

        // // 4. 判断到位退出
        // if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL 
        // && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED 
        // && Referee_Allow_Shoot 
        // && reload_servo_flag_lift == 2 // 确保已经更新过目标了
        // && fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - Booster->target_position_reload_angle) < 0.0025f/* 已经到达了位置 */)
        // {
        //     // 离开前复位标志位，供下次使用
        //     reload_servo_flag_lift = 0;
        //     reload_servo_lift_time = 0;
        //     Set_Status(Reload_Control_Type_HOLD); // 进下一阶段
        // }
    }
    break;
    case (Reload_Control_Type_HOLD):
    {
        //先复位标志位，供下次使用
        reload_servo_flag_lift = 0;
        reload_servo_lift_time = 0;

        static uint32_t hold_enter_referee_rise_cnt = 0;// 记录进入 HOLD 状态时裁判系统允许发射的计数值
        if (Status[Now_Status_Serial].Time == 1) 
        {
            hold_enter_referee_rise_cnt = referee_allow_rise_cnt;
        }

        // 换弹完成状态设置为：完成
        Booster->Set_Reload_Status(Reload_Status_FINISHED);

        if (Status[Now_Status_Serial].Time == 1)
        {
            reload_count += 1; // 仅在进入 HOLD 状态的第一帧计数一次
            reload_done_token += 1; // 仅在进入 HOLD 首帧发放一次 token
        }

        // // 保持当前的位置不动
        // Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);
        // Booster->Motor_Reload_Linear.Set_Target_Radian(Booster->init_position_reload_linear);

        // 换弹完成后，等待下一次发射指令，同时监测发射动作的发生（通过发射数量的变化来判断）
        if (Booster->Get_Booster_Control_Type() == Booster_Control_Type_NORMAL  
        && Booster->Get_Shooting_Control_Type() == Shooting_Control_Type_SHOOTING_FINISHED 
        && dart_fired_count > last_dart_fired_count /* 如果当前的发射数量比上一次记录的大（说明发射了一发）*/
        && referee_allow_rise_cnt > hold_enter_referee_rise_cnt /*计数有新增*/ ) 
        {
            // 更新上一时刻的数量
            last_dart_fired_count = dart_fired_count;
            // 换弹状态设置为：换弹中...
            Booster->Set_Reload_Status(Reload_Status_RELOADING);
            // 进入 PUSHING 状态开始新一轮换弹
            Set_Status(Reload_Control_Type_PUSHING);
        }
    }
    break;
    }
    // case(Reload_Control_Type_Test):
    // {
    //     //--------
    // }
    // break;
    Reload_Control_Type = static_cast<Enum_Reload_Control_Type>(Now_Status_Serial);
}


// 测试参数
float Motor_L_test_P = 270.0f;
float Motor_L_test_I = 25.f;
float Motor_R_test_P = 320.0f;
float Motor_R_test_I = 28.f;

float Motor_Push_Angle_P_test = 4200.0f;
float Motor_Push_Angle_I_test = 0.0f;

float Motor_Pull_Omega_test_P = 900.0f;
float Motor_Pull_Omega_test_I = 145.0f;
float Motor_Pull_Angle_P_test = 2200.0f;
float Motor_Pull_Angle_I_test = 0.0f;

// 2006换弹直线电机
float Motor_Reload_C610_Omega_P_test = 600.0f;
float Motor_Reload_C610_Omega_I_test = 400.0f;
float Motor_Reload_C610_Anlge_P_test = 170.0f;
float Motor_Reload_C610_Anlge_I_test = 0.0f;

// 6020换弹角度电机
float Motor_Reload_6020_Omega_P_test = 925.0f;
float Motor_Reload_6020_Omega_I_test = 2400.0f;
float Motor_Reload_6020_Omega_D_test = 0.0f;
float Motor_Reload_6020_Anlge_P_test = 43.0f;
float Motor_Reload_6020_Anlge_I_test = 0.0f;
float Motor_Reload_6020_Anlge_D_test = 0.3f;


/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(9, 0);

    FSM_Reload.Booster = this;
    FSM_Reload.Init(6, 0);

    FSM_Push_Calibration.Booster = this;
    FSM_Push_Calibration.Init(6, 0);

    FSM_Pull_Calibration.Booster = this;
    FSM_Pull_Calibration.Init(6, 0);

    FSM_Reload_Linear_Calibration.Booster = this;
    FSM_Reload_Linear_Calibration.Init(6, 0);
    // 舵机
    Servo_Trigger.Init(&htim2, TIM_CHANNEL_1, 270);
    Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);

    Servo_Reload.Init(&htim2, TIM_CHANNEL_3, 270);
    Servo_Reload.Set_Target_Angle(reload_lift_angle); // 暂时没写

    // 拉力电机
    Motor_Pull.PID_Angle.Init(Motor_Pull_Angle_P_test, Motor_Pull_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 130.0f * PI);
    Motor_Pull.PID_Omega.Init(Motor_Pull_Omega_test_P, Motor_Pull_Omega_test_I, 0.0f, 0.0f, 2000, Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    // Push电机左
    Motor_Push_L.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_L.PID_Omega.Init(Motor_L_test_P, Motor_L_test_I, 0.0f, 0.0f, Motor_Push_L.Get_Output_Max() * 0.5f, Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // Push电机右
    Motor_Push_R.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_R.PID_Omega.Init(Motor_R_test_P, Motor_R_test_I, 0.0f, 0.0f, Motor_Push_R.Get_Output_Max() * 0.5f, Motor_Push_R.Get_Output_Max());
    Motor_Push_R.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 换弹电机直线
    Motor_Reload_Linear.PID_Angle.Init(Motor_Reload_C610_Anlge_P_test, Motor_Reload_C610_Anlge_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Reload_Linear.PID_Omega.Init(Motor_Reload_C610_Omega_P_test, Motor_Reload_C610_Omega_I_test, 0.0f, 0.0f, Motor_Reload_Linear.Get_Output_Max() * 0.5f, Motor_Reload_Linear.Get_Output_Max());
    Motor_Reload_Linear.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OMEGA);

    // 换弹电机角度
    Motor_Reload_Angle.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    Motor_Reload_Angle.PID_Angle.Init(Motor_Reload_6020_Anlge_P_test, Motor_Reload_6020_Anlge_I_test, Motor_Reload_6020_Anlge_D_test, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Reload_Angle.PID_Omega.Init(Motor_Reload_6020_Omega_P_test, Motor_Reload_6020_Omega_I_test, Motor_Reload_6020_Omega_D_test, 0.0f, Motor_Reload_Angle.Get_Output_Max() * 0.8f, Motor_Reload_Angle.Get_Output_Max(),0.0f,0.0f,0.0f);

    Set_Reload_Status(Reload_Status_FINISHED); // 初始化为换弹完成状态
}

/**
 * @brief 输出到电机
 *
 */
int testtnum = 0;
float test_position_b = 79.0f;
float test_b = 3.0f;
// float aaaaaaa = 22.0f;

void Class_Booster::Output()
{
    // Servo_Reload.Set_Target_Angle(aaaaaaa);
    // // 下面是测试代码，正式使用时请删除
    // if(testtnum == 0)
    // {
    //     // 换弹角度电机转动60度
    //     target_position_reload_angle += 40.0f * PI / 180.0f;

    //     Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    //     Motor_Reload_Angle.Set_Target_Radian(test_position_b * PI / 180.0f);
    //     // Motor_Reload_Angle.Set_Target_Radian(target_position_reload_angle);

    //     FSM_Reload.Set_Status(Reload_Control_Type_Test);

    //     testtnum = 0;
    // }

    //  // 设置电机控制模式（调试用）
    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

    // //角度环调参
    // Motor_Pull.PID_Angle.Set_K_P(Motor_Pull_Angle_P_test);
    // Motor_Pull.PID_Angle.Set_K_I(Motor_Pull_Angle_I_test);
    // Motor_Push_L.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
    // Motor_Push_R.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
    // Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
    // Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
    // Motor_Reload_Linear.PID_Angle.Set_K_P(Motor_Reload_C610_Anlge_P_test);
    // Motor_Reload_Linear.PID_Angle.Set_K_I(Motor_Reload_C610_Anlge_I_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_P(Motor_Reload_6020_Anlge_P_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_I(Motor_Reload_6020_Anlge_I_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_D(Motor_Reload_6020_Anlge_D_test);

    // //角度环测试
    // Motor_Pull.Set_Target_Radian(Target_test_b_pull);
    // Motor_Push_L.Set_Target_Radian(target_position_b);
    // Motor_Push_R.Set_Target_Radian(target_position_b);
    // Motor_Reload_Linear.Set_Target_Radian(target_position_b);
    // Motor_Reload_Angle.Set_Target_Angle(test_angle_reload_multi_turn);

    // //速度环调参
    // Motor_Pull.PID_Omega.Set_K_P(Motor_Pull_Omega_test_P);
    // Motor_Pull.PID_Omega.Set_K_I(Motor_Pull_Omega_test_I);
    // Motor_Push_L.PID_Omega.Set_K_P(Motor_L_test_P);
    // Motor_Push_R.PID_Omega.Set_K_P(Motor_R_test_P);
    // Motor_Push_L.PID_Omega.Set_K_I(Motor_L_test_I);
    // Motor_Push_R.PID_Omega.Set_K_I(Motor_R_test_I);
    // Motor_Reload_Linear.PID_Omega.Set_K_P(Motor_Reload_C610_Omega_P_test);
    // Motor_Reload_Linear.PID_Omega.Set_K_I(Motor_Reload_C610_Omega_I_test);
    // Motor_Reload_Angle.PID_Omega.Set_K_P(Motor_Reload_6020_Omega_P_test);
    // Motor_Reload_Angle.PID_Omega.Set_K_I(Motor_Reload_6020_Omega_I_test);

    // //速度环测试
    // Motor_Pull.Set_Target_Omega_Radian(test_b);
    // Motor_Pull.Set_Transform_Angle(Motor_Pull.Get_Now_Radian());
    // Motor_Push_L.Set_Target_Omega_Radian(test_b);
    // Motor_Push_L.Set_Transform_Angle(Motor_Push_L.Get_Now_Radian());
    // Motor_Push_R.Set_Target_Omega_Radian(test_b);
    // Motor_Push_R.Set_Transform_Angle(Motor_Push_R.Get_Now_Radian());
    // Motor_Reload_Linear.Set_Target_Omega_Radian(test_b);
    // Motor_Reload_Linear.Set_Transform_Angle(Motor_Reload_Linear.Get_Now_Radian());
    // Motor_Reload_Linear.Set_Target_Omega_Radian(test_b);
    // Motor_Reload_Angle.Set_Transform_Angle(Motor_Reload_Angle.Get_Now_Angle());
    // Motor_Reload_Angle.Set_Target_Omega_Radian(test_b);

    switch (Booster_Control_Type)
    {
    case (Booster_Control_Type_DISABLE):
    {
        // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        // Motor_Pull.Set_Target_Torque(0.f);
        // Motor_Push_L.Set_Target_Torque(0.f);
        // Motor_Push_R.Set_Target_Torque(0.f);
        // Motor_Reload_Linear.Set_Target_Torque(0.0f);
    }
    break;
    case (Booster_Control_Type_NORMAL): // 校准结束，进入正常控制状态（发射/换弹等）
    {
        // // 其实这里都可以不要 这个out_put没啥用 就调试的时候用用
        // // 东西都在状态机里面跑了
        // //  //-----------------------
        // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        // Motor_Push_L.Set_Target_Radian(test_position_b);
        // Motor_Push_R.Set_Target_Radian(test_position_b);

        // Motor_Reload_Linear.Set_Target_Radian(-test_b); // 注意：这个电机的正反转和位置定义相反，所以要取负值
        // Motor_Reload_Angle.Set_Target_Angle(init_position_reload_angle + 40.0f * PI / 180.0f );

    }
    break;
    }
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{
    //
    Update_Referee_Allow_Edge();

    PB3_GPIO = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET ? 1 : 0;
    PD7_GPIO = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0;

    //调试代码

    PE15_GPIO = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET ? 1 : 0;

    if(enable_booster_flag == 1)
    {
    // 拉力机数值更新
    Measured_Tension = TensionMeter.Get_Tension();

    // 皮筋校准
    FSM_Push_Calibration.Push_Calibration_TIM_Status_PeriodElapsedCallback();

    // // 拉力校准
    FSM_Pull_Calibration.Pull_Calibration_TIM_Status_PeriodElapsedCallback();

    // 直线电机校准
    FSM_Reload_Linear_Calibration.Linear_Calibration_TIM_Status_PeriodElapsedCallback();

    // // 换弹状态机
    FSM_Reload.Reload_TIM_Status_PeriodElapsedCallback();

    // 发射状态机
    FSM_Shooting.Shooting_TIM_Status_PeriodElapsedCallback();
    }
    else
    {
        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Reload_Angle.Set_DJI_Motor_Control_Method((DJI_Motor_Control_Method_TORQUE));

        Motor_Pull.Set_Target_Torque(0.f);
        Motor_Push_L.Set_Target_Torque(0.f);
        Motor_Push_R.Set_Target_Torque(0.f);
        Motor_Reload_Linear.Set_Target_Torque(0.0f);
        Motor_Reload_Angle.Set_Target_Torque(0.0f);
    }

    Output();

    // PID输出
    Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();
    Motor_Reload_Angle.TIM_PID_PeriodElapsedCallback();
    Motor_Reload_Linear.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
