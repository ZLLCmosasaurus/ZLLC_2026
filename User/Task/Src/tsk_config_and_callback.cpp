/**
 * @file tsk_config_and_callback.cpp
 * @author cjw by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 * @copyright ZLLC 2026
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_bsp-boarda.h"
#include "drv_tim.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_dmmotor.h"
#include "ita_chariot.h"
#include "dvc_imu.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include "iwdg.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = 0;
// 机器人控制对象
Class_Chariot chariot;

#ifdef MY_DEBUG
bool debug_test_enable = true;   // Debug模式下测试使能标志
uint32_t debug_test_counter = 0; // 测试计数器

/*轨迹计算测试用变量*/
float q_start[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.5f, 0.0f};
float trajectory_xyz[600][3]; // 轨迹点数组，测试用
float trajectory_rpy[3] = {0.0000f, -1.5000f, 0.0000f};
uint32_t delta_cnt = 0;     // DWT测用时的计数值
float delta_s = 0.0f;       // DWT测得的用时
float total_s = 0.0f;       // 完成一次完整计算总用时
uint16_t calculate_cnt = 0; // 计算次数，用于控制计算只进行一次
float x_calc;
float y_calc;
float z_calc;
float q_to_show[6];
float xyz_rpy_to_show[6];
float s_test;
/*测试用 测完删*/
float start_xyz[3];
float start_rpy[3];
#endif

#ifdef MOTOR_TEST_CHASSIS
uint32_t delta_cnt = 0;     // DWT测用时的计数值
float delta_s = 0.0f;       // DWT测得的用时
#endif

/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
#ifdef MOTOR_TEST_CHASSIS
        // case (0x201):
        // {
        //     chariot.Test_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
#else
    // 麦克纳姆轮组
    case (0x201):
    {
        //chariot.Chassis.Mecanum_Wheels[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        chariot.Force_Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        //chariot.Chassis.Mecanum_Wheels[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        chariot.Force_Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        //chariot.Chassis.Mecanum_Wheels[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        chariot.Force_Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        //chariot.Chassis.Mecanum_Wheels[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        chariot.Force_Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif

#ifdef AGV
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif

#ifdef OMNI_WHEEL
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif

    case (0x150):
    {
    }
    break;
    case (0x207):
    {
    }
    break;
    }
}
#endif
/**
 * @brief Chassis_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
#ifdef MOTOR_TEST_CHASSIS
    case (0x204):
    {
        chariot.Test_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#else
    case (0x201):
    {
        chariot.Chassis.Uplift_Motor[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Uplift_Motor[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Uplift_Motor[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Uplift_Motor[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0xA1):
    {
        chariot.Chassis.Track_Motor[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0xA2):
    {
        chariot.Chassis.Track_Motor[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif

#ifdef AGV
    case (0x205):
    {
        chariot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x207):
    {
        chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x208):
    {
        chariot.Chassis.Motor_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0xD1):
    {
        chariot.Chassis.Motor_Steer[0].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD2):
    {
        chariot.Chassis.Motor_Steer[1].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD3):
    {
        chariot.Chassis.Motor_Steer[2].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD4):
    {
        chariot.Chassis.Motor_Steer[3].MA600_Data_Process(CAN_RxMessage);
    }
    break;
#endif

    case (0x67): // 超电接收
    {
        chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x55):
    {
        chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    }
}
#endif

#ifdef CHASSIS
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x77): // 留给上板通讯
    {
        chariot.CAN_Chassis_Rx_Gimbal_Callback(CAN_RxMessage->Data);
        break;
    }
    case (0x95):
    {
        chariot.CAN_Chassis_Rx_Gimbal_Callback(CAN_RxMessage->Data);
        break;
    }
    }
}
#endif
/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0xA1): // J0 - DM4310 (Yaw)
    {
        chariot.Gimbal.Motor_DM_J0_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0xA2): // J1 - DM8009P (Pitch)
    {
        chariot.Gimbal.Motor_DM_J1_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0xA3): // J2 - 4340
    {
        chariot.Gimbal.Motor_DM_J2_Pitch_2.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0xA4): // J3 - 2325
    {
        chariot.Gimbal.Motor_DM_J3_Roll.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif

/**
 * @brief Gimbal_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x205):
    {
        chariot.Gimbal.Motor_6020_J5_Roll_2.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x206):
    {
        chariot.Gimbal.Motor_C610_Gripper.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0xA5): // J4 - 4340
    {
        chariot.Gimbal.Motor_DM_J4_Pitch_3.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif

#ifdef GIMBAL
void Gimbal_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    }
}
#endif
/**
 * @brief SPI5回调函数
 *
 * @param Tx_Buffer SPI5发送的消息
 * @param Rx_Buffer SPI5接收的消息
 * @param Length 长度
 */
// void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//     if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//     {
//         boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//     }
// }

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
}

/**
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART9收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}
#elifdef CHASSIS_TEST
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    chariot.Chassis_Test_Control();
}
#endif
/**
 * @brief UART9遥控器回调函数
 *
 * @param Buffer UART9收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void VT13_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.VT13.VT13_UART_RxCpltCallback(Buffer);

    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}
#endif

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
#ifdef CHASSIS
void Referee_UART10_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer, Length);
}
#endif
/**
 * @brief UART1超电回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#if defined CHASSIS
void SuperCAP_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    //chariot.Chassis.Supercap.UART_RxCpltCallback(Buffer);
    chariot.Force_Chassis.Supercap.UART_RxCpltCallback(Buffer);
    /*紫板子功率计*/
    int16_t tmp_power;
    memcpy(&tmp_power,&Buffer[6],sizeof(int16_t));

    chariot.Force_Chassis.Supercap.Set_Now_Power((float)tmp_power/75.0f);
}
#endif
/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */
#ifdef GIMBAL
void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    chariot.MiniPC.USB_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief UART MiniPC回调函数
 *
 * @param Buffer UART收到的消息
 *
 * @param Length 长度
 */
#ifdef GIMBAL
void MiniPC_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.MiniPC.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief TIM4任务回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Task100us_TIM4_Callback()
{
#ifdef CHASSIS

#elif defined(GIMBAL)
    // 单给IMU消息开的定时器 ims
    chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
    static int mod100 = 0;
    mod100++;
    if (mod100 = 100)
    {
#ifdef defined(USE_DR16)
#ifdef DEBUG
        if (chariot.DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            chariot.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            chariot.Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            chariot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#else
        if (CAN3_Chassis_Rx_Data_A.game_process != 4)
        {
            if (chariot.DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                chariot.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                chariot.Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                chariot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }
        }
#endif
#elif defined(USE_VT13)
#ifdef DEBUG
        if (chariot.VT13.Get_VT13_Status() == VT13_Status_DISABLE)
        {
            chariot.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            chariot.Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            chariot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
        }
#else
        if (CAN3_Chassis_Rx_Data_A.game_process != 4)
        {
            if (chariot.VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                chariot.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                chariot.Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                chariot.Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            }
        }
#endif

#endif
        mod100 = 0;
    }

#endif
}

/**
 * @brief TIM5任务回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Task1ms_TIM5_Callback()
{
    init_finished++;
    if (init_finished > 2000)
        start_flag = 1;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/

    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
    HAL_IWDG_Refresh(&hiwdg1);

    /****************************** 交互层回调函数 1ms *****************************************/
    if (start_flag == 1)
    {
#ifdef GIMBAL

// ============ 电机测试模式：绕过遥控器检测 ============
#ifdef MY_DEBUG

        if (debug_test_enable)
        {
            // 强制使能云台
            chariot.Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

            chariot.Gimbal.Motor_DM_J0_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            chariot.Gimbal.Motor_DM_J1_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            chariot.Gimbal.Motor_DM_J2_Pitch_2.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            chariot.Gimbal.Motor_DM_J4_Pitch_3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        }
        else
#endif
        {
            // 正常模式：需要遥控器
            chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
        }
#endif

#ifdef CHASSIS_TEST
        chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
#endif

        chariot.TIM_Calculate_PeriodElapsedCallback();

        /****************************** 驱动层回调函数 1ms *****************************************/
        // 统一打包发送
        TIM_CAN_PeriodElapsedCallback();

        static int mod5 = 0, mod100 = 0, mod68 = 0;
        mod5++;
        mod100++;
        mod68++;
        if (mod5 == 4)
        {
            chariot.Chassis.Track_Motor[0].TIM_Process_PeriodElapsedCallback();
            chariot.Chassis.Track_Motor[1].TIM_Process_PeriodElapsedCallback();
        }
        if (mod5 == 5)
        {
            // 上位机
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);

            // 串口统一发送
            TIM_UART_PeriodElapsedCallback();
            mod5 = 0;
        }
        if (mod100 == 100)
        {
#ifdef CHASSIS
            // 裁判系统发送
            chariot.Referee.TIM_UART_Tx_PeriodElapsedCallback();
#endif
            mod100 = 0;
        }
        if (mod68 == 68)
        {
#ifdef CHASSIS
// 裁判系统发送
#endif
            mod68 = 0;
        }
    }
#ifdef MY_DEBUG
    /*测试大王奉命接管此区*/
    else // 充分利用启动前的2000ms进行轨迹计算
    {
        chariot.Gimbal.Trajectory_Tracer.dh_model.Fkine(q_start, start_xyz, start_rpy);
        // 将计算出的真实姿态，赋值给 trajectory_rpy
        trajectory_rpy[0] = start_rpy[2]; // Yaw
        trajectory_rpy[1] = start_rpy[1]; // Pitch
        trajectory_rpy[2] = start_rpy[0]; // Roll

        if (calculate_cnt < 600)
        // 前600次用于计算轨迹点，从0到599一共调用600次计算函数，可以把所有点都算完
        {
            chariot.Gimbal.Trajectory_Tracer.Trajectory_Generator(q_start, 1, trajectory_xyz); // 计算轨迹点
            x_calc = trajectory_xyz[calculate_cnt][0];
            y_calc = trajectory_xyz[calculate_cnt][1];
            z_calc = trajectory_xyz[calculate_cnt][2];
        }
        else if (calculate_cnt < 1200)
        {
            chariot.Gimbal.valid_solution_cnt = chariot.Gimbal.Trajectory_Tracer.Trajectory_Ikine(q_start, start_rpy, trajectory_xyz, chariot.Gimbal.q_solution, chariot.Gimbal.is_low_speed); // 逆运动学求解，测试用

            for (int i = 0; i < 6; i++)
            {
                q_to_show[i] = chariot.Gimbal.q_solution[calculate_cnt - 600][i];
                // show_FK_result(q_to_show, xyz_rpy_to_show); // 计算正运动学，得到末端xyzrpy，用于显示验证
                if (i < 3)
                {
                    start_xyz[i] = trajectory_xyz[calculate_cnt - 600][i];
                    start_rpy[i] = trajectory_rpy[i];
                }
            }
        }
        else
        {
            // 1200次之后，取点和逆解都算完了，停止计算
            calculate_cnt = 1200;
        }

        calculate_cnt++;
    }
/*测试大王奉命接管此区*/
#endif
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{

    DWT_Init(480);

/********************************** 驱动层初始化 **********************************/
#ifdef CHASSIS

    // 集中总线can1/can2
    CAN_Init(&hfdcan1, Chassis_Device_CAN1_Callback);
    CAN_Init(&hfdcan2, Chassis_Device_CAN2_Callback);
    CAN_Init(&hfdcan3, Chassis_Device_CAN3_Callback);

    // 裁判系统
    UART_Init(&huart10, Referee_UART10_Callback, 128); // 并未使用环形队列 尽量给长范围增加检索时间 减少丢包
    UART_Init(&huart1, SuperCAP_UART1_Callback, 32);

    SPI_Init(&hspi2, Device_SPI2_Callback);
#ifdef POWER_LIMIT

#endif

#endif

#ifdef GIMBAL

    // 集中总线can1/can2
    CAN_Init(&hfdcan1, Gimbal_Device_CAN1_Callback);
    CAN_Init(&hfdcan2, Gimbal_Device_CAN2_Callback);
    CAN_Init(&hfdcan3, Gimbal_Device_CAN3_Callback);

    // c板陀螺仪spi外设
    SPI_Init(&hspi2, Device_SPI2_Callback);
// 磁力计iic外设
// IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    //达妙无磁力计
// 遥控器接收
#ifdef USE_DR16
    UART_Init(&huart5, DR16_UART5_Callback, 18);
    // UART_Init(&huart6, Image_UART6_Callback, 40);
#elif defined(USE_VT13)
    UART_Init(&huart9, VT13_UART_Callback, 30);
#endif
    // 上位机USB
    USB_Init(&MiniPC_USB_Manage_Object, MiniPC_USB_Callback);
    // 上位机串口
    UART_Init(&huart8, MiniPC_UART_Callback, 56);

#endif

#ifdef CHASSIS_TEST
    UART_Init(&huart5, DR16_UART5_Callback, 18);
#endif

#ifdef MOTOR_TEST_CHASSIS
    chariot.Init_Motor_Test_Chassis();
#endif

    // 定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    // 遥控器死区给0.3f
    chariot.Init(0.3f);

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
#ifdef GIMBAL
    // // 解析法求解逆运动学
    // ikine_pieper_solutions(chariot.Gimbal.target_pos, chariot.Gimbal.target_rpy, &chariot.Gimbal.solutions[0]);
    // chariot.Gimbal.valid_IK_cnt = solution_filter(&chariot.Gimbal.solutions[0], chariot.Gimbal.valid_solution);
    // if (chariot.Gimbal.valid_IK_cnt > 0)
    // {
    //     float *current_angle = get_now_motor_angles(&chariot.Gimbal);
    //     chariot.Gimbal.solution_index = get_best_solution_index(chariot.Gimbal.solutions, chariot.Gimbal.valid_solution, current_angle);
    //     // 将选择的合法解转为电机控制角度
    //     for (int i = 0; i < 6; i++)
    //     {
    //         chariot.Gimbal.model_result[i] = chariot.Gimbal.solutions[chariot.Gimbal.solution_index][i][0];
    //     }
    // }        // 计算 q_start 对应的真实 XYZ 和 RPY

#endif
#ifdef CHASSIS

#endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
