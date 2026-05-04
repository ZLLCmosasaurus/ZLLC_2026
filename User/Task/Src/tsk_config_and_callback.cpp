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
#include "dvc_GraphicsSendTask.h"
#include "drv_can.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t last_cnt_1 ,last_cnt_2;
float dt_receive1,dt_receive2;
uint32_t init_finished =0 ;
bool start_flag=0;
//机器人控制对象
Class_Chariot chariot;

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
        #ifdef TRACK_LEG
        case (0x201):
        {
            chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            chariot.Force_Control_Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
            chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            chariot.Force_Control_Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
            chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
            chariot.Force_Control_Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x204):
        {
            chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
            chariot.Force_Control_Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        // case (0x67): //超电
        // {
        //     chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        //     chariot.Force_Control_Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
        // case (0x55):
        // {
        //     chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
        #endif


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
    
    #ifdef TRACK_LEG
        case(0xFB):
        {
            chariot.Chassis.Motor_Joint[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case(0x22):
        {
            //chariot.Chassis.Motor_Joint[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
					  chariot.Chassis.Motor_Leg[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case(0x201):
        {
            chariot.Force_Control_Chassis.Motor_Track[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case(0x202):
        {
            chariot.Force_Control_Chassis.Motor_Track[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x203):
        {
            chariot.Chassis.Motor_Guider[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x204):
        {
            chariot.Chassis.Motor_Guider[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x67):
        {
            chariot.Force_Control_Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    #endif
    }
}
#endif

#ifdef CHASSIS
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {

        case (0x52)://留给上板通讯
        {
            chariot.CAN_Chassis_Rx_Gimbal_Callback();
            break;
        }
        case (0x78):
        {
            chariot.CAN_Chassis_Rx_Gimbal_Callback_1();
            break;
        }
        case (0x13):
        {
            chariot.Motor_Yaw_DM4310.CAN_RxCpltCallback(CAN_RxMessage->Data);
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
uint32_t cnt_last = 0;
float dt;
uint32_t fire_flag,booster_flag;
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0xA1):
    {
        
        chariot.MiniPC.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    #ifdef Single_Friction
    case (0x201):
    {
        chariot.Booster.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Booster.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    #endif
    #ifdef Double_Friction
    case (0x201):
    {
        chariot.Booster.Fric[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Booster.Fric[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
	break;
    case (0x203):
    {
        chariot.Booster.Fric[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Booster.Fric[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    #endif
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
    case (0x11):
    {
        // chariot.Gimbal.dmIMU.IMU_UpdateData(CAN_RxMessage->Data);
    }
    break;
    case (0x14):
    {
        chariot.Gimbal.Motor_Pitch_DM4310.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    
    }
		
}
#endif

#ifdef GIMBAL
void Gimbal_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x51): //留给下板通讯
    {
        dt_receive1 = DWT_GetDeltaT(&last_cnt_1);
        chariot.CAN_Gimbal_Rx_Chassis_Callback();
    }
    break;
    case (0x20):
    {
        dt_receive2 = DWT_GetDeltaT(&last_cnt_2);
        chariot.CAN_Gimbal_Rx_Chassis_Callback_1();
    }
    break;
    case (0x13):
    {
        chariot.Gimbal.Motor_Yaw_DM4310.CAN_RxCpltCallback(CAN_RxMessage->Data);
        // chariot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Booster.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
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
//void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//    if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//    {
//        boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//    }
//}

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
    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}
#endif

/**
 * @brief UART1遥控器回调函数--图传
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void VT13_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.VT13.VT13_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    if (*(Buffer + 0) == 0xA9 && *(Buffer + 1) == 0x53)
    {
        chariot.TIM_Control_Callback();
    }
}
#endif

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t* Tx_Buffer, uint8_t* Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
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
    chariot.Referee.UART_RxCpltCallback(Buffer,Length);
}
#endif
#ifdef GIMBAL
void Referee_UART10_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer,Length);
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
    chariot.Chassis.Supercap.UART_RxCpltCallback(Buffer);
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
    static float freq;
    static uint32_t time_s;
    freq = 1 / DWT_GetDeltaT(&time_s);
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
    // chariot.MiniPC.UART_RxCpltCallback(Buffer);
}
#endif
/**
 * @brief TIM4任务回调函数
 *
 */
uint32_t a =0;
uint32_t b =0;
uint32_t Last_cnt3 = 0;
//float Dt3 = 0;
void Task100us_TIM4_Callback()
{
    #ifdef CHASSIS
    
    static uint16_t Referee_Sand_Cnt = 0;
    if (Referee_Sand_Cnt % 50 == 1)
    {
        Referee_Sand_Cnt = 0;
    }

    Referee_Sand_Cnt++;
    //Imu读取任务
    //chariot.Force_Control_Chassis.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
    // chariot.Chassis.BoardDM_BMI.TIM_Calculate_PeriodElapsedCallback();
    #elif defined(GIMBAL)
    dt = DWT_GetDeltaT(&cnt_last);
        // 单给IMU消息开的定时器 ims
        chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
        static uint8_t mod2 = 0;  
        mod2++;
        if(mod2%2 == 0)
        {
            // chariot.Gimbal.dmIMU.IMU_RequestData(&hfdcan2,0x01,2);
        }
        else
        {
            // chariot.Gimbal.dmIMU.IMU_RequestData(&hfdcan2,0x01,3);
        }

    static int mod100 = 0;
    mod100++;
    if(mod100 = 100)
    {
        mod100 = 0;
    }

    #endif
}



/**
 * @brief TIM5任务回调函数
 *
 */

void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished>2000)
    start_flag=1;
    

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
   // chariot.TIM_Unline_Protect_PeriodElapsedCallback();
    // HAL_IWDG_Refresh(&hiwdg1);

    /****************************** 交互层回调函数 1ms *****************************************/
    if(start_flag==1)
    {
        #ifdef GIMBAL
        #ifdef USE_DR16
        chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
        #endif
        #ifdef USE_VT13
        chariot.FSM_Alive_Control_VT13.Reload_TIM_Status_PeriodElapsedCallback();
        #endif
        #endif
        #ifdef CHASSIS

        #endif
       // __disable_irq();
        chariot.TIM_Calculate_PeriodElapsedCallback();
      //  __enable_irq();
        
    /****************************** 驱动层回调函数 1ms *****************************************/ 
        //统一打包发送
        TIM_CAN_PeriodElapsedCallback();
        buzzer_taskScheduler(&buzzer);
        #ifdef GIMBAL
        if(chariot.DR16.Get_Right_Switch()==DR16_Switch_Status_TRIG_MIDDLE_DOWN)
        {
            chariot.Flag_Message ++;
        }   
        #endif     

        static int mod5 = 0,mod100 = 0,mod68 = 0;
        mod5++;
        mod100++;
        mod68++;
        if (mod5 == 5)
        {
            // 上位机
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);

            // 串口统一发送
            // TIM_UART_PeriodElapsedCallback();
            mod5 = 0;
        }	 
        if(mod100 == 100)
        {
            #ifdef CHASSIS
            // 裁判系统发送
            // chariot.Referee.TIM_UART_Tx_PeriodElapsedCallback();
            #endif
            mod100 = 0;
        }
        if(mod68 == 68)
        {
            #ifdef CHASSIS
            // 裁判系统发送
            #endif
            mod68 = 0;
        }

    }
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

        //集中总线can1/can2
        CAN_Init(&hfdcan1, Chassis_Device_CAN1_Callback);
        CAN_Init(&hfdcan2, Chassis_Device_CAN2_Callback);
        CAN_Init(&hfdcan3, Chassis_Device_CAN3_Callback);
        //陀螺仪spi外设
        SPI_Init(&hspi2,Device_SPI2_Callback);
        //裁判系统
        UART_Init(&huart10, Referee_UART10_Callback, 128);//并未使用环形队列 尽量给长范围增加检索时间 减少丢包

        //功率计
        // UART_Init(&huart1, Power_Cale_UART_Callback, 8+1);
        #ifdef POWER_LIMIT


        #endif

    #endif

    #ifdef GIMBAL

        //集中总线can1/can2
        CAN_Init(&hfdcan1, Gimbal_Device_CAN1_Callback);
        CAN_Init(&hfdcan2, Gimbal_Device_CAN2_Callback);
        CAN_Init(&hfdcan3, Gimbal_Device_CAN3_Callback);

        //c板陀螺仪spi外设
        SPI_Init(&hspi2,Device_SPI2_Callback);
        //磁力计iic外设
        //IIC_Init(&hi2c3, Ist8310_IIC3_Callback);    //达妙无磁力计
        //遥控器接收
        #ifdef USE_DR16
        UART_Init(&huart5, DR16_UART5_Callback, 18);
        #endif
        #ifdef USE_VT13
        //图传
        UART_Init(&huart1, VT13_UART_Callback, 60);
        #endif
        //上位机USB
        USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);
        //上位机串口
        UART_Init(&huart8, MiniPC_UART_Callback, 56);

        //裁判系统
        // UART_Init(&huart10, Referee_UART10_Callback, 128);

    #endif

    //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
    __disable_irq();    
    chariot.Init();
    __enable_irq();
    
	buzzer_setTask(&buzzer, BUZZER_MARIO_SIMPLE_PRIORITY);

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

/**
 * @brief 前台循环任务
 *
 */
 extern "C" void Task_Loop()
{
    #ifdef GIMBAL
    
    #endif
    #ifdef CHASSIS
    if (start_flag == 1)
    {
        GraphicSendtask();
        static float freq;
        static uint32_t time_s;
        freq = 1 / DWT_GetDeltaT(&time_s);

        JudgeReceiveData.robot_id = chariot.Referee.Get_ID(); //Robot ID
        JudgeReceiveData.Bullet_Status = chariot.Bulletcap_Status;    // 弹舱
        JudgeReceiveData.Fric_Status = chariot.Fric_Status;           // 摩擦轮
        JudgeReceiveData.Minipc_Status = chariot.MiniPC_Status;       // 自瞄是否离线
        JudgeReceiveData.Supercap_Voltage =  chariot.Force_Control_Chassis.Supercap.Get_Supercap_Charge_Percentage() / 100.0f; // 超级电容容量
        JudgeReceiveData.Chassis_Control_Type = chariot.Chassis.Get_Chassis_Control_Type();      // 底盘控制模式
        JudgeReceiveData.Supercap_State = chariot.Sprint_Status;
        JudgeReceiveData.booster_fric_omega_left = chariot.Booster_fric_omega_left; // 左摩擦轮速度; 
        JudgeReceiveData.booster_fric_omega_right = chariot.Booster_fric_omega_right; // 右摩擦轮速度
		JudgeReceiveData.Booster_bullet_num = chariot.Booster_bullet_num-chariot.Booster_bullet_num_before;
        JudgeReceiveData.MiniPC_Aim_Status = chariot.Aim_Status;      // 自瞄是否控制打弹
		JudgeReceiveData.Minipc_Mode = chariot.MiniPC_Mode; // 上位机模式
        if (chariot.Referee_UI_Refresh_Status == Referee_UI_Refresh_Status_ENABLE)
            Init_Cnt = 10;
    }
    #endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
