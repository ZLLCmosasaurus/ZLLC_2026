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
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

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

        case (0x208):
        {
            chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):
        {
            chariot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x207):
        {
            chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x205):
        {
            chariot.Chassis.Motor_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x67)://超电接收
        {
            chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x55):
        {
            chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        //case(0x151):
        //break;//留给磁编舵

    }
}
#endif

#ifdef CHASSIS
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x77)://留给上板通讯
        {
            chariot.CAN_Chassis_Rx_Gimbal_Callback(CAN_RxMessage->Data);//这里其实如果要统一的话可以去掉的，因为已经绑定了CAN3了，可能有原因
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
 * @brief Gimbal_CAN1回调函数.按照结构划分，在云台上部存在MiniPC、两个摩擦轮以及Pitch电机
 * @brief MiniPC直接绑定CAN1通道，不按照形参顺序走 
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
	    case(0xA1):
		{
			chariot.MiniPC.CAN_RxCpltCallback();
		}
		break;
        case(0x021):
        {
            chariot.Booster.Motor_Friction_Right.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
		break;
		case(0x202):
		{
			chariot.Booster.Motor_Friction_Left.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
		break;
		case(0x205):
        {
            chariot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
		}
        break;
	}
}
#endif

/**
 * @brief Gimbal_CAN2回调函数
 * @brief 在中间，有yaw电机(给gimbal和chassis)和波弹盘电机
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case(0x206):
        {
            chariot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case(0x201):
        {
            chariot.Booster.Motor_Driver.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
    }
		
}
#endif

/**
 * @brief Gimbal_CAN3回调函数
 * @brief 底盘和云台的交互层，ita_chariot.h中直接绑定了CAN3
 * @param CAN_RxMessage CAN3收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage){
    switch (CAN_RxMessage->Header.Identifier)
    {
        case(0x188)://与下板进行通讯
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();//利用can通信，让云台接收底盘的回调信息
        }
        break;
        case(0x178):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case(0x199):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;    
        case(0x198):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case(0x197):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;        
        case(0x196):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
        }
        break;
        case(0x191):
        {
            chariot.CAN_Gimbal_Rx_Chassis_Callback();
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
 * @brief UART9遥控器回调函数
 *
 * @param Buffer UART9收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void VT13_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.VT13.VT13_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
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
    if(mod100 = 100)
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
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
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
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
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
    if(init_finished>2000)
    start_flag=1;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/
    
    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();
    HAL_IWDG_Refresh(&hiwdg1);//252ms进行一次喂狗

    /****************************** 交互层回调函数 1ms *****************************************/
    if(start_flag==1)
    {
        #ifdef GIMBAL
        chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
        #endif
        chariot.TIM_Calculate_PeriodElapsedCallback();
        
    /****************************** 驱动层回调函数 1ms *****************************************/ 
        //统一打包发送
        TIM_CAN_PeriodElapsedCallback();
        
        static int mod5 = 0,mod100 = 0,mod68 = 0;
        mod5++;
        mod100++;
        mod68++;
        if (mod5 == 5)
        {
            // 上位机
            TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);

            // 串口统一发送
            TIM_UART_PeriodElapsedCallback();
            mod5 = 0;
        }	 
        if(mod100 == 100)
        {
            #ifdef CHASSIS
            // 裁判系统发送
            chariot.Referee.TIM_UART_Tx_PeriodElapsedCallback();
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

        //裁判系统
        UART_Init(&huart10, Referee_UART10_Callback, 128);//并未使用环形队列 尽量给长范围增加检索时间 减少丢包

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
		UART_Init(&huart6, Image_UART6_Callback, 40);
        #elif defined(USE_VT13)
        UART_Init(&huart9, VT13_UART_Callback, 30);
        #endif
        //上位机USB
        USB_Init(&MiniPC_USB_Manage_Object,MiniPC_USB_Callback);
        //上位机串口
        UART_Init(&huart8, MiniPC_UART_Callback, 56);

    #endif

    //定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    //设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    chariot.Init();

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
    float now_angle_yaw = chariot.Gimbal.Motor_Yaw.Get_True_Angle_Yaw();
    float target_angle_yaw = chariot.Gimbal.MiniPC->Get_Rx_Yaw_Angle();
//    // 如果是自瞄开启并且距离装甲板的瞄准弧度小于0.1m
//    if (chariot.Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC &&
//        (chariot.Gimbal.MiniPC->Get_Distance() * abs(now_angle_yaw - target_angle_yaw) / 180.0f * PI) < 0.1)
//    {
//        chariot.MiniPC_Aim_Status = MinPC_Aim_Status_ENABLE;
//    }
//    else
//    {
//        chariot.MiniPC_Aim_Status = MinPC_Aim_Status_DISABLE;
	//  }//不同车的逻辑
#endif
#ifdef CHASSIS
    if (start_flag == 1)
    {
        static float freq;
        static uint32_t time_s;
        freq = 1 / DWT_GetDeltaT(&time_s);

        JudgeReceiveData.robot_id = chariot.Referee.Get_ID();
        JudgeReceiveData.Pitch_Angle = chariot.Gimbal_Tx_Pitch_Angle; // pitch角度
        JudgeReceiveData.Bullet_Status = chariot.Bulletcap_Status;    // 弹舱
        JudgeReceiveData.Fric_Status = chariot.Fric_Status;           // 摩擦轮
        JudgeReceiveData.Minipc_Status = chariot.MiniPC_Status;       // 自瞄是否离线
        JudgeReceiveData.Booster_User_Control_Type = chariot.Booster_User_Control_Type;
        // JudgeReceiveData.Supercap_Energy = chariot.Chassis.Supercap.Get_Stored_Energy();    // 超级电容储能
        JudgeReceiveData.Supercap_Voltage = chariot.Chassis.Supercap.Get_Now_Voltage() / 100.0f; // 超级电容容量
        JudgeReceiveData.Chassis_Control_Type = chariot.Chassis.Get_Chassis_Control_Type();      // 底盘控制模式
        JudgeReceiveData.Supercap_State = chariot.Sprint_Status;
        JudgeReceiveData.booster_fric_omega_left = chariot.Booster_fric_omega_left; // 左摩擦轮速度; // 左摩擦轮速度
        JudgeReceiveData.booster_fric_omega_right = chariot.Booster_fric_omega_right;
				JudgeReceiveData.Booster_bullet_num=chariot.Booster_bullet_num-chariot.Booster_bullet_num_before;
        JudgeReceiveData.Minipc_Mode = chariot.MiniPC_Type;
				JudgeReceiveData.Antispin_Type=chariot.Antispin_Type;
        if (chariot.Referee_UI_Refresh_Status == Referee_UI_Refresh_Status_ENABLE)
            Init_Cnt = 10;
    }

#endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
