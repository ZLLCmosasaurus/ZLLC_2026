/**
 * @file dvc_LKmotor.h
 * @author lez
 * @brief lk电机配置与操作
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef DVC_LKMOTOR_H
#define DVC_LKMOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
	* 	@brief 瓴控电机状态
 *
 */
enum Enum_LK_Motor_Status
{
    LK_Motor_Status_DISABLE = 0,
    LK_Motor_Status_ENABLE,
};

/**
* @brief 瓴控电机帧ID
 *
 */
enum Enum_LK_Motor_ID : uint8_t
{
    LK_Motor_ID_UNDEFINED = 0,
    LK_Motor_ID_0x141,
    LK_Motor_ID_0x142,
    LK_Motor_ID_0x143,
    LK_Motor_ID_0x144,
    LK_Motor_ID_0x145,
    LK_Motor_ID_0x146,
    LK_Motor_ID_0x147,
    LK_Motor_ID_0x148,
};

/**
* @brief  瓴控电机控制状态
 *
 */
enum Enum_LK_Motor_Control_Status
{
    LK_Motor_Control_Status_DISABLE = 0,
    LK_Motor_Control_Status_ENABLE,
};

/**
		命令报文标识符：0x140 + ID(1~32) 
		回复报文标识符：0x180 + ID(1~32) 
 * @brief     can发送的数据帧控制ID，每一个数据帧的第一个字节，进行命令控制
 * @brief    电机在收到命令后回复主机。电机回复数据和当前发送数据一样，数据帧第一个字节和控制命令一样，其余参考数据手册。瓴控电机是一发一收模式
 */
enum Enum_LK_Motor_Control_ID : uint8_t
{
		LK_Motor_Control_Read_Status=0x9A,   //该命令读取当前电机的温度、电压和错误状态标志 
		LK_Motor_Control_Shut_Down = 0x80,   //将电机从开启状态（上电后默认状态）切换到关闭状态
        LK_Motor_Control_Stop = 0x81, //停止电机，但不清除电机运行状态
        LK_Motor_Control_Run = 0x88,//将电机从关闭状态切换到开启状态
		LK_Motor_Control_Torque = 0xA1,//转矩闭环控制，主机发送该命令以控制电机的转矩电流输出，控制值iqControl为int16_t类型，数值范围-2048~ 2048(仅在MF、MH、MG上可用)
		LK_Motor_Control_Open_Loop=0xA0,//仅对MS电机生效，此模式输出开环电压，主机发送该命令以控制输出到电机的开环电压，控制值powerControl为int16_t类型，数值范围-850~850
		LK_Motor_Control_Omega=0xA2,//速度环主机发送该命令以控制电机的速度， 同时带有力矩限制。控制值speedControl为int32_t类型，对应实际转速为0.01dps/LSB；控制值iqControl为int16_t类型，数值范围-2048~ 2048
		LK_Motor_Control_Multi_Location=0xA3,//多圈位置闭环控制。主机发送该命令以控制电机的位置（多圈角度）。控制值angleControl为int32_t类型，对应实际位置为0.01degree/LSB，即36000代表360°，电机转动方向由目标位置和当前位置的差值决定。
		LK_Motor_Control_Multi_Location_And_Speed_Limit=0xA4,  //跟0xA3一样，只不过增加了到达目标角度的速度限制
		LK_Motor_Control_Single_Location=0xA5,//单圈位置闭环控制控制值  控制值angleControl 为uint32_t 类型，对应实际位置为0.01degree/LSB，即36000代表360°。
		LK_Motor_Control_Single_Location_And_Speed_Limit=0xA6,//单圈位置闭环，并进行速度限制（由上位机软件进行限制）
        Lk_Motor_Control_Delta_Location=0xA7,//增量位置闭环控制   int32_t 类型，对应实际位置为0.01degree/LSB，即36000 代表360°
        LK_Motor_Control_Delta_Location_And_Speed_Limit=0xA8//增量位置闭环控制，并增加了速度限制（上位机调参软件限制）
};

/**
 * @brief    瓴控电机控制的PID模式
 *
 */
enum Enum_LK_Motor_Control_Method
{
    LK_Motor_Control_Method_IMU_ANGLE = 0,  //惯性传感器的角度环
    LK_Motor_Control_Method_IMU_OMEGA,     //惯性传感器的角速度环
	LK_Motor_Control_Method_ANGLE,         //霍尔(磁编)角度环
    LK_Motor_Control_Method_OMEGA,         //霍尔(磁编)角速度环
    LK_Motor_Control_Method_TORQUE,        //霍尔(磁编)力矩环
	LK_Motor_Control_Method_OpenLoop        //无传感器 直接开环，一般不会用到
};

/**
* @brief   瓴控电机的can通信协议的数据包格式
 *
 */
struct Struct_LK_Motor_CAN_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;   //控制命令，数据帧第一个字节
	uint8_t Temperature_Centigrade;     // 回传电机温度
    uint16_t Current_Reverse;			//转矩电流  ，对MS系列是：输出功率
	uint16_t Omega_Reverse;				//电机速度
    uint16_t Encoder_Reverse;            //编码器位置
} __attribute__((packed));

/**
 * @brief      can数据回传后经过data_process处理后得到的数据包
 *
 */
struct Struct_LK_Motor_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID; //控制命令
	float Now_Angle;  				//当前角度，以°为单位
    float Now_Radian;  				//当前角度，以rad为单位		
	float Now_Omega_Angle;  		//当前角速度，从rpm转为°/s
	float Now_Omega_Radian;  	    //当前角速度，从rpm转为rad/s	
    float Now_Current;  			//当前电流， 对MS系列是输出功率·
    float Now_Temperature; 			//当前温度
    uint16_t Pre_Encoder; 
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
	* @brief LK电机类
 * 
 * 
 * 
 *
 */
class Class_LK_Motor
{
public:
    // 角度环
    Class_PID PID_Angle;
    // 角速度环
    Class_PID PID_Omega;
    // 力矩环
    Class_PID PID_Torque;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID, float __Omega_Max=200, int32_t __Position_Offset = 0, float __Current_Max = 33.0f ,Enum_LK_Motor_Control_Method __Control_Method = LK_Motor_Control_Method_ANGLE,Enum_LK_Motor_Control_ID __Control_ID=LK_Motor_Control_Omega);

		//Omega_Max是由调参助手决定的,这里只是定义但并没有使用
    inline Enum_LK_Motor_Control_Status Get_LK_Motor_Control_Status();
    inline Enum_LK_Motor_Status Get_LK_Motor_Status();
    inline float Get_Output_Max();
    inline float Get_Now_Angle();
    inline float Get_Now_Radian();
    inline float Get_Now_Omega_Angle();
    inline float Get_Now_Omega_Radian();
    inline float Get_Now_Torque();
    inline float Get_Now_Temperature();
    inline Enum_LK_Motor_Control_Method Get_LK_Motor_Control_Method();
	inline Enum_LK_Motor_Control_ID Get_LK_Motor_Control_ID();
    inline float Get_IMU_K_P();
    inline float Get_IMU_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Radian();
    inline float Get_Target_Omega_Angle();
    inline float Get_Target_Omega_Radian();
    inline float Get_Target_Torque();
    inline uint16_t Get_Speed_Limit();//获取角度环或者位置环的速度限制
    inline int16_t Get_Iq_Control();//获取扭矩限制
    
    inline void Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status);
	inline void Set_LK_Motor_Control_ID(Enum_LK_Motor_Control_ID __LK_Motor_Control_ID);
    inline void Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_IMU_K_P(float __IMU_K_P);
    inline void Set_IMU_K_D(float __IMU_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Radian(float __Target_Radian);
    inline void Set_Target_Omega_Angle(float __Target_Omega_Angle);
    inline void Set_Target_Omega_Radian(float __Target_Omega_Radian);
    inline void Set_Target_Current(float __Target_Current);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Speed_Limit(uint16_t __Speed_Limit);//设置角度环或者位置环的速度限制
    inline void Set_Iq_Control(int16_t __Iq_Control);    
    inline void Set_Out(float __Out);


    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();

protected:
    
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    
    Enum_LK_Motor_ID CAN_ID;
    
    uint8_t *CAN_Tx_Data;
    
    uint32_t Position_Offset;
    
    float Omega_Max=200;
    
    float Current_Max;
    
    const int16_t Current_Max_Cmd = 262144;//外环速度环262144限幅
    
    float Out = 0.0f;
    
    
    const float Torque_Current = 0.3;  

    
    uint32_t Position_Max = 16383;


    //开始标志位
    uint8_t Start_Flag = 0;

    uint32_t Flag = 0;
    
    uint32_t Pre_Flag = 0;

    uint16_t Speed_Limit=65535;//角度或者位置环的速度限制，对应实际转速1dps/LSB，即360代表360dps。 

    int16_t Iq_Control=2048;//扭矩限制

    Enum_LK_Motor_Status LK_Motor_Status = LK_Motor_Status_DISABLE;
    
    Struct_LK_Motor_Rx_Data Data;

//瓴控电机控制ID
    Enum_LK_Motor_Control_ID LK_Motor_Control_ID = LK_Motor_Control_Omega;
    
    Enum_LK_Motor_Control_Status LK_Motor_Control_Status = LK_Motor_Control_Status_DISABLE;
    //瓴控电机控制模式（跑pid模式）
    Enum_LK_Motor_Control_Method LK_Motor_Control_Method = LK_Motor_Control_Method_ANGLE;
    
    float IMU_K_P = 0.0f;
    
    float IMU_K_D = 0.0f;

    float Target_Radian = 0.0f;//目标角度，弧度制
    float Target_Angle = 0.0f;//目标角度
    float Target_Omega_Radian = 0.0f;//目标角速度，弧度制
    float Target_Omega_Angle = 0.0f;//角度制
    
    float Target_Current = 0.0f;
    
    float Target_Torque = 0.0f;
    
 
    void Output(void);
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
	* @brief 获取瓴控电机状态
 *
 * @return Enum_LK_Motor_Status 
 */
Enum_LK_Motor_Status Class_LK_Motor::Get_LK_Motor_Status()
{
    return (LK_Motor_Status);
}


/**
* @brief 获取瓴控电机控制状态
 *
 * @return Enum_LK_Motor_Status 
 */
Enum_LK_Motor_Control_Status Class_LK_Motor::Get_LK_Motor_Control_Status()
{
    return (LK_Motor_Control_Status);
}


/**
 * @brief 获取当前角度
 *
 * @return float   °
 */
float Class_LK_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前角度 
 *
 * @return float , rad
 */
float Class_LK_Motor::Get_Now_Radian()
{
    return (Data.Now_Radian);
}

/**
 * @brief 获取当前角速度 
 *
 * @return float , rad/s
 */
float Class_LK_Motor::Get_Now_Omega_Radian()
{
    return (Data.Now_Omega_Radian);
}

/**
 * @brief 获取当前角速度, °/s
 *
 * @return float , °/s
 */
float Class_LK_Motor::Get_Now_Omega_Angle()
{
    return (Data.Now_Omega_Angle);
}

/**
 * @brief 获取当前力矩或功率  对MS是功率
 *
 * @return float , 
 */
float Class_LK_Motor::Get_Now_Torque()
{
    return (Data.Now_Current);
}

/**
 * @brief 获取当前温度
 *
 * @return float 
 */
float Class_LK_Motor::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取当前控制命令
 *
 * @return Enum_LK_Motor_Control_Method 
 */
Enum_LK_Motor_Control_Method Class_LK_Motor::Get_LK_Motor_Control_Method()
{
    return (LK_Motor_Control_Method);
}

/**
 *	@brief 获取当前控制ID
 *
 *	@return Enum_LK_Motor_Control_ID
 */

Enum_LK_Motor_Control_ID Class_LK_Motor::Get_LK_Motor_Control_ID()
{
	return(LK_Motor_Control_ID);
}

/**
 * @brief ???MIT??Kp?, 0~500
 *
 * @return float MIT??Kp?, 0~500
 */
float Class_LK_Motor::Get_IMU_K_P()
{
    return (IMU_K_P);
}

/**
 * @brief ???MIT??Kd?, 0~5
 *
 * @return float MIT??Kd?, 0~5
 */
float Class_LK_Motor::Get_IMU_K_D()
{
    return (IMU_K_D);
}

/**
 * @brief 获取目标角度 
 *
 * @return float  °
 */
float Class_LK_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标角度 rad
 *
 * @return float  rad
 */
float Class_LK_Motor::Get_Target_Radian()
{
    return (Target_Radian);
}

/**
 * @brief ??????????, rad/s
 *
 * @return float ???????, rad/s
 */
float Class_LK_Motor::Get_Target_Omega_Angle()
{
    return (Target_Omega_Angle);
}

/**
 * @brief ??????????, rad/s
 *
 * @return float ???????, rad/s
 */
float Class_LK_Motor::Get_Target_Omega_Radian()
{
    return (Target_Omega_Radian);
}

/**
 * @brief 获取目标扭矩
 *
 * @return float 
 */
float Class_LK_Motor::Get_Target_Torque()
{
    return (Target_Current);
}


/**
 * @brief 获取瓴控ID为角度环下的速度限制值
 *
 * @return uint16_t 
 */
uint16_t Class_LK_Motor::Get_Speed_Limit()
{
    return (Speed_Limit);
}

/**
 * @brief 获取扭矩限制值
 *
 * @return int16_t 
 */

int16_t Class_LK_Motor::Get_Iq_Control()
{
   return (Iq_Control);
}
/**
 * @brief ?څ?????????
 *
 * @param __DM_Motor_Control_Status ?????????
 */
void Class_LK_Motor::Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status)
{
    LK_Motor_Control_Status = __DM_Motor_Control_Status;
}

/**
* @brief 设置瓴控电机控制ID
 *
 * @param __Control_ID
 */
void Class_LK_Motor::Set_LK_Motor_Control_ID(Enum_LK_Motor_Control_ID __LK_Motor_Control_ID)
{
	LK_Motor_Control_ID=__LK_Motor_Control_ID;
}


/**
* @brief 设置瓴控电机控制模式（跑哪种pid环）
 *
 * @param __Control_Method 
 */
void Class_LK_Motor::Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __Control_Method)
{
    LK_Motor_Control_Method = __Control_Method;
}

/**
 * @brief ?څMIT??Kp?, 0~500, ????6, ��????????
 *
 * @param __MIT_K_P MIT??Kp?, 0~500, ????6, ��????????
 */
void Class_LK_Motor::Set_IMU_K_P(float __IMU_K_P)
{
    IMU_K_P = __IMU_K_P;
}

/**
 * @brief ?څMIT??Kd?, 0~5, ????0.2, ��?��??????????
 *
 * @param __MIT_K_D MIT??Kd?, 0~5, ????0.2, ��?��??????????
 */
void Class_LK_Motor::Set_IMU_K_D(float __IMU_K_D)
{
    IMU_K_D = __IMU_K_D;
}

/**
 * @brief 设置目标角度 rad
 *
 * @param __Target_Angle  rad
 */
void Class_LK_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设置目标角度 rad
 *
 * @param __Target_Angle  rad
 */
void Class_LK_Motor::Set_Target_Radian(float __Target_Radian)
{
    Target_Radian = __Target_Radian;
}

/**
 * @brief 设置目标角速度 rad/s
 *
 * @param __Target_Omega  rad/s
 */
void Class_LK_Motor::Set_Target_Omega_Radian(float __Target_Omega_Radian)
{
    Target_Omega_Radian = __Target_Omega_Radian;
}

/**
 * @brief 设置目标, °/s
 *
 * @param __Target_Omega 
 */
void Class_LK_Motor::Set_Target_Omega_Angle(float __Target_Omega_Angle)
{
    Target_Omega_Angle = __Target_Omega_Angle;
}

/**
 * @brief 设置目标电流
 *
 * @param __Target_Current 
 */
void Class_LK_Motor::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief 设置目标扭矩
 *
 * @param __Target_Torque 
 */
void Class_LK_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}


 /**
 * @brief 设置位置或角度模式下的速度限制
 *
 * @param __Speed_Limit 
 */

void Class_LK_Motor::Set_Speed_Limit(uint16_t __Speed_Limit)
{
    Speed_Limit=__Speed_Limit;
}
 /**
 * @brief 设置扭矩上限
 *
 * @param __Iq_Control 
 */
void Class_LK_Motor::Set_Iq_Control(int16_t __Iq_Control)
{
    Iq_Control=__Iq_Control;
}

void Class_LK_Motor::Set_Out(float __Out)
{
    if(__Out > Current_Max_Cmd)
    {
        __Out = Current_Max_Cmd;
    }
    else if(__Out < -Current_Max_Cmd)
    {
        __Out = -Current_Max_Cmd;
    }
}
/**
 * @brief 获取PID输出最大值
 *
 * @return float 输出最大值
 */
float Class_LK_Motor::Get_Output_Max()
{
    return (Current_Max_Cmd);
}


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
