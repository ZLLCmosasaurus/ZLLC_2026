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
 * @brief ????????
 *
 */
enum Enum_LK_Motor_Status
{
    LK_Motor_Status_DISABLE = 0,
    LK_Motor_Status_ENABLE,
};

/**
 * @brief ????????ID???????
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
 * @brief ????????????
 *
 */
enum Enum_LK_Motor_Control_Status
{
    LK_Motor_Control_Status_DISABLE = 0,
    LK_Motor_Control_Status_ENABLE,
};

/**
 * @brief ??????can????cmd_id
 *
 */
enum Enum_LK_Motor_Control_ID : uint8_t
{
    LK_Motor_Control_Shut_Down = 0x80,   //??????
    LK_Motor_Control_Stop = 0x81, //?????
    LK_Motor_Control_Run = 0x88,//???????
    LK_Motor_Control_Torque = 0xA1,//??????????
};

/**
 * @brief ????????????
 *
 */
enum Enum_LK_Motor_Control_Method
{
    LK_Motor_Control_Method_IMU_ANGLE = 0,
    LK_Motor_Control_Method_IMU_OMEGA,
    LK_Motor_Control_Method_ANGLE,
    LK_Motor_Control_Method_OMEGA,
    LK_Motor_Control_Method_TORQUE,
};

/**
 * @brief ???????????
 *
 */
struct Struct_LK_Motor_CAN_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;
    uint8_t Temperature_Centigrade;  //?????
    uint16_t Current_Reverse;
    uint16_t Omega_Reverse;
    uint16_t Encoder_Reverse;
} __attribute__((packed));

/**
 * @brief ????????????????????, ???��????��??
 *
 */
struct Struct_LK_Motor_Rx_Data
{
    Enum_LK_Motor_Control_ID CMD_ID;  //????????ID
    float Now_Angle;  //��ǰ�Ƕ� ���Ƕ��ƣ�
    float Now_Radian;  //��ǰ�Ƕ� �������ƣ�
    float Now_Omega_Angle;  //��ǰ���ٶ� (�Ƕ���)
    float Now_Omega_Radian;  //��ǰ���ٶ� (������)
    float Now_Current;  
    float Now_Temperature; 
    uint16_t Pre_Encoder; 
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief LK??????, ?????????????????
 * DM_Motor_Control_Method_POSITION_OMEGA????, ????????????????��???PI????, ????250??0
 * 
 * PMAX?????????????????3.141593, ??PI, ???????IMU???????????
 *
 */
class Class_LK_Motor
{
public:
    // PID????????
    Class_PID PID_Angle;
    // PID??????????
    Class_PID PID_Omega;
    // PID????????
    Class_PID PID_Torque;

    void Init(CAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID, float __Omega_Max, int32_t __Position_Offset = 0, float __Current_Max = 33.0f ,Enum_LK_Motor_Control_Method __Control_Method = LK_Motor_Control_Method_IMU_ANGLE);

    inline Enum_LK_Motor_Control_Status Get_LK_Motor_Control_Status();
    inline Enum_LK_Motor_Status Get_LK_Motor_Status();
    inline float Get_Output_Max();
    inline float Get_Now_Angle();
    inline float Get_Now_Radian();
    inline float Get_Now_Omega_Angle();
    inline float Get_Now_Omega_Radian();
    inline float Get_Now_Torque();
    inline float Get_Now_Temperature();
    inline Enum_LK_Motor_Control_Method Get_Control_Method();
    inline float Get_IMU_K_P();
    inline float Get_IMU_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Radian();
    inline float Get_Target_Omega_Angle();
    inline float Get_Target_Omega_Radian();
    inline float Get_Target_Torque();
    
    inline void Set_LK_Control_Status(Enum_LK_Motor_Control_Status __DM_Motor_Control_Status);
    inline void Set_LK_Motor_Control_Method(Enum_LK_Motor_Control_Method __DM_Motor_Control_Method);
    inline void Set_IMU_K_P(float __IMU_K_P);
    inline void Set_IMU_K_D(float __IMU_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Radian(float __Target_Radian);
    inline void Set_Target_Omega_Angle(float __Target_Omega_Angle);
    inline void Set_Target_Omega_Radian(float __Target_Omega_Radian);
    inline void Set_Target_Current(float __Target_Current);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    
    Enum_LK_Motor_ID CAN_ID;
    
    uint8_t *CAN_Tx_Data;
    
    uint32_t Position_Offset;
    
    float Omega_Max;
    
    float Current_Max;
    
    const int16_t Current_Max_Cmd = 2000;
    
    float Out = 0.0f;
    
    
    const float Torque_Current = 0.3;  

    
    uint32_t Position_Max = 16383;


    //����ϵ��һ֡��־λ
    uint8_t Start_Flag = 0;

    uint32_t Flag = 0;
    
    uint32_t Pre_Flag = 0;


    Enum_LK_Motor_Status LK_Motor_Status = LK_Motor_Status_DISABLE;
    
    Struct_LK_Motor_Rx_Data Data;


    Enum_LK_Motor_Control_ID LK_Motor_Control_ID = LK_Motor_Control_Torque;
    
    Enum_LK_Motor_Control_Status LK_Motor_Control_Status = LK_Motor_Control_Status_DISABLE;
    
    Enum_LK_Motor_Control_Method LK_Motor_Control_Method = LK_Motor_Control_Method_IMU_ANGLE;
    
    float IMU_K_P = 0.0f;
    
    float IMU_K_D = 0.0f;

    float Target_Radian = 0.0f;
    float Target_Angle = 0.0f;
    float Target_Omega_Radian = 0.0f;
    float Target_Omega_Angle = 0.0f;
    
    float Target_Current = 0.0f;
    
    float Target_Torque = 0.0f;
    
 
    void Output(void);
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief ????????
 *
 * @return Enum_LK_Motor_Status ?????
 */
Enum_LK_Motor_Status Class_LK_Motor::Get_LK_Motor_Status()
{
    return (LK_Motor_Status);
}


/**
 * @brief ????????
 *
 * @return Enum_LK_Motor_Status ?????
 */
Enum_LK_Motor_Control_Status Class_LK_Motor::Get_LK_Motor_Control_Status()
{
    return (LK_Motor_Control_Status);
}


/**
 * @brief ??????????, rad
 *
 * @return float ???????, rad
 */
float Class_LK_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief ??????????, rad
 *
 * @return float ???????, rad
 */
float Class_LK_Motor::Get_Now_Radian()
{
    return (Data.Now_Radian);
}

/**
 * @brief ???????????, rad/s
 *
 * @return float ????????, rad/s
 */
float Class_LK_Motor::Get_Now_Omega_Radian()
{
    return (Data.Now_Omega_Radian);
}

/**
 * @brief ???????????, rad/s
 *
 * @return float ????????, rad/s
 */
float Class_LK_Motor::Get_Now_Omega_Angle()
{
    return (Data.Now_Omega_Angle);
}

/**
 * @brief ???????????, ?????��????
 *
 * @return float ????????, ?????��????
 */
float Class_LK_Motor::Get_Now_Torque()
{
    return (Data.Now_Current);
}

/**
 * @brief ??????MOS??????, ?????
 *
 * @return float ???MOS??????, ?????
 */
float Class_LK_Motor::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief ????????????
 *
 * @return Enum_LK_Motor_Control_Method ?????????
 */
Enum_LK_Motor_Control_Method Class_LK_Motor::Get_Control_Method()
{
    return (LK_Motor_Control_Method);
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
 * @brief ?????????, rad
 *
 * @return float ??????, rad
 */
float Class_LK_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief ?????????, rad
 *
 * @return float ??????, rad
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
 * @brief ??????????
 *
 * @return float ???????
 */
float Class_LK_Motor::Get_Target_Torque()
{
    return (Target_Current);
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
 * @brief ?څ?????????
 *
 * @param __Control_Method ?????????
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
 * @brief ?څ??????, rad
 *
 * @param __Target_Angle ??????, rad
 */
void Class_LK_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief ?څ??????, rad
 *
 * @param __Target_Angle ??????, rad
 */
void Class_LK_Motor::Set_Target_Radian(float __Target_Radian)
{
    Target_Radian = __Target_Radian;
}

/**
 * @brief ?څ???????, rad/s
 *
 * @param __Target_Omega ???????, rad/s
 */
void Class_LK_Motor::Set_Target_Omega_Radian(float __Target_Omega_Radian)
{
    Target_Omega_Radian = __Target_Omega_Radian;
}

/**
 * @brief ?څ???????, rad/s
 *
 * @param __Target_Omega ???????, rad/s
 */
void Class_LK_Motor::Set_Target_Omega_Angle(float __Target_Omega_Angle)
{
    Target_Omega_Angle = __Target_Omega_Angle;
}

/**
 * @brief ?څ???????????
 *
 * @param __Target_Current ???????????
 */
void Class_LK_Motor::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief ?څ???????????
 *
 * @param __Target_Torque ???????????
 */
void Class_LK_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
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

float Class_LK_Motor::Get_Output_Max()
{
    return (Current_Max_Cmd);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
