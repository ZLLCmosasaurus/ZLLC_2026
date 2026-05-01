/**
 * @file dvc_minipc.h
 * @author lez by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_MINIPC_H
#define DVC_MINIPC_H

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "main.h"
#include "drv_usb.h"
#include "drv_can.h"
//#include "dvc_dmimu.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_imu.h"
#include "dvc_referee.h"
#include "math.h"
#include "config.h"
/* Exported macros -----------------------------------------------------------*/
#define PI 3.14159f
/* Exported types ------------------------------------------------------------*/

static const uint16_t MINPC_CRC16_INIT = 0xFFFF;

static const uint16_t W_CRC_TABLE[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
        0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
        0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
        0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
        0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
        0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
        0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
        0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
        0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
        0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
        0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
        0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief 风车类型
 *
 */
enum Enum_Windmill_Type : uint8_t
{
    Windmill_Type_Small = 0,
    Windmill_Type_Big,
};

/**
 * @brief 迷你主机控制类型
 *
 */
enum Enum_MiniPC_Type : uint8_t
{
    MiniPC_Type_Nomal = 0, // 装甲板
    MiniPC_Type_Windmill,  // 风车
};
/**
 * @brief 反小陀螺状态
 *
 */
enum Enum_Antispin_Type : uint8_t
{
    Antispin_Off = 0,
    Antispin_On,
};

/**
 * @brief 迷你主机状态
 *
 */
enum Enum_MiniPC_Status : uint8_t
{
    MiniPC_Status_DISABLE = 0,
    MiniPC_Status_ENABLE,
};

/**
 * @brief 裁判系统UI刷新状态
 *
 */
enum Enum_Referee_UI_Refresh_Status : uint8_t
{
    Referee_UI_Refresh_Status_DISABLE = 0,
    Referee_UI_Refresh_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_MiniPC_Data_Status : uint8_t
{
    MiniPC_Data_Status_DISABLE = 0,
    MiniPC_Data_Status_ENABLE,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_MiniPC_Game_Stage : uint8_t
{
    MiniPC_Game_Stage_NOT_STARTED = 0,
    MiniPC_Game_Stage_READY,
    MiniPC_Game_Stage_SELF_TESTING,
    MiniPC_Game_Stage_5S_COUNTDOWN,
    MiniPC_Game_Stage_BATTLE,
    MiniPC_Game_Stage_SETTLEMENT,
};

/**
 * @brief 战车运动控制方式
 *
 */
enum Enum_MiniPC_Move_Control_Mode : uint8_t
{
    MiniPC_Move_Control_Mode_OMEGA = 0,
    MiniPC_Move_Control_Mode_ANGLE,
};

/**
 * @brief 战车运动控制方式
 *
 */
enum Enum_MiniPC_Self_Color : uint8_t
{
    MiniPC_Self_Color_RED = 0,
    MiniPC_Self_Color_BLUE,
};

/**
 * @brief 迷你主机源数据
 *
 */
struct Struct_MiniPC_USB_Data
{
    uint8_t Frame_Header;
    uint8_t Data[50];
} __attribute__((packed));

/**
 * @brief 迷你主机接收的规划数据,
 *
 */
struct Struct_MiniPC_Rx_Data
{
    uint8_t Reserved;
    float Chassis_Target_Velocity_X;
    float Chassis_Target_Velocity_Y;
    float Chassis_Target_Omega;
    float Chassis_Target_Angle;
    float Chassis_Now_Omega;
    float Chassis_Now_Angle;
    float Gimbal_Target_Yaw_Angle;
    float Gimbal_Target_Yaw_Omega;
    float Gimbal_Target_Pitch_Angle;
    float Gimbal_Target_Pitch_Omega;
    uint8_t Booster_Frequency;
    Enum_MiniPC_Move_Control_Mode Move_Control_Mode;
    uint8_t Frame_Rear;
} __attribute__((packed));

/**
 * @brief 迷你主机发送的反馈数据
 *
 */
struct Struct_MiniPC_Tx_Data
{
    Enum_MiniPC_Game_Stage Game_Stage;
    float Chassis_Now_Velocity_X;
    float Chassis_Now_Velocity_Y;
    float Chassis_Now_Omega;
    float Gimbal_Now_Yaw_Angle;
    float Gimbal_Now_Yaw_Omega;
    float Gimbal_Now_Pitch_Angle;
    float Gimbal_Now_Pitch_Omega;
    uint8_t Armor_Attacked_ID : 2;
    uint8_t Armor_Attacked_Ammo_Type_Enum : 1;
    uint8_t Self_Color : 1;
    uint8_t Outpost_Status_Enum : 1;
    uint8_t Outpost_Protect_Status_Enum : 1;
    uint8_t Reserved : 2;
    uint8_t Frame_Rear;
    uint8_t Carriage_Return;
} __attribute__((packed));

/**
 * @brief 发送数据包
 *
 */
struct Pack_tx_t
{
    int16_t q[4];
} __attribute__((packed));
/**
 * @brief 接收数据包
 *
 */
struct Pack_rx_t
{
    // int16_t target_x;
    // int16_t target_y;
    // int16_t target_z;
    // int8_t  Fire;
    uint8_t alive;
    uint8_t Fire;
    int16_t yaw;
    int16_t pitch;
} __attribute__((packed));

/**
 * @brief Specialized, 迷你主机类
 *
 */
class Class_MiniPC
{
public:
    void Init(CAN_HandleTypeDef *hcan);

    inline Enum_MiniPC_Status Get_MiniPC_Status();
    // inline Enum_Antispin_Type Get_Antispin_Type();
    inline float Get_Chassis_Target_Velocity_X();
    inline float Get_Chassis_Target_Velocity_Y();
    inline float Get_Chassis_Target_Omega();
    inline float Get_Chassis_Target_Angle();
    inline float Get_Chassis_Now_Omega();
    inline float Get_Chassis_Now_Angle();
    inline float Get_Gimbal_Target_Yaw_Angle();
    inline float Get_Gimbal_Target_Yaw_Omega();
    inline float Get_Gimbal_Target_Pitch_Angle();
    inline float Get_Gimbal_Target_Pitch_Omega();
    inline float Get_Booster_Frequency();
    inline float Get_Rx_Pitch_Angle();
    inline float Get_Rx_Yaw_Angle();
    inline uint8_t Get_Fire_Status();
    inline uint8_t Get_Alive_Status();
    inline float Get_Distance();
    inline Enum_MiniPC_Move_Control_Mode Get_Move_Control_Mode();

    inline void Set_Game_Stage(Enum_MiniPC_Game_Stage __Game_Stage);
    inline void Set_Chassis_Now_Velocity_X(float __Chassis_Now_Velocity_X);
    inline void Set_Chassis_Now_Velocity_Y(float __Chassis_Now_Velocity_Y);
    inline void Set_Chassis_Now_Omega(float __Chassis_Now_Omega);
    inline void Set_Gimbal_Now_Yaw_Angle(float __Gimbal_Now_Yaw_Angle);
    inline void Set_Gimbal_Now_Yaw_Omega(float __Gimbal_Now_Yaw_Omega);
    inline void Set_Gimbal_Now_Pitch_Angle(float __Gimbal_Now_Pitch_Angle);
    inline void Set_Gimbal_Now_Pitch_Omega(float __Gimbal_Now_Pitch_Omega);
    inline void Set_Armor_Attacked_ID(uint8_t __Armor_Attacked_ID);
    inline void Set_Armor_Attacked_Ammo_Status(Enum_MiniPC_Data_Status __Armor_Attacked_Ammo_Status);
    inline void Set_Self_Color(Enum_MiniPC_Self_Color __Self_Color);
    inline void Set_Outpost_Status(Enum_MiniPC_Data_Status __Outpost_Status);
    inline void Set_Outpost_Protect_Status(Enum_MiniPC_Data_Status __Outpost_Protect_Status);
    // inline void Set_Antispin_Type(Enum_Antispin_Type __Antispin_Type);
    void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
    bool Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);
    uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

    inline void Transform_Angle_Tx();

    float calc_yaw(float x, float y, float z);
    float calc_distance(float x, float y, float z);
    float calc_pitch(float x, float y, float z);
    void Self_aim(float x, float y, float z, float *yaw, float *pitch, float *distance);

    float meanFilter(float input);

    void CAN_RxCpltCallback(uint8_t *rx_data);
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void TIM_Write_PeriodElapsedCallback();

    Class_IMU *IMU;
    Class_Referee *Referee;

protected:
    // 初始化相关常量

    // 绑定的USB
    Struct_USB_Manage_Object *USB_Manage_Object;
    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;
    // 数据包头标
    uint8_t Frame_Header;
    // 数据包尾标
    uint8_t Frame_Rear;

    // 常量

    // 内部变量

    // 当前时刻的迷你主机接收flag
    uint32_t Flag = 0;
    // 前一时刻的迷你主机接收flag
    uint32_t Pre_Flag = 0;

    // 读变量

    // 迷你主机状态
    Enum_MiniPC_Status MiniPC_Status = MiniPC_Status_DISABLE;
    // 迷你主机对外接口信息
    Struct_MiniPC_Rx_Data Data_NUC_To_MCU;

    Pack_tx_t Pack_Tx_CAN;
    Pack_rx_t Pack_Rx;

    float Tx_Angle_Roll;
    float Tx_Angle_Pitch;
    float Tx_Angle_Yaw;
    float Tx_Angle_Gyro_Yaw;

    float Rx_Angle_Roll;
    float Rx_Angle_Pitch;
    float Rx_Angle_Yaw;

    uint8_t Fire;
    uint8_t alive;

    const float g = 9.8;         // 重力加速度
    const float bullet_v = 21.7; // 子弹速度
    const float a_d = 0.0595f;

    // 距离
    float Distance;

    // 写变量

    // 迷你主机对外接口信息
    Struct_MiniPC_Tx_Data Data_MCU_To_NUC;

    // 读写变量

    // 内部函数

    void Data_Process();
    void Output();
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取迷你主机状态
 *
 * @return Enum_MiniPC_Status 迷你主机状态
 */
Enum_MiniPC_Status Class_MiniPC::Get_MiniPC_Status()
{
    return (MiniPC_Status);
}

/**
 * @brief 获取底盘目标速度x
 *
 * @return float 底盘目标速度x
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_X()
{
    return (Data_NUC_To_MCU.Chassis_Target_Velocity_X);
}

/**
 * @brief 获取底盘目标速度y
 *
 * @return float 底盘目标速度y
 */
float Class_MiniPC::Get_Chassis_Target_Velocity_Y()
{
    return (Data_NUC_To_MCU.Chassis_Target_Velocity_Y);
}

/**
 * @brief 获取底盘目标速度omega
 *
 * @return float 获取底盘目标速度omega
 */
float Class_MiniPC::Get_Chassis_Target_Omega()
{
    return (Data_NUC_To_MCU.Chassis_Target_Omega);
}

/**
 * @brief 获取底盘目标角度
 *
 * @return float 获取底盘目标角度
 */
float Class_MiniPC::Get_Chassis_Target_Angle()
{
    return (Data_NUC_To_MCU.Chassis_Target_Angle);
}

/**
 * @brief 获取底盘当前速度omega
 *
 * @return float 获取底盘当前速度omega
 */
float Class_MiniPC::Get_Chassis_Now_Omega()
{
    return (Data_NUC_To_MCU.Chassis_Now_Omega);
}

/**
 * @brief 获取底盘当前角度
 *
 * @return float 获取底盘当前角度
 */
float Class_MiniPC::Get_Chassis_Now_Angle()
{
    return (Data_NUC_To_MCU.Chassis_Now_Angle);
}

/**
 * @brief 获取云台目标角度yaw
 *
 * @return float 云台目标角度yaw
 */
float Class_MiniPC::Get_Gimbal_Target_Yaw_Angle()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Yaw_Angle);
}

/**
 * @brief 获取云台目标速度yaw
 *
 * @return float 获取云台目标速度yaw
 */
float Class_MiniPC::Get_Gimbal_Target_Yaw_Omega()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Yaw_Omega);
}

/**
 * @brief 获取云台目标pitch
 *
 * @return float 云台目标pitch
 */
float Class_MiniPC::Get_Gimbal_Target_Pitch_Angle()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Pitch_Angle);
}

/**
 * @brief 获取距离
 *
 * @return float 距离
 */
float Class_MiniPC::Get_Distance()
{
    return (Distance);
}

/**
 * @brief 获取云台目标速度pitch
 *
 * @return float 云台目标速度pitch
 */
float Class_MiniPC::Get_Gimbal_Target_Pitch_Omega()
{
    return (Data_NUC_To_MCU.Gimbal_Target_Pitch_Omega);
}

/**
 * @brief 获取射频
 *
 * @return float 射频
 */
float Class_MiniPC::Get_Booster_Frequency()
{
    return (Data_NUC_To_MCU.Booster_Frequency);
}

/**
 * @brief 获取移动控制模式
 *
 * @return Enum_MiniPC_Move_Control_Mode 移动控制模式
 */
Enum_MiniPC_Move_Control_Mode Class_MiniPC::Get_Move_Control_Mode()
{
    return (Data_NUC_To_MCU.Move_Control_Mode);
}

float Class_MiniPC::Get_Rx_Pitch_Angle()
{
    return (Rx_Angle_Pitch);
}

float Class_MiniPC::Get_Rx_Yaw_Angle()
{
    return (Rx_Angle_Yaw);
}

inline uint8_t Class_MiniPC::Get_Fire_Status()
{
    return Fire;
}

inline uint8_t Class_MiniPC::Get_Alive_Status()
{
    return alive;
}

/**
 * @brief
 *
 * @param __Game_Stage
 */
void Class_MiniPC::Set_Game_Stage(Enum_MiniPC_Game_Stage __Game_Stage)
{
    Data_MCU_To_NUC.Game_Stage = __Game_Stage;
}

/**
 * @brief 设定底盘逆解速度x
 *
 * @param __Chassis_Now_Velocity_X 底盘逆解速度x
 */
void Class_MiniPC::Set_Chassis_Now_Velocity_X(float __Chassis_Now_Velocity_X)
{
    Data_MCU_To_NUC.Chassis_Now_Velocity_X = __Chassis_Now_Velocity_X;
}

/**
 * @brief 设定底盘逆解速度y
 *
 * @param __Chassis_Now_Velocity_Y 底盘逆解速度y
 */
void Class_MiniPC::Set_Chassis_Now_Velocity_Y(float __Chassis_Now_Velocity_Y)
{
    Data_MCU_To_NUC.Chassis_Now_Velocity_Y = __Chassis_Now_Velocity_Y;
}

/**
 * @brief 设定底盘逆解速度omega
 *
 * @param __Chassis_Now_Omega 底盘逆解速度omega
 */
void Class_MiniPC::Set_Chassis_Now_Omega(float __Chassis_Now_Omega)
{
    Data_MCU_To_NUC.Chassis_Now_Omega = __Chassis_Now_Omega;
}

/**
 * @brief 设定云台当前角度yaw
 *
 * @param __Gimbal_Now_Yaw_Angle 云台当前角度yaw
 */
void Class_MiniPC::Set_Gimbal_Now_Yaw_Angle(float __Gimbal_Now_Yaw_Angle)
{
    Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle = __Gimbal_Now_Yaw_Angle;
}

/**
 * @brief 设定云台当前角速度yaw
 *
 * @param __Gimbal_Now_Yaw_Angle 云台当前角速度yaw
 */
void Class_MiniPC::Set_Gimbal_Now_Yaw_Omega(float __Gimbal_Now_Yaw_Omega)
{
    Data_MCU_To_NUC.Gimbal_Now_Yaw_Omega = __Gimbal_Now_Yaw_Omega;
}

/**
 * @brief 设定云台当前角度pitch
 *
 * @param __Gimbal_Now_Pitch_Angle 云台当前角度pitch
 */
void Class_MiniPC::Set_Gimbal_Now_Pitch_Angle(float __Gimbal_Now_Pitch_Angle)
{
    Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle = __Gimbal_Now_Pitch_Angle;
}

/**
 * @brief 设定云台当前角速度pitch
 *
 * @param __Gimbal_Now_Pitch_Angle 云台当前角速度pitch
 */
void Class_MiniPC::Set_Gimbal_Now_Pitch_Omega(float __Gimbal_Now_Pitch_Omega)
{
    Data_MCU_To_NUC.Gimbal_Now_Pitch_Omega = __Gimbal_Now_Pitch_Omega;
}

/**
 * @brief 设定装甲板受击ID
 *
 * @param __Armor_Attacked 装甲板受击ID
 */
void Class_MiniPC::Set_Armor_Attacked_ID(uint8_t __Armor_Attacked_ID)
{
    Data_MCU_To_NUC.Armor_Attacked_ID = __Armor_Attacked_ID;
}

/**
 * @brief 设定装甲板受击子弹类型
 *
 * @param __Armor_Attacked_Ammo_Type 装甲板受击子弹类型
 */
void Class_MiniPC::Set_Armor_Attacked_Ammo_Status(Enum_MiniPC_Data_Status __Armor_Attacked_Ammo_Status)
{
    Data_MCU_To_NUC.Armor_Attacked_Ammo_Type_Enum = __Armor_Attacked_Ammo_Status;
}

/**
 * @brief 设定迷你主机类型
 *
 * @param __MiniPC_Type 迷你主机类型
 */
void Class_MiniPC::Set_Self_Color(Enum_MiniPC_Self_Color __Self_Color)
{
    Data_MCU_To_NUC.Self_Color = __Self_Color;
}

/**
 * @brief 设定前哨站状态
 *
 * @param __Outpost_Status 前哨站状态
 */
void Class_MiniPC::Set_Outpost_Status(Enum_MiniPC_Data_Status __Outpost_Status)
{
    Data_MCU_To_NUC.Outpost_Status_Enum = __Outpost_Status;
}

/**
 * @brief 设定前哨站保护状态
 *
 * @param __Outpost_Protect_Status 前哨站保护状态
 */
void Class_MiniPC::Set_Outpost_Protect_Status(Enum_MiniPC_Data_Status __Outpost_Protect_Status)
{
    Data_MCU_To_NUC.Outpost_Protect_Status_Enum = __Outpost_Protect_Status;
}

/**
 * @brief 发送数据处理
 *
 */
void Class_MiniPC::Transform_Angle_Tx()
{
    Tx_Angle_Pitch = -IMU->Get_Angle_Roll();
    Tx_Angle_Roll = IMU->Get_Angle_Pitch();
    Tx_Angle_Yaw = IMU->Get_Angle_Yaw();
}


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
