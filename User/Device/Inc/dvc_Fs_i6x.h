/**
 * @file dvc_Fs_i6x.h
 * @author 
 * @brief Fs_i6x设备
 * @version 0.1
 * @date 
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef DVC_Fs_i6x_H
#define DVC_Fs_i6x_H

/* Includes ------------------------------------------------------------------*/

#include <limits.h>
#include <string.h>
#include "drv_uart.h"
/* Exported macros -----------------------------------------------------------*/

//拨动开关位置
#define SWITCH_UP (240)
#define SWITCH_DOWN (1807)
#define SWITCH_MIDDLE (1023)


/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控器状态
 *
 */
enum Enum_FS_Status
{
    FS_Status_DISABLE = 0,
    FS_Status_ENABLE,
};

/**
 * @brief 遥控器数据更新状态
 *
 */
enum Enum_FS_Updata_Status
{
    FS_Status_DisUpdata = 0,
    FS_Status_Updata,
};

/**
 * @brief 拨动开关状态
 *
 */
enum Enum_FS_Switch_Status
{
    FS_Switch_Status_UP = 0,           //上状态
    FS_Switch_Status_TRIG_UP_MIDDLE,   //上到中的突变状态
    FS_Switch_Status_TRIG_MIDDLE_UP,   //中到上的突变状态
    FS_Switch_Status_MIDDLE,           //中状态
    FS_Switch_Status_TRIG_MIDDLE_DOWN, //中到下的突变状态
    FS_Switch_Status_TRIG_DOWN_MIDDLE, //下到中的突变状态
    FS_Switch_Status_DOWN,             //下状态
    FS_Switch_Status_TRIG_UP_DOWN,     //上到下的突变状态
    FS_Switch_Status_TRIG_DOWN_UP,     //下到上的突变状态
};


/**
 * @brief FS_i6x源数据
 *
 */
struct Struct_FS_UART_Data 
{
    uint8_t start;          // 字节0: 0x0F
    uint16_t ch0 : 11;      // 通道0，原始值 0~2047
    uint16_t ch1 : 11;      // 通道1
    uint16_t ch2 : 11;      // 通道2
    uint16_t ch3 : 11;      // 通道3
    uint16_t ch4 : 11;      // 通道4
    uint16_t ch5 : 11;      // 通道5
    uint16_t s0  : 11;      // 开关0（原始值）
    uint16_t s1  : 11;      // 开关1
    uint16_t s2  : 11;      // 开关2
    uint16_t s3  : 11;      // 开关3
    uint8_t flags;          // 其他标志位（如果协议有）
    uint8_t end;            // 字节24: 0x00
} __attribute__((packed)) ;
/**
 * @brief FS_i6x经过处理的的数据, 摇杆信息经过归一化到-1~1
 *
 */
struct Struct_FS_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    Enum_FS_Switch_Status Switch_0;
    Enum_FS_Switch_Status Switch_1;
    Enum_FS_Switch_Status Switch_2;
    Enum_FS_Switch_Status Switch_3;
    float Yaw_left;
    float Yaw_right;
};
/**
 * @brief 富斯i6x遥控器
 *
 */
class Class_Fs_i6x
{
public:
    void Init(UART_HandleTypeDef *huart);
    inline Enum_FS_Status Get_FS_Status();
    inline Enum_FS_Updata_Status Get_FS_Updata_Status();
    inline float Get_Right_X();
    inline float Get_Right_Y();
    inline float Get_Left_X();
    inline float Get_Left_Y();
    inline Enum_FS_Switch_Status Get_Switch_0();
    inline Enum_FS_Switch_Status Get_Switch_1();
    inline Enum_FS_Switch_Status Get_Switch_2();
    inline Enum_FS_Switch_Status Get_Switch_3();
    inline float Get_Yaw_left();
    inline float Get_Yaw_right();

    inline void Set_Right_X(float __Right_X);
    inline void Set_Right_Y(float __Right_Y);
    inline void Set_Left_X(float __Left_X);
    inline void Set_Left_Y(float __Left_Y);
    inline void Set_Switch_0(Enum_FS_Switch_Status __Switch_0);
    inline void Set_Switch_1(Enum_FS_Switch_Status __Switch_1);
    inline void Set_Switch_2(Enum_FS_Switch_Status __Switch_2);
    inline void Set_Switch_3(Enum_FS_Switch_Status __Switch_3);
    inline void Set_Yaw_left(float __Yaw_left);
    inline void Set_Yaw_right(float __Yaw_right);
    inline void Set_FS_Status(Enum_FS_Status __FS_Status);

    void FS_UART_RxCpltCallback(uint8_t *Rx_Data);
    void TIM1msMod50_Alive_PeriodElapsedCallback();


protected:
    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;

    //常量

    //摇杆偏移量
    float Rocker_Offset = 1023.0f;
    //摇杆总刻度
    float Rocker_Num = 783.0f;

    //内部变量
    //前一时刻的遥控器状态信息
    Struct_FS_UART_Data Now_UART_Rx_Data;
    //前一时刻的遥控器状态信息
    Struct_FS_UART_Data Pre_UART_Rx_Data;

    //当前时刻的遥控器接收flag
    uint32_t FS_Flag = 0;
    //前一时刻的遥控器接收flag
    uint32_t Pre_FS_Flag = 0;

    //遥控器50ms离线次数
    uint16_t Unline_Cnt = 0;
    
    //遥控器状态
    Enum_FS_Status FS_Status = FS_Status_DISABLE;
    //遥控器数据更新状态
    Enum_FS_Updata_Status FS_Updata_Status = FS_Status_DisUpdata;
    //富斯i6x对外接口信息
    Struct_FS_Data FS_Data;

    //写变量

    //读写变量

    //内部函数

    void Judge_Switch(Enum_FS_Switch_Status *Switch, uint16_t Status, uint16_t Pre_Status);
    void Judge_Switch_3(Enum_FS_Switch_Status *Switch, uint16_t Status, uint16_t Pre_Status);
    void Judge_Updata(Struct_FS_UART_Data Pre_UART_Rx_Data,Struct_FS_UART_Data Now_UART_Rx_Data);
    void FS_Data_Process();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取遥控器在线状态
 *
 * @return Enum_FS_Status 遥控器在线状态
 */
Enum_FS_Status Class_Fs_i6x::Get_FS_Status()
{
    return (FS_Status);
}

/**
 * @brief 获取遥控器数据更新状态
 *
 * @return Enum_FS_Updata_Status 遥控器在线状态
 */
Enum_FS_Updata_Status Class_Fs_i6x::Get_FS_Updata_Status()
{
    return (FS_Updata_Status);
}

/**
 * @brief 获取遥控器右侧x轴摇杆状态
 *
 * @return float 遥控器右侧x轴摇杆状态
 */
float Class_Fs_i6x::Get_Right_X()
{
    return (FS_Data.Right_X);
}

/**
 * @brief 获取遥控器右侧y轴摇杆状态
 *
 * @return float 遥控器右侧y轴摇杆状态
 */
float Class_Fs_i6x::Get_Right_Y()
{
    return (FS_Data.Right_Y);
}

/**
 * @brief 获取遥控器左侧x轴摇杆状态
 *
 * @return float 遥控器左侧x轴摇杆状态
 */
float Class_Fs_i6x::Get_Left_X()
{
    return (FS_Data.Left_X);
}

/**
 * @brief 获取遥控器左侧y轴摇杆状态
 *
 * @return float 遥控器左侧y轴摇杆状态
 */
float Class_Fs_i6x::Get_Left_Y()
{
    return (FS_Data.Left_Y);
}

/**
 * @brief 获取遥控器左1拨动开关状态
 * 
 */
Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_0()
{
    return (FS_Data.Switch_0);
}

/**
 * @brief 获取遥控器左2拨动开关状态
 * 
 */
Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_1()
{
    return (FS_Data.Switch_1);
}

/**
 * @brief 获取遥控器右1拨动开关状态
 * 
 */
Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_2()
{
    return (FS_Data.Switch_2);
}

/**
 * @brief 获取遥控器右2拨动开关状态
 * 
 */
Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_3()
{
    return (FS_Data.Switch_3);
}
/**
 * @brief 获取遥控器左侧yaw轴状态
 * 
 */
float Class_Fs_i6x::Get_Yaw_left()
{
    return (FS_Data.Yaw_left);
}
/**
 * @brief 获取遥控器右侧yaw轴状态
 * 
 */
float Class_Fs_i6x::Get_Yaw_right()
{
    return (FS_Data.Yaw_right);
}
/**
 * @brief 设置遥控器右侧x轴摇杆状态
 * 
 */
void Class_Fs_i6x::Set_Right_X(float __Right_X)
{
    FS_Data.Right_X = __Right_X;
}
/**
 * @brief 设置遥控器右侧y轴摇杆状态
 * 
 */
void Class_Fs_i6x::Set_Right_Y(float __Right_Y)
{
    FS_Data.Right_Y = __Right_Y;
}
/**
 * @brief 设置遥控器左侧x轴摇杆状态
 * 
 */
void Class_Fs_i6x::Set_Left_X(float __Left_X)
{
    FS_Data.Left_X = __Left_X;
}
/**
 * @brief 设置遥控器左侧y轴摇杆状态
 * 
 */
void Class_Fs_i6x::Set_Left_Y(float __Left_Y)
{
    FS_Data.Left_Y = __Left_Y;
}
/**
 * @brief 设置遥控器左1拨动开关状态
 * 
 */
void Class_Fs_i6x::Set_Switch_0(Enum_FS_Switch_Status __Switch_0)
{
    FS_Data.Switch_0 = __Switch_0;
}
/**
 * @brief 设置遥控器左2拨动开关状态
 * 
 */
void Class_Fs_i6x::Set_Switch_1(Enum_FS_Switch_Status __Switch_1)
{
    FS_Data.Switch_1 = __Switch_1;
}
/**
 * @brief 设置遥控器右1拨动开关状态
 * 
 */
void Class_Fs_i6x::Set_Switch_2(Enum_FS_Switch_Status __Switch_2)
{
    FS_Data.Switch_2 = __Switch_2;
}
/**
 * @brief 设置遥控器右2拨动开关状态
 * 
 */
void Class_Fs_i6x::Set_Switch_3(Enum_FS_Switch_Status __Switch_3)
{
    FS_Data.Switch_3 = __Switch_3;
}
/**
 * @brief 设置遥控器左侧yaw轴状态
 * 
 */
void Class_Fs_i6x::Set_Yaw_left(float __Yaw_left)
{
    FS_Data.Yaw_left = __Yaw_left;
}
/**
 * @brief 设置遥控器右侧yaw轴状态
 * 
 */
void Class_Fs_i6x::Set_Yaw_right(float __Yaw_right)
{
    FS_Data.Yaw_right = __Yaw_right;
}
/**
 * @brief 设置遥控器在线状态
 * 
 */
void Class_Fs_i6x::Set_FS_Status(Enum_FS_Status __FS_Status)
{
    FS_Status = __FS_Status;
}
#endif

