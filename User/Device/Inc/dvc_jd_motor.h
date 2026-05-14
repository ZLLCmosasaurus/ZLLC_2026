#include <cmath>
#include <string.h>
#include "drv_uart.h"
#include "drv_math.h"
#include "agile_modbus.h"
#include "agile_modbus_rtu.h"

enum Jodell_Motor_Comm_Status
{
    Jodell_Motor_Comm_OFFLINE = 0,
    Jodell_Motor_Comm_ONLINE
};

enum Jodell_Motor_Working_Status
{
    Jodell_Motor_Working_DISABLE = 0,
    Jodell_Motor_Working_ENABLE
};

enum Jodell_Motor_Control_Status
{
    Jodell_Motor_Control_ENABLE = 0,
    Jodell_Motor_Control_DISABLE,
};

enum Jodell_Tx_Frame_Type
{
    Jodell_Tx_Frame_READ,
    Jodell_Tx_Frame_WRITE
};

struct Jodell_Roll_Rx_Data
{
    bool Enable_Status = false;
    float Now_Angle = 0.0f;
    float Now_Absolute_Angle = 0.0f;
    float Now_Relative_Angle = 0.0f;
    float Now_Omega = 0.0f;
    float Now_Torque = 0.0f;
    int8_t Now_Turns = 0;
    uint8_t Motion_Mode = 0;
};

struct Jodell_Gripper_Rx_Data
{
    bool Enable_Status = false;
    uint8_t Now_Position = 0;
    float Now_Omega = 0.0f;
    float Now_Torque = 0.0f;
};

class Class_Jodell_Motor
{
public:
    void Init(UART_HandleTypeDef *huart, int __Slave_Address);

    void Jodell_Motor_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Length);

    void TIM1msMod50_Alive_PeriodElapsedCallback();

    void TIM_UART_Tx_PeriodElapsedCallback();

    inline void Set_Motor_Control_Status(Jodell_Motor_Control_Status __Motor_Control_Status);
    inline Jodell_Motor_Working_Status Get_Motor_Working_Status();

    inline void Set_Target_Roll(float __Target_Roll);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);

    inline float Get_Target_Roll();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();

    inline float Get_Now_Omega();
    inline float Get_Now_Roll();
    inline uint8_t Get_Now_Gripper_Position();

    inline void Set_Gripper_Position(uint8_t __Target_Gripper_Position);
    inline uint8_t Get_Gripper_Position();

private:
    Struct_UART_Manage_Object *UART_Manage_Object;

    agile_modbus_rtu_t ctx_rtu;
    agile_modbus_t *ctx = &ctx_rtu._ctx;
    uint8_t Modbus_Tx_Buffer[128];
    uint8_t Modbus_Rx_Buffer[128];
    int Slave_Address;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Jodell_Motor_Comm_Status Motor_Communication_Status = Jodell_Motor_Comm_OFFLINE;
    Jodell_Motor_Control_Status Motor_Control_Status = Jodell_Motor_Control_DISABLE;
    Jodell_Motor_Working_Status Motor_Working_Status = Jodell_Motor_Working_DISABLE;
    Jodell_Tx_Frame_Type Motor_Tx_Frame_Type = Jodell_Tx_Frame_WRITE;

    float Target_Roll = 0.0f;
    int16_t Target_Absolute_Position_Deg = 0;
    int8_t Target_Absolute_Position_Turns = 0;
    float Target_Omega = 17.4527f;
    float Target_Torque = 1.5f;
    uint8_t Target_Gripper_Position = 0;

    // 堵转检测相关变量
    uint16_t Locked_cnt = 0;
    bool is_locked = false;
    // 堵转时的当前角度
    float Locked_Angle;

    Jodell_Gripper_Rx_Data Gripper_Data;
    Jodell_Roll_Rx_Data Roll_Data;

    void Data_Process(uint16_t *data, int regs);

    void Modbus_Clear_Receive_Buffer();

    // 检测堵转函数
    void Jodell_Safety_Check();
};

inline uint8_t Class_Jodell_Motor::Get_Gripper_Position()
{
    return Target_Gripper_Position;
}

inline float Class_Jodell_Motor::Get_Target_Omega()
{
    return Target_Omega;
}

inline float Class_Jodell_Motor::Get_Target_Roll()
{
    return Target_Roll;
}

inline float Class_Jodell_Motor::Get_Target_Torque()
{
    return Target_Torque;
}

inline float Class_Jodell_Motor::Get_Now_Omega()
{
    return Roll_Data.Now_Omega;
}

inline float Class_Jodell_Motor::Get_Now_Roll()
{
    return Roll_Data.Now_Angle;
}

inline uint8_t Class_Jodell_Motor::Get_Now_Gripper_Position()
{
    return Gripper_Data.Now_Position;
}

inline void Class_Jodell_Motor::Set_Gripper_Position(uint8_t __Target_Gripper_Position)
{
    Target_Gripper_Position = __Target_Gripper_Position;
}

inline void Class_Jodell_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Jodell_Motor::Set_Target_Roll(float __Target_Roll)
{
    Target_Roll = __Target_Roll;
    const float total_degrees = __Target_Roll * 180.0f / PI;
    const int32_t rounded_total_degrees = static_cast<int32_t>(std::lround(total_degrees));

    int32_t turns = 0;
    int32_t absolute_position_deg = rounded_total_degrees;

    // Absolute mode uses "turn count + signed degree position". This keeps
    // the continuous target monotonic after crossing 360° while preserving
    // small negative targets as negative positions instead of wrapping them.
    if (absolute_position_deg > 360)
    {
        turns = static_cast<int32_t>(std::floor(static_cast<float>(absolute_position_deg) / 360.0f));
        absolute_position_deg -= turns * 360;
    }
    else if (absolute_position_deg < -360)
    {
        turns = static_cast<int32_t>(std::ceil(static_cast<float>(absolute_position_deg) / 360.0f));
        absolute_position_deg -= turns * 360;
    }

    if (turns > 127)
    {
        turns = 127;
    }
    else if (turns < -128)
    {
        turns = -128;
    }

    if (absolute_position_deg > 32767)
    {
        absolute_position_deg = 32767;
    }
    else if (absolute_position_deg < -32768)
    {
        absolute_position_deg = -32768;
    }

    Target_Absolute_Position_Deg = static_cast<int16_t>(absolute_position_deg);
    Target_Absolute_Position_Turns = static_cast<int8_t>(turns);
}

inline void Class_Jodell_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

inline void Class_Jodell_Motor::Set_Motor_Control_Status(Jodell_Motor_Control_Status __Motor_Control_Status)
{
    Motor_Control_Status = __Motor_Control_Status;
}

inline Jodell_Motor_Working_Status Class_Jodell_Motor::Get_Motor_Working_Status()
{
    return Motor_Working_Status;
}
