#include <string.h>
#include "drv_uart.h"
#include "drv_math.h"
#include "agile_modbus.h"
#include "agile_modbus_rtu.h"

enum Enum_Gripper_Status
{
    Gripper_Clamp = 0, // 夹紧
    Gripper_Release    // 松开
};

enum Jodell_Motor_Status
{
    Jodell_Motor_Status_OFFLINE = 0,
    Jodell_Motor_Status_ONLINE
};

enum Jodell_Motor_Control_Status
{
    Jodell_Motor_Control_DISABLE = 0,
    Jodell_Motor_Control_ENABLE
};

enum Jodell_Control_Type
{
    Jodell_Control_Type_ENABLE = 0,
    Jodell_Control_Type_DISABLE,
    Jodell_Control_Type_GRIPPER,
    Jodell_Control_Type_ROLL_ANGLE
};

struct Jodell_Roll_Rx_Data
// 查询帧返回的电机旋转轴数据
{
    bool Enable_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
};

struct Jodell_Gripper_Rx_Data
// 查询帧返回的夹爪轴数据
{
    bool Enable_Status;
    float Now_Position;
    float Now_Omega;
    float Now_Torque;
};

class Class_Jodell_Motor
{
public:
    void Init(UART_HandleTypeDef *huart, int __Slave_Address);

    void Jodell_Motor_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Length);

    void TIM1msMod50_Alive_PeriodElapsedCallback();

    void TIM_UART_Tx_PeriodElapsedCallback();

    inline void Set_Target_Roll(float __Target_Roll);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);

    inline float Get_Target_Roll();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();

    inline void Set_Gripper_Status(Enum_Gripper_Status __Gripper_Status);
    inline void Set_Gripper_Clamp(uint8_t __Clamp_Position);
    inline Enum_Gripper_Status Get_Gripper_Status();

private:

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;

    // Modbus结构体
    agile_modbus_rtu_t ctx_rtu;
    agile_modbus_t *ctx = &ctx_rtu._ctx;
    // Modbus发送缓冲区
    uint8_t Modbus_Tx_Buffer[256];
    // Modbus接收缓冲区
    uint8_t Modbus_Rx_Buffer[256];
    // 电机的从机地址
    int Slave_Address;

    // 当前时刻的电机通信flag
    uint32_t Flag = 0;
    // 前一时刻的电机通信flag
    uint32_t Pre_Flag = 0;

    Jodell_Motor_Status Motor_Status = Jodell_Motor_Status_OFFLINE;
    Jodell_Motor_Control_Status Motor_Control_Status = Jodell_Motor_Control_DISABLE;
    Jodell_Control_Type Control_Type = Jodell_Control_Type_ENABLE;

    float Target_Roll;
    float Target_Omega;
    float Target_Torque;
    Enum_Gripper_Status Gripper_Status;
    uint8_t Target_Clamp_Position;

    // 查询帧电机返回的数据
    Jodell_Gripper_Rx_Data Gripper_Data;
    Jodell_Roll_Rx_Data Roll_Data;

    void Data_Process(uint16_t *data, int regs);

    void Modbus_Clear_Receive_Buffer();
};