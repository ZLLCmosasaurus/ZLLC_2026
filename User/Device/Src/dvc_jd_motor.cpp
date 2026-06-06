/**
 * @file dvc_jd_motor.cpp
 * @author hsl
 * @brief jodell motor
 * @version 0.1
 * @date 2026-3-2
 *
 * @copyright ZLLC 2026
 *
 */

#include "dvc_jd_motor.h"

#define MAX_OMEGA 17.4527f
#define MAX_TORQUE 1.5f

#define Gripper_rACT_Write_Register 0x03E8 // 夹持端使能寄存器地址
#define Roll_rACT_Write_Register 0x03E9    // 旋转轴使能寄存器地址
#define Gripper_Write_Register_A 0x03EA    // 夹爪轴功能设置寄存器，高字节为夹持速度，低字节为夹持位置
#define Gripper_Write_Register_B 0x03EB    // 夹爪轴功能设置寄存器，高字节为夹持力矩，低字节为夹持触发标志
#define Roll_Write_Register_C 0x03EC       // 旋转轴功能设置寄存器，旋转绝对位置，2 字节有符号数，范围 -32768~32767，360 对应 360°
#define Roll_Write_Register_D 0x03ED       // 旋转轴功能设置寄存器，高字节为旋转扭矩，低字节为旋转速度
#define Roll_Write_Register_E 0x03EE       // 旋转轴功能设置寄存器，高字节为旋转圈数，低字节为运动触发（0x01 为绝对位置运动，0x02 为相对位置运动）

#define Gripper_rACT_Read_Register 0x07D0 // 夹爪轴使能状态查询寄存器地址
#define Roll_rACT_Read_Register 0x07D1    // 旋转轴使能状态查询寄存器
#define Gripper_Read_Register_1 0x07D2    // 夹爪轴状态查询寄存器，高字节为速度，低字节为位置
#define Gripper_Read_Register_2 0x07D3    // 夹爪轴状态查询寄存器，高字节为电流值
#define Roll_Read_Register_1 0x07D4       // 旋转轴状态查询寄存器，绝对位置
#define Roll_Read_Register_2 0x07D5       // 旋转轴状态查询寄存器，高字节扭矩，低字节速度

void Class_Jodell_Motor::Init(UART_HandleTypeDef *huart, int __Slave_Address)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }

    Slave_Address = __Slave_Address;

    agile_modbus_rtu_init(&ctx_rtu, Modbus_Tx_Buffer, sizeof(Modbus_Tx_Buffer), Modbus_Rx_Buffer, sizeof(Modbus_Rx_Buffer));
    agile_modbus_set_slave(ctx, this->Slave_Address);
}

void Class_Jodell_Motor::Jodell_Motor_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Length)
{
    // 搬运接收数据到ctx结构体
    memcpy(ctx->read_buf, Rx_Data, Length);
    // Modbus解析返回数据
    int rc = agile_modbus_receive_judge(ctx, Length, AGILE_MODBUS_MSG_CONFIRMATION);
    if (rc > 0)
    {
        // 通信正常，更新通信Flag
        Flag += 1;

        // 获取功能码
        uint8_t function = ctx->read_buf[ctx->backend->header_length];
        if (function == AGILE_MODBUS_FC_READ_HOLDING_REGISTERS)
        {
            // 返回的数据
            uint16_t read_data[8];
            // 读取的寄存器个数
            int regs_read = agile_modbus_deserialize_read_registers(ctx, Length, read_data);

            if (regs_read >= 8)
                Data_Process(read_data, regs_read);

            // 下一帧为写入帧
            Motor_Tx_Frame_Type = Jodell_Tx_Frame_WRITE;
        }
        else if (function == AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
        {
            // 下一帧为读取帧
            Motor_Tx_Frame_Type = Jodell_Tx_Frame_READ;
        }
    }
}

void Class_Jodell_Motor::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_Communication_Status = Jodell_Motor_Comm_OFFLINE;
    }
    else
    {
        Motor_Communication_Status = Jodell_Motor_Comm_ONLINE;
    }

    Pre_Flag = Flag;
}

HAL_StatusTypeDef HAL_Tx_Status;
void Class_Jodell_Motor::TIM_UART_Tx_PeriodElapsedCallback()
{
    // 清空缓存区
    Modbus_Clear_Receive_Buffer();

    // 记录数据包长度
    int Data_Length = 0;

    // 写入数据包相关变量
    uint16_t write_data[8] = {0};
    uint16_t write_addr = 0;
    int write_nb = 0;

    // 查询数据包相关变量
    uint16_t read_addr = 0;
    int read_nb = 0;

    if (Motor_Tx_Frame_Type == Jodell_Tx_Frame_WRITE)
    {
        switch (Motor_Control_Status)
        {
        case Jodell_Motor_Control_ENABLE:
        {
            // 每次写入前检测是否堵转
            if(is_locked)
            {
                Set_Target_Roll(Locked_Angle);
            }

            write_addr = Gripper_rACT_Write_Register;

            write_nb = 8;

            // 夹持端使能 0x03E8
            write_data[0] = 0x0001;
            // 旋转端使能 0x03E9
            write_data[1] = 0x0001;
            // 夹持端位置以及运动模式 0x03EA 0x03EB
            write_data[2] = (0xF0 << 8) | Target_Gripper_Position;
            write_data[3] = (0x80 << 8) | 0x01;
            // 旋转端位置，速度以及运动模式 0x03EC 0x03ED
            write_data[4] = static_cast<uint16_t>(Target_Absolute_Position_Deg);
            write_data[5] = ((uint8_t)((Target_Torque / MAX_TORQUE) * 255.0f) << 8) |
                            (uint8_t)((Target_Omega / MAX_OMEGA) * 255.0f);
            write_data[6] = 0x0000;                          // 0x03EE
            write_data[7] = (static_cast<uint16_t>(static_cast<uint8_t>(Target_Absolute_Position_Turns)) << 8) | 0x01; // 0x03EF

            break;
        }

        case Jodell_Motor_Control_DISABLE:
        {
            write_addr = Gripper_rACT_Write_Register;

            write_nb = 2;

            write_data[0] = 0x0000;
            write_data[1] = 0x0000;

            break;
        }

        default:
            return;
        }

        Data_Length = agile_modbus_serialize_write_registers(ctx, write_addr, write_nb, write_data);
    }
    else if (Motor_Tx_Frame_Type == Jodell_Tx_Frame_READ)
    {
        read_addr = Gripper_rACT_Read_Register;
        read_nb = 8;

        Data_Length = agile_modbus_serialize_read_registers(ctx, read_addr, read_nb);
    }

    // UART发送
    if (Data_Length > 0)
    {
        // memcpy(UART_Manage_Object->Tx_Buffer, ctx->send_buf, Data_Length);
        UART_Send_Data(UART_Manage_Object->UART_Handler, ctx->send_buf, Data_Length);
        // HAL_UART_Transmit_DMA(UART_Manage_Object->UART_Handler, ctx->send_buf, Data_Length);
    }
}

void Class_Jodell_Motor::Modbus_Clear_Receive_Buffer()
// 清空接收缓存区
{
    memset(Modbus_Rx_Buffer, 0, sizeof(Modbus_Rx_Buffer));
}

void Class_Jodell_Motor::Data_Process(uint16_t *data, int regs)
{
    // 电机使能状态数据
    uint8_t gripper_enable_status = ((data[0] & 0xFF) >> 3) & 0x03;
    Gripper_Data.Enable_Status = (gripper_enable_status == 0x03 ? true : false);
    uint8_t roll_enable_status = ((data[1] & 0xFF) >> 3) & 0x03;
    Roll_Data.Enable_Status = (roll_enable_status == 0x03 ? true : false);

    Gripper_Data.Fault_Code = static_cast<uint8_t>(data[0] >> 8);
    Gripper_Data.Has_Fault = (Gripper_Data.Fault_Code != Jodell_Motor_Fault_NONE);
    Roll_Data.Fault_Code = static_cast<uint8_t>(data[1] >> 8);
    Roll_Data.Has_Fault = (Roll_Data.Fault_Code != Jodell_Motor_Fault_NONE);

    // 更新电机类中的电机运行状态
    Motor_Working_Status = (Gripper_Data.Enable_Status && Roll_Data.Enable_Status ? Jodell_Motor_Working_ENABLE : Jodell_Motor_Working_DISABLE);

    // 夹爪数据
    uint8_t gripper_omega_raw = data[2] >> 8;
    uint8_t gripper_pos_raw = data[2] & 0xFF;
    uint8_t gripper_torque_raw = data[3] >> 8;

    Gripper_Data.Now_Omega = ((float)gripper_omega_raw / 255.0f) * MAX_OMEGA;
    Gripper_Data.Now_Position = gripper_pos_raw;
    Gripper_Data.Now_Torque = ((float)gripper_torque_raw / 255.0f) * MAX_TORQUE;

    // 旋转轴数据
    int16_t roll_pos_raw = (int16_t)data[4];
    uint8_t roll_torque_raw = data[5] >> 8;
    uint8_t roll_omega_raw = data[5] & 0xFF;
    uint8_t roll_motion_mode = data[6] & 0xFF;
    int8_t roll_turns_raw = static_cast<int8_t>(data[6] >> 8);
    int16_t roll_relative_pos_raw = static_cast<int16_t>(data[7]);

    Roll_Data.Now_Absolute_Angle = ((float)roll_pos_raw / 180.0f) * PI;
    Roll_Data.Now_Relative_Angle = ((float)roll_relative_pos_raw / 180.0f) * PI;
    Roll_Data.Now_Turns = roll_turns_raw;
    Roll_Data.Motion_Mode = roll_motion_mode;
    Roll_Data.Now_Angle = ((float)(roll_turns_raw * 360 + roll_pos_raw) / 180.0f) * PI;
    Roll_Data.Now_Torque = ((float)roll_torque_raw / 255.0f) * MAX_TORQUE;
    Roll_Data.Now_Omega = ((float)roll_omega_raw / 255.0f) * MAX_OMEGA;

    // 每次返回数据后堵转检测
    Jodell_Safety_Check();
}

void Class_Jodell_Motor::Jodell_Safety_Check()
{
    float Now_Total_Angle = Roll_Data.Now_Absolute_Angle + Roll_Data.Now_Turns * 2*PI;
    // 检测当前位置和目标位置以及扭矩值并计数
    if(fabs(Now_Total_Angle - Target_Roll) >= 0.5f && Roll_Data.Now_Torque >= 1.0f)
    {
        if(Locked_cnt < 25)
        {
            Locked_cnt++;
        }
    }
    else
    {
        if(Locked_cnt > 0)
        {
            Locked_cnt--;
        }
    }

    if(Locked_cnt >= 25)
    {
        if(!is_locked)
        {
            is_locked = true;
            Locked_Angle = Now_Total_Angle;
        }
    }
    else if(Locked_cnt == 0)
    {
        is_locked = false;
    }
}

void Class_Jodell_Motor::Jodell_Error_Check_Buzzer()
{
    if(Get_Motor_Has_Fault())
    {
        if(Get_Gripper_Has_Fault() && !Get_Roll_Has_Fault())
        {
            buzzer_setTask(&buzzer, BUZZER_CALIBRATING_PRIORITY);
        }
        else if(Get_Roll_Has_Fault() && !Get_Gripper_Has_Fault())
        {
            buzzer_setTask(&buzzer, BUZZER_CALIBRATED_PRIORITY);
        }
        else
        {
            buzzer_setTask(&buzzer, BUZZER_POKEMON_HEALED);
        }
    }
}
