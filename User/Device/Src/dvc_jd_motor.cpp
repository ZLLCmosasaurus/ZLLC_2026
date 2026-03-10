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

int rACT_Register = 0x03E8; // 电机使能寄存器地址

int Gripper_Register_A = 0x03EA; // 夹爪轴，高字节为夹持速度，低字节为夹持位置
int Gripper_Register_B = 0x03EB; // 夹爪轴，高字节为夹持力矩，低字节为夹持触发标志

int Roll_Register_C = 0x03EC; // 旋转绝对位置，2 字节有符号数，范围 -32768~32767，360 对应 360°
int Roll_Register_D = 0x03ED; // 高字节为旋转扭矩，低字节为旋转速度
int Roll_Register_E = 0x03EE; // 高字节为旋转圈数，低字节为运动触发（0x01 为绝对位置运动，0x02 为相对位置运动）

void Class_Jodell_Motor::Init(UART_HandleTypeDef *UART, int __Slave_Address)
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
    agile_modbus_set_slave(this->Slave_Address);
}

void Class_Jodell_Motor::Jodell_Motor_UART_RxCplt_Callback(uint8_t *Rx_Data, uint16_t Max_Length)
{
    Flag += 1;

    Data_Process();
}

void Class_Jodell_Motor::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_Status = Jodell_Motor_Status_OFFLINE;
    }
    else
    {
        Motor_Status = Jodell_Motor_Status_ONLINE;
    }

    Pre_Flag = Flag;
}

void Class_Jodell_Motor::Modbus_Send_Request()
{
    HAL_UART_Transmit(UART_Manage_Object->UART_Handler, UART_Manage_Object->Tx_Buffer, UART_Manage_Object->Tx_Length, 10);
}

void Class_Jodell_Motor::TIM_UART_Tx_PeriodElapsedCallback()
{
    switch (Jodell_Control_Type)
    {
        case (Jodell_Control_Type_rACT)
        // 电机使能帧
        {
            Modbus_Clear_Receive_Buffer();
            if(Motor_Control_Status == Jodell_Motor_Control_ENABLE)
            {
                agile_modbus_serialize_write_registers(ctx, rACT_Register, 1, 0x0001);
            }
            else if(Motor_Control_Status == Jodell_Motor_Control_DISABLE)
            {
                agile_modbus_serialize_write_registers(ctx, rACT_Register, 1, 0x0000);
            }

            break;
        }
        case (Jodell_Control_Type_GRIPPER)
        {
            // 夹爪张合控制帧
            uint16_t reg_data[2];

            Modbus_Clear_Receive_Buffer();
            if(Gripper_Status == Gripper_Clamp)
            // 夹爪闭合
            {
                reg_data[0] = (0x80) << 8 | Clamp_Position; // 半速加持到指定位置
                reg_data[1] = (0x80) << 8 | 0x01; // 半力触发
            }
            else if(Gripper_Clamp == Gripper_Release)
            // 夹爪张开
            {
                reg_data[0] = (0x80) << 8 | 0x00; // 移动到0位置即张开
                reg_data[1] = (0x80) << 8 | 0x01;
            }
            
            agile_modbus_serialize_write_registers(ctx, Gripper_Register_A, 2, reg_data);

            break;
        }
        case (Jodell_Control_Type_ROLL_ANGLE)
        {
            // Roll轴角度控制帧
            uint16_t reg_data[4];

            Modbus_Clear_Receive_Buffer();

            uint8_t target_omega_byte = (uint8_t)((Target_Omega / MAX_OMEGA) * 255.0f);
            uint8_t target_torque_byte = (uint8_t)((Target_Torque / MAX_TORQUE) * 255.0f);
            reg_data[0] = (int16_t)(Target_Roll * 180.0f / PI);
            reg_data[1] = (target_torque_byte << 8) | target_omega_byte;
            reg_data[2] = 0x0000; // 旋转相对位置置0
            reg_data[3] = (0x01 << 8) | 0x01; // 绝对位置模式，默认1圈

            agile_modbus_serialize_write_registers(ctx, Roll_Register_C, 4, reg_data);

            break;
        }

    }

}