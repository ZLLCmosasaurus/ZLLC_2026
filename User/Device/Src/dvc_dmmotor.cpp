/**
 * @file dvc_dmmotor.cpp
 * @author cjw by yssickjgd
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dmmotor.h"
#include "dvc_dwt.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//清除电机错误信息
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
//使能电机
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
//失能电机
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
//保存当前电机位置为零点
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (DM_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN1_0xxf1_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN1_0xxf2_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN1_0xxf3_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN1_0xxf4_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN1_0xxf5_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN1_0xxf6_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN1_0xxf7_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN1_0xxf8_Tx_Data;
        }
        break;
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (DM_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN2_0xxf1_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN2_0xxf2_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN2_0xxf3_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN2_0xxf4_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN2_0xxf5_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN2_0xxf6_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN2_0xxf7_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN2_0xxf8_Tx_Data;
        }
        break;
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式, 默认角度
 * @param __Position_Offset 编码器偏移, 默认0
 * @param __Omega_Max 最大速度, 调参助手设置
 * @param __Torque_Max 最大扭矩, 调参助手设置
 */
void Class_DM_Motor_J4310::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method, int32_t __Position_Offset, float __Omega_Max, float __Torque_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    DM_Motor_Control_Method = __Control_Method;
    Position_Offset = __Position_Offset;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);

    for(int i = 0; i < 10;i++){
        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, DM_Motor_CAN_Message_Enter, 8);
    }

}

/**
 * @brief 数据处理过程
 *
 */
void Class_DM_Motor_J4310::Data_Process()
{
    //数据处理过程
    int32_t delta_position;
    uint16_t tmp_position, tmp_omega, tmp_torque;
    Struct_DM_Motor_CAN_Rx_Data *tmp_buffer = (Struct_DM_Motor_CAN_Rx_Data *)CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Position_Reverse, &tmp_position);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    Data.CAN_ID = tmp_buffer->CAN_ID;

    if(Data.CAN_ID == (CAN_ID + 0x10 + 0x03)){
        this->Flag ++;
    }

    //计算圈数与总角度值
    delta_position = tmp_position - Data.Pre_Position;
    if (delta_position < -(Position_Max / 2))
    {
        //正方向转过了一圈
        Data.Total_Round++;
    }
    else if (delta_position > (Position_Max / 2))
    {
        //反方向转过了一圈
        Data.Total_Round--;
    }
    Data.Total_Position = Data.Total_Round * Position_Max + tmp_position + Position_Offset;

    //计算电机本身信息
    Data.Now_Angle = (float)tmp_position / (float)Position_Max * 2.0f * PI;
    Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -Omega_Max, Omega_Max);
    Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
    Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    //存储预备信息
    Data.Pre_Position = tmp_position;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DM_Motor_J4310::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_DM_Motor_J4310::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        DM_Motor_Status = DM_Motor_Status_DISABLE;
        for (int i = 0; i < 3; i++)
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, DM_Motor_CAN_Message_Clear_Error, 8); // 清除错误状态
            DWT_Delay_us(50);
        }

        for (int i = 0; i < 3; i++)
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, DM_Motor_CAN_Message_Enter, 8); // 使能电机
            DWT_Delay_us(50);
        }
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
       // PID_Torque.Set_Integral_Error(0.0f);
    }
    else
    {
        //电机保持连接
        DM_Motor_Status = DM_Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断发送出去的回调函数
 *
 */
void Class_DM_Motor_J4310::TIM_Process_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_POSITION):
    {
        uint16_t tmp_position = Math_Float_To_Int(Output_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Output_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(MIT_K_P, 0.0f, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Output_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OMEGA):
    {
        uint16_t tmp_position = Math_Float_To_Int(Output_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Output_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Output_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_TORQUE):
    {
        uint16_t tmp_position = Math_Float_To_Int(Output_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Output_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = 0;
        uint16_t tmp_torque = Math_Float_To_Int(Output_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_POSITION_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Output_Angle, sizeof(float));

        memcpy(&CAN_Tx_Data[4], &Output_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Output_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x03, CAN_Tx_Data, 8);
    }
    break;
    default:
    {
    }
    break;
    }
}

void Class_DM_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
        case (DM_Motor_Control_Method_MIT_TORQUE):
        {
            if(DM_Motor_Control_Alg == DM_PID_Omega){
                PID_Omega.Set_Now(Transform_Omega);
                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.TIM_Adjust_PeriodElapsedCallback();

                Out = PID_Omega.Get_Out();
                Output_Torque = Torque_Max / 2048.0f * Out;
            }
            else if(DM_Motor_Control_Alg == DM_PID_Angle){
                PID_Angle.Set_Now(Transform_Angle);
                PID_Angle.Set_Target(Target_Angle);
                PID_Angle.TIM_Adjust_PeriodElapsedCallback();
                Target_Omega = PID_Angle.Get_Out();

                PID_Omega.Set_Now(Transform_Omega);
                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.TIM_Adjust_PeriodElapsedCallback();

                Out = PID_Omega.Get_Out();
                float tmp_Torque = J * Transform_Target_Acc + B * Transform_Target_Vel;  //简单的动力学补偿，参数需要根据实际负载测量后赋值
                Out += tmp_Torque / (Torque_Max / 2048.0f);

                Output_Torque = Torque_Max / 2048.0f * Out;
            }
            else if(DM_Motor_Control_Alg == DM_Motor_DISANLE){
                Out           = 0.0f;
                Output_Angle  = 0.0f;
                Output_Omega  = 0.0f;
                Output_Torque = 0.0f;
            }
            break;
        }
        default:
        {
            Out           = 0.0f;
            Output_Angle  = 0.0f;
            Output_Omega  = 0.0f;
            Output_Torque = 0.0f;
            break;
        }
    }
}

void Class_DM_Motor_J4310::Compensite_Out(float Compensite_Value)
{
    Output_Torque += Compensite_Value;
    Math_Constrain(&Output_Torque, -Torque_Max, Torque_Max);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/