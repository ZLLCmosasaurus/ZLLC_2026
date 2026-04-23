/**
 * @file dvc_LKmotor.cpp
 * @author lez
 * @brief lk电机配置与操作
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_lkmotor.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//清除电机错误信息
uint8_t LK_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
//使能电机
uint8_t LK_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
//失能电机
uint8_t LK_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
//保存当前电机位置为零点
uint8_t LK_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (LK_Motor_ID_0x141):
        {
            tmp_tx_data_ptr = CAN1_0x141_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x142):
        {
            tmp_tx_data_ptr = CAN1_0x142_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x143):
        {
            tmp_tx_data_ptr = CAN1_0x143_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x144):
        {
            tmp_tx_data_ptr = CAN1_0x144_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x145):
        {
            tmp_tx_data_ptr = CAN1_0x145_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x146):
        {
            tmp_tx_data_ptr = CAN1_0x146_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x147):
        {
            tmp_tx_data_ptr = CAN1_0x147_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x148):
        {
            tmp_tx_data_ptr = CAN1_0x148_Tx_Data;
        }
        break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (__CAN_ID)
        {
        case (LK_Motor_ID_0x141):
        {
            tmp_tx_data_ptr = CAN2_0x141_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x142):
        {
            tmp_tx_data_ptr = CAN2_0x142_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x143):
        {
            tmp_tx_data_ptr = CAN2_0x143_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x144):
        {
            tmp_tx_data_ptr = CAN2_0x144_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x145):
        {
            tmp_tx_data_ptr = CAN2_0x145_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x146):
        {
            tmp_tx_data_ptr = CAN2_0x146_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x147):
        {
            tmp_tx_data_ptr = CAN2_0x147_Tx_Data;
        }
        break;
        case (LK_Motor_ID_0x148):
        {
            tmp_tx_data_ptr = CAN2_0x148_Tx_Data;
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
void Class_LK_Motor::Init(CAN_HandleTypeDef *hcan, Enum_LK_Motor_ID __CAN_ID,  float __Omega_Max, int32_t __Position_Offset, float __Current_Max, Enum_LK_Motor_Control_Method __Control_Method)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    LK_Motor_Control_Method = __Control_Method;
    Position_Offset = __Position_Offset;
    Omega_Max = __Omega_Max;
    Current_Max = __Current_Max;
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 数据处理过程
 *
 */
void Class_LK_Motor::Data_Process()
{
    //数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_current;
    Struct_LK_Motor_CAN_Rx_Data *tmp_buffer = (Struct_LK_Motor_CAN_Rx_Data *)CAN_Manage_Object->Rx_Buffer.Data;
    
    //处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, &tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, &tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse, &tmp_current);

    //计算圈数与总角度值
    if(Start_Flag==0)
    {
        delta_encoder = tmp_encoder - Data.Pre_Encoder;
        if (delta_encoder < -(Position_Max / 2))
        {
            //正方向转过了一圈
            Data.Total_Round++;
        }
        else if (delta_encoder > (Position_Max / 2))
        {
            //反方向转过了一圈
            Data.Total_Round--;
        }        
    }
    Data.Total_Encoder = Data.Total_Round * Position_Max + tmp_encoder + Position_Offset;

    //计算电机本身信息
    Data.CMD_ID = tmp_buffer->CMD_ID;
    Data.Now_Angle = (float)Data.Total_Encoder / (float)Position_Max *360.0f; 
    Data.Now_Radian = Data.Now_Angle * DEG_TO_RAD;
    Data.Now_Omega_Angle = tmp_omega * RPM_TO_DEG;
    Data.Now_Omega_Radian = tmp_omega *RPM_TO_RADPS; 
    Data.Now_Current = Math_Int_To_Float(tmp_current, 0, (1 << 12) - 1, -Current_Max, Current_Max); 
    Data.Now_Temperature = tmp_buffer->Temperature_Centigrade;  

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
    if(Start_Flag==0)   Start_Flag = 1;
}

void Class_LK_Motor::Output(void)
{
    switch(LK_Motor_Control_ID)
    {
        case(LK_Motor_Control_Torque):
            CAN_Tx_Data[0] = LK_Motor_Control_Torque;
            CAN_Tx_Data[4] = (int16_t)Out; 
            CAN_Tx_Data[5] = (int16_t)Out >> 8;
        break;
        case(LK_Motor_Control_Run):
            CAN_Tx_Data[0] = LK_Motor_Control_Run;
        break;
        case(LK_Motor_Control_Stop):
            CAN_Tx_Data[0] = LK_Motor_Control_Stop;
        break;
        case(LK_Motor_Control_Shut_Down):
            CAN_Tx_Data[0] = LK_Motor_Control_Shut_Down;
        break;
        default:
        break;
    }   
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_LK_Motor::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    this->Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_LK_Motor::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        LK_Motor_Status = LK_Motor_Status_DISABLE;
    }
    else
    {
        //电机保持连接
        LK_Motor_Status = LK_Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断发送出去的回调函数
 *
 */
void Class_LK_Motor::TIM_Process_PeriodElapsedCallback()
{
    switch (LK_Motor_Control_Method)
    {
        case (LK_Motor_Control_Method_TORQUE):
        {        
            Out = Target_Torque*Torque_Current/Current_Max*Current_Max_Cmd;
        }
        break;
        case (LK_Motor_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            Out = Target_Current;
        }
        break;
        case (LK_Motor_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            Out = Target_Current;
        }
        break;
        default:
        {
            Out = 0.0f;
        }
        break;
    }    
    //发送数据
    Output();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

