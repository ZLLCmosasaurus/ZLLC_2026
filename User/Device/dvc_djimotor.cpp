/**
 * @file dvc_motor.cpp
 * @author lez by yssickjgd
 * @brief 大疆电机配置与操作
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "buzzer.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (DJI_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x209):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x20A):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x20B):
        {
            tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
        }
        break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (__CAN_ID)
        {
        case (DJI_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x209):
        {
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x20A):
        {
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x20B):
        {
            tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
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
 * @param __DJI_Motor_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Omega_Max 最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 */
void Class_DJI_Motor_GM6020::Init(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, int32_t __Encoder_Offset, float __Omega_Max)
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
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Encoder_Offset = __Encoder_Offset;
    Omega_Max = __Omega_Max;
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DJI_Motor_GM6020::Data_Process()
{
    //数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque, tmp_temperature;
    Struct_DJI_Motor_CAN_Data *tmp_buffer = (Struct_DJI_Motor_CAN_Data *)CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Torque_Reverse, (void *)&tmp_torque);

    //计算圈数与总编码器值
    if(Start_Falg==1)
    {
        delta_encoder = tmp_encoder - Data.Pre_Encoder;
        if (delta_encoder < -Encoder_Num_Per_Round / 2)
        {
            //正方向转过了一圈
            Data.Total_Round++;
        }
        else if (delta_encoder > Encoder_Num_Per_Round / 2)
        {
            //反方向转过了一圈
            Data.Total_Round--;
        }        
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;

    //计算电机本身信息
    // Data.Now_Angle = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round * 360.0f;
    // Data.Now_Radian = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI;
    Data.Now_Angle = (float)tmp_encoder / (float)Encoder_Num_Per_Round * 360.0f;
    Data.Now_Radian = (float)tmp_encoder / (float)Encoder_Num_Per_Round * 2.0f * PI;
    // Data.Now_Omega_Angle = (float)(Data.Total_Encoder - Data.Pre_Total_Encoder)/8191.0f*60.0f*1000.0f;  //rpm
    Data.Now_Omega_Radian = (float)tmp_omega * RPM_TO_RADPS;
    Data.Now_Omega_Angle = (float)tmp_omega * RPM_TO_DEG;  
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature =CAN_Manage_Object->Rx_Buffer.Data[6] ;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
    Data.Pre_Total_Encoder = Data.Total_Encoder;
    if(Start_Falg==0)  Start_Falg = 1;
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_DJI_Motor_GM6020::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DJI_Motor_GM6020::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_DJI_Motor_GM6020::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        PID_Torque.Set_Integral_Error(0.0f);
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_DJI_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Out = Target_Torque / Omega_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Torque.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Angle);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Torque.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Data.Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Angle = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Data.Now_Omega_Angle);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Torque.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Control_Method 电机控制方式, 默认角度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 */
void Class_DJI_Motor_C610::Init(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate, float __Torque_Max)
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
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DJI_Motor_C610::Data_Process()
{
    //数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque, tmp_temperature;
    Struct_DJI_Motor_CAN_Data *tmp_buffer = (Struct_DJI_Motor_CAN_Data *)CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Torque_Reverse, (void *)&tmp_torque);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Temperature, (void *)&tmp_temperature);

    //计算圈数与总编码器值
    if(Start_Falg==1)
    {
        delta_encoder = tmp_encoder - Data.Pre_Encoder;
        if (delta_encoder < -Encoder_Num_Per_Round / 2)
        {
            //正方向转过了一圈
            Data.Total_Round++;
        }
        else if (delta_encoder > Encoder_Num_Per_Round / 2)
        {
            //反方向转过了一圈
            Data.Total_Round--;
        }        
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;

    //计算电机本身信息
    Data.Now_Angle = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round *360.f / Gearbox_Rate;
    Data.Now_Radian = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI  / Gearbox_Rate;
    Data.Now_Omega_Radian = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Data.Now_Omega_Angle = (float)tmp_omega * RPM_TO_DEG / Gearbox_Rate;
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature = tmp_temperature + CELSIUS_TO_KELVIN;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
    if(Start_Falg==0)  Start_Falg = 1;
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_DJI_Motor_C610::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DJI_Motor_C610::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_DJI_Motor_C610::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_DJI_Motor_C610::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Radian);
        PID_Angle.Set_Now(Data.Now_Radian);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Radian = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Torque_Max 最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意
 */
void Class_DJI_Motor_C620::Init(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate, float __Torque_Max)
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
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    this->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DJI_Motor_C620::Data_Process()
{
    //数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_torque, tmp_temperature;
    Struct_DJI_Motor_CAN_Data *tmp_buffer = (Struct_DJI_Motor_CAN_Data *)CAN_Manage_Object->Rx_Buffer.Data;

    //处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Torque_Reverse, (void *)&tmp_torque);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Temperature, (void *)&tmp_temperature);

    //计算圈数与总编码器值
    if(Start_Falg==1)
    {
        delta_encoder = tmp_encoder - Data.Pre_Encoder;
        if (delta_encoder < -Encoder_Num_Per_Round / 2)
        {
            //正方向转过了一圈
            Data.Total_Round++;
        }
        else if (delta_encoder > Encoder_Num_Per_Round / 2)
        {
            //反方向转过了一圈
            Data.Total_Round--;
        }        
    }
    Data.Total_Encoder = Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder;

    //计算电机本身信息
    Data.Now_Radian = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Data.Now_Angle = (float)Data.Total_Encoder / (float)Encoder_Num_Per_Round * 360.f / Gearbox_Rate;
    Data.Now_Omega_Radian = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Data.Now_Omega_Angle = (float)tmp_omega * RPM_TO_DEG / Gearbox_Rate;
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature = tmp_temperature + CELSIUS_TO_KELVIN;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
    if(Start_Falg==0)  Start_Falg = 1;
}


void Class_DJI_Motor_C620::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    Set_Out(0.0f);
    Output();
	//pidout也清零
	PID_Omega.Set_Out(0);
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_DJI_Motor_C620::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DJI_Motor_C620::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_DJI_Motor_C620::TIM_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        //电机断开连接
        DJI_Motor_Status = DJI_Motor_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
    }
    else
    {
        //电机保持连接
        DJI_Motor_Status = DJI_Motor_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_DJI_Motor_C620::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Out;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Radian);
        PID_Angle.Set_Now(Data.Now_Radian);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Radian = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
