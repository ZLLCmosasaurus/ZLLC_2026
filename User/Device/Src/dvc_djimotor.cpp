/**
 * @file dvc_motor.cpp
 * @author cjw by yssickjgd
 * @brief 大疆电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
extern uint8_t CAN2_0x1fe_Tx_Data[8];
/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hfdcan1)
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
            #ifdef GIMBAL
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
            #else
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            #endif
            //tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x206):
        {
            #ifdef GIMBAL
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
            #else
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            #endif
            //tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
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
    else if (hcan == &hfdcan2)
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
    else if( hcan == &hfdcan3)
    {
        switch (__CAN_ID)
        {
        case (DJI_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x205):
        {
            tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x206):
        {
            tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x207):
        {
            tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x208):
        {
            tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[6]);
        }
        break;
        case (DJI_Motor_ID_0x209):
        {
            tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x20A):
        {
            tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x20B):
        {
            tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[4]);
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
void Class_DJI_Motor_GM6020::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, int32_t __Encoder_Offset, float __Omega_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
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
    Math_Endian_Reverse_16((void *)&tmp_buffer->Temperature, (void *)&tmp_temperature);

    tmp_encoder += Encoder_Offset;                          //加上预设的偏移量

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
    Data.Now_Angle = (float)tmp_encoder / (float)Encoder_Num_Per_Round * 360.0f;

    //因为加上了Encoder_Offset，角度可能大于360
    if(Data.Now_Angle > 360.0f){
        Data.Now_Angle -= 360.0f;
    }
    else if(Data.Now_Angle < 0.0f){
        Data.Now_Angle += 360.0f;
    }

    Data.Now_Radian = Data.Now_Angle * DEG_TO_RAD;
    // Data.Now_Omega_Angle = (float)(Data.Total_Encoder - Data.Pre_Total_Encoder)/8191.0f*60.0f*1000.0f;  //rpm
    Data.Now_Omega_Radian = (float)tmp_omega * RPM_TO_RADPS;
    Data.Now_Omega_Angle = (float)tmp_omega * RPM_TO_DEG;  
    Data.Now_Torque = tmp_torque;
    Data.Now_Temperature = tmp_temperature + CELSIUS_TO_KELVIN;			

    Zero_Offset_Radian = Normalize_Angle_Radian_PI_to_PI(Data.Now_Radian - Zero_Position);
    Zero_Offset_Angle  = Zero_Offset_Radian * 180.0f / 3.14159f;

    //存储预备信息
    Data.Pre_Encoder = tmp_encoder;
    Data.Pre_Total_Encoder = Data.Total_Encoder;
    Data.Pre_Angle = Data.Now_Angle;
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
        PID_Omega.Set_Now(Transform_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Transform_Angle);                 //转换后的角度，右手螺旋定律，标准坐标系
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Angle = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Angle);
        PID_Omega.Set_Now(Transform_Omega);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        float tmp_Torque = J * Transform_Target_Acc + B * Transform_Target_Vel + Mgl * arm_cos_f32(Transform_Angle/57.3f) + C;  //简单的动力学补偿，参数需要根据实际负载测量后赋值
        Out = PID_Omega.Get_Out() + tmp_Torque * 16384.0f / (3.0f * 0.741f);
        Math_Constrain(&Out, -(float)Output_Max, (float)Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_AGV_MODE):
    {       
        
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

void Class_DJI_Motor_GM6020::TIM_SMC_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
        case DJI_Motor_Control_Method_OPENLOOP:
        {
            Out = 0.0f;
            Output();
            break;
        }
       
        default:
        {
            SMC_Control.Set_Target(Target_Angle);
            SMC_Control.Set_Now(Transform_Angle, Transform_Omega * 57.3f);                   

            SMC_Control.TIM_Adjust_PeriodElapsedCallback();
            Out = SMC_Control.Get_Out();
            //Out = Test_Out;
            Output();
            break;
        }
    }
}

/**
 * @brief 额外的力矩补偿
 * @param Compensite_Value 
 */
void Class_DJI_Motor_GM6020::Compensite_Output(float Compensite_Value)
{
    Out += Compensite_Value;

    if(Out > Output_Max){
        Out = Output_Max;
    }
    else if(Out < -Output_Max){
        Out = -Output_Max;
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
void Class_DJI_Motor_C610::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate, float __Torque_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
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
        Out = Target_Torque / Torque_Max * Output_Max;
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
void Class_DJI_Motor_C620::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
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

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_DJI_Motor_C620::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}

void Class_DJI_Motor_C620::Disable()
{
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);

    PID_Omega.Set_Integral_Error(0.0f);
    PID_Angle.Set_Integral_Error(0.0f);

    Set_Out(0.0f);
    Output();
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
        Out = 0.0f;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Out = Target_Torque * Output_Max / ((Gearbox_Rate * 0.3f * 20.0f)/ 19.2032f);
        Math_Constrain(&Out, -16384.0f, 16384.0f);
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
    //Out = 0.0f;//test
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
void Class_DJI_Motor_C620_Steer::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    this->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_DJI_Motor_C620_Steer::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
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
    case (DJI_Motor_Control_Method_AGV_MODE):
    {               //注意，直接用大疆电机的数据和用磁编的数据角度范围什么的是不一样的
        PID_Angle.Set_Target(Target_Radian);
        PID_Angle.Set_Now(Transform_Radian);
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
    //Out = 0.0f;//test
    Output();
}


/**
 * @brief 磁编存活判断函数
 */
void Class_DJI_Motor_C620_Steer::MA600_TIM_Alive_PeriodElapsedCallback()
{
    if (MA600_Flag == MA600_Pre_Flag)
    {
        MA600_Status = MA600_Status_DISABLE;
    }
    else
    {
        MA600_Status = MA600_Status_ENABLE;
    }
    MA600_Pre_Flag = MA600_Flag;
}

void Class_DJI_Motor_C620_Steer::MA600_Omega_Updata()
{
    delta_angle = (Zero_Offset_Radian - Pre_Zero_Offset_Radian);
    delta_angle = Normalize_Angle_Radian_PI_to_PI(delta_angle) * 150.0f;
    MA600_KF.MeasuredVector[0] =  delta_angle;
    Kalman_Filter_Update(&MA600_KF, NULL);
    MA600_Omega = MA600_KF.FilteredValue[0];
}

void Class_DJI_Motor_C620_Steer::MA600_Data_Process(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    if(CAN_RxMessage->Data[0] != 0xA5 || CAN_RxMessage->Data[7] != 0xB5){
        return;
    }
    
    MA600_Flag ++;

    int16_t temp_Single_Radian = CAN_RxMessage->Data[2] << 8 | CAN_RxMessage->Data[1];
    int16_t temp_Multi_Radian  = CAN_RxMessage->Data[4] << 8 | CAN_RxMessage->Data[3];
    int16_t temp_Omega         = CAN_RxMessage->Data[6] << 8 | CAN_RxMessage->Data[5];

    MA600_Data.Single_Radian = -temp_Single_Radian / 100.0f;                //注意磁编数据和3508数据正负应该一致
    MA600_Data.Multi_Radian  = -temp_Multi_Radian  / 100.0f;
    MA600_Data.Omega         = -temp_Omega         / 100.0f;

    Pre_Zero_Offset_Radian = Zero_Offset_Radian;

    float delta_rad = MA600_Data.Single_Radian - Zero_Position;
    Zero_Offset_Radian = Normalize_Angle_Radian_PI_to_PI(delta_rad); 

    // if(MA600_Flag == 1){
    //     Kalman_Filter_Init(&MA600_KF, 1, 0, 1);
    //     float F = 1.0f;
    //     float H = 1.0f;
    //     float P = 1.0f;
    //     float Q = 0.15f;             //测量噪声
    //     float R = 10.0f;

    //     memcpy(MA600_KF.F_data, &F, 4);
    //     memcpy(MA600_KF.H_data, &H, 4);
    //     memcpy(MA600_KF.P_data, &P, 4);
    //     memcpy(MA600_KF.Q_data, &Q, 4);
    //     memcpy(MA600_KF.R_data, &R, 4);

    //     Pre_Zero_Offset_Radian = Zero_Offset_Radian;
    // }

    // MA600_Omega_Updata();
}


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
