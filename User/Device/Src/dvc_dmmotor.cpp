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
        case (DM_Motor_ID_0xA1):
        {
            tmp_tx_data_ptr = CAN1_0xxf1_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA2):
        {
            tmp_tx_data_ptr = CAN1_0xxf2_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA3):
        {
            tmp_tx_data_ptr = CAN1_0xxf3_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA4):
        {
            tmp_tx_data_ptr = CAN1_0xxf4_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA5):
        {
            tmp_tx_data_ptr = CAN1_0xxf5_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA6):
        {
            tmp_tx_data_ptr = CAN1_0xxf6_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA7):
        {
            tmp_tx_data_ptr = CAN1_0xxf7_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA8):
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
        case (DM_Motor_ID_0xA1):
        {
            tmp_tx_data_ptr = CAN2_0xxf1_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA2):
        {
            tmp_tx_data_ptr = CAN2_0xxf2_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA3):
        {
            tmp_tx_data_ptr = CAN2_0xxf3_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA4):
        {
            tmp_tx_data_ptr = CAN2_0xxf4_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA5):
        {
            tmp_tx_data_ptr = CAN2_0xxf5_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA6):
        {
            tmp_tx_data_ptr = CAN2_0xxf6_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA7):
        {
            tmp_tx_data_ptr = CAN2_0xxf7_Tx_Data;
        }
        break;
        case (DM_Motor_ID_0xA8):
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
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DM_Motor_J4310::Data_Process(uint8_t* Rx_Data)
{

    int16_t delta_position = 0;
    int16_t tmp_omega, tmp_torque;
    int16_t tmp_position;
    uint8_t DM_Rx_Data[8];
    memcpy(DM_Rx_Data,Rx_Data, 8);
				
    tmp_position=(DM_Rx_Data[1] << 8) | (DM_Rx_Data[2]);
    tmp_omega = (DM_Rx_Data[3] << 4) | (DM_Rx_Data[4] >> 4);
    tmp_torque = ((DM_Rx_Data[4] & 0x0f) << 8) | (DM_Rx_Data[5]);

    Data.CAN_ID = (Enum_DM_Motor_ID)(DM_Rx_Data[0] & 0x0f);
    Data.ErrorCode = (Enum_DM_Motor_ErrorCode)(DM_Rx_Data[0] >> 4);

    //计算圈数与总角度值
    delta_position = (float)tmp_position / 65536.0f * 360.0f + Data.Total_Round * 360.0f + Position_Offset - Data.Pre_Position;
    if (delta_position < -180.0f)
    {
        //正方向转过了一圈
        Data.Total_Round++;
    }
    else if (delta_position > 180.0f)
    {
       //反方向转过了一圈
        Data.Total_Round--;
    }

    //计算电机本身信息
    Data.Now_Angle = (float)tmp_position / 65536.0f * PI + 1.57f;//因为用了位置速度改的
    Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -30.0f, 30.0f);
    Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -10.0, 10.0);
    Data.Now_MOS_Temperature = DM_Rx_Data[6];
    Data.Now_Rotor_Temperature = DM_Rx_Data[7];

    //存储预备信息
    Data.Pre_Position = Data.Now_Angle;    
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DM_Motor_J4310::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    this->Flag += 1;
		
    Data_Process(Rx_Data);
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
    }
    else
    {
        //电机保持连接
        DM_Motor_Status = DM_Motor_Status_ENABLE;
    }
   //控制电机使能或失能
   switch (DM_Motor_Status)
   {
   case (DM_Motor_Status_DISABLE):
   {
       switch (DM_Motor_Control_Method)
       {
       case (DM_Motor_Control_Method_MIT_POSITION):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Exit, 8);
       }
       break;
       case (DM_Motor_Control_Method_MIT_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Exit, 8);
       }
       break;
       case (DM_Motor_Control_Method_MIT_ANGLE):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Exit, 8);
       }
       break;
       case (DM_Motor_Control_Method_POSITION_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x1f0, DM_Motor_CAN_Message_Exit, 8);
       }
       break;
       case (DM_Motor_Control_Method_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x2f0, DM_Motor_CAN_Message_Exit, 8);
       }
       break;
       }
   }
   break;
   case (DM_Motor_Status_ENABLE):
   {
       switch (DM_Motor_Control_Method)
       {
       case (DM_Motor_Control_Method_MIT_POSITION):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Enter, 8);
       }
       break;
       case (DM_Motor_Control_Method_MIT_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Enter, 8);
       }
       break;
       case (DM_Motor_Control_Method_MIT_ANGLE):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Enter, 8);
       }
       break;
       case (DM_Motor_Control_Method_POSITION_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x1f0, DM_Motor_CAN_Message_Enter, 8);
       }
       break;
       case (DM_Motor_Control_Method_OMEGA):
       {
           CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x2f0, DM_Motor_CAN_Message_Enter, 8);
       }
       break;
       }
   }
   break;
   }
    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, DM_Motor_CAN_Message_Enter, 8);
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
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(MIT_K_P, 0.0f, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

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

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_OMEGA):
    {
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

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

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_MIT_ANGLE):
    {
        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -PI, PI, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = 0;
        uint16_t tmp_k_d = 0;
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

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

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0xf0, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_POSITION_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Target_Angle, sizeof(float));

        memcpy(&CAN_Tx_Data[4], &Target_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x1f0, CAN_Tx_Data, 8);
    }
    break;
    case (DM_Motor_Control_Method_OMEGA):
    {
        memcpy(&CAN_Tx_Data[0], &Target_Omega, sizeof(float));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, static_cast<Enum_DM_Motor_ID>(CAN_ID) + 0x2f0, CAN_Tx_Data, 4);
    }
    break;
    default:
    {
    }
    break;
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_DM_Motor_J4310::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_Control_Method)
    {
    case (DM_Motor_Control_Method_MIT_POSITION):
    {
        // Set_Target_Angle();
        
    }
    break;
    case (DM_Motor_Control_Method_MIT_OMEGA):
    {
        //速度环
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Data.Now_Omega);

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
    }
    break;
    case (DM_Motor_Control_Method_MIT_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        //角度环
        PID_Angle.Set_Now(Data.Now_Angle);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        //速度环
        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Data.Now_Omega);

        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
    }
    break;
    case (DM_Motor_Control_Method_POSITION_OMEGA):
    {
       

    }
    break;
    case (DM_Motor_Control_Method_OMEGA):
    {
       

    }
    break;
    default:
    {
        Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
    break;
    }
    TIM_Process_PeriodElapsedCallback();
}

void Class_DM_Motor_J4310::Output()
{
    switch (DM_Motor_Control_Method)
    {
        case(DM_Motor_Control_Method_MIT_IMU_Angle):
        {
            Limit_Out();

            uint16_t tmp_out=Math_Float_To_Int(Out,-4095,4095,0,(1<<12)-1);

            uint8_t tmp_torque_6=(tmp_out>>8)&0x0F;
            memcpy(&CAN_Tx_Data[6],&tmp_torque_6,sizeof(uint8_t));

            uint8_t tmp_torque_7=tmp_out & 0xFF;
            memcpy(&CAN_Tx_Data[7],&tmp_torque_7,sizeof(uint8_t));

            uint8_t tmp_torque[6]={0x7F,0xFF,0x7F,0xF0,0x00,0x00};
            for (uint8_t i = 0; i < 6; i++)
            {
                memcpy(&CAN_Tx_Data[i],&tmp_torque[i],sizeof(uint8_t));
            }    
        }
        break;

        case(DM_Motor_Control_Method_MIT_OPENLOOP):
        {
            Limit_Out();

            uint16_t tmp_out=Math_Float_To_Int(Out,-4095,4095,0,(1<<12)-1);

            uint8_t tmp_torque_6=(tmp_out>>8)&0x0F;
            memcpy(&CAN_Tx_Data[6],&tmp_torque_6,sizeof(uint8_t));

            uint8_t tmp_torque_7=tmp_out & 0xFF;
            memcpy(&CAN_Tx_Data[7],&tmp_torque_7,sizeof(uint8_t));

            uint8_t tmp_torque[6]={0x7F,0xFF,0x7F,0xF0,0x00,0x00};
            for (uint8_t i = 0; i < 6; i++)
            {
                memcpy(&CAN_Tx_Data[i],&tmp_torque[i],sizeof(uint8_t));
            }            
        }
    }
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
