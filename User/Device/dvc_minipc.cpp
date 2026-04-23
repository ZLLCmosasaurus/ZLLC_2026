/**
 * @file dvc_minipc.cpp
 * @author lez by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"


/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化,usb
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object *__USB_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
  USB_Manage_Object = __USB_Manage_Object;
  Frame_Header = __frame_header;
  Frame_Rear = __frame_rear;
  Pack_Tx.target_type = MiniPC_Type_Nomal;
  Pack_Tx.windmill_type = Windmill_Type_Small;
  Pack_Tx.game_stage = MiniPC_Game_Stage_NOT_STARTED;
}

/**
 * @brief 迷你主机初始化,can
 *
 */
void Class_MiniPC::Init(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_Manage_Object = &CAN1_Manage_Object;
    CAN_Tx_Data = CAN1_MiniPc_Tx_Data;
  }
  else if (hcan->Instance == CAN2)
  {
    CAN_Manage_Object = &CAN2_Manage_Object;
    CAN_Tx_Data = CAN2_MiniPc_Tx_Data;
  }
}

float camera_distance_z = -0.036;
float camera_distance_y = -0.036;
float camera_distance_x =0;
/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
#ifdef MINIPC_COMM_USB
  memcpy(&Pack_Rx, (Pack_rx_t *)USB_Manage_Object->Rx_Buffer, USB_Manage_Object->Rx_Buffer_Length);
  // 解析数据包
  // uint8_t sum_data[16];
  // for (auto i = 0; i < 16; i++)
  // {
  //   sum_data[i] = USB_Manage_Object->Rx_Buffer[i];
  // }
  
  if(Verify_CRC16_Check_Sum(USB_Manage_Object->Rx_Buffer,16) == true)
  {
    Rx_Target_X = Pack_Rx.target_x;
    Rx_Target_Y = Pack_Rx.target_y;
    Rx_Target_Z = Pack_Rx.target_z;
    Math_Constrain(&Rx_Target_X,-640.0f,640.0f);
    Math_Constrain(&Rx_Target_Y,-480.0f,480.0f);
  }
  memset(USB_Manage_Object->Rx_Buffer, 0, USB_Manage_Object->Rx_Buffer_Length);
#endif

#ifdef MINIPC_COMM_CAN
  // CAN通信的数据处理
  float tmp_yaw, tmp_pitch;
  // 将CAN接收到的数据转换为实际值 (除以1000转换回浮点数)
  float target_x = Pack_Rx.target_x / 1000.0f;
  float target_y = Pack_Rx.target_y / 1000.0f;
  float target_z = Pack_Rx.target_z / 1000.0f;
  
  Self_aim(target_x, target_y, target_z, &tmp_yaw, &tmp_pitch, &Distance);
  Rx_Angle_Pitch = tmp_pitch;
  Rx_Angle_Yaw = tmp_yaw;
  Math_Constrain(&Rx_Angle_Pitch, -19.5f, 38.5f);
#endif
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
/**
 * @brief 迷你主机发送数据输出
 *
 */
void Class_MiniPC::Output()
{
#ifdef MINIPC_COMM_USB
  Pack_Tx.header = Frame_Header;

  // 根据referee判断红蓝方
  if (Referee->Get_ID() >= 101)
    Pack_Tx.detect_color = 101;
  else
    Pack_Tx.detect_color = 0;

  Pack_Tx.target_id = 0x08;
  Pack_Tx.roll = Tx_Angle_Roll;
  Pack_Tx.pitch = Tx_Angle_Pitch; // 2024.5.7 未知原因添加负号，使得下位机发送数据不满足右手螺旋定则，但是上位机意外可以跑通
  Pack_Tx.yaw = Tx_Angle_Yaw;
  Pack_Tx.crc16 = 0xffff;
  Pack_Tx.game_stage = (Enum_MiniPC_Game_Stage)Referee->Get_Game_Stage();
  memcpy(USB_Manage_Object->Tx_Buffer, &Pack_Tx, sizeof(Pack_Tx));
  Append_CRC16_Check_Sum(USB_Manage_Object->Tx_Buffer, sizeof(Pack_Tx));
  USB_Manage_Object->Tx_Buffer_Length = sizeof(Pack_Tx);
#endif

#ifdef MINIPC_COMM_CAN
  // 设置发送数据
  Pack_Tx.game_stage = (Enum_MiniPC_Game_Stage)Referee->Get_Game_Stage();
  Pack_Tx.roll = (int16_t)(Tx_Angle_Roll * 100.0f);
  Pack_Tx.pitch = (int16_t)(Tx_Angle_Pitch * 100.0f);
  Pack_Tx.yaw = (int16_t)(Tx_Angle_Yaw * 100.0f);
  
  // 直接将整个结构体复制到CAN发送缓冲区
  memcpy(CAN_Tx_Data, &Pack_Tx, sizeof(Pack_Tx));
  

#endif
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
  Transform_Angle_Tx();
  Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
  // 滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process();
}

/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC																																																																																																																												::TIM1msMod50_Alive_PeriodElapsedCallback()
{
  // 判断该时间段内是否接收过迷你主机数据
  if (Flag == Pre_Flag)
  {
    // 迷你主机断开连接
    MiniPC_Status = MiniPC_Status_DISABLE;
    // Buzzer.Set_NowTask(BUZZER_DEVICE_OFFLINE_PRIORITY);
  }
  else
  {
    // 迷你主机保持连接
    MiniPC_Status = MiniPC_Status_ENABLE;
  }

  Pre_Flag = Flag;
}

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL)
    return 0xFFFF;
  while (dwLength--)
  {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
      (w_expected & 0xff) == pchMessage[dwLength - 2] &&
      ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2))
    return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z)
{
  // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
  float yaw = atan2f(y, x);

  // 将弧度制的偏航角转换为角度制
  yaw = (yaw * 180 / PI); // 向左为正，向右为负

  return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z)
{
  // 计算各分量的平方和，并取其平方根得到欧几里德距离
  float distance = sqrtf(x * x + y * y + z * z);

  return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */

float Class_MiniPC::calc_pitch(float x, float y, float z)
{
  // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
  float pitch = atan2f(z, sqrtf(x * x + y * y));
  // 使用重力加速度模型迭代更新俯仰角
  for (size_t i = 0; i < 20; i++)
  {
    float v_x;
    float v_y;
    if (Referee->Get_Referee_Status() == Referee_Status_ENABLE && Referee->Get_Shoot_Speed() > 15)
    {
      v_x = Referee->Get_Shoot_Speed() * cosf(pitch);
      v_y = Referee->Get_Shoot_Speed() * sinf(pitch);
    }
    else
    {
      v_x = bullet_v * cosf(pitch);
      v_y = bullet_v * sinf(pitch);
    } // 计算子弹飞行时间
    float t = sqrtf(x * x + y * y) / v_x;
    float h = v_y * t - 0.5 * g * t * t;
    float dz = z - h;

    if (abs(dz) < 0.01)
    {
      break;
    }
    // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
    pitch += asinf(dz / calc_distance(x, y, z));
  }

  // 将弧度制的俯仰角转换为角度制
  pitch = (pitch * 180 / PI); // 向上为负，向下为正

  return pitch;
}
/**
 * 计算计算yaw，pitch
 *
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */

void Class_MiniPC::Self_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{

  *yaw = calc_yaw(x, y, z);
  *pitch = calc_pitch(x, y, z);
  *distance = calc_distance(x, y, z);
}

float Class_MiniPC::meanFilter(float input)
{
  static float buffer[5] = {0};
  static uint64_t index = 0;
  float sum = 0;

  // Replace the oldest value with the new input value
  buffer[index] = input;

  // Increment the index, wrapping around to the start of the array if necessary
  index = (index + 1) % 5;

  // Calculate the sum of the buffer's values
  for (int i = 0; i < 5; i++)
  {
    sum += buffer[i];
  }

  // Return the mean of the buffer's values
  return sum / 5.0;
}

/************************ copyright(c) ustc-robowalker **************************/

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::CAN_RxCpltCallback(uint8_t *rx_data)
{
#ifdef MINIPC_COMM_CAN
    // 滑动窗口, 判断迷你主机是否在线
    Flag += 1;
    
    // 直接将接收到的数据复制到结构体
    memcpy(&Pack_Rx, rx_data, sizeof(Pack_Rx));
    
    // 处理数据
    Data_Process();
  
#endif
}
