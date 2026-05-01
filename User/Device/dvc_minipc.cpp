/**
 * @file dvc_minipc.cpp
 * @author cjw by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
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

float camera_distance = 0.036;
/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
  // CAN通信的数据处理
  float tmp_yaw, tmp_pitch;
  // // 将CAN接收到的数据转换为实际值 (除以1000转换回浮点数)
  // float target_x = Pack_Rx.target_x / 1000.0f;
  // float target_y = Pack_Rx.target_y / 1000.0f;
  // float target_z = Pack_Rx.target_z / 1000.0f;
  tmp_yaw = Pack_Rx.yaw / 10000.0f;
  tmp_pitch = Pack_Rx.pitch / 10000.0f;
  Fire = Pack_Rx.Fire;
  alive = Pack_Rx.alive;

  // Self_aim(target_x, target_y, target_z + camera_distance, &tmp_yaw, &tmp_pitch, &Distance);
  // Self_aim(target_x, target_y, target_z, &tmp_yaw, &tmp_pitch, &Distance);
  Rx_Angle_Pitch = -tmp_pitch * 180 / PI;
  Rx_Angle_Yaw = tmp_yaw * 180 / PI;
  Math_Constrain(&Rx_Angle_Pitch, -45.0f, 10.0f);
}

/**
 * @brief 迷你主机发送数据输出
 *
 */
void Class_MiniPC::Output()
{
  // 设置发送数据

  float Yaw_rad = Tx_Angle_Yaw * PI / 180.0f;
  float Pitch_rad = Tx_Angle_Pitch * PI / 180.0f;
  float Roll_rad = Tx_Angle_Roll * PI / 180.0f;

  Pack_Tx_CAN.q[3] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[0] = (int16_t)((arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) - arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[1] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f)) * 10000.f);
  Pack_Tx_CAN.q[2] = (int16_t)((arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f) - arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f)) * 10000.f);

  memcpy(CAN_Tx_Data, &Pack_Tx_CAN, sizeof(Pack_tx_t));
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
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
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
float dist;
float Class_MiniPC::calc_pitch(float x, float y, float z)
{
  float d = calc_distance(x, y, z);
  if (d < a_d)
  {
    // return 当前pitch，因为无解1
    return IMU->Get_Angle_Pitch();
  }

  float v0 =
      Referee->Get_Referee_Status() == Referee_Status_ENABLE &&
              Referee->Get_Shoot_Speed()
          ? Referee->Get_Shoot_Speed()
          : bullet_v;

  // 初始估值一定偏小一点点
  float t = (d - a_d) / v0;
  const float t1 = 2.0f * a_d * v0, t2 = v0 * v0 - z * g;

  // 牛顿迭代法，可省略，最好两次
  t -= (d * d - a_d * a_d + t * (-t1 + t * (-t2 + 0.25f * g * g * t * t))) / (-t1 + t * (-2.0f * t2 + g * g * t * t));
  t -= (d * d - a_d * a_d + t * (-t1 + t * (-t2 + 0.25f * g * g * t * t))) / (-t1 + t * (-2.0f * t2 + g * g * t * t));

  // pitch向下为正，加负号
  return 180.0f * atanf((z + 0.5 * g * t * t) / sqrtf(x * x + y * y)) / PI;
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
  // 滑动窗口, 判断迷你主机是否在线
  Flag += 1;

  // 直接将接收到的数据复制到结构体
  memcpy(&Pack_Rx, rx_data, 6);
  // memcpy(&Pack_Rx_test, rx_data, sizeof(Pack_Rx_test));
  //  处理数据
  Data_Process();
}