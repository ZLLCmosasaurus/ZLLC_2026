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
#include "dvc_dwt.h"
/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/************************ copyright(c) ustc-robowalker **************************/

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
  Pack_Tx_CAN_B.target_type = MiniPC_Type_Nomal;
  Pack_Tx_CAN_B.windmill_type = Windmill_Type_Small;
  Pack_Tx_CAN_B.game_stage = MiniPC_Game_Stage_NOT_STARTED;
}

/**
 * @brief 迷你主机初始化,can
 *
 */
void Class_MiniPC::Init(FDCAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == FDCAN1)
  {
    CAN_Manage_Object = &CAN1_Manage_Object;
    CAN_Tx_Data_A = CAN1_MiniPc_Tx_Data;
  }
  else if (hcan->Instance == FDCAN2)
  {
    CAN_Manage_Object = &CAN2_Manage_Object;
    CAN_Tx_Data_B = CAN2_MiniPc_Tx_Data;
  }
  else if (hcan->Instance == FDCAN3)
  {
  }
}

float camera_distance = 0.0f;
/**
 * @brief 数据处理过程
 *
 */
void Class_MiniPC::Data_Process()
{
#ifdef MINIPC_COMM_USB
  memcpy(&Pack_Rx, (Pack_rx_t *)USB_Manage_Object->Rx_Buffer, USB_Manage_Object->Rx_Buffer_Length);
  static float tmp_yaw, tmp_pitch;
  static float tmp_x, tmp_y, tmp_z;
  // if (Lidar_if_Lob == 2)
  // {
  //   // 检测包序号是否变化（新计算完成）
  //   bool is_new_packet = (Pack_Rx.lob_time != last_lob_time);
  //   if (is_new_packet)
  //   {
  //     last_lob_time = Pack_Rx.lob_time;

  //     // 只在首次进入雷达模式或用户主动重置锁定时才更新目标
  //     if (!radar_locked)
  //     {
  //       Convert_Radar_to_GunCoordinate(Pack_Rx.lidar_lob_x, Pack_Rx.lidar_lob_y, Pack_Rx.lidar_lob_z,
  //                                      &tmp_x, &tmp_y, &tmp_z);
  //       Distance = calc_distance(tmp_x, tmp_y, tmp_z);
  //       locked_yaw = calc_yaw(tmp_x, tmp_y, 0.0f);
  //       locked_pitch = Calc_Pitch_Compensated(tmp_x, tmp_y, tmp_z, tmp_x, tmp_y, tmp_z);
  //       radar_locked = true;
  //     }
  //   }
  //   // 始终使用锁定的角度
  //   Rx_Angle_Yaw = locked_yaw;
  //   Rx_Angle_Pitch = -locked_pitch;
  // }
  // else
  // {
  //   // 非雷达模式时，清除锁定，以便下次重新进入
  //   if (radar_locked)
  //   {
  //     radar_locked = false;
  //     Rx_Angle_Yaw = IMU->Get_Angle_Yaw();
  //     Rx_Angle_Pitch = -IMU->Get_Angle_Pitch();
  //   }
  // }
  if (lob_exec_enabled)
  {
    Convert_Radar_to_GunCoordinate(Pack_Rx.lidar_lob_x, Pack_Rx.lidar_lob_y, Pack_Rx.lidar_lob_z,
                                   &tmp_x, &tmp_y, &tmp_z);
    float x = tmp_x;
    float y = tmp_y;
    float z = tmp_z;
    Distance = calc_distance(x, y, z);
    float yaw_calc = calc_yaw(x, y, 0.0f);
    float pitch_calc = Calc_Pitch_Compensated(x, y, z, x, y, z);
    Rx_Angle_Yaw = yaw_calc;
    Rx_Angle_Pitch = -pitch_calc; // 符号与你的电机方向一致
    Lidar_Lob_Stability = Pack_Rx.lidar_lob_stability;
    // 如果需要区分点位的弹道参数，可以在这里使用 lob_point
    // if (lob_point == 0) { ... } else { ... }
  }
  else
  {
    Mode = Pack_Rx.mode;
    Rx_Angle_Pitch = -Pack_Rx.target_pitch;
    Rx_Gyro_Pitch = Pack_Rx.target_pitch_vel;
    Rx_Acc_Pitch = Pack_Rx.target_pitch_acc;
    Rx_Angle_Yaw = Pack_Rx.target_yaw;
    Rx_Gyro_Yaw = Pack_Rx.target_yaw_vel;
    Rx_Acc_Yaw = Pack_Rx.target_yaw_acc;
  }
  Math_Constrain(&Rx_Angle_Pitch, -15.0f, 43.0f);
  Math_Constrain(&Rx_Angle_Yaw, -180.0f, 180.0f);
  memset(USB_Manage_Object->Rx_Buffer, 0, USB_Manage_Object->Rx_Buffer_Length);
#endif

#ifdef MINIPC_COMM_CAN
  // CAN通信的数据处理
  float tmp_yaw, tmp_pitch;
  // 将CAN接收到的数据转换为实际值 (除以1000转换回浮点数)
  float target_x = Pack_Rx.target_x / 1000.0f;
  float target_y = Pack_Rx.target_y / 1000.0f;

  Fire = Pack_Rx.Fire;

  // Self_aim(target_x, target_y, target_z + camera_distance, &tmp_yaw, &tmp_pitch, &Distance);
  Rx_Angle_Pitch = target_y; // tmp_pitch;
  Rx_Angle_Yaw = target_x;   // tmp_yaw;
  alive = Pack_Rx.alive;
  Math_Constrain(&Rx_Angle_Pitch, -20.0f, 35.0f);

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
  // Pack_Tx.pitch = Tx_Angle_Pitch; // 2024.5.7 未知原因添加负号，使得下位机发送数据不满足右手螺旋定则，但是上位机意外可以跑通
  // Pack_Tx.roll = Tx_Angle_Roll;
  // Pack_Tx.yaw = Tx_Angle_Yaw;
  Pack_Tx.x = static_cast<int16_t>(Tx_Quaternion.x * 10000.0f); // 原 qy → 新 qx
  Pack_Tx.y = static_cast<int16_t>(Tx_Quaternion.y * 10000.0f); // 原 qx → 新 qy
  Pack_Tx.z = static_cast<int16_t>(Tx_Quaternion.z * 10000.0f); // 原 qw → 新 qz (注意符号)
  Pack_Tx.w = static_cast<int16_t>(Tx_Quaternion.w * 10000.0f); // 原 qz → 新 qw
  Pack_Tx.yaw_vel = Tx_Gyro_Yaw;
  Pack_Tx.pitch_vel = Tx_Gyro_Pitch;
  Pack_Tx.bullet_speed = Tx_Bullet_Speed;
  Pack_Tx.bullet_num = Tx_Bullet_Num;
  // if (Lidar_if_Lob == 2)
  // {
  //   // 第0位 = 1（吊射使能）
  //   // 第4位和第5位 = 点位（0或1）
  //   Pack_Tx.lidar_start_lob = 0x01 | ((lob_point & 0x03) << 4);
  // }
  // else
  // {
  //   // 非吊射模式时，清0
  //   Pack_Tx.lidar_start_lob = 0;
  // }
  if (lob_exec_enabled)
    // 最低位置1，bit4和bit5放点位
    Pack_Tx.lidar_start_lob = 0x01 | (lob_point  << 3);
  else
    Pack_Tx.lidar_start_lob = 0;
  Pack_Tx.game_stage = (Enum_MiniPC_Game_Stage)Referee->Get_Game_Stage();
  Pack_Tx.crc16 = 0xffff;
  memcpy(USB_Manage_Object->Tx_Buffer, &Pack_Tx, sizeof(Pack_Tx));
  Append_CRC16_Check_Sum(USB_Manage_Object->Tx_Buffer, sizeof(Pack_Tx));
  USB_Manage_Object->Tx_Buffer_Length = sizeof(Pack_Tx);
#endif

#ifdef MINIPC_COMM_CAN
  // 设置发送数据
  Pack_Tx_CAN_B.game_stage = (Enum_MiniPC_Game_Stage)Referee->Get_Game_Stage();
  Pack_Tx_CAN_B.target_type = Get_MiniPC_Type();

  Pack_Tx_CAN_A.Roll = Tx_Angle_Roll * 100.0f;
  Pack_Tx_CAN_A.Yaw = Tx_Angle_Yaw * 100.0f;
  Pack_Tx_CAN_A.Pitch = 1.0f * Tx_Angle_Pitch * 100.0f;
  // Pack_Tx_CAN_A.Gyro_Yaw = Tx_Angle_Gyro_Yaw * 100.0f;
  memcpy(CAN_Tx_Data_A, &Pack_Tx_CAN_A, sizeof(Pack_tx_can_t_A));
  memcpy(CAN_Tx_Data_B, &Pack_Tx_CAN_B, sizeof(Pack_tx_can_t_B));

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
float Class_MiniPC::calc_pitch(float x, float y, float z, uint8_t mode)
{
#ifdef OLD
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
  dist = sqrtf(x * x + y * y);

  // 将弧度制的俯仰角转换为角度制
  pitch = (pitch * 180 / PI); // 向上为负，向下为正

  return pitch;
#endif
  float tmp_g = 0.0f;
  switch (mode)
  {
  case 0:
  {
    tmp_g = g;
  }
  break;
  case 1:
  {
    tmp_g = g * cos(Tx_Angle_Roll);
  }
  break;
  }
  const float a_d = 0.0595f; // 改为pitch旋转中心到摩擦轮的距离
  float d = calc_distance(x, y, z);
  if (d < a_d)
  {
    // return 当前pitch，因为无解1
    // return dmIMU->Get_DMIMU_Pitch();
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
  *pitch = calc_pitch(x, y, z, 0);
  *distance = calc_distance(x, y, z);
}

void Class_MiniPC::Auto_aim_Add_Roll(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
  float Now_Angle_Roll = Tx_Angle_Roll;
  if (fabs(Now_Angle_Roll) > 0.001f)
  {
    // 将点从视觉坐标系转换到云台坐标系
    float x_rotated = x;
    float y_rotated = y * cosf(Now_Angle_Roll) + z * sinf(Now_Angle_Roll); // 符合右手定则 x轴指向
    float z_rotated = -y * sinf(Now_Angle_Roll) + z * cosf(Now_Angle_Roll);
    *yaw = calc_yaw(x_rotated, y_rotated, z_rotated);
    *pitch = calc_pitch(x_rotated, y_rotated, z_rotated, 1);
    *distance = calc_distance(x_rotated, y_rotated, z_rotated);
  }
  else
  {
    // 没有roll角度，使用原始坐标
    *yaw = calc_yaw(x, y, z);
    *pitch = calc_pitch(x, y, z, 0);
    *distance = calc_distance(x, y, z);
  }
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

/**
 * @brief 将雷达坐标系转换为枪口坐标系
 *
 * @param radar_x
 * @param radar_y
 * @param radar_z
 * @param gun_x
 * @param gun_y
 * @param gun_z
 */
void Class_MiniPC::Convert_Radar_to_GunCoordinate(float radar_x, float radar_y, float radar_z,
                                                  float *gun_x, float *gun_y, float *gun_z)
{

  const float offset = 0.131f;
  // 当前云台 pitch（角度制 → 弧度）
  float pitch_rad = -Tx_Angle_Pitch * PI / 180.0f; // Tx_Angle_Pitch 来自 IMU

  // 绕 Y 轴旋转（角度 = -pitch_rad）
  float c = cosf(pitch_rad);
  float s = sinf(pitch_rad);

  *gun_x = radar_x - cosf(pitch_rad) * offset;
  *gun_y = radar_y;
  *gun_z = radar_z - sinf(pitch_rad) * offset;
}

/**
 * @brief 计算球体弹丸的空气阻力
 * @param velocity 球体相对于空气的速度（单位：m/s）
 * @param diameter 球体直径（单位：m）
 * @param air_density 空气密度（单位：kg/m³，默认值：1.225）
 * @param drag_coefficient 阻力系数（默认值：0.47，对应光滑球体亚音速）
 * @return 空气阻力（单位：N）
 */
float calculate_sphere_drag_force(float velocity, float diameter)
{
  const float air_density = 1.225f;
  const float drag_coefficient = 0.47f;
  // 输入参数校验
  if (diameter <= 0 || velocity < 0)
    return 0.0f;

  // 计算横截面积 A = π*(d/2)^2
  const float radius = diameter / 2.0f;
  const float area = PI * radius * radius;

  // 应用公式 Fd = 0.5 * Cd * ρ * v² * A
  const float force = 0.5f * drag_coefficient * air_density * velocity * velocity * area;
  return force;
}

/**
 * @brief 迭代重力-空气阻力补偿计算俯仰角
 *
 * @param x
 * @param y
 * @param z
 * @param init_x
 * @param init_y
 * @param init_z
 * @return float
 */
float Class_MiniPC::Calc_Pitch_Compensated(float x, float y, float z, float init_x, float init_y, float init_z)
{
  const float diameter = 0.042f;
  float k = calculate_sphere_drag_force(bullet_v, diameter); // 空气阻力系数
  float epsilon = 0.01f;                                     // 收敛阈值

  // 初始俯仰角计算（基于几何投影）
  float pitch = atan2f(z, sqrtf(x * x + y * y));

  // 迭代补偿重力和空气阻力影响
  for (int i = 0; i < 20; ++i)
  {
    float target_rho = sqrtf(x * x + y * y); // 水平投影距离
    float cos_pitch = cosf(pitch);

    // 处理无效的cos值
    if (fabsf(cos_pitch) < 1e-6f)
      break;

    // 计算弹丸飞行时间（考虑水平方向阻力）
    float t_numerator = target_rho * k;
    float t_denominator = bullet_v * cos_pitch;
    if (t_numerator >= t_denominator)
      break; // 弹道不可达

    float fly_time = (-logf(1 - t_numerator / t_denominator)) / k;

    // 计算z轴实际位移（含阻力模型）
    float sin_pitch = sinf(pitch);
    float exp_term = expf(-k * fly_time);
    float real_z =
        (bullet_v * sin_pitch + g / k) * (1 - exp_term) / k -
        (g * fly_time) / k;

    // 计算高度误差并调整俯仰角
    float dz = z - real_z;
    if (fabsf(dz) < epsilon)
      break;

    // 安全计算角度修正量（限制在asin有效范围）
    float distance = sqrtf(x * x + y * y + z * z);
    float clamped_dz = fmaxf(fminf(dz, distance), -distance);
    float delta_pitch = asinf(clamped_dz / distance);

    pitch += delta_pitch;
  }

  // 转换为角度制并符合坐标系定义
  return -(pitch * 180.0f / PI);
}

/************************ copyright(c) ustc-robowalker **************************/

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
float Dt3;
uint32_t last_cnt3 = 0;
void Class_MiniPC::CAN_RxCpltCallback(uint8_t *rx_data)
{
#ifdef MINIPC_COMM_CAN
  // 滑动窗口, 判断迷你主机是否在线
  Flag += 1;

  // 直接将接收到的数据复制到结构体
  memcpy(&Pack_Rx, rx_data, sizeof(Pack_Rx));

  Dt3 = DWT_GetDeltaT(&last_cnt3);
  // 处理数据
  Data_Process();
#endif
}
