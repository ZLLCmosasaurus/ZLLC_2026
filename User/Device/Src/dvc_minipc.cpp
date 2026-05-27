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
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object* __USB_Manage_Object, Struct_UART_Manage_Object* __UART_Manage_Object, Struct_CAN_Manage_Object* __CAN_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
	  USB_Manage_Object = __USB_Manage_Object;
    UART_Manage_Object = __UART_Manage_Object;
    CAN_Manage_Object = __CAN_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}

/**
 * @brief 返回己方颜色
 * @return 
 */
Enum_MiniPC_Self_Color Class_MiniPC::Get_Self_Color()
{
    Enum_Referee_Data_Robots_ID id = Referee->Get_ID();
    
    // 红方 ID 范围：1 ~ 11
    if (id >= Referee_Data_Robots_ID_RED_HERO_1 && id <= Referee_Data_Robots_ID_RED_OUTPOST_11)
    {
        return MiniPC_Self_Color_RED;
    }
    // 蓝方 ID 范围：101 ~ 111
    else if (id >= Referee_Data_Robots_ID_BLUE_HERO_1 && id <= Referee_Data_Robots_ID_BLUE_OUTPOST_11)
    {
        return MiniPC_Self_Color_BLUE;
    }
    // 其他情况（如 ID 为 0 或未定义值）视为中立方
    else
    {
        return MiniPC_Self_Color_NEUTRAL;
    }
}

/**
 * @brief 数据处理过程
 *
 */
int head = 0;
uint8_t temp_data[128];
int PACKET_LEN  = sizeof(Struct_MiniPC_Rx_Data);
void Class_MiniPC::Data_Process(Enum_MiniPC_Data_Source Data_Source)
{
  if (Data_Source == USB)
  {
    if(!Verify_CRC16_Check_Sum(USB_Manage_Object->Rx_Buffer,USB_Manage_Object->Rx_Buffer_Length)) return;
    memcpy(&Data_NUC_To_MCU, USB_Manage_Object->Rx_Buffer, PACKET_LEN);
   
    Rx_Chassis_Target_Omega               = Data_NUC_To_MCU.Chassis_Angular_Velocity_Yaw / 100.0f;
    Rx_Chassis_Target_Velocity_X          = Data_NUC_To_MCU.Move_Linear_Velocity_X / 100.0f;
    Rx_Chassis_Target_Velocity_Y          = Data_NUC_To_MCU.Move_Linear_Velocity_Y / 100.0f;
    Rx_Gimbal_Angular_Velocity_Yaw_Main   = Data_NUC_To_MCU.Gimbal_Angular_Velocity_Yaw_Main / 100.0f;
    mode = Data_NUC_To_MCU.mode;
    Rx_Angle_Yaw = Data_NUC_To_MCU.yaw;
    Rx_Angle_Pitch = Data_NUC_To_MCU.pitch;
    Math_Constrain(&Rx_Angle_Pitch, -25.0f, 22.0f);

    if(mode == 2){
      MiniPC_Fire_Updata_Flag = 1;
    }

    //要发给裁判系统的数据
    //Referee->Set_Sentry_Cmd(0x200000); // 进攻姿态
    //Referee->Set_Sentry_Cmd(0x400000); // 防御姿态
    //Referee->Set_Sentry_Cmd(0x600000); // 移动姿态 
    Referee->Set_Sentry_Cmd(Data_NUC_To_MCU.Sentry_Cmd);
  }
}

/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
extern Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
extern Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
extern Referee_Rx_D_t CAN3_Chassis_Rx_Data_D;
extern Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
extern Referee_Rx_F_t CAN3_Chassis_Rx_Data_F;
extern Referee_Rx_G_t CAN3_Chassis_Rx_Data_G;
volatile int index = 0;
void Class_MiniPC::Output()
{
 // static uint8_t mod2 = 0;
  static uint8_t  Color = 0, Invincible_Flag[6] = {0,0,0,0,0,0};
  uint16_t Self_Outpost_HP = 0, Oppo_Outpost_HP = 0, Self_Base_HP = 0;

  Self_Base_HP = Referee->Get_Self_Base_HP();
  Self_Outpost_HP = Referee->Get_Self_Outpose_HP();

  Data_MCU_To_NUC.header                         = Frame_Header;
  Data_MCU_To_NUC.mode           = 1;
  Data_MCU_To_NUC.self_color          = Get_Self_Color();

  float Yaw_rad = Now_Angle_Yaw * PI / 180.0f;
  float Pitch_rad = Now_Angle_Pitch * PI / 180.0f;
  float Roll_rad = Now_Angle_Roll * PI / 180.0f;

  Data_MCU_To_NUC.q[0] = arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f);
  Data_MCU_To_NUC.q[1] = arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) - arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f);
  Data_MCU_To_NUC.q[2] = arm_cos_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f) + arm_sin_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f);
  Data_MCU_To_NUC.q[3] = arm_cos_f32(Roll_rad / 2.0f) * arm_cos_f32(Pitch_rad / 2.0f) * arm_sin_f32(Yaw_rad / 2.0f) - arm_sin_f32(Roll_rad / 2.0f) * arm_sin_f32(Pitch_rad / 2.0f) * arm_cos_f32(Yaw_rad / 2.0f);

  // //发送自瞄的数据
  // Data_MCU_To_NUC_Aimmer.tmode = 1;
  // Data_MCU_To_NUC_Aimmer.q[0] = Data_MCU_To_NUC.q[0];
  // Data_MCU_To_NUC_Aimmer.q[1] = Data_MCU_To_NUC.q[1];
  // Data_MCU_To_NUC_Aimmer.q[2] = Data_MCU_To_NUC.q[2];
  // Data_MCU_To_NUC_Aimmer.q[3] = Data_MCU_To_NUC.q[3];
  // Data_MCU_To_NUC_Aimmer.yaw = Now_Angle_Yaw;
  // Data_MCU_To_NUC_Aimmer.yaw_vel = External_IMU->Get_Gyro_Yaw() * 57.3f;
  // Data_MCU_To_NUC_Aimmer.pitch = Now_Angle_Pitch;
  // Data_MCU_To_NUC_Aimmer.pitch_vel = External_IMU->Get_Gyro_Pitch() * 57.3f;
  // Data_MCU_To_NUC_Aimmer.bullet_speed = Referee->Get_Shoot_Speed();
  // Data_MCU_To_NUC_Aimmer.bullet_count = 10;                                    //累计发弹数量
  // Append_CRC16_Check_Sum((uint8_t*)&Data_MCU_To_NUC_Aimmer, sizeof(Struct_MiniPC_Aimmer_Tx_Data));


  Data_MCU_To_NUC.yaw = Now_Angle_Yaw;
  Data_MCU_To_NUC.yaw_vel = External_IMU->Get_Gyro_Yaw() * 57.3f;
  Data_MCU_To_NUC.pitch = Now_Angle_Pitch;
  Data_MCU_To_NUC.pitch_vel = External_IMU->Get_Gyro_Pitch() * 57.3f;
  Data_MCU_To_NUC.bullet_speed = Referee->Get_Shoot_Speed();
  Data_MCU_To_NUC.bullet_count = 10;                                    //累计发弹数量
  //Data_MCU_To_NUC.Gimbal_Now_Pitch_Angle         = int16_t(Now_Angle_Pitch * 100.0f);
  Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle_Main      = Now_Angle_Main_Yaw;
  Data_MCU_To_NUC.Gimbal_Now_Yaw_Angle           = Now_Angle_Yaw;
  Data_MCU_To_NUC.Chassis_Now_yaw_Angle          = (IMU->Get_Angle_Yaw() - Now_Angle_Relative);
  Data_MCU_To_NUC.Game_Progress                  = (uint8_t)Referee->Get_Game_Stage();
  Data_MCU_To_NUC.Self_HP                        = Referee->Get_HP();
  Data_MCU_To_NUC.Self_Outpost_HP                = Self_Outpost_HP;
  Data_MCU_To_NUC.Enemy_Outpost_HP               = Oppo_Outpost_HP;
  Data_MCU_To_NUC.Self_Base_HP                   = Self_Base_HP;
  Data_MCU_To_NUC.Projectile_Allowance           = Referee->Get_17mm_Remaining();
  Data_MCU_To_NUC.Remaining_Time                 = Referee->Get_Remaining_Time();
  
  //无敌状态辨认没有了，都是0
  Data_MCU_To_NUC.Invincivle_State               = Color << 7 | Invincible_Flag[5] << 5 | Invincible_Flag[4] << 4 | Invincible_Flag[3] << 3 | Invincible_Flag[2] << 2 | Invincible_Flag[1] << 1 | Invincible_Flag[0] << 0;
  Data_MCU_To_NUC.Target_Position_X              = Referee->Get_Map_Command_Taregt_Position_X() * 100.0f;
  Data_MCU_To_NUC.Target_Position_Y              = Referee->Get_Map_Command_Taregt_Position_Y() * 100.0f;
  Data_MCU_To_NUC.Dart_Target                    = Referee->Get_Dart_Command_Target();
  Data_MCU_To_NUC.free_respawn_ready             = Referee->Get_Sentry_info();
  Data_MCU_To_NUC.remaining_energy               = Supercap->Get_Consuming_Power();


  //顺序发送版本    可以从雷达获取敌方车辆位置，发送给上位机
  switch(index)
  {
    case 0:
    {
      Data_MCU_To_NUC.enemy_position_x = 0x00 << 14 | Referee->Get_Hero_Position_X();
      Data_MCU_To_NUC.enemy_position_y = 0x00 << 14 | Referee->Get_Hero_Position_Y();
      break;
    }
    case 1:
    {
      Data_MCU_To_NUC.enemy_position_x = 0x01 << 14 | Referee->Get_Infantry_3_Position_X();
      Data_MCU_To_NUC.enemy_position_y = 0x01 << 14 | Referee->Get_Infantry_3_Position_Y();
      break;
    }
    case 2:
    {
      Data_MCU_To_NUC.enemy_position_x = 0x02 << 14 | Referee->Get_Infantry_4_Position_X();
      Data_MCU_To_NUC.enemy_position_y = 0x02 << 14 | Referee->Get_Infantry_4_Position_Y();
      break;
    }
    case 3:
    {
      Data_MCU_To_NUC.enemy_position_x = 0x03 << 14 | Referee->Get_Sentry_Position_X();
      Data_MCU_To_NUC.enemy_position_y = 0x03 << 14 | Referee->Get_Sentry_Position_Y();
      break;
    }
  }
  //crc16 校验
  Append_CRC16_Check_Sum((uint8_t *)&Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));

  memcpy(USB_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
  USB_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Tx_Data);
  
  // // USB通信
  // memset(USB_Manage_Object->Tx_Buffer, 0, sizeof(USB_Manage_Object->Tx_Buffer));
  // if(mod2 == 0){
  //   mod2 = 1;
  //   memcpy(USB_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC, sizeof(Struct_MiniPC_Tx_Data));
  //   USB_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Tx_Data);
  // }
  // else {
  //   mod2 = 0;
  //   memcpy(USB_Manage_Object->Tx_Buffer, &Data_MCU_To_NUC_Aimmer, sizeof(Struct_MiniPC_Aimmer_Tx_Data));
  //   USB_Manage_Object->Tx_Buffer_Length = sizeof(Struct_MiniPC_Aimmer_Tx_Data);
  // }


  //重新排序
  index++;
  if(index == 4)index = 0;
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
  Output();
}

uint8_t Class_MiniPC::Get_mode()
{
  return mode;
}


/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(USB);
}

/**
 * @brief uart通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::UART_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(UART);
}

/**
 * @brief can通信接收回调函数
 *
 * @param 
 */
void Class_MiniPC::CAN_RxCpltCallback()
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(CAN);
}



/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        //迷你主机断开连接
        MiniPC_Status =  MiniPC_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPC_Status =  MiniPC_Status_ENABLE ;
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
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
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

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

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
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

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
//extern Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
float Class_MiniPC::calc_pitch(float x, float y, float z) 
{
  // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
  float pitch = atan2f(z, sqrtf(x * x + y * y));
  //使用重力加速度模型迭代更新俯仰角
  for (size_t i = 0; i < 20; i++) {
    float v_x = bullet_v * cosf(pitch);
    float v_y = bullet_v * sinf(pitch);
    // 计算子弹飞行时间
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
  pitch = pitch * 180 / PI; // 向上为负，向下为正

  return pitch;

}

/**
 * 计算当前瞄准点与目标点的偏差 (映射到同一球面的弦长)
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @param now_yaw   实际yaw轴角度（弧度制）
 * @param now_pitch 实际pitch轴角度（弧度制）
 * @return 偏差值
 */
float Class_MiniPC::Calc_Error(float x, float y, float z, float now_yaw, float now_pitch)
{
    float dis = sqrtf(x*x + y*y + z*z);
    float x0 = dis*cos(now_pitch)*cos(now_yaw);
    float y0 = dis*cos(now_pitch)*sin(now_yaw);
    float z0 = dis*sin(now_pitch);
    float err = sqrtf(pow(x-x0,2) + pow(y-y0,2) + pow(z-z0,2));
    return err;
}

/**
 * 计算计算yaw，pitch
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Class_MiniPC::Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
    *yaw = calc_yaw(x, y, z);//第一次标- 现改为正
    *pitch = calc_pitch(x, y, z);//第一次标- 现改为正
    *distance = calc_distance(x, y, z);
     //这里的z为上位机直接发的z，不是弹道解算后的z1，判断时存在一定误差（瞄准误差允许范围为5cm时，不影响10m内打弹）
}

/************************ copyright(c) ustc-robowalker **************************/
