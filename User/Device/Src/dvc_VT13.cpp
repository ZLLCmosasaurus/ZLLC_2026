/**
 * @file VT13.cpp
 * @author cjw
 * @brief HX711  tension meter
 * @version 0.1
 * @date 2025-07-01 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */
#include "dvc_VT13.h"

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
  uint8_t data;

  if (p_msg == NULL)
  {
    return 0xffff;
  }

  while (len--)
  {
    data = *p_msg++;
    (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
  }

  return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
  uint16_t w_expected = 0;

  if ((p_msg == NULL) || (len <= 2))
  {
    return false;
  }
  w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

  return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

void Class_VT13::VT13_UART_RxCpltCallback(uint8_t *Rx_Data)
{
  // 判断是否为遥控器数据(0xA9 0x53)或图传链路数据(0xA5)
  if ((*(Rx_Data + 0) != 0xA9 || *(Rx_Data + 1) != 0x53) && (*(Rx_Data + 0) != 0xA5))
  {
    return;
  }

  // 图传链路数据解包
  if (*(Rx_Data + 0) == 0xA5)
  {
    VT13_Flag++;

    uint16_t cmd_id, data_length;
    // 数据处理过程
    cmd_id = (Rx_Data[6]) & 0xff;
    cmd_id = (cmd_id << 8) | Rx_Data[5];
    data_length = Rx_Data[2] & 0xff;
    data_length = (data_length << 8) | Rx_Data[1];

    // 0x0302为自定义控制器指令
    if ((cmd_id == 0x0302) && (data_length == CUSTOM_CONTROLLER_DATA_LENGTH))
    {
      // 自定义控制器数据解包
      Custom_Controller.Custom_Controller_Data_Process(Rx_Data + FRAME_HEADER_LENGTH + CMD_ID_LENGTH);
    }
    // 0x0304键鼠数据
    else if ((cmd_id == 0x0304) && (data_length == KEYBOARD_DATA_LENGTH))
    {
      // 图传链路键鼠数据转换
      Custom_Controller.Image_Keyboard_Data_Process(Rx_Data + FRAME_HEADER_LENGTH + CMD_ID_LENGTH);
      // // 手动解包，同步给Now_Rx_Data
      // Now_RX_Data.mouse_x = Custom_Controller.Image_Keyborad_Data.mouse_x;
      // Now_RX_Data.mouse_y = Custom_Controller.Image_Keyborad_Data.mouse_y;
      // Now_RX_Data.mouse_z = Custom_Controller.Image_Keyborad_Data.mouse_z;
      // Now_RX_Data.mouse_left = (uint8_t)(Custom_Controller.Image_Keyborad_Data.left_button_down != 0 ? 1 : 0);
      // Now_RX_Data.mouse_right = (uint8_t)(Custom_Controller.Image_Keyborad_Data.right_button_down != 0 ? 1 : 0);
      // Now_RX_Data.mouse_middle = 0; // 图传键鼠包未包含中键，默认清零以防乱码
      // Now_RX_Data.key = Custom_Controller.Image_Keyborad_Data.keyboard_value;

      // // 复用 VT13 的计算逻辑刷新应用层调用的 VT13_Data
      // VT13_Data.Mouse_X = Now_RX_Data.mouse_x / 32768.0f;
      // VT13_Data.Mouse_Y = Now_RX_Data.mouse_y / 32768.0f;
      // VT13_Data.Mouse_Z = Now_RX_Data.mouse_z / 32768.0f;

      // Judge_Key(&VT13_Data.Mouse_Key_Left, Now_RX_Data.mouse_left, Pre_RX_Data.mouse_left);
      // Judge_Key(&VT13_Data.Mouse_Key_Right, Now_RX_Data.mouse_right, Pre_RX_Data.mouse_right);
      // Judge_Key(&VT13_Data.Mouse_Key_Middle, Now_RX_Data.mouse_middle, Pre_RX_Data.mouse_middle);

      // for (int i = 0; i < 16; i++)
      // {
      //   Judge_Key(&VT13_Data.Keyboard_Key[i], ((Now_RX_Data.key) >> i) & 0x1, ((Pre_RX_Data.key) >> i) & 0x1);
      // }

      // // 4. 更新 Pre_RX_Data 中键鼠相关的数据，确保下一帧能够正确捕捉按下/松开事件（防连发、检测边沿）
      // Pre_RX_Data.mouse_x = Now_RX_Data.mouse_x;
      // Pre_RX_Data.mouse_y = Now_RX_Data.mouse_y;
      // Pre_RX_Data.mouse_z = Now_RX_Data.mouse_z;
      // Pre_RX_Data.mouse_left = Now_RX_Data.mouse_left;
      // Pre_RX_Data.mouse_right = Now_RX_Data.mouse_right;
      // Pre_RX_Data.mouse_middle = Now_RX_Data.mouse_middle;
      // Pre_RX_Data.key = Now_RX_Data.key;
    }
  }
  // VT13遥控器及键鼠数据解包
  else
  {
    if (!verify_crc16_check_sum(Rx_Data, 21))
    {
      return;
    }

    VT13_Flag++;

    VT13_Data_Process(Rx_Data);

    memcpy(&Pre_RX_Data, Rx_Data, sizeof(Struct_VT13_UART_Data));
  }
}

void Class_VT13::TIM1msMod50_Alive_PeriodElapsedCallback()
{

  if (VT13_Flag == VT13_Pre_Flag)
  {
    VT13_Status = VT13_Status_DISABLE;
    Unline_Cnt++;
  }
  else
  {
    VT13_Status = VT13_Status_ENABLE;
    Unline_Cnt = 0;
  }

  VT13_Pre_Flag = VT13_Flag;
}

void Class_VT13::Judeg_Trigger(Enum_VT13_Trigger_Status *Trigger, uint8_t Status, uint8_t Pre_Status)
{
  switch (Pre_Status)
  {
  case (VT13_Trigger_FREE):
  {
    switch (Status)
    {
    case (VT13_Trigger_FREE):
    {
      *Trigger = VT13_Trigger_FREE;
    }
    break;
    case (VT13_Trigger_PRESSED):
    {
      *Trigger = VT13_Trigger_TRIG_FREE_PRESSED;
    }
    break;
    }
  }
  break;
  case (VT13_Button_PRESSED):
  {
    switch (Status)
    {
    case (VT13_Trigger_PRESSED):
    {
      *Trigger = VT13_Trigger_PRESSED;
    }
    break;
    case (VT13_Trigger_FREE):
    {
      *Trigger = VT13_Trigger_TRIG_PRESSED_FREE;
    }
    break;
    }
  }
  break;
  }
}

void Class_VT13::Judge_Button(Enum_VT13_Button_Status *Button, uint8_t Status, uint8_t Pre_Status)
{
  switch (Pre_Status)
  {
  case (VT13_Button_FREE):
  {
    switch (Status)
    {
    case (VT13_Button_FREE):
    {
      *Button = VT13_Button_FREE;
    }
    break;
    case (VT13_Button_PRESSED):
    {
      *Button = VT13_Button_TRIG_FREE_PRESSED;
    }
    break;
    }
  }
  break;
  case (VT13_Button_PRESSED):
  {
    switch (Status)
    {
    case (VT13_Button_PRESSED):
    {
      *Button = VT13_Button_PRESSED;
    }
    break;
    case (VT13_Button_FREE):
    {
      *Button = VT13_Button_TRIG_PRESSED_FREE;
    }
    break;
    }
  }
  break;
  }
}

void Class_VT13::Judge_Switch(Enum_VT13_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
  switch (Pre_Status)
  {
  case (VT13_Switch_Status_Left):
  {
    switch (Status)
    {
    case (VT13_Switch_Status_Left):
    {
      *Switch = VT13_Switch_Status_Left;
    }
    break;
    case (VT13_Switch_Status_Middle):
    {
      *Switch = VT13_Switch_Status_TRIG_Left_Middle;
    }
    break;
    case (VT13_Switch_Status_Right):
    {
      *Switch = VT13_Switch_Status_TRIG_Left_Right;
    }
    break;
    }
  }
  break;
  case (VT13_Switch_Status_Middle):
  {
    switch (Status)
    {
    case (VT13_Switch_Status_Middle):
    {
      *Switch = VT13_Switch_Status_Middle;
    }
    break;
    case (VT13_Switch_Status_Left):
    {
      *Switch = VT13_Switch_Status_TRIG_Middle_Left;
    }
    break;
    case (VT13_Switch_Status_Right):
    {
      *Switch = VT13_Switch_Status_TRIG_Middle_Right;
    }
    break;
    }
  }
  break;
  case (VT13_Switch_Status_Right):
  {
    switch (Status)
    {
    case (VT13_Switch_Status_Right):
    {
      *Switch = VT13_Switch_Status_Right;
    }
    break;
    case (VT13_Switch_Status_Middle):
    {
      *Switch = VT13_Switch_Status_TRIG_Right_Middle;
    }
    break;
    case (VT13_Switch_Status_Left):
    {
      *Switch = VT13_Switch_Status_TRIG_Right_Left;
    }
    break;
    }
  }
  break;
  }
}

void Class_VT13::Judge_Key(Enum_VT13_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
  switch (Pre_Status)
  {
  case (VT13_Key_Status_FREE):
  {
    switch (Status)
    {
    case (VT13_Key_Status_FREE):
    {
      *Key = VT13_Key_Status_FREE;
    }
    break;
    case (VT13_Key_Status_PRESSED):
    {
      *Key = VT13_Key_Status_TRIG_FREE_PRESSED;
    }
    break;
    }
  }
  break;
  case (VT13_Key_Status_PRESSED):
  {
    switch (Status)
    {
    case (VT13_Key_Status_PRESSED):
    {
      *Key = VT13_Key_Status_PRESSED;
    }
    break;
    case (VT13_Key_Status_FREE):
    {
      *Key = VT13_Key_Status_TRIG_PRESSED_FREE;
    }
    break;
    }
  }
  break;
  }
}

void Class_VT13::VT13_Data_Process(uint8_t *Rx_Data)
{
  memcpy(&Now_RX_Data, Rx_Data, sizeof(Struct_VT13_UART_Data));

  VT13_Data.Right_X = (float)(Now_RX_Data.ch_0 - Rocker_Offset) / Rocker_Num;
  VT13_Data.Right_Y = (float)(Now_RX_Data.ch_1 - Rocker_Offset) / Rocker_Num;
  VT13_Data.Left_Y = (float)(Now_RX_Data.ch_2 - Rocker_Offset) / Rocker_Num;
  VT13_Data.Left_X = (float)(Now_RX_Data.ch_3 - Rocker_Offset) / Rocker_Num;

  VT13_Data.Yaw = (float)(Now_RX_Data.wheel - Rocker_Offset) / Rocker_Num;

  Judeg_Trigger(&VT13_Data.Trigger, Now_RX_Data.trigger, Pre_RX_Data.trigger);
  Judge_Button(&VT13_Data.Pause_Key, Now_RX_Data.pause, Pre_RX_Data.pause);
  Judge_Button(&VT13_Data.Button_Left, Now_RX_Data.fn_1, Pre_RX_Data.fn_1);
  Judge_Button(&VT13_Data.Button_Right, Now_RX_Data.fn_2, Pre_RX_Data.fn_2);

  Judge_Switch(&VT13_Data.Switch, Now_RX_Data.mode_sw, Pre_RX_Data.mode_sw);

  VT13_Data.Mouse_X = Now_RX_Data.mouse_x / 32768.0f;
  VT13_Data.Mouse_Y = Now_RX_Data.mouse_y / 32768.0f;
  VT13_Data.Mouse_Z = Now_RX_Data.mouse_z / 32768.0f;

  Judge_Key(&VT13_Data.Mouse_Key_Left, Now_RX_Data.mouse_left, Pre_RX_Data.mouse_left);
  Judge_Key(&VT13_Data.Mouse_Key_Right, Now_RX_Data.mouse_right, Pre_RX_Data.mouse_right);
  Judge_Key(&VT13_Data.Mouse_Key_Middle, Now_RX_Data.mouse_middle, Pre_RX_Data.mouse_middle);

  // 判断键盘触发
  for (int i = 0; i < 16; i++)
  {
    Judge_Key(&VT13_Data.Keyboard_Key[i], ((Now_RX_Data.key) >> i) & 0x1, ((Pre_RX_Data.key) >> i) & 0x1);
  }
}