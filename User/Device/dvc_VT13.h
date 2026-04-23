#ifndef DVC_VT13_H

#define DVC_VT13_H

#include "stdint.h"
#include "string.h"

#define KEY_W 0
#define KEY_S 1
#define KEY_A 2
#define KEY_D 3
#define KEY_SHIFT 4
#define KEY_CTRL 5
#define KEY_Q 6
#define KEY_E 7
#define KEY_R 8
#define KEY_F 9
#define KEY_G 10
#define KEY_Z 11
#define KEY_X 12
#define KEY_C 13
#define KEY_V 14
#define KEY_B 15

enum Enum_VT13_Status{
  VT13_Status_DISABLE = 0,
  VT13_Status_ENABLE,
};

enum Enum_VT13_Trigger_Status{
  VT13_Trigger_FREE = 0,
  VT13_Trigger_PRESSED,
  VT13_Trigger_TRIG_FREE_PRESSED,
  VT13_Trigger_TRIG_PRESSED_FREE,
  
};

enum Enum_VT13_Button_Status{
  VT13_Button_FREE = 0,
  VT13_Button_PRESSED, 
  VT13_Button_TRIG_FREE_PRESSED,
  VT13_Button_TRIG_PRESSED_FREE,
};

enum Enum_VT13_Switch_Status{
  VT13_Switch_Status_Left = 0,
  VT13_Switch_Status_Middle,
  VT13_Switch_Status_Right,
  
  VT13_Switch_Status_TRIG_Left_Middle,      //左到中突变
  VT13_Switch_Status_TRIG_Middle_Left,      

  VT13_Switch_Status_TRIG_Middle_Right,     //中到右突变
  VT13_Switch_Status_TRIG_Right_Middle,

  VT13_Switch_Status_TRIG_Left_Right,
  VT13_Switch_Status_TRIG_Right_Left,
};

enum Enum_VT13_Key_Status
{
  VT13_Key_Status_FREE = 0,           //松开状态
  VT13_Key_Status_PRESSED,            //按下状态
  VT13_Key_Status_TRIG_FREE_PRESSED,  //松开到按下的突变状态
  VT13_Key_Status_TRIG_PRESSED_FREE,  //按下到松开的突变状态
};

struct Struct_VT13_UART_Data
{
    uint8_t sof_1;                    //帧头1
    uint8_t sof_2;                    //帧头2
    uint64_t ch_0:11;                 //右X
    uint64_t ch_1:11;                 //右Y
    uint64_t ch_2:11;                 //左Y
    uint64_t ch_3:11;                 //左X
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}__attribute__((packed)); 

struct Struct_VT13_Data{

  float Left_X;
  float Left_Y;
  float Right_X;
  float Right_Y;
  float Yaw;

  Enum_VT13_Button_Status Button_Left;
  Enum_VT13_Button_Status Button_Right;
  Enum_VT13_Button_Status Pause_Key;
  Enum_VT13_Trigger_Status Trigger;
  Enum_VT13_Switch_Status Switch;

  float Mouse_X;
  float Mouse_Y;
  float Mouse_Z;
  Enum_VT13_Key_Status Mouse_Key_Left;
  Enum_VT13_Key_Status Mouse_Key_Right;
  Enum_VT13_Key_Status Mouse_Key_Middle;
  Enum_VT13_Key_Status Keyboard_Key[16];
};

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

class Class_VT13
{
  public:

  inline Enum_VT13_Status Get_VT13_Status();

  inline float Get_Right_X();
  inline float Get_Right_Y();
  inline float Get_Left_X();
  inline float Get_Left_Y();
  inline float Get_Yaw();
  inline Enum_VT13_Switch_Status Get_Switch();
  inline Enum_VT13_Button_Status Get_Button_Left();
  inline Enum_VT13_Button_Status Get_Button_Right();
  inline Enum_VT13_Trigger_Status Get_Trigger();

  inline float Get_Mouse_X();
  inline float Get_Mouse_Y();
  inline float Get_Mouse_Z();
  inline Enum_VT13_Key_Status Get_Mouse_Left_Key();
  inline Enum_VT13_Key_Status Get_Mouse_Right_Key();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_W();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_S();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_A();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_D();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_Shift();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_Ctrl();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_Q();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_E();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_R();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_F();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_G();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_Z();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_X();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_C();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_V();
  inline Enum_VT13_Key_Status Get_Keyboard_Key_B();

  void VT13_UART_RxCpltCallback(uint8_t *Rx_Data);
  void TIM1msMod50_Alive_PeriodElapsedCallback();

  private:

  const int16_t Min_Channel_Num = 364;
  const int16_t Max_Channel_Num = 1684;
  const float Rocker_Offset = 1024.0f;
  const float Rocker_Num = 660.0f;

  uint16_t Unline_Cnt = 0;

  int32_t VT13_Flag;
  int32_t VT13_Pre_Flag;

  Enum_VT13_Status VT13_Status;

  Struct_VT13_UART_Data Now_RX_Data;
  Struct_VT13_UART_Data Pre_RX_Data;

  Struct_VT13_Data VT13_Data;

  void Judeg_Trigger(Enum_VT13_Trigger_Status *Trigger, uint8_t Status, uint8_t Pre_Status);
  void Judge_Button(Enum_VT13_Button_Status *Button, uint8_t Status, uint8_t Pre_Status);
  void Judge_Switch(Enum_VT13_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
  void Judge_Key(Enum_VT13_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
  void VT13_Data_Process(uint8_t *Rx_Data);

};

inline float Class_VT13::Get_Right_X(){
  return (VT13_Data.Right_X);
}

inline float Class_VT13::Get_Right_Y(){
  return (VT13_Data.Right_Y);
}

inline float Class_VT13::Get_Left_X(){
  return (VT13_Data.Left_X);
}

inline float Class_VT13::Get_Left_Y(){
  return (VT13_Data.Left_Y);
}

inline Enum_VT13_Switch_Status Class_VT13::Get_Switch()
{
  return (VT13_Data.Switch);
}

inline Enum_VT13_Button_Status Class_VT13::Get_Button_Left()
{
  return (VT13_Data.Button_Left);
}

inline Enum_VT13_Button_Status Class_VT13::Get_Button_Right()
{
  return (VT13_Data.Button_Right);
}

inline Enum_VT13_Trigger_Status Class_VT13::Get_Trigger()
{
  return (VT13_Data.Trigger);
}

float Class_VT13::Get_Yaw(){
  return (VT13_Data.Yaw);
}

inline float Class_VT13::Get_Mouse_X()
{
  return (VT13_Data.Mouse_X);
}

inline float Class_VT13::Get_Mouse_Y()
{
  return (VT13_Data.Mouse_Y);
}

inline float Class_VT13::Get_Mouse_Z()
{
  return (VT13_Data.Mouse_Z);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Mouse_Left_Key()
{
  return (VT13_Data.Mouse_Key_Left);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Mouse_Right_Key()
{
  return (VT13_Data.Mouse_Key_Right);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_W()
{
  return (VT13_Data.Keyboard_Key[KEY_W]);
}
inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_S()
{
  return (VT13_Data.Keyboard_Key[KEY_S]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_A()
{
  return (VT13_Data.Keyboard_Key[KEY_A]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_D()
{
  return (VT13_Data.Keyboard_Key[KEY_D]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_Shift()
{
  return (VT13_Data.Keyboard_Key[KEY_SHIFT]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_Ctrl()
{
  return (VT13_Data.Keyboard_Key[KEY_CTRL]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_Q()
{
  return (VT13_Data.Keyboard_Key[KEY_Q]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_E()
{
  return (VT13_Data.Keyboard_Key[KEY_E]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_R()
{
  return (VT13_Data.Keyboard_Key[KEY_R]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_F()
{
  return (VT13_Data.Keyboard_Key[KEY_F]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_G()
{
  return (VT13_Data.Keyboard_Key[KEY_G]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_Z()
{
  return (VT13_Data.Keyboard_Key[KEY_Z]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_X()
{
  return (VT13_Data.Keyboard_Key[KEY_X]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_C()
{
  return (VT13_Data.Keyboard_Key[KEY_C]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_V()
{
  return (VT13_Data.Keyboard_Key[KEY_V]);
}

inline Enum_VT13_Key_Status Class_VT13::Get_Keyboard_Key_B()
{
  return (VT13_Data.Keyboard_Key[KEY_B]);
}

inline Enum_VT13_Status Class_VT13::Get_VT13_Status()
{
  return (VT13_Status);
}

#endif