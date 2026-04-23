/**
 * @file DR16.h
 * @author lez by yssickjgd
 * @brief DR16遥控器
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

 #ifndef DVC_DR16_H
 #define DVC_DR16_H
 
 /* Includes ------------------------------------------------------------------*/
 
 #include <limits.h>
 #include <string.h>
 #include "drv_uart.h"
 
 /* Exported macros -----------------------------------------------------------*/
 
 // 拨动开关位置
 #define SWITCH_UP (1)
 #define SWITCH_DOWN (2)
 #define SWITCH_MIDDLE (3)
 
 // 按键开关位置
 #define KEY_FREE (0)
 #define KEY_PRESSED (1)
 
 // 键位宏定义
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
 
 /* Exported types ------------------------------------------------------------*/
 
 /**
  * @brief 遥控器状态
  *
  */
 enum Enum_DR16_Status
 {
     DR16_Status_DISABLE = 0,
     DR16_Status_ENABLE,
 };
 
 /**
  * @brief 遥控器数据更新状态
  *
  */
 enum Enum_DR16_Updata_Status
 {
     DR16_Status_DisUpdata = 0,
     DR16_Status_Updata,
 };
 /**
  * @brief dt7存活状态
  *
  */
 enum Enum_DT7_Status
 {
     DT7_Status_DISABLE,
     DT7_Status_ENABLE,
 };
 
 /**
  * @brief image存活状态
  *
  */
 enum Enum_Image_Status
 {
     Image_Status_DISABLE,
     Image_Status_ENABLE,
 };
 /**
  * @brief 拨动开关状态
  *
  */
 enum Enum_DR16_Switch_Status
 {
     DR16_Switch_Status_UP = 0,           // 上状态
     DR16_Switch_Status_TRIG_UP_MIDDLE,   // 上到中的突变状态
     DR16_Switch_Status_TRIG_MIDDLE_UP,   // 中到上的突变状态
     DR16_Switch_Status_MIDDLE,           // 中状态
     DR16_Switch_Status_TRIG_MIDDLE_DOWN, // 中到下的突变状态
     DR16_Switch_Status_TRIG_DOWN_MIDDLE, // 下到中的突变状态
     DR16_Switch_Status_DOWN,             // 下状态
 };
 
 /**
  * @brief 按键状态
  *
  */
 enum Enum_DR16_Key_Status
 {
     DR16_Key_Status_FREE = 0,          // 松开状态
     DR16_Key_Status_TRIG_FREE_PRESSED, // 松开到按下的突变状态
     DR16_Key_Status_TRIG_PRESSED_FREE, // 按下到松开的突变状态
     DR16_Key_Status_PRESSED,           // 按下状态
 };
 
 /**
  * @brief 图传键鼠源数据
  *
  */
 struct Struct_Image_UART_Data
 {
     int16_t Mouse_X;
     int16_t Mouse_Y;
     int16_t Mouse_Z;
     int8_t Mouse_Left_Key;
     int8_t Mouse_Right_Key;
     uint16_t Keyboard_Key;
     uint16_t reserved;
 } __attribute__((packed));
 
 /**
  * @brief DR16源数据
  *
  */
 struct Struct_DR16_UART_Data
 {
     uint64_t Channel_0 : 11;
     uint64_t Channel_1 : 11;
     uint64_t Channel_2 : 11;
     uint64_t Channel_3 : 11;
     uint64_t Switch_2 : 2;
     uint64_t Switch_1 : 2;
     int16_t Mouse_X;
     int16_t Mouse_Y;
     int16_t Mouse_Z;
     uint64_t Mouse_Left_Key : 8;
     uint64_t Mouse_Right_Key : 8;
     uint64_t Keyboard_Key : 16;
     uint64_t Channel_Yaw : 11;
 } __attribute__((packed));
 
 /**
  * @brief DR16经过处理的的数据, 摇杆信息经过归一化到-1~1, 鼠标信息有待进一步标定
  *
  */
 struct Struct_DR16_Data
 {
     float Right_X;
     float Right_Y;
     float Left_X;
     float Left_Y;
     Enum_DR16_Switch_Status Left_Switch;
     Enum_DR16_Switch_Status Right_Switch;
     float Mouse_X;
     float Mouse_Y;
     float Mouse_Z;
     Enum_DR16_Key_Status Mouse_Left_Key;
     Enum_DR16_Key_Status Mouse_Right_Key;
     Enum_DR16_Key_Status Keyboard_Key[16];
     float Yaw;
 };
 
 /**
  * @brief Specialized, 遥控器
  *
  */
 class Class_DR16
 {
 public:
     void Init(UART_HandleTypeDef *huart_1, UART_HandleTypeDef *huart_2);
 
     inline Enum_DR16_Status Get_DR16_Status();
     inline Enum_DR16_Updata_Status Get_DR16_Updata_Status();
     inline Enum_Image_Status Get_Image_Status() { return (Image_Status); };
     inline Enum_DT7_Status Get_DT7_Status() { return (DT7_Status); };
     inline float Get_Right_X();
     inline float Get_Right_Y();
     inline float Get_Left_X();
     inline float Get_Left_Y();
     inline Enum_DR16_Switch_Status Get_Left_Switch();
     inline Enum_DR16_Switch_Status Get_Right_Switch();
     inline float Get_Mouse_X();
     inline float Get_Mouse_Y();
     inline float Get_Mouse_Z();
     inline Enum_DR16_Key_Status Get_Mouse_Left_Key();
     inline Enum_DR16_Key_Status Get_Mouse_Right_Key();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_W();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_S();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_A();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_D();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_Shift();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_Ctrl();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_Q();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_E();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_R();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_F();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_G();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_Z();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_X();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_C();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_V();
     inline Enum_DR16_Key_Status Get_Keyboard_Key_B();
 
     inline float Get_Yaw();
 
     void DR16_UART_RxCpltCallback(uint8_t *Rx_Data);
     void Image_UART_RxCpltCallback(uint8_t *Rx_Data);
 
     void TIM1msMod50_Alive_PeriodElapsedCallback();
 
 protected:
     // 初始化相关常量
 
     // 绑定的UART
     Struct_UART_Manage_Object *UART_Manage_Object_1;
     Struct_UART_Manage_Object *UART_Manage_Object_2;
 
     // 常量
 
     // 摇杆偏移量
     float Rocker_Offset = 1024.0f;
     // 摇杆总刻度
     float Rocker_Num = 660.0f;
 
     // 内部变量
     // 前一时刻的遥控器状态信息
     Struct_DR16_UART_Data Now_UART_Rx_Data;
     // 前一时刻的遥控器状态信息
     Struct_DR16_UART_Data Pre_UART_Rx_Data;
 
     Struct_Image_UART_Data Now_UART_Image_Rx_Data;
     Struct_Image_UART_Data Pre_UART_Image_Rx_Data;
 
     // 当前时刻的遥控器接收flag
     uint32_t DR16_Flag = 0;
     // 前一时刻的遥控器接收flag
     uint32_t Pre_DR16_Flag = 0;
 
     // 当前时刻的遥控器接收flag
     uint32_t Image_Flag = 0;
     // 前一时刻的遥控器接收flag
     uint32_t Pre_Image_Flag = 0;
 
     // 遥控器50ms离线次数
     uint16_t Unline_Cnt = 0;
     // 遥控器50ms串口错误次数
     uint16_t Error_Cnt = 0;
     // 读变量
 
     // 遥控器状态
     Enum_DR16_Status DR16_Status = DR16_Status_DISABLE;
     // 遥控器数据更新状态
     Enum_DR16_Updata_Status DR16_Updata_Status = DR16_Status_DisUpdata;
     // 图传存活状态
     Enum_Image_Status Image_Status = Image_Status_DISABLE;
     // dt7存活状态
     Enum_DT7_Status DT7_Status = DT7_Status_DISABLE;
     // DR16对外接口信息
     Struct_DR16_Data Data;
 
     // 写变量
 
     // 读写变量
 
     // 内部函数
 
     void Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
     void Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);
     void Judge_Updata(Struct_DR16_UART_Data Pre_UART_Rx_Data, Struct_DR16_UART_Data Now_UART_Rx_Data);
     void DR16_Data_Process();
     void Image_Data_Process(uint8_t *__rx_buffer);
 };
 
 /* Exported variables --------------------------------------------------------*/
 
 /* Exported function declarations --------------------------------------------*/
 
 /**
  * @brief 获取遥控器在线状态
  *
  * @return Enum_DR16_Status 遥控器在线状态
  */
 Enum_DR16_Status Class_DR16::Get_DR16_Status()
 {
     return (DR16_Status);
 }
 
 /**
  * @brief 获取遥控器数据更新状态
  *
  * @return Enum_DR16_Updata_Status 遥控器在线状态
  */
 Enum_DR16_Updata_Status Class_DR16::Get_DR16_Updata_Status()
 {
     return (DR16_Updata_Status);
 }
 
 /**
  * @brief 获取遥控器右侧x轴摇杆状态
  *
  * @return float 遥控器右侧x轴摇杆状态
  */
 float Class_DR16::Get_Right_X()
 {
     return (Data.Right_X);
 }
 
 /**
  * @brief 获取遥控器右侧y轴摇杆状态
  *
  * @return float 遥控器右侧y轴摇杆状态
  */
 float Class_DR16::Get_Right_Y()
 {
     return (Data.Right_Y);
 }
 
 /**
  * @brief 获取遥控器左侧x轴摇杆状态
  *
  * @return float 遥控器左侧x轴摇杆状态
  */
 float Class_DR16::Get_Left_X()
 {
     return (Data.Left_X);
 }
 
 /**
  * @brief 获取遥控器左侧y轴摇杆状态
  *
  * @return float 遥控器左侧y轴摇杆状态
  */
 float Class_DR16::Get_Left_Y()
 {
     return (Data.Left_Y);
 }
 
 /**
  * @brief 获取遥控器左侧拨动开关状态
  *
  * @return Enum_DR16_Switch_Status 遥控器左侧拨动开关状态
  */
 Enum_DR16_Switch_Status Class_DR16::Get_Left_Switch()
 {
     return (Data.Left_Switch);
 }
 
 /**
  * @brief 获取遥控器右侧拨动开关状态
  *
  * @return Enum_DR16_Switch_Status 遥控器右侧拨动开关状态
  */
 Enum_DR16_Switch_Status Class_DR16::Get_Right_Switch()
 {
     return (Data.Right_Switch);
 }
 
 /**
  * @brief 获取鼠标x轴状态
  *
  * @return float 鼠标x轴状态
  */
 float Class_DR16::Get_Mouse_X()
 {
     return (Data.Mouse_X);
 }
 
 /**
  * @brief 获取鼠标y轴状态
  *
  * @return float 鼠标y轴状态
  */
 float Class_DR16::Get_Mouse_Y()
 {
     return (Data.Mouse_Y);
 }
 
 /**
  * @brief 获取鼠标z轴状态
  *
  * @return float 鼠标z轴状态
  */
 float Class_DR16::Get_Mouse_Z()
 {
     return (Data.Mouse_Z);
 }
 
 /**
  * @brief 获取鼠标左键状态
  *
  * @return Enum_DR16_Key_Status 鼠标左键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Mouse_Left_Key()
 {
     return (Data.Mouse_Left_Key);
 }
 
 /**
  * @brief 获取鼠标右键状态
  *
  * @return Enum_DR16_Key_Status 鼠标右键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Mouse_Right_Key()
 {
     return (Data.Mouse_Right_Key);
 }
 
 /**
  * @brief 获取键盘W键状态
  *
  * @return Enum_DR16_Key_Status 键盘W键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_W()
 {
     return (Data.Keyboard_Key[KEY_W]);
 }
 
 /**
  * @brief 获取键盘S键状态
  *
  * @return Enum_DR16_Key_Status 键盘S键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_S()
 {
     return (Data.Keyboard_Key[KEY_S]);
 }
 
 /**
  * @brief 获取键盘A键状态
  *
  * @return Enum_DR16_Key_Status 键盘A键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_A()
 {
     return (Data.Keyboard_Key[KEY_A]);
 }
 
 /**
  * @brief 获取键盘D键状态
  *
  * @return Enum_DR16_Key_Status 键盘D键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_D()
 {
     return (Data.Keyboard_Key[KEY_D]);
 }
 
 /**
  * @brief 获取键盘Shift键状态
  *
  * @return Enum_DR16_Key_Status 键盘Shift键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Shift()
 {
     return (Data.Keyboard_Key[KEY_SHIFT]);
 }
 
 /**
  * @brief 获取键盘Ctrl键状态
  *
  * @return Enum_DR16_Key_Status 键盘Ctrl键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Ctrl()
 {
     return (Data.Keyboard_Key[KEY_CTRL]);
 }
 
 /**
  * @brief 获取键盘Q键状态
  *
  * @return Enum_DR16_Key_Status 键盘Q键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Q()
 {
     return (Data.Keyboard_Key[KEY_Q]);
 }
 
 /**
  * @brief 获取键盘E键状态
  *
  * @return Enum_DR16_Key_Status 键盘E键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_E()
 {
     return (Data.Keyboard_Key[KEY_E]);
 }
 
 /**
  * @brief 获取键盘R键状态
  *
  * @return Enum_DR16_Key_Status 键盘R键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_R()
 {
     return (Data.Keyboard_Key[KEY_R]);
 }
 
 /**
  * @brief 获取键盘F键状态
  *
  * @return Enum_DR16_Key_Status 键盘F键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_F()
 {
     return (Data.Keyboard_Key[KEY_F]);
 }
 
 /**
  * @brief 获取键盘G键状态
  *
  * @return Enum_DR16_Key_Status 键盘G键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_G()
 {
     return (Data.Keyboard_Key[KEY_G]);
 }
 
 /**
  * @brief 获取键盘Z键状态
  *
  * @return Enum_DR16_Key_Status 键盘Z键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_Z()
 {
     return (Data.Keyboard_Key[KEY_Z]);
 }
 
 /**
  * @brief 获取键盘X键状态
  *
  * @return Enum_DR16_Key_Status 键盘X键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_X()
 {
     return (Data.Keyboard_Key[KEY_X]);
 }
 
 /**
  * @brief 获取键盘C键状态
  *
  * @return Enum_DR16_Key_Status 键盘C键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_C()
 {
     return (Data.Keyboard_Key[KEY_C]);
 }
 
 /**
  * @brief 获取键盘V键状态
  *
  * @return Enum_DR16_Key_Status 键盘V键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_V()
 {
     return (Data.Keyboard_Key[KEY_V]);
 }
 
 /**
  * @brief 获取键盘B键状态
  *
  * @return Enum_DR16_Key_Status 键盘B键状态
  */
 Enum_DR16_Key_Status Class_DR16::Get_Keyboard_Key_B()
 {
     return (Data.Keyboard_Key[KEY_B]);
 }
 
 /**
  * @brief 获取遥控器yaw轴状态
  *
  * @return float 遥控器yaw轴状态
  */
 float Class_DR16::Get_Yaw()
 {
     return (Data.Yaw);
 }
 
 #endif
 
 /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/